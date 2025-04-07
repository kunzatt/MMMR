
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, PoseStamped
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, sqrt, atan2
import numpy as np
import time

class FollowTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        
        #도착 후 좌표 값 전송
        self.coordinate_pub = self.create_publisher(Point,'/navigation_coordinates',10)
        
        #도착 후 목표 방향 전송
        self.rotate_pub = self.create_publisher(PoseStamped,'/rotation',10)
        
        # 새로운 목표 지점 구독자 추가
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)

        # 라이더 데이터 값 받아오기
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # 제어 주기 및 타이머 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

         # 상태 플래그 초기화
        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_goal = False
        self.is_rotated = False
        self.is_lidar = False
        self.tracking_enabled = True
        self.goal_reached = False  # 목표 도달 상태 추가
        self.goal_message_sent = False  # 목표 도달 메시지 전송 여부 추적
        self.new_goal_received = False  # 새 목표 수신 여부
        self.goal_transition_time = None  # 목표 전환 시간 추적
        self.goal_transition_cooldown = 0.5  # 목표 전환 후 안정화 시간 (초)

        # 충돌 상태 관리 변수
        self.collision_state = False
        self.recovery_mode = False
        self.recovery_start_time = None
        self.recovery_direction = 1
        self.stuck_counter = 0
        self.prev_pose_x = 0.0
        self.prev_pose_y = 0.0
        self.movement_threshold = 0.01
        self.stuck_threshold = 20
        
        # 메시지 초기화
        self.odom_msg = Odometry()
        self.robot_yaw = 0.0
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.goal_pose = None
        self.point_msg = Point()
        self.rotate_msg = PoseStamped()
        self.lidar_data = None
        self.status_msg = None

        #좌표 변환을 위한 맵 정보
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
        self.GRIDSIZE=350 

        # 파라미터 설정
        self.lfd = 0.1  # 전방 주시 거리 (look forward distance)
        self.min_lfd = 0.1
        self.max_lfd = 1.0
        
        # 목표 도달 허용 오차 (미터)
        self.goal_tolerance = 0.1

        # 로봇 위치 변수 초기화 (오류 방지)
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0

        # 장애물 감지 파라미터
        self.obstacle_distance_threshold = 0.3
        self.front_angle_range = 20
        self.front_angles = list(range(-self.front_angle_range, self.front_angle_range+1))

        # 장애물 회피 파라미터
        self.safe_distance = 0.4
        self.avoidance_speed = 0.1
        self.avoidance_turn_speed = 0.5
        
        # 복구 행동 파라미터
        self.recovery_time = 2.0
        self.recovery_linear_speed = -0.1
        self.recovery_angular_speed = 0.3

        # 로봇 위치 변수 초기화
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        
        # 방향 전환 중 장애물 감지 억제
        self.turning_mode = False
        self.turning_start_time = 0
        self.turning_threshold = 0.5
        self.turning_cooldown = 1.0

    def goal_callback(self, msg):
        """목표 지점 콜백"""
        # 로봇을 잠시 정지시키고 새 목표로 즉시 전환
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_msg)
        
        # 새 목표 정보 업데이트
        self.goal_pose = msg.pose
        self.rotate_msg = msg
        self.is_goal = True
        self.tracking_enabled = True
        self.goal_reached = False  # 새 목표 설정 시 목표 도달 상태 초기화
        self.goal_message_sent = False  # 메시지 전송 상태 초기화
        
        # 새 경로 전환 관련 변수 설정
        self.new_goal_received = True
        self.goal_transition_time = time.time()
        
        # 복구 및 회피 모드 초기화
        self.recovery_mode = False
        self.turning_mode = False
        
        self.get_logger().info('New goal received - Switching to new path')


    def lidar_callback(self, msg):
        """라이다 데이터 콜백"""
        self.is_lidar = True
        self.lidar_data = msg

    def has_obstacle_ahead(self):
        """전방에 장애물이 있는지 확인"""
        # 목표 도달 상태면 장애물 감지 비활성화
        if self.goal_reached:
            return False
            
        # 회전 중일 때는 장애물 감지 비활성화
        if self.turning_mode and time.time() - self.turning_start_time < self.turning_cooldown:
            return False
            
        if not self.is_lidar or self.lidar_data is None:
            return False
            
        ranges = np.array(self.lidar_data.ranges)
        max_range = self.lidar_data.range_max
        ranges = np.nan_to_num(ranges, nan=max_range, posinf=max_range)
        
        center_idx = len(ranges) // 2
        angle_per_scan = 360 / len(ranges)
        front_indices = [int(center_idx + i / angle_per_scan) % len(ranges) for i in self.front_angles]
        
        # 전방 영역 거리 확인 + 강화된 필터링
        front_ranges = [ranges[i] for i in front_indices]
        min_front_distance = min(front_ranges)
        
        # 거리뿐만 아니라 최소 거리 값이 일정 개수 이상 있는지 확인
        obstacle_count = sum(1 for r in front_ranges if r < self.obstacle_distance_threshold)
        obstacle_percentage = obstacle_count / len(front_ranges)
        
        # 최소 거리가 임계값보다 작고, 일정 비율(20%) 이상이 장애물로 감지될 때만 반환
        return min_front_distance < self.obstacle_distance_threshold and obstacle_percentage > 0.2
    
    def find_escape_direction(self):
        """충돌 상황에서 빠져나올 방향 결정"""
        if not self.is_lidar or self.lidar_data is None:
            return 1
        
        ranges = np.array(self.lidar_data.ranges)
        max_range = self.lidar_data.range_max
        ranges = np.nan_to_num(ranges, nan=max_range, posinf=max_range)
        
        # 더 넓은 영역 검사
        left_idx = len(ranges) // 4
        right_idx = 3 * len(ranges) // 4
        
        # 더 넓은 영역의 평균 거리 계산
        left_distance = np.mean(ranges[left_idx-20:left_idx+20])
        right_distance = np.mean(ranges[right_idx-20:right_idx+20])
        
        # 왼쪽/오른쪽 중 더 넓은 공간이 있는 방향으로 이동
        return 1 if left_distance > right_distance else -1
    
    def detect_collision(self, current_linear_speed):
        """충돌 또는 갇힘 상태 감지"""
        # 목표 도달 상태면 충돌 감지 비활성화
        if self.goal_reached:
            return False
            
        # 속도가 명령되었지만 실제로 움직이지 않는 경우 갇힘 상태로 판단
        if abs(current_linear_speed) > 0.05:
            distance_moved = sqrt(
                pow(self.robot_pose_x - self.prev_pose_x, 2) +
                pow(self.robot_pose_y - self.prev_pose_y, 2)
            )
            
            if distance_moved < self.movement_threshold:
                self.stuck_counter += 1
                if self.stuck_counter > self.stuck_threshold:
                    return True
            else:
                self.stuck_counter = 0
                
        # 위치 업데이트
        self.prev_pose_x = self.robot_pose_x
        self.prev_pose_y = self.robot_pose_y
        
        return False

    #odometry 값을 grid 좌표 값으로 변환 함수수
    def pose_to_grid_cell(self,x,y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return map_point_x,map_point_y

    def check_goal_reached(self):
        """목표 지점 도달 여부 확인"""
        if not self.is_goal or self.goal_pose is None:
            return False
        
        # 현재 로봇 위치와 목표 지점 간 거리 계산
        distance = sqrt(
            pow(self.robot_pose_x - self.goal_pose.position.x, 2) +
            pow(self.robot_pose_y - self.goal_pose.position.y, 2)
        )
        
        # 마지막 경로 포인트와 목표 지점 간 거리 확인
        if len(self.path_msg.poses) > 0:
            last_path_point = self.path_msg.poses[-1].pose.position
            last_point_distance = sqrt(
                pow(last_path_point.x - self.goal_pose.position.x, 2) +
                pow(last_path_point.y - self.goal_pose.position.y, 2)
            )
            
            # 로봇이 목표 지점 근처이고, 마지막 경로 포인트도 목표 지점과 가까우면 정지
            if distance <= self.goal_tolerance and last_point_distance <= self.goal_tolerance:
                if not self.goal_reached:  # 첫 도달 시에만 로그 출력
                    self.get_logger().info('Goal reached!')
                    self.goal_reached = True
                return True
        
        return False

    def timer_callback(self):
        # 필요한 데이터 확인
        if not (self.is_status and self.is_odom):
            return

        # 새 목표가 입력된 후 안정화 시간 동안 대기
        if self.new_goal_received:
            current_time = time.time()
            if current_time - self.goal_transition_time <= self.goal_transition_cooldown:
                # 안정화 시간 동안 정지 상태 유지
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)
                return
            else:
                # 안정화 시간이 지나면 목표 전환 완료
                self.new_goal_received = False
                self.get_logger().info('Path transition complete - Starting navigation to new goal')

        # 회전 중인지 확인하고 turning_mode 업데이트
        if hasattr(self, 'cmd_msg') and abs(self.cmd_msg.angular.z) > self.turning_threshold:
            if not self.turning_mode:
                self.turning_mode = True
                self.turning_start_time = time.time()
        else:
            # 회전이 끝난 경우, cooldown 시간이 지나면 turning_mode를 False로 변경
            if self.turning_mode and time.time() - self.turning_start_time > self.turning_cooldown:
                self.turning_mode = False

        # 목표 지점 도달 확인 - 가장 먼저 체크
        if self.check_goal_reached():
            # 로봇 정지
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            
            # 목표 도달 메시지는 한 번만 전송
            if not self.goal_message_sent:
                self.rotate_pub.publish(self.rotate_msg)
                self.get_logger().info(f"Published rotate_pub")
                
                now_cell = self.pose_to_grid_cell(self.robot_pose_x, self.robot_pose_y)
                self.point_msg.x = float(now_cell[0])
                self.point_msg.y = float(350-now_cell[1])
                self.coordinate_pub.publish(self.point_msg)
                self.get_logger().info(f"Published goal coordinate")
                
                self.goal_message_sent = True  # 메시지 전송 완료 표시
            return
        
        # 현재 명령된 선속도 확인 (충돌 감지용)
        current_linear_speed = self.cmd_msg.linear.x if hasattr(self, 'cmd_msg') else 0.0
        
        # 복구 모드 처리 - 목표 도달 확인 후에 실행
        if self.recovery_mode and not self.goal_reached:  # 목표 도달 상태면 복구 모드 비활성화
            current_time = time.time()
            
            if current_time - self.recovery_start_time > self.recovery_time:
                # 복구 모드 종료
                self.recovery_mode = False
                self.tracking_enabled = True
                self.get_logger().info('Recovery mode completed')
            else:
                # 복구 모드 동작 수행 (후진 및 회전)
                self.cmd_msg.linear.x = self.recovery_linear_speed
                self.cmd_msg.angular.z = self.recovery_direction * self.recovery_angular_speed
                self.cmd_pub.publish(self.cmd_msg)
                return
        elif self.recovery_mode and self.goal_reached:
            # 목표 도달 상태면 복구 모드 즉시 종료
            self.recovery_mode = False
            self.tracking_enabled = True
        
        # 충돌/갇힘 감지 - 목표에 도달하지 않은 경우에만 실행
        if not self.goal_reached and self.detect_collision(current_linear_speed):
            self.get_logger().warning('Collision or stuck state detected!')
            self.recovery_mode = True
            self.recovery_start_time = time.time()
            self.recovery_direction = self.find_escape_direction()
            return
                
        # 장애물 감지 및 회피 - 목표에 도달하지 않은 경우에만 실행
        if not self.goal_reached and self.has_obstacle_ahead():
            self.get_logger().info('Obstacle detected ahead! Avoiding...')
            
            # 장애물 회피 방향 결정
            avoidance_direction = self.find_escape_direction()
            
            # 장애물 회피 명령 (감속 및 회전)
            self.cmd_msg.linear.x = self.avoidance_speed
            self.cmd_msg.angular.z = avoidance_direction * self.avoidance_turn_speed
            self.cmd_pub.publish(self.cmd_msg)
            return

        # 기존의 경로 추종 로직
        if not self.is_path:
            return

        if not self.tracking_enabled:
            return

        if len(self.path_msg.poses) > 1:
            self.is_look_forward_point = False

            # 로봇의 현재 위치를 나타내는 변수
            robot_pose_x = self.robot_pose_x
            robot_pose_y = self.robot_pose_y

            # 로봇이 경로에서 떨어진 거리를 나타내는 변수
            lateral_error = sqrt(
                pow(self.path_msg.poses[0].pose.position.x - robot_pose_x, 2) +
                pow(self.path_msg.poses[0].pose.position.y - robot_pose_y, 2)
            )

            # 전방 주시 거리 설정 - 수정됨
            # 속도와 오차에 덜 민감하게 반응하도록 계수 조정
            self.lfd = (self.status_msg.twist.linear.x + lateral_error) * 0.5

            if self.lfd < self.min_lfd:
                self.lfd = self.min_lfd
            if self.lfd > self.max_lfd:
                self.lfd = self.max_lfd

            min_dis = float('inf')
            forward_point = None

            # 전방 주시 포인트 설정
            for waypoint in self.path_msg.poses:
                current_point = waypoint.pose.position

                dis = sqrt(
                    pow(current_point.x - robot_pose_x, 2) +
                    pow(current_point.y - robot_pose_y, 2)
                )

                if abs(dis - self.lfd) < min_dis:
                    min_dis = abs(dis - self.lfd)
                    forward_point = current_point
                    self.is_look_forward_point = True

            if self.is_look_forward_point and forward_point is not None:
                global_forward_point = [forward_point.x, forward_point.y, 1]
                
                # 전방 주시 포인트와 로봇 헤딩과의 각도 계산
                trans_matrix = np.array([
                    [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                    [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                    [0, 0, 1]
                ])
                det_trans_matrix = np.linalg.inv(trans_matrix)
                local_forward_point = det_trans_matrix.dot(global_forward_point)
                theta = -atan2(local_forward_point[1], local_forward_point[0])

                # 선속도와 각속도 정하기
                out_vel = 1.0
                out_rad_vel = theta * 2

                # 메시지에 선속도 및 각속도 설정
                self.cmd_msg.linear.x = out_vel
                self.cmd_msg.angular.z = out_rad_vel

            else:
                print("전방 주시 포인트를 찾을 수 없습니다.")
                # 정지 명령 설정 (전방 포인트 없음)
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0

            # 속도 명령 메시지 발행
            self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        """로봇 위치 및 헤딩 업데이트"""
        self.is_odom = True
        self.odom_msg = msg

        # 로봇 위치 업데이트
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y

        # Quaternion → Euler 변환 (Yaw 값 설정)
        q = Quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        _, _, self.robot_yaw = q.to_euler()

    #경로 메시지 업데이트
    def path_callback(self, msg):
        self.is_path = True
        self.path_msg = msg

    #로봇 상태 메시지 업데이트
    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

def main(args=None):
    rclpy.init(args=args)

    path_tracker = FollowTheCarrot()

    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()