import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2
import numpy as np

# path_tracking 노드는 로봇의 위치(/odom), 로봇의 속도(/turtlebot_status), 주행 경로(/local_path)를 받아서,
# 주어진 경로를 따라가게 하는 제어 입력값(/cmd_vel)을 계산합니다.
# 제어입력값은 선속도와 각속도로 두 가지를 구합니다.


class FollowTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)

        # 로직 1. 제어 주기 및 타이머 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        # 상태 플래그 초기화
        self.is_odom = False
        self.is_path = False
        self.is_status = False

        # 메시지 초기화
        self.odom_msg = Odometry()
        self.robot_yaw = 0.0
        self.path_msg = Path()
        self.cmd_msg = Twist()

        # 로직 2. 파라미터 설정
        self.lfd = 0.1  # 전방 주시 거리 (look forward distance)
        self.min_lfd = 0.1
        self.max_lfd = 1.0

        # 로봇 위치 변수 초기화 (오류 방지)
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0

    def timer_callback(self):
        if not (self.is_status and self.is_odom and self.is_path):
            # 필요한 데이터가 모두 수신되지 않았을 경우 로그 출력 후 종료
            self.get_logger().warning('필요한 데이터가 모두 수신되지 않았습니다.')
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
            print(robot_pose_x, robot_pose_y, lateral_error)

            # 로직 4. 전방 주시 거리 설정
            self.lfd = (self.status_msg.twist.linear.x + lateral_error) * 0.5

            if self.lfd < self.min_lfd:
                self.lfd = self.min_lfd
            if self.lfd > self.max_lfd:
                self.lfd = self.max_lfd

            min_dis = float('inf')

            # 로직 5. 전방 주시 포인트 설정
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

            if self.is_look_forward_point:
                global_forward_point = [forward_point.x, forward_point.y, 1]

                # 로직 6. 전방 주시 포인트와 로봇 헤딩과의 각도 계산
                trans_matrix = np.array([
                    [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                    [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                    [0, 0, 1]
                ])
                det_trans_matrix = np.linalg.inv(trans_matrix)
                local_forward_point = det_trans_matrix.dot(global_forward_point)
                theta = -atan2(local_forward_point[1], local_forward_point[0])

                # 로직 7. 선속도와 각속도 정하기
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

    def path_callback(self, msg):
        """경로 메시지 업데이트"""
        self.is_path = True
        self.path_msg = msg

    def status_callback(self, msg):
        """로봇 상태 메시지 업데이트"""
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
