import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from squaternion import Quaternion
from math import pi, atan2, sqrt

class SimpleOrientationAligner(Node):
    def __init__(self):
        super().__init__('simple_orientation_aligner')
        
        # 퍼블리셔 및 서브스크라이버 설정
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/rotation', self.goal_callback, 10)
        
        # 제어 주기 설정
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 상태 변수 초기화
        self.is_odom = False
        self.is_goal = False
        self.aligning_complete = False
        
        # 쿼터니언 값 저장
        self.current_w = 0.0
        self.current_z = 0.0
       
        self.target_w = 0.0
        self.target_z = 0.0

       
        
        # 회전 속도 설정
        self.rotation_speed = 0.4  # 낮은 회전 속도로 설정
        
        # 허용 오차 - 더 크게 설정
        self.quaternion_tolerance = 0.02
        
        self.get_logger().info('Simple Orientation Aligner Node has been initialized')
        
    def odom_callback(self, msg):
        """오도메트리 메시지 수신 콜백"""
        self.is_odom = True
        self.current_w = msg.pose.pose.orientation.w
        self.current_z = msg.pose.pose.orientation.z
        #print(f"current orientation : {self.current_w}, {self.current_z}")
        
    def goal_callback(self, msg):
        """목표 방향 수신 콜백"""
        self.is_goal = True
        self.aligning_complete = False
        self.target_w = msg.pose.orientation.w
        self.target_z = msg.pose.orientation.z

        self.stop_robot()
        
        self.get_logger().info(f'목표 방향 수신: w={self.target_w}, z={self.target_z}')
    
    def stop_robot(self):
        cmd_msg = Twist()
        cmd_msg.angular.z = 0.0
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info('로봇 정지')

    def timer_callback(self):
        if not self.is_odom or not self.is_goal:
            return
        
        if self.aligning_complete:
            self.stop_robot()  # 목표에 도달한 후에도 계속해서 정지 명령을 보냅니다.
            return
        
        # w와 z 값의 차이 계산
        w_diff = abs(self.current_w - self.target_w)
        z_diff = abs(self.current_z - self.target_z)
        
        # 항상 현재 값과 목표 값, 그리고 차이를 로그로 출력
        self.get_logger().info(f'current: w={self.current_w:.4f}, z={self.current_z:.4f}, goal: w={self.target_w:.4f}, z={self.target_z:.4f}, diff: w_diff={w_diff:.4f}, z_diff={z_diff:.4f}')
        
        # 목표 방향에 도달했는지 확인 - 두 가지 방식으로 체크
        if (w_diff <= self.quaternion_tolerance and z_diff <= self.quaternion_tolerance):
            # 정렬 완료, 로봇 정지
            cmd_msg = Twist()
            cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            self.aligning_complete = True
            self.get_logger().info('방향 정렬 완료! 로봇 정지')
        else:
            # 한쪽 방향으로 계속 회전
            cmd_msg = Twist()
            cmd_msg.angular.z = self.rotation_speed
            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f'회전 중: 속도={self.rotation_speed}')

def main(args=None):
    rclpy.init(args=args)
    aligner = SimpleOrientationAligner()
    
    try:
        rclpy.spin(aligner)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 로봇 정지 명령 전송
        stop_cmd = Twist()
        aligner.cmd_pub.publish(stop_cmd)
        aligner.get_logger().info('노드 종료, 로봇 정지')
        aligner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
