import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)

        # 제어 주기 설정 (0.05초)
        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom=False
        self.is_path=False
        self.is_status=False

        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()

        # 제어 파라미터
        self.lfd=0.1
        self.min_lfd=0.1
        self.max_lfd=1.0
        self.lfd_gain=0.8  # 전방주시거리 계수

    def timer_callback(self):
        if self.is_status and self.is_odom and self.is_path:
            if len(self.path_msg.poses)> 1:
                self.is_look_forward_point= False
                
                # 로봇 현재 위치
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y

                # 경로 이탈 거리 계산
                lateral_error = sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2) +
                                   pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))

                # 전방주시거리 계산
                self.lfd = self.lfd_gain * self.status_msg.twist.linear.x
                self.lfd = max(min(self.lfd, self.max_lfd), self.min_lfd)

                # 전방 주시 포인트 탐색
                min_dis = float('inf')
                for waypoint in self.path_msg.poses:
                    current_point = waypoint.pose.position
                    dx = current_point.x - robot_pose_x
                    dy = current_point.y - robot_pose_y
                    dis = sqrt(dx**2 + dy**2)
                    
                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = current_point
                        self.is_look_forward_point = True

                if self.is_look_forward_point:
                    # 좌표계 변환 행렬 생성
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]
                    ])
                    
                    # 역변환 행렬 계산
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    global_forward_point = np.array([[self.forward_point.x], [self.forward_point.y], [1]])
                    local_forward_point = det_trans_matrix @ global_forward_point
                    
                    # 목표 각도 계산
                    theta = atan2(local_forward_point[1,0], local_forward_point[0,0])

                    # 제어값 계산
                    out_vel = 0.3  # 고정 선속도
                    out_rad_vel = 1.5 * theta  # 각속도 제어

                    self.cmd_msg.linear.x = out_vel
                    self.cmd_msg.angular.z = out_rad_vel
                else:
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    
                self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        
        # 쿼터니언 -> 오일러 변환
        q = Quaternion(
            x=msg.pose.pose.orientation.x,
            y=msg.pose.pose.orientation.y,
            z=msg.pose.pose.orientation.z,
            w=msg.pose.pose.orientation.w
        )
        _, _, self.robot_yaw = q.to_euler()

    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg

    def status_callback(self, msg):
        self.is_status=True
        self.status_msg=msg

def main(args=None):
    rclpy.init(args=args)
    path_tracker = followTheCarrot()
    rclpy.spin(path_tracker)
    path_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
