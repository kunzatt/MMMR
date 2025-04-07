#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class goalPub(Node):
    def __init__(self):
        super().__init__('goal_pub')
        
        # 위치 좌표 딕셔너리 (하드코딩)
        self.location_coordinates = {
            'living_room': (-8.51980, -6.02957, 0.0, 0.71, 0.695),
            'kitchen': (-8.86362 , -8.05053, 0.0, 0.88 , -0.474),
            'entrance': (-4.57842 , -7.14306, 0.0, 0.85, -0.502),
            'room1': (-12.47062, -5.92287, 0.0, 0.58, 0.817),
            'room2': (-5.20458, -6.06225, 0.0, 0.83, 0.55),
            'room3': (-2.55, -5.687176, 0.0, 0.90, 0.4234),
            'room4': (-12.51195, -7.22389, 0.0, 0.82, -0.574)
        }
        
        # Publisher/Subscriber 설정
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal', 10)
        self.command_subscriber = self.create_subscription(
            String,
            '/navigation_command',
            self.command_callback,
            10
        )
        
        self.get_logger().info("Navigation system ready")

    def command_callback(self, msg):
        location = msg.data.lower()  # 입력 메시지 소문자 변환
        self.get_logger().info(f"Received command: {location}")
        
        if location in self.location_coordinates:
            self.publish_goal(location)
        else:
            self.get_logger().warn(f"Unknown location: {location}")

    def publish_goal(self, location):
        x, y, z, w,ori_z = self.location_coordinates[location]
        
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.w = w
        goal_msg.pose.orientation.z = ori_z
        
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal for {location}: X={x}, Y={y}")

def main(args=None):
    rclpy.init(args=args)
    goalpub = goalPub()
    rclpy.spin(goalpub)
    goalpub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
