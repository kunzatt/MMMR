import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,EnviromentStatus
from std_msgs.msg import Float32,Int8MultiArray

class Controller(Node):

    def __init__(self):
        super().__init__('sub1_controller')
        ## 메시지 송신을 위한 PUBLISHER 생성
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.app_control_pub = self.create_publisher(Int8MultiArray, 'app_control', 10)

        ## 메시지 수신을 위한 SUBSCRIBER 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.envir_status_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.envir_callback,10)
        self.app_status_sub = self.create_subscription(Int8MultiArray,'/app_status',self.app_callback,10)
        self.timer = self.create_timer(1.5, self.timer_callback)
        self.cnt = 0
        self.iotcnt = 0

        ## 제어 메시지 변수 생성 
        self.cmd_msg=Twist()
        
        self.app_control_msg=Int8MultiArray()
        for i in range(17):
            self.app_control_msg.data.append(0)


        self.turtlebot_status_msg=TurtlebotStatus()
        self.envir_status_msg=EnviromentStatus()
        self.app_status_msg=Int8MultiArray()
        self.is_turtlebot_status=False
        self.is_app_status=False
        self.is_envir_status=False


    def listener_callback(self, msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg

    def envir_callback(self, msg):
        self.is_envir_status=True
        self.envir_status_msg=msg

    def app_callback(self, msg):
        self.is_app_status=True
        self.app_status_msg=msg  

    def app_all_on(self):
        print("on")
        for i in range(17):
            self.app_control_msg.data[i]=1
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_all_off(self):
        for i in range(17):
            self.app_control_msg.data[i]=2
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_on_select(self,num):
        # 특정 가전 제품 ON
        self.app_control_msg.data[num] = 1
        self.app_control_pub.publish(self.app_control_msg)

    def app_off_select(self,num):
        # 특정 가전 제품 OFF
        self.app_control_msg.data[num] = 2
        self.app_control_pub.publish(self.app_control_msg)

    def turtlebot_go(self) :
        self.cmd_msg.linear.x=0.3
        self.cmd_msg.angular.z=0.0

    def turtlebot_stop(self) :
        # 터틀봇 정지
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=0.0

    def turtlebot_cw_rot(self) :
        # 터틀봇 시계방향 회전
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=1.0

    def turtlebot_cww_rot(self) :
        # 터틀봇 반시계방향 회전
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=-1.0


    def timer_callback(self):
        print(f"linear: {self.turtlebot_status_msg.twist.linear.x}, angular: {self.turtlebot_status_msg.twist.linear.z}")
        print(f"battery percentage: {self.turtlebot_status_msg.battery_percentage}, charge status: {self.turtlebot_status_msg.power_supply_status}")
        print(f"date: {self.envir_status_msg.month}/{self.envir_status_msg.day} {self.envir_status_msg.hour}:{self.envir_status_msg.minute}, temperature: {self.envir_status_msg.temperature}, weather: {self.envir_status_msg.weather}")
        print(f"app status: {self.app_status_msg}")
        print("count: " + str(self.cnt) + ", IOT count: " + str(self.iotcnt))
        

        ## IOT(가전) 제어 함수
        if self.iotcnt % 19 == 17:
            self.app_all_on()
        elif self.iotcnt % 19 == 18:
            self.app_all_off()
        else:
            if self.app_control_msg.data[self.iotcnt % 19] == 1:
                self.app_off_select(self.iotcnt % 19)
            else:
                self.app_on_select(self.iotcnt % 19)


        ## 터틀봇 제어 함수
        if self.cnt % 4 == 0:
            self.turtlebot_go()
        elif self.cnt % 4 == 1:
            self.turtlebot_stop()
        elif self.cnt % 4 == 2:
            self.turtlebot_cw_rot()
        else:
            self.turtlebot_cww_rot()

        self.cmd_publisher.publish(self.cmd_msg)
        self.cnt += 1
        self.iotcnt += 1


def main(args=None):
    rclpy.init(args=args)
    sub1_controller = Controller()
    rclpy.spin(sub1_controller)
    sub1_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()