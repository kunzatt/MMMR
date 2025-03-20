import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,EnviromentStatus
from std_msgs.msg import Float32,Int8MultiArray, Int32
import time

# IoT 제어 코드 변수
IOT_ON = 1
IOT_OFF = 2
iot_idx = {"room1_light": 1, "room2_light": 2}
iot_control = {"room1_light": 1, "room2_light": 1}
iot_str = ["Entrance Light", "Room1 Light", "Room2 Light", "Room3 Light", "Room4 Light",
           "Kitchen Light", "LivingRoom Light", "Room1 AirConditioner", "Room2 AirConditioner",
           "Room3 AirConditioner", "LivingRoom AirConditioner", "AirPurifier", "TV", "Room1 Curtain",
           "Room2 Curtain", "Room3 Curtain", "LivingRoom Curtain"]
sim_month = ["", "January", "February", "March", "April", "May", "June", "July", "August", "September", "October",
             "November", "December"]

class Controller(Node):

    def __init__(self):
        super().__init__('sub1_controller')
        ## 메시지 송신을 위한 PUBLISHER 생성
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.app_control_pub = self.create_publisher(Int8MultiArray, 'app_control', 10)
        self.app_count_pub = self.create_publisher(Int32, 'app_cnt', 10)

        ## 메시지 수신을 위한 SUBSCRIBER 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.envir_status_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.envir_callback,10)
        self.app_status_sub = self.create_subscription(Int8MultiArray,'/app_status',self.app_callback,10)
        self.timer = self.create_timer(1, self.timer_callback)

        self.cnt = 0
        self.iotcnt = 0

        ## 제어 메시지 변수 생성 
        self.cmd_msg=Twist()
        self.cnt_msg = Int32()
        self.cnt_msg.data = self.cnt
        
        self.app_control_msg=Int8MultiArray()
        for i in range(17):
            self.app_control_msg.data.append(0)

        self.turtlebot_status_msg=TurtlebotStatus()
        self.envir_status_msg=EnviromentStatus()
        self.app_status_msg=Int8MultiArray()

        self.turtlebot_status_msg.battery_percentage = 100.0
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

    def timer_callback(self):
        menu = int(input("1) date  2) temperature  3) weather  4) IoT state : "))

        if menu == 1:
            self.print_date()
        elif menu == 2:
            self.print_temp()
        elif menu == 3:
            self.print_weather()
        elif menu == 4:
            self.print_IoT()
        else:
            print("invalid input. try again.")

    def print_date(self):

        print(f"{sim_month[self.envir_status_msg.month]} {self.envir_status_msg.day}"
              f"{'st' if self.envir_status_msg.day in [1, 21, 31] else 'nd' if self.envir_status_msg.day in [2, 22] else 'rd' if self.envir_status_msg.day in [3, 23] else 'th'} "
              f"{self.envir_status_msg.hour:02d}:{self.envir_status_msg.minute:02d}")

    def print_temp(self):
        print(f"Temperature: {self.envir_status_msg.temperature}")

    def print_weather(self):
        print(f"Weather: {self.envir_status_msg.weather}")
    
    def print_IoT(self):
        for i in range(17):
            print(f"{iot_str[i]}: {'ON' if self.app_status_msg.data[i] == 1 else 'OFF' if self.app_status_msg.data[i] == 2 else '상태유지'}")


def main(args=None):
    rclpy.init(args=args)
    sub1_controller = Controller()
    rclpy.spin(sub1_controller)
    sub1_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()