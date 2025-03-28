#웹소켓으로 목적지 입력 받고 좌표 서버로 전송하는 코드
import asyncio
import rclpy
import websockets
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class NavigationCommander(Node):
    def __init__(self, server_url):
        super().__init__('navigation_commander_client')
        
        # ROS2 퍼블리셔 생성
        self.publisher = self.create_publisher(String, '/navigation_command', 10)
        
        # ROS2 좌표 서브스크라이버 생성
        self.coordinate_subscriber = self.create_subscription(
            Point, 
            '/navigation_coordinates', 
            self.coordinate_callback, 
            10
        )
        
        # 서버 URL 저장
        self.server_url = server_url
        
        # 웹소켓 연결 상태 플래그
        self.is_connected = False
        
        # 웹소켓 연결 객체
        self.websocket = None
        
        # 비동기 태스크 생성을 위한 이벤트 루프
        self.loop = asyncio.get_event_loop()

        # 웹소켓 클라이언트 태스크 생성
        self.client_task = self.loop.create_task(self.connect_to_websocket_server())


    async def connect_to_websocket_server(self):
        try: 
            while True:
                try:
                    print(f"서버 연결 시도: {self.server_url}")  # 디버깅용 출력 추가
                    async with websockets.connect(
                        self.server_url 
                    ) as websocket:
                        self.websocket = websocket
                        self.is_connected = True
                        print(f"ROS2 클라이언트가 서버에 연결됨: {self.server_url}")
                        
                        # 서버에 ROS2 클라이언트 등록 메시지 JSON 형식으로 전송
                        registration_msg = {
                            "type": "register",
                            "client_type": "navigation",
                            "data": ""
                        }
                        await websocket.send(json.dumps(registration_msg))
                        
                        async for message in websocket:
                            try:
                                # JSON 메시지 파싱
                                parsed_msg = json.loads(message)
                                
                                # type이 message이고 client_type이 navigation인 경우에만 처리
                                if (parsed_msg.get('type') == 'message' and 
                                    parsed_msg.get('client_type') == 'navigation'):
                                    
                                    # ROS2 토픽으로 메시지 퍼블리시
                                    ros_msg = String()
                                    ros_msg.data = parsed_msg.get('data', '').replace(' ', '_')
                                    print(f"ROS2로 메시지 수신 및 퍼블리시: {ros_msg.data}")
                                    
                                    self.publisher.publish(ros_msg)
                                    self.get_logger().info(f'Published: {ros_msg.data}')
                            
                            except json.JSONDecodeError:
                                print(f"잘못된 JSON 형식: {message}")
                            except Exception as e:
                                print(f"메시지 처리 중 오류: {e}")
                
                except websockets.exceptions.ConnectionClosed:
                    print("WebSocket 연결 종료. 재연결 시도...")
                    self.is_connected = False
                    await asyncio.sleep(5)
                
                except Exception as e:
                    print(f"서버 연결 중 상세 오류: {e}")  # 구체적인 오류 메시지 출력
                    self.is_connected = False
                    await asyncio.sleep(5)
            
        except asyncio.CancelledError:
            print("WebSocket 연결 태스크 취소")

    


    def coordinate_callback(self, msg):
        # ROS2에서 좌표 메시지 수신 시 WebSocket 서버로 전송
        if not self.is_connected or self.websocket is None:
            print("WebSocket 서버에 연결되지 않았습니다.")
            return
        
        # 좌표 메시지를 JSON 형식으로 구성
        coordinate_msg = {
            "type": "message",
            "client_type": "navigation",
            "data": {
                "x": msg.x,
                "y": msg.y
            }
        }
        
        # 비동기 함수에서 메시지 전송
        async def send_coordinate():
            try:
                await self.websocket.send(json.dumps(coordinate_msg))
                print(f"좌표 메시지 전송: x={msg.x}, y={msg.y}")
            except Exception as e:
                print(f"좌표 메시지 전송 중 오류: {e}")
        
        # 이벤트 루프에 태스크 추가
        asyncio.run_coroutine_threadsafe(send_coordinate(), self.loop)

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # 서버 URL 지정
    server_url = "ws://(serverip):12345"
    
    # 노드 생성
    node = NavigationCommander(server_url)
    
    try:
        # ROS2 스핀과 웹소켓 클라이언트 동시 실행
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        # 웹소켓 클라이언트를 백그라운드 태스크로 실행
        node.loop.run_until_complete(node.client_task)
        
        # ROS2 스핀
        executor.spin()

    
    except KeyboardInterrupt:
        print("인터럽트")
    finally:
        # 정리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()