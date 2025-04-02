#웹소켓으로 목적지 입력 받고 좌표 서버로 전송하는 코드

import asyncio
import rclpy
import websockets
import json
import threading
import queue
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
        
        # 웹소켓 연결 객체 (스레드 간 공유)
        self.websocket = None
        
        # 스레드 안전한 파이썬 큐 사용 (asyncio.Queue 대신)
        self.coordinate_queue = queue.Queue()
        
        # 웹소켓 스레드 종료 이벤트
        self.exit_event = threading.Event()
        
        # 디버깅을 위한 카운터 추가
        self.msg_counter = 0
        
        # 웹소켓 스레드 시작
        self.websocket_thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        self.websocket_thread.start()
        
        self.get_logger().info("NavigationCommander 클라이언트 초기화 완료")

    def run_websocket_client(self):
        """웹소켓 클라이언트를 별도 스레드에서 실행"""
        # 이 스레드 전용 이벤트 루프 생성
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # 웹소켓 클라이언트와 큐 처리 코루틴 실행
            loop.run_until_complete(
                self.websocket_client_handler(loop)
            )
        except Exception as e:
            print(f"웹소켓 클라이언트 스레드 오류: {e}")
        finally:
            loop.close()
            print("웹소켓 클라이언트 스레드 종료")

    async def websocket_client_handler(self, loop):
        """웹소켓 클라이언트 처리 메인 코루틴"""
        # 주기적으로 큐를 확인하는 태스크 생성
        queue_handler = asyncio.create_task(self.process_coordinate_queue(loop))
        
        # 연결 처리 태스크 생성
        connection_handler = asyncio.create_task(self.connect_to_websocket_server(loop))
        
        # 종료 이벤트 확인 태스크 생성
        exit_check = asyncio.create_task(self.check_exit_event(loop))
        
        # 모든 태스크가 완료되거나 예외가 발생할 때까지 대기
        await asyncio.gather(
            queue_handler, 
            connection_handler,
            exit_check,
            return_exceptions=True
        )

    async def check_exit_event(self, loop):
        """종료 이벤트를 비동기적으로 확인"""
        while not self.exit_event.is_set():
            await asyncio.sleep(0.1)  # 짧은 대기 후 다시 확인
        
        # 종료 요청 발생 시 모든 태스크 취소
        for task in asyncio.all_tasks(loop):
            if task is not asyncio.current_task():
                task.cancel()

    async def process_coordinate_queue(self, loop):
        """파이썬 표준 큐에서 메시지를 주기적으로 확인하고 처리"""
        while not self.exit_event.is_set():
            # 큐에 메시지가 있는지 확인 (논블로킹)
            try:
                # 큐에서 메시지 가져오기 (0.1초 타임아웃)
                coordinate_msg = self.coordinate_queue.get(block=True, timeout=0.1)
                
                # 웹소켓이 연결되어 있는지 확인
                if self.is_connected and self.websocket is not None:
                    try:
                        # JSON 메시지 전송
                        await self.websocket.send(json.dumps(coordinate_msg))
                        print(f"좌표 메시지 전송 성공: {coordinate_msg}")
                    except Exception as e:
                        print(f"좌표 메시지 전송 실패: {e}")
                else:
                    print("웹소켓 연결이 없어 메시지를 전송할 수 없습니다.")
                
                # 큐 작업 완료 표시
                self.coordinate_queue.task_done()
                
            except queue.Empty:
                # 큐가 비어있으면 잠시 대기
                await asyncio.sleep(0.1)
            except Exception as e:
                print(f"큐 처리 중 오류: {e}")
                await asyncio.sleep(0.1)

    async def connect_to_websocket_server(self, loop):
        """웹소켓 서버에 연결하고 메시지를 처리하는 코루틴"""
        while not self.exit_event.is_set():
            try:
                print(f"서버 연결 시도: {self.server_url}")
                async with websockets.connect(self.server_url) as websocket:
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
                    print("등록 메시지 전송 완료")
                    
                    # 서버로부터 메시지 수신 및 처리
                    try:
                        async for message in websocket:
                            try:
                                # JSON 메시지 파싱
                                parsed_msg = json.loads(message)
                                print(f"서버로부터 메시지 수신: {parsed_msg}")
                                
                                # type이 message이고 client_type이 navigation인 경우에만 처리
                                if (parsed_msg.get('type') == 'message' and 
                                    parsed_msg.get('client_type') == 'navigation'):
                                    
                                    # ROS2 토픽으로 메시지 퍼블리시
                                    ros_msg = String()
                                    ros_msg.data = str(parsed_msg.get('data', '')).replace(' ', '_')
                                    print(f"ROS2로 메시지 발행: {ros_msg.data}")
                                    
                                    # ROS2 메시지는 메인 스레드에서 처리되어야 함
                                    self.publisher.publish(ros_msg)
                            
                            except json.JSONDecodeError:
                                print(f"잘못된 JSON 형식: {message}")
                            except Exception as e:
                                print(f"메시지 처리 중 오류: {e}")
                    
                    except websockets.exceptions.ConnectionClosed:
                        print("웹소켓 연결이 서버에 의해 종료됨")
                    
                    finally:
                        self.is_connected = False
                        self.websocket = None
            
            except websockets.exceptions.ConnectionClosed:
                print("WebSocket 연결 종료. 재연결 시도...")
                self.is_connected = False
                self.websocket = None
                await asyncio.sleep(5)
            
            except Exception as e:
                print(f"서버 연결 중 상세 오류: {e}")
                self.is_connected = False
                self.websocket = None
                await asyncio.sleep(5)

    def coordinate_callback(self, msg):
        """ROS2에서 좌표 메시지 수신 시 처리하는 콜백 함수"""
        # 메시지 카운터 증가
        self.msg_counter += 1
        
        # 로그 출력
        print("\n" + "=" * 50)
        print(f"좌표 메시지 #{self.msg_counter} 수신:")
        print(f"x: {msg.x}, y: {msg.y}, z: {msg.z}")
        print("=" * 50)
        
        # ROS2 로그에도 출력
        self.get_logger().info(f'좌표 수신 #{self.msg_counter}: x={msg.x}, y={msg.y}, z={msg.z}')
        
        # 좌표 메시지를 JSON 형식으로 구성
        coordinate_msg = {
            "type": "message", 
            "client_type": "navigation",
            "data": {
                "x": int(msg.x),  # float 타입으로 명시적 변환
                "y": int(msg.y)
            }
        }
        
        print(f"큐에 메시지 추가: {coordinate_msg}")
        
        # 스레드 안전한 큐에 메시지 추가
        try:
            self.coordinate_queue.put(coordinate_msg)
            print("메시지가 큐에 성공적으로 추가됨")
        except Exception as e:
            print(f"메시지를 큐에 추가하는 중 오류: {e}")

    def shutdown(self):
        """노드 종료 시 정리 작업"""
        print("NavigationCommander 종료 중...")
        self.exit_event.set()  # 웹소켓 스레드에 종료 신호 전송
        if self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)  # 최대 2초간 스레드 종료 대기

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # 서버 URL 지정
    server_url = "ws://70.12.246.31:12345"
    
    # 노드 생성
    node = NavigationCommander(server_url)
    
    try:
        # ROS2 스핀만 실행 (웹소켓은 별도 스레드에서 실행됨)
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        print("인터럽트 - 프로그램 종료")
    finally:
        # 정리
        node.shutdown()  # 웹소켓 스레드 정리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()