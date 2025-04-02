import asyncio
import json
import websockets
from websockets.server import serve
import logging
import data_processor
import re
import queue

message_queue = queue.Queue()

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
)

logger = logging.getLogger('websocket_server')
class WebSocketServer:
    def __init__(self, host='0.0.0.0', port=12345):
        self.host = host
        self.port = port
        self.clients = set()
        
        # 클라이언트 정보 저장 (유형별로 관리)
        self.navigation_clients = set()  # 주행 클라이언트 목록
        self.iot_clients = set()         # IoT 클라이언트 목록
        
        # 클라이언트 정보 매핑 (websocket -> info)
        self.client_info = {}  # 클라이언트 정보 저장
        self.access_token = None
        self.refresh_token = None

        self.message_queue = message_queue
    
    def update_tokens(self, access_token, refresh_token):
        self.access_token = access_token
        self.refresh_token = refresh_token
        logger.info(f"토큰 업데이트: access_token={access_token}, refresh_token={refresh_token}")
    
    async def handle_connection(self, websocket):
        # 클라이언트 추가
        self.clients.add(websocket)
        client_ip = websocket.remote_address[0]
        logger.info(f"새 클라이언트 연결됨: {client_ip}")
        
        try:
            async for message in websocket:
                # 메시지 처리 결과 반환
                response = await self.process_message(websocket, message)
                if response:
                    await websocket.send(response)
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"클라이언트 연결 해제: {client_ip}, 코드: {e.code}, 이유: {e.reason}")
        except json.JSONDecodeError:
            logger.error(f"클라이언트로부터 유효하지 않은 JSON 형식: {client_ip}")
        except Exception as e:
            logger.error(f"메시지 처리 오류: {str(e)}")
        finally:
            # 연결 종료 시 클라이언트 제거
            self.clients.remove(websocket)
            
            # 클라이언트 유형별 목록에서도 제거
            if websocket in self.navigation_clients:
                self.navigation_clients.remove(websocket)
            if websocket in self.iot_clients:
                self.iot_clients.remove(websocket)
                
            if websocket in self.client_info:
                del self.client_info[websocket]
                
            logger.info(f"연결 종료: {client_ip}")
    

    async def process_message(self, websocket, message):
        client_ip = websocket.remote_address[0]
        logger.info(f"{client_ip}로부터 수신: {message}")
        
        try:
            data = json.loads(message)
            
            if "type" in data and data["type"] == "register":
                if "client_type" in data:
                    client_type = data["client_type"]
                    
                    # 클라이언트 정보 저장
                    self.client_info[websocket] = {
                        "type": client_type,
                        "ip": client_ip
                    }
                    
                    # 클라이언트 유형별 목록에 추가
                    if client_type == "navigation":
                        self.navigation_clients.add(websocket)
                        logger.info(f"주행 클라이언트 등록됨: {client_ip}")
                    elif client_type == "iot":
                        self.iot_clients.add(websocket)
                        logger.info(f"IoT 클라이언트 등록됨: {client_ip}")
                    else:
                        logger.warning(f"알 수 없는 클라이언트 유형: {client_type} (IP: {client_ip})")
                    
                    # 등록 성공 응답
                    return json.dumps({
                        "type": "register_response",
                        "status": "success",
                        "client_type": client_type
                    })
                else:
                    logger.error(f"{client_ip}에서 등록 메시지에 client_type 필드 누락")
                    return json.dumps({
                        "type": "register_response",
                        "status": "error",
                        "message": "client_type 필드가 없습니다"
                    })
            elif "type" in data and data["type"] == "message":
                # 메시지 처리 (주행 클라이언트에 전송)
                if websocket in self.navigation_clients:
                    
                    return json.dumps({
                        "type": "message_response",
                        "status": "success",
                        "message": "주행 클라이언트에 메시지 전송됨"
                    })            
            return message
            
        except json.JSONDecodeError:
            logger.error(f"{client_ip}에서 유효하지 않은 JSON 형식")
            return json.dumps({
                "type": "error",
                "message": "유효하지 않은 JSON 형식"
            })
        except Exception as e:
            logger.error(f"메시지 처리 오류: {str(e)}")
            return json.dumps({
                "type": "error",
                "message": f"서버 오류: {str(e)}"
            })
    
    async def send_to_navigation(self, message):
        if not self.navigation_clients:
            logger.warning("연결된 주행 클라이언트가 없습니다")
            return
        
        if isinstance(message, dict) and message.get("type") == "homecam" and "contents" in message:
            transformed_message = {
                "type": "message",
                "client_type": "iot",
                "data": message["contents"].get("data", "")
            }
            message = transformed_message
    
        # dict를 JSON 문자열로 변환
        if isinstance(message, dict):
            message = json.dumps(message)
        
        logger.info(f"{len(self.navigation_clients)}개의 주행 클라이언트에 전송: {message}")

        for client in self.navigation_clients:
            client_ip = client.remote_address[0] if hasattr(client, 'remote_address') else "알 수 없음"
            client_info = self.client_info.get(client, {})
            logger.info(f"클라이언트에 전송: IP={client_ip}, 타입={client_info.get('type', '알 수 없음')}")
        
        # 비동기로 모든 주행 클라이언트에 메시지 전송
        await asyncio.gather(
            *[client.send(message) for client in self.navigation_clients],
            return_exceptions=True
        )
    
    async def send_to_iot(self, message):
        if not self.iot_clients:
            logger.warning("연결된 IoT 클라이언트가 없습니다")
            return
        
        if isinstance(message, dict) and message.get("type") == "control" and "contents" in message:
            logger.info(f"원본 메시지: {message}")
            
            # 디바이스 이름과 상태(ON/OFF) 파싱
            device = ""
            turned = ""
            temperature = ""
            brightness = ""
            
            # contents.data 파싱
            data = message.get("contents", {}).get("data", "")
            if data:
                # 지원하는 기기 목록
                devices_response, new_tokens = data_processor.getDevices(self.access_token, self.refresh_token)
                supported_devices = []
                device_id_map = {}
                if devices_response and "data" in devices_response:
                    for device_info in devices_response["data"]:
                        device_name = device_info.get("device")
                        if device_name:
                            supported_devices.append(device_name)
                            device_id_map[device_name] = device_info.get("id")
                
                # 온도 패턴 (예: "25도", "20도")
                temp_pattern = re.compile(r'(\d+)도')
                
                # 데이터에서 디바이스와 상태 추출
                for dev in supported_devices:
                    if dev in data:
                        device = dev
                        break
                
                # ON/OFF 상태 추출
                if "ON" in data:
                    turned = "ON"
                elif "OFF" in data:
                    turned = "OFF"
                
                # 온도 추출 (에어컨인 경우)
                if device == "airConditioner":
                    temp_match = temp_pattern.search(data)
                    if temp_match:
                        temperature = temp_match.group(1)
            
            # 명령이 없는 경우 contents.default에서 값 가져오기
            if not turned:
                default_value = message.get("contents", {}).get("default", "")
                if default_value in ["ON", "OFF"]:
                    turned = default_value
            
            # 결과 메시지 구성
            transformed_message = {
                "type": "control",
                "device": device,
                "data": {
                    "turned": turned,
                    "temperature": temperature,
                    "brightness": brightness
                }
            }
            
            logger.info(f"변환된 메시지: {transformed_message}")
            message = transformed_message

        # dict를 JSON 문자열로 변환
        if isinstance(message, dict):
            message = json.dumps(message)
        
        logger.info(f"전송할 메시지: {message}")        
        logger.info(f"{len(self.iot_clients)}개의 IoT 클라이언트에 전송: {message}")
        if len(self.iot_clients) > 0:
            new_tokens = data_processor.deviceUpdate(device_id_map[device], turned, access_token, refresh_token)
        
        # 비동기로 모든 IoT 클라이언트에 메시지 전송
        await asyncio.gather(
            *[client.send(message) for client in self.iot_clients],
            return_exceptions=True
        )
        return new_tokens
    
    def send_navigation_message(self, message):

        loop = asyncio.get_event_loop()
        if loop.is_running():
            # 이미 실행 중인 이벤트 루프가 있다면 새 태스크로 추가
            asyncio.create_task(self.send_to_navigation(message))
        else:
            # 이벤트 루프가 실행 중이 아니라면 새로 실행
            loop.run_until_complete(self.send_to_navigation(message))
        
        if isinstance(message, str):
            logger.info(f"주행 메시지 전송됨: {message}")
        else:
            logger.info(f"주행 메시지 전송됨: {json.dumps(message)}")
    
    def send_iot_message(self, message):
        loop = asyncio.get_event_loop()
        if loop.is_running():
            # 이미 실행 중인 이벤트 루프가 있다면 새 태스크로 추가
            asyncio.create_task(self.send_to_iot(message))
        else:
            # 이벤트 루프가 실행 중이 아니라면 새로 실행
            loop.run_until_complete(self.send_to_iot(message))
        
        if isinstance(message, str):
            logger.info(f"IoT 메시지 전송됨: {message}")
        else:
            logger.info(f"IoT 메시지 전송됨: {json.dumps(message)}")
    
    async def start_server(self):
        self.server = await serve(
            self.handle_connection, 
            self.host, 
            self.port
        )
        
        logger.info(f"웹소켓 서버가 {self.host}:{self.port}에서 시작되었습니다")
        return self.server
    
    def start(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.start_server())
        loop.run_forever()
    
    async def stop_server(self):
        if hasattr(self, 'server'):
            self.server.close()
            await self.server.wait_closed()
            logger.info("웹소켓 서버가 중지되었습니다")
        else:
            logger.warning("중지할 서버 인스턴스가 없습니다")
    
    def stop(self):
        if hasattr(self, 'server'):
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.stop_server())
            logger.info("웹소켓 서버가 중지되었습니다")
        else:
            logger.warning("중지할 서버 인스턴스가 없습니다")


if __name__ == "__main__":
    server = WebSocketServer()
    
    try:
        # 서버 시작
        print("웹소켓 서버를 시작합니다. 중지하려면 Ctrl+C를 누르세요")
        server.start()
    except KeyboardInterrupt:
        print("사용자에 의해 서버가 중지되었습니다")