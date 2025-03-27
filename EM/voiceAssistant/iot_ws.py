import asyncio
import json
import websockets
import logging
from localServer_gcp import logger
from websockets.server import serve

class WebSocketServer:
    def __init__(self, host='0.0.0.0', port=12345):

        self.host = host
        self.port = port
        self.clients = set()
        self.client_info = {}  # 클라이언트 정보 저장
        self.devices = {
            "airConditioner": "OFF",
            "curtain": "OFF",
            "TV": "OFF"
        }
        
        # 로깅 설정
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
        )
        logger = logging.getLogger('websocket_server')
        
        self.server = None
    
    async def handle_connection(self, websocket):

        # 클라이언트 추가
        self.clients.add(websocket)
        client_ip = websocket.remote_address[0]
        logger.info(f"New client connected: {client_ip}")
        
        try:
            async for message in websocket:
                await self.process_message(websocket, message)
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"Client disconnected: {client_ip}, code: {e.code}, reason: {e.reason}")
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON format from client: {client_ip}")
        except Exception as e:
            logger.error(f"Error processing message: {str(e)}")
        finally:
            # 연결 종료 시 클라이언트 제거
            self.clients.remove(websocket)
            if websocket in self.client_info:
                del self.client_info[websocket]
            logger.info(f"Connection closed: {client_ip}")
    
    async def process_message(self, websocket, message):

        client_ip = websocket.remote_address[0]
        logger.info(f"Received from {client_ip}: {message}")
        
        data = json.loads(message)
        
        if "type" not in data:
            logger.error(f"Missing 'type' field in message from {client_ip}")
            return
        
        msg_type = data["type"]
        
        if msg_type == "sender":
            if "message" in data:
                self.client_info[websocket] = {"type": "sender", "ip": client_ip}
                logger.info(f"Sender registered: {client_ip}")
            else:
                logger.error(f"Missing 'message' field for sender type from {client_ip}")
        
        elif msg_type == "control":
            if "device" not in data or "state" not in data:
                logger.error(f"Missing 'device' or 'state' field for control type from {client_ip}")
                return
            
            device = data["device"]
            state = data["state"]
            
            if device not in self.devices:
                logger.error(f"Unknown device: {device} from {client_ip}")
                return
                
            if state not in ["ON", "OFF"]:
                logger.error(f"Invalid state: {state} from {client_ip}")
                return
            
            # 장치 상태 업데이트
            self.devices[device] = state
            logger.info(f"Device {device} set to {state}")
        
        else:
            logger.error(f"Unknown message type: {msg_type} from {client_ip}")
        
    async def broadcast(self, message):
        if not self.clients:
            return
        
        logger.info(f"Broadcasting to {len(self.clients)} clients: {message}")
        
        # 모든 클라이언트에게 메시지 전송
        await asyncio.gather(
            *[client.send(message) for client in self.clients],
            return_exceptions=True
        )
        
    def iotControl(self, device, state):
        # 유효한 장치와 상태인지 확인
        if device not in ["airConditioner", "curtain", "TV"]:
            logger.error(f"Invalid device: {device}")
            return False
            
        if state not in ["ON", "OFF"]:
            logger.error(f"Invalid state: {state}")
            return False
            
        # 장치 상태 업데이트
        self.devices[device] = state
        
        # JSON 메시지 생성
        message = {
            "type": "control",
            "device": device,
            "state": state
        }
        
        # 메시지를 문자열로 변환
        json_message = json.dumps(message)
        
        # 비동기 함수를 동기적으로 실행하기 위한 설정
        loop = asyncio.get_event_loop()
        if loop.is_running():
            # 이미 실행 중인 이벤트 루프가 있다면 새 태스크로 추가
            asyncio.create_task(self.broadcast(json_message))
        else:
            # 이벤트 루프가 실행 중이 아니라면 새로 실행
            loop.run_until_complete(self.broadcast(json_message))
            
        logger.info(f"IoT control message sent: {device} set to {state}")
        return True
    
    async def start_server(self):
        self.server = await serve(
            self.handle_connection, 
            self.host, 
            self.port
        )
        
        logger.info(f"WebSocket server started on {self.host}:{self.port}")
        
        # 서버가 종료될 때까지 대기
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
            logger.info("WebSocket server stopped")
        else:
            logger.warning("No server instance to stop")
    
    def stop(self):
        if hasattr(self, 'server'):
            asyncio.get_event_loop().run_until_complete(self.server.close())
            logger.info("WebSocket server stopped")
        else:
            logger.warning("No server instance to stop")


if __name__ == "__main__":
    # 기본 호스트와 포트로 서버 인스턴스 생성
    server = WebSocketServer()
    
    try:
        # 서버 시작
        print("Starting WebSocket server, press Ctrl+C to stop")
        server.start()
    except KeyboardInterrupt:
        print("Server stopped by user")
    
    # IoT 제어 예제
    # server.iotControl("airConditioner", "ON")
    # server.iotControl("curtain", "OFF")
    # server.iotControl("TV", "ON")