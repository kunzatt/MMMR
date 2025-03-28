import asyncio
import json
import websockets
from localServer_gcp import logger
from websockets.server import serve
"""
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
)

logger = logging.getLogger('websocket_server')
"""
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
    
    async def handle_connection(self, websocket):
        # 클라이언트 추가
        self.clients.add(websocket)
        client_ip = websocket.remote_address[0]
        logger.info(f"New client connected: {client_ip}")
        
        try:
            async for message in websocket:
                # 메시지 처리 결과 반환
                response = await self.process_message(websocket, message)
                if response:
                    await websocket.send(response)
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"Client disconnected: {client_ip}, code: {e.code}, reason: {e.reason}")
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON format from client: {client_ip}")
        except Exception as e:
            logger.error(f"Error processing message: {str(e)}")
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
                
            logger.info(f"Connection closed: {client_ip}")
    
    async def process_message(self, websocket, message):
        client_ip = websocket.remote_address[0]
        logger.info(f"Received from {client_ip}: {message}")
        
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
                        logger.info(f"Registered navigation client: {client_ip}")
                    elif client_type == "iot":
                        self.iot_clients.add(websocket)
                        logger.info(f"Registered IoT client: {client_ip}")
                    else:
                        logger.warning(f"Unknown client type: {client_type} from {client_ip}")
                    
                    # 등록 성공 응답
                    return json.dumps({
                        "type": "register_response",
                        "status": "success",
                        "client_type": client_type
                    })
                else:
                    logger.error(f"Missing client_type in register message from {client_ip}")
                    return json.dumps({
                        "type": "register_response",
                        "status": "error",
                        "message": "Missing client_type field"
                    })
            
            # 그 외 모든 메시지는 원본 그대로 반환 (에코)
            return message
            
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON format from {client_ip}")
            return json.dumps({
                "type": "error",
                "message": "Invalid JSON format"
            })
        except Exception as e:
            logger.error(f"Error processing message: {str(e)}")
            return json.dumps({
                "type": "error",
                "message": f"Server error: {str(e)}"
            })
    
    async def send_to_navigation(self, message):
        if not self.navigation_clients:
            logger.warning("No navigation clients connected")
            return
        
        # dict를 JSON 문자열로 변환
        if isinstance(message, dict):
            message = json.dumps(message)
        
        logger.info(f"Sending to {len(self.navigation_clients)} navigation clients: {message}")
        
        # 비동기로 모든 주행 클라이언트에 메시지 전송
        await asyncio.gather(
            *[client.send(message) for client in self.navigation_clients],
            return_exceptions=True
        )
    
    async def send_to_iot(self, message):
        if not self.iot_clients:
            logger.warning("No IoT clients connected")
            return
        
        # dict를 JSON 문자열로 변환
        if isinstance(message, dict):
            message = json.dumps(message)
        
        logger.info(f"Sending to {len(self.iot_clients)} IoT clients: {message}")
        
        # 비동기로 모든 IoT 클라이언트에 메시지 전송
        await asyncio.gather(
            *[client.send(message) for client in self.iot_clients],
            return_exceptions=True
        )
    
    def send_navigation_message(self, message):

        loop = asyncio.get_event_loop()
        if loop.is_running():
            # 이미 실행 중인 이벤트 루프가 있다면 새 태스크로 추가
            asyncio.create_task(self.send_to_navigation(message))
        else:
            # 이벤트 루프가 실행 중이 아니라면 새로 실행
            loop.run_until_complete(self.send_to_navigation(message))
        
        if isinstance(message, str):
            logger.info(f"Navigation message sent: {message}")
        else:
            logger.info(f"Navigation message sent: {json.dumps(message)}")
    
    def send_iot_message(self, message):
        loop = asyncio.get_event_loop()
        if loop.is_running():
            # 이미 실행 중인 이벤트 루프가 있다면 새 태스크로 추가
            asyncio.create_task(self.send_to_iot(message))
        else:
            # 이벤트 루프가 실행 중이 아니라면 새로 실행
            loop.run_until_complete(self.send_to_iot(message))
        
        if isinstance(message, str):
            logger.info(f"IoT message sent: {message}")
        else:
            logger.info(f"IoT message sent: {json.dumps(message)}")
    
    async def start_server(self):
        self.server = await serve(
            self.handle_connection, 
            self.host, 
            self.port
        )
        
        logger.info(f"WebSocket server started on {self.host}:{self.port}")
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
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.stop_server())
            logger.info("WebSocket server stopped")
        else:
            logger.warning("No server instance to stop")


if __name__ == "__main__":
    server = WebSocketServer()
    
    try:
        # 서버 시작
        print("Starting WebSocket server, press Ctrl+C to stop")
        server.start()
    except KeyboardInterrupt:
        print("Server stopped by user")
    
    # 사용 예제:
    # 주행 클라이언트에 메시지 전송
    # server.send_navigation_message({
    #     "type": "command",
    #     "action": "move",
    #     "direction": "forward",
    #     "speed": 5
    # })
    
    # IoT 클라이언트에 메시지 전송
    # server.send_iot_message({
    #     "type": "control",
    #     "device": "airConditioner",
    #     "state": "ON"
    # })