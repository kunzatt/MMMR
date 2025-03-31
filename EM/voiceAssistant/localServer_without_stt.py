import asyncio
import json
import logging
import os
import time
from typing import Dict, List, Set, Any, Optional

import websockets
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
from openai import OpenAI

import data_processor

# 환경 변수 설정
load_dotenv()

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger('integrated_server')

# 환경 변수
HOST = os.getenv("HOST", "0.0.0.0")
WS_PORT = int(os.getenv("WS_PORT", "12345"))
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

class IntegratedWebSocketServer:
    def __init__(self, host: str = '0.0.0.0', port: int = 12345):
        self.host = host
        self.port = port
        
        # 클라이언트 관리
        self.navigation_clients: Set[WebSocket] = set()
        self.iot_clients: Set[WebSocket] = set()
        self.mirror_clients: Set[WebSocket] = set()  # 라즈베리파이 클라이언트 (음성 비서)
        
        # 클라이언트 정보 매핑
        self.client_info: Dict[WebSocket, Dict[str, str]] = {}
        
        # 토큰 관리
        self.access_token: Optional[str] = None
        self.refresh_token: Optional[str] = None
        
        # OpenAI 클라이언트 초기화
        self.openai_client = OpenAI(api_key=OPENAI_API_KEY) if OPENAI_API_KEY else None
        
        # FastAPI 앱 초기화
        self.app = FastAPI(title="Integrated WebSocket Server")
        self.setup_app()
        
        # 서버 실행 중 상태
        self.running = True
    
    def setup_app(self) -> None:
        """FastAPI 앱 설정"""
        # CORS 설정
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # 시작/종료 이벤트 핸들러
        @self.app.on_event("startup")
        async def startup_event():
            # 초기 로그인
            tokens = data_processor.login()
            if tokens:
                self.access_token = tokens["access_token"]
                self.refresh_token = tokens["refresh_token"]
                logger.info("서비스 로그인 성공")
            else:
                logger.warning("서비스 로그인 실패")
                
            # OpenAI API 키 확인
            if not OPENAI_API_KEY:
                logger.warning("OPENAI_API_KEY가 설정되지 않았습니다. LLM 기능이 작동하지 않을 수 있습니다.")
        
        @self.app.on_event("shutdown")
        async def shutdown_event():
            logger.info("서버 종료 중...")
            self.running = False
            
            # 연결된 모든 클라이언트에 종료 알림 전송
            shutdown_msg = json.dumps({
                "type": "system",
                "contents": {
                    "default": "OFF",
                    "data": "서버가 종료됩니다."
                }
            })
            
            for client_set in [self.mirror_clients, self.iot_clients, self.navigation_clients]:
                if client_set:
                    await asyncio.gather(
                        *[self.try_send(client, shutdown_msg) for client in client_set],
                        return_exceptions=True
                    )
        
        # 루트 경로
        @self.app.get("/")
        async def root():
            return {"message": "Integrated WebSocket Server is running", "status": "ok"}
        
        # 웹소켓 경로
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await self.handle_connection(websocket)
    
    async def try_send(self, websocket: WebSocket, message: str) -> bool:
        """안전하게 메시지 전송 시도"""
        try:
            await websocket.send_text(message)
            return True
        except Exception:
            return False
    
    async def handle_connection(self, websocket: WebSocket) -> None:
        """웹소켓 연결 처리"""
        await websocket.accept()
        client_ip = websocket.client.host
        logger.info(f"새 클라이언트 연결됨: {client_ip}")
        
        try:
            while self.running:
                try:
                    # 텍스트 메시지 수신
                    message = await websocket.receive_text()
                    msg = json.loads(message)
                    response = await self.process_message(websocket, message)
                    logger.info(f"클라이언트 {client_ip}로부터 수신한 메시지: {msg}")
                    logger.info(f"클라이언트 {client_ip}에 응답: {response}")
                    if response:
                        await websocket.send_text(response)
                except WebSocketDisconnect:
                    logger.info(f"클라이언트 연결 종료: {client_ip}")
                    break
                except Exception as e:
                    logger.error(f"메시지 처리 중 오류: {e}")
                    try:
                        await websocket.send_text(json.dumps({
                            "type": "error",
                            "message": f"Message processing error: {str(e)}"
                        }))
                    except Exception:
                        # 연결이 이미 끊어진 경우
                        break
        finally:
            # 연결 종료 시 클라이언트 제거
            self.navigation_clients.discard(websocket)
            self.iot_clients.discard(websocket)
            self.mirror_clients.discard(websocket)
            
            if websocket in self.client_info:
                del self.client_info[websocket]
                
            logger.info(f"연결 종료됨: {client_ip}")
    
    async def process_message(self, websocket: WebSocket, message: str) -> Optional[str]:
        """메시지 처리"""
        client_ip = websocket.client.host
        logger.info(f"{client_ip}로부터 수신: {message}")
        
        try:
            data = json.loads(message)
            
            # 클라이언트 등록 처리
            if "type" in data and data["type"] == "register":
                return await self.handle_register(websocket, data, client_ip)
            
            # 음성 비서에서 온 음성 인식 텍스트 처리
            elif "text" in data and "keyword" in data and websocket in self.mirror_clients:
                return await self.handle_speech_text(websocket, data)
            
            # 이미 처리된 JSON 명령 처리 (기존 호환성)
            elif "type" in data and "contents" in data and websocket in self.mirror_clients:
                return await self.handle_voice_command(websocket, data)
            
            # 그 외 메시지는 그대로 응답 (에코)
            return message
            
        except json.JSONDecodeError:
            logger.error(f"잘못된 JSON 형식: {message}")
            return json.dumps({
                "type": "error",
                "message": "Invalid JSON format"
            })
        except Exception as e:
            logger.error(f"메시지 처리 중 오류: {str(e)}")
            import traceback
            logger.error(traceback.format_exc())
            return json.dumps({
                "type": "error",
                "message": f"Server error: {str(e)}"
            })
    
    async def handle_register(self, websocket: WebSocket, data: Dict[str, Any], client_ip: str) -> str:
        """클라이언트 등록 처리"""
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
                logger.info(f"네비게이션 클라이언트 등록됨: {client_ip}")
            elif client_type == "iot":
                self.iot_clients.add(websocket)
                logger.info(f"IoT 클라이언트 등록됨: {client_ip}")
            elif client_type == "mirror":
                self.mirror_clients.add(websocket)
                logger.info(f"미러 클라이언트 등록됨: {client_ip}")
            else:
                logger.warning(f"알 수 없는 클라이언트 유형: {client_type} ({client_ip})")
            
            # 등록 성공 응답
            return json.dumps({
                "type": "register_response",
                "status": "success",
                "client_type": client_type
            })
        else:
            logger.error(f"등록 메시지에 client_type 필드 없음: {client_ip}")
            return json.dumps({
                "type": "register_response",
                "status": "error",
                "message": "Missing client_type field"
            })
    
    async def handle_speech_text(self, websocket: WebSocket, data: Dict[str, Any]) -> str:
        """
        라즈베리파이에서 보낸 STT 처리된 텍스트를 LLM으로 해석하여 처리
        
        Args:
            websocket: 웹소켓 연결
            data: {"text": "인식된 텍스트", "keyword": "호출어"}
        """
        try:
            text = data.get("text", "")
            detected_keyword = data.get("keyword", "")
            
            if not text:
                logger.warning("음성 인식 텍스트가 없습니다.")
                return json.dumps({
                    "type": "none",
                    "contents": {
                        "default": "OFF",
                        "data": ""
                    },
                    "keyword": detected_keyword,
                    "result": "-1"
                })
            
            # 텍스트를 JSON으로 변환 (LLM 처리)
            json_result = await self.text_to_json(text, detected_keyword)
            
            # 명령 처리
            processed_result = await self.process_command(json_result)
            
            return processed_result
            
        except Exception as e:
            logger.error(f"음성 텍스트 처리 오류: {str(e)}")
            import traceback
            logger.error(traceback.format_exc())
            return json.dumps({
                "type": "error",
                "message": f"Text processing error: {str(e)}",
                "result": "-1",
                "keyword": data.get("keyword", "")
            })
    
    async def text_to_json(self, text: str, detected_keyword: str) -> str:
        """
        인식된 텍스트를 JSON 형식으로 변환 (LLM 사용)
        
        Args:
            text: 변환할 텍스트
            detected_keyword: 감지된 호출어
            
        Returns:
            str: JSON 형식 문자열
        """
        try:
            if not self.openai_client:
                raise ValueError("OpenAI 클라이언트가 초기화되지 않았습니다.")
                
            start_time = time.time()
            
            # 시스템 프롬프트 정의
            system_prompt = """당신은 한국어 텍스트를 분석하여 정확한 JSON으로 변환하는 전문가입니다.
다음 형식으로만 결과를 반환하세요:
{
    "type": "[카테고리]",
    "contents": {
        "default": "[ON/OFF]",
        "data": "[관련 데이터]"
    }
}

type은 다음 중 하나여야 합니다: "iot", "weather", "news", "youtube", "timer", "todo", "schedule", "time", "transportation", "none"

- iot: 전등, 조명, 가전제품 등의 제어 명령 (예: "전등 켜줘", "불 꺼줘", "TV 켜줘")
- weather: 날씨 정보 요청 (예: "오늘 날씨 어때?", "비 올 예정이야?")
- news: 뉴스 정보 요청 (예: "오늘 뉴스 보여줘", "최신 뉴스 알려줘", "3번째 뉴스 알려줘")
- youtube: 유튜브 관련 요청 (예: "유튜브 틀어줘", "음악 동영상 보여줘")
- timer: 타이머 설정 요청 (예: "5분 타이머 설정해줘", "30초 타이머")
- todo: 할 일 관련 요청 (예: "오늘 할 일 추가해줘", "할 일 목록 보여줘")
- schedule: 일정 관련 요청 (예: "내일 회의 일정 추가해줘", "이번 주 일정 알려줘")
- time: 시간 관련 요청 (예: "지금 몇 시야?", "시계 보여줘")
- transportation: 교통 정보 요청 (예: "버스 언제 와?", "지하철 운행 정보")
- none: 위 분류에 해당하지 않는 경우

contents.default는 기능을 켜는 명령의 경우 "ON", 끄는 명령인 경우 "OFF"로 설정합니다.

contents.data는 유형에 따라 다르게 설정합니다:
- iot: "light ON" 또는 "light OFF"와 같은 형태
- news: "1"부터 "5" 사이의 숫자 (뉴스 번호) 혹은 빈 문자열
- timer: "00H05M00S"와 같은 형태 (시간, 분, 초)
- youtube : "남자 요가 영상"와 같은 검색어
- 다른 유형: 빈 문자열

응답은 유효한 JSON 형식이어야 하며, 추가 설명이나 주석 없이 JSON만 반환합니다."""

            # OpenAI API 호출
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"다음 텍스트를 분석하여 JSON으로 변환해주세요: {text}"}
                ],
                response_format={"type": "json_object"},
                temperature=0.2
            )
            
            result = response.choices[0].message.content.strip()
            
            # 결과에 감지된 호출어 추가
            json_result = json.loads(result)
            json_result["keyword"] = detected_keyword
            
            # 기본 결과값 추가
            if json_result["type"] == "none":
                json_result["result"] = "-1"
            else:
                json_result["result"] = "0"  # 서버에서 처리할 결과값
                
            result = json.dumps(json_result)
            process_time = time.time() - start_time
            
            logger.info(f"JSON 변환 결과: {json_result}")
            logger.info(f"JSON 변환 시간: {process_time:.2f}초")
            
            return result
            
        except Exception as e:
            logger.error(f"JSON 변환 오류: {e}")
            import traceback
            logger.error(traceback.format_exc())
            
            # 오류 발생 시 기본값 반환
            return json.dumps({
                "type": "none",
                "contents": {
                    "default": "OFF",
                    "data": ""
                },
                "keyword": detected_keyword,
                "result": "-1"
            })
    
    async def handle_voice_command(self, websocket: WebSocket, data: Dict[str, Any]) -> str:
        """음성 비서 클라이언트에서 온 명령 처리 (이미 JSON 형식)"""
        return await self.process_command(json.dumps(data))
    
    async def process_command(self, json_command: str) -> str:
        """
        명령 JSON 처리
        
        Args:
            json_command: 처리할 JSON 명령 (문자열)
            
        Returns:
            str: 처리 결과 JSON 문자열
        """
        try:
            if isinstance(json_command, str):
                data = json.loads(json_command)
            else:
                data = json_command
                
            type_value = data.get("type", "none")
            contents = data.get("contents", {})
            
            # 뉴스 명령 처리
            if type_value == "news":
                if contents.get("data"):
                    if self.access_token:
                        news_id = int(contents["data"])
                        news_result, new_tokens = data_processor.getNews(
                            self.access_token, 
                            self.refresh_token, 
                            news_id
                        )
                        
                        # 토큰 갱신 처리
                        if new_tokens:
                            self.access_token = new_tokens["access_token"]
                            self.refresh_token = new_tokens["refresh_token"]
                        
                        if news_result:
                            data["result"] = news_result
                        else:
                            data["result"] = "-1"
                    else:
                        data["result"] = "로그인 정보가 없습니다."
                        logger.warning("액세스 토큰이 없어 뉴스 요청을 처리할 수 없습니다.")
                else:
                    data["result"] = "네, 알겠습니다."
            
            # IoT 명령 처리
            elif type_value == "iot":
                await self.broadcast_to_iot_clients(data)
                data["result"] = "IoT 장치에 명령을 전송했습니다."
            
            # 결과가 없는 경우 기본값 설정
            if "result" not in data:
                data["result"] = "-1"
            
            return json.dumps(data)
            
        except Exception as e:
            logger.error(f"명령 처리 중 오류: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return json.dumps({
                "type": "error",
                "message": f"Command processing error: {str(e)}",
                "result": "-1"
            })
    
    async def broadcast_to_iot_clients(self, message: Dict[str, Any]) -> None:
        """모든 IoT 클라이언트에 메시지 전송"""
        if not self.iot_clients:
            logger.warning("연결된 IoT 클라이언트가 없습니다.")
            return
        
        # dict를 JSON 문자열로 변환
        if isinstance(message, dict):
            message = json.dumps(message)
        
        logger.info(f"{len(self.iot_clients)}개의 IoT 클라이언트에 전송: {message}")
        
        # 비동기로 모든 IoT 클라이언트에 메시지 전송
        await asyncio.gather(
            *[self.try_send(client, message) for client in self.iot_clients],
            return_exceptions=True
        )
    
    async def broadcast_to_mirror_clients(self, message: Dict[str, Any]) -> None:
        """모든 미러 클라이언트에 메시지 전송"""
        if not self.mirror_clients:
            logger.warning("연결된 미러 클라이언트가 없습니다.")
            return
        
        # dict를 JSON 문자열로 변환
        if isinstance(message, dict):
            message = json.dumps(message)
        
        logger.info(f"{len(self.mirror_clients)}개의 미러 클라이언트에 전송: {message}")
        
        # 비동기로 모든 미러 클라이언트에 메시지 전송
        await asyncio.gather(
            *[self.try_send(client, message) for client in self.mirror_clients],
            return_exceptions=True
        )
    
    def send_to_clients(self, client_set: str, message: Dict[str, Any]) -> None:
        """특정 클라이언트 세트에 메시지 전송 (외부에서 호출용)"""
        if isinstance(message, dict):
            message = json.dumps(message)
        
        loop = asyncio.get_event_loop()
        if loop.is_running():
            # 이미 실행 중인 이벤트 루프가 있다면 새 태스크로 추가
            if client_set == "iot":
                asyncio.create_task(self.broadcast_to_iot_clients(message))
            elif client_set == "mirror":
                asyncio.create_task(self.broadcast_to_mirror_clients(message))
        else:
            # 이벤트 루프가 실행 중이 아니라면 새로 실행
            if client_set == "iot":
                loop.run_until_complete(self.broadcast_to_iot_clients(message))
            elif client_set == "mirror":
                loop.run_until_complete(self.broadcast_to_mirror_clients(message))
    
    def send_iot_message(self, message: Dict[str, Any]) -> None:
        """IoT 클라이언트에 메시지 전송 (호환성 유지용)"""
        self.send_to_clients("iot", message)
    
    def send_mirror_message(self, message: Dict[str, Any]) -> None:
        """미러 클라이언트에 메시지 전송"""
        self.send_to_clients("mirror", message)
    
    async def run(self) -> None:
        """서버 실행"""
        config = uvicorn.Config(
            self.app, 
            host=self.host, 
            port=self.port,
            log_level="info",
            reload=False
        )
        server = uvicorn.Server(config)
        
        logger.info(f"통합 WebSocket 서버 시작됨: {self.host}:{self.port}")
        await server.serve()

# 서버 실행 함수
def run_server(host: str = HOST, port: int = WS_PORT) -> None:
    """서버 실행 함수"""
    server = IntegratedWebSocketServer(host, port)
    
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        logger.info("사용자에 의해 서버가 중지되었습니다.")
    except Exception as e:
        logger.error(f"서버 실행 중 오류 발생: {e}")
        import traceback
        logger.error(traceback.format_exc())

# 메인 실행 부분
if __name__ == "__main__":
    run_server()