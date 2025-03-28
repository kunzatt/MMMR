import asyncio
import numpy as np
import os
import time
import torch
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import logging
import json
from typing import Dict, List, Any
import openai
import data_processor
from google.cloud import speech

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("stt-server")

load_dotenv()

# 환경 변수 설정
LANGUAGE = os.getenv("LANGUAGE", "ko-KR")
HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8000"))
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

# OpenAI API 키 설정
openai.api_key = OPENAI_API_KEY

# 음성 감지 설정
ENERGY_THRESHOLD = float(os.getenv("ENERGY_THRESHOLD", "0.02"))
SILENCE_THRESHOLD = int(os.getenv("SILENCE_THRESHOLD", "25"))
MIN_AUDIO_LENGTH = float(os.getenv("MIN_AUDIO_LENGTH", "0.5"))

app = FastAPI(title="Speech-to-Text WebSocket Server")

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Google Cloud Speech 클라이언트 초기화
speech_client = None

# 서버 시작 시 Google Cloud Speech 클라이언트 초기화
@app.on_event("startup")
async def startup_event():
    global speech_client
    logger.info("서버 시작 중...")
    
    # OpenAI API 키 확인
    if not OPENAI_API_KEY:
        logger.warning("OPENAI_API_KEY가 설정되지 않았습니다. JSON 변환 기능이 작동하지 않을 수 있습니다.")
    
    # Google Cloud 인증 확인
    google_credentials = os.environ.get('GOOGLE_APPLICATION_CREDENTIALS')
    if not google_credentials:
        logger.warning("GOOGLE_APPLICATION_CREDENTIALS가 설정되지 않았습니다. Google Speech-to-Text API가 작동하지 않을 수 있습니다.")
    else:
        logger.info(f"Google Cloud 인증 파일: {google_credentials}")
    
    # Google Cloud Speech 클라이언트 초기화
    try:
        speech_client = speech.SpeechClient()
        logger.info("Google Cloud Speech 클라이언트 초기화 완료!")
    except Exception as e:
        logger.error(f"Google Cloud Speech 클라이언트 초기화 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())

class AudioProcessor:
    def __init__(self, sample_rate: int = 16000):
        self.sample_rate = sample_rate
        self.audio_buffer: List[np.ndarray] = []
        self.is_speaking = False
        self.silence_counter = 0
        self.total_frames = 0
        self.metadata = {}
        self.frame_buffer = bytearray()  # 바이트 프레임 버퍼
        self.channels = 1  # 기본값
        self.encoding = "PCM_16BIT"  # 기본값
        self.received_bytes = 0  # 수신된 총 바이트 수 추적
    
    def reset(self):
        """음성 처리 상태 초기화"""
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.total_frames = 0
        self.frame_buffer = bytearray()
        self.received_bytes = 0
    
    def process_frame(self, frame_data: bytes) -> bool:
        """
        오디오 프레임 처리
        
        Args:
            frame_data: 바이너리 오디오 데이터
            
        Returns:
            bool: 음성 종료 감지 여부
        """
        # 수신된 데이터 크기 기록 (디버깅용)
        self.received_bytes += len(frame_data)
        
        # 프레임 버퍼에 추가
        self.frame_buffer.extend(frame_data)
        
        # 정렬된 샘플 처리를 위해 적절한 크기(2바이트 정수)로 나눠지는지 확인
        bytes_per_sample = 2  # 16비트 PCM
        bytes_per_frame = bytes_per_sample * self.channels
        
        # 나머지 계산
        remainder = len(self.frame_buffer) % bytes_per_frame
        
        # 온전한 프레임을 처리할 수 있는 양만 처리
        usable_length = len(self.frame_buffer) - remainder
        
        if usable_length <= 0:
            logger.debug(f"처리 가능한 프레임 없음. 버퍼 크기: {len(self.frame_buffer)}")
            return False
            
        # 완전한 프레임만 처리
        usable_buffer = self.frame_buffer[:usable_length]
        self.frame_buffer = self.frame_buffer[usable_length:]
        
        # 바이트 데이터를 numpy 배열로 변환
        try:
            audio_frame = np.frombuffer(usable_buffer, dtype=np.int16)
            self.total_frames += len(audio_frame)
            
            # 정규화
            audio_float = audio_frame.astype(np.float32) / 32767.0
            
            # 에너지 계산 (음성 감지용)
            energy = np.mean(np.abs(audio_float))
            max_amplitude = np.max(np.abs(audio_float))
            
            # 디버깅 - 주기적으로 오디오 품질 정보 로깅
            if self.total_frames % 8000 == 0:  # 약 0.5초마다
                logger.debug(f"오디오 처리 중: 총 프레임 {self.total_frames}, 최근 최대진폭: {max_amplitude:.4f}, 에너지: {energy:.4f}")
            
            # 음성 감지 로직
            if not self.is_speaking and max_amplitude > ENERGY_THRESHOLD:
                self.is_speaking = True
                self.silence_counter = 0
                logger.info(f"음성 감지됨. 오디오 캡처 중... (진폭: {max_amplitude:.4f})")
            
            # 현재 말하고 있는 상태면 버퍼에 추가
            self.audio_buffer.append(audio_float)
            
            # 무음 감지
            if self.is_speaking:
                if max_amplitude <= ENERGY_THRESHOLD:
                    self.silence_counter += 1
                    if self.silence_counter % 10 == 0:
                        logger.debug(f"무음 카운터: {self.silence_counter}/{SILENCE_THRESHOLD}")
                else:
                    self.silence_counter = 0
                
                # 충분한 무음이 감지되면 음성 종료로 판단
                if self.silence_counter >= SILENCE_THRESHOLD:
                    logger.info(f"무음 감지. 음성 종료됨. 총 처리된 프레임: {self.total_frames}, 총 수신 바이트: {self.received_bytes}")
                    return True
        
        except Exception as e:
            logger.error(f"오디오 프레임 처리 오류: {e}")
            import traceback
            logger.error(traceback.format_exc())
        
        return False
    
    def get_audio_data(self) -> np.ndarray:
        """모든 오디오 버퍼를 하나의 배열로 연결"""
        if not self.audio_buffer:
            return np.array([], dtype=np.float32)
        
        audio_data = np.concatenate(self.audio_buffer)
        logger.info(f"최종 오디오 데이터 길이: {len(audio_data)} 샘플, 길이: {len(audio_data)/self.sample_rate:.2f}초")
        
        # 최소한의 노이즈 제거 및 음질 개선
        if len(audio_data) > 0:
            # 매우 낮은 임계값의 노이즈 게이팅 
            noise_gate_threshold = 0.005
            audio_data = np.where(
                np.abs(audio_data) < noise_gate_threshold, 
                0, 
                audio_data
            )
            
            # 정규화 (볼륨 조정)
            max_value = np.max(np.abs(audio_data))
            if max_value > 0:
                audio_data = audio_data / max_value * 0.9
        
        return audio_data
    
    def get_duration(self) -> float:
        """녹음된 오디오의 길이(초)"""
        return self.total_frames / self.sample_rate
    
async def transcribe_audio(audio_data: np.ndarray, metadata: Dict[str, Any]) -> str:
    """
    오디오 데이터를 텍스트로 변환 (Google Cloud Speech-to-Text 사용)
    
    Args:
        audio_data: 처리할 오디오 데이터
        metadata: 메타데이터 (키워드 등)
        
    Returns:
        str: 인식된 텍스트
    """
    global speech_client
    
    if speech_client is None:
        logger.error("Google Cloud Speech 클라이언트가 초기화되지 않았습니다.")
        return ""
    
    if len(audio_data) == 0:
        logger.warning("빈 오디오 데이터. 처리를 건너뜁니다.")
        return ""
    
    audio_duration = len(audio_data) / 16000  # 16kHz 기준
    if audio_duration < MIN_AUDIO_LENGTH:
        logger.warning(f"오디오가 너무 짧습니다: {audio_duration:.2f}초")
        return ""
    
    logger.info(f"처리할 오디오 길이: {audio_duration:.2f}초")
    
    start_time = time.time()
    
    detected_keyword = metadata.get("keyword", "")
    
    try:
        # 오디오 데이터를 int16 형식으로 변환 (볼륨 정규화 적용)
        audio_int16 = (audio_data * 32767).astype(np.int16)
        
        # 바이트 배열로 변환
        audio_bytes = audio_int16.tobytes()
        
        # Google Cloud Speech 요청 생성
        audio = speech.RecognitionAudio(content=audio_bytes)
        
        # 음성 인식 설정 - 오류 수정: model_variant 필드 제거
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code=LANGUAGE,
            model="command_and_search",  # 짧은 명령어에 최적화된 모델
            enable_automatic_punctuation=True,
            use_enhanced=True,  # 향상된 모델 사용
            audio_channel_count=1,  # 명시적으로 모노 채널 지정
            # 추가: 말하는 환경 최적화
            speech_contexts=[
                speech.SpeechContext(
                    phrases=[detected_keyword] if detected_keyword else [],
                    boost=10.0  # 가중치 추가
                )
            ],
            # 추가: 명령어 인식에 최적화된 설정
            profanity_filter=False,
            enable_word_time_offsets=False,
            enable_word_confidence=True
            # 오류 수정: model_variant 필드 제거
        )
        
        # 음성 인식 요청
        response = speech_client.recognize(config=config, audio=audio)
        
        # 결과 처리
        full_text = ""
        confidence = 0.0
        result_count = 0
        
        for result in response.results:
            alternative = result.alternatives[0]
            full_text += alternative.transcript
            confidence += alternative.confidence
            result_count += 1
        
        # 평균 신뢰도 계산
        avg_confidence = confidence / result_count if result_count > 0 else 0
        
        process_time = time.time() - start_time
        
        logger.info(f"인식된 텍스트: {full_text} (신뢰도: {avg_confidence:.2f})")
        logger.info(f"처리 시간: {process_time:.2f}초")
        
        return full_text
        
    except Exception as e:
        logger.error(f"음성 인식 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return ""

async def text_to_json(text: str) -> str:
    """
    인식된 텍스트를 JSON 형식으로 변환
    
    Args:
        text: 변환할 텍스트
        
    Returns:
        str: JSON 형식 문자열
    """
    if not text:
        logger.warning("변환할 텍스트가 없습니다.")
        return json.dumps({
            "type": "none",
            "contents": {
                "default": "OFF",
                "data": ""
            }
        })
    
    try:
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
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"다음 텍스트를 분석하여 JSON으로 변환해주세요: {text}"}
            ],
            response_format={"type": "json_object"},
            temperature=0.2
        )
        
        result = response.choices[0].message.content.strip()
        json_result = json.loads(result)
        if json_result["type"] == "none":
            json_result["result"] = "-1"
        else:
            json_result["result"] = "네 알겠습니다"
        result = json.dumps(json_result)
        process_time = time.time() - start_time
        
        # 결과 검증
        json_result = json.loads(result)
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
            "result": "-1"
        })

@app.websocket("/listen")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("웹소켓 클라이언트 연결됨")
    
    processor = AudioProcessor()
    streaming = True
    
    # 웹소켓 수신 버퍼 최적화 설정
    # 참고: FastAPI/Starlette에서 지원하는 경우 아래 코드 적용
    # websocket.client.transport.get_extra_info('socket').setsockopt(
    #     socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    
    pending_bytes = bytearray()
    
    try:
        while streaming:
            try:
                # 텍스트 메시지 처리
                data = await websocket.receive_text()
                
                if data == "STREAMING_END":
                    logger.info("클라이언트에서 스트리밍 종료 신호 수신")
                    streaming = False
                elif data.startswith("METADATA:"):
                    # 메타데이터 처리
                    metadata_str = data[len("METADATA:"):]
                    metadata_pairs = metadata_str.split(',')
                    
                    for pair in metadata_pairs:
                        if '=' in pair:
                            key, value = pair.split('=', 1)
                            processor.metadata[key.strip()] = value.strip()
                    
                    logger.info(f"메타데이터 수신: {processor.metadata}")
                    
                    # 샘플레이트 설정
                    if 'sample_rate' in processor.metadata:
                        processor.sample_rate = int(processor.metadata['sample_rate'])
                    
                    # 채널 설정
                    if 'channels' in processor.metadata:
                        processor.channels = int(processor.metadata['channels'])
                        
                    # 인코딩 형식 설정
                    if 'encoding' in processor.metadata:
                        processor.encoding = processor.metadata['encoding']
                
            except WebSocketDisconnect:
                logger.info("클라이언트 연결 종료")
                streaming = False
                break
            except Exception as e:
                try:
                    # 바이너리 데이터 수신
                    binary_data = await websocket.receive_bytes()
                    
                    # 바이너리 오디오 데이터 처리
                    should_end = processor.process_frame(binary_data)
                    
                    # 무음 감지시 스트리밍 종료
                    if should_end:
                        logger.info("무음 감지로 스트리밍 종료")
                        await websocket.send_text("STREAMING_END")
                        streaming = False
                except Exception as inner_e:
                    logger.error(f"메시지 처리 오류: {inner_e}")
                    streaming = False
                    break
        
        # 오디오 처리 및 STT 수행
        audio_data = processor.get_audio_data()
        duration = processor.get_duration()
        
        logger.info(f"총 수신된 오디오 길이: {duration:.2f}초")
        
        if duration >= MIN_AUDIO_LENGTH:
            # STT 처리
            transcription = await transcribe_audio(audio_data, processor.metadata)
            
            if transcription:
                # STT 결과를 JSON으로 변환
                json_result = await text_to_json(transcription)
                json_obj = json.loads(json_result)
                
                """ 
                이 부분에 json 데이터 처리 
                json에 결과 추가 필요
                
                """
                type = json_obj['type']
                contents = json_obj["contents"]
                if access_token:
                    if type == "news" and contents["data"]:
                        news_result = data_processor.getNews(access_token, int(contents["data"]))
                        if news_result:
                            json_obj["result"] = news_result
                        else:
                            json_obj["result"] = "-1"
                json_result = json.dumps(json_obj)
                # JSON 결과 전송
                logger.info(f"JSON 변환 결과: {json_result}")
                await websocket.send_text(json_result)
                logger.info("JSON 변환 결과 전송 완료")
            else:
                # 빈 결과 전송
                default_json = json.dumps({
                    "type": "none",
                    "contents": {
                        "default": "OFF",
                        "data": ""
                    },
                    "result" : "-1"
                })
                await websocket.send_text(default_json)
                logger.info("빈 STT 결과에 대한 기본 JSON 전송")
        else:
            logger.warning("오디오가 너무 짧아 처리하지 않습니다.")
            default_json = json.dumps({
                "type": "none",
                "contents": {
                    "default": "OFF",
                    "data": ""
                },
                "result" : "-1"
            })
            await websocket.send_text(default_json)
        
    except WebSocketDisconnect:
        logger.info("클라이언트 연결 종료")
    except Exception as e:
        logger.error(f"웹소켓 처리 중 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info("웹소켓 연결 종료")

@app.get("/")
async def root():
    return {"message": "Speech-to-Text WebSocket 서버가 실행 중입니다. WebSocket 엔드포인트에 연결하세요: /listen"}

if __name__ == "__main__":
    import uvicorn
    tokens = data_processor.login()
    if tokens:
        access_token = tokens["access_token"]
        refresh_token = tokens['refresh_token']
    else:
        access_token = None
        refresh_token = None
    logger.info(f"서버 시작: {HOST}:{PORT}")
    uvicorn.run(app, host=HOST, port=PORT)