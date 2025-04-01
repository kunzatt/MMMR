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
from iot_ws import WebSocketServer
import threading


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
app.state.access_token = None
app.state.refresh_token = None

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

important_phrases = [
    # 공통 명령어 단어
    "켜줘", "꺼줘", "켜", "꺼", "알려줘", "보여줘", "설정해줘", "추가해줘",
    
    # iot 관련
    "전등", "조명", "불", "불빛", "TV", "티비", "에어컨", "선풍기", "보일러", "가습기", "청소기",
    "전원", "스위치", "밝기", "온도", "전자레인지", "냉장고", "세탁기",
    
    # weather 관련
    "날씨", "기온", "온도", "습도", "미세먼지", "비", "눈", "우산", "맑음", "흐림", "더움", "추움",
    "오늘 날씨", "내일 날씨", "주간 날씨", "예보",
    
    # news 관련
    "뉴스", "속보", "최신 뉴스", "오늘 뉴스", "뉴스 알려줘", 
    "1번", "2번", "3번", "4번", "5번", "첫 번째", "두 번째", "세 번째",
    
    # youtube 관련
    "유튜브", "영상", "동영상", "채널", "음악", "뮤직비디오", "트레일러", "예고편", "강의", "요가",
    
    # timer 관련
    "타이머", "알람", "초", "분", "시간", "카운트다운", "스톱워치",
    "5분", "10분", "30분", "1시간", "30초", "1분",
    
    # todo 관련
    "할 일", "투두리스트", "목록", "체크리스트", "할일", "태스크", "일정",
    
    # schedule 관련
    "일정", "약속", "스케줄",
    "오늘", "내일", "다음 주", "이번 주",
    
    # time 관련
    "시간", "시계", "몇 시", "지금", "현재 시간", "현재",
    
    # transportation 관련
    "버스", "지하철", "운행", "도착", "출발", "역", "정류장",
    "언제", "몇 분", "지연", "배차",

    # 이동형 홈 카메라 관련
    "홈 카메라", "홈 캠", "이동", "주방", "거실", "입구", "방1", "방2", "방3", "방4",
    "이동해줘", "이동해", "이동시켜줘", "이동시켜", "움직여줘", "움직여"
]

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
                    phrases=important_phrases,
                    boost=20.0  # 가중치 추가
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
- schedule: 일정 관련 요청 (예: "오늘 일정 알려줘", "내일 일정 알려줘", "이번 주 일정 알려줘")
- time: 시간 관련 요청 (예: "지금 몇 시야?", "시계 보여줘")
- transportation: 교통 정보 요청 (예: "버스 언제 와?", "지하철 운행 정보")
- homecam : 이동형 홈 카메라 제어 요청 (예: "홈 카메라 켜줘", "홈 카메라 꺼줘", "홈 캠 켜줘", "홈 캠 꺼줘", "홈 캠 주방으로 이동해줘", "홈 캠 거실로 이동해줘")
- none: 위 분류에 해당하지 않는 경우

contents.default는 기능을 켜는 명령의 경우 "ON", 끄는 명령인 경우 "OFF"로 설정합니다.

contents.data는 type에 따라 다르게 설정합니다:
- iot: "light ON" 또는 "light OFF"와 같은 형태
- news: "1"부터 "5" 사이의 숫자 (뉴스 번호) 혹은 빈 문자열
- timer: "00H05M00S"와 같은 형태 (시간, 분, 초)
- scehdule: "today", "tomorrow", "this_week", "next_week"
- youtube : "남자 요가 영상"와 같은 검색어
- homecam :  "living_room", "kitchen", "entrance", "room1", "room2", "room3", "room4"(이동 관련) 혹은은 빈 문자열(카메라 제어 관련)
- 다른 유형: 빈 문자열
contents.data는 필수 항목이 아니며, 필요하지 않은 경우 빈 문자열로 설정합니다. 또한, 정의된 내용으로만 설정해야 합니다.
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
    
async def process_and_send_json_result(websocket: WebSocket, transcription: str = None, keyword: str = "미미"):
    if transcription:
        # STT 결과를 JSON으로 변환
        json_result = await text_to_json(transcription)
        json_obj = json.loads(json_result)
        
        # 추가 처리 (뉴스 등)
        type = json_obj['type']
        contents = json_obj["contents"]
        
        # 토큰이 있는 경우 추가 처리
        if app.state.access_token:
            if type == "news" and contents["data"]:
                news_result, new_tokens = data_processor.getNews(
                    app.state.access_token, 
                    app.state.refresh_token, 
                    int(contents["data"])
                )
                if news_result:
                    json_obj["result"] = news_result
                else:
                    json_obj["result"] = "-1"
                    
                if new_tokens:
                    app.state.access_token = new_tokens["access_token"]
                    app.state.refresh_token = new_tokens["refresh_token"]
            elif type == "weather":
                weather_result, new_tokens = data_processor.getWeather(
                    app.state.access_token, 
                    app.state.refresh_token
                )
                if weather_result:
                    json_obj["result"] = weather_result
                else:
                    json_obj["result"] = "-1"
                    
                if new_tokens:
                    app.state.access_token = new_tokens["access_token"]
                    app.state.refresh_token = new_tokens["refresh_token"]
            elif type == "homecam":
                if contents["data"]:
                    logger.info(f"홈 카메라 이동 요청: {contents['data']}")
                    await iot_ws.send_to_navigation(json_obj)
            elif type == "schedule":
                schedule_result, new_tokens = data_processor.getSchedules(
                    keyword,
                    app.state.access_token, 
                    app.state.refresh_token,
                    contents["data"]
                )
                if schedule_result:
                    json_obj["result"] = schedule_result
                else:
                    json_obj["result"] = "-1"
                    
                if new_tokens:
                    app.state.access_token = new_tokens["access_token"]
                    app.state.refresh_token = new_tokens["refresh_token"]
        
        json_result = json.dumps(json_obj)
        logger.info(f"JSON 변환 결과: {json_result}")
    else:
        # 빈 결과이거나 오디오가 너무 짧은 경우 기본 JSON 전송
        json_result = json.dumps({
            "type": "none",
            "contents": {
                "default": "OFF",
                "data": ""
            },
            "result": "-1"
        })
        reason = "빈 STT 결과" if transcription is None else "오디오가 너무 짧음"
        logger.info(f"{reason}에 대한 기본 JSON 전송")
    
    # 결과 전송
    await websocket.send_text(json_result)
    logger.info("JSON 결과 전송 완료")

@app.websocket("/listen")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("웹소켓 클라이언트 연결됨")
    
    processor = AudioProcessor()
    streaming = True
        
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
            await process_and_send_json_result(websocket, transcription, processor.metadata["keyword"])
           
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
    iot_ws = WebSocketServer()
    iot_ws_thread = threading.Thread(target=iot_ws.start, daemon=True)
    iot_ws_thread.start()
    if tokens:
        app.state.access_token = tokens["access_token"]
        app.state.refresh_token = tokens['refresh_token']
    else:
        access_token = None
        refresh_token = None
    logger.info(f"서버 시작: {HOST}:{PORT}")
    uvicorn.run(app, host=HOST, port=PORT)