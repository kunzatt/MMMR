import pvporcupine
import pyaudio
import numpy as np
import time
import os
import signal
import asyncio
import websockets
import json
import logging
from dotenv import load_dotenv
import subprocess
from google.cloud import texttospeech
from google.cloud import speech

load_dotenv()

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 환경 변수 설정
ACCESS_KEY = os.getenv("ACCESS_KEY")
KEYWORD_PATHS = [
    os.getenv("MIMI_KEYWORD_PATH"),       #미미
    os.getenv("HAETAE_KEYWORD_PATH")      #해태
]
MODEL_PATH = os.getenv("MODEL_PATH")
ALERT_SOUND_PATH = os.getenv("ALERT_SOUND_PATH", "alert.mp3")
KEYWORD_NAMES = ["미미", "해태"]
LANGUAGE = os.getenv("LANGUAGE", "ko-KR")

# WebSocket 서버 설정 - 서버 IP와 포트를 정확히 설정해주세요
SERVER_URL = os.getenv("SERVER_URL", "ws://70.12.246.31:12345/ws")
CONNECTION_TIMEOUT = int(os.getenv("CONNECTION_TIMEOUT", "10"))  # 웹소켓 연결 타임아웃(초)

# 웹 클라이언트용 웹소켓 서버 설정
WS_HOST = os.getenv("WS_HOST", "0.0.0.0")
WS_PORT = int(os.getenv("WS_PORT", "8765"))

# TTS 파일 경로
TTS_DIR = "./tts_files"
MIMI_TTS_FILE = "not_understand_f.wav"
HAETAE_TTS_FILE = "not_understand_m.wav"

# 민감도 설정
SENSITIVITIES = [0.7, 0.7]

# 음성 감지 설정
ENERGY_THRESHOLD = float(os.getenv("ENERGY_THRESHOLD", "0.02"))
SILENCE_THRESHOLD = int(os.getenv("SILENCE_THRESHOLD", "25"))
MIN_AUDIO_LENGTH = float(os.getenv("MIN_AUDIO_LENGTH", "0.5"))

# 프로그램 실행 중 상태
running = True
websocket_connection = None

# 연결된 웹 클라이언트들을 저장할 세트
connected_clients = set()

# Ctrl+C 핸들러
def signal_handler(sig, frame):
    global running
    logger.info("\n프로그램 종료 중...")
    running = False
    # 비상 종료 시스템 (5초 후 강제 종료)
    asyncio.create_task(force_shutdown())

async def force_shutdown():
    await asyncio.sleep(5)
    logger.info("프로그램이 자동으로 종료됩니다.")
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)

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
    "일정", "약속", "회의", "미팅", "스케줄", "캘린더", "알림",
    "오늘", "내일", "모레", "다음 주", "이번 주", "월요일", "화요일", "수요일", "목요일", "금요일",
    
    # time 관련
    "시간", "시계", "몇 시", "지금", "현재 시간", "현재",
    
    # transportation 관련
    "버스", "지하철", "운행", "도착", "출발", "역", "정류장",
    "언제", "몇 분", "지연", "배차"
]

def init_speech_client():
    global speech_client
    try:
        speech_client = speech.SpeechClient()
        logger.info("Google Cloud Speech 클라이언트 초기화 완료")
        return True
    except Exception as e:
        logger.error(f"Google Cloud Speech 클라이언트 초기화 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return False

# 웹 클라이언트 관련 함수
async def register(websocket):
    logger.info(f"웹 클라이언트 연결됨")
    connected_clients.add(websocket)

async def unregister(websocket):
    logger.info(f"웹 클라이언트 연결 해제")
    connected_clients.remove(websocket)

async def broadcast_message(message):
    if connected_clients:
        # JSON 문자열로 변환
        if isinstance(message, dict):
            message_str = json.dumps(message)
        else:
            message_str = message
            
        # 연결된 모든 클라이언트에게 전송
        await asyncio.gather(
            *[client.send(message_str) for client in connected_clients],
            return_exceptions=True
        )
        logger.info(f"웹 클라이언트에 메시지 전달됨: {message_str} (클라이언트 {len(connected_clients)}개)")
    else:
        logger.info("연결된 웹 클라이언트 없음, 웹 메시지 전달 불가")

async def handle_web_client(websocket):
    await register(websocket)
    try:
        async for message in websocket:
            try:
                # JSON 파싱
                data = json.loads(message)
                logger.info(f"웹 클라이언트로부터 메시지 수신: {data}")

                # 클라이언트로부터의 메시지 처리
                if isinstance(data, dict):
                    if data.get('command') == 'ping':
                        await websocket.send(json.dumps({'command': 'pong'}))
                    elif data.get('command') == 'get_status':
                        await websocket.send(json.dumps({'command': 'status', 'status': 'ok'}))
                    # 기타 명령은 기본적으로 무시

            except json.JSONDecodeError:
                logger.error(f"JSON 디코딩 오류: {message}")
            except Exception as e:
                logger.error(f"웹 클라이언트 메시지 처리 중 오류: {e}")
    except websockets.exceptions.ConnectionClosed:
        logger.info("웹 클라이언트 연결이 종료됨")
    except Exception as e:
        logger.error(f"웹소켓 핸들러 오류: {e}")
    finally:
        await unregister(websocket)

async def start_websocket_server():
    server = await websockets.serve(handle_web_client, WS_HOST, WS_PORT)
    logger.info(f"웹소켓 서버가 시작되었습니다: ws://{WS_HOST}:{WS_PORT}")
    return server

def play_alert_sound():
    if os.path.exists(ALERT_SOUND_PATH):
        try:
            # MP3 파일 확장자 확인
            if ALERT_SOUND_PATH.endswith('.mp3'):
                subprocess.call(["mpg123", "-q", ALERT_SOUND_PATH])
                logger.info("알림음 출력 성공")
            else:
                subprocess.call(["aplay", ALERT_SOUND_PATH])
        except Exception as e:
            logger.error(f"알림음 재생 중 오류 발생: {e}")
    else:
        logger.error(f"알림음 파일을 찾을 수 없습니다: {ALERT_SOUND_PATH}")

def play_tts_file(keyword):
    try:
        tts_file = MIMI_TTS_FILE if keyword == "미미" else HAETAE_TTS_FILE
        tts_path = os.path.join(TTS_DIR, tts_file)

        if os.path.exists(tts_path):
            logger.info(f"TTS 파일 재생 중: {tts_path}")
            subprocess.call(["aplay", tts_path])
            logger.info("TTS 파일 재생 완료")
        else:
            logger.error(f"TTS 파일을 찾을 수 없습니다: {tts_path}")
    except Exception as e:
        logger.error(f"TTS 파일 재생 중 오류 발생: {e}")

def speak_text(text, gender="female"):
    try:
        client = texttospeech.TextToSpeechClient()

        synthesis_input = texttospeech.SynthesisInput(text=text)

        # 성별에 따른 음성 설정
        if gender.lower() == "male":
            voice_name = "ko-KR-Standard-C"  # 남성 음성
            ssml_gender = texttospeech.SsmlVoiceGender.MALE
        else:
            voice_name = "ko-KR-Standard-A"  # 여성 음성
            ssml_gender = texttospeech.SsmlVoiceGender.FEMALE

        # 음성 설정
        voice = texttospeech.VoiceSelectionParams(
            language_code="ko-KR",
            name=voice_name,
            ssml_gender=ssml_gender
        )

        # 오디오 출력 설정
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            speaking_rate=1.1
        )

        # TTS 요청 실행
        logger.info(f"텍스트 '{text}'를 음성으로 변환 중...")
        response = client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        # 임시 파일로 저장
        temp_file = os.path.join(TTS_DIR, "temp_tts.wav")
        with open(temp_file, "wb") as out:
            out.write(response.audio_content)

        # 음성 재생
        subprocess.call(["aplay", temp_file])

        # 임시 파일 삭제
        os.remove(temp_file)

        logger.info("TTS 재생 완료")
    except Exception as e:
        logger.error(f"TTS 변환 또는 재생 중 오류 발생: {e}")

class AudioProcessor:
    """오디오 처리 클래스"""
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.total_frames = 0
        self.detected_keyword = ""
    
    def reset(self):
        """오디오 처리 상태 초기화"""
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.total_frames = 0
    
    def process_frame(self, frame_data):
        try:
            self.total_frames += len(frame_data)
            
            # float32로 변환 및 정규화
            audio_float = frame_data.astype(np.float32) / 32767.0
            
            # 에너지 계산 (음성 감지용)
            energy = np.mean(np.abs(audio_float))
            max_amplitude = np.max(np.abs(audio_float))
            
            # 음성 감지 로직
            if not self.is_speaking and max_amplitude > ENERGY_THRESHOLD:
                self.is_speaking = True
                self.silence_counter = 0
                logger.info(f"음성 감지됨. 오디오 캡처 중... (진폭: {max_amplitude:.4f})")
            
            # 현재 말하고 있는 상태면 버퍼에 추가
            if self.is_speaking:
                self.audio_buffer.append(audio_float)
                
                # 무음 감지
                if max_amplitude <= ENERGY_THRESHOLD:
                    self.silence_counter += 1
                else:
                    self.silence_counter = 0
                
                # 충분한 무음이 감지되면 음성 종료로 판단
                if self.silence_counter >= SILENCE_THRESHOLD:
                    logger.info(f"무음 감지. 음성 종료됨. 총 프레임: {self.total_frames}")
                    return True
            
            return False
            
        except Exception as e:
            logger.error(f"오디오 프레임 처리 오류: {e}")
            return False
    
    def get_audio_data(self):
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
    
    def get_duration(self):
        """녹음된 오디오의 길이(초)"""
        return self.total_frames / self.sample_rate
    


async def transcribe_audio(audio_data, detected_keyword):
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
    
    try:
        # 오디오 데이터를 int16 형식으로 변환
        audio_int16 = (audio_data * 32767).astype(np.int16)
        
        # 바이트 배열로 변환
        audio_bytes = audio_int16.tobytes()
        
        # Google Cloud Speech 요청 생성
        audio = speech.RecognitionAudio(content=audio_bytes)
        
        # 음성 인식 설정
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
                    boost=15.0  # 가중치 추가
                )
            ],
            # 추가: 명령어 인식에 최적화된 설정
            profanity_filter=False,
            enable_word_time_offsets=False,
            enable_word_confidence=True
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

async def process_command_and_play_response(detected_keyword, json_result):
    try:
        # JSON 문자열을 딕셔너리로 변환
        if isinstance(json_result, str):
            data = json.loads(json_result)
        else:
            data = json_result
        
        # 결과 처리
        if "result" in data:
            result_value = data.get("result")
            
            if result_value not in ["-1", "0"]:
                # 유효한 결과가 있으면 TTS로 읽어주기
                logger.info("서버 응답을 TTS로 읽어줍니다.")
                # 호출어에 따라 다른 음성으로 TTS 실행
                gender = "female" if detected_keyword == "미미" else "male"
                speak_text(result_value, gender)
            else:
                # 결과가 없는 경우 호출어에 맞는 기본 안내 음성 재생
                logger.info(f"유효한 결과가 없습니다. '{detected_keyword}'에 해당하는 TTS 파일을 재생합니다.")
                play_tts_file(detected_keyword)
        else:
            # result 필드가 없는 경우
            logger.warning("서버 응답에 result 필드가 없습니다.")
            play_tts_file(detected_keyword)
    except Exception as e:
        logger.error(f"응답 처리 오류: {e}")
        play_tts_file(detected_keyword)

def is_websocket_closed(websocket):
    try:
        # 다양한 웹소켓 라이브러리에 대한 처리
        # 1. websockets 라이브러리
        if hasattr(websocket, 'closed'):
            return websocket.closed
        
        # 2. 소켓 객체가 None인 경우
        if hasattr(websocket, 'socket') and websocket.socket is None:
            return True
            
        # 3. 연결 상태 속성이 있는 경우
        if hasattr(websocket, 'client') and websocket.client is None:
            return True
        
        # 위 조건에 해당하지 않으면 연결 중으로 간주
        return False
        
    except Exception:
        # 어떤 오류가 발생하면 연결이 끊긴 것으로 간주
        return True

async def ensure_websocket_connection():
    """웹소켓 연결 확인 및 필요시 재연결"""
    global websocket_connection
    
    if websocket_connection is None or is_websocket_closed(websocket_connection):
        # 연결 재시도
        try:
            websocket_connection = await asyncio.wait_for(
                websockets.connect(SERVER_URL), 
                timeout=CONNECTION_TIMEOUT
            )
            
            # 재연결 후 클라이언트 등록
            register_msg = {
                "type": "register",
                "client_type": "mirror",
                "info": "Raspberry Pi Voice Assistant"
            }
            await websocket_connection.send(json.dumps(register_msg))
            
            # 등록 응답 수신
            response = await websocket_connection.recv()
            logger.info(f"서버 재연결 및 등록 성공: {response}")
            
            return True
        except Exception as e:
            logger.error(f"웹소켓 재연결 실패: {e}")
            websocket_connection = None
            return False
    
    return True

async def handle_voice_command(websocket, detected_keyword, audio_data):
    try:
        # 연결 상태 확인 및 재연결
        if is_websocket_closed(websocket):
            logger.warning("웹소켓 연결이 끊어졌습니다. 재연결 시도...")
            if not await ensure_websocket_connection():
                logger.error("재연결 실패. 명령을 처리할 수 없습니다.")
                play_tts_file(detected_keyword)
                return
            # 재연결 성공 시 새 웹소켓 객체 사용
            websocket = websocket_connection
            
        # STT 처리
        transcription = await transcribe_audio(audio_data, detected_keyword)
        
        if not transcription:
            # 인식 실패 시 기본 응답 재생
            logger.warning("음성 인식 실패")
            play_tts_file(detected_keyword)
            return
        
        # 텍스트를 서버로 전송하기 위한 메시지 구성
        message_id = str(int(time.time()))
        message = {
            "id": message_id,
            "text": transcription, 
            "keyword": detected_keyword
        }
        
        # 서버로 전송
        await websocket.send(json.dumps(message))
        logger.info(f"서버로 전송된 텍스트: {transcription}")
        
        # 서버 응답 대기 (타임아웃 5초)
        try:
            response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            logger.info(f"서버 응답: {response}")
            
            # 응답 데이터 파싱
            json_result = json.loads(response)
            
            # 웹 클라이언트에 명령 결과 전송
            await broadcast_message(json_result)
            logger.info("웹 클라이언트에 결과 전달됨")
            
            # 응답 처리 및 TTS 출력
            await process_command_and_play_response(detected_keyword, response)
            
        except asyncio.TimeoutError:
            logger.error("서버 응답 대기 시간 초과")
            play_tts_file(detected_keyword)
        
    except Exception as e:
        logger.error(f"음성 명령 처리 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
        play_tts_file(detected_keyword)

async def keepalive_task():
    global websocket_connection
    
    while running:
        if websocket_connection and not is_websocket_closed(websocket_connection):
            try:
                ping_msg = {"type": "ping", "timestamp": time.time()}
                await websocket_connection.send(json.dumps(ping_msg))
                logger.debug("Ping 메시지 전송")
            except Exception as e:
                logger.error(f"Ping 전송 실패: {e}")
        
        await asyncio.sleep(25)  # 25초마다 ping 전송

async def connect_websocket():
    global websocket_connection, running
    
    # 연결 재시도 간격 (초)
    retry_interval = 5
    max_retries = 3
    
    while running:
        retries = 0
        
        while running and retries < max_retries:
            try:
                logger.info(f"서버 연결 시도: {SERVER_URL}")
                # 타임아웃 설정하여 연결
                websocket = await asyncio.wait_for(
                    websockets.connect(SERVER_URL), 
                    timeout=CONNECTION_TIMEOUT
                )
                
                websocket_connection = websocket
                logger.info("서버 연결 성공!")
                
                # 클라이언트 등록
                register_msg = {
                    "type": "register",
                    "client_type": "mirror",
                    "info": "Raspberry Pi Voice Assistant"
                }
                await websocket.send(json.dumps(register_msg))
                
                # 등록 응답 수신
                response = await websocket.recv()
                logger.info(f"서버 응답: {response}")
                
                
                # Wake word 감지 및 음성 명령 처리
                await run_wake_word_detection(websocket)
                
                # 연결이 종료되면 재연결 시도
                websocket_connection = None
                break
                
            except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosedError, 
                   websockets.exceptions.InvalidStatusCode, OSError) as e:
                retries += 1
                websocket_connection = None
                if isinstance(e, asyncio.TimeoutError):
                    logger.error(f"서버 연결 타임아웃 ({retries}/{max_retries})")
                else:
                    logger.error(f"웹소켓 연결 오류: {e} ({retries}/{max_retries})")
                
                if retries < max_retries and running:
                    # 지수 백오프 (재시도 간격을 점점 늘림)
                    wait_time = retry_interval * (2 ** (retries - 1))
                    logger.info(f"{wait_time}초 후 재연결 시도...")
                    await asyncio.sleep(wait_time)
                else:
                    if running:
                        logger.error("최대 재시도 횟수 초과, 30초 후 다시 시도합니다.")
                        await asyncio.sleep(30)  # 일정 시간 후 재시도 사이클 재시작
                    break
                    
            except Exception as e:
                logger.error(f"예상치 못한 오류: {e}")
                import traceback
                logger.error(traceback.format_exc())
                websocket_connection = None
                if running:
                    logger.info("15초 후 재연결 시도...")
                    await asyncio.sleep(15)
                break

async def run_wake_word_detection(websocket):
    """Wake word 감지 및 오디오 처리"""
    global running
    porcupine = None
    audio = None
    stream = None
    processor = AudioProcessor()

    try:
        # Porcupine 초기화 - 여러 키워드 설정
        porcupine = pvporcupine.create(
            access_key=ACCESS_KEY,
            keyword_paths=KEYWORD_PATHS,
            model_path=MODEL_PATH,
            sensitivities=SENSITIVITIES
        )

        # 오디오 설정
        sample_rate = porcupine.sample_rate
        frame_length = porcupine.frame_length

        audio = pyaudio.PyAudio()

        # 오디오 스트림 생성
        stream = audio.open(
            rate=sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=frame_length
        )

        logger.info("Wake word 감지 시작... ('Ctrl+C'로 종료)")
        logger.info(f"'{KEYWORD_NAMES[0]}' 또는 '{KEYWORD_NAMES[1]}'라고 말해보세요...")

        # TTS 디렉토리 확인 및 생성
        if not os.path.exists(TTS_DIR):
            logger.info(f"TTS 디렉토리가 없습니다. 생성합니다: {TTS_DIR}")
            os.makedirs(TTS_DIR)

        # 웹소켓 연결 상태 확인 루프
        check_interval = 0  # 1초마다 확인 (0은 매 프레임 확인)
        last_check_time = time.time()

        while running:
            try:
                # 오디오 프레임 읽기
                pcm = stream.read(frame_length, exception_on_overflow=False)
                pcm_np = np.frombuffer(pcm, dtype=np.int16)

                # 주기적으로 웹소켓 연결 상태 확인
                current_time = time.time()
                if check_interval > 0 and current_time - last_check_time > check_interval:
                    if is_websocket_closed(websocket):
                        logger.warning("웹소켓 연결이 끊어졌습니다. Wake word 감지를 중단합니다.")
                        break
                    last_check_time = current_time

                # Wake word 감지 처리
                keyword_index = porcupine.process(pcm_np)

                # Wake word가 감지되면
                if keyword_index >= 0:
                    detected_keyword = KEYWORD_NAMES[keyword_index]
                    logger.info(f"'{detected_keyword}' 감지됨! 명령을 말씀해주세요...")

                    # 알림음 재생
                    play_alert_sound()
                    
                    # 음성 명령 녹음 및 처리
                    processor.reset()
                    processor.detected_keyword = detected_keyword
                    
                    # 명령 녹음
                    speaking_ended = False
                    while not speaking_ended and running:
                        pcm = stream.read(frame_length, exception_on_overflow=False)
                        pcm_np = np.frombuffer(pcm, dtype=np.int16)
                        speaking_ended = processor.process_frame(pcm_np)
                    
                    # 녹음된 오디오 처리
                    audio_data = processor.get_audio_data()
                    
                    # 웹소켓 연결 상태 확인
                    if is_websocket_closed(websocket):
                        logger.warning("웹소켓 연결이 끊어졌습니다. 명령을 처리할 수 없습니다.")
                        play_tts_file(detected_keyword)
                        break
                        
                    if len(audio_data) > 0:
                        # 음성 명령 처리 및 서버 통신
                        await handle_voice_command(websocket, detected_keyword, audio_data)
                    else:
                        logger.warning("녹음된 오디오가 없습니다.")
                        play_tts_file(detected_keyword)
                    
                    logger.info("명령 처리 완료. 다시 대기 중...")

                # 스트리밍 처리 최적화 (CPU 사용량 감소)
                await asyncio.sleep(0.01)
                
            except websockets.exceptions.ConnectionClosedError:
                logger.error("웹소켓 연결이 끊어졌습니다.")
                break
            except Exception as e:
                logger.error(f"오디오 프레임 처리 중 오류: {e}")
                await asyncio.sleep(0.1)  # 오류 발생 시 잠시 대기

    except Exception as e:
        logger.error(f"Wake word 감지 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info("리소스 정리 중...")
        # 리소스 정리
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if audio is not None:
            audio.terminate()
        if porcupine is not None:
            porcupine.delete()

async def main():
    """메인 함수"""
    try:
        # 환경 변수 확인
        check_env_vars()
        
        # Google Cloud Speech 클라이언트 초기화
        if not init_speech_client():
            logger.error("Google Cloud Speech 클라이언트 초기화 실패. 프로그램을 종료합니다.")
            return
        

        # 웹소켓 서버 시작
        web_server = await start_websocket_server()
        logger.info("웹 클라이언트용 웹소켓 서버 시작됨")

        await broadcast_message({
            "type": "system",
            "contents": {
                "default": "ON",
                "data": "음성 비서 시스템이 실행되었습니다."
            }
        })

        # 서버 연결 태스크 생성
        connect_task = asyncio.create_task(connect_websocket())
        
        # 종료 신호 대기
        while running:
            await asyncio.sleep(0.1)
            
        # 태스크 정리
        logger.info("태스크 정리 중...")
        connect_task.cancel()
        keepalive_task.cancel()
        await web_server.wait_closed()

        try:
            await connect_task
        except asyncio.CancelledError:
            logger.info("연결 태스크가 정상적으로 취소되었습니다.")
            
    except KeyboardInterrupt:
        logger.info("키보드 인터럽트로 종료합니다.")
    except Exception as e:
        logger.error(f"실행 중 오류 발생: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info("프로그램을 종료합니다.")

def check_env_vars():
    """필수 환경 변수 확인"""
    missing_vars = []
    if not ACCESS_KEY:
        missing_vars.append("ACCESS_KEY")
    if not KEYWORD_PATHS[0]:
        missing_vars.append("MIMI_KEYWORD_PATH")
    if not KEYWORD_PATHS[1]:
        missing_vars.append("HAETAE_KEYWORD_PATH")
    if not MODEL_PATH:
        missing_vars.append("MODEL_PATH")
    if not os.environ.get('GOOGLE_APPLICATION_CREDENTIALS'):
        missing_vars.append("GOOGLE_APPLICATION_CREDENTIALS")

    if missing_vars:
        logger.error(f"오류: .env 파일에 필요한 환경변수가 설정되지 않았습니다: {', '.join(missing_vars)}")
        exit(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        running = False
        logger.info("\n프로그램이 종료되었습니다.")