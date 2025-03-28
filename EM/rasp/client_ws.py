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
SERVER_URL = os.getenv("SERVER_URL", "ws://localhost:8000/listen")

# WebSocket 서버 설정
WS_HOST = os.getenv("WS_HOST", "0.0.0.0")
WS_PORT = int(os.getenv("WS_PORT", "8765"))

# TTS 파일 경로
TTS_DIR = "./tts_files"
MIMI_TTS_FILE = "not_understand_f.wav"
HAETAE_TTS_FILE = "not_understand_m.wav"

# 민감도 설정
SENSITIVITIES = [0.7, 0.7]

# 프로그램 실행 중 상태
running = True

# 연결된 웹 클라이언트들을 저장할 세트
connected_clients = set()

# Ctrl+C 핸들러
def signal_handler(sig, frame):
    global running
    running = False
    print("\n프로그램 종료 중...")

signal.signal(signal.SIGINT, signal_handler)

async def register(websocket):
    """웹 클라이언트 연결 등록"""
    logger.info(f"웹 클라이언트 연결됨")
    connected_clients.add(websocket)

async def unregister(websocket):
    """웹 클라이언트 연결 해제"""
    logger.info(f"웹 클라이언트 연결 해제")
    connected_clients.remove(websocket)

async def broadcast_message(message):
    """모든 연결된 웹 클라이언트에게 메시지 전달"""
    if connected_clients:
        # JSON 문자열로 변환
        message_str = json.dumps(message)

        # 연결된 모든 클라이언트에게 전송
        await asyncio.gather(
            *[client.send(message_str) for client in connected_clients]
        )
        logger.info(f"웹 클라이언트에 메시지 전달됨: {message_str} (클라이언트 {len(connected_clients)}개)")
    else:
        logger.info("연결된 웹 클라이언트 없음, 웹 메시지 전달 불가")

async def handle_web_client(websocket):
    """웹 클라이언트 웹소켓 연결 처리 핸들러"""
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

def play_alert_sound():
    if os.path.exists(ALERT_SOUND_PATH):
        try:
            # MP3 파일 확장자 확인
            if ALERT_SOUND_PATH.endswith('.mp3'):
                subprocess.call(["mpg123", "-q", ALERT_SOUND_PATH])
                print("알림음 출력 성공")
            else:
                subprocess.call(["aplay", ALERT_SOUND_PATH])
        except Exception as e:
            print(f"알림음 재생 중 오류 발생: {e}")
    else:
        print(f"알림음 파일을 찾을 수 없습니다: {ALERT_SOUND_PATH}")

def play_tts_file(keyword):
    """호출어에 따른 TTS 파일 재생"""
    try:
        tts_file = MIMI_TTS_FILE if keyword == "미미" else HAETAE_TTS_FILE
        tts_path = os.path.join(TTS_DIR, tts_file)

        if os.path.exists(tts_path):
            print(f"TTS 파일 재생 중: {tts_path}")
            subprocess.call(["aplay", tts_path])
            print("TTS 파일 재생 완료")
        else:
            print(f"TTS 파일을 찾을 수 없습니다: {tts_path}")
    except Exception as e:
        print(f"TTS 파일 재생 중 오류 발생: {e}")

def speak_text(text, gender="female"):
    """Google TTS로 텍스트를 음성으로 변환하여 출력"""
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
        print(f"텍스트 '{text}'를 음성으로 변환 중...")
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

        print("TTS 재생 완료")
    except Exception as e:
        print(f"TTS 변환 또는 재생 중 오류 발생: {e}")

async def stream_audio_to_server(audio_stream, sample_rate, frame_length, detected_keyword):
    """웹소켓을 통해 서버로 오디오 스트리밍"""
    try:
        print(f"서버 연결 시도 중: {SERVER_URL}")
        async with websockets.connect(SERVER_URL) as websocket:
            # 초기 메타데이터 전송 - 명확한 샘플레이트와 오디오 형식 정보 추가
            metadata_msg = f"METADATA:sample_rate={sample_rate},keyword={detected_keyword},encoding=PCM_16BIT,channels=1"
            await websocket.send(metadata_msg)
            print(f"메타데이터 전송: {metadata_msg}")
            
            # 오디오 품질 관련 변수 설정
            CHUNK_SIZE = frame_length  # 정확히 한 프레임씩 전송
            buffer = bytearray()
            total_bytes_sent = 0
            
            # 스트리밍 시작 시간
            start_time = time.time()
            streaming = True

            # 진행 정보 출력 타이머
            last_report_time = time.time()
            
            while streaming and running:
                try:
                    # 오디오 데이터 읽기 (정확히 한 프레임씩)
                    pcm = audio_stream.read(frame_length, exception_on_overflow=False)
                    
                    # 바로 전송 (버퍼링 하지 않음)
                    await websocket.send(pcm)
                    total_bytes_sent += len(pcm)
                    
                    # 5초마다 진행 상황 출력
                    current_time = time.time()
                    if current_time - last_report_time > 5:
                        duration = current_time - start_time
                        print(f"오디오 스트리밍 진행 중: {duration:.1f}초, 전송된 데이터: {total_bytes_sent/1024:.1f}KB")
                        last_report_time = current_time

                    # 최대 10초간 스트리밍
                    if time.time() - start_time > 10:
                        # 종료 알림 전송
                        await websocket.send("STREAMING_END")
                        streaming = False
                        print(f"스트리밍 시간 종료 (10초). 총 전송된 바이트: {total_bytes_sent}")

                    # 서버로부터 응답 체크 (비동기로 처리)
                    try:
                        response = await asyncio.wait_for(websocket.recv(), timeout=0.01)
                        if response == "STREAMING_END":
                            streaming = False
                            print("서버에서 스트리밍 종료 요청.")
                    except asyncio.TimeoutError:
                        # 타임아웃은 정상적인 상황
                        pass

                except Exception as e:
                    print(f"스트리밍 중 오류: {e}")
                    break
            
            # 스트리밍 종료 메시지 전송
            if streaming:  # 아직 종료되지 않았다면
                await websocket.send("STREAMING_END")
            
            print(f"오디오 스트리밍 완료. 총 {total_bytes_sent} 바이트 전송됨.")

            # 서버의 STT 결과 대기
            try:
                result = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"STT 결과: {result}")
                
                # 결과 JSON 파싱
                try:
                    json_result = json.loads(result)
                    
                    # 결과 처리 및 웹 클라이언트 메시지 전송
                    contents_type = json_result.get("type")
                    contents = json_result.get("contents", {})
                    
                    # 웹 클라이언트에 명령 결과 전송
                    if contents_type:
                        # 웹 클라이언트에 메시지 전달
                        try:
                            await broadcast_message(json_result)
                        except Exception as e:
                            print(f"웹 클라이언트 메시지 전송 오류: {e}")
                    
                    # 뉴스 타입 및 유효한 결과인지 확인
                    if "result" in json_result:
                        if json_result["result"] not in ["-1", "0"]:
                            # 유효한 뉴스 결과가 있으면 TTS로 읽어주기
                            print("뉴스 결과를 TTS로 읽어줍니다.")
                            # 호출어에 따라 다른 음성으로 TTS 실행
                            gender = "female" if detected_keyword == "미미" else "male"
                            speak_text(json_result["result"], gender)
                        else:
                            # 결과가 없는 경우 호출어에 맞는 기본 안내 음성 재생
                            print(f"유효한 뉴스 결과가 없습니다. '{detected_keyword}'에 해당하는 TTS 파일을 재생합니다.")
                            play_tts_file(detected_keyword)
                    elif "result" in json_result and json_result["result"] == "-1":
                        # 다른 타입의 명령에서 결과가 -1인 경우 기본 TTS 파일 재생
                        print(f"결과가 -1입니다. '{detected_keyword}'에 해당하는 TTS 파일을 재생합니다.")
                        play_tts_file(detected_keyword)
                
                except json.JSONDecodeError:
                    print("JSON 파싱 오류")

                
            except asyncio.TimeoutError:
                print("STT 결과를 받지 못했습니다.")

    except Exception as e:
        print(f"웹소켓 연결 오류: {e}")
        import traceback
        print(traceback.format_exc())

async def async_wake_word_detection():
    global running
    porcupine = None
    audio = None
    stream = None

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

        print("Wake word 감지 시작... ('Ctrl+C'로 종료)")
        print(f"'{KEYWORD_NAMES[0]}' 또는 '{KEYWORD_NAMES[1]}'라고 말해보세요...")

        # 웹 클라이언트에 시작 상태 알림
        await broadcast_message({
            "type": "system",
            "contents": {
                "default": "ON",
                "data": "음성 비서 시스템이 실행되었습니다."
            }
        })

        # TTS 디렉토리 확인 및 생성
        if not os.path.exists(TTS_DIR):
            print(f"TTS 디렉토리가 없습니다. 생성합니다: {TTS_DIR}")
            os.makedirs(TTS_DIR)

        while running:
            # 오디오 프레임 읽기
            pcm = stream.read(frame_length, exception_on_overflow=False)
            pcm_np = np.frombuffer(pcm, dtype=np.int16)

            # Wake word 감지 처리
            keyword_index = porcupine.process(pcm_np)

            # Wake word가 감지되면
            if keyword_index >= 0:
                detected_keyword = KEYWORD_NAMES[keyword_index]
                print(f"'{detected_keyword}' 감지됨! 명령을 말씀해주세요...")

                # 알림음 재생
                play_alert_sound()

                # 웹소켓을 통해 서버로 오디오 스트리밍
                await stream_audio_to_server(stream, sample_rate, frame_length, detected_keyword)

                print("명령 처리 완료. 다시 대기 중...")

            await asyncio.sleep(0.01)

    except Exception as e:
        print(f"오류 발생: {e}")
        # 오류 발생 시 웹 클라이언트에 알림
        await broadcast_message({
            "type": "error",
            "contents": {
                "default": "OFF",
                "data": f"시스템 오류: {str(e)}"
            }
        })
    finally:
        print("리소스 정리 중...")
        # 웹 클라이언트에 종료 알림
        if running:  # 정상 종료가 아닌 경우에만
            await broadcast_message({
                "type": "system",
                "contents": {
                    "default": "OFF",
                    "data": "음성 비서 시스템이 종료되었습니다."
                }
            })
        # 리소스 정리
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if audio is not None:
            audio.terminate()
        if porcupine is not None:
            porcupine.delete()

def check_environment_vars():
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
        print(f"오류: .env 파일에 필요한 환경변수가 설정되지 않았습니다: {', '.join(missing_vars)}")
        exit(1)

async def start_websocket_server():
    """웹 클라이언트를 위한 웹소켓 서버 시작"""
    server = await websockets.serve(handle_web_client, WS_HOST, WS_PORT)
    logger.info(f"웹소켓 서버가 시작되었습니다: ws://{WS_HOST}:{WS_PORT}")
    return server

async def main():
    global running
    check_environment_vars()

    # 웹소켓 서버 시작
    web_server = await start_websocket_server()

    try:
        # 메인 태스크 실행
        await async_wake_word_detection()
    except KeyboardInterrupt:
        running = False
        print("\n프로그램이 종료되었습니다.")
    finally:
        # 웹소켓 서버 종료
        web_server.close()
        await web_server.wait_closed()
        print("웹소켓 서버가 종료되었습니다.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        running = False
        print("\n프로그램이 종료되었습니다.")
