import pvporcupine
import pyaudio
import numpy as np
import time
import os
import signal
import asyncio
import websockets
import json
from dotenv import load_dotenv
import subprocess

load_dotenv()

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

# TTS 파일 경로
TTS_DIR = "./tts_files"
MIMI_TTS_FILE = "not_understand_f.wav"
HAETAE_TTS_FILE = "not_understand_m.wav"

# 민감도 설정
SENSITIVITIES = [0.7, 0.7]

# 프로그램 실행 중 상태
running = True

# Ctrl+C 핸들러
def signal_handler(sig, frame):
    global running
    running = False
    print("\n프로그램 종료 중...")

signal.signal(signal.SIGINT, signal_handler)

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

async def stream_audio_to_server(audio_stream, sample_rate, frame_length, detected_keyword):
    """웹소켓을 통해 서버로 오디오 스트리밍"""
    try:
        print(f"서버 연결 시도 중: {SERVER_URL}")
        async with websockets.connect(SERVER_URL) as websocket:
            # 초기 메타데이터 전송
            await websocket.send(f"METADATA:sample_rate={sample_rate},keyword={detected_keyword}")
            print(f"서버에 연결되었습니다. 오디오 스트리밍 시작...")

            # 스트리밍 시작 시간
            start_time = time.time()
            streaming = True

            while streaming and running:
                # 오디오 데이터 읽기
                try:
                    pcm = audio_stream.read(frame_length, exception_on_overflow=False)
                    # 데이터를 바이너리로 변환하여 전송
                    await websocket.send(pcm)

                    #최대 10초간 스트리밍
                    if time.time() - start_time > 10:
                        # 종료 알림 전송
                        await websocket.send("STREAMING_END")
                        streaming = False
                        print("스트리밍 시간 종료.")

                    # 서버로부터 응답 체크 (비동기로 처리)
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
            await websocket.send("STREAMING_END")
            print("오디오 스트리밍 완료.")

            # 서버의 STT 결과 대기
            try:
                result = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"STT 결과: {result}")
                
                # 결과 JSON 파싱
                try:
                    json_result = json.loads(result)
                    
                    # 결과가 -1인 경우 호출어에 따른 TTS 파일 재생
                    if "result" in json_result and json_result["result"] == "-1":
                        print(f"결과가 -1입니다. '{detected_keyword}'에 해당하는 TTS 파일을 재생합니다.")
                        play_tts_file(detected_keyword)
                except json.JSONDecodeError:
                    print("JSON 파싱 오류")
                
            except asyncio.TimeoutError:
                print("STT 결과를 받지 못했습니다.")

    except Exception as e:
        print(f"웹소켓 연결 오류: {e}")

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
    finally:
        print("리소스 정리 중...")
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

    if missing_vars:
        print(f"오류: .env 파일에 필요한 환경변수가 설정되지 않았습니다: {', '.join(missing_vars)}")
        exit(1)

async def main():
    check_environment_vars()
    await async_wake_word_detection()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        running = False
        print("\n프로그램이 종료되었습니다.")