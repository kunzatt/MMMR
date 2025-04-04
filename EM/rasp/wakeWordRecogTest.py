import pvporcupine
import pyaudio
import numpy as np
import time as time_module
import wave
import os
import signal
from dotenv import load_dotenv
import subprocess
import pvporcupine

load_dotenv()

ACCESS_KEY = os.getenv("ACCESS_KEY")

KEYWORD_PATHS = [
    os.getenv("MIMI_KEYWORD_PATH"),       #미미
    os.getenv("HAETAE_KEYWORD_PATH")      #해태
]
MODEL_PATH = os.getenv("MODEL_PATH")
ALERT_SOUND_PATH = os.getenv("ALERT_SOUND_PATH", "alert.mp3") 

KEYWORD_NAMES = ["미미", "해태"]


# 민감도 설정 (각 키워드마다 설정 가능)
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

def recog_wake_word():
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

        # 감지 후 녹음 설정
        recording = False
        recorded_frames = []
        recording_start_time = 0
        detected_keyword = ""

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

        global running
        while running:
            # 오디오 프레임 읽기
            pcm = stream.read(frame_length, exception_on_overflow=False)
            pcm = np.frombuffer(pcm, dtype=np.int16)

            # Wake word 감지 처리
            keyword_index = porcupine.process(pcm)

            # 녹음 중이면 프레임 저장
            if recording:
                recorded_frames.append(pcm.tobytes())

                # 3초 녹음 후 종료
                if time_module.time() - recording_start_time > 3:
                    recording = False
                    print("녹음 완료. 명령어를 처리합니다...")

                    # 녹음된 오디오 저장 (테스트용)
                    save_recording(recorded_frames, sample_rate, detected_keyword)

                    print("명령 처리 완료. 다시 대기 중...")
                    recorded_frames = []

            # Wake word가 감지되면
            if keyword_index >= 0 and not recording:
                detected_keyword = KEYWORD_NAMES[keyword_index]
                print(f"'{detected_keyword}' 감지됨! 명령을 말씀해주세요...")
                
                # 알림음 재생
                play_alert_sound()
                
                recording = True
                recording_start_time = time_module.time()
                recorded_frames = [pcm.tobytes()]  # 현재 프레임부터 녹음 시작

            time_module.sleep(0.01)

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

def save_recording(frames, sample_rate, keyword):
    """테스트용 녹음 파일 저장 함수"""
    # 파일 이름에 감지된 키워드 포함
    filename = f"recorded_command_{keyword}.wav"

    wf = wave.open(filename, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(2)  # 16비트
    wf.setframerate(sample_rate)
    wf.writeframes(b''.join(frames))
    wf.close()

    print(f"녹음된 오디오를 {filename}에 저장했습니다.")

if __name__ == "__main__":
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
    recog_wake_word()
