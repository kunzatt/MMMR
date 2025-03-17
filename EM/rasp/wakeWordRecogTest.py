import pvporcupine
import pyaudio
import numpy as np
import time as time_module
import wave
import os
import signal
from dotenv import load_dotenv
import subprocess

# .env 파일 로드
load_dotenv()

# 환경변수에서 설정 로드
ACCESS_KEY = os.getenv("ACCESS_KEY")
# 여러 키워드 경로 설정
KEYWORD_PATHS = [
    os.getenv("MIMI_KEYWORD_PATH"),       # 첫 번째 키워드 ("미미")
    os.getenv("HAETAE_KEYWORD_PATH")      # 두 번째 키워드 ("해태")
]
MODEL_PATH = os.getenv("MODEL_PATH")

# 키워드 이름 목록 (감지 시 출력용)
KEYWORD_NAMES = ["미미", "해태"]

# 알림음 파일 경로 설정
ALERT_SOUND_PATH = os.getenv("ALERT_SOUND_PATH", "alert.mp3")  # 기본값: 현재 디렉토리의 alert.mp3

# 민감도 설정 (각 키워드마다 설정 가능)
SENSITIVITIES = [0.7, 0.7]

# 프로그램 실행 중 상태
running = True

# Ctrl+C 핸들러
def signal_handler(sig, frame):
    global running
    running = False
    print("\n프로그램 종료 중...")

# 시그널 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)

def play_alert_sound():
    """알림음 재생 함수"""
    if os.path.exists(ALERT_SOUND_PATH):
        try:
            # MP3 파일 확장자 확인
            if ALERT_SOUND_PATH.endswith('.mp3'):
                # MP3 재생은 mpg123 또는 mplayer 사용 (라즈베리파이에 설치 필요)
                try:
                    # mpg123 시도
                    subprocess.call(["mpg123", "-q", ALERT_SOUND_PATH])
                    print("알림음 출력 성공")
                except FileNotFoundError:
                    try:
                        # mplayer 시도
                        subprocess.call(["mplayer", "-really-quiet", ALERT_SOUND_PATH])
                        print("알림음 출력 성공")
                    except FileNotFoundError:
                        print("MP3 재생을 위해 mpg123 또는 mplayer를 설치하세요:")
                        print("sudo apt-get install mpg123")
                        print("또는")
                        print("sudo apt-get install mplayer")
            else:
                # WAV 파일은 aplay 사용
                subprocess.call(["aplay", ALERT_SOUND_PATH])
        except Exception as e:
            print(f"알림음 재생 중 오류 발생: {e}")
    else:
        print(f"알림음 파일을 찾을 수 없습니다: {ALERT_SOUND_PATH}")

def test_wake_word():
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

            # CPU 사용량 최적화
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

def generate_beep_sound(output_path="alert.wav", frequency=1000, duration=0.5):
    """간단한 비프음 생성 함수 - 알림음 파일이 없을 경우 활용"""
    sample_rate = 44100
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    tone = np.sin(frequency * 2 * np.pi * t)
    
    # 음량을 줄이기 위해 진폭 조정 (선택사항)
    tone = tone * 0.5
    
    # 16비트 정수로 변환
    tone_int = (tone * 32767).astype(np.int16)
    
    # WAV 파일로 저장
    wf = wave.open(output_path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(2)
    wf.setframerate(sample_rate)
    wf.writeframes(tone_int.tobytes())
    wf.close()
    
    print(f"비프음 생성 완료: {output_path}")
    return output_path

if __name__ == "__main__":
    # 환경변수가 있는지 확인
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
        print("ACCESS_KEY, MIMI_KEYWORD_PATH, HAETAE_KEYWORD_PATH, MODEL_PATH를 .env 파일에 설정해주세요.")
        exit(1)
    
    # 알림음 파일이 없으면 생성 (MP3는 생성 불가능하므로 WAV로 생성)
    if not os.path.exists(ALERT_SOUND_PATH):
        print(f"알림음 파일({ALERT_SOUND_PATH})이 없습니다. 기본 비프음을 생성합니다.")
        # MP3 경로를 WAV로 변경하여 기본 알림음 생성
        wav_path = "alert.wav"
        generate_beep_sound(wav_path)
        print(f"MP3 대신 WAV 파일({wav_path})을 생성했습니다. MP3 파일을 다운로드 받아 사용하세요.")
    
    # 라즈베리파이에서는 설치 확인
    try:
        import pvporcupine
        import pyaudio
    except ImportError as e:
        print(f"필요한 라이브러리가 설치되지 않았습니다: {e}")
        print("아래 명령어로 필요한 라이브러리를 설치하세요:")
        print("sudo apt-get update")
        print("sudo apt-get install python3-pip libportaudio2 portaudio19-dev")
        print("pip3 install pvporcupine pyaudio numpy python-dotenv")
        exit(1)

    test_wake_word()
