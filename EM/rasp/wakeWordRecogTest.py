import pvporcupine
import sounddevice as sd
import numpy as np
import time as time_module
import wave
import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# 환경변수에서 설정 로드
ACCESS_KEY = os.getenv("ACCESS_KEY")
KEYWORD_PATH = os.getenv("KEYWORD_PATH")
MODEL_PATH = os.getenv("MODEL_PATH")

def test_wake_word():
    porcupine = None
    
    try:
        # Porcupine 초기화
        porcupine = pvporcupine.create(
            access_key=ACCESS_KEY,
            keyword_paths=[KEYWORD_PATH],
            model_path=MODEL_PATH,
            sensitivities=[0.7]
        )
        
        # 오디오 설정
        sample_rate = porcupine.sample_rate
        frame_length = porcupine.frame_length
        
        print("Wake word 감지 시작... ('Ctrl+C'로 종료)")
        print("'미미'라고 말해보세요...")
        
        # 감지 후 녹음 설정
        recording = False
        recorded_frames = []
        recording_start_time = 0
        
        def audio_callback(indata, frames, time_info, status):
            nonlocal recording, recording_start_time, recorded_frames
            
            # 16비트 정수로 변환
            audio_frame = np.int16(indata[:, 0] * 32767).tobytes()
            
            # 녹음 중이면 프레임 저장
            if recording:
                recorded_frames.append(audio_frame)
                
                # 3초 녹음 후 종료
                if time_module.time() - recording_start_time > 3:
                    recording = False
                    print("녹음 완료. 명령어를 처리합니다...")
                    
                    # 녹음된 오디오 저장 (테스트용)
                    save_recording(recorded_frames, sample_rate)
                    
                    print("명령 처리 완료. 다시 대기 중...")
                    recorded_frames = []
            
            # Wake word 감지 처리
            pcm = np.frombuffer(audio_frame, dtype=np.int16)
            keyword_index = porcupine.process(pcm)
            
            # Wake word가 감지되면
            if keyword_index >= 0 and not recording:
                print("'미미' 감지됨! 명령을 말씀해주세요...")
                recording = True
                recording_start_time = time_module.time()
                recorded_frames = [audio_frame]  # 현재 프레임부터 녹음 시작
        
        # 오디오 스트림 시작
        with sd.InputStream(samplerate=sample_rate, channels=1, dtype='float32', 
                           callback=audio_callback, blocksize=frame_length):
            print("Press Ctrl+C to stop")
            while True:
                time_module.sleep(0.1)
                
    except KeyboardInterrupt:
        print("프로그램 종료...")
    finally:
        if porcupine is not None:
            porcupine.delete()

def save_recording(frames, sample_rate):
    """테스트용 녹음 파일 저장 함수"""
    filename = "recorded_command.wav"
    
    wf = wave.open(filename, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(2)  # 16비트
    wf.setframerate(sample_rate)
    wf.writeframes(b''.join(frames))
    wf.close()
    
    print(f"녹음된 오디오를 {filename}에 저장했습니다.")

if __name__ == "__main__":
    # 환경변수가 있는지 확인
    if not ACCESS_KEY or not KEYWORD_PATH or not MODEL_PATH:
        print("오류: .env 파일에 필요한 환경변수가 설정되지 않았습니다.")
        print("ACCESS_KEY, KEYWORD_PATH, MODEL_PATH를 .env 파일에 설정해주세요.")
        exit(1)
    
    test_wake_word()