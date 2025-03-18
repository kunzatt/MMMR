import sounddevice as sd
import numpy as np
import threading
import queue
import time
import torch
from faster_whisper import WhisperModel
import wave
import os

SAMPLE_RATE = 16000
CHANNELS = 1
BLOCK_SIZE = 2048

ENERGY_THRESHOLD = 0.05
SILENCE_THRESHOLD = 25
MIN_AUDIO_LENGTH = 0.5

MODEL_SIZE = "small"
LANGUAGE = "ko"

SAVE_AUDIO = True
AUDIO_DIR = "recorded_audio"

def list_audio_devices():
    """사용 가능한 오디오 장치 목록 출력"""
    devices = sd.query_devices()
    print("\n=== 사용 가능한 오디오 입력 장치 ===")
    for i, device in enumerate(devices):
        if device['max_input_channels'] > 0:  
            print(f"장치 {i}: {device['name']}")
    print("===============================\n")
    return devices

class RealTimeSTT:
    def __init__(self, device_id=None):
        if SAVE_AUDIO and not os.path.exists(AUDIO_DIR):
            os.makedirs(AUDIO_DIR)
        
        devices = list_audio_devices()
        self.device_id = device_id
        
        if device_id is not None:
            print(f"선택된 입력 장치: {devices[device_id]['name']}")
        else:
            print("기본 입력 장치를 사용합니다.")
        
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.is_speaking = False
        self.silence_counter = 0
        self.audio_buffer = []
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"사용 중인 장치: {self.device}")
        
        if self.device == "cuda":
            gpu_name = torch.cuda.get_device_name(0)
            print(f"GPU: {gpu_name}")
            print(f"CUDA 버전: {torch.version.cuda}")
        
        # Whisper 모델 로드
        print(f"Whisper 모델({MODEL_SIZE}) 로딩 중...")
        try:
            compute_type = "float16" if self.device == "cuda" else "int8"
            self.model = WhisperModel(MODEL_SIZE, device=self.device, compute_type=compute_type)
            print("모델 로드 완료!")
        except Exception as e:
            print(f"모델 로드 오류: {e}")
            raise
    
    def audio_callback(self, indata, frames, time, status):
        if status:
            print(f"오디오 상태: {status}")
        
        if indata.shape[1] > 1 and CHANNELS == 1:
            audio_data = indata[:, 0]
        else:
            audio_data = indata[:, 0] if indata.ndim > 1 else indata
        
        # 디버깅: 주기적으로 오디오 레벨 출력
        if self.is_speaking or np.max(np.abs(audio_data)) > ENERGY_THRESHOLD:
            print(f"오디오 레벨: {np.max(np.abs(audio_data)):.6f}")
        
        # 오디오 큐에 데이터 추가
        self.audio_queue.put(audio_data.copy())  # 복사본 사용
    
    def process_audio(self):
        """오디오 처리 메인 루프"""
        while self.is_recording:
            try:
                # 큐에서 오디오 데이터 가져오기
                audio_data = self.audio_queue.get(timeout=1)
                
                # 에너지 계산 (음성 감지용)
                energy = np.sqrt(np.mean(audio_data**2))
                max_amplitude = np.max(np.abs(audio_data))
                
                # 디버깅: 일정 수준 이상의 에너지일 때 값 출력
                if max_amplitude > ENERGY_THRESHOLD * 0.5:
                    print(f"에너지: {energy:.6f}, 최대진폭: {max_amplitude:.6f}, 임계값: {ENERGY_THRESHOLD:.6f}")
                
                # 음성 감지 로직
                if not self.is_speaking and max_amplitude > ENERGY_THRESHOLD:
                    self.is_speaking = True
                    self.silence_counter = 0
                    print("음성 감지됨. 녹음 중...")
                
                # 현재 말하고 있는 상태
                if self.is_speaking:
                    # 오디오 버퍼에 데이터 추가
                    self.audio_buffer.append(audio_data)
                    
                    # 무음 감지
                    if max_amplitude <= ENERGY_THRESHOLD:
                        self.silence_counter += 1
                        if self.silence_counter % 5 == 0:  # 디버깅용
                            print(f"무음 카운터: {self.silence_counter}/{SILENCE_THRESHOLD}")
                    else:
                        self.silence_counter = 0
                    
                    # 충분한 무음이 감지되면 처리
                    if self.silence_counter >= SILENCE_THRESHOLD:
                        print("무음 감지. 음성 처리 중...")
                        self.process_speech()
                        self.is_speaking = False
                        self.audio_buffer = []
            
            except queue.Empty:
                pass  # 타임아웃, 계속 진행
            
            except Exception as e:
                print(f"오디오 처리 오류: {e}")
                import traceback
                traceback.print_exc()
    
    def save_audio_file(self, audio_data):
        """디버깅용: 오디오 파일 저장"""
        if not SAVE_AUDIO:
            return
        
        timestamp = int(time.time())
        file_path = os.path.join(AUDIO_DIR, f"recording_{timestamp}.wav")
        
        with wave.open(file_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(SAMPLE_RATE)
            
            gain = 1.0  # 필요에 따라 조정 (1.0보다 크면 볼륨 증가)
            audio_int16 = np.clip(audio_data * 32767 * gain, -32768, 32767).astype(np.int16)
            wf.writeframes(audio_int16.tobytes())
        
        print(f"오디오 저장됨: {file_path}")
        return file_path
    
    def process_speech(self):
        """녹음된 음성을 STT로 처리"""
        # 녹음된 오디오가 너무 짧으면 무시
        if len(self.audio_buffer) < int(MIN_AUDIO_LENGTH * SAMPLE_RATE / BLOCK_SIZE):
            print("오디오가 너무 짧습니다. 무시합니다.")
            return
        
        try:
            audio_data = np.concatenate(self.audio_buffer)
            
            audio_float = audio_data.astype(np.float32)
            
            max_value = np.max(np.abs(audio_float))
            if max_value > 0:
                audio_float = audio_float / max_value
            
            file_path = self.save_audio_file(audio_float)
            
            print(f"처리할 오디오 길이: {len(audio_float) / SAMPLE_RATE:.2f}초")
            start_time = time.time()
            
            # Whisper로 음성 인식
            segments, info = self.model.transcribe(
                audio_float, 
                language=LANGUAGE,
                beam_size=5,
                vad_filter=True,
                initial_prompt="오늘 날씨 어때"  # 인식 힌트 추가
            )
            
            # 결과 처리
            texts = []
            for segment in segments:
                texts.append(segment.text)
            
            full_text = " ".join(texts).strip()
            process_time = time.time() - start_time
            
            print("\n===== STT 결과 =====")
            print(f"인식된 텍스트: {full_text}")
            print(f"처리 시간: {process_time:.2f}초")
            if SAVE_AUDIO:
                print(f"저장된 오디오: {file_path}")
            print("====================\n")
            
        except Exception as e:
            print(f"음성 인식 오류: {e}")
            import traceback
            traceback.print_exc()
    
    def start(self):
        """녹음 시작"""
        if self.is_recording:
            print("이미 녹음 중입니다.")
            return
        
        self.is_recording = True
        self.is_speaking = False
        self.audio_buffer = []
        
        # 오디오 처리 스레드 시작
        self.process_thread = threading.Thread(target=self.process_audio)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        # 오디오 스트림 시작
        self.stream = sd.InputStream(
            device=self.device_id,
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            callback=self.audio_callback,
            blocksize=BLOCK_SIZE
        )
        
        try:
            self.stream.start()
            print("실시간 STT가 시작되었습니다. 말씀해주세요...")
            print(f"오디오 설정: 샘플레이트={SAMPLE_RATE}Hz, 음성 감지 임계값={ENERGY_THRESHOLD}")
            print("(종료하려면 Ctrl+C를 누르세요)")
        except Exception as e:
            print(f"오디오 스트림 시작 오류: {e}")
            import traceback
            traceback.print_exc()
            self.is_recording = False
            raise
    
    def stop(self):
        """녹음 중지"""
        if not self.is_recording:
            return
        
        self.is_recording = False
        
        # 남은 오디오 처리
        if self.is_speaking and len(self.audio_buffer) > 0:
            self.process_speech()
        
        # 스트림 정리
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        
        print("STT가 중지되었습니다.")

def main():
    try:
        device_id = None
        
        stt = RealTimeSTT(device_id)
        stt.start()
        
        # 메인 스레드는 키보드 인터럽트를 기다림
        while True:
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n종료 중...")
    
    except Exception as e:
        print(f"오류 발생: {e}")
        import tracebacky
        traceback.print_exc()
    
    finally:
        if 'stt' in locals():
            stt.stop()

if __name__ == "__main__":
    main()