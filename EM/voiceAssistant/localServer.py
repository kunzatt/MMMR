import asyncio
import numpy as np
import os
import time
import torch
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from faster_whisper import WhisperModel
from dotenv import load_dotenv
import logging
import json
from typing import Dict, List, Any

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("stt-server")

load_dotenv()

# 환경 변수 설정
MODEL_SIZE = os.getenv("MODEL_SIZE", "small")
LANGUAGE = os.getenv("LANGUAGE", "ko")
HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8000"))

# 음성 감지 설정
ENERGY_THRESHOLD = float(os.getenv("ENERGY_THRESHOLD", "0.05"))
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

# 모델 로드
@app.on_event("startup")
async def startup_event():
    global model
    logger.info("서버 시작 중...")
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    logger.info(f"사용 중인 장치: {device}")
    
    if device == "cuda":
        gpu_name = torch.cuda.get_device_name(0)
        logger.info(f"GPU: {gpu_name}")
        logger.info(f"CUDA 버전: {torch.version.cuda}")
    
    # Whisper 모델 로드
    logger.info(f"Whisper 모델({MODEL_SIZE}) 로딩 중...")
    try:
        compute_type = "float16" if device == "cuda" else "int8"
        model = WhisperModel(MODEL_SIZE, device=device, compute_type=compute_type)
        logger.info("모델 로드 완료!")
    except Exception as e:
        logger.error(f"모델 로드 오류: {e}")
        raise

class AudioProcessor:
    def __init__(self, sample_rate: int = 16000):
        self.sample_rate = sample_rate
        self.audio_buffer: List[np.ndarray] = []
        self.is_speaking = False
        self.silence_counter = 0
        self.total_frames = 0
        self.metadata = {}
    
    def reset(self):
        """음성 처리 상태 초기화"""
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.total_frames = 0
    
    def process_frame(self, frame_data: bytes) -> bool:
        """
        오디오 프레임 처리
        
        Args:
            frame_data: 바이너리 오디오 데이터
            
        Returns:
            bool: 음성 종료 감지 여부
        """
        # 바이트 데이터를 numpy 배열로 변환
        audio_frame = np.frombuffer(frame_data, dtype=np.int16)
        self.total_frames += len(audio_frame)
        
        # 정규화
        audio_float = audio_frame.astype(np.float32) / 32767.0
        
        # 에너지 계산 (음성 감지용)
        max_amplitude = np.max(np.abs(audio_float))
        
        # 로깅 (디버깅용)
        if max_amplitude > ENERGY_THRESHOLD * 0.5:
            logger.debug(f"최대진폭: {max_amplitude:.6f}, 임계값: {ENERGY_THRESHOLD:.6f}")
        
        # 음성 감지 로직
        if not self.is_speaking and max_amplitude > ENERGY_THRESHOLD:
            self.is_speaking = True
            self.silence_counter = 0
            logger.info("음성 감지됨. 오디오 캡처 중...")
        
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
                logger.info("무음 감지. 음성 종료됨.")
                return True
        
        return False
    
    def get_audio_data(self) -> np.ndarray:
        """모든 오디오 버퍼를 하나의 배열로 연결"""
        if not self.audio_buffer:
            return np.array([], dtype=np.float32)
        
        audio_data = np.concatenate(self.audio_buffer)
        
        # 정규화
        max_value = np.max(np.abs(audio_data))
        if max_value > 0:
            audio_data = audio_data / max_value
        
        return audio_data
    
    def get_duration(self) -> float:
        """녹음된 오디오의 길이(초)"""
        return self.total_frames / self.sample_rate

async def transcribe_audio(audio_data: np.ndarray, metadata: Dict[str, Any]) -> str:
    """
    오디오 데이터를 텍스트로 변환
    
    Args:
        audio_data: 처리할 오디오 데이터
        metadata: 메타데이터 (키워드 등)
        
    Returns:
        str: 인식된 텍스트
    """
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
    initial_prompt = f"{detected_keyword}를 호출했습니다" if detected_keyword else None
    
    try:
        # Whisper로 음성 인식
        segments, info = model.transcribe(
            audio_data, 
            language=LANGUAGE,
            beam_size=5,
            vad_filter=True,
            initial_prompt=initial_prompt
        )
        
        # 결과 처리
        texts = []
        for segment in segments:
            texts.append(segment.text)
        
        full_text = " ".join(texts).strip()
        process_time = time.time() - start_time
        
        logger.info(f"인식된 텍스트: {full_text}")
        logger.info(f"처리 시간: {process_time:.2f}초")
        
        return full_text
        
    except Exception as e:
        logger.error(f"음성 인식 오류: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return ""

@app.websocket("/listen")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("웹소켓 클라이언트 연결됨")
    
    processor = AudioProcessor()
    streaming = True
    
    try:
        while streaming:
            try:
                # 텍스트와 바이너리 메시지를 모두 처리
                data = await websocket.receive_text()
                
                # 텍스트 메시지 처리
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
                
            except WebSocketDisconnect:
                logger.info("클라이언트 연결 종료")
                streaming = False
                break
            except Exception as e:
                try:
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
            transcription = await transcribe_audio(audio_data, processor.metadata)
            # 결과 전송
            await websocket.send_text(transcription)
            logger.info("STT 결과 전송 완료")
        else:
            logger.warning("오디오가 너무 짧아 처리하지 않습니다.")
            await websocket.send_text("")
        
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
    logger.info(f"서버 시작: {HOST}:{PORT}")
    uvicorn.run(app, host=HOST, port=PORT)