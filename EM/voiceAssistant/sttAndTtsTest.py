import os
import numpy as np
import sounddevice as sd
import soundfile as sf
import time
from google.cloud import speech
from google.cloud import texttospeech
import pygame
from dotenv import load_dotenv

def record_audio(seconds=8, sample_rate=16000, output_file="recorded_audio.wav"):
    """
    sounddevice를 사용하여 마이크에서 지정된 시간(초) 동안 오디오를 녹음합니다.
    저렴한 옵션을 위해 16kHz 샘플레이트와 mono 채널을 사용합니다.
    항상 같은 파일 이름을 사용하여 덮어쓰기 방식으로 저장합니다.
    """
    # 녹음 설정
    channels = 1
    
    print(f"{seconds}초 동안 말씀해주세요...")
    
    # 녹음 시작
    recording = sd.rec(int(seconds * sample_rate), 
                       samplerate=sample_rate, 
                       channels=channels,
                       dtype='int16')
    
    # 녹음이 완료될 때까지 대기
    sd.wait()
    
    print("녹음이 완료되었습니다.")
    
    # 고정된 WAV 파일로 저장 (덮어쓰기)
    sf.write(output_file, recording, sample_rate)
    
    return output_file

def transcribe_audio(audio_file_path):
    """
    오디오 파일을 텍스트로 변환합니다. (가장 저렴한 옵션 사용)
    """
    # Speech 클라이언트 초기화
    client = speech.SpeechClient()
    
    # 오디오 파일 읽기
    with open(audio_file_path, "rb") as audio_file:
        content = audio_file.read()
    
    # 오디오 설정 - 저렴한 옵션 사용
    audio = speech.RecognitionAudio(content=content)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="ko-KR",
        # 고급 모델 옵션은 비용이 더 높으므로 사용하지 않음
        use_enhanced=False,
        # mono 채널
        audio_channel_count=1
    )
    
    # STT 요청 실행
    response = client.recognize(config=config, audio=audio)
    
    # 결과 처리
    transcript = ""
    for result in response.results:
        transcript += result.alternatives[0].transcript
    
    return transcript

def synthesize_text(text, output_file="tts_output.mp3"):
    """
    텍스트를 오디오로 변환합니다. (가장 저렴한 옵션 사용)
    항상 같은 파일 이름을 사용하여 덮어쓰기 방식으로 저장합니다.
    """
    # TTS 클라이언트 초기화
    client = texttospeech.TextToSpeechClient()
    
    # 입력 텍스트 설정
    synthesis_input = texttospeech.SynthesisInput(text=text)
    
    # 음성 설정 - 저렴한 Standard 음성 사용
    voice = texttospeech.VoiceSelectionParams(
        language_code="ko-KR",
        # WaveNet이나 Studio 음성은 비용이 높으므로 Standard 음성 사용
        name="ko-KR-Standard-A",
        ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
    )
    
    # 오디오 출력 설정
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3
    )
    
    # TTS 요청 실행
    response = client.synthesize_speech(
        input=synthesis_input, voice=voice, audio_config=audio_config
    )
    
    # 고정된 MP3 파일로 저장 (덮어쓰기)
    with open(output_file, "wb") as out:
        out.write(response.audio_content)
    
    return output_file

def play_audio(audio_file_path):
    """
    pygame을 사용하여 오디오 파일을 재생합니다.
    """
    pygame.mixer.init()
    pygame.mixer.music.load(audio_file_path)
    pygame.mixer.music.play()
    
    # 재생이 끝날 때까지 대기
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)
    
    # 명시적으로 리소스 해제
    pygame.mixer.music.unload()
    pygame.mixer.quit()

def main():
    """
    전체 프로세스를 실행합니다:
    1. 오디오 녹음 (8초)
    2. STT로 변환
    3. TTS로 다시 변환
    4. 결과 재생
    """
    try:
        # .env 파일에서 환경 변수 로드
        load_dotenv()
        
        # Google Cloud 인증 설정 확인
        if not os.environ.get('GOOGLE_APPLICATION_CREDENTIALS'):
            print("경고: GOOGLE_APPLICATION_CREDENTIALS 환경 변수가 설정되지 않았습니다.")
            print(".env 파일에 GOOGLE_APPLICATION_CREDENTIALS를 설정해주세요.")
            return
        
        # 고정된 파일 이름 설정
        audio_file = "recorded_audio.wav"
        tts_file = "tts_output.mp3"
        
        # 1. 오디오 녹음
        record_audio(seconds=3, output_file=audio_file)
        print(f"녹음된 파일: {audio_file}")
        
        # 2. STT로 변환
        print("음성을 텍스트로 변환 중...")
        transcript = transcribe_audio(audio_file)
        print(f"인식된 텍스트: {transcript}")
        
        if not transcript:
            print("텍스트를 인식하지 못했습니다.")
            return
            
        # 3. TTS로 다시 변환
        print("텍스트를 음성으로 변환 중...")
        synthesize_text(transcript, output_file=tts_file)
        print(f"생성된 음성 파일: {tts_file}")
        
        # 4. 결과 재생
        print("변환된 음성을 재생합니다...")
        play_audio(tts_file)
        
        print("프로세스가 완료되었습니다.")
        
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == "__main__":
    main()
