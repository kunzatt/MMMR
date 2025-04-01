import os
import time
from google.cloud import texttospeech
from dotenv import load_dotenv

def synthesize_text(text, output_file, gender="female"):
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
    
    # 파일로 저장
    with open(output_file, "wb") as out:
        out.write(response.audio_content)
    
    print(f"음성 파일이 저장되었습니다: {output_file}")
    return output_file

def main():
    try:
        load_dotenv()
        
        if not os.environ.get('GOOGLE_APPLICATION_CREDENTIALS'):
            print("경고: GOOGLE_APPLICATION_CREDENTIALS 환경 변수가 설정되지 않았습니다.")
            print(".env 파일에 GOOGLE_APPLICATION_CREDENTIALS를 설정해주세요.")
            return
        
        file_number = 1
        
        while True:
            # 사용자 입력 받기
            text = input("\nTTS 변환할 내용 (종료하려면 'q' 입력): ")
            
            if text.lower() == 'q':
                print("프로그램을 종료합니다.")
                break
            
            # 성별 선택 (기본값은 여성 음성)
            gender = input("성별 선택(남/여) [기본값: 여]: ").strip()
            
            # 성별 값 처리
            if gender.lower() in ["남", "남성", "male", "m"]:
                gender = "male"
            else:
                gender = "female"
            
            output_file = f"{file_number}.wav"
            
            synthesize_text(text, output_file, gender)
            
            file_number += 1
            
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == "__main__":
    main()