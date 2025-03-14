import subprocess
import sys
import os

def install_requirements():
    print("필요한 패키지 설치 중...")
    
    # requirements.txt 파일 생성
    with open('requirements.txt', 'w') as f:
        f.write('soundfile\n')
        f.write('sounddevice\n')
        f.write('numpy\n')
        f.write('python-dotenv\n')
        f.write('pygame\n')
        f.write('google-cloud-speech\n')
        f.write('google-cloud-texttospeech\n')
        
    try:    
        # 필요한 패키지 설치
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-r', 'requirements.txt'])
        
        print("설치 완료!")
    except subprocess.CalledProcessError:
        print("패키지 설치 중 오류가 발생했습니다.")
        sys.exit(1)

if __name__ == "__main__":
    install_requirements()
