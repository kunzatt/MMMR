import time
import threading
from iot_ws import WebSocketServer

def main():
    # 웹소켓 서버 인스턴스 생성
    print("IoT 컨트롤러를 시작합니다...")
    server = WebSocketServer(host='0.0.0.0', port=12345)
    
    # 서버를 별도 스레드에서 실행
    server_thread = threading.Thread(target=server.start)
    server_thread.daemon = True  # 메인 프로그램 종료 시 스레드도 종료
    server_thread.start()
    
    print("웹소켓 서버가 백그라운드에서 실행 중입니다.")
    print("서버가 시작되는 동안 잠시 기다려주세요...")
    time.sleep(2)  # 서버 시작 대기
    
    # IoT 장치 제어 인터페이스
    try:
        while True:
            print("\n=== IoT 장치 제어 메뉴 ===")
            print("1. 에어컨 제어")
            print("2. 커튼 제어")
            print("3. TV 제어")
            print("0. 종료")
            
            choice = input("메뉴를 선택하세요: ")
            
            if choice == '0':
                break
                
            if choice in ['1', '2', '3']:
                # 장치 선택
                if choice == '1':
                    device = "airConditioner"
                    device_name = "에어컨"
                elif choice == '2':
                    device = "curtain"
                    device_name = "커튼"
                else:
                    device = "TV"
                    device_name = "TV"
                
                # 상태 선택
                print(f"\n{device_name} 상태 설정")
                print("1. ON (켜기)")
                print("2. OFF (끄기)")
                
                state_choice = input("상태를 선택하세요: ")
                
                if state_choice == '1':
                    state = "ON"
                elif state_choice == '2':
                    state = "OFF"
                else:
                    print("잘못된 선택입니다. 메인 메뉴로 돌아갑니다.")
                    continue
                
                # IoT 장치 제어
                success = server.iotControl(device, state)
                if success:
                    print(f"{device_name}을(를) {state}으로 설정했습니다.")
                else:
                    print(f"{device_name} 제어 실패!")
            else:
                print("잘못된 선택입니다. 다시 시도하세요.")
    
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    finally:
        # 서버 종료
        print("웹소켓 서버를 종료합니다...")
        server.stop()
        print("종료되었습니다.")

if __name__ == "__main__":
    main()