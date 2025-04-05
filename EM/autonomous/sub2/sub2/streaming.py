import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from flask import Flask, Response, render_template
import threading
from threading import Lock

app = Flask(__name__)
frame_lock = Lock()
current_frame = None  # 최신 프레임 저장용 전역 변수

class IMGParser(Node):
    def __init__(self):
        super().__init__(node_name='image_convertor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

    def img_callback(self, msg):
        global current_frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 그레이스케일 변환 및 리사이징
        #img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        #img_resize = cv2.resize(img_gray, (640, 480))
        
        # 프레임 업데이트 (스레드 안전)
        with frame_lock:
            _, buffer = cv2.imencode('.jpg', img_bgr)
            current_frame = buffer.tobytes()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global current_frame
        while True:
            with frame_lock:
                if current_frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host='0.0.0.0', port=5000, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    image_parser = IMGParser()
    
    # Flask 서버를 별도 스레드에서 실행
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    rclpy.spin(image_parser)

if __name__ == '__main__':
    main()
