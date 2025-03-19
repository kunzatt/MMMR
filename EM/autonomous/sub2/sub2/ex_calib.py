import numpy as np
import cv2
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 2368,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.6,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 640, # image width
    "HEIGHT": 480, # image height
    "FOV": 90, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0., # meter
    "Y": 0,
    "Z":  1.0,
    "YAW": 0, # deg
    "PITCH": 0.0,
    "ROLL": 0
}

# ex 노드 설명
# 로봇에 달려있는 라이다와 카메라 간의 위치 및 자세 정보를 위의 params_lidar, params_cam으로
# 받은 다음, 이를 가지고 좌표 변환 행렬을 만들고, 카메라 이미지에 라이다 포인트들을 projection
# 하는 노드입니다.
# 2d 공간만 표현되는 카메라는 3d 위치정보를 포함하지 않기 때문에,
# 라이다의 포인트들을 프레임에 정사영시켜, 카메라 내 객체들의 위치 정보를 추정하도록 만들 수
# 있습니다.

# 노드 로직 순서
# 1. 노드에 필요한 라이다와 카메라 topic의 subscriber 생성
# 2. Params를 받아서 라이다 포인트를 카메라 이미지에 projection 하는 transformation class 정의하기
# 3. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장.
# 4. 라이다 콜백함수에서 2d scan data(거리와 각도)를 가지고 x,y 좌표계로 변환
# 5. 라이다 x,y 좌표 데이터 중 정면 부분만 crop
# 6. transformation class 의 transform_lidar2cam로 라이다 포인트를 카메라 3d좌표로 변환
# 7. transformation class 의 project_pts2img 로 라이다 포인트를 2d 픽셀 좌표상으로 정사영
# 8. draw_pts_img()로 카메라 이미지에 라이다 포인트를 draw 하고 show


# 좌표변환을 하는데 필요한 rotation, translation 행렬을 아래와 같이 완성시켜 놓았습니다. 
# 이를 활용하여 라이다 scan 포인트들을 이미지 프레임 상으로 변환시켜주는 클래스인 
# LIDAR2CAMTransform 를 완성시키십시오.

def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1.0,         0.0,              0.0,                0.0],
                    [0.0,         math.cos(roll), -math.sin(roll) , 0.0],
                    [0.0,         math.sin(roll), math.cos(roll)  , 0.0],
                    [0.0,         0.0,              0.0,               1.0],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0.0,      math.sin(pitch) , 0.0],
                    [0.0,                  1.0,      0.0               , 0.0],
                    [-math.sin(pitch),   0.0,      math.cos(pitch) , 0.0],
                    [0.0,         0.0,              0.0,               1.0],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0.0,    0.0],
                    [math.sin(yaw),    math.cos(yaw),     0.0,    0.0],
                    [0.0,                0.0,                 1.0,    0.0],
                    [0.0,         0.0,              0.0,               1.0],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R

def translationMtx(x, y, z):
     
    M = np.array([[1.0,         0.0,              0.0,               x],
                  [0.0,         1.0,              0.0,               y],
                  [0.0,         0.0,              1.0,               z],
                  [0.0,         0.0,              0.0,               1],
                  ])
    
    return M



def transformMTX_lidar2cam(params_lidar, params_cam):

    """
    transformMTX_lidar2cam 내 좌표 변환행렬 로직 순서
    1. params에서 라이다와 카메라 센서들의 자세, 위치 정보를 뽑기.
    2. 라이다에서 카메라 위치까지 변환하는 translation 행렬을 정의
    3. 카메라의 자세로 맞춰주는 rotation 행렬을 정의.
    4. 위의 두 행렬을 가지고 최종 라이다-카메라 변환 행렬을 정의.
    """

    """
    로직 1. params에서 라이다와 카메라 센서들의 자세, 위치 정보를 뽑기.
    """

    lidar_yaw, lidar_pitch, lidar_roll = params_lidar["YAW"], params_lidar["PITCH"], params_lidar["ROLL"]
    cam_yaw, cam_pitch, cam_roll = params_cam["YAW"], params_cam["PITCH"], params_cam["ROLL"]
    
    lidar_pos = np.array([params_lidar["X"], params_lidar["Y"], params_lidar["Z"]])
    cam_pos = np.array([params_cam["X"], params_cam["Y"], params_cam["Z"]])


    """

    로직 2. 라이다에서 카메라 까지 변환하는 translation 행렬을 정의
    """
    pose_diff = cam_pos - lidar_pos
    Tmtx = translationMtx(*pose_diff) # *을 통해 리스트나 튜플 등의 반복 가능 한 요소를 분해해서 전달


    """
    로직 3. 카메라의 자세로 맞춰주는 rotation 행렬을 정의
    카메라의 좌표는 z축이 카메라 렌즈를 향하고, x축이 이미지 화면 아래,  y축이 오른쪽을 향하는게 일반적임
    이에 맞는 순소러 rotation을 여러 개 만들어서 곱해줘야한다.
    """

    # 먼저 라이다의 orientation을 제거한 다음, 카메라의 orientation을 적용
    # 이를 위해 라이다 회전 행렬의 역행렬(전치행렬)과 카메라 회전 행렬을 곱합니다
    R_lidar = rotationMtx(lidar_yaw, lidar_pitch, lidar_roll)
    R_lidar_inv = np.transpose(R_lidar)  # 회전 행렬의 역행렬은 전치행렬
    
    R_cam = rotationMtx(cam_yaw, cam_pitch, cam_roll)
    
    # 축 방향 보정 행렬 (좌표계 변환)
    R_axis = np.array([
        [0.0, -1.0, 0.0, 0.0],  # X_lidar → -Y_cam
        [0.0, 0.0, -1.0, 0.0],  # Y_lidar → -Z_cam 
        [1.0, 0.0, 0.0, 0.0],   # Z_lidar → X_cam
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    # 최종 회전 행렬 조합
    Rmtx = R_cam @ R_axis @ R_lidar_inv  # 순서 중요!


    """

    로직 4. 위의 두 행렬을 가지고 최종 라이다-카메라 변환 행렬을 정의
    """
    RT = Tmtx @ Rmtx


    """
    테스트

    params_lidar = {
        "X": 0, # meter
        "Y": 0,
        "Z": 0.6,
        "YAW": 0, # deg
        "PITCH": 0,
        "ROLL": 0
    }


    params_cam = {
        "WIDTH": 640, # image width
        "HEIGHT": 480, # image height
        "FOV": 90, # Field of view
        "X": 0., # meter
        "Y": 0,
        "Z":  1.0,
        "YAW": 0, # deg
        "PITCH": 0.0,
        "ROLL": 0
    }

    이면

    R_T = 
    [[ 6.12323400e-17 -1.00000000e+00  0.00000000e+00  0.00000000e+00]
    [ 6.12323400e-17  3.74939946e-33 -1.00000000e+00  4.00000000e-01]
    [ 1.00000000e+00  6.12323400e-17  6.12323400e-17 -2.44929360e-17]
    [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

    """

    # return np.eye(4)
    return RT


def project2img_mtx(params_cam):

    """
    project2img_mtx 내 projection 행렬 계산 로직 순서
    1. params에서 카메라의 width, height, fov를 가져와서 focal length를 계산.
    2. 카메라의 파라메터로 이미지 프레임 센터를 계산.
    3. Projection 행렬을 계산 

    """

    
    """
    로직 1. params에서 카메라의 width, height, fov를 가져와서 focal length를 계산.
    """
    #focal length(렌즈-카메라 거리), working distance(렌즈-피사체 거리), FOV(카메라의 이미지 센서 채우는 피사체 영역- 시야각)
    #focal length = Image sensor size * working distance / FOV
    #FOV = 2 * (working distance * tan(세타/2))

    fc_x = params_cam["WIDTH"] / (2.0 * np.tan(params_cam["FOV"] * np.pi / 360.0))
    fc_y = params_cam["HEIGHT"] / (2.0 * np.tan(params_cam["FOV"] * np.pi / 360.0))

    """
    로직 2. 카메라의 파라메터로 이미지 프레임 센터를 계산.
    """
    cx = params_cam["WIDTH"] / 2.0
    cy = params_cam["HEIGHT"] / 2.0

    """

    로직 3. Projection 행렬을 계산.
    """
    R_f = np.array([[fc_x,  0,     cx],
                    [0,     fc_y,  cy]])


    

    # return np.zeros((2,3))
    return R_f


def draw_pts_img(img, xi, yi):

    point_np = img

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    return point_np


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        """

        LIDAR2CAMTransform 정의 및 기능 로직 순서
        1. Params를 입력으로 받아서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의. 
        2. 클래스 내 self.RT로 라이다 포인트들을 카메라 좌표계로 변환.
        4*4 매트릭스에 3차원의 좌표를 곱하기 위해서 augmentation 해서 차원 맞추기기
        3. RT로 좌표 변환된 포인트들의 normalizing plane 상의 위치를 계산. 
        4. normalizing plane 상의 라이다 포인트들에 proj_mtx를 곱해 픽셀 좌표값 계산.
        5. 이미지 프레임 밖을 벗어나는 포인트들을 crop.
        """
        
        # 로직 1. Params에서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        
        
        """
        로직 2. 클래스 내 self.RT로 라이다 포인트들을 카메라 좌표계로 변환시킨다.
        """
        # 4*4 행렬로 변환
        xyz_aug = np.hstack([xyz_p, np.ones((xyz_p.shape[0],1))]).T
        # 행렬 곱 연산 후 차원 축소
        xyz_c = (self.RT @ xyz_aug).T[:,:3]

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyi=np.zeros((xyz_c.shape[0], 2))

        """
        로직 3. RT로 좌표 변환된 포인트들의 normalizing plane 상의 위치를 계산.
        """
        xn = xyz_c[:,0] / xyz_c[:,2] # x값 나누기 z값
        yn = xyz_c[:,1] / xyz_c[:,2] # y값 나누기 z값

        # 로직 4. normalizing plane 상의 라이다 포인트들에 proj_mtx를 곱해 픽셀 좌표값 계산.
        # np.stack을 사용하여 xn, yn, ones를 3차원 배열로 결합
        
        ones = np.ones_like(xn)
        points = np.stack((xn, yn, ones), axis=-1)
        xyi = np.matmul(self.proj_mtx, points.T).T


        """
        로직 5. 이미지 프레임 밖을 벗어나는 포인트들을 crop.
        """

        if crop:
            # 조건을 만족하는 요소의 인덱스 튜플 반환
            # valid_idx = np.where(
            #     (xyi[:,0] >= 0) & (xyi[:,0] < self.width) & # 이미지 범위 내의 x 좌표
            #     ((xyi[:,1] >= 0) & (xyi[:,1] < self.height)) # 이미지 범위 내의 y 좌표
            # )[0]

            # xyi = xyi[valid_idx]
            xyi = self.crop_pts(xyi)
        else:
            pass
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

    


class SensorCalib(Node):

    def __init__(self):
        super().__init__(node_name='ex_calib')

        # 로직 1. 노드에 필요한 라이다와 카메라 topic의 subscriber 생성

        self.subs_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback, 10)

        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        # 로직 2. Params를 받아서 라이다 포인트를 카메라 이미지에 projection 하는
        # transformation class 정의하기

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):
        
        """
   
        로직 3. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장.
        """
        # np.frombuffer() 로 압축된 바이트 데이터 numpy로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)

        #cv2.imdecode() 로 numpy 배열을 컬러 이미지로 디코딩
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
    
        """

        로직 4. 라이다 2d scan data(거리와 각도)를 가지고 x,y 좌표계로 변환
        """

        self.R = np.array(msg.ranges)
        # cos, sin 함수로 각각 x,y축으로 매핑
        # np.linspace() 함수로 0 ~ 2pi 를 360개의 각도로 나눔
        x = self.R * np.cos(np.linspace(0, 2*np.pi, 360)) 
        y = self.R * np.sin(np.linspace(0, 2*np.pi, 360))
        z = np.zeros_like(x) # 2d 데이터이므로 z 값은 0으로 설정정


        self.xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)
        

    def timer_callback(self):

        if self.xyz is not None and self.img is not None :

            """
            로직 5. 라이다 x,y 좌표 데이터 중 정면 부분만 crop
            """
            xyz_p = self.xyz[np.where(self.xyz[:,0] > 0)]

            """
            로직 6. transformation class 의 transform_lidar2cam 로 카메라 3d 좌표 변환
            """
            xyz_c = self.l2c_trans.transform_lidar2cam(xyz_p)

            """
            로직 7. transformation class 의 project_pts2img로 카메라 프레임으로 정사영
            """
            xy_i = self.l2c_trans.project_pts2img(xyz_c) 

            """
            로직 8. draw_pts_img()로 카메라 이미지에 라이다 포인트를 draw 하고 show
            """
            
            img_l2c = draw_pts_img(self.img, xy_i[:,0].astype(int), xy_i[:,1].astype(int))

            cv2.imshow("Lidar2Cam", img_l2c)
            cv2.waitKey(1)

        else:

            print("waiting for msg")
            pass


def main(args=None):

    rclpy.init(args=args)

    calibrator = SensorCalib()

    rclpy.spin(calibrator)


if __name__ == '__main__':

    main()