import sys
if sys.prefix == 'c:\\python37':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = 'C:\\Users\\SSAFY\\Desktop\\catkin_ws\\src\\ssafy_bridge\\install\\ssafy_bridge'
