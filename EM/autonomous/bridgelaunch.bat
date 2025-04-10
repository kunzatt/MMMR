@echo off
call C:\dev\ros2_eloquent\setup.bat  //ROS 실행 파일 실행
call .\install\local_setup.bat
cd  .\ssafy_bridge\launch
ros2 launch ssafybridge_launch.py