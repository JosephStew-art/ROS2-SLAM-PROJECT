import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi/ROS2-SLAM-PROJECT/ros2_ws/install/my_robot_controller'
