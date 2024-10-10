import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nakahira/ros2_ws/src/sea_drone_ros2/install/sea_drone_ros2'
