import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yousef/ARTPARK_Assignment/Task2/ros2_ws/install/sum_pkg'
