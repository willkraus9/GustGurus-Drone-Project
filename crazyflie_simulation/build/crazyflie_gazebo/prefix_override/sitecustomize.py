import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/denis/Desktop/crazyflie_ws/install/crazyflie_gazebo'
