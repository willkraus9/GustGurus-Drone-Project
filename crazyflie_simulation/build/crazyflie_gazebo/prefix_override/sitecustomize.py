import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/install/crazyflie_gazebo'