import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/final-turtwig/ros2_ws/install/rat_roulette_pkg'
