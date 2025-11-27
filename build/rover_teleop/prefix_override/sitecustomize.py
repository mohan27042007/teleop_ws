import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohanarangan-t-r/teleop_ws/install/rover_teleop'
