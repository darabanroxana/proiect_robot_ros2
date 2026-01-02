import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roxana/Desktop/proiect_robot/install/turtlebot3_football'
