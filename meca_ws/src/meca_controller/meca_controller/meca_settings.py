# ADJUST THESE GLOBAL CONSTANTS:
ROBOT1 = {'namespace':'/robot1', 'ip':'192.17.103.16'} # left robot
ROBOT2 = {'namespace':'/robot2', 'ip':'192.17.103.17'} # right robot
ASSUMED_RESTING_STATE_CONFIG = [0, 0, .1, 1.250, 20.500, 2.500] # if other robot is off, use this default position in motion planning (degrees)