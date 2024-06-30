import numpy as np
#from typing import final
DEFAULT_ROOT_CONFIG_PATH = 'config/panda_config.toml'
PANDA_ARM_MJCF_PATH = 'urdf/robot/panda_arm_mjcf.xml' #xml file (assumes this is in the same folder as this file)

ZERO_JOINTS = [0, 0, 0, 0, 0, 0, 0]
HOME_JOINTS = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
JOINT_NUM = 7
CARTESIAN_POSE_SIZE = 6
