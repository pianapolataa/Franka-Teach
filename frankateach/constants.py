import numpy as np

# Network constants
LOCALHOST = 'localhost' # '127.0.0.1' # '0.0.0.0' # "localhost"
INTERNAL_IP =  "172.24.71.240" # "100.82.146.36" # "172.24.71.240"
PUBLIC_IP = "100.82.146.36"
CAM_PORT = 10005
VR_CONTROLLER_STATE_PORT = 8889
STATE_PORT = 8900
CONTROL_PORT = 8901
COMMANDED_STATE_PORT = 8902
RESKIN_STREAM_PORT = 12005
RUKA_STATE_PORT = 6009
RUKA_COMMANDED_STATE_PORT = 6010

transformed_position_keypoint_port = 8093

STATE_TOPIC = "state"
CONTROL_TOPIC = "control"

# VR constants
VR_TCP_HOST = "10.21.121.254"# "10.19.225.15" # "10.21.7.174" # "10.19.225.15"
VR_TCP_PORT = 5555
VR_CONTROLLER_TOPIC = b"oculus_controller"

#RESOLUTION SPECIFIC parameters
ARM_HIGH_RESOLUTION = 1 #  for arm teleoperation
ARM_LOW_RESOLUTION = 0

ARM_TELEOP_CONT = 1
ARM_TELEOP_STOP = 0

# Joint Information
OCULUS_NUM_KEYPOINTS = 24

OCULUS_JOINTS = {
    'metacarpals': [2, 6, 9, 12, 15],
    'knuckles': [6, 9, 12, 16],
    'thumb': [2, 3, 4, 5, 19],
    'index': [6, 7, 8, 20],
    'middle': [9, 10, 11, 21],
    'ring': [12, 13, 14, 22],
    'pinky': [15, 16, 17, 18, 23]
}

OCULUS_VIEW_LIMITS = {
    'x_limits': [-0.04, 0.04],
    'y_limits': [-0.02, 0.25],
    'z_limits': [-0.04, 0.04]
}

# Robot constants
GRIPPER_OPEN = -1
GRIPPER_CLOSE = 1
# translation matrix
H_R_U_star = np.array([[-1, 0, 0, 0], 
                       [0, 0, 1, 0], 
                       [0, 1, 0, 0], 
                       [0, 0, 0, 1]]) # for mirror flipping
H_R_U = np.array([ # from robot frame to unity frame （unity z dir = robot base y dir, unity y dir = robot base z dir)
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
]) # switching y and z axis

x_min, x_max = 0.2, 0.75
y_min, y_max = -0.4, 0.4
z_min, z_max = 0.05, 0.7  # 232, 550
ROBOT_WORKSPACE_MIN = np.array([x_min, y_min, z_min])
ROBOT_WORKSPACE_MAX = np.array([x_max, y_max, z_max])

TRANSLATIONAL_POSE_VELOCITY_SCALE = 5
ROTATIONAL_POSE_VELOCITY_SCALE = 0.75
ROTATION_VELOCITY_LIMIT = 0.5
TRANSLATION_VELOCITY_LIMIT = 1

# Frequencies
# TODO: Separate VR and deploy frequencies
VR_FREQ = 20
CONTROL_FREQ = 20
STATE_FREQ = 100
CAM_FPS = 30
DEPTH_PORT_OFFSET = 1000
