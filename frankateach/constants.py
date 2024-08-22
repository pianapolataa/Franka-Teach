import numpy as np

VR_FREQ = 90
VR_TCP_HOST = "10.19.195.45"
VR_TCP_PORT = 5555
VR_CONTROLLER_TOPIC = b"oculus_controller"

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1
H_R_V = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
H_R_V_star = np.array([[-1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
# Robot workspace position limits in meters
x_min, x_max = 0.2, 0.75
y_min, y_max = -0.4, 0.4
z_min, z_max = 0.05, 0.7  # 232, 550
ROBOT_WORKSPACE_MIN = np.array([[x_min], [y_min], [z_min]])
ROBOT_WORKSPACE_MAX = np.array([[x_max], [y_max], [z_max]])
