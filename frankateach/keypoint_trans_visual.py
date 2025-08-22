import open3d as o3d
import numpy as np
import pickle
from copy import copy
from frankateach.vectorops import *
from frankateach.constants import *
from numpy.linalg import pinv

import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D

### ------------------ Step 1: Load Pickle ------------------ ###
with open("oculus_data.pkl", "rb") as f:
    data = pickle.load(f)

### ------------------ Step 2: Extract Keypoints ------------------ ###
def _extract_data_from_token(token):        
    data = token.decode("utf-8")
    keypoint_vals = [0] if data.startswith('absolute') else [1]
    vector_strings = data.split(':')[1].strip().split('|')
    for vector_str in vector_strings:
        vector_vals = vector_str.split(',')
        for float_str in vector_vals[:3]:
            keypoint_vals.append(float(float_str))
    return {'keypoints': keypoint_vals}

### ------------------ Step 3: Transform Hand ------------------ ###
def transform_keypoints(hand_coords):
    knuckle_points = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][-1])
    translated_coords = copy(hand_coords) - hand_coords[0]

    index_coord = translated_coords[knuckle_points[0]]
    pinky_coord = translated_coords[knuckle_points[1]]
    
    palm_normal = normalize_vector(np.cross(index_coord, pinky_coord))   # Z
    palm_direction = normalize_vector(index_coord + pinky_coord)         # Y
    cross_product = normalize_vector(np.cross(palm_direction, palm_normal))  # X

    coord_frame = [cross_product, palm_direction, palm_normal]
    rotation_matrix = np.linalg.solve(coord_frame, np.eye(3)).T
    transformed_coords = (rotation_matrix @ translated_coords.T).T

    # Get wrist frame
    origin = hand_coords[0]
    hand_frame = [origin, cross_product, palm_normal, palm_direction]  # 4x3
    return transformed_coords, np.array(hand_frame)

### ------------------ Step 4: Convert to Robot Frame ------------------ ###
def to_robot_frame(init_affine, current_affine):
    H_V_des = pinv(init_affine) @ current_affine
    relative_affine_rot = (pinv(H_R_V) @ H_V_des @ H_R_V)[:3, :3]
    relative_affine_trans = (pinv(H_R_V_star) @ H_V_des @ H_R_V_star)[:3, 3]
    relative_affine = np.block(
        [[relative_affine_rot, relative_affine_trans.reshape(3, 1)], [0, 0, 0, 1]]
    )
    return relative_affine

### ------------------ Step 5: Visualize ------------------ ###
def create_arrow(origin, direction, color):
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.003, cone_radius=0.006,
        cylinder_height=0.02, cone_height=0.01
    )
    R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, 0])
    arrow.rotate(R, center=(0, 0, 0))
    arrow.paint_uniform_color(color)
    arrow.translate(origin + direction * 0.01)
    return arrow

vis_objs = []
robot_traj = []
frames = []
init_hand_affine = None

for frame in data[:]:
    info = _extract_data_from_token(frame['raw_keypoints'])
    hand_coords = np.array(info['keypoints'][1:]).reshape(-1, 3)
    transformed_coords, hand_frame = transform_keypoints(hand_coords)
    origin = hand_frame[0]  # wrist coordinate
    x_dir, y_dir, z_dir = hand_frame[1:4]

    # save for visualization
    frames.append(transformed_coords)

    H = np.eye(4)
    H[:3, 0] = x_dir
    H[:3, 1] = y_dir
    H[:3, 2] = z_dir
    H[:3, 3] = origin
    if init_hand_affine is None:
        init_hand_affine = H   # which is self.hand_moving_H
    print(init_hand_affine)
    print(H)
    print("---------")
    relative_affine = to_robot_frame(init_hand_affine, H)
    print(relative_affine)
    robot_traj.append(relative_affine[:3, 3])


os.makedirs("./data", exist_ok=True)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([-0.5, 0.5])
ax.set_title("Fingertip Trajectory")


sc = ax.scatter([], [], [], c='r', s=5)

#
home_pose = np.array([0.47, 0.04, 0.37])
home_sc = ax.scatter([], [], [], c='b', s=50, label='Home Pose')

robot_traj = np.array(robot_traj)
line, = ax.plot([], [], [], 'm-', lw=2)

def update(i):
    coords = frames[i]
    sc._offsets3d = (coords[:, 0], coords[:, 1], coords[:, 2])
    if i > 0:
        line.set_data(robot_traj[:i, 0], robot_traj[:i, 1])
        line.set_3d_properties(robot_traj[:i, 2])
    home_sc._offsets3d = ([home_pose[0]], [home_pose[1]], [home_pose[2]])
    return sc, line, home_sc

# 保存为 mp4
writer = FFMpegWriter(fps=15, metadata=dict(artist='Zhuoran'), bitrate=1800)
with writer.saving(fig, "./data/raw_fingertip.mp4", dpi=100):
    for i in range(len(frames)):
        update(i)
        writer.grab_frame()

print("✅ Finihsh:./data/raw_fingertip.mp4")
