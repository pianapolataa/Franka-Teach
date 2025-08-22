import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D
import ast
# Load data from txt file

def get_relative(data_path, video_name):
    positions = []
    rotations = []
    buffer = []

    with open(data_path, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue

        # Try Format-1
        if '|' in line:
            try:
                pos_str, rot_str = line.split('|')
                pos = np.array(ast.literal_eval(pos_str.strip()))
                rot = np.array(ast.literal_eval(rot_str.strip()))
                if rot.shape == (3, 3) and pos.shape == (3,):
                    positions.append(pos)
                    rotations.append(rot)
                    i += 1
                    continue  
                else:
                    print(f"⚠️ Format-1 shape invalid: pos={pos.shape}, rot={rot.shape}")
            except Exception:
                pass  # If fails, try Format-2

            if i + 2 < len(lines):
                try:
                    line1 = lines[i].strip()
                    line2 = lines[i + 1].strip()
                    line3 = lines[i + 2].strip()

                    pos_str, rot0_part = line1.split('|')
                    pos = np.array(ast.literal_eval(pos_str.strip()))

                    rot_str_full = "[" + \
                        rot0_part.strip().lstrip('[').rstrip(',') + "," + \
                        line2.rstrip(',') + "," + \
                        line3.rstrip(']') + "]"

                    rot = np.array(ast.literal_eval(rot_str_full))
                    if rot.shape == (3, 3) and pos.shape == (3,):
                        positions.append(pos)
                        rotations.append(rot)
                    else:
                        print(f"⚠️ Format-2 shape invalid: pos={pos.shape}, rot={rot.shape}")
                    i += 3 
                    continue
                except Exception as e:
                    print(f"❌ Error: Failed to parse both formats at line {i+1}:\n{lines[i:i+3]}\n{e}")
                    i += 3  #
            else:
                print(f"❌ Error: Incomplete data block starting at line {i+1}")
                break
        else:
            i += 1


    # 最终转换为 numpy array
    rotations = np.array(rotations)   # shape: (N, 2, 3, 3)
    positions = np.array(positions)   # shape: (N, 2, 3)

    print("✅ Loaded:", len(rotations), "matrix pairs.")

    # Set up the figure
    x_min, x_max = positions[:, 0].min(), positions[:, 0].max()
    y_min, y_max = positions[:, 1].min(), positions[:, 1].max()
    z_min, z_max = positions[:, 2].min(), positions[:, 2].max()

    # 设置 figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 设置坐标范围并稍微加点 margin
    margin = 0.02
    ax.set_xlim([x_min - margin, x_max + margin])
    ax.set_ylim([y_min - margin, y_max + margin])
    ax.set_zlim([z_min - margin, z_max + margin])
    ax.set_title("Robot Target Pose Visualization")
    ax.set_title("Robot Target Pose Visualization")

    # Home pose
    home_pose = np.array([0.47, 0.04, 0.37])
    home_sc = ax.scatter([home_pose[0]], [home_pose[1]], [home_pose[2]], c='b', s=60, label='Home Pose')

    sc = ax.scatter([], [], [], c='r', s=20)
    line, = ax.plot([], [], [], 'm-', lw=2)

    quiver_handles = []

    def update(frame_idx):
        # global quiver_handles
        # # Clear previous quivers
        # for q in quiver_handles:
        #     q.remove()
        quiver_handles = []

        pos = positions[frame_idx]
        rot = rotations[frame_idx]
        print(pos)
        # Update scatter
        sc._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
        line.set_data(positions[:frame_idx+1, 0], positions[:frame_idx+1, 1])
        line.set_3d_properties(positions[:frame_idx+1, 2])

        # Draw coordinate frame (x=red, y=green, z=blue)
        # length = 0.03
        # colors = ['r', 'g', 'b']
        # for i in range(3):
        #     dir_vec = rot[:, i] * length
        #     q = ax.quiver(pos[0], pos[1], pos[2],
        #                 dir_vec[0], dir_vec[1], dir_vec[2],
        #                 color=colors[i], linewidth=2)
        #     quiver_handles.append(q)

        return sc, line # *quiver_handles

    # Save as MP4
    writer = FFMpegWriter(fps=10, metadata=dict(artist='Zhuoran'), bitrate=1800)
    output_path = f"/home/jolia/vr-hand-tracking/Franka-Teach/frankateach/data/{video_name}"
    with writer.saving(fig, output_path, dpi=100):
        for i in range(len(positions)):
            update(i)
            writer.grab_frame()




    
def get_graph(data_path, video_name):
    # Load data from txt file
    positions = []
    rotations = []

    with open(data_path, 'r') as f:
        for line in f:
            try:
                pos_str, rot_str = line.strip().split('|')
                pos = np.array(ast.literal_eval(pos_str.strip()))
                rot = np.array(ast.literal_eval(rot_str.strip()))
                positions.append(pos)
                rotations.append(rot)
            except Exception as e:
                print("⚠️ Error parsing line:", line)
                print(e)


    positions = np.array(positions)
    rotations = np.array(rotations)

    # Set up the figure
    x_min, x_max = positions[:, 0].min(), positions[:, 0].max()
    y_min, y_max = positions[:, 1].min(), positions[:, 1].max()
    z_min, z_max = positions[:, 2].min(), positions[:, 2].max()

    # 设置 figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 设置坐标范围并稍微加点 margin
    margin = 0.02
    ax.set_xlim([x_min - margin, x_max + margin])
    ax.set_ylim([y_min - margin, y_max + margin])
    ax.set_zlim([z_min - margin, z_max + margin])
    ax.set_title("Robot Target Pose Visualization")

    # Home pose
    home_pose = np.array([0.47, 0.04, 0.37])
    home_sc = ax.scatter([home_pose[0]], [home_pose[1]], [home_pose[2]], c='b', s=60, label='Home Pose')

    sc = ax.scatter([], [], [], c='r', s=20)
    line, = ax.plot([], [], [], 'm-', lw=2)

    quiver_handles = []

    def update(frame_idx):
        # global quiver_handles
        # # Clear previous quivers
        # for q in quiver_handles:
        #     q.remove()
        quiver_handles = []

        pos = positions[frame_idx]
        rot = rotations[frame_idx]
        print(pos)
       
        # Update scatter
        sc._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
        line.set_data(positions[:frame_idx+1, 0], positions[:frame_idx+1, 1])
        line.set_3d_properties(positions[:frame_idx+1, 2])

        # Draw coordinate frame (x=red, y=green, z=blue)
        # length = 0.03
        # colors = ['r', 'g', 'b']
        # for i in range(3):
        #     dir_vec = rot[:, i] * length
        #     q = ax.quiver(pos[0], pos[1], pos[2],
        #                 dir_vec[0], dir_vec[1], dir_vec[2],
        #                 color=colors[i], linewidth=2)
        #     quiver_handles.append(q)

        return sc, line # *quiver_handles

    # Save as MP4
    writer = FFMpegWriter(fps=10, metadata=dict(artist='Zhuoran'), bitrate=1800)
    output_path = f"/home/jolia/vr-hand-tracking/Franka-Teach/frankateach/data/{video_name}"
    with writer.saving(fig, output_path, dpi=100):
        for i in range(len(positions)):
            update(i)
            writer.grab_frame()

    import os
    os.path.exists(output_path)

route_unity = "/home/jolia/vr-hand-tracking/Franka-Teach/frankateach/data/unity_relative_pose_log.txt"
relative_route = "/home/jolia/vr-hand-tracking/Franka-Teach/frankateach/data/robot_relative_pose_log.txt"
robot_route = "/home/jolia/vr-hand-tracking/Franka-Teach/frankateach/data/robot_target_pose_log.txt"
# get_target()
# get_relative(route_unity) # for unity data processing and robot data processsing
get_relative(route_unity, "unity_pose_visualization.mp4") # for robot target pose visualization
get_graph(robot_route, "robot_target_pose_visualization.mp4") # for robot target pose visualization