import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FFMpegWriter

def extract_hand_frames(data):

    frames = []
    for frame_data in data:
        b_str = frame_data['raw_keypoints']
        decoded = b_str.decode()
        coord_strs = decoded.split(":")[1].strip().split("|")
        points = [list(map(float, p.split(","))) for p in coord_strs if len(p.strip()) > 0]
        frames.append(np.array(points))
    return frames  # List of (num_points, 3)

def extract_wrist_track(frames):

    wrist_positions = [frame[0] for frame in frames]  # 每帧第一个点为 wrist
    return np.array(wrist_positions)  # shape: (num_frames, 3)


with open("oculus_data.pkl", "rb") as f:
    data = pickle.load(f)

frames = extract_hand_frames(data)
wrist_positions = extract_wrist_track(frames)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)

sc = ax.scatter([], [], [], c='r')  # 所有关键点
wrist_line, = ax.plot([], [], [], c='b', label='Wrist Path')  # wrist 轨迹

def update(frame_idx):
    frame = frames[frame_idx]
    sc._offsets3d = (frame[:, 0], frame[:, 1], frame[:, 2])

    wrist_so_far = wrist_positions[:frame_idx + 1]
    wrist_line.set_data(wrist_so_far[:, 0], wrist_so_far[:, 1])
    wrist_line.set_3d_properties(wrist_so_far[:, 2])
    return sc, wrist_line


writer = FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
with writer.saving(fig, "./data/frame_standard.mp4", dpi=100):
    for i in range(len(frames)):
        update(i)
        writer.grab_frame()
