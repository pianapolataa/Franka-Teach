# #!/usr/bin/env python3
# import os
# import pickle
# import time
# import numpy as np
# from tqdm import tqdm
# from pathlib import Path

# from frankateach.messages import FrankaAction

# def convert_processed_to_replay(processed_pkl_path, replay_folder, time_scale=3.0):
#     """
#     Convert processed BAKU-style data into replayable PKLs using timestamps from the processed file.
#     time_scale > 1 makes the replay slower (e.g. 3.0 = 3× slower).
#     """
#     os.makedirs(replay_folder, exist_ok=True)

#     # Load processed BAKU data
#     with open(processed_pkl_path, "rb") as f:
#         data = pickle.load(f)

#     obs = data["observations"]
#     timestamps = np.array(data["timestamps"], dtype=np.float64)
#     print(f"Loaded processed data with {len(obs)} frames")

#     # Scale timestamps
#     start_time = time.time()
#     ts_scaled = start_time + time_scale * (timestamps - timestamps[0])

#     # ----------------------------
#     # Arm replay data
#     # ----------------------------
#     arm_entries = []
#     for i, o in enumerate(tqdm(obs, desc="Preparing arm data")):
#         cmd = o["commanded_cartesian_states"]  # 7D: [x, y, z, qx, qy, qz, qw]
#         pos = np.array(cmd[:3], dtype=np.float32)
#         quat = np.array(cmd[3:7], dtype=np.float32)

#         action = FrankaAction(
#             pos=pos,
#             quat=quat,
#             gripper=-1,
#             reset=False,
#             timestamp=ts_scaled[i],
#         )

#         arm_entries.append({
#             "timestamp": action.timestamp,
#             "state": action
#         })

#     with open(Path(replay_folder) / "commanded_states.pkl", "wb") as f:
#         pickle.dump(arm_entries, f)
#     print(f"Saved arm replay data → {replay_folder}/commanded_states.pkl")

#     # ----------------------------
#     # Hand replay data
#     # ----------------------------
#     hand_entries = []
#     for i, o in enumerate(tqdm(obs, desc="Preparing hand data")):
#         state = np.array(o["gripper_states"], dtype=np.float32)
#         hand_entries.append({
#             "timestamp": ts_scaled[i],
#             "state": state
#         })

#     with open(Path(replay_folder) / "ruka_commanded_states.pkl", "wb") as f:
#         pickle.dump(hand_entries, f)
#     print(f"Saved hand replay data → {replay_folder}/ruka_commanded_states.pkl")


# if __name__ == "__main__":
#     processed_pkl_path = "processed_data_pkl/demo_task.pkl"
#     replay_folder = "replay_ready/demo_task"

#     # Adjust time_scale for slower/faster replay
#     convert_processed_to_replay(processed_pkl_path, replay_folder, time_scale=3.0)
import pickle

with open("processed_data_pkl/demo_task.pkl", "rb") as f:
    data = pickle.load(f)

print(data.keys())
print("First observation keys:", data["observations"][0].keys())
