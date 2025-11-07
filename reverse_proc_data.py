#!/usr/bin/env python3
import os
import pickle
import time
import numpy as np
from tqdm import tqdm
from pathlib import Path
from scipy.spatial.transform import Rotation as R, Slerp

from frankateach.messages import FrankaAction

def convert_processed_to_replay(processed_pkl_path, replay_folder, time_scale=1.0, interp_frames=5):
    """
    Convert processed BAKU-style data into replayable PKLs using timestamps from the processed file.
    Interpolates ARM states/commands with `interp_frames` frames between every two frames.
    time_scale > 1 makes the replay slower.
    """
    os.makedirs(replay_folder, exist_ok=True)

    # Load processed BAKU data
    with open(processed_pkl_path, "rb") as f:
        data = pickle.load(f)

    obs = data["observations"]
    timestamps = np.array(data["timestamps"], dtype=np.float64)
    print(f"Loaded processed data with {len(obs)} frames")

    # ----------------------------
    # Interpolate ARM states and commands
    # ----------------------------
    arm_entries = []
    for i in tqdm(range(len(obs)-1), desc="Preparing interpolated arm data"):
        o1 = obs[i]
        o2 = obs[i+1]
        t1 = timestamps[i]
        t2 = timestamps[i+1]

        # Original frame
        action1 = FrankaAction(
            pos=o1["commanded_cartesian_states"][:3],
            quat=o1["commanded_cartesian_states"][3:],
            gripper=-1,
            reset=False,
            timestamp=t1
        )
        arm_entries.append({"timestamp": t1, "state": action1})

        # Interpolated frames
        pos1, quat1 = np.array(o1["commanded_cartesian_states"][:3], dtype=np.float32), np.array(o1["commanded_cartesian_states"][3:], dtype=np.float32)
        pos2, quat2 = np.array(o2["commanded_cartesian_states"][:3], dtype=np.float32), np.array(o2["commanded_cartesian_states"][3:], dtype=np.float32)

        r = R.from_quat([quat1, quat2])
        slerp = Slerp([0, 1], r)

        for j in range(1, interp_frames+1):
            alpha = j / (interp_frames + 1)  # fraction along the interval
            interp_pos = (1 - alpha) * pos1 + alpha * pos2
            interp_quat = slerp(alpha).as_quat()
            interp_time = t1 + alpha * (t2 - t1)

            action_interp = FrankaAction(
                pos=interp_pos,
                quat=interp_quat,
                gripper=-1,
                reset=False,
                timestamp=interp_time
            )
            arm_entries.append({"timestamp": interp_time, "state": action_interp})

    # Append last original frame
    last = obs[-1]
    action_last = FrankaAction(
        pos=last["commanded_cartesian_states"][:3],
        quat=last["commanded_cartesian_states"][3:],
        gripper=-1,
        reset=False,
        timestamp=timestamps[-1]
    )
    arm_entries.append({"timestamp": timestamps[-1], "state": action_last})

    # Scale timestamps
    start_time = time.time()
    for entry in arm_entries:
        entry["timestamp"] = start_time + time_scale * (entry["timestamp"] - timestamps[0])

    with open(Path(replay_folder) / "commanded_states.pkl", "wb") as f:
        pickle.dump(arm_entries, f)
    print(f"Saved arm replay data → {replay_folder}/commanded_states.pkl")

    # ----------------------------
    # Hand replay data (unchanged)
    # ----------------------------
    hand_entries = []
    for i, o in enumerate(tqdm(obs, desc="Preparing hand data")):
        state = np.array(o["gripper_states"], dtype=np.float32)
        hand_entries.append({
            "timestamp": start_time + time_scale * (timestamps[i] - timestamps[0]),
            "state": state
        })

    with open(Path(replay_folder) / "ruka_commanded_states.pkl", "wb") as f:
        pickle.dump(hand_entries, f)
    print(f"Saved hand replay data → {replay_folder}/ruka_commanded_states.pkl")


if __name__ == "__main__":
    processed_pkl_path = "processed_data_pkl/demo_task.pkl"
    replay_folder = "replay_ready/demo_task"

    # Adjust time_scale for slower/faster replay
    convert_processed_to_replay(processed_pkl_path, replay_folder, time_scale=3.0, interp_frames=5)
