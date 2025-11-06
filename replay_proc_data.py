import os
import pickle
import time
import numpy as np
from tqdm import tqdm
from pathlib import Path

from frankateach.messages import FrankaAction

def convert_processed_to_replay(processed_pkl_path, replay_folder, arm_hz=30, hand_hz=60):
    os.makedirs(replay_folder, exist_ok=True)

    with open(processed_pkl_path, "rb") as f:
        data = pickle.load(f)

    obs = data["observations"]
    print(f"Loaded processed data with {len(obs)} frames")

    # Synthetic timestamps based on assumed frequency
    start_time = time.time()
    arm_dt = 1.0 / arm_hz
    hand_dt = 1.0 / hand_hz

    # Arm replay data
    arm_entries = []
    for i, o in enumerate(tqdm(obs, desc="Preparing arm data")):
        cmd = o["commanded_cartesian_states"]  # 7D: [x,y,z,qx,qy,qz,qw]
        pos = np.array(cmd[:3], dtype=np.float32)
        quat = np.array(cmd[3:7], dtype=np.float32)

        action = FrankaAction(
            pos=pos,
            quat=quat,
            gripper=-1,
            reset=False,
            timestamp=start_time + i * arm_dt,
        )

        arm_entries.append({
            "timestamp": action.timestamp,
            "state": action
        })

    with open(Path(replay_folder) / "commanded_states.pkl", "wb") as f:
        pickle.dump(arm_entries, f)
    print(f"Saved arm replay data → {replay_folder}/commanded_states.pkl")

    # Hand replay data
    hand_entries = []
    for i, o in enumerate(tqdm(obs, desc="Preparing hand data")):
        # Hand state (shape should match ruka format, 16 or so joints)
        state = np.array(o["gripper_states"], dtype=np.float32)
        t = start_time + i * hand_dt
        hand_entries.append({
            "timestamp": t,
            "state": state
        })

    with open(Path(replay_folder) / "ruka_commanded_states.pkl", "wb") as f:
        pickle.dump(hand_entries, f)
    print(f"Saved hand replay data → {replay_folder}/ruka_commanded_states.pkl")


if __name__ == "__main__":
    processed_pkl_path = "processed_data_pkl/demo_task.pkl"
    replay_folder = "replay_ready/demo_task"
    convert_processed_to_replay(processed_pkl_path, replay_folder)
