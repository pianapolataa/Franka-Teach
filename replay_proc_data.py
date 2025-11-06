import os
import pickle
import threading
import time
from pathlib import Path

# Use your existing replayers
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer


def save_for_replay(processed_pkl_path, replay_folder, hz=30.0):
    """
    Convert processed BAKU-format .pkl into replayable format for ArmReplayer/HandReplayer.
    Synthesizes timestamps assuming 'hz' sampling rate.
    """
    with open(processed_pkl_path, "rb") as f:
        data = pickle.load(f)

    observations = data["observations"]
    os.makedirs(replay_folder, exist_ok=True)

    # Build synthetic timestamps (e.g. 30Hz)
    timestamps = [i / hz for i in range(len(observations))]

    # Arm commanded poses
    arm_data = [
        {"timestamp": t, "state": obs["commanded_cartesian_states"]}
        for t, obs in zip(timestamps, observations)
    ]
    with open(Path(replay_folder) / "commanded_states.pkl", "wb") as f:
        pickle.dump(arm_data, f)
    print(f"✅ Saved arm replay file with {len(arm_data)} frames.")

    # Hand commanded states
    hand_data = [
        {"timestamp": t, "state": obs["commanded_gripper_states"]}
        for t, obs in zip(timestamps, observations)
    ]
    with open(Path(replay_folder) / "ruka_commanded_states.pkl", "wb") as f:
        pickle.dump(hand_data, f)
    print(f"✅ Saved hand replay file with {len(hand_data)} frames.")


def run_arm(folder):
    arm = ArmReplayer(folder)
    arm.replay()


def run_hand(folder):
    hand = HandReplayer(folder)
    hand.replay()


def replay_processed_demo(processed_pkl_path, replay_folder="replay_from_processed", hz=30.0):
    """Prepare replay files and start both replayers in parallel."""
    save_for_replay(processed_pkl_path, replay_folder, hz=hz)

    print("🚀 Starting both replays...")
    arm_thread = threading.Thread(target=run_arm, args=(replay_folder,), daemon=True)
    hand_thread = threading.Thread(target=run_hand, args=(replay_folder,), daemon=True)

    arm_thread.start()
    hand_thread.start()

    arm_thread.join()
    hand_thread.join()

    print("✅ Both replays finished successfully.")


if __name__ == "__main__":
    processed_pkl_path = "processed_data_pkl/demo_task.pkl"
    replay_processed_demo(processed_pkl_path, hz=30.0)
