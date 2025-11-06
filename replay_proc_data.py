import os
import pickle
import threading
from pathlib import Path
import time

# Import your existing replayers
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer


def save_for_replay(processed_pkl_path, replay_folder):
    """Convert processed .pkl data into replayable format."""
    with open(processed_pkl_path, "rb") as f:
        data = pickle.load(f)

    os.makedirs(replay_folder, exist_ok=True)

    # Prepare arm replay file
    arm_data = [
        {"timestamp": t, "state": a} for t, a in zip(data["timestamps"], data["arm"])
    ]
    with open(Path(replay_folder) / "commanded_states.pkl", "wb") as f:
        pickle.dump(arm_data, f)
    print(f"✅ Saved arm replay file with {len(arm_data)} frames.")

    # Prepare hand replay file
    hand_data = [
        {"timestamp": t, "state": h} for t, h in zip(data["timestamps"], data["hand"])
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


def replay_processed_demo(processed_pkl_path, replay_folder="replay_from_processed"):
    # Step 1: Convert to replay format
    save_for_replay(processed_pkl_path, replay_folder)

    # Step 2: Launch replay threads
    arm_thread = threading.Thread(target=run_arm, args=(replay_folder,), daemon=True)
    hand_thread = threading.Thread(target=run_hand, args=(replay_folder,), daemon=True)

    print("Starting both replays...")
    arm_thread.start()
    hand_thread.start()

    arm_thread.join()
    hand_thread.join()

    print("✅ Both replays finished successfully.")


if __name__ == "__main__":
    processed_pkl_path = "processed_data_pkl/demo_task.pkl"
    replay_processed_demo(processed_pkl_path)
