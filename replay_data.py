import pickle
import time
import threading
from pathlib import Path
from copy import deepcopy as copy
import numpy as np

from frankateach.franka_server import Robot
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

# ---- Constants ----
TRAJ_LEN = 25
REPLAY_RATE = 0.05  # ~20 Hz


def load_pickle(path):
    with open(path, "rb") as f:
        return pickle.load(f)


class ReplayOperator:
    def __init__(self, demo_folder):
        folder = Path(demo_folder)
        print(f"Loading demo from: {folder}")

        self.franka_cmds = load_pickle(folder / "commanded_states.pkl")
        self.ruka_cmds = load_pickle(folder / "ruka_commanded_states.pkl")
        self.n_frames = min(len(self.franka_cmds), len(self.ruka_cmds))

        print(f"Loaded {self.n_frames} frames.")

        # Initialize hardware
        print("Initializing Franka robot...")
        self.robot = Robot()
        print("Initializing Ruka hand...")
        self.hand = Hand(hand_type="right")
        self.curr_hand_pos = self.hand.read_pos()

        # Threads and stop event
        self.run_event = threading.Event()
        self.run_event.set()
        self.arm_thread = threading.Thread(target=self.replay_arm, daemon=True)
        self.ruka_thread = threading.Thread(target=self.replay_ruka, daemon=True)

        # Frame index
        self.frame_idx = 0

    def start(self):
        print("Starting replay threads...")
        self.arm_thread.start()
        self.ruka_thread.start()
        try:
            while self.frame_idx < self.n_frames and self.run_event.is_set():
                time.sleep(0.01)
                self.frame_idx += 1
        except KeyboardInterrupt:
            print("Stopping replay...")
            self.run_event.clear()
            self.arm_thread.join()
            self.ruka_thread.join()
            self.hand.close()
            print("Replay stopped.")

    def replay_arm(self):
        while self.run_event.is_set() and self.frame_idx < self.n_frames:
            try:
                arm_cmd = self.franka_cmds[self.frame_idx]
                # If your commanded_states.pkl contains FrankaAction objects
                self.robot.move_to_pose(arm_cmd.pos, arm_cmd.quat)
            except Exception as e:
                print(f"[WARN] Franka command failed at frame {self.frame_idx}: {e}")
            time.sleep(REPLAY_RATE)

    def replay_ruka(self):
        while self.run_event.is_set() and self.frame_idx < self.n_frames:
            try:
                hand_cmd = self.ruka_cmds[self.frame_idx]
                move_to_pos(curr_pos=self.curr_hand_pos, des_pos=hand_cmd, hand=self.hand, traj_len=TRAJ_LEN)
                self.curr_hand_pos = self.hand.read_pos()
            except Exception as e:
                print(f"[WARN] Ruka command failed at frame {self.frame_idx}: {e}")
            time.sleep(REPLAY_RATE)


if __name__ == "__main__":
    demo_folder = "path/to/demonstration_0"
    replay = ReplayOperator(demo_folder)
    replay.start()
