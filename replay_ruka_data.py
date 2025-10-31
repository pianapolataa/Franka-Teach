import pickle
import time
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

class HandReplayer:
    def __init__(self, folder: str):
        self.folder = folder
        ruka_cmds_path = f"{folder}/ruka_commanded_states.pkl"

        # Load commanded states
        with open(ruka_cmds_path, "rb") as f:
            self.ruka_cmds = pickle.load(f)

        print(f"Loaded {len(self.ruka_cmds)} frames.")

        # Initialize Ruka hand
        self.hand = Hand(hand_type="right")
        self.curr_pos = self.hand.read_pos()
        time.sleep(0.5)
        test_pos = self.hand.tensioned_pos
        move_to_pos(curr_pos=self.curr_pos, des_pos=test_pos, hand=self.hand, traj_len=50)
        time.sleep(1)
        self.curr_pos = self.hand.read_pos()

    def replay(self):
        """Replay hand actions respecting original timestamps."""
        if not self.ruka_cmds:
            print("No hand commands to replay.")
            return

        start_timestamp = self.ruka_cmds[0]['timestamp']
        replay_start = time.time()

        for i, entry in enumerate(self.ruka_cmds):
            target_pos = entry['state']
            original_timestamp = entry['timestamp']

            # Compute sleep time to match original timing
            target_time = replay_start + (original_timestamp - start_timestamp)
            sleep_time = max(0, target_time - time.time())
            time.sleep(sleep_time)

            # Move hand
            move_to_pos(curr_pos=self.curr_pos, des_pos=target_pos, hand=self.hand, traj_len=25)
            self.curr_pos = self.hand.read_pos()

            print(f"Replayed frame {i+1}/{len(self.ruka_cmds)}")

        self.hand.close()
        print("Replay finished.")
