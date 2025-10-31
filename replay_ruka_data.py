import pickle
import time
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

# Paths to recorded files
folder = "data/demonstration_0"
ruka_cmds_path = f"{folder}/ruka_commanded_states.pkl"

# Load commanded states
with open(ruka_cmds_path, "rb") as f:
    ruka_cmds = pickle.load(f)

print(f"Loaded {len(ruka_cmds)} frames.")

# Initialize Ruka hand
hand = Hand(hand_type="right")
curr_pos = hand.read_pos()

if ruka_cmds:
    start_timestamp = ruka_cmds[0]['timestamp']
    replay_start = time.time()

    # Replay loop with original timestamps
    for i, entry in enumerate(ruka_cmds):
        target_pos = entry['state']
        original_timestamp = entry['timestamp']

        # Compute sleep time to match original timing
        target_time = replay_start + (original_timestamp - start_timestamp)
        sleep_time = max(0, target_time - time.time())
        time.sleep(sleep_time)

        # Move hand
        move_to_pos(curr_pos=curr_pos, des_pos=target_pos, hand=hand, traj_len=25)
        curr_pos = hand.read_pos()

        print(f"Replayed frame {i+1}/{len(ruka_cmds)}")

hand.close()
print("Replay finished.")