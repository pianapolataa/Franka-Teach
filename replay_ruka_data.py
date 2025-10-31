# import pickle
# import time
# import numpy as np
# from ruka_hand.control.hand import Hand
# from ruka_hand.utils.trajectory import move_to_pos

# # Paths to recorded files
# folder = "demonstration_0"
# ruka_states_path = f"{folder}/ruka_states.pkl"
# ruka_cmds_path = f"{folder}/ruka_commanded_states.pkl"

# # Load data
# with open(ruka_states_path, "rb") as f:
#     ruka_states = pickle.load(f)
# with open(ruka_cmds_path, "rb") as f:
#     ruka_cmds = pickle.load(f)

# print(f"Loaded {len(ruka_cmds)} frames.")

# # Initialize Ruka hand
# hand = Hand(hand_type="right")
# curr_pos = hand.read_pos()

# # Replay loop
# for i, target_pos in enumerate(ruka_cmds):
#     print(target_pos)
#     move_to_pos(curr_pos=curr_pos, des_pos=target_pos, hand=hand, traj_len=25)
#     curr_pos = hand.read_pos()
#     print(f"Replayed frame {i+1}/{len(ruka_cmds)}")
#     time.sleep(0.05)  # match original teleop rate (~20 Hz)

# hand.close()
# print("Replay finished.")

import pickle
import time
import numpy as np
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

# Paths to recorded files
folder = "demonstration_0"
ruka_states_path = f"{folder}/ruka_states.pkl"
ruka_cmds_path = f"{folder}/ruka_commanded_states.pkl"

# Load data
with open(ruka_states_path, "rb") as f:
    ruka_states = pickle.load(f)
with open(ruka_cmds_path, "rb") as f:
    ruka_cmds = pickle.load(f)

print(f"Loaded {len(ruka_cmds)} frames.")

# Replay loop
for i, target_pos in enumerate(ruka_cmds):
    print(target_pos)

print("Replay finished.")
