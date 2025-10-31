import threading
import time
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer

arm = ArmReplayer("data/demonstration_0")
hand = HandReplayer("data/demonstration_0") 

# common start time
replay_start = time.time()

arm_thread = threading.Thread(target=lambda: arm.replay(replay_start), daemon=True)
hand_thread = threading.Thread(target=lambda: hand.replay(replay_start), daemon=True)

arm_thread.start()
hand_thread.start()

arm_thread.join()
hand_thread.join()
print("Both replays finished.")
