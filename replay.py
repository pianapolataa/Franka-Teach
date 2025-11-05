import threading
import time
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer

def run_arm():
    arm = ArmReplayer("data/demonstration_1")
    arm.replay()

def run_hand():
    hand = HandReplayer("data/demonstration_1")
    hand.replay()

arm_thread = threading.Thread(target=run_arm, daemon=True)
hand_thread = threading.Thread(target=run_hand, daemon=True)

arm_thread.start()
hand_thread.start()

arm_thread.join()
hand_thread.join()

print("Both replays finished.")