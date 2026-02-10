import threading
import time
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer

# === Initialize first ===
print("[Init] Initializing ArmReplayer...")
arm = ArmReplayer("replay_ready/demo_task")
print("[Init] Initializing HandReplayer...")
# hand = HandReplayer("replay_ready/demo_task")
print("[Init] Both replayers initialized. Starting replays...")

def run_arm():
    arm.replay()

def run_hand():
    hand.replay()

arm_thread = threading.Thread(target=run_arm, daemon=True)
hand_thread = threading.Thread(target=run_hand, daemon=True)

# === Start both simultaneously ===
arm_thread.start()
# hand_thread.start()

arm_thread.join()
# hand_thread.join()

print("Both replays finished.")