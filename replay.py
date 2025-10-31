import threading
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer

arm = ArmReplayer("data/demonstration_0")
hand = HandReplayer("data/demonstration_0") 

arm_thread = threading.Thread(target=arm.replay, daemon=True)
hand_thread = threading.Thread(target=hand.replay, daemon=True)

arm_thread.start()
hand_thread.start()

arm_thread.join()
hand_thread.join()
print("Both replays finished.")
