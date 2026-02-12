# fake_oculus_sender.py
import zmq
import time
import pickle
import numpy as np
from frankateach.constants import *

context = zmq.Context()

# Using 127.0.0.1 if testing on the same machine as the relay
# or keep INTERNAL_IP if you want to test the network card
TARGET_IP = INTERNAL_IP

socket_raw = context.socket(zmq.PUSH)
socket_raw.connect(f"tcp://{TARGET_IP}:8087")

socket_button = context.socket(zmq.PUSH)
socket_button.connect(f"tcp://{TARGET_IP}:8095")

socket_pause = context.socket(zmq.PUSH)
socket_pause.connect(f"tcp://{TARGET_IP}:8100")

print(f"Starting fake sender. Sending to {TARGET_IP}...")
dummy_keypoints = np.ones(21 * 3).astype(np.float32).tobytes() 
dummy_button = pickle.dumps({"button_pressed": True}, protocol=-1)
dummy_pause = pickle.dumps({"paused": False}, protocol=-1)

count = 0
while True:
    try:
        # Send the dummy data
        socket_raw.send(dummy_keypoints)
        socket_button.send(dummy_button)
        socket_pause.send(dummy_pause)
        
        count += 1
        print(f"Sent packet batch {count}")
        
        time.sleep(1 / 30)  # 30Hz
    except KeyboardInterrupt:
        break

print("Fake sender stopped.")