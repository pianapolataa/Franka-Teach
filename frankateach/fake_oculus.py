# fake_oculus_sender.py
import zmq
import time
import pickle
from frankateach.constants import *

# Load data
with open("oculus_data.pkl", "rb") as f:
    data = pickle.load(f)

context = zmq.Context()

socket_raw = context.socket(zmq.PUSH)
socket_raw.connect(f"tcp://{INTERNAL_IP}:8087")

socket_button = context.socket(zmq.PUSH)
socket_button.connect(f"tcp://{INTERNAL_IP}:8095")

socket_pause = context.socket(zmq.PUSH)
socket_pause.connect(f"tcp://{INTERNAL_IP}:8100")

while True:
    for msg in data:
        socket_raw.send(msg["raw_keypoints"])
        socket_button.send(msg["button_feedback"])
        socket_pause.send(msg["pause_status"])
        print(msg)
        time.sleep(1 / 30)  # Match your VR frequency (e.g., 30Hz)
