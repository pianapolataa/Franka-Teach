# record_oculus_data.py
import pickle
import zmq
from frankateach.constants import *
from frankateach.constants import *
from frankateach.utils import FrequencyTimer
from frankateach.network import create_pull_socket, ZMQKeypointPublisher, ZMQKeypointSubscriber
from frankateach.utils import parse_controller_state, notify_component_start
import zmq



raw_keypoint_socket = create_pull_socket(INTERNAL_IP, 8087)
button_keypoint_socket = create_pull_socket(INTERNAL_IP, 8095)
teleop_reset_socket = create_pull_socket(INTERNAL_IP, 8100)
        

recorded_data = []

try:
    while True:
        raw_keypoints = raw_keypoint_socket.recv()
        button_feedback = button_keypoint_socket.recv()
        pause_status = teleop_reset_socket.recv()

        print(raw_keypoints)
        print(button_feedback)
        print(pause_status)
        print()
        recorded_data.append({
            "raw_keypoints": raw_keypoints,
            "button_feedback": button_feedback,
            "pause_status": pause_status
        })
except KeyboardInterrupt:
    print("Stopped recording.")

# Save to file
with open("oculus_data_1.pkl", "wb") as f:
    pickle.dump(recorded_data, f)
