import zmq
import time

def run_relay():
    context = zmq.Context()

    # 1. Listen to the Oculus Quest (Port 8095)
    # This script BINDS here because only one thing can listen to 8095.
    quest_receiver = context.socket(zmq.PULL)
    quest_receiver.bind("tcp://*:8095")

    # 2. Forward to your Detectors (Ports 9095 and 9096)
    # This script CONNECTS to your teleop processes.
    right_hand_forwarder = context.socket(zmq.PUSH)
    right_hand_forwarder.connect("tcp://127.0.0.1:9095")

    left_hand_forwarder = context.socket(zmq.PUSH)
    left_hand_forwarder.connect("tcp://127.0.0.1:9096")

    print("Relay started: Quest(8095) -> Right(9095) & Left(9096)")

    while True:
        # Get data from Quest
        message = quest_receiver.recv()
        # Send to both internal ports
        right_hand_forwarder.send(message)
        left_hand_forwarder.send(message)

if __name__ == "__main__":
    run_relay()