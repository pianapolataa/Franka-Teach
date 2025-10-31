import pickle
import time
from pathlib import Path

from frankateach.messages import FrankaAction
from frankateach.network import create_request_socket
from frankateach.constants import LOCALHOST, CONTROL_PORT


class ArmReplayer:
    def __init__(self, storage_path):
        self.storage_path = Path(storage_path)
        self.load_data()
        # Socket to send actions to the Franka server
        self.action_socket = create_request_socket(LOCALHOST, CONTROL_PORT)

    def load_data(self):
        # Load only commanded actions
        with open(self.storage_path / "commanded_states.pkl", "rb") as f:
            self.arm_actions = pickle.load(f)

    def replay(self, delay=0.04):
        """Replay arm actions sequentially. 'delay' is seconds between frames."""
        print("Starting arm replay...")
        for action in self.arm_actions:
            self.action_socket.send(pickle.dumps(action))
            _ = self.action_socket.recv()
            time.sleep(delay)

        print("Replay finished.")
        self.action_socket.close()


if __name__ == "__main__":
    # CHANGE THIS PATH to your demonstration folder
    demo_path = "data/demonstration_0"

    replayer = ArmReplayer(demo_path)
    replayer.replay()
