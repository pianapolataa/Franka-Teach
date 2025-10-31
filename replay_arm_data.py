import pickle
import time
from pathlib import Path
import numpy as np

from frankateach.messages import FrankaAction
from frankateach.network import create_request_socket
from frankateach.constants import LOCALHOST, CONTROL_PORT


class ArmReplayer:
    def __init__(self, storage_path):
        self.storage_path = Path(storage_path)
        self.load_data()
        # Socket to send actions to the Franka server
        self.action_socket = create_request_socket(LOCALHOST, CONTROL_PORT)

        # Reset robot at the start
        reset_action = FrankaAction(
            pos=np.zeros(3),
            quat=np.zeros(4),
            gripper=-1,
            reset=True,
            timestamp=time.time(),
        )
        self.action_socket.send(bytes(pickle.dumps(reset_action, protocol=-1)))            
        _ = pickle.loads(self.action_socket.recv())

    def load_data(self):
        # Load commanded states with timestamps
        with open(self.storage_path / "commanded_states.pkl", "rb") as f:
            self.arm_actions = pickle.load(f)

    def replay(self, replay_start):
        """Replay arm actions respecting original timestamps."""
        print("Starting arm replay...")

        if not self.arm_actions:
            print("No actions to replay.")
            return

        # Start from first timestamp
        start_time = self.arm_actions[0]['timestamp']

        for entry in self.arm_actions:
            action = entry['state']
            original_timestamp = entry['timestamp']

            # Calculate how long to wait based on original timing
            target_time = replay_start + (original_timestamp - start_time)
            sleep_time = max(0, target_time - time.time())
            time.sleep(sleep_time)

            # Send action
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            _ = self.action_socket.recv()

        print("Replay finished.")
        self.action_socket.close()


if __name__ == "__main__":
    demo_path = "data/demonstration_0"

    replayer = ArmReplayer(demo_path)
    replayer.replay()
