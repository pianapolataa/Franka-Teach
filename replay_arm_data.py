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
        # Load the numpy array: expected shape (N, 7) 
        # [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
        self.arm_trajectory = np.load("arm_write_trajectory.npy")
        print(f"Loaded trajectory with {self.arm_trajectory.shape[0]} frames.")

    def replay(self):
        print("Starting arm replay with subsampling...")
        if self.arm_trajectory is None or len(self.arm_trajectory) == 0:
            print("No actions to replay.")
            return
        
        replay_data = self.arm_trajectory[100000::117]
        if len(replay_data) == 0:
            print("Error: No data left after trimming and subsampling.")
            return

        first = True
        
        for row in replay_data:
            if first:
                if row[0] < 0.46: 
                    continue
                else:
                    first = False
                    print("Threshold met! Replaying movement...")

            loop_start = time.time()
            
            target_pos = row[:3]
            target_quat = row[3:]
            
            action = FrankaAction(
                pos=target_pos.astype(np.float32),
                quat=target_quat.astype(np.float32),
                gripper=-1, 
                reset=False,
                timestamp=time.time(),
            )

            # Send and wait for server response (clears the buffer)
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            _ = self.action_socket.recv()
                
        print("Replay finished.")
        self.action_socket.close()


if __name__ == "__main__":
    demo_path = "data/demonstration_0"

    replayer = ArmReplayer(demo_path)
    replayer.replay()
