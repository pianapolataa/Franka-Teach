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
        print("Starting arm replay...")
        if self.arm_trajectory is None or len(self.arm_trajectory) == 0:
            print("No actions to replay.")
            return

        # Slice the trajectory to skip the first 100,000 steps
        # This is much more efficient than checking an 'if' condition inside the loop
        replay_data = self.arm_trajectory[100000:]

        if len(replay_data) == 0:
            print("Error: Trajectory is shorter than 100,000 steps. Nothing to replay.")
            return
        # TRAJECTORY REPLAY
        dt = 0.005
        first = True
        
        for row in replay_data:
            print(first, row[0])
            if first == True and row[0] < 0.47:
                first = False
                continue
            loop_start = time.time()
            
            target_pos = row[:3]
            target_quat = row[3:]
            action = FrankaAction(
                pos=target_pos.astype(np.float32),
                quat=target_quat.astype(np.float32),
                gripper=1.0, 
                reset=False,
                timestamp=time.time(),
            )

            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            _ = self.action_socket.recv()

            # Maintain timing consistency (100Hz)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
        print("Replay finished.")
        self.action_socket.close()


if __name__ == "__main__":
    demo_path = "data/demonstration_0"

    replayer = ArmReplayer(demo_path)
    replayer.replay()
