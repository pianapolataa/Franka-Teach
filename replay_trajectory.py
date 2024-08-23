from frankateach.constants import CONTROL_PORT, HOST
from frankateach.franka_env import FrankaEnv
import pickle
import numpy as np
import time

from frankateach.messages import FrankaAction, FrankaState
from frankateach.network import create_request_socket


def main_frankaenv():
    franka_env = FrankaEnv()
    obs, _ = franka_env.reset()

    # read the previous trajectory
    with open("./extracted_data/test/demonstration_4/states.pkl", "rb") as f:
        data = pickle.load(f)

    time_past = data[-1].timestamp - data[0].timestamp
    state_freq = len(data) / time_past
    print(f"Trajectory is {time_past} seconds, sampled at {state_freq} Hz.")

    idx = 0
    while True:
        franka_state: FrankaState = data[idx]
        target_pos, target_quat, target_gripper = (
            franka_state.pos,
            franka_state.quat,
            franka_state.gripper,
        )
        action = np.concatenate((target_pos, target_quat, [target_gripper]))

        tic = time.time()
        obs, reward, done, info = franka_env.step(action)
        print(f"Step time: {time.time() - tic}")
        idx += 1

        if idx > len(data) - 1:
            print("End of trajectory.")
            break


def main():
    with open("./extracted_data/test/demonstration_5/states.pkl", "rb") as f:
        data = pickle.load(f)

    time_past = data[-1].timestamp - data[0].timestamp
    state_freq = len(data) / time_past
    print(f"Trajectory is {time_past} seconds, sampled at {state_freq} Hz.")

    # reset once
    reset_action = FrankaAction(
        pos=np.zeros(3),
        quat=np.zeros(4),
        gripper=0,
        reset=True,
        timestamp=time.time(),
    )

    action_request_socket = create_request_socket(HOST, CONTROL_PORT)
    action_request_socket.send(bytes(pickle.dumps(reset_action, protocol=-1)))
    state = pickle.loads(action_request_socket.recv())

    idx = 0
    while True:
        franka_state: FrankaState = data[idx]

        franka_action = FrankaAction(
            pos=franka_state.pos,
            quat=franka_state.quat,
            gripper=franka_state.gripper,
            reset=False,
            timestamp=time.time(),
        )

        tic = time.time()
        action_request_socket.send(bytes(pickle.dumps(franka_action, protocol=-1)))
        state = pickle.loads(action_request_socket.recv())
        print(f"Time taken to send action: {time.time() - tic}")

        idx += 1
        if idx > len(data) - 1:
            print("End of trajectory.")
            break


if __name__ == "__main__":
    main_frankaenv()

# replay the trajectory
