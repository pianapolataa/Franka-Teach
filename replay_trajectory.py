from frankateach.constants import CONTROL_PORT, HOST
from frankateach.franka_env import FrankaEnv
import pickle
import numpy as np
import time
import sys

from frankateach.messages import FrankaAction, FrankaState
from frankateach.network import create_request_socket
from frankateach.utils import FrequencyTimer


# def main():
#     franka_env = FrankaEnv()
#     freq_timer = FrequencyTimer(90)
#
#     obs, _ = franka_env.reset()
#
#     # read the previous trajectory
#     with open("./extracted_data/test/demonstration_1/states.pkl", "rb") as f:
#         data = pickle.load(f)
#
#     idx = 0
#     while True:
#         freq_timer.start_loop()
#         franka_state: FrankaState = data[idx]
#         target_pos, target_quat, target_gripper = (
#             franka_state.quat,
#             franka_state.pos,
#             franka_state.gripper,
#         )
#         action = np.concatenate((target_pos, target_quat, [target_gripper]))
#
#         obs, reward, done, trunc, info = franka_env.step(action)
#         idx += 1
#         freq_timer.end_loop()


def main():
    replay_freq = 30
    freq_timer = FrequencyTimer(replay_freq)
    with open("./extracted_data/test/demonstration_4/states.pkl", "rb") as f:
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
    ok_msg = action_request_socket.recv()
    if ok_msg != b"ok":
        print("Failed to reset the robot.")

    step_idx = int(state_freq // replay_freq)
    idx = 0
    while True:
        freq_timer.start_loop()
        franka_state: FrankaState = data[idx]

        franka_action = FrankaAction(
            pos=franka_state.pos,
            quat=franka_state.quat,
            gripper=franka_state.gripper,
            reset=False,
            timestamp=time.time(),
        )
        print(franka_action)

        tic = time.time()
        action_request_socket.send(bytes(pickle.dumps(franka_action, protocol=-1)))
        ok_msg = action_request_socket.recv()
        print(f"Time taken to send action: {time.time() - tic}")

        if ok_msg != b"ok":
            print("Failed to send action to the robot.")

        idx += step_idx
        if idx > len(data) - 1:
            print("End of trajectory.")
            break
        freq_timer.end_loop()


if __name__ == "__main__":
    main()

# replay the trajectory
