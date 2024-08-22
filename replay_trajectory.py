from frankateach.franka_env import FrankaEnv
import pickle
import numpy as np

from frankateach.messages import FrankaState
from frankateach.utils import FrequencyTimer


def main():
    franka_env = FrankaEnv()
    freq_timer = FrequencyTimer(90)

    obs, _ = franka_env.reset()

    # read the previous trajectory
    with open("./extracted_data/test/demonstration_1/states.pkl", "rb") as f:
        data = pickle.load(f)

    idx = 0
    while True:
        freq_timer.start_loop()
        franka_state: FrankaState = data[idx]
        target_pos, target_quat, target_gripper = (
            franka_state.quat,
            franka_state.pos,
            franka_state.gripper,
        )
        action = np.concatenate((target_pos, target_quat, [target_gripper]))

        obs, reward, done, trunc, info = franka_env.step(action)
        idx += 1
        freq_timer.end_loop()


if __name__ == "__main__":
    main()

# replay the trajectory
