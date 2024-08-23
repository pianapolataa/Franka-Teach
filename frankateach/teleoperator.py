import pickle
from frankateach.utils import notify_component_start
from frankateach.network import ZMQKeypointSubscriber, create_request_socket
from frankateach.constants import (
    CONTROL_PORT,
    HOST,
    STATE_PORT,
    VR_CONTROLLER_STATE_PORT,
    H_R_V,
    H_R_V_star,
    ROBOT_WORKSPACE_MIN,
    ROBOT_WORKSPACE_MAX,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
)
from frankateach.messages import FrankaAction, FrankaState

from deoxys.utils import transform_utils

import time
import numpy as np
from numpy.linalg import pinv


def get_relative_affine(init_affine, current_affine):
    H_V_des = pinv(init_affine) @ current_affine

    # Transform to robot frame.
    relative_affine_rot = (pinv(H_R_V) @ H_V_des @ H_R_V)[:3, :3]
    relative_affine_trans = (pinv(H_R_V_star) @ H_V_des @ H_R_V_star)[:3, 3]

    # Homogeneous coordinates
    relative_affine = np.block(
        [[relative_affine_rot, relative_affine_trans.reshape(3, 1)], [0, 0, 0, 1]]
    )

    return relative_affine


class FrankaOperator:
    def __init__(self, save_states=False) -> None:
        # Subscribe controller state
        self._controller_state_subscriber = ZMQKeypointSubscriber(
            host=HOST, port=VR_CONTROLLER_STATE_PORT, topic="controller_state"
        )

        self.action_socket = create_request_socket(HOST, CONTROL_PORT)
        self.state_socket = create_request_socket(HOST, STATE_PORT)

        # Class variables
        self._save_states = save_states
        self.is_first_frame = True
        self.gripper_state = GRIPPER_OPEN
        self.start_teleop = False
        self.init_affine = None

    def _apply_retargeted_angles(self) -> None:
        self.controller_state = self._controller_state_subscriber.recv_keypoints()

        if self.is_first_frame:
            print("Resetting robot..")
            action = FrankaAction(
                pos=np.zeros(3),
                quat=np.zeros(4),
                gripper=GRIPPER_OPEN,
                reset=True,
                timestamp=time.time(),
            )
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            robot_state = pickle.loads(self.action_socket.recv())

            print(robot_state)
            self.home_rot, self.home_pos = (
                transform_utils.quat2mat(robot_state.quat),
                robot_state.pos,
            )
            self.is_first_frame = False
        if self.controller_state.right_a:
            self.start_teleop = True
            self.init_affine = self.controller_state.right_affine
        if self.controller_state.right_b:
            self.start_teleop = False
            self.init_affine = None
            # receive the robot state
            self.action_socket.send(b"get_state")
            robot_state: FrankaState = pickle.loads(self.action_socket.recv())
            if robot_state == b"state_error":
                print("Error getting robot state")
                return

            self.home_rot, self.home_pos = (
                transform_utils.quat2mat(robot_state.quat),
                robot_state.pos,
            )

        if self.start_teleop:
            relative_affine = get_relative_affine(
                self.init_affine, self.controller_state.right_affine
            )
        else:
            relative_affine = np.zeros((4, 4))
            relative_affine[3, 3] = 1

        gripper_action = None
        if self.controller_state.right_index_trigger > 0.5:
            gripper_action = GRIPPER_CLOSE
        elif self.controller_state.right_hand_trigger > 0.5:
            gripper_action = GRIPPER_OPEN

        if gripper_action is not None and gripper_action != self.gripper_state:
            self.gripper_state = gripper_action

        if self.start_teleop:
            relative_pos, relative_rot = (
                relative_affine[:3, 3],
                relative_affine[:3, :3],
            )

            target_pos = self.home_pos + relative_pos
            target_rot = self.home_rot @ relative_rot
            target_quat = transform_utils.mat2quat(target_rot)

            target_pos = np.clip(
                target_pos,
                a_min=ROBOT_WORKSPACE_MIN,
                a_max=ROBOT_WORKSPACE_MAX,
            )

        else:
            target_pos, target_quat = (
                self.home_pos,
                transform_utils.mat2quat(self.home_rot),
            )

        # Save the states here
        # quat, pos = self._robot.last_eef_quat_and_pos
        # self._poses.append(np.concatenate((pos.flatten(), quat.flatten())))
        # self._commanded_poses.append(
        #     np.concatenate((target_pos.flatten(), target_quat.flatten()))
        # )
        # self._gripper_states.append(self.gripper_state)
        # self._timestamps.append(time.time())

        action = FrankaAction(
            pos=target_pos.flatten().astype(np.float32),
            quat=target_quat.flatten().astype(np.float32),
            gripper=self.gripper_state,
            reset=False,
            timestamp=time.time(),
        )

        tic = time.time()
        self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
        robot_state = self.action_socket.recv()
        print(f"Action takes: {time.time() - tic}")

        if self._save_states:
            tic = time.time()
            self.state_socket.send(robot_state)
            ok_msg = self.state_socket.recv()
            print(f"Saving state takes: {time.time() - tic}")

    # def save_states(self):
    #     teleop_time = self._timestamps[-1] - self._timestamps[0]
    #     print(f"Took {teleop_time} seconds")
    #     print(f"Saved {len(self._timestamps)} datapoints..")
    #     print(f"Action save frequency : {len(self._timestamps) / teleop_time} Hz")

    #     save_path = Path(self._storage_path) / f"demonstration_{self._demo_num}"
    #     save_path.mkdir(parents=True, exist_ok=True)

    #     with open(save_path / "states.pkl", "wb") as f:
    #         pickle.dump(
    #             {
    #                 "poses": self._poses,
    #                 "commanded_poses": self._commanded_poses,
    #                 "gripper_states": self._gripper_states,
    #                 "timestamps": self._timestamps,
    #             },
    #             f,
    #         )

    def stream(self):
        notify_component_start("Franka teleoperator control")
        print("Start controlling the robot hand using the Oculus Headset.\n")

        try:
            while True:
                # Retargeting function
                self._apply_retargeted_angles()
        except KeyboardInterrupt:
            pass
        finally:
            self._controller_state_subscriber.stop()
            self.action_socket.close()

        print("Stopping the teleoperator!")


def main():
    operator = FrankaOperator(save_states=True)
    operator.stream()


if __name__ == "__main__":
    main()
