import cv2
import gym
import numpy as np
import time
import pickle

from frankateach.constants import (
    CAM_PORT,
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    HOST,
    CONTROL_PORT,
)
from frankateach.messages import FrankaAction, FrankaState
from frankateach.network import (
    ZMQCameraSubscriber,
    create_request_socket,
)


class FrankaEnv(gym.Env):
    def __init__(
        self, num_cameras=4, width=640, height=480, use_robot=True, use_egocentric=False
    ):
        super(FrankaEnv, self).__init__()
        self.width = width
        self.height = height
        self.channels = 3
        self.feature_dim = 8
        self.action_dim = 7  # (pos, axis angle, gripper)

        self.use_robot = use_robot
        self.use_egocentric = use_egocentric

        self.n_channels = 3
        self.reward = 0

        self.franka_state = None
        self.curr_images = None

        self.action_space = gym.spaces.Box(
            low=-float("inf"), high=float("inf"), shape=(self.action_dim,)
        )
        self.observation_space = gym.spaces.Box(
            low=0, high=255, shape=(height, width, self.n_channels), dtype=np.uint8
        )

        if self.use_robot:
            self.image_subscribers = []
            for cam_idx in range(num_cameras):
                port = CAM_PORT + cam_idx
                self.image_subscribers.append(
                    ZMQCameraSubscriber(
                        host=HOST,
                        port=port,
                        topic_type="RGB",
                    )
                )

            self.action_request_socket = create_request_socket(HOST, CONTROL_PORT)

    def get_state(self):
        self.action_request_socket.send(b"get_state")
        franka_state: FrankaState = pickle.loads(self.action_request_socket.recv())
        self.franka_state = franka_state
        return franka_state

    def step(self, abs_action):
        pos = abs_action[:3]
        quat = abs_action[3:7]
        gripper = abs_action[-1]
        if gripper > 0.5:
            gripper = GRIPPER_OPEN
        else:
            gripper = GRIPPER_CLOSE

        # Send action to the robot
        franka_action = FrankaAction(
            pos=pos,
            quat=quat,
            gripper=gripper,
            reset=False,
            timestamp=time.time(),
        )
        # print("sending action to robot: ", franka_action)

        self.action_request_socket.send(bytes(pickle.dumps(franka_action, protocol=-1)))
        franka_state: FrankaState = pickle.loads(self.action_request_socket.recv())
        self.franka_state = franka_state

        image_list = []
        for subscriber in self.image_subscribers:
            image, _ = subscriber.recv_rgb_image()
            image_list.append(image)

        self.curr_images = image_list

        obs = {
            "features": np.concatenate(
                (franka_state.pos, franka_state.quat, [franka_state.gripper])
            ),
        }

        for i, image in enumerate(image_list):
            obs[f"pixels{i}"] = cv2.resize(image, (self.width, self.height))

        return obs, self.reward, False, None

    def reset(self):
        if self.use_robot:
            print("resetting")
            # TODO: send b"reset" to the robot instead of this action
            franka_action = FrankaAction(
                pos=np.zeros(3),
                quat=np.zeros(4),
                gripper=GRIPPER_OPEN,
                reset=True,
                timestamp=time.time(),
            )

            self.action_request_socket.send(
                bytes(pickle.dumps(franka_action, protocol=-1))
            )
            franka_state: FrankaState = pickle.loads(self.action_request_socket.recv())
            self.franka_state = franka_state
            print("reset done: ", franka_state)

            image_list = []
            for subscriber in self.image_subscribers:
                image, _ = subscriber.recv_rgb_image()
                image_list.append(image)

            self.curr_images = image_list

            obs = {
                "features": np.concatenate(
                    (franka_state.pos, franka_state.quat, [franka_state.gripper])
                ),
            }
            for i, image in enumerate(image_list):
                obs[f"pixels{i}"] = cv2.resize(image, (self.width, self.height))

            return obs

        else:
            obs = {}
            obs["features"] = np.zeros(self.feature_dim)
            obs["pixels"] = np.zeros((self.height, self.width, self.n_channels))

            return obs

    def render(self, mode="rgb_array", width=640, height=480):
        assert self.curr_images is not None, "Must call reset() before render()"
        if mode == "rgb_array":
            image_list = []
            for im in self.curr_images:
                image_list.append(cv2.resize(im, (width, height)))

            return np.concatenate(image_list, axis=1)
        else:
            raise NotImplementedError


if __name__ == "__main__":
    env = FrankaEnv()
    images = []
    obs = env.reset()

    apply_deltas = False
    if apply_deltas:
        delta_pos = 0.03
        delta_angle = 0.05
        for i in range(100):
            obs, reward, done, _ = env.step([delta_pos, 0, 0, 0, 0, 0, GRIPPER_OPEN])
            images.append(obs["pixels_0"])

        for i in range(100):
            obs, reward, done, _ = env.step([0, delta_pos, 0, 0, 0, 0, GRIPPER_OPEN])
            images.append(obs["pixels_0"])

        for i in range(100):
            obs, reward, done, _ = env.step([0, 0, delta_pos, 0, 0, 0, GRIPPER_OPEN])
            images.append(obs["pixels_0"])

        for i in range(100):
            obs, reward, done, _ = env.step([0, 0, 0, delta_angle, 0, 0, GRIPPER_OPEN])
            images.append(obs["pixels_0"])

        for i in range(100):
            obs, reward, done, _ = env.step([0, 0, 0, 0, delta_angle, 0, GRIPPER_OPEN])
            images.append(obs["pixels_0"])

        for i in range(100):
            obs, reward, done, _ = env.step([0, 0, 0, 0, 0, delta_angle, GRIPPER_OPEN])
            images.append(obs["pixels_0"])

        np.save("images.npy", np.array(images))
