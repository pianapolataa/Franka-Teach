import cv2
import gymnasium as gym
import numpy as np
import time
import pickle

from frankateach.constants import CAM_PORT, GRIPPER_OPEN, HOST, CONTROL_PORT
from frankateach.messages import FrankaAction, FrankaState
from frankateach.network import (
    ZMQCameraSubscriber,
    create_request_socket,
)


class FrankaEnv(gym.Env):
    def __init__(self, num_cameras=4, width=640, height=480):
        self._width = width
        self._height = height
        self._channels = 3
        self._action_dim = 8  # (pos, quat, gripper)

        self.action_space = gym.spaces.Box(
            low=-float("inf"), high=float("inf"), shape=(self._action_dim,)
        )
        self.observation_space = gym.spaces.Box(
            low=0, high=255, shape=(), dtype=np.uint8
        )

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

    def step(self, action):
        action = np.array(action)

        # Send action to the robot
        franka_action = FrankaAction(
            pos=action[:3],
            quat=action[3:7],
            gripper=action[7],
            reset=False,
            timestamp=time.time(),
        )

        self.action_request_socket.send(bytes(pickle.dumps(franka_action, protocol=-1)))
        franka_state: FrankaState = pickle.loads(self.action_request_socket.recv())

        image_list = []
        for subscriber in self.image_subscribers:
            image, _ = subscriber.recv_rgb_image()
            image_list.append(image)

        obs = {
            "features": np.concatenate(
                (franka_state.pos, franka_state.quat, [franka_state.gripper])
            ),
        }

        for i, image in enumerate(image_list):
            obs[f"pixels_{i}"] = cv2.resize(image, (self._width, self._height))

        return obs, 0, False, {}

    def reset(self):
        franka_action = FrankaAction(
            pos=np.zeros(3),
            quat=np.zeros(4),
            gripper=GRIPPER_OPEN,
            reset=True,
            timestamp=time.time(),
        )

        self.action_request_socket.send(bytes(pickle.dumps(franka_action, protocol=-1)))
        franka_state: FrankaState = pickle.loads(self.action_request_socket.recv())

        image_list = []
        for subscriber in self.image_subscribers:
            image, _ = subscriber.recv_rgb_image()
            image_list.append(image)

        obs = {
            "features": np.concatenate(
                (franka_state.pos, franka_state.quat, [franka_state.gripper])
            ),
        }
        for i, image in enumerate(image_list):
            obs[f"pixels_{i}"] = cv2.resize(image, (self._width, self._height))

        return obs, {}
