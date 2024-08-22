from multiprocessing import Process
import os
from pathlib import Path
import time
import numpy as np

from deoxys.utils import YamlConfig
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import (
    get_default_controller_config,
    verify_controller_config,
)

from frankateach.utils import FrequencyTimer, notify_component_start
from frankateach.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from frankateach.messages import FrankaAction, FrankaState

TRANSLATIONAL_POSE_VELOCITY_SCALE = 5
ROTATIONAL_POSE_VELOCITY_SCALE = 0.75
ROTATION_VELOCITY_LIMIT = 0.5
TRANSLATION_VELOCITY_LIMIT = 1
VR_FREQ = 90
STATE_FREQ = 100

CONFIG_ROOT = Path(__file__).parent / "configs"


class FrankaServer:
    def __init__(
        self,
        cfg,
        host,
        state_port,
        control_port,
        state_freq=VR_FREQ,
        control_freq=STATE_FREQ,
    ):
        self._robot = Robot(cfg, control_freq)
        self.state_publisher = ZMQKeypointPublisher(host, state_port)
        self.control_subscriber = ZMQKeypointSubscriber(host, control_port)

        self.control_timer = FrequencyTimer(control_freq)
        self.state_timer = FrequencyTimer(STATE_FREQ)

        # start publisher and subscriber
        publisher_process = Process(target=self.publish_state)
        subscriber_process = Process(target=self.control_robot)

        publisher_process.start()
        subscriber_process.start()

        publisher_process.join()
        subscriber_process.join()

    def publish_state(self):
        notify_component_start(component_name="Franka State Publisher")
        try:
            while True:
                self.state_timer.start_loop()
                quat, pos = self._robot.last_eef_rot_and_pos
                gripper = self._robot.last_gripper_q
                if quat is not None and pos is not None and gripper is not None:
                    state = FrankaState(
                        pos=pos.flatten().astype(np.float32),
                        quat=quat.flatten().astype(np.float32),
                        gripper=gripper.flatten().astype(np.float32),
                        timestamp=time.time(),
                    )
                    self.state_publisher.pub_keypoints(state, topic_name="robot_state")
                self.state_timer.end_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.state_publisher.close()

    def control_robot(self):
        notify_component_start(component_name="Franka Control Subscriber")
        try:
            while True:
                self.control_timer.start_loop()

                franka_control: FrankaAction = self.control_subscriber.recv_keypoint()
                if franka_control.reset:
                    self._robot.reset_joints()
                else:
                    self._robot.osc_move(
                        franka_control.pos, franka_control.quat, franka_control.gripper
                    )

                self.control_timer.end_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.control_subscriber.close()


class Robot(FrankaInterface):
    def __init__(self, cfg, control_freq):
        super(Robot, self).__init__(
            general_cfg_file=os.path.join(CONFIG_ROOT, cfg),
            use_visualizer=False,
            control_freq=control_freq,
        )
        self.velocity_controller_cfg = verify_controller_config(
            YamlConfig(
                os.path.join(CONFIG_ROOT, "osc-pose-controller.yml")
            ).as_easydict()
        )

        self.reset_robot()

    def reset_robot(self):
        self.reset()

        print("Waiting for the robot to connect...")
        while len(self._state_buffer) > 0:
            time.sleep(0.01)

        print("Franka is connected")

    def osc_move(self, target_pos, target_quat, gripper_state):
        target_mat = transform_utils.pose2mat(pose=(target_pos, target_quat))

        current_quat, current_pos = self.last_eef_quat_and_pos
        current_mat = transform_utils.pose2mat(
            pose=(current_pos.flatten(), current_quat.flatten())
        )

        pose_error = transform_utils.get_pose_error(
            target_pose=target_mat, current_pose=current_mat
        )

        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat

        quat_diff = transform_utils.quat_distance(target_quat, current_quat)
        axis_angle_diff = transform_utils.quat2axisangle(quat_diff)

        action_pos = pose_error[:3] * TRANSLATIONAL_POSE_VELOCITY_SCALE
        action_axis_angle = axis_angle_diff.flatten() * ROTATIONAL_POSE_VELOCITY_SCALE

        action_pos, _ = transform_utils.clip_translation(
            action_pos, TRANSLATION_VELOCITY_LIMIT
        )
        action_axis_angle = np.clip(
            action_axis_angle, -ROTATION_VELOCITY_LIMIT, ROTATION_VELOCITY_LIMIT
        )

        action = (
            action_pos.tolist() + action_axis_angle.tolist() + gripper_state.tolist()
        )

        self.control(
            controller_type="OSC_POSE",
            action=action,
            controller_cfg=self.velocity_controller_cfg,
        )

    def reset_joints(
        self,
        timeout=7,
        gripper_open=False,
    ):
        start_joint_pos = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]
        assert type(start_joint_pos) is list or type(start_joint_pos) is np.ndarray
        controller_cfg = get_default_controller_config(controller_type="JOINT_POSITION")

        if gripper_open:
            gripper_action = -1
        else:
            gripper_action = 1

        # This is for varying initialization of joints a little bit to
        # increase data variation.
        start_joint_pos = [
            e + np.clip(np.random.randn() * 0.005, -0.005, 0.005)
            for e in start_joint_pos
        ]
        if type(start_joint_pos) is list:
            action = start_joint_pos + [gripper_action]
        else:
            action = start_joint_pos.tolist() + [gripper_action]
        start_time = time.time()
        while True:
            if self.received_states and self.check_nonzero_configuration():
                if (
                    np.max(np.abs(np.array(self.last_q) - np.array(start_joint_pos)))
                    < 1e-3
                ):
                    break
            self.control(
                controller_type="JOINT_POSITION",
                action=action,
                controller_cfg=controller_cfg,
            )
            end_time = time.time()

            # Add timeout
            if end_time - start_time > timeout:
                break
        return True


if __name__ == "__main__":
    franka_server = FrankaServer(
        cfg="deoxys_right.yml",
        host="localhost",
        state_port=8900,
        control_port=8901,
    )
