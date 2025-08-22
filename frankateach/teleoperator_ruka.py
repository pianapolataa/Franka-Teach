import numpy as np
import zmq
import time
import pickle
from frankateach.utils import notify_component_start
from copy import deepcopy as copy
from frankateach.constants import *
from frankateach.utils import FrequencyTimer
from frankateach.network import ZMQKeypointSubscriber, ZMQKeypointPublisher, create_request_socket
from frankateach.vectorops import *
from scipy.spatial.transform import Rotation, Slerp
from frankateach.messages import FrankaAction, FrankaState
from deoxys.utils import transform_utils
from numpy.linalg import pinv
from frankateach.franka_server import Robot
import os
from scipy.spatial.transform import Rotation as R

from ruka_hand.control.operator import RUKAOperator
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.vectorops import *
from ruka_hand.utils.zmq import ZMQPublisher, create_pull_socket
from ruka_hand.utils.trajectory import move_to_pos
from ruka_hand.control.hand import Hand

np.set_printoptions(precision=2, suppress=True)

class RukaOperator:
    def __init__(
        self,
        transformed_keypoints_port,
        teleoperation_reset_port = None,
        hand_type = "right"
    ):
        notify_component_start('franka arm operator')
        # Subscribers for the transformed hand keypoints
        print("create subscriber", LOCALHOST, transformed_keypoints_port)
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=LOCALHOST,
            port=transformed_keypoints_port,
            topic='transformed_hand_coords'
        )
        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = LOCALHOST, 
            port = teleoperation_reset_port,
            topic = 'pause'
        )
        
   
        # self.state_socket = ZMQKeypointPublisher(LOCALHOST, STATE_PORT)
        # self.commanded_state_socket = ZMQKeypointPublisher(LOCALHOST, COMMANDED_STATE_PORT)

        self.is_first_frame = True
        self.start_teleop = False
        self.init_affine = None

        # self.home_offset = (
        #     np.array(home_offset) if home_offset is not None else np.zeros(3)
        # )
        
        self.hand_type = hand_type
            
    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat
    
    def _get_hand_coords(self):
        for i in range(10):
            data = self._transformed_hand_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if not data is None: break 
        if data is None: return None
        return np.asanyarray(data)
    
    
    # Converts Homogenous Transformation Matrix to Cartesian Coords
    def _homo2cart(self, homo_mat):
        
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(
            homo_mat[:3, :3]).as_quat()

        cart = np.concatenate(
            [t, R], axis=0
        )
        
        return cart
    
    def _cart2homo(self, cart_pose):
        """
        Converts a 7D cartesian pose [x, y, z, qw, qx, qy, qz]
        into a 4x4 homogeneous transformation matrix.
        """
        pos = cart_pose[:3]
        quat = cart_pose[3:]  # [qw, qx, qy, qz]

        # Convert quaternion to rotation matrix
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()  # scipy uses [x, y, z, w]

        homo_mat = np.eye(4)
        homo_mat[:3, :3] = rot
        homo_mat[:3, 3] = pos

        return homo_mat
    
    
    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()

        if reset_stat:
            reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        else:
            return ARM_TELEOP_STOP
        return reset_stat

    def _init_hand(self):
        self.hand_dict = {}
        self.hand_dict[self.hand_type] = RUKAOperator(
            hand_type=self.hand_type,
            moving_average_limit=5,
        )
        
    def _get_ordered_joints(self, projected_translated_coords):
        # Extract joint data based on HAND_JOINTS
        extracted_joints = {
            joint: projected_translated_coords[indices]
            for joint, indices in HAND_JOINTS.items()
        }
        # Concatenate the extracted joint data in the same order as the dictionary keys
        ordered_joints = np.concatenate(
            [extracted_joints[joint] * 100.0 for joint in HAND_JOINTS],
            axis=0,
        )
        reshaped_joints = ordered_joints.reshape(5, 5, 3)

        return reshaped_joints

    
    def _apply_retargeted_angles(self) -> None:
        arm_teleop_state = self._get_arm_teleop_state()
        print(arm_teleop_state)
        transformed_hand_coords = self._get_hand_coords() # (24, 3)
        print(transformed_hand_coords)

        if arm_teleop_state ==  ARM_TELEOP_CONT:
            self.start_teleop = True
            
        if arm_teleop_state ==  ARM_TELEOP_STOP:
            self.start_teleop = False
            # receive the robot state
            
        if self.is_first_frame:
            # reset hand positiion
            hand = Hand(self.hand_type)
            print("hand")
            print(type(hand))
            curr_pos = hand.read_pos()
            time.sleep(0.5)
            print(f"curr_pos: {curr_pos}, des_pos: {hand.tensioned_pos}")
            test_pos = hand.tensioned_pos
            move_to_pos(curr_pos=curr_pos, des_pos=test_pos, hand=hand, traj_len=50)
            self.is_first_frame = False
            time.sleep(1)
           

        if self.start_teleop:
            self.hand_dict[self.hand_type].step(transformed_hand_coords)
  

    def stream(self):
        notify_component_start("Franka teleoperator control")
        print("Start controlling the robot hand using the Oculus Headset.\n")
        self._init_hand()
        try:
            while True:
                # Retargeting function
                print("start teleop", self.start_teleop)
                self._apply_retargeted_angles()
        except KeyboardInterrupt:
            pass
        finally:
            self._transformed_hand_keypoint_subscriber.stop()
            # self.action_socket.close()

        print("Stopping the ruka teleoperator!")


def main():
    operator = RukaOperator(save_states=True)
    operator.stream()


if __name__ == "__main__":
    main()
