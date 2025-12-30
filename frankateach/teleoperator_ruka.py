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

from ruka_hand.control.rukav2_teleop import *
from ruka_hand.utils.timer import FrequencyTimer
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
        self.ruka_state_socket = ZMQKeypointPublisher(LOCALHOST, RUKA_STATE_PORT)        
        self.ruka_commanded_state_socket = ZMQKeypointPublisher(LOCALHOST, RUKA_COMMANDED_STATE_PORT)
   
        # self.state_socket = ZMQKeypointPublisher(LOCALHOST, STATE_PORT)
        # self.commanded_state_socket = ZMQKeypointPublisher(LOCALHOST, COMMANDED_STATE_PORT)

        self.is_first_frame = True
        self.start_teleop = False
        self.init_affine = None

        # self.home_offset = (
        #     np.array(home_offset) if home_offset is not None else np.zeros(3)
        # )
        
        self.hand_type = hand_type
        
    def _get_hand_coords(self):
        for i in range(10):
            data = self._transformed_hand_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if not data is None: break 
        if data is None: return None
        return np.asanyarray(data)
    
    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()

        if reset_stat:
            reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        else:
            return ARM_TELEOP_STOP
        return reset_stat

    def _init_hand(self):
        self.handler = RUKAv2Handler()
        self.cnt = 0
    
    def _apply_retargeted_angles(self) -> None:
        arm_teleop_state = self._get_arm_teleop_state()
        print(arm_teleop_state)
        transformed_hand_coords = self._get_hand_coords() # (24, 3)
        self.cnt += 1

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
            time.sleep(0.5)
            self.handler.reset()
            self.is_first_frame = False
            time.sleep(1)
           

        if self.start_teleop:
            motor_positions = self.handler.get_command(transformed_hand_coords)
            if (self.cnt % 5 == 0): 
                motor_positions += np.random.normal(0, 40, size=16)
                motor_positions = np.clip(motor_positions, np.minimum(self.handler.hand.tensioned_pos, self.handler.hand.curled_bound), np.maximum(self.handler.hand.tensioned_pos, self.handler.hand.curled_bound))
            curr_pos = self.handler.hand.read_pos()
            self.ruka_state_socket.pub_keypoints(curr_pos, "ruka_state")
            self.ruka_commanded_state_socket.pub_keypoints(motor_positions, "commanded_ruka_state")
            move_to_pos(curr_pos=curr_pos, des_pos=motor_positions, hand=self.handler.hand, traj_len=20)
  

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
            self.handler.hand.close()
            # self.action_socket.close()

        print("Stopping the ruka teleoperator!")


def main():
    operator = RukaOperator(save_states=True)
    operator.stream()


if __name__ == "__main__":
    main()