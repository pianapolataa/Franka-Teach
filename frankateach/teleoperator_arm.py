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

np.set_printoptions(precision=2, suppress=True)
# Filter to smooth out the arm cartesian state
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_quat(
            np.stack([self.ori_state, next_state[3:7]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])

class FrankaArmOperator:
    def __init__(
        self,
        transformed_keypoints_port,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        init_gripper_state='open',
        home_offset=[0, 0, 0],
    ):
        notify_component_start('franka arm operator')

        # Subscribers for the transformed arm frame
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=LOCALHOST,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )
        
        self.resolution_scale = 1 # NOTE: Get this from a socket
        self.arm_teleop_state = ARM_TELEOP_STOP # We will start as the cont

        # Subscribers for the resolution scale and teleop state
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host = LOCALHOST,
            port = arm_resolution_port,
            topic = 'button'
        )

        # Continuous or stop teleoperation
        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = LOCALHOST, 
            port = teleoperation_reset_port,
            topic = 'pause'
        )
        
        self.action_socket = create_request_socket(LOCALHOST, CONTROL_PORT)
        self.state_socket = ZMQKeypointPublisher(LOCALHOST, STATE_PORT)
        self.commanded_state_socket = ZMQKeypointPublisher(LOCALHOST, COMMANDED_STATE_PORT)

  # Class variables
        # self._save_states = save_states
        self.is_first_frame = True
        self.gripper_state = (
            GRIPPER_OPEN if init_gripper_state == "open" else GRIPPER_CLOSE
        )
        self.start_teleop = False
        self.init_affine = None

        # if  home_offset is None:
        #     home_offset = [-0.22, 0.0, 0.1]
        self.home_offset = (
            np.array(home_offset) if home_offset is not None else np.zeros(3)
        )

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)
         
            
# Get the teleop state (Pause or Continue)
    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()
        print(reset_stat)
        if reset_stat:
            reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        else:
            return ARM_TELEOP_STOP
        return reset_stat
    
    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat
    
    def _get_hand_frame(self):
        for i in range(10):
            data = self._transformed_arm_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if not data is None: break 
        if data is None: return None
        return np.asanyarray(data).reshape(4, 3)
    
    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        res_scale = np.asanyarray(data).reshape(1)[0] # Make sure this data is one dimensional
        return res_scale  
    
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
    
    # Gets the Scaled Resolution pose
    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        # Get the cart pose without the scaling
        unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat)

        # Get the current cart pose
        # REVISION !! CHANGE TO FRANKATEACH
        # current_homo_mat = copy(self.robot.get_pose()['position'])
        self.action_socket.send(b"get_state")
        robot_state = pickle.loads(self.action_socket.recv())
        # HOME <- Pos: [0.457632  0.0321814 0.2653815], Quat: [0.9998586  0.00880853 0.01421072 0.00179784]

        self.home_rot, self.home_pos = (
            transform_utils.quat2mat(robot_state.quat),
            robot_state.pos,
        )

        # robot init affine
        current_homo_mat = np.eye(4)
        current_homo_mat[:3, :3] = self.home_rot
        current_homo_mat[:3, 3] = self.home_pos

        current_cart_pose = self._homo2cart(current_homo_mat)
        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * self.resolution_scale
        # print('SCALED_DIFF_IN_TRANSLATION: {}'.format(scaled_diff_in_translation))
        
        scaled_cart_pose = np.zeros(7)
        scaled_cart_pose[3:] = unscaled_cart_pose[3:] # Get the rotation directly
        scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation # Get the scaled translation only
        scaled_homo_mat = self._cart2homo(scaled_cart_pose)
        return scaled_homo_mat
    
    def _to_robot_frame(self, init_affine, current_affine):
            # # Rotation from allegro to franka     This is for hand tracking not wrist tracking
            # H_A_R = np.array( 
            #     [[1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
            #     [-1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
            #     [0, 0, 1, -0.06], # The height of the allegro mount is 6cm
            #     [0, 0, 0, 1]])  

            # H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH # Homo matrix that takes P_HT to P_HI

            # # transform from hand frame to robot frame？
            # H_RT_RH = np.linalg.pinv(H_RH_HH) @ H_HT_HI @ H_RH_HH
            # # With allegro hand -- Q: why after getting relative in robot frame, turn it into H_RI_RH? --- to map hand frame to the arm frame
            # H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R) # Homo matrix that takes P_RT to P_RH
            # self.robot_moving_H = copy(H_RT_RH)

        H_V_des = pinv(init_affine) @ current_affine
        relative_affine_rot = (pinv(H_R_U) @ H_V_des @ H_R_U)[:3, :3]
        relative_affine_trans = (pinv(H_R_U_star) @ H_V_des @ H_R_U_star)[:3, 3]
        relative_affine = np.block(
            [[relative_affine_rot, relative_affine_trans.reshape(3, 1)], [0, 0, 0, 1]]
        )
        return relative_affine

    def _scale_angle(self, angle, min_angle, max_angle):
        angle = max(angle, min_angle)
        angle = min(angle, max_angle)
        return angle


    def _angles_around_axes(self, relative_rot: np.ndarray, hand_axes_mat: np.ndarray):
        """
        Returns signed rotation angles about initial wrist axes (degrees)
        """
        hand_axes_mat = np.asarray(hand_axes_mat).reshape(3, 3)
        hand_axes_mat = R.from_matrix(hand_axes_mat).as_matrix()
        Q, _ = np.linalg.qr(hand_axes_mat)
        if np.linalg.det(Q) < 0:
            Q[:, -1] *= -1
        hand_axes_mat = Q
        R_rel_wrist = hand_axes_mat.T @ relative_rot @ hand_axes_mat
        
        # Decompose into xyz euler angles (hand frame)
        angles = R.from_matrix(R_rel_wrist).as_euler('XYZ', degrees=True)
        return angles

    def _rot_from_hand_axes(self, angles_deg, hand_axes_mat: np.ndarray):
        """
        Rebuild rotation matrix from angles about wrist x,y,z axes
        """
        angles_deg = np.asarray(angles_deg).reshape(3,)
        hand_axes_mat = np.asarray(hand_axes_mat).reshape(3, 3)
        hand_axes_mat = R.from_matrix(hand_axes_mat).as_matrix()
        Q, _ = np.linalg.qr(hand_axes_mat)
        if np.linalg.det(Q) < 0:
            Q[:, -1] *= -1
        hand_axes_mat = Q
        R_wrist_scaled = R.from_euler('XYZ', angles_deg, degrees=True).as_matrix()
        
        # Convert back to world frame
        R_world_scaled = hand_axes_mat @ R_wrist_scaled @ hand_axes_mat.T
        return R_world_scaled

    def _apply_retargeted_angles(self) -> None:
        arm_teleop_state = self._get_arm_teleop_state()
        arm_teleoperation_scale_mode = self._get_resolution_scale_mode()
        print("entering apply retargeted", arm_teleop_state , self.start_teleop, arm_teleop_state)
        if arm_teleop_state ==  ARM_TELEOP_CONT:
            self.start_teleop = True
            
        if arm_teleop_state ==  ARM_TELEOP_STOP:
            self.start_teleop = False
            self.hand_init_H = None
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

        if arm_teleoperation_scale_mode == ARM_HIGH_RESOLUTION:
            self.resolution_scale = 1
        elif arm_teleoperation_scale_mode == ARM_LOW_RESOLUTION:
            self.resolution_scale = 0.6
            
        if self.is_first_frame:
            wrist_state = self._get_hand_frame()
            while  wrist_state is None:
                wrist_state = self._get_hand_frame()
                return None
            ##
            origin = wrist_state[0]
            x_axis = wrist_state[1]
            y_axis = wrist_state[2]
            z_axis = wrist_state[3]
            # 180° rotation around the palm normal (y-axis)
            rot_180 = R.from_rotvec(np.pi * y_axis / np.linalg.norm(y_axis)).as_matrix()
            x_rot = rot_180 @ x_axis
            y_rot = rot_180 @ y_axis
            z_rot = rot_180 @ z_axis
            rotated_frame = [origin, x_rot, y_rot, z_rot]
            self.hand_init_H = self._turn_frame_to_homo_mat(rotated_frame)
            ##
            # self.hand_init_H = self._turn_frame_to_homo_mat(wrist_state)  # wrist 4x4 matrix

            print("Resetting robot..")
            action = FrankaAction(
                pos=np.zeros(3),
                quat=np.zeros(4),
                gripper=self.gripper_state,
                reset=True,
                timestamp=time.time(),
            )
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            print("1st frame: finish sending reset action")
            robot_state = pickle.loads(self.action_socket.recv())
            print("1st frame: finish receiving robot state")
            # Move to offset position
            target_pos = robot_state.pos + self.home_offset
            target_quat = robot_state.quat
            action = FrankaAction(
                pos=target_pos.flatten().astype(np.float32),
                quat=target_quat.flatten().astype(np.float32),
                gripper=self.gripper_state,
                reset=False,
                timestamp=time.time(),
            )
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            
            robot_state = pickle.loads(self.action_socket.recv())
            print("1st frame: finish moving to offset position")
            # HOME <- Pos: [0.457632  0.0321814 0.2653815], Quat: [0.9998586  0.00880853 0.01421072 0.00179784]

            self.home_rot, self.home_pos = (
                transform_utils.quat2mat(robot_state.quat),
                robot_state.pos,
            )

            # robot init affine
            self.robot_init_H = np.eye(4)
            self.robot_init_H[:3, :3] = self.home_rot
            self.robot_init_H[:3, 3] = self.home_pos

            self.is_first_frame = False


        if self.start_teleop:
            moving_wrist = self._get_hand_frame()
            while (moving_wrist is None):
                moving_wrist = self._get_hand_frame()
    
            origin = moving_wrist[0]
            x_axis = moving_wrist[1]
            y_axis = moving_wrist[2]
            z_axis = moving_wrist[3]
            rot_180 = R.from_rotvec(np.pi * y_axis / np.linalg.norm(y_axis)).as_matrix()
            x_rot = rot_180 @ x_axis
            y_rot = rot_180 @ y_axis
            z_rot = rot_180 @ z_axis
            rotated_frame = [origin, x_rot, y_rot, z_rot]
            self.hand_moving_H = self._turn_frame_to_homo_mat(rotated_frame)

            # Transformation code
            # all 4x4 matrix
            H_HI_HH = copy(self.hand_init_H) # Homo matrix that takes P_HI  to P_HH - Point in Inital Hand Frame to Point in current hand Frame
            H_HT_HH = copy(self.hand_moving_H) # changing Homo matrix that takes P_HT to P_HH
            H_RI_RH = copy(self.robot_init_H) # not change robot home pos; Homo matrix that takes P_RI to P_RH

            
            self.robot_moving_H = self._to_robot_frame(H_HI_HH, H_HT_HH)
            relative_affine = self.robot_moving_H

            # Use a Filter
            if self.use_filter:
                relative_affine = self.comp_filter(relative_affine)
            # print("home_pose", self.home_pos)
            relative_pos = relative_affine[:3, 3]
            relative_rot = relative_affine[:3, :3]

            ##
            hand_axes_mat = self.hand_init_H[:3, :3].copy()
            angles = self._angles_around_axes(relative_rot, hand_axes_mat)
            print("Raw angles:", angles)
            # calculate hand angles
            hand_angles = angles.copy()
            hand_angles[2] = self._scale_angle(hand_angles[2], -25, 25)   # palm normal
            hand_angles[0] = self._scale_angle(hand_angles[0], -60, 0)  # side axis
            hand_angles[1] = 0
            print("Hand angles:", hand_angles)

            # rebuild hand matrix and find residual for arm
            R_hand_limited = self._rot_from_hand_axes(hand_angles, hand_axes_mat)
            R_arm_compensation = relative_rot @ R_hand_limited.T
            target_rot = self.home_rot @ R_arm_compensation
            ##
            
            target_pos = self.home_pos + relative_pos
            target_quat = transform_utils.mat2quat(target_rot)


            target_pos = np.clip(
                target_pos,
                a_min=ROBOT_WORKSPACE_MIN,
                a_max=ROBOT_WORKSPACE_MAX,
            )

        else:
            target_pos, target_quat = (
                self.home_pos + self.home_offset,
                transform_utils.mat2quat(self.home_rot),
            )

        print("send action")
        
        action = FrankaAction(
            pos=target_pos.flatten().astype(np.float32),
            quat=target_quat.flatten().astype(np.float32),
            gripper=self.gripper_state,
            reset=False,
            timestamp=time.time(),
        )

        if self.start_teleop: 
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
        else:
            self.action_socket.send(b"get_state")
    
        robot_state = self.action_socket.recv()
        robot_state = pickle.loads(robot_state)
        robot_state.start_teleop = self.start_teleop
        # self.state_socket.send(bytes(pickle.dumps(robot_state, protocol=-1)))
        self.state_socket.pub_keypoints(robot_state, "robot_state")
        self.commanded_state_socket.pub_keypoints(action, "commanded_robot_state")

    def stream(self):
        notify_component_start("Franka teleoperator control")
        print("Start controlling the robot arm using the Oculus Headset.\n")
        try:
            while True:
                # Retargeting function
                print("start teleop", self.start_teleop)
                self._apply_retargeted_angles()
        except KeyboardInterrupt:
            pass
        finally:
            self._transformed_arm_keypoint_subscriber.stop()
            self.action_socket.close()

        print("Stopping the teleoperator!")


def main():
    operator = FrankaArmOperator(save_states=True)
    operator.stream()


if __name__ == "__main__":
    main()