import numpy as np
from copy import deepcopy as copy
from frankateach.constants import *
from frankateach.vectorops import *
from frankateach.network import ZMQKeypointPublisher, ZMQKeypointSubscriber,ZMQButtonFeedbackSubscriber
from frankateach.utils import notify_component_start, FrequencyTimer
from ruka_hand.utils.constants import HAND_JOINTS
import h5py

class TransformHandPositionCoords():
    def __init__(self, keypoint_port, transformation_port,moving_average_limit = 5):
        notify_component_start('keypoint position transform')
        
        # Initializing the subscriber for right hand keypoints
        self.original_keypoint_subscriber = ZMQKeypointSubscriber(LOCALHOST, keypoint_port, 'right')
        # Initializing the publisher for transformed right hand keypoints
        self.transformed_keypoint_publisher = ZMQKeypointPublisher(LOCALHOST, transformation_port)
        # Timer
        self.timer = FrequencyTimer(VR_FREQ)
        # Keypoint indices for knuckles
        self.knuckle_points = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][-1])
        # Moving average queue
        self.moving_average_limit = moving_average_limit
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []

    # Function to get the hand coordinates from the VR
    def _get_hand_coords(self):
        data = self.original_keypoint_subscriber.recv_keypoints()
        if data[0] == 0:
            data_type = 'absolute'
        else:
            data_type = 'relative'
        return data_type, np.asanyarray(data[1:]).reshape(OCULUS_NUM_KEYPOINTS, 3)
    
    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[0]

    # Create a coordinate frame for the hand
    def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord):
        palm_normal = normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Current Z
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Current Y
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))              # Current X
        return [cross_product, palm_direction, palm_normal]

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
    
    # Create a coordinate frame for the arm 
    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord):

        palm_normal = normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Unity space - Y
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = normalize_vector(index_knuckle_coord - pinky_knuckle_coord)              # Unity space - X
        
        return [origin_coord, cross_product, palm_normal, palm_direction]

    def transform_keypoints(self, hand_coords):
        translated_coords = self._translate_coords(hand_coords)
        original_coord_frame = self._get_coord_frame(
            translated_coords[self.knuckle_points[0]], 
            translated_coords[self.knuckle_points[1]]
        )

        # Finding the rotation matrix and rotating the coordinates
        # rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
        # transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
        transformed_hand_coords = translated_coords
        
        # For teleoperating ruka hand
        ordered_joints = self._get_ordered_joints(transformed_hand_coords)
        
        hand_dir_frame = self._get_hand_dir_frame(
            hand_coords[0],
            translated_coords[self.knuckle_points[0]], 
            translated_coords[self.knuckle_points[1]]
        )

        return ordered_joints, hand_dir_frame

    def stream(self):
        while True:
            try:
                self.timer.start_loop()
                print("started")
                data_type, hand_coords = self._get_hand_coords()
                print("recvd")
                self.logging_enabled = True
                # Shift the points to required axes
                transformed_hand_coords, translated_hand_coord_frame = self.transform_keypoints(hand_coords)
                # Passing the transformed coords into a moving average
                self.averaged_hand_coords = moving_average(
                    transformed_hand_coords, 
                    self.coord_moving_average_queue, 
                    self.moving_average_limit
                )

                self.averaged_hand_frame = moving_average(
                    translated_hand_coord_frame, 
                    self.frame_moving_average_queue, 
                    self.moving_average_limit
                )
                self.transformed_keypoint_publisher.pub_keypoints(self.averaged_hand_coords, 'transformed_hand_coords')
                if data_type == 'absolute':
                    self.transformed_keypoint_publisher.pub_keypoints(self.averaged_hand_frame, 'transformed_hand_frame')

                self.timer.end_loop()
            except:
                break
        
        self.original_keypoint_subscriber.stop()
        self.transformed_keypoint_publisher.stop()

        print('Stopping the keypoint position transform process.')