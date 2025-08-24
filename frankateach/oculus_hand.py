from frankateach.constants import *
from frankateach.utils import FrequencyTimer
from frankateach.network import create_pull_socket, ZMQKeypointPublisher, ZMQKeypointSubscriber
from frankateach.utils import parse_controller_state, notify_component_start
import zmq
'''
Receive hand keypoints from the Oculus VR controller and publish them.
If self.start_teleop = True, it will start collecting data
'''
class OculusVRHandDetector():
    def __init__(self, oculus_port, keypoint_pub_port, button_port, button_publish_port, teleop_reset_port, teleop_reset_publish_port):
        notify_component_start("vr detector for hand tracking")

        # Initializing the network socket for getting the raw right hand keypoints
        print(INTERNAL_IP, oculus_port) 
        self.raw_keypoint_socket = create_pull_socket(INTERNAL_IP, oculus_port)
        self.button_keypoint_socket = create_pull_socket(INTERNAL_IP, button_port)
        self.teleop_reset_socket = create_pull_socket(INTERNAL_IP, teleop_reset_port)
        
         # ZMQ Keypoint publisher
        self.hand_keypoint_publisher = ZMQKeypointPublisher(
            host = LOCALHOST,
            port = keypoint_pub_port
        )

        # Socket For Resolution Button
        self.button_socket_publisher = ZMQKeypointPublisher(
            host =LOCALHOST,
            port =button_publish_port
        ) 
        # Socket For Teleop Reset
        self.pause_info_publisher = ZMQKeypointPublisher(
            host =LOCALHOST,
            port =teleop_reset_publish_port
        ) 
        self.timer = FrequencyTimer(VR_FREQ) 
        
    # Function to process the data token received from the VR
    def _process_data_token(self, data_token):
        return data_token.decode().strip()

    # Function to Extract the Keypoints from the String Token sent by the VR
    def _extract_data_from_token(self, token):        
        data = self._process_data_token(token)
        information = dict()
        keypoint_vals = [0] if data.startswith('absolute') else [1]
        # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
        vector_strings = data.split(':')[1].strip().split('|')
        for vector_str in vector_strings:
            vector_vals = vector_str.split(',')
            for float_str in vector_vals[:3]:
                keypoint_vals.append(float(float_str))
            
        information['keypoints'] = keypoint_vals
        return information

    # Function to Publish the transformed Keypoints
    def _publish_data(self, keypoint_dict):
        self.hand_keypoint_publisher.pub_keypoints(
            keypoint_array = keypoint_dict['keypoints'], 
            topic_name = 'right'
        )

    # Function to Publish the Resolution Button Feedback
    def _publish_button_data(self,button_feedback):
        self.button_socket_publisher.pub_keypoints(
            keypoint_array = button_feedback, 
            topic_name = 'button'
        )

    # Function to Publish the Teleop Reset Status
    def _publish_pause_data(self,pause_status):
        self.pause_info_publisher.pub_keypoints(
            keypoint_array = pause_status, 
            topic_name = 'pause'
        )

    # Function to Stream the Keypoints
    def stream(self):
        i = 0
        while True:
            try:
                self.timer.start_loop()
                
                raw_keypoints = self.raw_keypoint_socket.recv()
                button_feedback = self.button_keypoint_socket.recv()
                pause_status = self.teleop_reset_socket.recv()


                if button_feedback==b'low':
                    button_feedback_num = ARM_LOW_RESOLUTION
                else:
                    button_feedback_num = ARM_HIGH_RESOLUTION

                if pause_status==b'Low':
                    pause_status = ARM_TELEOP_STOP 
                else:
                    pause_status = ARM_TELEOP_CONT
 
                keypoint_dict = self._extract_data_from_token(raw_keypoints)

                self._publish_data(keypoint_dict)
                self._publish_button_data(button_feedback_num)
                self._publish_pause_data(pause_status)
                self.timer.end_loop()
                i +=1
            except:
                break

        self.raw_keypoint_socket.close()
        self.hand_keypoint_publisher.stop()

        print('Stopping the oculus keypoint extraction process.')