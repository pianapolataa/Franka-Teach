from collections import defaultdict
from pathlib import Path
import pickle
import cv2
import time
import threading
import h5py
import numpy as np

from frankateach.network import (
    ZMQCameraSubscriber,
    ZMQKeypointSubscriber,
)
# from frankateach.sensors.reskin import ReskinSensorSubscriber
from frankateach.utils import notify_component_start

from frankateach.constants import (
    COMMANDED_STATE_PORT,
    LOCALHOST,
    CAM_PORT,
    STATE_PORT,
    DEPTH_PORT_OFFSET,
    RESKIN_STREAM_PORT,
    RUKA_COMMANDED_STATE_PORT,
    RUKA_STATE_PORT
)


class DataCollector:
    def __init__(
        self,
        storage_path: str,
        demo_num: int,
        cams=[],  # camera info dicts
        cam_config=None,  # camera configs by type
        collect_img=False,
        collect_state=False,
        collect_depth=False,
        collect_ruka=False,
        collect_reskin=False,
    ):
        self.image_subscribers = {}
        self.depth_subscribers = {}
        if collect_img:
            for camera in cams:
                self.image_subscribers[camera.cam_id] = ZMQCameraSubscriber(
                    LOCALHOST, CAM_PORT + camera.cam_id, "RGB"
                )

        if collect_depth:
            for camera in cams:
                if camera.type == "realsense":
                    self.depth_subscribers[camera.cam_id] = ZMQCameraSubscriber(
                        LOCALHOST, CAM_PORT + DEPTH_PORT_OFFSET + camera.cam_id, "Depth"
                    )

        if collect_state:
            self.state_socket = ZMQKeypointSubscriber(
                host=LOCALHOST, port=STATE_PORT, topic="robot_state"
            )
            # self.state_socket = create_response_socket(HOST, STATE_PORT)
            self.commanded_state_socket = ZMQKeypointSubscriber(
                host=LOCALHOST, port=COMMANDED_STATE_PORT, topic="commanded_robot_state"
            )

        if collect_reskin:
            self.reskin_subscriber = ReskinSensorSubscriber()

        if collect_ruka:
            self.ruka_socket = ZMQKeypointSubscriber(
                host=LOCALHOST, port=RUKA_STATE_PORT, topic="ruka_state"
            )
            self.ruka_commanded_state_socket = ZMQKeypointSubscriber(
                host=LOCALHOST, port=RUKA_COMMANDED_STATE_PORT, topic="commanded_ruka_state"
            )

        # Create the storage directory
        self.storage_path = Path(storage_path) / f"demonstration_{demo_num}"
        self.storage_path.mkdir(parents=True, exist_ok=True)
        print("Storage path: ", self.storage_path)

        self.run_event = threading.Event()
        self.run_event.set()
        self.threads = []

        # Set up image subscribers
        for camera in cams:
            if collect_img:
                self.threads.append(
                    threading.Thread(
                        target=self.save_rgb,
                        args=(camera.cam_id, cam_config[camera.type]),
                        daemon=True,
                    )
                )
            if collect_depth and camera.type == "realsense":
                self.threads.append(
                    threading.Thread(
                        target=self.save_depth,
                        args=(camera.cam_id, cam_config[camera.type]),
                        daemon=True,
                    )
                )

        if collect_state:
            self.threads.append(threading.Thread(target=self.save_states, daemon=True))

        if collect_reskin:
            self.threads.append(threading.Thread(target=self.save_reskin, daemon=True))

        if collect_ruka:
            self.threads.append(threading.Thread(target=self.save_ruka_states, daemon=True))

    def start(self):
        for thread in self.threads:
            thread.start()
        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("Stopping data collection...")
            self.run_event.clear()  # Ensure this clears the event to stop threads
            for thread in self.threads:
                thread.join()

    def save_rgb(self, cam_idx, cam_config):
        notify_component_start(component_name="RGB Image Collector")

        filename = self.storage_path / f"cam_{cam_idx}_rgb_video.avi"
        metadata_filename = self.storage_path / f"cam_{cam_idx}_rgb_video.metadata"

        recorder = cv2.VideoWriter(
            str(filename),
            cv2.VideoWriter_fourcc(*"XVID"),
            cam_config["fps"],
            (cam_config["width"], cam_config["height"]),
        )

        timestamps = []
        metadata = dict(
            cam_idx=cam_idx,
            width=cam_config.width,
            height=cam_config.height,
            fps=cam_config.fps,
            filename=filename,
            record_start_time=time.time(),
        )

        try:
            # Loop to capture frames until stopped
            while self.run_event.is_set():
                rgb_image, timestamp = self.image_subscribers[cam_idx].recv_rgb_image()
                recorder.write(rgb_image)
                timestamps.append(timestamp)
        finally:
            # Ensure resources are released regardless of exit conditions
            recorder.release()
            metadata["record_end_time"] = time.time()
            metadata["num_image_frames"] = len(timestamps)
            metadata["timestamps"] = timestamps
            with open(metadata_filename, "wb") as f:
                pickle.dump(metadata, f)
            self.image_subscribers[cam_idx].stop()
            print(f"Saved video to {filename}")

    # def save_depth(self, cam_idx, cam_config):
    #     raise NotImplementedError("Depth recording is not yet implemented")

    def save_depth(self, cam_idx, cam_config):
        notify_component_start(component_name="Depth Image Collector")

        filename = self.storage_path / f"cam_{cam_idx}_depth.pkl"
        metadata_filename = self.storage_path / f"cam_{cam_idx}_depth.metadata"

        depth_frames = []

        timestamps = []
        metadata = dict(
            cam_idx=cam_idx,
            width=cam_config.width,
            height=cam_config.height,
            fps=cam_config.fps,
            filename=filename,
            record_start_time=time.time(),
        )

        try:
            # Loop to capture frames until stopped
            while self.run_event.is_set():
                depth_frame, timestamp = self.depth_subscribers[
                    cam_idx
                ].recv_depth_image()
                depth_frames.append(depth_frame)
                timestamps.append(timestamp)
        finally:
            # Ensure resources are released regardless of exit conditions
            metadata["record_end_time"] = time.time()
            metadata["num_image_frames"] = len(timestamps)
            metadata["timestamps"] = timestamps
            with open(metadata_filename, "wb") as f:
                pickle.dump(metadata, f)
            with open(filename, "wb") as f:
                pickle.dump(depth_frames, f)
            self.depth_subscribers[cam_idx].stop()
            print(f"Saved depth to {filename}")

    def save_states(self):
        notify_component_start(component_name="State Collector")

        filename = self.storage_path / "states.pkl"
        cmd_filename = self.storage_path / "commanded_states.pkl"
        states = []
        commanded_states = []

        while self.run_event.is_set():
            # state = pickle.loads(self.state_socket.recv())
            state = self.state_socket.recv_keypoints()
            commanded_state = self.commanded_state_socket.recv_keypoints()
            states.append(state)
            commanded_states.append(commanded_state)

        with open(filename, "wb") as f:
            pickle.dump(states, f)

        with open(cmd_filename, "wb") as f:
            pickle.dump(commanded_states, f)

        print("Saved states to ", filename)
        # self.state_socket.close()
        self.state_socket.stop()
        self.commanded_state_socket.stop()

        print(
            "Frequency of state savings: ",
            (len(states) - 10) / (states[-1].timestamp - states[10].timestamp),
        )

        print(
            "Frequency of commanded state savings: ",
            (len(commanded_states) - 10)
            / (commanded_states[-1].timestamp - commanded_states[10].timestamp),
        )

    def save_reskin(self):
        notify_component_start(component_name="Reskin Collector")

        sensor_information = defaultdict(list)
        filename = self.storage_path / "reskin_sensor_values.h5"

        print("Starting to record Reskin frames from port:", RESKIN_STREAM_PORT)

        while self.run_event.is_set():
            reskin_state = self.reskin_subscriber.get_sensor_state()
            for attr in reskin_state.keys():
                sensor_information[attr].append(reskin_state[attr])

        print("Finished recording Reskin frames")

        with h5py.File(filename, "w") as hf:
            for key in sensor_information.keys():
                sensor_information[key] = np.array(
                    sensor_information[key],
                    dtype=np.float32 if key != "timestamp" else np.float64,
                )
                hf.create_dataset(
                    key,
                    data=sensor_information[key],
                    compression="gzip",
                    compression_opts=6,
                )

        print("Saved Reskin sensor data to ", filename)
        print(
            "ReSkin Data duration: ",
            sensor_information["timestamp"][-1] - sensor_information["timestamp"][0],
        )

        print(
            "Frequency of ReSkin savings: ",
            len(sensor_information["timestamp"])
            / (
                sensor_information["timestamp"][-1] - sensor_information["timestamp"][0]
            ),
        )

    def save_ruka_states(self):
        notify_component_start(component_name="Ruka State Collector")

        filename = self.storage_path / "ruka_states.pkl"
        cmd_filename = self.storage_path / "ruka_commanded_states.pkl"
        ruka_states = []
        commanded_ruka_states = []

        print("Starting to record Ruka joint data...")

        while self.run_event.is_set():
            try:
                ruka_state = self.ruka_socket.recv_keypoints()
                commanded_ruka_state = self.ruka_commanded_state_socket.recv_keypoints()

                # optionally include timestamps if your teleop sends them
                ruka_states.append(ruka_state)
                commanded_ruka_states.append(commanded_ruka_state)
            except Exception as e:
                print("Ruka recv error:", e)
                continue

        with open(filename, "wb") as f:
            pickle.dump(ruka_states, f)
        with open(cmd_filename, "wb") as f:
            pickle.dump(commanded_ruka_states, f)

        print(f"Saved Ruka states to {filename}")
        self.ruka_socket.stop()
        self.ruka_commanded_state_socket.stop()

    
