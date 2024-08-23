from pathlib import Path
import pickle
import cv2
import time
import threading

from frankateach.network import (
    ZMQCameraSubscriber,
    create_response_socket,
)
from frankateach.utils import notify_component_start


from frankateach.constants import (
    HOST,
    CAM_PORT,
    STATE_PORT,
    DEPTH_PORT_OFFSET,
)


class DataCollector:
    def __init__(
        self,
        storage_path: str,
        demo_num: int,
        cams=[],  # cam serial numbers
        cam_config=None,  # cam config
        collect_state=False,
        collect_depth=False,
    ):
        self.image_subscribers = []
        self.depth_subscribers = []
        for cam_idx, _ in enumerate(cams):
            self.image_subscribers.append(
                ZMQCameraSubscriber(HOST, CAM_PORT + cam_idx, "RGB")
            )

        if collect_depth:
            for cam_idx, _ in enumerate(cams):
                self.depth_subscribers.append(
                    ZMQCameraSubscriber(
                        HOST, CAM_PORT + DEPTH_PORT_OFFSET + cam_idx, "Depth"
                    )
                )
        if collect_state:
            self.state_socket = create_response_socket(HOST, STATE_PORT)

        # Create the storage directory
        self.storage_path = Path(storage_path) / f"demonstration_{demo_num}"
        self.storage_path.mkdir(parents=True, exist_ok=True)

        self.run_event = threading.Event()
        self.run_event.set()
        self.threads = []

        for cam_idx, _ in enumerate(cams):
            self.threads.append(
                threading.Thread(
                    target=self.save_rgb,
                    args=(cam_idx, cam_config),
                    daemon=True,
                )
            )

            if collect_depth:
                self.threads.append(
                    threading.Thread(
                        target=self.save_depth,
                        args=(cam_idx, cam_config),
                        daemon=True,
                    )
                )

        if collect_state:
            self.threads.append(threading.Thread(target=self.save_states, daemon=True))

    def start(self):
        for thread in self.threads:
            thread.start()

        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("Stopping the data collection...")
            self.run_event.clear()
            for thread in self.threads:
                thread.join()

    def save_rgb(self, cam_idx, cam_config):
        notify_component_start(component_name="RGB Image Collector")

        filename = self.storage_path / f"cam_{cam_idx}_rgb_video.avi"
        metadata_filename = self.storage_path / f"cam_{cam_idx}_rgb_video.metadata"

        recorder = cv2.VideoWriter(
            str(filename),
            cv2.VideoWriter_fourcc(*"XVID"),
            cam_config.fps,
            (cam_config.width, cam_config.height),
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
        num_image_frames = 0

        while self.run_event.is_set():
            rgb_image, timestamp = self.image_subscribers[cam_idx].recv_rgb_image()
            # Save the image
            recorder.write(rgb_image)
            timestamps.append(timestamp)
            num_image_frames += 1

        print("finished recording")

        record_end_time = time.time()
        metadata["record_end_time"] = record_end_time
        metadata["num_image_frames"] = num_image_frames
        metadata["timestamps"] = timestamps
        recorder.release()
        with open(metadata_filename, "wb") as f:
            pickle.dump(metadata, f)
        self.image_subscribers[cam_idx].stop()

    def save_depth(self, cam_idx, cam_config):
        pass

    def save_states(self):
        notify_component_start(component_name="State Collector")

        filename = self.storage_path / "states.pkl"
        states = []

        while self.run_event.is_set():
            state = pickle.loads(self.state_socket.recv())
            self.state_socket.send(b"ok")
            states.append(state)

        print("Saving states...")
        with open(filename, "wb") as f:
            pickle.dump(states, f)
        self.state_socket.close()


#  def save_controls(self):
#      notify_component_start(component_name="Control Collector")

#      filename = self.storage_path / "controls.pkl"
#      controls = []

#      while self.run_event.is_set():
#          control = self.control_subscriber.recv_keypoints()
#          print(control)
#          controls.append(control)

#      with open(filename, "wb") as f:
#          pickle.dump(controls, f)
#      self.control_subscriber.stop()
