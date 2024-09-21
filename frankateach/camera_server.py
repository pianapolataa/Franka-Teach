import pyrealsense2 as rs
import time
import threading
import argparse

from frankateach.sensors.realsense import RealsenseCamera


class RealsenseServer:
    def __init__(self, host, cam_port, cam_configs):
        self._host = host
        self._cam_port = cam_port
        self._cam_configs = cam_configs

        ctx = rs.context()
        devices = ctx.query_devices()

        for dev in devices:
            dev.hardware_reset()

        print("Waiting for hardware reset on cameras for 15 seconds...")
        time.sleep(15)

        self.cam_threads = []

    def _start_component(self, cam_idx, cam_config):
        component = RealsenseCamera(
            host=self._host,
            port=self._cam_port + cam_idx,
            cam_id=cam_idx,
            cam_config=cam_config,
        )
        component.stream()

    def _init_camera_threads(self):
        for cam_idx, cam_config in enumerate(self._cam_configs):
            cam_thread = threading.Thread(
                target=self._start_component,
                args=(cam_idx, cam_config),
                daemon=True,
            )
            cam_thread.start()
            self.cam_threads.append(cam_thread)

        for cam_thread in self.cam_threads:
            cam_thread.join()



