from frankateach.camera_server import FishEyeServer, RealsenseServer
from frankateach.constants import FISHEYE_CAM_PORT, HOST, CAM_PORT
import hydra
import argparse


@hydra.main(version_base="1.2", config_path="configs", config_name="camera")
def main(cfg):
    realsense_cam_configs = [
        argparse.Namespace(
            cam_serial_num=cfg.cam_serial_numbers[i],
            depth=cfg.cam_config.depth,
            fps=cfg.cam_config.fps,
            height=cfg.cam_config.height,
            width=cfg.cam_config.width,
            processing_preset=cfg.cam_config.processing_preset,
        )
        for i in range(len(cfg.cam_serial_numbers))
    ]

    realsense_server = RealsenseServer(
        host=HOST,
        cam_port=CAM_PORT,
        cam_configs=realsense_cam_configs,
    )

    fisheye_cam_configs = [
        argparse.Namespace(
            cam_serial_num=serial_num,
            fps=cfg.fisheye_cam_config.fps,
            height=cfg.fisheye_cam_config.height,
            width=cfg.fisheye_cam_config.width,
        )
        for _, serial_num in range(len(cfg.fisheye_cam_numbers))
    ]

    fisheye_server = FishEyeServer(
        host=HOST,
        cam_port=FISHEYE_CAM_PORT,
        cam_configs=fisheye_cam_configs,
    )

    realsense_server._init_camera_threads()
    fisheye_server._init_camera_threads()


if __name__ == "__main__":
    main()
