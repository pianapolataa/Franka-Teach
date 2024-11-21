from frankateach.camera_server import FishEyeServer
from frankateach.constants import HOST, FISHEYE_CAM_PORT
import hydra
import argparse


@hydra.main(version_base="1.2", config_path="configs", config_name="fisheyecamera")
def main(cfg):
    cam_configs = [
        argparse.Namespace(
            cam_serial_num=serial,
            fps=cfg.fisheye_cam_config.fps,
            height=cfg.fisheye_cam_config.height,
            width=cfg.fisheye_cam_config.width,
        )
        for i, serial in cfg.fisheye_cam_numbers.items()
    ]

    # Initialize FishEyeServer
    server = FishEyeServer(
        host=HOST,
        cam_port=FISHEYE_CAM_PORT,
        cam_configs=cam_configs,
    )

    server._init_camera_threads()  # Start the camera threads
    server.start()


if __name__ == "__main__":
    main()
