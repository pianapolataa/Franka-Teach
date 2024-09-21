from frankateach.camera_server import RealsenseServer
from frankateach.constants import HOST, CAM_PORT
import hydra
import argparse


@hydra.main(version_base="1.2", config_path="configs", config_name="camera")
def main(cfg):
    cam_configs = [
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

    server = RealsenseServer(
        host=HOST,
        cam_port=CAM_PORT,
        cam_configs=cam_configs,
    )

    server._init_camera_threads()


if __name__ == "__main__":
    main()