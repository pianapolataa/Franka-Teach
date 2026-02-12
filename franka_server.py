from frankateach.franka_server import FrankaServer
import hydra

@hydra.main(version_base="1.2", config_path="configs", config_name="franka_server")
def main(cfg):
    fs_right = FrankaServer(cfg.deoxys_config_path, cfg.hand)
    fs_right.init_server()


if __name__ == "__main__":
    main()