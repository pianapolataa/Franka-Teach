import hydra
from frankateach.sensors.reskin import ReskinSensorPublisher


@hydra.main(config_path="configs", config_name="reskin")
def main(cfg):
    reskin_publisher = ReskinSensorPublisher(reskin_config=cfg.reskin_config)
    reskin_publisher.stream()


if __name__ == "__main__":
    main()
