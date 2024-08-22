import hydra
from frankateach.data_collector import DataCollector


@hydra.main(version_base="1.2", config_path="configs", config_name="collect_data")
def main(cfg):
    data_collector = DataCollector(
        storage_path=cfg.storage_path,
        demo_num=cfg.demo_num,
        cams=cfg.cam_serial_numbers,
        cam_config=cfg.cam_config,
        collect_depth=cfg.collect_depth,
        collect_state=cfg.collect_state,
        collect_control=cfg.collect_control,
    )
    data_collector.start()


if __name__ == "__main__":
    main()
