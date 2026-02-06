import hydra
from multiprocessing import Process
from hydra.utils import instantiate
from frankateach.teleoperator_stick import FrankaOperator
from frankateach.oculus_stick import OculusVRStickDetector


def start_oculus_stick(cfg):
    detector = instantiate(cfg.oculus_stick_detector)
    detector.stream()
    

def start_oculus_hand(cfg):
    detector = instantiate(cfg.oculus_hand_detector)
    detector.stream()

def start_oculus_hand_left(cfg):
    detector = instantiate(cfg.oculus_hand_detector_left)
    detector.stream()

def start_hand_transform(cfg):
    transformer = instantiate(cfg.hand_transform)
    transformer.stream()

def start_hand_transform_left(cfg):
    transformer = instantiate(cfg.hand_transform_left)
    transformer.stream()
    
def start_teleop_stick(cfg):
    operator = instantiate(cfg.stick_operators)
    operator.stream()

def start_teleop_arm(cfg):
    operator = instantiate(cfg.arm_operators)
    operator.stream()

def start_teleop_arm_left(cfg):
    operator = instantiate(cfg.arm_operators_left)
    operator.stream()
    
def start_teleop_hand(cfg):
    ruka = instantiate(cfg.ruka_operators)
    ruka.stream()



@hydra.main(version_base="1.2", config_path="configs", config_name="teleop")
def main(cfg):
    processes = []
    
    if cfg.use_oculus_stick:
        oculus_process = Process(target=start_oculus_stick, args=(cfg,))
        teleop_process = Process(target=start_teleop_stick, args=(cfg,))
    else:
        oculus_process = Process(target=start_oculus_hand, args=(cfg,))
        # oculus_process_left = Process(target=start_oculus_hand_left, args=(cfg,))
        # transform_process = Process(target=start_hand_transform, args=(cfg,))
        # teleop_process_left = Process(target=start_teleop_arm_left, args=(cfg,))
        # teleop_process_right = Process(target=start_teleop_arm, args=(cfg,))
        if cfg.use_hand_tracking:
            ruka_process = Process(target=start_teleop_hand, args=(cfg,))
            processes.append(ruka_process)
        # processes.append(transform_process)
    
    processes.append(oculus_process)
    # processes.append(oculus_process_left)
    # processes.append(teleop_process_left)
    # processes.append(teleop_process_right)

    for p in processes:
        p.start()

    for p in processes:
        p.join()


if __name__ == "__main__":
    main()