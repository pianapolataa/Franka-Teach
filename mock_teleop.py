import numpy as np
import time

from frankateach.utils import FrequencyTimer
from frankateach.network import ZMQKeypointPublisher
from frankateach.messages import FrankaAction

timer = FrequencyTimer(90)
publisher = ZMQKeypointPublisher("localhost", 8901)

while True:
    timer.start_loop()
    state = FrankaAction(
        pos=np.random.rand(3).astype(np.float32),
        quat=np.random.rand(4).astype(np.float32),
        gripper=0,
        reset=True,
        timestamp=time.time(),
    )
    publisher.pub_keypoints(state, "control")
    print(f"Published {state}")
    timer.end_loop()
