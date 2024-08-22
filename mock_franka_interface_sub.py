import numpy as np
import time

from frankateach.utils import FrequencyTimer
from frankateach.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from frankateach.messages import FrankaState

timer = FrequencyTimer(90)
sub = ZMQKeypointSubscriber("localhost", 8901, "control")

while True:
    timer.start_loop()
    action = sub.recv_keypoints()
    print(f"Received {action}")
    timer.end_loop()
