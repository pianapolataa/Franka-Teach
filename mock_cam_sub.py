import numpy as np
import time
import cv2

from frankateach.utils import FrequencyTimer
from frankateach.network import ZMQCameraSubscriber
from frankateach.messages import FrankaAction

cam_port = 10005
timer = FrequencyTimer(30)
cam_sub = ZMQCameraSubscriber("localhost", cam_port, "RGB")

while True:
    timer.start_loop()
    image, timestamp = cam_sub.recv_rgb_image()
    print(image.shape)
    print(timestamp)
    cv2.imwrite("image.jpg", image)
    timer.end_loop()
