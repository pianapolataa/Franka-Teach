from frankateach.messages import FrankaState
from frankateach.network import ZMQKeypointSubscriber


subscriber = ZMQKeypointSubscriber(host="localhost", port=8900, topic="state")

while True:
    robot_state: FrankaState = subscriber.recv_keypoints()
    print(
        f"Pos: {robot_state.pos.round(2)} | Quat: {robot_state.quat.round(2)} | Gripper: {robot_state.gripper} | Timestamp: {robot_state.timestamp}"
    )
