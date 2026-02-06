import socket
import zmq

def run_relay():
    context = zmq.Context()

    # 1. Setup ZMQ to talk to your Teleop Script
    # We CONNECT because your Teleop script is BINDING to 9095 and 9096
    right_push = context.socket(zmq.PUSH)
    right_push.connect("tcp://127.0.0.1:9095")
    
    left_push = context.socket(zmq.PUSH)
    left_push.connect("tcp://127.0.0.1:9096")

    # 2. Setup RAW UDP to talk to the Oculus Quest App
    # We use a standard socket because Unity/Oculus sends raw UDP
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_sock.bind(('0.0.0.0', 8095))
        print("Successfully listening for Quest UDP on 8095")
    except Exception as e:
        print(f"Error: Could not bind to 8095. Make sure no other script is using it: {e}")
        return

    print("Relay Active: Quest(UDP:8095) -> Right(ZMQ:9095) & Left(ZMQ:9096)")

    while True:
        # Receive the raw data from the Quest
        data, addr = udp_sock.recvfrom(4096)
        
        # Wrap it in a ZMQ message and send it to your detectors
        right_push.send(data)
        left_push.send(data)

if __name__ == "__main__":
    run_relay()