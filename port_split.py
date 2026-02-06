import socket
import zmq

def run_hybrid_relay():
    # 1. Setup ZMQ to talk to your Teleop Script
    context = zmq.Context()
    
    # We use PUSH because your detectors are using PULL
    # We CONNECT because your detectors are BINDING to 9095/9096
    right_push = context.socket(zmq.PUSH)
    right_push.connect("tcp://127.0.0.1:9095")
    
    left_push = context.socket(zmq.PUSH)
    left_push.connect("tcp://127.0.0.1:9096")

    # 2. Setup UDP to talk to the Oculus Quest
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_sock.bind(('0.0.0.0', 8095))
        print("Success: Listening for Quest UDP data on 8095")
    except Exception as e:
        print(f"Error: Could not bind to 8095. Is another script running? {e}")
        return

    print("Forwarding Quest(UDP:8095) -> Right(ZMQ:9095) & Left(ZMQ:9096)")

    while True:
        # Receive raw bytes from Quest
        data, addr = udp_sock.recvfrom(4096)
        
        # Forward via ZMQ to both detectors
        right_push.send(data)
        left_push.send(data)

if __name__ == "__main__":
    run_hybrid_relay()