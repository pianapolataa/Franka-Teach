import socket
import zmq

def start_bridge():
    context = zmq.Context()
    
    # Setup ZMQ Pushers (These connect to your Teleop PULL sockets)
    # We use CONNECT here because your code will BIND to the internal ports
    right_hand_pub = context.socket(zmq.PUSH)
    right_hand_pub.connect("tcp://127.0.0.1:9087")
    
    left_hand_pub = context.socket(zmq.PUSH)
    left_hand_pub.connect("tcp://127.0.0.1:9110")

    button_pub = context.socket(zmq.PUSH)
    button_pub.connect("tcp://127.0.0.1:9095") # For Right Hand process
    button_pub_left = context.socket(zmq.PUSH)
    button_pub_left.connect("tcp://127.0.0.1:9096") # For Left Hand process

    # Setup UDP Listeners (To catch the Oculus App)
    udp_ports = [8087, 8110, 8095]
    socks = {}
    for p in udp_ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('0.0.0.0', p))
        s.setblocking(False)
        socks[p] = s

    print("Bridge Active. Quest (UDP) -> Teleop (ZMQ)")

    while True:
        for p, s in socks.items():
            try:
                data, addr = s.recvfrom(4096)
                if p == 8087: right_hand_pub.send(data)
                elif p == 8110: left_hand_pub.send(data)
                elif p == 8095: 
                    button_pub.send(data)
                    button_pub_left.send(data) # Split the button to both!
            except BlockingIOError:
                continue

if __name__ == "__main__":
    start_bridge()