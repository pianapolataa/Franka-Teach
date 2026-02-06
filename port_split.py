import zmq
import threading
from frankateach.network import create_pull_socket
from frankateach.constants import INTERNAL_IP

def relay_worker(source_port, target_ports):
    context = zmq.Context()
    
    # 1. Receive from the Quest (Port 8095 or 8100)
    # This script BINDS to the Oculus ports
    receiver = create_pull_socket(INTERNAL_IP, source_port)
    
    # 2. Setup Pushers for the Teleop Code
    # We use PUSH because the teleop script is a PULL socket
    # We BIND to the new ports so teleop can connect/bind as usual
    pushers = []
    for port in target_ports:
        pusher = context.socket(zmq.PUSH)
        pusher.bind(f"tcp://127.0.0.1:{port}")
        pushers.append(pusher)
    
    print(f"Relay Active: {source_port} -> {target_ports}")
    
    while True:
        # Get data from Quest
        message = receiver.recv()
        # Send a copy to every target port
        for p in pushers:
            p.send(message)

if __name__ == "__main__":
    # Split Button Port 8095 -> 9095 (Right Hand) and 9096 (Left Hand)
    t1 = threading.Thread(target=relay_worker, args=(8095, [9095, 9096]))
    
    # Split Reset Port 8100 -> 9100 (Right Hand) and 9101 (Left Hand)
    t2 = threading.Thread(target=relay_worker, args=(8100, [9100, 9101]))
    
    t1.start()
    t2.start()
    
    print("Splitter is running. Update your YAML to use ports 9095-9101.")