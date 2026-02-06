import zmq
import threading
from frankateach.network import create_pull_socket
from frankateach.constants import INTERNAL_IP

def relay_worker(source_port, target_ports):
    context = zmq.Context()
    
    # 1. Catch the data from the Quest
    # This BINDS to the original port (e.g., 8095)
    receiver = create_pull_socket(INTERNAL_IP, source_port)
    
    # 2. Prepare the split ports
    # We use CONNECT because the OculusVRHandDetector class BINDS to its ports
    pushers = []
    for port in target_ports:
        p = context.socket(zmq.PUSH)
        p.connect(f"tcp://127.0.0.1:{port}") 
        pushers.append(p)
    
    print(f"Relay started: Listening on {source_port} -> Feeding {target_ports}")
    
    while True:
        # Get raw data (e.g., b'low')
        message = receiver.recv()
        
        # Send that EXACT same byte string to the new ports
        for p in pushers:
            p.send(message)

if __name__ == "__main__":
    # Split Button (8095) into two local ports for the two hand detectors
    t1 = threading.Thread(target=relay_worker, args=(8095, [9095, 9096]))
    
    # Split Reset (8100) into two local ports
    t2 = threading.Thread(target=relay_worker, args=(8100, [9100, 9101]))
    
    t1.start()
    t2.start()

    print("\n--- Splitter Active ---")
    print("Point your Right Hand config to ports 9095 and 9100")
    print("Point your Left Hand config to ports 9096 and 9101")