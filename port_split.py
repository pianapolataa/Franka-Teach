import zmq
import threading
from frankateach.network import create_pull_socket
from frankateach.constants import INTERNAL_IP

def relay_worker(source_port, target_ports):
    context = zmq.Context()
    
    # 1. Catch the data from the Quest
    receiver = create_pull_socket(INTERNAL_IP, source_port)
    
    # 2. Setup dedicated Pushers
    # By maintaining a list and looping, we send to EVERYONE
    pushers = []
    for port in target_ports:
        p = context.socket(zmq.PUSH)
        p.connect(f"tcp://172.24.71.240:{port}") 
        pushers.append(p)
    
    print(f"Relay started: {source_port} BROADCASTING to {target_ports}")
    
    while True:
        # Get raw data from Quest
        message = receiver.recv()
        
        # MANUALLY BROADCAST: Send the exact same message to EVERY pusher
        for p in pushers:
            # We use NOBLOCK here to ensure one slow hand doesn't stop the whole relay
            try:
                p.send(message, flags=zmq.NOBLOCK)
            except zmq.Again:
                pass 

if __name__ == "__main__":
    # Split Button (8095) -> Explicitly to both 9095 AND 9096
    threading.Thread(target=relay_worker, args=(8095, [9095, 9096]), daemon=True).start()
    
    # Split Reset (8100) -> Explicitly to both 9100 AND 9101
    threading.Thread(target=relay_worker, args=(8100, [9100, 9101]), daemon=True).start()
    
    import time
    while True:
        time.sleep(1)