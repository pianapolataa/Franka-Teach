import zmq
import threading
from frankateach.network import create_pull_socket
from frankateach.constants import INTERNAL_IP

def relay_worker(source_port, target_ports):
    context = zmq.Context()
    receiver = create_pull_socket(INTERNAL_IP, source_port)
    
    pushers = []
    for port in target_ports:
        p = context.socket(zmq.PUSH)
        p.connect(f"tcp://{INTERNAL_IP}:{port}") 
        pushers.append(p)
    
    print(f"Relay started: {source_port} BROADCASTING to {target_ports}")
    
    while True:
        # Get raw data from Quest
        message = receiver.recv()
        print("1")
        
        # MANUALLY BROADCAST: Send the exact same message to EVERY pusher
        for p in pushers:
            try:
                p.send(message, flags=zmq.NOBLOCK)
            except zmq.Again:
                pass 

if __name__ == "__main__":
    # Split Button (8095) -> Explicitly to both 9095 AND 9096
    threading.Thread(target=relay_worker, args=(8095, [9095, 9096]), daemon=True).start()
    threading.Thread(target=relay_worker, args=(8100, [9100, 9101]), daemon=True).start()
    
    import time
    while True:
        time.sleep(1)