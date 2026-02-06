import zmq
import threading

def zmq_relay(source_port, target_ports):
    context = zmq.Context()
    
    # This socket "steals" the Quest's data from port 8095
    # (Assuming the Quest is pushing raw data or using a compatible ZMQ setup)
    frontend = context.socket(zmq.PULL)
    frontend.bind(f"tcp://*:{source_port}")
    
    # These sockets send that data to your hand detectors
    backend_sockets = []
    for port in target_ports:
        s = context.socket(zmq.PUSH)
        s.bind(f"tcp://*:{port}") # Your teleop code will CONNECT to these
        backend_sockets.append(s)
    
    print(f"ZMQ Relay Active: {source_port} -> {target_ports}")
    
    while True:
        msg = frontend.recv()
        for s in backend_sockets:
            s.send(msg)

if __name__ == "__main__":
    # Relay Button data
    threading.Thread(target=zmq_relay, args=(8095, [9095, 9096]), daemon=True).start()
    # Relay Reset data
    threading.Thread(target=zmq_relay, args=(8100, [9100, 9101]), daemon=True).start()
    
    while True:
        import time
        time.sleep(1)