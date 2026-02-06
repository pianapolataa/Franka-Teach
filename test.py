import zmq

def simple_receiver():
    context = zmq.Context()
    # Use a PULL socket because that's what your detector uses
    socket = context.socket(zmq.PULL)
    
    # We BIND here because your code expects to "own" the port
    port = 8087
    try:
        socket.bind(f"tcp://127.0.0.1:{port}")
        print(f"Successfully bound to port {port}. Waiting for data...")
    except zmq.ZMQError as e:
        print(f"Error: Could not bind to {port}. Is your teleop script already running? {e}")
        return

    while True:
        try:
            # Wait for data
            message = socket.recv()
            print(f"Received: {message}")
        except KeyboardInterrupt:
            print("\nStopping...")
            break

if __name__ == "__main__":
    simple_receiver()