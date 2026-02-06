import socket
import threading

def listen_to_port(port, hand_label):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind to all interfaces on the specified port
    try:
        sock.bind(('0.0.0.0', port))
        print(f"Listening for {hand_label} hand on port {port}...")
    except Exception as e:
        print(f"Error binding to port {port}: {e}")
        return

    while True:
        data, addr = sock.recvfrom(4096)  # Buffer size 4096 bytes
        # Print a snippet of the received data so it doesn't flood your terminal
        print(f"[{hand_label} - Port {port}] Received from {addr}: {data[:50]}...")

if __name__ == "__main__":
    # Start thread for Right Hand (8087)
    right_thread = threading.Thread(target=listen_to_port, args=(8087, "RIGHT"), daemon=True)
    
    # Start thread for Left Hand (8110)
    left_thread = threading.Thread(target=listen_to_port, args=(8110, "LEFT"), daemon=True)

    right_thread.start()
    left_thread.start()

    print("Checking for Oculus data... Press Ctrl+C to stop.")
    
    try:
        while True:
            pass # Keep main thread alive
    except KeyboardInterrupt:
        print("\nStopping listeners.")