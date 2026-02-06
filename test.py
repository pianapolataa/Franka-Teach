import socket

def test_raw_udp():
    port = 8087
    # Create a standard UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Bind to ALL interfaces to catch the Quest data
        sock.bind(('0.0.0.0', port))
        print(f"Listening for RAW UDP on port {port}...")
    except Exception as e:
        print(f"Could not bind: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            print(f"RECEIVED from {addr}: {data[:50]}...") # Print first 50 chars
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    test_raw_udp()