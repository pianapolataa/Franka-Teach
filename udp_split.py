import socket

def split_udp(input_port, output_ports):
    # Create the listening socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', input_port))
    
    print(f"Relaying Port {input_port} -> {output_ports}")
    
    while True:
        data, addr = sock.recvfrom(4096)
        for port in output_ports:
            sock.sendto(data, ('127.0.0.1', port))

# You can run multiple splitters for different ports
if __name__ == "__main__":
    from threading import Thread
    
    # Split Button Port 8095 to 9095 (Right) and 9096 (Left)
    Thread(target=split_udp, args=(8095, [9095, 9096])).start()
    
    # Split Reset Port 8100 to 9100 (Right) and 9101 (Left)
    Thread(target=split_udp, args=(8100, [9100, 9101])).start()