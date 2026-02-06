import zmq
from frankateach.network import create_pull_socket

def test_internal_ports():
    # These are the ports the Splitter is pushing TO
    button_port = 9095
    reset_port = 9100
    localhost = "127.0.0.1"

    print(f"--- Internal Splitter Test ---")
    
    try:
        # Initializing the sockets exactly like your detector class
        print(f"Binding to Internal Button Port: {button_port}")
        button_sock = create_pull_socket(localhost, button_port)
        
        print(f"Binding to Internal Reset Port: {reset_port}")
        reset_sock = create_pull_socket(localhost, reset_port)
        
    except zmq.ZMQError as e:
        print(f"\n[!] ERROR: {e}")
        print("Check if your teleop script is already running and using these ports.")
        return

    print("\nSuccessfully connected to Splitter. Waiting for messages...")

    while True:
        try:
            # We use NOBLOCK or a timeout if you want to see both ports, 
            # but for a simple test, we'll do standard blocking recvs.
            
            print("Checking for Button Data (8095 -> 9095)...")
            btn_msg = button_sock.recv()
            print(f">>> RECEIVED ON 9095: {btn_msg}")

            print("Checking for Reset Data (8100 -> 9100)...")
            rst_msg = reset_sock.recv()
            print(f">>> RECEIVED ON 9100: {rst_msg}")

        except KeyboardInterrupt:
            print("\nStopping test.")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")
            break

if __name__ == "__main__":
    test_internal_ports()