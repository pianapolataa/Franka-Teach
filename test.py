import zmq
from frankateach.network import create_pull_socket
from frankateach.constants import INTERNAL_IP

def test_oculus_sockets():
    # Ports as defined in your setup
    button_port = 8095
    teleop_reset_port = 8100

    print(f"--- Socket Test ---")
    print(f"IP: {INTERNAL_IP}")
    
    try:
        # 1. Initialize sockets exactly like your class does
        print(f"Attempting to bind Button Port: {button_port}")
        button_keypoint_socket = create_pull_socket(INTERNAL_IP, button_port)
        
        print(f"Attempting to bind Reset Port: {teleop_reset_port}")
        teleop_reset_socket = create_pull_socket(INTERNAL_IP, teleop_reset_port)
        
    except zmq.ZMQError as e:
        print(f"\n[!] ERROR: {e}")
        print("This usually means the port is already in use by another script.")
        return

    print("\nSuccessfully bound to sockets. Waiting for data (press buttons on Oculus)...")

    while True:
        try:
            # 2. Replicate the .recv() logic
            # These are blocking calls; the script will wait here until data arrives
            button_data = button_keypoint_socket.recv()
            print(f"[PORT 8095] Received Button Data: {button_data}")

            reset_data = teleop_reset_socket.recv()
            print(f"[PORT 8100] Received Reset Data: {reset_data}")

        except KeyboardInterrupt:
            print("\nStopping test.")
            break
        except Exception as e:
            print(f"Error during recv: {e}")
            break

    button_keypoint_socket.close()
    teleop_reset_socket.close()

if __name__ == "__main__":
    test_oculus_sockets()