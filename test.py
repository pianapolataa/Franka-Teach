import zmq
from frankateach.network import create_pull_socket

def test_internal_ports():
    # These are the ports the Splitter is pushing TO
    button_port = 8093
    reset_port = 8094
    localhost = "172.24.71.240"

    print(f"Binding to Internal Button Port: {button_port}")
    button_sock = create_pull_socket(localhost, button_port)
    reset_sock = create_pull_socket(localhost, reset_port)

    print("\nSuccessfully connected to Splitter. Waiting for messages...")

    while True:
        try:
            btn_msg = button_sock.recv()
            print(f">>> RECEIVED ON 8093: {btn_msg}")

            rst_msg = reset_sock.recv()
            print(f">>> RECEIVED ON 8094: {rst_msg}")

        except KeyboardInterrupt:
            print("\nStopping test.")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")
            break

if __name__ == "__main__":
    test_internal_ports()