import time
from frankateach.network import ZMQKeypointSubscriber
from frankateach.constants import LOCALHOST

def test_transformed_frames(port1, port2):
    print(f"--- Transformed Frame Tester ---")
    print(f"Connecting to {LOCALHOST} on ports {port1} and {port2}...")

    # 1. Initialize the subscribers exactly like your class
    sub1 = ZMQKeypointSubscriber(
        host=LOCALHOST,
        port=port1,
        topic='transformed_hand_frame'
    )

    sub2 = ZMQKeypointSubscriber(
        host=LOCALHOST,
        port=port2,
        topic='transformed_hand_frame'
    )

    print("Subscribers active. Waiting for data (Topic: 'transformed_hand_frame')...")

    try:
        while True:
            # 2. Receive from Port 1
            frame1 = sub1.recv_keypoints()
            if frame1 is not None:
                print(f"\n[PORT {port1}] Received Frame:")
                # Typically index 0 is the wrist/origin, 1-3 are the axes
                print(f"  Origin: {frame1[0]}") 

            # 3. Receive from Port 2
            frame2 = sub2.recv_keypoints()
            if frame2 is not None:
                print(f"\n[PORT {port2}] Received Frame:")
                print(f"  Origin: {frame2[0]}")

            time.sleep(0.01) # Small sleep to prevent CPU spiking

    except KeyboardInterrupt:
        print("\nStopping test.")
    finally:
        sub1.stop()
        sub2.stop()

if __name__ == "__main__":
    # You can change these port numbers to whichever ports your 
    # transformation scripts are publishing to.
    PORT_A = 8093 
    PORT_B = 8094
    test_transformed_frames(PORT_A, PORT_B)