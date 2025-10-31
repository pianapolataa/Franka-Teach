import threading
import time
import subprocess

def replay_arm():
    subprocess.run(["python", "replay_arm_data.py"])

def replay_ruka():
    subprocess.run(["python", "replay_ruka_data.py"])

def main():
    # Create threads
    arm_thread = threading.Thread(target=replay_arm)
    ruka_thread = threading.Thread(target=replay_ruka)

    # Start threads
    arm_thread.start()
    ruka_thread.start()

    # Wait for both threads to finish
    arm_thread.join()
    ruka_thread.join()

    print("Synchronized replay finished.")

if __name__ == "__main__":
    main()
