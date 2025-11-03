import threading
import cv2
from replay_arm_data import ArmReplayer
from replay_ruka_data import HandReplayer

def run_arm():
    arm = ArmReplayer("data/demonstration_0")
    arm.replay()

def run_hand():
    hand = HandReplayer("data/demonstration_0")
    hand.replay()

def show_video(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Cannot open video:", video_path)
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Replay Video", frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()

arm_thread = threading.Thread(target=run_arm, daemon=True)
hand_thread = threading.Thread(target=run_hand, daemon=True)
video_thread = threading.Thread(
    target=show_video, args=("data/demonstration_0/cam_1_rgb_video.avi",), daemon=True
)

arm_thread.start()
hand_thread.start()
video_thread.start()

arm_thread.join()
hand_thread.join()
video_thread.join()

print("Both replays finished.")