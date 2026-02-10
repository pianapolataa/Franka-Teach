import pickle
import time
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

class HandReplayer:
    def __init__(self, folder: str):
        self.hand = Hand(hand_type="right")
        self.curr_pos = self.hand.read_pos()
        test_pos = self.hand.tensioned_pos
        move_to_pos(curr_pos=self.curr_pos, des_pos=test_pos, hand=self.hand, traj_len=50)
        time.sleep(1)
        self.curr_pos = self.hand.read_pos()

    def replay(self):
        while True:
            target_pos = [1872.99, 2540.,   1617.,   1860.,   1792.78, 2900.,   1700, 1903, 1817, 2990, 2959, 1444,  765, 1414, 1990 ,1630]
            move_to_pos(curr_pos=self.curr_pos, des_pos=target_pos, hand=self.hand, traj_len=35)
            self.curr_pos = self.hand.read_pos()
            
        self.hand.close()
        print("Replay finished.")
