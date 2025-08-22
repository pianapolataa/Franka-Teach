#!/usr/bin/env python3
import argparse, signal, sys
from frankateach.network import ZMQKeypointSubscriber
from frankateach.constants import *


# self.action_socket = create_request_socket(LOCALHOST, CONTROL_PORT)



#!/usr/bin/env python3
import zmq
from frankateach.network import create_request_socket
from frankateach.constants import LOCALHOST, CONTROL_PORT  # 按你项目里的常量

def main():
    # 创建 REQ socket；如果 create_request_socket 已经 connect 了就不用再连
    sock = create_request_socket(LOCALHOST, CONTROL_PORT)

    # 防止一直卡死：设置 3 秒接收超时
    sock.setsockopt(zmq.RCVTIMEO, 3000)   # ms
    sock.setsockopt(zmq.LINGER, 0)

    try:
        # 发送一条无害的探活请求
        sock.send_json({"type": "PING", "from": "tester"})
        reply = sock.recv_json()          # REQ-REP 必须先 send 再 recv
        print("✅ Got reply from robot:", reply)
    except zmq.Again:
        print("⛔ Timeout: no reply within 3s (server not running? wrong host/port?)")
    except Exception as e:
        print("❌ Error:", e)

if __name__ == "__main__":
    main()
