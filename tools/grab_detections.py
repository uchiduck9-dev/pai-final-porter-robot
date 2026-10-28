"""/porter/detections（YOLO の判定画像）を一定時間保存し、ロボットの向きと一緒に記録する診断用スクリプト。

使い方: python3 grab_detections.py [秒数] [保存先フォルダ]
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 16
out = sys.argv[2] if len(sys.argv) > 2 else '/workspace/tools/test_logs/frames'
os.makedirs(out, exist_ok=True)

rclpy.init()
node = rclpy.create_node('grab_detections')
state = {'yaw': None, 'n': 0}


def on_odom(m):
    q = m.pose.pose.orientation
    state['yaw'] = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))


def on_img(m):
    if state['yaw'] is None:
        return
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, ::-1]
    name = f'{out}/{state["n"]:03d}_yaw{state["yaw"]:+04.0f}.png'
    cv2.imwrite(name, img)
    state['n'] += 1


node.create_subscription(Odometry, '/odom', on_odom, 10)
node.create_subscription(Image, '/porter/detections', on_img, 1)
end = time.time() + dur
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f'{state["n"]} 枚保存しました: {out}')
