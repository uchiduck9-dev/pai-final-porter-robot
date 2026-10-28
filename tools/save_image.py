"""指定した画像トピックから1枚保存する確認用スクリプト。使い方: python3 save_image.py トピック名 保存先.png"""
import sys

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

topic, out = sys.argv[1], sys.argv[2]
rclpy.init()
node = rclpy.create_node('save_image')
got = {}
node.create_subscription(Image, topic, lambda m: got.setdefault('m', m), 1)
while 'm' not in got:
    rclpy.spin_once(node, timeout_sec=1.0)
m = got['m']
img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
cv2.imwrite(out, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
print('saved', out)
