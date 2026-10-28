"""カメラ画像を1枚取得して YOLO で物体検出し、結果を表示・保存する確認用スクリプト。

使い方: python3 yolo_snapshot.py [保存先png]
"""
import sys

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from ultralytics import YOLO

out = sys.argv[1] if len(sys.argv) > 1 else '/workspace/yolo_check.png'

rclpy.init()
node = rclpy.create_node('yolo_snapshot')
frame = {}
node.create_subscription(
    Image, '/camera/image_raw',
    lambda m: frame.setdefault('img', np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)), 1)
while 'img' not in frame:
    rclpy.spin_once(node, timeout_sec=1.0)

bgr = cv2.cvtColor(frame['img'], cv2.COLOR_RGB2BGR)
model = YOLO('yolov8n.pt')
result = model(bgr, conf=0.25, verbose=False)[0]
for box in result.boxes:
    name = model.names[int(box.cls)]
    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
    print(f'{name:15s} conf={float(box.conf):.2f}  box=({x1},{y1})-({x2},{y2})')
if len(result.boxes) == 0:
    print('(何も検出されませんでした)')
cv2.imwrite(out, result.plot())
print('saved', out)
