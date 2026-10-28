"""カメラ画像を QoS 設定ごとに何枚受信できるかを数える診断用スクリプト。"""
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('qos_probe')
counts = {}
profiles = {
    'BEST_EFFORT 深さ1': QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST),
    'BEST_EFFORT 深さ5': QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST),
    'RELIABLE 深さ1': QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST),
}
for name, qos in profiles.items():
    counts[name] = 0
    node.create_subscription(Image, '/camera/image_raw',
                             lambda m, n=name: counts.__setitem__(n, counts[n] + 1), qos)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
for name, c in counts.items():
    print(f'{name:18s}: 6秒で {c} 枚')
