"""画像トピックの遅れ（画像のタイムスタンプとシミュレーション時刻の差）を測る診断用スクリプト。"""
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('measure_latency')
now = {'t': None}
lags = {'/camera/image_raw': [], '/porter/detections': []}


def on_clock(m):
    now['t'] = m.clock.sec + m.clock.nanosec * 1e-9


def make_cb(topic):
    def cb(m):
        if now['t'] is not None:
            lags[topic].append(now['t'] - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    return cb


node.create_subscription(Clock, '/clock', on_clock, 10)
node.create_subscription(Image, '/camera/image_raw', make_cb('/camera/image_raw'), qos_profile_sensor_data)
node.create_subscription(Image, '/porter/detections', make_cb('/porter/detections'), 1)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
for t, v in lags.items():
    if v:
        print(f'{t:22s} 遅れ 平均 {sum(v)/len(v):.2f}s / 最大 {max(v):.2f}s（{len(v)} 枚）')
    else:
        print(f'{t:22s} 受信なし')
