"""ロボットの現在の状態（位置・向き・傾き・回転速度・LiDAR の最短距離）を数回表示する診断用スクリプト。"""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('robot_status')
data = {}
node.create_subscription(Odometry, '/odom', lambda m: data.__setitem__('odom', m), 10)
node.create_subscription(LaserScan, '/scan', lambda m: data.__setitem__('scan', m), qos_profile_sensor_data)

for _ in range(3):
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    o, s = data.get('odom'), data.get('scan')
    if o is None:
        print('odom がまだ届いていません')
        continue
    p, q, w = o.pose.pose.position, o.pose.pose.orientation, o.twist.twist.angular.z
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
    line = f'位置=({p.x:.2f}, {p.y:.2f})  向き={yaw:7.1f}度  回転速度={w:.2f}rad/s  傾き(qx,qy)=({q.x:.3f}, {q.y:.3f})'
    if s is not None:
        valid = [(r, s.angle_min + i * s.angle_increment) for i, r in enumerate(s.ranges)
                 if math.isfinite(r) and r > s.range_min]
        r, a = min(valid)
        line += f'  LiDAR最短={r:.2f}m（{math.degrees(a):.0f}度方向）'
    print(line)
