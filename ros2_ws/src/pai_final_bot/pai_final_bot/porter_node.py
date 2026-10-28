"""ポーターロボットのメインノード。

  段階1: YOLO で探して近づく
  段階2: LiDAR で障害物を避ける
  段階3: スーツケースを受け取って運び、椅子の横に置く

流れ:
  スーツケースを探す → 近づいて止まる → 荷台に載せて固定する
  → 折りたたみ椅子を探す → 近づいて止まる → 固定を外して椅子の横に置く → 完了
  途中で目的の物ではない障害物が正面に来たら、空いている側へ回り込んでから探し直す

段階3の「載せる」「置く」について:
  このロボットには腕がないため、スーツケースの移動は Gazebo の set_pose サービスで行い、
  運ぶ間は DetachableJoint（/porter/attach・/porter/detach）でロボットに固定する

「目的の物」と「障害物」の見分け方:
  カメラ: 目的の物の見かけの高さ（画像上の枠の高さ）と実物の高さから、目的の物までの距離を推定する
  LiDAR : 正面にある物までの距離を測る
  LiDAR の距離がカメラの推定より明らかに近ければ、正面の物は目的の物ではなく障害物と判断する

使うトピック:
  入力: /camera/image_raw（カメラ）, /scan（LiDAR）, /odom（移動距離）, /porter/attach_state（固定の状態）
  出力: /cmd_vel（移動指示）, /porter/detections（YOLO の判定結果を描いた画像）,
        /porter/attach・/porter/detach（スーツケースの固定・解除）
"""
import math
import re
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String
import torch
from ultralytics import YOLO

# YOLO のクラス名 → ログに出す日本語名
JP_NAMES = {'suitcase': 'スーツケース', 'chair': '折りたたみ椅子'}

# 目的の物の実際の高さ [m]（カメラ画像から距離を推定するのに使う）
TARGET_HEIGHTS = {'suitcase': 0.64, 'chair': 0.84}

# カメラの焦点距離 [px]（画像幅 640px・水平画角 1.2rad から計算）
FOCAL_PX = 320 / math.tan(0.6)

# 状態ごとの日本語ラベル
STATE_LABELS = {
    'SEARCH': '段階1/探索',
    'APPROACH': '段階1/接近',
    'ARRIVED': '段階1/到着',
    'AVOID': '段階2/回避',
    'PICKUP': '段階3/受取',
    'DROP': '段階3/配達',
    'CONFIRM': '段階3/確認',
    'DONE': '完了',
}

# スーツケースを載せる位置（ロボットから見た位置 [m]。荷台の上面の中央）と向き。
# 荷台は x=-0.35〜0.07、上面の高さ 0.06。スーツケース（奥行0.35 x 幅0.21）が収まる
CARRY_OFFSET = (-0.15, 0.0, 0.065)
CARRY_YAW = math.pi / 2
# スーツケースを置く位置: 椅子の正面、椅子の中心からこの距離 [m]
DROP_IN_FRONT = 0.45
# 折りたたみ椅子のモデルは「向き − 90度」の方向が正面（座る人が向く方向）
CHAIR_FRONT_YAW_OFFSET = -math.pi / 2
# 椅子の位置が取れなかったときの置き場所（ロボットから見て正面の床）
DROP_FALLBACK_OFFSET = (0.32, 0.0)
# 置くときに少し斜めにする角度 [rad]。真正面だと平らな板に見え、YOLO がスーツケースと認識しにくい
DROP_ANGLE = 0.5
# 置いた後に下がる距離 [m]（置いたスーツケースをカメラで確認するため）
BACKUP_DISTANCE = 0.45
# 配達後の確認で使う確信度の下限（置いた物が見えるかの確認なので、探索時より低めでよい）
CONFIRM_MIN_CONFIDENCE = 0.3


class PorterNode(Node):

    def __init__(self):
        super().__init__('porter_node')

        # --- パラメータ（launch や ros2 run の --ros-args -p で変更可能） ---
        self.declare_parameter('model_path', '/workspace/weights/yolov8n.pt')
        self.declare_parameter('targets', ['suitcase', 'chair'])   # 訪れる順番
        self.declare_parameter('min_confidence', 0.45)   # これ未満の判定は無視
        self.declare_parameter('confirm_frames', 3)      # 何回続けて見えたら「発見」とするか
        self.declare_parameter('search_speed', 0.4)      # 探索時の旋回速度 [rad/s]
        self.declare_parameter('approach_speed', 0.22)   # 接近時の最高速度 [m/s]
        self.declare_parameter('stop_distance', 0.55)    # 目的の物にこの距離まで近づいたら止まる [m]
        self.declare_parameter('avoid_distance', 0.6)    # 障害物がこの距離より近づいたら回避を始める [m]
        self.declare_parameter('clear_distance', 1.0)    # 正面がこの距離まで空いたら回り込みを始める [m]
        self.declare_parameter('pass_distance', 0.8)     # 障害物の横を進む距離 [m]
        self.declare_parameter('lost_timeout', 2.0)      # これ以上見えなければ見失ったとみなす [s]
        self.declare_parameter('arrive_wait', 2.0)       # 到着後、次へ進むまでの待ち時間 [s]
        self.declare_parameter('carry', True)            # 段階3（スーツケースを運ぶ）を行うか
        self.declare_parameter('world_name', 'arena')    # Gazebo のワールド名
        self.declare_parameter('robot_model', 'diffbot')
        self.declare_parameter('cargo_model', 'suitcase')
        self.declare_parameter('chair_model', 'folding_chair')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.targets = list(p('targets'))
        self.min_conf = p('min_confidence')
        self.confirm_frames = p('confirm_frames')
        self.search_speed = p('search_speed')
        self.approach_speed = p('approach_speed')
        self.stop_distance = p('stop_distance')
        self.avoid_distance = p('avoid_distance')
        self.clear_distance = p('clear_distance')
        self.pass_distance = p('pass_distance')
        self.lost_timeout = p('lost_timeout')
        self.arrive_wait = p('arrive_wait')
        self.carry = p('carry')
        self.world_name = p('world_name')
        self.robot_model = p('robot_model')
        self.cargo_model = p('cargo_model')
        self.chair_model = p('chair_model')

        # GPU があれば GPU で推論する（CPU だと Gazebo の画面表示と取り合いになり遅くなる）
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        device_jp = f'GPU（{torch.cuda.get_device_name(0)}）' if self.device == 0 else 'CPU'
        self.log('準備', f'YOLO モデルを読み込んでいます: {p("model_path")}（推論に使う装置: {device_jp}）')
        self.model = YOLO(p('model_path'))
        self.model(np.zeros((480, 640, 3), np.uint8), device=self.device, verbose=False)  # 初回の準備運転
        self.log('準備', 'YOLO モデルの読み込みが完了しました')

        # --- ROS の入出力 ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/porter/detections', 1)
        # 画像は「最新の1枚」だけを受け取る（古い画像が溜まって判定が遅れないようにする）。
        # BEST_EFFORT だと、Gazebo の画面表示で負荷が高いときに大きな画像が途中で欠けて届かなくなるため RELIABLE にする
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, latest_only)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.attach_pub = self.create_publisher(Empty, '/porter/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/porter/detach', 10)
        self.create_subscription(String, '/porter/attach_state', self.on_attach_state, 10)
        self.attach_state = None      # 'attached' / 'detached'（Gazebo から届いた固定の状態）
        self.cargo_step = None        # 受取・配達の途中経過
        self.target_override = None   # 一時的に探す物を変えるとき（配達後の確認など）に使う

        # --- 内部状態 ---
        self.started = time.monotonic()
        self.image_received = False
        self.last_infer = 0.0         # 最後に YOLO を実行した時刻
        self.last_lag_warn = 0.0
        self.scan = None              # (角度の配列, 距離の配列)。角度は左がプラス
        self.front_distance = math.inf
        self.position = (0.0, 0.0)
        self.target_index = 0
        self.state = None
        self.state_since = time.monotonic()
        self.search_dir = 1.0         # 探索時の回転方向（1: 左、-1: 右）
        self.seen_count = 0           # 連続で見えたフレーム数
        self.last_seen = 0.0          # 最後に目的の物体が見えた時刻
        self.last_detection = None    # dict(cx=中心x の正規化値 -1〜1, conf=確信度, dist=推定距離)
        self.last_progress_log = 0.0
        self.waiting_logged = False
        self.stats = {'received': 0, 'judged': 0, 'seen': set()}  # 探索中の状況報告用
        self.avoid = {}               # 回避中の情報（向き・段階・開始位置）

        self.create_timer(0.1, self.control)  # 制御は 10Hz（YOLO は画像が届いたときに最大 5Hz で実行）
        self.change_state('SEARCH')

    # ------------------------------------------------------------------
    # ログ
    # ------------------------------------------------------------------
    def log(self, label, message):
        self.get_logger().info(f'[{label}] {message}')

    @property
    def target(self):
        if self.target_override is not None:
            return self.target_override
        return self.targets[self.target_index]

    @property
    def target_jp(self):
        return JP_NAMES.get(self.target, self.target)

    def change_state(self, state):
        self.state = state
        self.state_since = time.monotonic()
        label = STATE_LABELS[state]
        if state == 'SEARCH':
            self.seen_count = 0
            self.stats = {'received': 0, 'judged': 0, 'seen': set()}
            self.last_progress_log = self.state_since
            way = '左' if self.search_dir > 0 else '右'
            self.log(label, f'{self.target_jp}を探しています（その場で{way}に回転中）')
        elif state == 'APPROACH':
            self.log(label, f'{self.target_jp}に向かって進みます')
        elif state == 'ARRIVED':
            self.stop()
            self.log(label, f'{self.target_jp}の手前に到着しました'
                            f'（距離 {self.front_distance:.2f}m）。停止します')
        elif state == 'PICKUP':
            self.stop()
            self.cargo_step = None
            self.log(label, f'{self.target_jp}を受け取ります')
        elif state == 'DROP':
            self.stop()
            self.cargo_step = None
            self.log(label, f'{self.target_jp}の正面にスーツケースを置きます')
        elif state == 'CONFIRM':
            self.cargo_step = None
            self.target_override = 'suitcase'
            self.seen_count = 0
            self.log(label, f'{BACKUP_DISTANCE:.2f}m 下がって、置いたスーツケースをカメラで確認します')
        elif state == 'DONE':
            self.stop()
            if self.carry:
                self.log(label, 'スーツケースを折りたたみ椅子の前に届けました。配達完了です')
            else:
                self.log(label, 'すべての目的地を訪れました。タスク完了です')

    # ------------------------------------------------------------------
    # センサー
    # ------------------------------------------------------------------
    def on_image(self, msg):
        self.image_received = True
        self.stats['received'] += 1
        now = time.monotonic()
        if self.state in ('ARRIVED', 'DONE') or now - self.last_infer < 0.2:
            return
        self.last_infer = now
        self.check_image_lag(msg)
        self.detect(msg)

    def check_image_lag(self, msg):
        """画像がシミュレーション時刻より大きく遅れていたら警告する（5秒に1回まで）。"""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lag = self.get_clock().now().nanoseconds * 1e-9 - stamp
        now = time.monotonic()
        if lag > 1.0 and now - self.last_lag_warn > 5.0:
            self.get_logger().warn(f'[警告] カメラ画像が {lag:.1f} 秒遅れています。'
                                   '処理が追いついていない可能性があります')
            self.last_lag_warn = now

    def on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges > msg.range_min)
        ranges = np.where(valid, ranges, msg.range_max)
        self.scan = (angles, ranges)
        # 正面 ±15度の範囲で一番近い距離を「正面距離」とする
        self.front_distance = self.min_range(-15, 15)

    def min_range(self, deg_from, deg_to):
        """LiDAR の deg_from〜deg_to 度（左がプラス）の範囲で一番近い距離。"""
        if self.scan is None:
            return math.inf
        angles, ranges = self.scan
        sel = ranges[(angles >= math.radians(deg_from)) & (angles <= math.radians(deg_to))]
        return float(sel.min()) if sel.size else math.inf

    def mean_range(self, deg_from, deg_to, cap=3.0):
        """LiDAR の deg_from〜deg_to 度の範囲の平均距離（空き具合の目安。遠すぎる値は cap で頭打ち）。"""
        angles, ranges = self.scan
        sel = ranges[(angles >= math.radians(deg_from)) & (angles <= math.radians(deg_to))]
        return float(np.minimum(sel, cap).mean()) if sel.size else 0.0

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)

    def on_attach_state(self, msg):
        self.attach_state = msg.data

    def detect(self, msg):
        """カメラ画像を YOLO にかけ、目的の物体の位置と推定距離を記録する。"""
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        min_conf = CONFIRM_MIN_CONFIDENCE if self.state == 'CONFIRM' else self.min_conf
        result = self.model(bgr, conf=min_conf, device=self.device, verbose=False)[0]
        self.stats['judged'] += 1

        best = None
        for box in result.boxes:
            name = self.model.names[int(box.cls)]
            self.stats['seen'].add(JP_NAMES.get(name, name))
            if name != self.target:
                continue  # 目的以外の物体（誤検出を含む）は使わない
            conf = float(box.conf)
            if best is None or conf > best['conf']:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = ((x1 + x2) / 2 - msg.width / 2) / (msg.width / 2)
                # 枠が画像の上下の端で切れているときは、すぐ近くにあるとみなす
                if y1 <= 2 or y2 >= msg.height - 2:
                    dist = 0.0
                else:
                    dist = FOCAL_PX * TARGET_HEIGHTS.get(self.target, 0.7) / max(y2 - y1, 1.0)
                best = {'cx': cx, 'conf': conf, 'dist': dist}

        if best is not None:
            self.seen_count += 1
            self.last_seen = time.monotonic()
            self.last_detection = best
        else:
            self.seen_count = 0

        self.publish_debug(result, msg)

    def publish_debug(self, result, src):
        """YOLO の判定結果（枠と名前）を描いた画像を流す。動画撮影や確認用。"""
        plotted = result.plot()[:, :, ::-1]  # BGR → RGB
        out = Image()
        out.header = src.header
        out.height, out.width = plotted.shape[:2]
        out.encoding = 'rgb8'
        out.step = out.width * 3
        out.data = np.ascontiguousarray(plotted).tobytes()
        self.debug_pub.publish(out)

    # ------------------------------------------------------------------
    # 制御（状態ごとの動き）
    # ------------------------------------------------------------------
    def control(self):
        now = time.monotonic()
        cmd = Twist()

        if not self.image_received or self.scan is None:
            if now - self.started > 3.0 and not self.waiting_logged:
                self.log('準備', 'カメラ画像と LiDAR を待っています（シミュレーションは起動していますか？）')
                self.waiting_logged = True
            return

        if self.state == 'SEARCH':
            if self.seen_count >= self.confirm_frames:
                d = self.last_detection
                side = '中央' if abs(d['cx']) < 0.2 else ('右寄り' if d['cx'] > 0 else '左寄り')
                self.log(STATE_LABELS['SEARCH'],
                         f'{self.target_jp}を発見しました（確信度 {d["conf"]:.0%}、画面の{side}、'
                         f'推定 約{d["dist"]:.1f}m 先）')
                self.change_state('APPROACH')
                return
            cmd.angular.z = self.search_dir * self.search_speed
            if now - self.last_progress_log > 5.0:
                s = self.stats
                seen = '、'.join(sorted(s['seen'])) if s['seen'] else 'なし'
                self.log(STATE_LABELS['SEARCH'],
                         f'{self.target_jp}を探索中（直近5秒: 画像受信 {s["received"]} 枚、'
                         f'AI判定 {s["judged"]} 回、見えた物: {seen}）')
                self.stats = {'received': 0, 'judged': 0, 'seen': set()}
                self.last_progress_log = now

        elif self.state == 'APPROACH':
            d = self.last_detection
            if now - self.last_seen > self.lost_timeout:
                # 近づきすぎて画面からはみ出し、AI が全体を認識できなくなった場合は到着とみなす
                if (self.front_distance < self.stop_distance + 0.35
                        and d['dist'] - self.front_distance <= 0.5):
                    self.log(STATE_LABELS['APPROACH'],
                             f'{self.target_jp}が近すぎて画面からはみ出しましたが、'
                             f'正面 {self.front_distance:.2f}m にあるため到着とします')
                    self.change_state('ARRIVED')
                    return
                self.log(STATE_LABELS['APPROACH'], f'{self.target_jp}を見失いました。もう一度探します')
                self.change_state('SEARCH')
                return
            # 正面に近い物があるとき、それが目的の物か障害物かを見分ける
            if self.front_distance < self.avoid_distance:
                if d['dist'] - self.front_distance > 0.5:
                    self.start_avoid(d['dist'])
                    return
                if self.front_distance <= self.stop_distance:
                    self.change_state('ARRIVED')
                    return
            # 物体が画面の中央に来るように向きを直しながら進む（ずれが大きいほど減速）
            cmd.angular.z = -0.9 * d['cx']
            cmd.linear.x = self.approach_speed * max(0.0, 1.0 - 1.5 * abs(d['cx']))
            # 近づいたら減速
            if self.front_distance < 1.0:
                cmd.linear.x *= max(0.3, (self.front_distance - self.stop_distance) / 0.45)
            if now - self.last_progress_log > 2.0:
                front = '不明' if math.isinf(self.front_distance) else f'{self.front_distance:.2f}m'
                self.log(STATE_LABELS['APPROACH'],
                         f'{self.target_jp}へ移動中（正面の距離 {front}、'
                         f'カメラによる推定 約{d["dist"]:.1f}m）')
                self.last_progress_log = now

        elif self.state == 'AVOID':
            cmd = self.avoid_step(now)
            if cmd is None:
                return

        elif self.state == 'ARRIVED':
            if now - self.state_since >= self.arrive_wait:
                last = self.target_index + 1 >= len(self.targets)
                if self.carry and self.target_index == 0:
                    self.change_state('PICKUP')
                elif self.carry and last:
                    self.change_state('DROP')
                else:
                    self.next_target()
            return

        elif self.state == 'PICKUP':
            self.pickup_step(now)
            return

        elif self.state == 'DROP':
            self.drop_step(now)
            return

        elif self.state == 'CONFIRM':
            cmd = self.confirm_step(now)
            if cmd is None:
                return

        elif self.state == 'DONE':
            return

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # 段階2: 障害物の回避
    # ------------------------------------------------------------------
    def start_avoid(self, target_dist):
        """回避を始める。左右の空いている方を選ぶ。"""
        left = self.mean_range(20, 90)
        right = self.mean_range(-90, -20)
        side = 1.0 if left >= right else -1.0
        way = '左' if side > 0 else '右'
        self.change_state('AVOID')
        self.avoid = {'side': side, 'phase': 'TURN', 'start': None}
        self.log(STATE_LABELS['AVOID'],
                 f'正面 {self.front_distance:.2f}m に障害物があります'
                 f'（{self.target_jp}はまだ約 {target_dist:.1f}m 先）')
        self.log(STATE_LABELS['AVOID'],
                 f'左の空き 平均{left:.1f}m / 右の空き 平均{right:.1f}m → {way}へ回り込みます')

    def avoid_step(self, now):
        """回避中の動き: 向きを変える（TURN）→ 障害物の横を進む（PASS）→ 探し直す。"""
        a = self.avoid
        cmd = Twist()
        label = STATE_LABELS['AVOID']
        way = '左' if a['side'] > 0 else '右'

        if now - self.state_since > 25.0:
            self.log(label, '回避に時間がかかりすぎています。いったん止まって探し直します')
            self.search_dir = -a['side']
            self.change_state('SEARCH')
            return None

        if a['phase'] == 'TURN':
            # 正面（±30度）が空くまで、選んだ側へその場で回る
            if self.min_range(-30, 30) > self.clear_distance:
                a['phase'] = 'PASS'
                a['start'] = self.position
                self.log(label, f'正面が空きました。障害物の横を {self.pass_distance:.1f}m 進みます')
            else:
                cmd.angular.z = a['side'] * 0.5
            return cmd

        # PASS: 障害物の横をまっすぐ進む
        moved = math.dist(self.position, a['start'])
        if moved >= self.pass_distance:
            self.log(label, f'障害物を{way}から回り込みました。もう一度{self.target_jp}を探します')
            self.search_dir = -a['side']  # 回り込んだのと逆向きに回ると、目的の物が見つかりやすい
            self.change_state('SEARCH')
            return None
        if self.min_range(-20, 20) < 0.4:
            self.log(label, '進む先にも障害物があります。もう一度向きを変えます')
            a['phase'] = 'TURN'
            return Twist()
        cmd.linear.x = 0.18
        return cmd

    def next_target(self):
        """次の目的地へ。最後なら完了。"""
        self.search_dir = 1.0
        if self.target_index + 1 < len(self.targets):
            self.target_index += 1
            self.change_state('SEARCH')
        else:
            self.change_state('DONE')

    # ------------------------------------------------------------------
    # 段階3: スーツケースの受け取りと配達
    # ------------------------------------------------------------------
    def pickup_step(self, now):
        """受取: スーツケースを荷台に載せる → 固定する → 固定を確認して次へ。"""
        label = STATE_LABELS['PICKUP']
        if self.cargo_step is None:
            robot = self.get_model_pose(self.robot_model)
            if robot is None:
                self.log(label, 'ロボットの位置が取得できませんでした。受け取りをやめて次へ進みます')
                self.next_target()
                return
            x, y, z, yaw = robot
            ox, oy, oz = CARRY_OFFSET
            cx = x + ox * math.cos(yaw) - oy * math.sin(yaw)
            cy = y + ox * math.sin(yaw) + oy * math.cos(yaw)
            if not self.set_model_pose(self.cargo_model, cx, cy, z + oz, yaw + CARRY_YAW):
                self.log(label, 'スーツケースを荷台に載せられませんでした。受け取りをやめて次へ進みます')
                self.next_target()
                return
            self.log(label, 'スーツケースを荷台に載せました。落ちないように固定します')
            self.attach_state = None
            for _ in range(3):
                self.attach_pub.publish(Empty())
            self.cargo_step = now
            return
        if self.attach_state == 'attached':
            self.log(label, 'スーツケースを固定しました。このまま運びます')
            self.next_target()
        elif now - self.cargo_step > 3.0:
            self.get_logger().warn(f'[{label}] 固定の確認が取れませんでした（状態: {self.attach_state}）。'
                                   'このまま運びますが、落ちる可能性があります')
            self.next_target()

    def drop_step(self, now):
        """配達: 固定を外す → 外れたのを確認 → 椅子の正面の床に置く → 確認へ。"""
        label = STATE_LABELS['DROP']
        if self.cargo_step is None:
            self.attach_state = None
            for _ in range(3):
                self.detach_pub.publish(Empty())
            self.log(label, 'スーツケースの固定を外します')
            self.cargo_step = now
            return
        if self.attach_state != 'detached' and now - self.cargo_step < 3.0:
            return
        if self.attach_state != 'detached':
            self.get_logger().warn(f'[{label}] 固定が外れたか確認できませんでした（状態: {self.attach_state}）')
        chair = self.get_model_pose(self.chair_model)
        if chair is not None:
            # 椅子の正面方向に DROP_IN_FRONT だけ離れた床に、椅子の方へ向けて置く
            cx, cy, _, cyaw = chair
            front = cyaw + CHAIR_FRONT_YAW_OFFSET
            dx = cx + DROP_IN_FRONT * math.cos(front)
            dy = cy + DROP_IN_FRONT * math.sin(front)
            place = (dx, dy, front + DROP_ANGLE, '椅子の正面')
        else:
            robot = self.get_model_pose(self.robot_model)
            if robot is None:
                self.log(label, '椅子とロボットの位置が取得できず、スーツケースを置けませんでした')
                self.change_state('DONE')
                return
            x, y, _, yaw = robot
            ox, oy = DROP_FALLBACK_OFFSET
            place = (x + ox * math.cos(yaw) - oy * math.sin(yaw),
                     y + ox * math.sin(yaw) + oy * math.cos(yaw), yaw, 'ロボットの正面')
            self.log(label, '椅子の位置が取得できなかったため、ロボットの正面に置きます')
        dx, dy, dyaw, where = place
        if self.set_model_pose(self.cargo_model, dx, dy, 0.0, dyaw):
            self.log(label, f'{where}（位置 x={dx:.2f}, y={dy:.2f}）にスーツケースを置きました')
            self.change_state('CONFIRM')
        else:
            self.log(label, 'スーツケースを床に置けませんでした')
            self.change_state('DONE')

    def confirm_step(self, now):
        """確認: 少し下がる → 置いたスーツケースが YOLO で見えるか確かめる → 完了。"""
        label = STATE_LABELS['CONFIRM']
        if self.cargo_step is None:
            self.cargo_step = {'start': self.position, 'stopped_at': None}
        step = self.cargo_step
        if step['stopped_at'] is None:
            if math.dist(self.position, step['start']) < BACKUP_DISTANCE and now - self.state_since < 8.0:
                cmd = Twist()
                cmd.linear.x = -0.15
                return cmd
            self.stop()
            step['stopped_at'] = now
            self.seen_count = 0
            return None
        if self.seen_count >= 1:
            self.log(label, f'置いたスーツケースをカメラで確認できました'
                            f'（確信度 {self.last_detection["conf"]:.0%}）')
        elif now - step['stopped_at'] < 5.0:
            return None
        else:
            self.get_logger().warn(f'[{label}] 置いたスーツケースをカメラで確認できませんでした')
        self.target_override = None
        self.change_state('DONE')
        return None

    def get_model_pose(self, model):
        """Gazebo からモデルの位置 (x, y, z, yaw) を取得する。失敗したら None。"""
        try:
            out = subprocess.run(['ign', 'model', '-m', model, '-p'], capture_output=True,
                                 text=True, timeout=10).stdout
            # 出力には "Model: [42]" のような番号もあるため、数値が3つ並ぶ [x y z] と [roll pitch yaw] だけを読む
            num = r'(-?[0-9.]+(?:e-?[0-9]+)?)'
            rows = re.findall(rf'\[\s*{num}\s+{num}\s+{num}\s*\]', out)
            x, y, z = (float(v) for v in rows[0])
            _, _, yaw = (float(v) for v in rows[1])
            return x, y, z, yaw
        except (subprocess.SubprocessError, IndexError, ValueError):
            return None

    def set_model_pose(self, model, x, y, z, yaw):
        """Gazebo のモデルを指定の位置・向きに動かす。成功したら True。"""
        req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        try:
            out = subprocess.run(
                ['ign', 'service', '-s', f'/world/{self.world_name}/set_pose',
                 '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '3000', '--req', req],
                capture_output=True, text=True, timeout=10).stdout
            return 'true' in out
        except subprocess.SubprocessError:
            return False

    def stop(self):
        self.cmd_pub.publish(Twist())


_stop_requested = False


def _raise_interrupt(signum, frame):
    global _stop_requested
    _stop_requested = True
    raise KeyboardInterrupt


def main():
    # 終了処理（ロボットを止める）を確実に行うため、Ctrl+C / 終了シグナルは自分で受け取る
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = PorterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # 終了シグナルがメッセージ受信の途中に届くと RuntimeError に変わることがある。終了要求中なら正常終了扱い
        if not _stop_requested:
            raise
    finally:
        if rclpy.ok():
            node.stop()
            node.get_logger().info('[終了] ロボットを停止し、ノードを終了しました')
        node.destroy_node()
        rclpy.try_shutdown()
