# AI ポーターロボット（ROS 2 × Gazebo × YOLO）

ロビーに置き忘れられたスーツケースを、ロボットが**自分で探して受け取り、障害物を避けながらお客様の席（折りたたみ椅子）まで届ける**シミュレーションです。
Physical AI 応用1講座の最終課題として作成しました。

![荷台にスーツケースを載せたロボット](tools/carry_view.png)

## 動作の流れ

| 段階 | 内容 | 使うセンサー |
|---|---|---|
| 1. 探す・近づく | その場で回りながらカメラ画像を YOLO で判定し、スーツケースを見つけたら近づいて止まる | カメラ、LiDAR |
| 3. 受け取る | スーツケースを荷台に載せて固定する | — |
| 1. 探す・近づく | 折りたたみ椅子を探して向かう | カメラ、LiDAR |
| 2. 避ける | 通り道の箱を「障害物」と見分け、空いている側から回り込んで探し直す | カメラ、LiDAR |
| 3. 届ける | 椅子の正面にスーツケースを置き、少し下がって YOLO で置いた荷物を確認する | カメラ |

途中の状況は、日本語のログで表示されます。

> **注意：** ロボットに腕がないため、荷物を「載せる」「置く」動作は Gazebo の機能でスーツケースの位置を直接移しています（運ぶ間は DetachableJoint でロボットに固定）。

## AI の使い方

- **YOLOv8n（COCO で学習済み）の推論**で、カメラ画像からスーツケースと椅子を見つけ、その位置で進む向きを決めます。
- カメラで推定した目的の物までの距離と、LiDAR で測った正面の距離を比べて、**目的の物と障害物を見分けます**。

## 環境

- Windows 11 + Docker Desktop（WSL 2）。Ubuntu の PC でも動きます
- Docker イメージ `airobotbook/ros2-desktop-ai-robot-book-humble`（Ubuntu 22.04 / ROS 2 Humble / Ignition Gazebo Fortress / Ultralytics）
- NVIDIA GPU（任意）。GPU があれば YOLO を GPU で動かします。なくても CPU で動きます

## セットアップ

1. このリポジトリを取得します。
   ```bash
   git clone https://github.com/uchiduck9-dev/pai-final-porter-robot.git
   ```
2. 取得したフォルダで、PowerShell からコンテナを作成します（初回のみ。2回目以降は `docker start pai_final`）。
   ```powershell
   .\run_final_container.ps1
   ```
   ブラウザで http://localhost:6081 を開くと、コンテナのデスクトップ画面が表示されます。このフォルダはコンテナ内の `/workspace` につながっています。
3. デスクトップのターミナルで、YOLO の学習済みモデル（yolov8n.pt）を取得します（約6MB、初回のみ）。
   ```bash
   mkdir -p /workspace/weights && cd /workspace/weights && python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
   ```
4. ビルドします。
   ```bash
   cd /workspace/ros2_ws && colcon build
   ```

## 実行（ターミナル3つ）

| ターミナル | 役割 | コマンド |
|---|---|---|
| A | シミュレーション（Gazebo と ROS 2 のブリッジ） | `source /workspace/ros2_ws/install/setup.bash && ros2 launch pai_final_bot sim.launch.py` |
| B | ポーターロボット（日本語ログ） | `source /workspace/ros2_ws/install/setup.bash && ros2 launch pai_final_bot porter.launch.py` |
| C | ロボットの目線と YOLO の判定 | `ros2 run rqt_image_view rqt_image_view`（`/porter/detections` を選択） |

A を起動して3秒ほど待ってから B を起動します。スタートから完了まで約70秒です。
もう一度動かすときは、A を Ctrl+C で止めて起動し直してください（ロボットと荷物が初期位置に戻ります）。

キーボードで操作したい場合は、A を `ros2 launch pai_final_bot sim.launch.py teleop:=true` で起動します。

ロボットの動きを決めるコードは `ros2_ws/src/pai_final_bot/pai_final_bot/porter_node.py` です。

## 出典とライセンス

### 3D モデル（Gazebo Fuel）
- **Suitcase2**：作者 Wan Yi Seow / OpenRobotics。[Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase2) より。ライセンス [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)。
  変更点：ロボットで運べるよう、固定を外して重さ（0.5kg）と箱型の当たり判定を付けました。
- **foldable_chair**：作者 Rumman Waqar / OpenRobotics。[Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair) より。ライセンス [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。
  変更点：なし（フォルダ名を `folding_chair` にしてパッケージ内に配置）。

### ソフトウェア
- 物体検出：[Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)（AGPL-3.0）。学習済みモデル `yolov8n.pt` はこのリポジトリに含めていません（セットアップ手順で取得します）。
- ROS 2 Humble、Ignition Gazebo Fortress、ros_gz（いずれも Apache-2.0）。

### 参考にしたもの
- 最終課題サンプル「ROS2とGazeboの連携によるロボット操作」（https://qiita.com/N622/items/9d89f77d85d9da0af29e ）。ROS 2 と Ignition Gazebo を連携させる構成を参考にしました。

### このリポジトリのコード
[MIT License](LICENSE)。ただし、実行時に使う Ultralytics は AGPL-3.0 です。
