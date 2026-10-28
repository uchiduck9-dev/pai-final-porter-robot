#!/bin/bash
# テスト用ワールドで、ロボットを各列の 1.5m 手前にワープさせて YOLO にかける
export ROS_DOMAIN_ID=42 IGN_PARTITION=pai_test
source /opt/ros/humble/setup.bash
cd /tmp
warp() {  # x y qz qw
  ign service -s /world/candidates/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
    --timeout 3000 --req "name: \"diffbot\", position: {x: $1, y: $2, z: 0.06}, orientation: {z: $3, w: $4}" > /dev/null
  sleep 2
}
warp 0.3 0 0 1;  echo "== 前方の列";  timeout 120 python3 /workspace/tools/yolo_snapshot.py /workspace/tools/lineup_front.png 2>&1 | grep -E "conf=|検出"
warp -0.3 0 1 0; echo "== 後方の列";  timeout 120 python3 /workspace/tools/yolo_snapshot.py /workspace/tools/lineup_back.png 2>&1 | grep -E "conf=|検出"
