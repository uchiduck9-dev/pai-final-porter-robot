#!/bin/bash
# 見えない別空間（IGN_PARTITION / ROS_DOMAIN_ID を分離）で、シミュレーションとポーターノードを動かして記録する
# 使い方: bash run_test.sh [秒数]
# 終了時は、このスクリプトが起動したプロセスだけを止める（画面で動かしている Gazebo やブリッジには触れない）
DUR=${1:-150}
export IGN_PARTITION=pai_test ROS_DOMAIN_ID=42 DISPLAY=:0
source /workspace/ros2_ws/install/setup.bash
OUT=/workspace/tools/test_logs; mkdir -p $OUT
# setsid で独立したプロセスグループにし、後でグループごと止められるようにする
setsid ros2 launch pai_final_bot sim.launch.py headless:=true > $OUT/sim.log 2>&1 &
SIM_PGID=$!
sleep 20
# 2秒ごとにロボットの位置を記録
( while true; do timeout 3 ros2 topic echo --once /odom --field pose.pose.position 2>/dev/null | tr '\n' ' ' | sed "s/^/$(date +%s) /"; echo; sleep 2; done ) > $OUT/odom.log 2>&1 &
ODOM=$!
cd /tmp && timeout $DUR ros2 run pai_final_bot porter_node --ros-args -p use_sim_time:=true > $OUT/porter.log 2>&1
kill $ODOM
# 終了時点の各モデルの位置（Gazebo 上の正確な位置）を記録
for m in diffbot suitcase folding_chair; do
  echo "$m $(ign model -m $m -p 2>/dev/null | grep -A2 Pose | tail -2 | tr -s ' \n' ' ')"
done > $OUT/final_poses.log
kill -INT -- -$SIM_PGID 2>/dev/null; sleep 5
kill -KILL -- -$SIM_PGID 2>/dev/null
echo finished
