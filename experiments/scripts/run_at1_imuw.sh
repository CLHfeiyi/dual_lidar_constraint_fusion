#!/bin/bash
# AT1 (Aberdeen) imuWeight 消融：验证"IMU 在 AT1 是否有害"。
# 用法: bash run_at1_imuw.sh <imuWeight> <suffix>
#   例: bash run_at1_imuw.sh 0.5 050  -> AT1_dual_imuw050.tum
set -u
source /opt/ros/noetic/setup.bash
source /home/clh/catkin_ws/devel/setup.bash

W="${1:?usage: run_at1_imuw.sh <imuWeight> <suffix>}"
SFX="${2:?usage: run_at1_imuw.sh <imuWeight> <suffix>}"

BAG="/home/clh/视频/AT1_aberdeen_tunnel_2024-01-30-16-08-03_compressed.bag"
OUT="/home/clh/桌面/data/HKMEMS-results/AT1_dual_imuw${SFX}.tum"
LOG="/tmp/at1_imuw${SFX}_run.log"
rm -f "$OUT" "$LOG"

cleanup() {
    pkill -f "lio_sam_paper" 2>/dev/null
    pkill -f "trajectory_saver.py" 2>/dev/null
    pkill -f "rosbag play" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    sleep 2
}
cleanup

if ! rostopic list >/dev/null 2>&1; then
    roscore >/tmp/at1_imuw${SFX}_roscore.log 2>&1 &
    sleep 5
fi

roslaunch lio_sam run_at1_imuw.launch traj_output:="$OUT" imuWeight:="$W" >"$LOG" 2>&1 &
LPID=$!
sleep 8

rosbag play --clock --rate 1.0 "$BAG" --topics /ouster/points /rslidar_points /imu/data \
    >/tmp/at1_imuw${SFX}_play.log 2>&1 &
BAGPID=$!
wait "$BAGPID" 2>/dev/null || true
sleep 12

kill -INT "$LPID" 2>/dev/null
sleep 3
pkill -f "lio_sam_paper" 2>/dev/null
pkill -f "trajectory_saver.py" 2>/dev/null
cleanup

if [ -f "$OUT" ]; then
    N=$(wc -l < "$OUT")
    echo "DONE $OUT : $N poses"
else
    echo "NO TRAJECTORY"
fi
