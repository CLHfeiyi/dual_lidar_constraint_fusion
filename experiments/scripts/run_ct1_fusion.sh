#!/bin/bash
# CT1 (Central-Wan Chai Bypass) 约束级融合 run
set -u
source /opt/ros/noetic/setup.bash
source /home/clh/catkin_ws/devel/setup.bash

BAG="/home/clh/视频/CT1_central_bypass1_2024-01-11-17-06_compressed.bag"
OUT="/home/clh/桌面/data/HKMEMS-results/CT1_dual_fusion.tum"
LOG="/tmp/ct1_fusion_run.log"
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
    roscore >/tmp/ct1_fusion_roscore.log 2>&1 &
    sleep 5
fi

roslaunch lio_sam run_ct1_dual.launch traj_output:="$OUT" >"$LOG" 2>&1 &
LPID=$!
sleep 8

rosbag play --clock --rate 1.0 "$BAG" --topics /ouster/points /rslidar_points /imu/data \
    >/tmp/ct1_fusion_play.log 2>&1 &
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
