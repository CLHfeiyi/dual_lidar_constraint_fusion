#!/bin/bash
# Run WT3 dual-lidar with degeneracy epsilon=1e-3 (degeneracy ON) + extrinsic fix.
set -u
source /opt/ros/noetic/setup.bash
source /home/clh/catkin_ws/devel/setup.bash

BAG="/home/clh/视频/HK_MEMS_Dataset/bag/tunnel/western_tunnel/WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag"
OUT="/home/clh/桌面/data/HKMEMS-results/WT3_dual_eps1e-3.tum"
LOG="/tmp/wt3_eps1e-3_run.log"
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
    roscore >/tmp/wt3_roscore.log 2>&1 &
    sleep 5
fi

roslaunch lio_sam run_hkmems_dual.launch traj_output:="$OUT" >"$LOG" 2>&1 &
LPID=$!
sleep 8

rosbag play --clock --rate 1.0 "$BAG" --topics /ouster/points /rslidar_points /imu/data \
    >/tmp/wt3_eps1e-3_play.log 2>&1 &
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
