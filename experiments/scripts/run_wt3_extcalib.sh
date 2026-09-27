#!/bin/bash
# WT3 (western tunnel) online extrinsic calibration: back-lidar extrinsics GN update, imuWeight=1.0 baseline.
set -u
source /opt/ros/noetic/setup.bash
source /home/clh/catkin_ws/devel/setup.bash

BAG="/home/clh/视频/HK_MEMS_Dataset/bag/tunnel/western_tunnel/WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag"
OUT="/home/clh/桌面/data/HKMEMS-results/WT3_dual_extcalib.tum"
LOG="/tmp/wt3_extcalib_run.log"
rm -f "$OUT" "$LOG"

cleanup() {
    pkill -f "lio_sam_paper" 2>/dev/null
    pkill -f "trajectory_saver.py" 2>/dev/null
    pkill -f "rosbag play" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    pkill -f "rosmaster" 2>/dev/null
    pkill -f "roscore" 2>/dev/null
    sleep 2
}
cleanup

if ! rostopic list >/dev/null 2>&1; then
    roscore >/tmp/wt3_extcalib_roscore.log 2>&1 &
    sleep 5
fi

roslaunch lio_sam run_wt3_extcalib.launch traj_output:="$OUT" >"$LOG" 2>&1 &
LPID=$!
sleep 8

rosbag play --clock --rate 1.0 "$BAG" --topics /ouster/points /rslidar_points /imu/data /clock \
    >/tmp/wt3_extcalib_play.log 2>&1 &
BAGPID=$!
wait "$BAGPID" 2>/dev/null || true
sleep 12

kill -INT "$LPID" 2>/dev/null
sleep 3
cleanup

if [ -f "$OUT" ]; then
    N=$(wc -l < "$OUT")
    echo "DONE $OUT : $N poses"
else
    echo "NO TRAJECTORY"
fi
