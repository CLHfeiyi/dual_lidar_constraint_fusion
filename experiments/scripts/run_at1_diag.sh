#!/bin/bash
# AT1 退化诊断：degeneracyLogOnly=true，只记 EVD 不投影（轨迹应与融合版一致）
set -u
source /opt/ros/noetic/setup.bash
source /home/clh/catkin_ws/devel/setup.bash

BAG="/home/clh/视频/AT1_aberdeen_tunnel_2024-01-30-16-08-03_compressed.bag"
OUT="/tmp/at1_diag.tum"
LOG="/tmp/at1_diag_run.log"
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
    roscore >/tmp/at1_diag_roscore.log 2>&1 &
    sleep 5
fi

roslaunch lio_sam run_at1_diag.launch traj_output:="$OUT" >"$LOG" 2>&1 &
LPID=$!
sleep 8

rosbag play --clock --rate 1.0 "$BAG" --topics /ouster/points /rslidar_points /imu/data \
    >/tmp/at1_diag_play.log 2>&1 &
BAGPID=$!
wait "$BAGPID" 2>/dev/null || true
sleep 12

kill -INT "$LPID" 2>/dev/null
sleep 3
pkill -f "lio_sam_paper" 2>/dev/null
pkill -f "trajectory_saver.py" 2>/dev/null
cleanup

echo "=== DEGEN lines ==="
grep -c "DEGEN" "$LOG" 2>/dev/null || echo 0
if [ -f "$OUT" ]; then
    echo "DONE $OUT : $(wc -l < "$OUT") poses"
else
    echo "NO TRAJECTORY"
fi
