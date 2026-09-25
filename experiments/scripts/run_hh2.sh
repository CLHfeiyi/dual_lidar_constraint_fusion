#!/bin/bash
set -u
source /opt/ros/noetic/setup.bash
source /home/clh/catkin_ws/devel/setup.bash
BAG="/home/clh/视频/HK_MEMS_Dataset/bag/dyna_pedestrain/hung_hom/HH2_coastal_footpath.bag"
OUT="$1"; LOG="${OUT%.tum}.log"
rm -f "$OUT" "$LOG"
pkill -f "lio_sam_paper" 2>/dev/null; pkill -f "trajectory_saver" 2>/dev/null; pkill -f "roslaunch" 2>/dev/null
pkill -f "rosbag play" 2>/dev/null; pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "rosmaster" 2>/dev/null; pkill -f "roscore" 2>/dev/null
sleep 3
roscore > /tmp/hh2_roscore.log 2>&1 &
sleep 4
roslaunch lio_sam run_hkmems_dual.launch traj_output:="$OUT" > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 9
rosbag play --clock --rate 1.0 "$BAG" --topics /ouster/points /rslidar_points /imu/data /clock > /tmp/hh2_play.log 2>&1 &
PLAY_PID=$!
wait $PLAY_PID
sleep 12
kill $LAUNCH_PID 2>/dev/null
pkill -f "lio_sam_paper" 2>/dev/null; pkill -f "trajectory_saver" 2>/dev/null; pkill -f "roslaunch" 2>/dev/null
pkill -f "rosmaster" 2>/dev/null; pkill -f "roscore" 2>/dev/null; pkill -f "static_transform_publisher" 2>/dev/null
echo "=== DONE ==="; wc -l "$OUT" 2>/dev/null || echo "NO_TRAJ"
