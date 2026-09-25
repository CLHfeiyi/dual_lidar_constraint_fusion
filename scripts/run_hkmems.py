#!/usr/bin/env python3
"""
One-shot runner: launch LIO-SAM, play bag, save trajectory.

Usage:
  python3 run_hkmems.py <bag_path> --mode dual|single [--output DIR] [--rate 1.0]
"""

import argparse
import os
import signal
import subprocess
import sys
import time


def run(cmd, **kwargs):
    return subprocess.Popen(cmd, **kwargs)


def kill_all():
    for p in ["lio_sam_paper", "dual_lidar_merger", "trajectory_saver",
               "static_transform_publisher", "roslaunch", "roscore"]:
        subprocess.run(["pkill", "-f", p], stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--mode", choices=["dual", "single"], default="dual")
    parser.add_argument("--output-dir", default="/home/clh/桌面/data/HKMEMS-results")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--no-kill", action="store_true",
                        help="Don't kill existing ROS processes")
    args = parser.parse_args()

    bag = args.bag_path
    if not os.path.exists(bag):
        print(f"Bag not found: {bag}")
        sys.exit(1)

    seq = os.path.splitext(os.path.basename(bag))[0]
    traj_path = os.path.join(args.output_dir, f"{seq}_{args.mode}.tum")
    os.makedirs(args.output_dir, exist_ok=True)

    env = os.environ.copy()
    ws = "/home/clh/catkin_ws"

    # Kill old processes
    if not args.no_kill:
        kill_all()
        time.sleep(1)

    # Start roscore
    print("[1/4] Starting roscore...")
    roscore = run(["roscore"], env=env,
                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)

    # Launch LIO-SAM
    print(f"[2/4] Launching LIO-SAM ({args.mode})...")
    launch = run([
        "roslaunch", "lio_sam", f"run_hkmems_{args.mode}.launch",
        f"traj_output:={traj_path}"
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait for LIO-SAM to initialize
    print("[3/4] Waiting for initialization...")
    time.sleep(8)

    # Check if launch is still alive
    if launch.poll() is not None:
        print("ERROR: roslaunch died. Check ROS setup.")
        roscore.terminate()
        sys.exit(1)

    # Play bag
    print(f"[4/4] Playing bag ({args.rate}x)...")
    bag_proc = run([
        "rosbag", "play", "--clock", "-r", str(args.rate), bag
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    bag_proc.wait()
    print("Bag finished. Waiting for finalization...")
    time.sleep(8)

    # Shutdown
    launch.send_signal(signal.SIGINT)
    time.sleep(3)
    roscore.send_signal(signal.SIGINT)
    time.sleep(1)
    kill_all()

    # Verify output
    if os.path.exists(traj_path):
        n = len(open(traj_path).readlines())
        print(f"Done. {n} poses saved to: {traj_path}")
    else:
        print(f"WARNING: Trajectory file not found: {traj_path}")


if __name__ == "__main__":
    main()
