#!/usr/bin/env python3
"""
Plot dual vs single LiDAR trajectories from saved bag files.

Usage:
  python3 plot_hkmems.py <dual_bag> <single_bag> [--output-dir DIR]
"""

import argparse
import os
import subprocess
import sys
import glob


def extract_tum(bag_path, topic="/lio_sam/mapping/odometry"):
    """Extract TUM-format trajectory from rosbag using evo."""
    tum_path = bag_path.replace(".bag", ".tum")
    if os.path.exists(tum_path):
        return tum_path

    cmd = ["evo_traj", "bag", bag_path, topic, "--save_as_tum"]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if os.path.exists(tum_path):
        return tum_path
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Plot dual vs single LiDAR LIO-SAM trajectories")
    parser.add_argument("traj_dir",
                        help="Directory containing trajectory bags")
    parser.add_argument("--sequence", default=None,
                        help="Sequence name filter (e.g., WT3, HH2)")
    parser.add_argument("--output-dir", default=None,
                        help="Output directory for plots (default: traj_dir)")
    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = args.traj_dir

    os.makedirs(args.output_dir, exist_ok=True)

    # Find trajectory bags
    all_bags = glob.glob(os.path.join(args.traj_dir, "*_traj.bag"))
    if not all_bags:
        print(f"No trajectory bags found in {args.traj_dir}")
        sys.exit(1)

    dual_bags = sorted([b for b in all_bags if "_dual_traj" in b])
    single_bags = sorted([b for b in all_bags if "_single_traj" in b])

    if args.sequence:
        dual_bags = [b for b in dual_bags if args.sequence in b]
        single_bags = [b for b in single_bags if args.sequence in b]

    if not dual_bags or not single_bags:
        print("Need at least one dual and one single trajectory bag")
        print(f"  Dual:   {dual_bags}")
        print(f"  Single: {single_bags}")
        sys.exit(1)

    # Extract TUM trajectories
    tum_files = []
    labels = []

    for bag, label in [(dual_bags[0], "Dual-LiDAR"),
                        (single_bags[0], "Single-LiDAR")]:
        tum = extract_tum(bag)
        if tum:
            tum_files.append(tum)
            labels.append(label)
            print(f"  {label}: {tum}")

    if len(tum_files) < 2:
        print("Could not extract TUM trajectories")
        sys.exit(1)

    # Plot comparison
    seq_name = os.path.basename(traj_dir).replace("_traj.bag", "")
    if args.sequence:
        seq_name = args.sequence

    plot_path = os.path.join(args.output_dir, f"{seq_name}_comparison.pdf")
    png_path = os.path.join(args.output_dir, f"{seq_name}_comparison.png")

    print(f"\nPlotting trajectories...")

    # XY trajectory comparison
    cmd = [
        "evo_traj", "tum", tum_files[0], tum_files[1],
        "--ref", tum_files[0],
        "-p", "--plot_mode", "xy",
        "--save_plot", plot_path
    ]
    print(f"  Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
        print(f"  Plot saved: {plot_path}")

    # XYZ trajectory comparison
    plot_xyz = os.path.join(args.output_dir, f"{seq_name}_comparison_xyz.pdf")
    cmd = [
        "evo_traj", "tum", tum_files[0], tum_files[1],
        "--ref", tum_files[0],
        "-p", "--plot_mode", "xyz",
        "--save_plot", plot_xyz
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  Plot saved: {plot_xyz}")

    print(f"\nDone. Plots in: {args.output_dir}")


if __name__ == "__main__":
    main()
