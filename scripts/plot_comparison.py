#!/usr/bin/env python3
"""
Compare dual vs single LiDAR LIO-SAM trajectories.

Usage:
  python3 plot_comparison.py <dual.tum> <single.tum> [--title TITLE] [--output-dir DIR]
"""

import argparse
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Plot dual vs single LiDAR trajectory comparison")
    parser.add_argument("dual_tum", help="Dual-LiDAR TUM trajectory file")
    parser.add_argument("single_tum", help="Single-LiDAR TUM trajectory file")
    parser.add_argument("--title", default="Dual vs Single LiDAR LIO-SAM")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--sequence", default=None,
                        help="Sequence name for output filename")
    args = parser.parse_args()

    for f in [args.dual_tum, args.single_tum]:
        if not os.path.exists(f):
            print(f"File not found: {f}")
            sys.exit(1)

    out_dir = args.output_dir or os.path.dirname(args.dual_tum)
    os.makedirs(out_dir, exist_ok=True)

    seq = args.sequence or "comparison"

    # Compare using evo_ape (no ground truth, so use dual as reference)
    print("=" * 60)
    print(f"Dual:   {args.dual_tum}")
    print(f"Single: {args.single_tum}")
    print("=" * 60)

    # XY trajectory plot
    plot_xy = os.path.join(out_dir, f"{seq}_xy.pdf")
    cmd_xy = [
        "evo_traj", "tum",
        args.dual_tum, args.single_tum,
        "-p", "--plot_mode", "xy",
        "--save_plot", plot_xy,
        "--t_max_diff", "0.5"
    ]
    print(f"\nGenerating XY plot...")
    subprocess.run(cmd_xy)
    if os.path.exists(plot_xy):
        print(f"  -> {plot_xy}")

    # XYZ trajectory plot
    plot_xyz = os.path.join(out_dir, f"{seq}_xyz.pdf")
    cmd_xyz = [
        "evo_traj", "tum",
        args.dual_tum, args.single_tum,
        "-p", "--plot_mode", "xyz",
        "--save_plot", plot_xyz
    ]
    print(f"\nGenerating XYZ plot...")
    subprocess.run(cmd_xyz)
    if os.path.exists(plot_xyz):
        print(f"  -> {plot_xyz}")

    # Trajectory statistics
    print(f"\n{'='*60}")
    print("Trajectory Statistics:")
    print(f"{'='*60}")

    for label, f in [("Dual-LiDAR", args.dual_tum),
                      ("Single-LiDAR", args.single_tum)]:
        # Count points and compute trajectory length
        with open(f) as fp:
            lines = fp.readlines()
        poses = [l.strip().split() for l in lines if l.strip()]
        n_poses = len(poses)

        # Compute total path length
        import math
        total_dist = 0.0
        for i in range(1, min(len(poses), 2000)):
            x1, y1, z1 = float(poses[i-1][1]), float(poses[i-1][2]), float(poses[i-1][3])
            x2, y2, z2 = float(poses[i][1]), float(poses[i][2]), float(poses[i][3])
            total_dist += math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

        # Compute Z variation
        zs = [float(p[3]) for p in poses[:2000]]
        z_range = max(zs) - min(zs) if zs else 0

        print(f"\n{label}:")
        print(f"  Poses:       {n_poses}")
        print(f"  Path length: {total_dist:.1f} m (first 2000 poses)")
        print(f"  Z range:     {z_range:.2f} m")

    print(f"\nDone. Plots saved to: {out_dir}")


if __name__ == "__main__":
    main()
