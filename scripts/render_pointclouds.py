#!/usr/bin/env python3
"""Render point-cloud maps (top-view + 3D) for every algorithm of every scenario.

Each (scenario, algorithm) pair writes two PNGs into the scenario folder:
  9_map_topview_<alg>.png  10_map_3d_<alg>.png

Maps are cropped to the algorithm's own trajectory (+margin) so distant lidar
returns (up to the 1000 m max range) don't shrink the actual scene. Cropping
uses a trajectory PCD or TUM file; algorithms without one are shown uncropped.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import open3d as o3d

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["DejaVu Serif", "Times New Roman", "STIXGeneral"],
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.linewidth": 0.8,
})

OUT_ROOT = "/home/clh/文章算法/figures"
MAX_PTS = 300000
CROP_MARGIN = 25.0
CROP_Z_UP = 25.0
CROP_Z_DN = 12.0

# corridor (own single-LiDAR data) maps — already tight, no crop needed
CORR_OUR = "/home/clh/catkin_ws/bags/eval_tcorridor/pointcloud_screenshots/our_cleaned_hq.pcd"
CORR_ALOAM = "/home/clh/catkin_ws/bags/eval_tcorridor/pointcloud_screenshots/aloam_cleaned_hq.pcd"
CORR_LEGO = "/home/clh/catkin_ws/bags/paper_comparison/lego_loam_map_corridor.pcd"
CORR_RTAB = "/home/clh/catkin_ws/bags/eval_tcorridor/pointcloud_screenshots/rtabmap_cleaned.pcd"

# public-dataset baseline maps + trajectories
BMAP = "/home/clh/桌面/data/HKMEMS-baselines/maps"
BTRAJ = "/home/clh/桌面/data/HKMEMS-baselines/trajectories"


def alg(label, pcd, traj=None):
    return {"label": label, "pcd": pcd, "traj": traj}


SCENES = {
    "01_WT3_dual": {
        "title": "WT3 (degenerate tunnel)",
        "algs": [
            alg("Ours", "/home/clh/catkin_ws/bags/wt3_map/GlobalMap.pcd",
                "/home/clh/catkin_ws/bags/wt3_map/trajectory.pcd"),
            alg("A-LOAM", f"{BMAP}/aloam_WT3.pcd", f"{BTRAJ}/aloam_WT3.tum"),
            alg("LeGO-LOAM", f"{BMAP}/legoloam_WT3.pcd", f"{BTRAJ}/legoloam_WT3.tum"),
        ],
    },
    "02_HH2_dual": {
        "title": "HH2 (coastal footpath)",
        "algs": [
            alg("Ours", "/home/clh/catkin_ws/bags/hh2_map/GlobalMap.pcd",
                "/home/clh/catkin_ws/bags/hh2_map/trajectory.pcd"),
            alg("A-LOAM", f"{BMAP}/aloam_HH2.pcd", f"{BTRAJ}/aloam_HH2.tum"),
            alg("LeGO-LOAM", f"{BMAP}/legoloam_HH2.pcd", f"{BTRAJ}/legoloam_HH2.tum"),
        ],
    },
    "03_corridor_v4_single": {
        "title": "Corridor (single-LiDAR)",
        "algs": [
            alg("Ours", CORR_OUR),
            alg("A-LOAM", CORR_ALOAM),
            alg("LeGO-LOAM", CORR_LEGO),
            alg("RTAB-Map", CORR_RTAB),
        ],
    },
    "04_corridor_doors_single": {
        "title": "Corridor with doors (single-LiDAR)",
        "algs": [
            alg("Ours", CORR_OUR),
            alg("A-LOAM", CORR_ALOAM),
            alg("LeGO-LOAM", CORR_LEGO),
            alg("RTAB-Map", CORR_RTAB),
        ],
    },
}


def load_traj_crop(traj_path):
    """Return (xy_box, z_box) crop from a trajectory PCD or TUM file, or None."""
    if not traj_path or not os.path.exists(traj_path):
        return None
    if traj_path.endswith(".tum"):
        d = np.loadtxt(traj_path)
        if d.ndim != 2 or len(d) < 2:
            return None
        p = d[:, 1:4]
    else:
        c = o3d.io.read_point_cloud(traj_path)
        p = np.asarray(c.points)
    if len(p) == 0:
        return None
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    xy = (x.min() - CROP_MARGIN, x.max() + CROP_MARGIN,
          y.min() - CROP_MARGIN, y.max() + CROP_MARGIN)
    zr = (z.min() - CROP_Z_DN, z.max() + CROP_Z_UP)
    return xy, zr


def load_pts(pcd_path, crop):
    if not os.path.exists(pcd_path):
        return None
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return None
    if crop is not None:
        xy, zr = crop
        m = ((pts[:, 0] >= xy[0]) & (pts[:, 0] <= xy[1]) &
             (pts[:, 1] >= xy[2]) & (pts[:, 1] <= xy[3]) &
             (pts[:, 2] >= zr[0]) & (pts[:, 2] <= zr[1]))
        pts = pts[m]
        if len(pts) == 0:
            return None
    if len(pts) > MAX_PTS:
        idx = np.random.RandomState(0).choice(len(pts), MAX_PTS, replace=False)
        pts = pts[idx]
    return pts


def point_size(n):
    return float(np.clip(60000.0 / max(n, 1), 0.6, 6.0))


def render(pts, out_dir, title, label):
    z = pts[:, 2]
    s = point_size(len(pts))

    fig, ax = plt.subplots(figsize=(6.6, 5.6))
    sc = ax.scatter(pts[:, 0], pts[:, 1], c=z, s=s, cmap="viridis",
                    linewidths=0, alpha=0.7, rasterized=True)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("z (m)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{title} — {label} map (top view)")
    ax.axis("equal")
    ax.grid(alpha=0.3, lw=0.5)
    fig.tight_layout()
    fig.savefig(f"{out_dir}/9_map_topview_{label}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved 9_map_topview_{label}.png")

    fig = plt.figure(figsize=(6.8, 5.4))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=z, s=s * 0.8, cmap="viridis",
               linewidths=0, alpha=0.5, rasterized=True)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"{title} — {label} map (3D)")
    ax.grid(True, alpha=0.3, lw=0.4)
    fig.tight_layout()
    fig.savefig(f"{out_dir}/10_map_3d_{label}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved 10_map_3d_{label}.png")


def main():
    for folder, cfg in SCENES.items():
        out_dir = os.path.join(OUT_ROOT, folder)
        os.makedirs(out_dir, exist_ok=True)
        print(f"\n== {folder} ==")
        for a in cfg["algs"]:
            crop = load_traj_crop(a["traj"])
            pts = load_pts(a["pcd"], crop)
            if pts is None:
                print(f"  [{a['label']}] MISSING/EMPTY: {a['pcd']}")
                continue
            print(f"  [{a['label']}] {os.path.basename(a['pcd'])}: {len(pts)} pts")
            render(pts, out_dir, cfg["title"], a["label"])


if __name__ == "__main__":
    main()
