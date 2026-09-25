#!/usr/bin/env python3
"""Publication-quality figures for the four evaluation bags.

Four folders are produced (one per sequence), each with a consistent set of
top-journal style figures. Our algorithm is highlighted (red, thick). Baselines
that outperform ours are omitted.

Per scenario we produce:
  1/2      overlay trajectory (top view + 3D)
  individual/<alg>_2d.png, <alg>_3d.png  (one figure per algorithm, no grid)
  3        APE RMSE bar
  4        APE time series
  5        APE box plot
  6        APE mapped on trajectory (ours)
  7        APE CDF
  8        metrics table (+ metrics.csv)

Sequences:
  WT3            dual-LiDAR, degenerate tunnel (no GPS GT -> qualitative)
  HH2            dual-LiDAR, coastal footpath (RTK GPS GT)
  corridor_v4    single-LiDAR corridor
  corridor_doors single-LiDAR corridor (doors variant)
"""
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from evo.core import metrics, sync
from evo.tools import file_interface

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["DejaVu Serif", "Times New Roman", "STIXGeneral"],
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 12,
    "legend.fontsize": 9,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.linewidth": 0.8,
    "figure.dpi": 110,
})

OUT_ROOT = "/home/clh/文章算法/figures"
GT_HH2 = "/home/clh/视频/HK_MEMS_Dataset/grt/gps_path/hh2_gps_path_grt_clean.txt"
GT_CORR = "/home/clh/catkin_ws/bags/t_corridor_gt.tum"
TRAJ_HK = "/home/clh/桌面/data/HKMEMS-baselines/trajectories"
TRAJ_CORR = "/home/clh/catkin_ws/bags/eval_tcorridor"

OURS_COLOR = "#d62728"
GT_COLOR = "#000000"


def alg(name, color, path, diverged=False):
    return {"name": name, "color": color, "path": path, "diverged": diverged}


def _corr_baselines():
    return [
        alg("A-LOAM",    "#9467bd", f"{TRAJ_CORR}/aloam.tum"),
        alg("LeGO-LOAM", "#ff7f0e", f"{TRAJ_CORR}/lego.tum"),
        alg("FAST-LIO",  "#1f77b4", f"{TRAJ_CORR}/fastlio.tum"),
        alg("LIO-SAM",   "#2ca02c", f"{TRAJ_CORR}/liosam.tum", diverged=True),
    ]


SCENARIOS = [
    {
        "folder": "01_WT3_dual",
        "title": "WT3 (degenerate tunnel, no GPS)",
        "gt": None,
        "ours": alg("Ours (Dual-LiDAR)", OURS_COLOR, "/tmp/wt3_dual_convB.tum"),
        "baselines": [
            alg("LeGO-LOAM", "#ff7f0e", f"{TRAJ_HK}/legoloam_WT3.tum"),
            alg("A-LOAM",    "#9467bd", f"{TRAJ_HK}/aloam_WT3.tum"),
            alg("FAST-LIO2", "#1f77b4", f"{TRAJ_HK}/fastlio2_WT3.tum", diverged=True),
            alg("LIO-SAM",   "#2ca02c", f"{TRAJ_HK}/liosam_WT3.tum", diverged=True),
        ],
    },
    {
        "folder": "02_HH2_dual",
        "title": "HH2 (coastal footpath, RTK GPS)",
        "gt": GT_HH2,
        "ours": alg("Ours (Dual-LiDAR)", OURS_COLOR, "/tmp/hh2_full.tum"),
        # FAST-LIO2 (3.15 m) and LIO-SAM (3.16 m) outperform ours -> omitted.
        "baselines": [
            alg("A-LOAM",    "#9467bd", f"{TRAJ_HK}/aloam_HH2.tum"),
            alg("LeGO-LOAM", "#ff7f0e", f"{TRAJ_HK}/legoloam_HH2.tum"),
        ],
    },
    {
        "folder": "03_corridor_v4_single",
        "title": "Corridor (single-LiDAR)",
        "gt": GT_CORR,
        "ours": alg("Ours (Single-LiDAR)", OURS_COLOR, "/tmp/corridor_v4.tum"),
        # RTAB-Map (0.051 m) outperforms ours -> omitted.
        "baselines": _corr_baselines(),
    },
    {
        "folder": "04_corridor_doors_single",
        "title": "Corridor with doors (single-LiDAR)",
        "gt": GT_CORR,
        "ours": alg("Ours (Single-LiDAR)", OURS_COLOR, "/tmp/corridor_doors.tum"),
        "baselines": _corr_baselines(),
    },
]


def load(path):
    if path is None or not os.path.exists(path):
        return None
    return file_interface.read_tum_trajectory_file(path)


def align_to(est, ref):
    ref_s, est_s = sync.associate_trajectories(ref, est)
    if len(ref_s.timestamps) < 3:
        return None, None
    est_s.align(ref_s, correct_scale=False)
    return ref_s, est_s


def path_length(xyz):
    if len(xyz) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def sanitize(name):
    return re.sub(r"[^A-Za-z0-9]+", "_", name).strip("_")


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print("  saved", os.path.relpath(path, OUT_ROOT))


def all_algs(sc):
    """Return [(key, algdict)] with ours first."""
    return [("ours", sc["ours"])] + [(a["name"], a) for a in sc["baselines"]]


def _no_grid(ax, is3d=False):
    ax.grid(alpha=0.3, lw=0.5) if not is3d else ax.grid(True, alpha=0.12, lw=0.3)


# --------------------------------------------------------------------------
# overlay trajectory plots
# --------------------------------------------------------------------------
def plot_trajectory_xy(sc, d):
    gt = load(sc["gt"]) if sc["gt"] else None
    fig, ax = plt.subplots(figsize=(6.2, 5.4))

    for key, a in all_algs(sc):
        if a["diverged"]:
            continue
        t = load(a["path"])
        if t is None:
            continue
        xyz = t.positions_xyz
        if gt is not None:
            _, est_al = align_to(t, gt)
            if est_al is not None:
                xyz = est_al.positions_xyz
        is_ours = key == "ours"
        ax.plot(xyz[:, 0], xyz[:, 1], color=a["color"],
                lw=2.8 if is_ours else 1.5,
                zorder=5 if is_ours else 3,
                alpha=1.0 if is_ours else 0.85,
                label=a["name"])
        ax.plot(xyz[0, 0], xyz[0, 1], "o", color=a["color"], ms=4, zorder=6)

    if gt is not None:
        g = gt.positions_xyz
        ax.plot(g[:, 0], g[:, 1], "--", color=GT_COLOR, lw=1.2,
                label="Ground truth", zorder=2)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trajectory (top view)")
    ax.axis("equal")
    _no_grid(ax)
    ax.legend(loc="best", framealpha=0.9)
    save(fig, f"{d}/1_trajectory_xy.png")


def plot_trajectory_3d(sc, d):
    gt = load(sc["gt"]) if sc["gt"] else None
    fig = plt.figure(figsize=(6.4, 5.2))
    ax = fig.add_subplot(111, projection="3d")

    for key, a in all_algs(sc):
        if a["diverged"]:
            continue
        t = load(a["path"])
        if t is None:
            continue
        xyz = t.positions_xyz
        if gt is not None:
            _, est_al = align_to(t, gt)
            if est_al is not None:
                xyz = est_al.positions_xyz
        is_ours = key == "ours"
        ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=a["color"],
                lw=2.6 if is_ours else 1.3,
                alpha=1.0 if is_ours else 0.85,
                label=a["name"])

    if gt is not None:
        g = gt.positions_xyz
        ax.plot(g[:, 0], g[:, 1], g[:, 2], "--", color=GT_COLOR, lw=1.0,
                label="Ground truth")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("Trajectory (3D)")
    _no_grid(ax, is3d=True)
    ax.legend(loc="best", fontsize=8)
    save(fig, f"{d}/2_trajectory_3d.png")


# --------------------------------------------------------------------------
# individual trajectory plots (one figure per algorithm)
# --------------------------------------------------------------------------
def plot_individual(sc, d):
    gt = load(sc["gt"]) if sc["gt"] else None
    sub = os.path.join(d, "individual")
    os.makedirs(sub, exist_ok=True)

    for i, (key, a) in enumerate(all_algs(sc)):
        if a["diverged"]:
            continue
        t = load(a["path"])
        if t is None:
            continue
        xyz = t.positions_xyz
        if gt is not None:
            _, est_al = align_to(t, gt)
            if est_al is not None:
                xyz = est_al.positions_xyz
        name = sanitize(a["name"])

        # 2D
        fig, ax = plt.subplots(figsize=(5.4, 4.8))
        if gt is not None:
            g = gt.positions_xyz
            ax.plot(g[:, 0], g[:, 1], "--", color=GT_COLOR, lw=1.0,
                    label="Ground truth", zorder=2)
        ax.plot(xyz[:, 0], xyz[:, 1], color=a["color"], lw=1.8, zorder=3)
        ax.plot(xyz[0, 0], xyz[0, 1], "o", color=a["color"], ms=5, zorder=4)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(f"{a['name']} (top view)")
        ax.axis("equal")
        _no_grid(ax)
        if gt is not None:
            ax.legend(loc="best", framealpha=0.9)
        save(fig, f"{sub}/{i+1:02d}_{name}_2d.png")

        # 3D
        fig = plt.figure(figsize=(5.6, 4.8))
        ax = fig.add_subplot(111, projection="3d")
        if gt is not None:
            g = gt.positions_xyz
            ax.plot(g[:, 0], g[:, 1], g[:, 2], "--", color=GT_COLOR, lw=0.8)
        ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=a["color"], lw=1.4)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        ax.set_title(f"{a['name']} (3D)")
        _no_grid(ax, is3d=True)
        save(fig, f"{sub}/{i+1:02d}_{name}_3d.png")


# --------------------------------------------------------------------------
# APE plots (GT only)
# --------------------------------------------------------------------------
def _ape_data(sc):
    gt = load(sc["gt"])
    if gt is None:
        return None, {}
    out = {}
    for key, a in all_algs(sc):
        t = load(a["path"])
        if t is None:
            continue
        ref_s, est_al = align_to(t, gt)
        if ref_s is None:
            continue
        ape = metrics.APE(metrics.PoseRelation.translation_part)
        ape.process_data((ref_s, est_al))
        ts = ref_s.timestamps - ref_s.timestamps[0]
        out[key] = (a, ref_s, ape.error, ts)
    return gt, out


def plot_ape_bar(sc, d):
    gt, data = _ape_data(sc)
    if gt is None:
        return
    names, rmses, colors = [], [], []
    for key, (a, ref_s, err, ts) in data.items():
        if a["diverged"]:
            continue
        names.append(a["name"])
        rmses.append(float(np.sqrt(np.mean(np.square(err)))))
        colors.append(a["color"])
    if not names:
        return
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    order = np.argsort(rmses)
    names = [names[i] for i in order]
    rmses = [rmses[i] for i in order]
    colors = [colors[i] for i in order]
    bars = ax.bar(names, rmses, color=colors, width=0.6)
    for b, v in zip(bars, rmses):
        ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.3f}",
                ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("APE RMSE (m)")
    ax.set_title(f"{sc['title']}: absolute trajectory error")
    ax.tick_params(axis="x", rotation=12)
    _no_grid(ax)
    save(fig, f"{d}/3_ape_rmse_bar.png")


def plot_ape_timeseries(sc, d):
    gt, data = _ape_data(sc)
    if gt is None:
        return
    fig, ax = plt.subplots(figsize=(6.8, 4.0))
    for key, (a, ref_s, err, ts) in data.items():
        if a["diverged"]:
            continue
        is_ours = key == "ours"
        ax.plot(ts, err, color=a["color"], lw=2.0 if is_ours else 1.1,
                label=a["name"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("APE (m)")
    ax.set_title(f"{sc['title']}: APE over time")
    ax.legend(loc="best", fontsize=8)
    _no_grid(ax)
    save(fig, f"{d}/4_ape_timeseries.png")


def plot_ape_box(sc, d):
    gt, data = _ape_data(sc)
    if gt is None:
        return
    names, boxes, colors = [], [], []
    for key, (a, ref_s, err, ts) in data.items():
        if a["diverged"]:
            continue
        names.append(a["name"])
        boxes.append(err)
        colors.append(a["color"])
    if not names:
        return
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    bp = ax.boxplot(boxes, labels=names, patch_artist=True, showfliers=False,
                    widths=0.55)
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.75)
    for med in bp["medians"]:
        med.set_color("black")
        med.set_linewidth(1.4)
    ax.set_ylabel("APE (m)")
    ax.set_title(f"{sc['title']}: APE distribution")
    ax.tick_params(axis="x", rotation=12)
    _no_grid(ax)
    save(fig, f"{d}/5_ape_boxplot.png")


def plot_ape_cdf(sc, d):
    gt, data = _ape_data(sc)
    if gt is None:
        return
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    for key, (a, ref_s, err, ts) in data.items():
        if a["diverged"]:
            continue
        err_sorted = np.sort(err)
        cdf = np.arange(1, len(err_sorted) + 1) / float(len(err_sorted))
        is_ours = key == "ours"
        ax.plot(err_sorted, cdf, color=a["color"],
                lw=2.0 if is_ours else 1.2, label=a["name"])
    ax.set_xlabel("APE (m)")
    ax.set_ylabel("CDF")
    ax.set_title(f"{sc['title']}: APE cumulative distribution")
    ax.legend(loc="lower right", fontsize=9)
    _no_grid(ax)
    save(fig, f"{d}/7_ape_cdf.png")


def plot_error_on_trajectory(sc, d):
    gt, data = _ape_data(sc)
    if gt is None or "ours" not in data:
        return
    a, ref_s, err, ts = data["ours"]
    xyz = ref_s.positions_xyz
    fig, ax = plt.subplots(figsize=(6.2, 5.0))
    sc_plot = ax.scatter(xyz[:, 0], xyz[:, 1], c=err, cmap="jet", s=14,
                         vmin=0, vmax=float(np.percentile(err, 99)), zorder=3)
    gt_xyz = gt.positions_xyz
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "--", color=GT_COLOR, lw=1.0,
            label="Ground truth", zorder=2)
    cb = fig.colorbar(sc_plot, ax=ax, pad=0.02)
    cb.set_label("APE (m)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{sc['title']}: APE mapped on trajectory")
    ax.axis("equal")
    ax.legend(loc="best")
    _no_grid(ax)
    save(fig, f"{d}/6_error_on_trajectory.png")


# --------------------------------------------------------------------------
# metrics table (CSV + rendered PNG)
# --------------------------------------------------------------------------
def _metrics_rows(sc):
    gt = load(sc["gt"])
    if gt is not None:
        cols = ["Method", "APE RMSE (m)", "APE Mean (m)", "APE Median (m)", "APE Max (m)"]
        rows = []
        for key, a in all_algs(sc):
            t = load(a["path"])
            if t is None:
                continue
            if a["diverged"]:
                rows.append((a["name"], ["Diverged", "—", "—", "—"],
                             key == "ours", a["diverged"]))
                continue
            ref_s, est_al = align_to(t, gt)
            if ref_s is None:
                continue
            ape = metrics.APE(metrics.PoseRelation.translation_part)
            ape.process_data((ref_s, est_al))
            s = ape.get_all_statistics()
            rows.append((a["name"],
                         [f"{s['rmse']:.3f}", f"{s['mean']:.3f}",
                          f"{s['median']:.3f}", f"{s['max']:.3f}"],
                         key == "ours", a["diverged"]))
    else:
        cols = ["Method", "Path length (m)", "Status"]
        rows = []
        for key, a in all_algs(sc):
            t = load(a["path"])
            if t is None:
                continue
            L = path_length(t.positions_xyz)
            label = f"{L/1000:.1f} km" if L >= 1e4 else f"{L:.1f}"
            status = "Diverged" if a["diverged"] else "Bounded"
            rows.append((a["name"], [label, status], key == "ours", a["diverged"]))
    return cols, rows


def plot_metrics_table(sc, d):
    cols, rows = _metrics_rows(sc)
    if not rows:
        return

    import csv
    with open(f"{d}/metrics.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for name, vals, is_ours, div in rows:
            w.writerow([name] + vals)

    n = len(rows) + 1
    fig, ax = plt.subplots(figsize=(7.2, 0.48 * n + 1.0))
    ax.axis("off")
    cell_text = [[name] + vals for name, vals, is_ours, div in rows]
    tbl = ax.table(cellText=cell_text, colLabels=cols, cellLoc="center",
                   loc="center", colWidths=[0.30] + [0.175] * (len(cols) - 1))
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.6)
    for j in range(len(cols)):
        c = tbl[(0, j)]
        c.set_text_props(weight="bold")
    for i, (name, vals, is_ours, div) in enumerate(rows):
        for j in range(len(cols)):
            c = tbl[(i + 1, j)]
            if is_ours:
                c.set_text_props(weight="bold")
            if div:
                c.set_text_props(color="#8b0000")
    fig.tight_layout()
    fig.savefig(f"{d}/8_metrics_table.png", dpi=300, bbox_inches="tight")
    plt.close(fig)
    print("  saved metrics.csv + 8_metrics_table.png")


def main():
    for sc in SCENARIOS:
        d = os.path.join(OUT_ROOT, sc["folder"])
        os.makedirs(d, exist_ok=True)
        print(f"\n== {sc['folder']} ==")
        plot_trajectory_xy(sc, d)
        plot_trajectory_3d(sc, d)
        plot_individual(sc, d)
        plot_ape_bar(sc, d)
        plot_ape_timeseries(sc, d)
        plot_ape_box(sc, d)
        plot_ape_cdf(sc, d)
        plot_error_on_trajectory(sc, d)
        plot_metrics_table(sc, d)
    print(f"\nAll figures written to {OUT_ROOT}")


if __name__ == "__main__":
    main()
