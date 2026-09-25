#!/usr/bin/env python3
"""单雷达 vs 双雷达（WT3）：轨迹 + 对数尺度指标。"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.tools import file_interface
from scipy.spatial import cKDTree

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"
RES = "/home/clh/桌面/data/HKMEMS-results/"
FIG = "/home/clh/文章算法/experiments/figures/"

RUNS = [
    ("M1-only (5ch)",        RES + "WT3_single_m1.tum",    "#7f7f7f"),
    ("Ouster-only (32ch)",   RES + "WT3_single_ouster.tum", "#ff7f0e"),
    ("DUAL merge",           RES + "WT3_dual_merge.tum",    "#1f77b4"),
    ("DUAL fusion",          RES + "WT3_dual_extfix.tum",   "#d62728"),
]

plt.rcParams.update({
    "font.size": 11, "axes.grid": True, "grid.alpha": 0.3,
    "grid.linewidth": 0.5, "figure.dpi": 110, "savefig.dpi": 150,
    "savefig.bbox": "tight",
    "font.sans-serif": ["Noto Sans CJK JP", "Droid Sans Fallback", "AR PL UMing CN", "DejaVu Sans"],
    "axes.unicode_minus": False,
})


def load_bus(path):
    raw = open(path).read().strip().splitlines()
    pts = []
    for line in raw[1:]:
        p = [x for x in line.replace(",", " ").split() if x]
        if len(p) >= 2:
            pts.append([float(p[0]), float(p[1])])
    return np.array(pts)


def plen(a):
    return float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())


def umeyama(P, Q):
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    return R, mq - R @ mp


def icp(P, Q, it=60):
    P = P.copy(); tree = cKDTree(Q)
    for _ in range(it):
        _, idx = tree.query(P)
        R, t = umeyama(P, Q[idx])
        P = (R @ P.T).T + t
    return P


bus = load_bus(BUS)
Q = bus[:, [1, 0]]
cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))])

res = {}
for name, traj, c in RUNS:
    t = file_interface.read_tum_trajectory_file(traj)
    xyz = t.positions_xyz
    P = xyz[:, :2]
    slen = plen(xyz)
    Pa = icp(P, Q)
    dists, idx = cKDTree(Q).query(Pa)
    resid = float(dists.mean())
    cov = cum[idx.max()] - cum[idx.min()]
    infl = slen / max(cov, 1.0)
    res[name] = dict(resid=resid, slen=slen, cov=cov, infl=infl,
                     raw=P, aligned=Pa, c=c)
    print(f"{name:22s} resid={resid:9.1f}m  path={slen:9.1f}m  infl={infl:5.2f}x")

# ---- Fig 1: trajectory ----
fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
for name, _, _ in RUNS:
    P = res[name]["raw"]
    ax[0].plot(P[:, 0], P[:, 1], res[name]["c"], lw=0.6, label=name)
ax[0].set_title("raw local coords (single-lidar diverges)")
ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
ax[0].axis("equal"); ax[0].legend(fontsize=8)

ax[1].plot(Q[:, 0], Q[:, 1], "k-", lw=0.6, alpha=0.7, label="bus GT (3.27 km)")
for name in ["DUAL fusion", "DUAL merge"]:
    Pa = res[name]["aligned"]
    ax[1].plot(Pa[:, 0], Pa[:, 1], res[name]["c"], lw=0.9, label=name)
ax[1].set_title("ICP-aligned onto bus GT (dual methods track)")
ax[1].set_xlabel("east (m)"); ax[1].set_ylabel("north (m)")
ax[1].axis("equal"); ax[1].legend(fontsize=8)
fig.suptitle("WT3 单雷达 vs 双雷达")
fig.savefig(FIG + "wt3_sensor_ablation_trajectory.png"); plt.close(fig)

# ---- Fig 2: log-scale metrics ----
names = [n for n, _, _ in RUNS]
cols = [res[n]["c"] for n in names]
resid = [res[n]["resid"] for n in names]
infl = [res[n]["infl"] for n in names]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
ax[0].bar(names, resid, color=cols)
ax[0].set_yscale("log")
ax[0].set_title("ICP residual (m, log scale)")
ax[0].set_ylabel("residual (m)")
for i, v in enumerate(resid):
    ax[0].text(i, v, f"{v:.0f}", ha="center", va="bottom", fontsize=8)
ax[0].tick_params(axis="x", rotation=15)
ax[1].bar(names, infl, color=cols)
ax[1].set_yscale("log")
ax[1].axhline(1.0, color="k", ls="--", lw=0.8, alpha=0.6)
ax[1].set_title("scale inflation (x, log scale; 1.0 = no drift)")
ax[1].set_ylabel("inflation (x)")
for i, v in enumerate(infl):
    ax[1].text(i, v, f"{v:.1f}x", ha="center", va="bottom", fontsize=8)
ax[1].tick_params(axis="x", rotation=15)
fig.suptitle("WT3 单雷达 vs 双雷达（对数尺度）")
fig.savefig(FIG + "wt3_sensor_ablation_metrics.png"); plt.close(fig)

print("\nSaved:", FIG + "wt3_sensor_ablation_trajectory.png")
print("Saved:", FIG + "wt3_sensor_ablation_metrics.png")
