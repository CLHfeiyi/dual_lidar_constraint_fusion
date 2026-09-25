#!/usr/bin/env python3
"""约束级融合 vs 点云拼接 消融：全套图 + 数据 dump。

生成：
  WT3（退化隧道，对齐巴士 GT）:
    wt3_ablation_trajectory.png   原始 XY + ICP 对齐 XY
    wt3_ablation_error.png        残差随沿程距离 / 时间
    wt3_ablation_progress.png     累计里程（暴露停滞 vs 单调前进）
    wt3_ablation_decompose.png    横向/纵向误差分解
    wt3_ablation_cdf.png          残差 CDF
    wt3_ablation_metrics.png      汇总柱状图
  HH2（开阔步道，对齐 RTK GT）:
    hh2_ablation_trajectory.png   XY + 3D
    hh2_ablation_ape_cdf.png      APE CDF
    hh2_ablation_metrics.png      汇总柱状图

  data/ablation_metrics.json      全部指标
  data/wt3_aligned.npz            对齐轨迹 + 巴士 GT
  data/hh2_aligned.npz            对齐轨迹 + RTK GT
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.tools import file_interface

# ---------- paths ----------
BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"
HH2_GT = "/home/clh/视频/HK_MEMS_Dataset/grt/gps_path/hh2_gps_path_grt_clean.txt"
RES = "/home/clh/桌面/data/HKMEMS-results/"
FIG = "/home/clh/文章算法/experiments/figures/"
DAT = "/home/clh/文章算法/experiments/data/"

WT3_RUNS = {
    "CONSTRAINT-FUSION": RES + "WT3_dual_extfix.tum",
    "POINT-CLOUD-MERGE": RES + "WT3_dual_merge.tum",
}
HH2_RUNS = {
    "CONSTRAINT-FUSION": RES + "HH2_dual_fusion.tum",
    "POINT-CLOUD-MERGE": RES + "HH2_dual_merge.tum",
}
COL = {"CONSTRAINT-FUSION": "#d62728", "POINT-CLOUD-MERGE": "#1f77b4"}

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


def umeyama(P, Q):
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, mq - R @ mp


def icp2d(P, Q, it=60):
    P = P.copy()
    for _ in range(it):
        d = np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=2)
        Qc = Q[d.argmin(1)]
        R, t = umeyama(P, Qc)
        P = (R @ P.T).T + t
    return P


def plen(a):
    return float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())


# ================= WT3 =================
bus = load_bus(BUS)
Q = bus[:, [1, 0]]                       # (east, north) -> (x, y)
dc = np.linalg.norm(np.diff(Q, axis=0), axis=1)
cum = np.concatenate([[0], np.cumsum(dc)])


def wt3_metrics(P, xyz):
    slen = plen(xyz)
    Pa = icp2d(P, Q)
    d = np.linalg.norm(Pa[:, None, :] - Q[None, :, :], axis=2)
    resid = d.min(1)
    idx = d.argmin(1)
    cov = cum[idx.max()] - cum[idx.min()]
    infl = slen / max(cov, 1.0)
    # tangent-based along/cross decomposition
    i_lo = np.clip(idx - 1, 0, len(Q) - 1)
    i_hi = np.clip(idx + 1, 0, len(Q) - 1)
    tang = Q[i_hi] - Q[i_lo]
    tn = np.linalg.norm(tang, axis=1, keepdims=True)
    tang = tang / np.maximum(tn, 1e-9)
    rvec = Pa - Q[idx]
    along = (rvec * tang).sum(1)                       # signed along-track
    cross = tang[:, 0] * rvec[:, 1] - tang[:, 1] * rvec[:, 0]  # signed cross-track
    diff = np.diff(idx)
    fwd = (diff > 0).mean() * 100
    back = (diff < 0).mean() * 100
    return dict(resid=float(resid.mean()), resid_std=float(resid.std()),
                cov=float(cov), slen=float(slen), infl=float(infl),
                fwd=float(fwd), back=float(back),
                along_mean=float(np.abs(along).mean()), along_std=float(along.std()),
                cross_mean=float(np.abs(cross).mean()), cross_std=float(cross.std()),
                n=len(P)), Pa, idx, resid, along, cross, cum


wt3 = {}
for name, traj in WT3_RUNS.items():
    t = file_interface.read_tum_trajectory_file(traj)
    xyz = t.positions_xyz
    P = xyz[:, :2]
    m, Pa, idx, resid, along, cross, cumv = wt3_metrics(P, xyz)
    wt3[name] = dict(metrics=m, Pa=Pa, idx=idx, resid=resid,
                     along=along, cross=cross, xyz=xyz)
    print(f"[WT3] {name}: resid={m['resid']:.3f}  cov={m['cov']:.1f}  "
          f"infl={m['infl']:.2f}  fwd={m['fwd']:.0f}%/back={m['back']:.0f}%  "
          f"cross|mean|={m['cross_mean']:.2f}  along|mean|={m['along_mean']:.2f}")

# --- fig 1: trajectory (raw + aligned) ---
fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
for name, traj in WT3_RUNS.items():
    t = file_interface.read_tum_trajectory_file(traj)
    ax[0].plot(t.positions_xyz[:, 0], t.positions_xyz[:, 1],
               COL[name], lw=0.7, label=name)
ax[0].set_title("WT3 raw local coords (m)")
ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
ax[0].axis("equal"); ax[0].legend(loc="best")

ax[1].plot(Q[:, 0], Q[:, 1], "k-", lw=0.5, alpha=0.7, label="bus GT (3.27 km)")
for name in WT3_RUNS:
    ax[1].plot(wt3[name]["Pa"][:, 0], wt3[name]["Pa"][:, 1],
               COL[name], lw=0.8, label=name)
ax[1].set_title("WT3 ICP-aligned onto bus GT")
ax[1].set_xlabel("east (m)"); ax[1].set_ylabel("north (m)")
ax[1].axis("equal"); ax[1].legend(loc="best")
fig.suptitle("WT3 消融：约束级融合 vs 点云拼接（轨迹）")
fig.savefig(FIG + "wt3_ablation_trajectory.png"); plt.close(fig)

# --- fig 2: error vs along-track index ---
fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
for name in WT3_RUNS:
    ax[0].plot(wt3[name]["idx"], wt3[name]["resid"], COL[name], lw=0.6, label=name)
ax[0].set_title("residual vs bus along-track index")
ax[0].set_xlabel("bus GT index"); ax[0].set_ylabel("nearest distance (m)")
ax[0].legend()
for name in WT3_RUNS:
    ax[1].plot(np.arange(len(wt3[name]["resid"])), wt3[name]["resid"],
               COL[name], lw=0.6, label=name)
ax[1].set_title("residual vs trajectory sample")
ax[1].set_xlabel("sample #"); ax[1].set_ylabel("nearest distance (m)")
ax[1].legend()
fig.suptitle("WT3 残差时间序列")
fig.savefig(FIG + "wt3_ablation_error.png"); plt.close(fig)

# --- fig 3: cumulative progress (stall vs monotonic) ---
fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
for name in WT3_RUNS:
    P = wt3[name]["xyz"][:, :2]
    step = np.linalg.norm(np.diff(P, axis=0), axis=1)
    ax[0].plot(np.concatenate([[0], np.cumsum(step)]), COL[name], lw=1.2, label=name)
ax[0].set_title("cumulative SLAM path vs sample")
ax[0].set_xlabel("sample #"); ax[0].set_ylabel("cumulative path (m)")
ax[0].legend()
for name in WT3_RUNS:
    ax[1].plot(wt3[name]["idx"], COL[name], lw=0.7, label=name)
ax[1].set_title("bus GT index vs trajectory sample (monotonic = progress)")
ax[1].set_xlabel("sample #"); ax[1].set_ylabel("bus GT index")
ax[1].legend()
fig.suptitle("WT3 进度（拼接停滞 vs 融合单调前进）")
fig.savefig(FIG + "wt3_ablation_progress.png"); plt.close(fig)

# --- fig 4: cross/along decomposition ---
fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
for name in WT3_RUNS:
    ax[0].plot(wt3[name]["cross"], COL[name], lw=0.6, label=name)
ax[0].set_title("cross-track error (signed, m)")
ax[0].legend()
for name in WT3_RUNS:
    ax[1].plot(wt3[name]["along"], COL[name], lw=0.6, label=name)
ax[1].set_title("along-track error (signed, m)")
ax[1].set_xlabel("sample #"); ax[1].legend()
fig.suptitle("WT3 横向/纵向误差分解")
fig.savefig(FIG + "wt3_ablation_decompose.png"); plt.close(fig)

# --- fig 5: CDF ---
fig, ax = plt.subplots(figsize=(7, 4.5))
for name in WT3_RUNS:
    r = np.sort(wt3[name]["resid"])
    ax.plot(r, np.linspace(0, 1, len(r)), COL[name], lw=1.4, label=name)
ax.set_title("WT3 残差 CDF")
ax.set_xlabel("residual (m)"); ax.set_ylabel("cumulative fraction")
ax.legend(); ax.set_xlim(0, 40)
fig.savefig(FIG + "wt3_ablation_cdf.png"); plt.close(fig)

# --- fig 6: summary bars ---
labels = list(WT3_RUNS)
names = list(WT3_RUNS)
resid = [wt3[n]["metrics"]["resid"] for n in names]
cov = [wt3[n]["metrics"]["cov"] for n in names]
infl = [wt3[n]["metrics"]["infl"] for n in names]
cross = [wt3[n]["metrics"]["cross_mean"] for n in names]
fig, ax = plt.subplots(1, 4, figsize=(14, 3.6))
ax[0].bar(names, resid, color=[COL[n] for n in names])
ax[0].set_title("ICP residual (m)"); ax[0].tick_params(axis="x", rotation=20)
ax[1].bar(names, cov, color=[COL[n] for n in names])
ax[1].set_title("bus coverage (m)"); ax[1].tick_params(axis="x", rotation=20)
ax[2].bar(names, infl, color=[COL[n] for n in names])
ax[2].set_title("scale inflation (x)"); ax[2].tick_params(axis="x", rotation=20)
ax[3].bar(names, cross, color=[COL[n] for n in names])
ax[3].set_title("cross-track |mean| (m)"); ax[3].tick_params(axis="x", rotation=20)
fig.suptitle("WT3 消融汇总")
fig.savefig(FIG + "wt3_ablation_metrics.png"); plt.close(fig)

# ================= HH2 =================
hh2 = {}
for name, traj in HH2_RUNS.items():
    gt = file_interface.read_tum_trajectory_file(HH2_GT)
    tr = file_interface.read_tum_trajectory_file(traj)
    gtt, trt = gt.timestamps, tr.timestamps
    gi = np.clip(np.searchsorted(gtt, trt), 0, len(gtt) - 1)
    mask = np.abs(gtt[gi] - trt) <= 0.2
    P = tr.positions_xyz[mask]
    Qg = gt.positions_xyz[gi[mask]]
    R, t = umeyama(P, Qg)
    Pa = (R @ P.T).T + t
    err = np.linalg.norm(Pa - Qg, axis=1)
    m = dict(ape_rmse=float(np.sqrt((err ** 2).mean())),
             ape_mean=float(err.mean()), ape_std=float(err.std()),
             matched=int(len(P)), path=plen(tr.positions_xyz))
    hh2[name] = dict(metrics=m, Pa=Pa, Qg=Qg, err=err, tr=tr)
    print(f"[HH2] {name}: APE rmse={m['ape_rmse']:.3f}  mean={m['ape_mean']:.3f}  "
          f"matched={m['matched']}")

# --- HH2 trajectory ---
fig = plt.figure(figsize=(13, 5.5))
ax = fig.add_subplot(1, 2, 1)
gtp = gt.positions_xyz
ax.plot(gtp[:, 0], gtp[:, 1], "k-", lw=0.5, alpha=0.6, label="RTK GT")
for name in HH2_RUNS:
    ax.plot(hh2[name]["Pa"][:, 0], hh2[name]["Pa"][:, 1], COL[name], lw=0.8, label=name)
ax.set_title("HH2 XY (aligned)"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.axis("equal"); ax.legend()
ax3 = fig.add_subplot(1, 2, 2, projection="3d")
ax3.plot(gtp[:, 0], gtp[:, 1], gtp[:, 2], "k-", lw=0.5, alpha=0.5, label="RTK GT")
for name in HH2_RUNS:
    Pa = hh2[name]["Pa"]
    ax3.plot(Pa[:, 0], Pa[:, 1], Pa[:, 2], COL[name], lw=0.8, label=name)
ax3.set_title("HH2 3D (aligned)"); ax3.legend()
fig.suptitle("HH2 消融：约束级融合 vs 点云拼接（轨迹）")
fig.savefig(FIG + "hh2_ablation_trajectory.png"); plt.close(fig)

# --- HH2 APE CDF ---
fig, ax = plt.subplots(figsize=(7, 4.5))
for name in HH2_RUNS:
    r = np.sort(hh2[name]["err"])
    ax.plot(r, np.linspace(0, 1, len(r)), COL[name], lw=1.4, label=name)
ax.set_title("HH2 APE CDF")
ax.set_xlabel("APE (m)"); ax.set_ylabel("cumulative fraction")
ax.legend()
fig.savefig(FIG + "hh2_ablation_ape_cdf.png"); plt.close(fig)

# --- HH2 summary bars ---
names = list(HH2_RUNS)
fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
ax[0].bar(names, [hh2[n]["metrics"]["ape_rmse"] for n in names],
          color=[COL[n] for n in names])
ax[0].set_title("APE RMSE (m)"); ax[0].tick_params(axis="x", rotation=20)
ax[1].bar(names, [hh2[n]["metrics"]["ape_mean"] for n in names],
          color=[COL[n] for n in names])
ax[1].set_title("APE mean (m)"); ax[1].tick_params(axis="x", rotation=20)
fig.suptitle("HH2 消融汇总")
fig.savefig(FIG + "hh2_ablation_metrics.png"); plt.close(fig)

# ================= dump data =================
dump = {
    "WT3": {n: wt3[n]["metrics"] for n in WT3_RUNS},
    "HH2": {n: hh2[n]["metrics"] for n in HH2_RUNS},
    "WT3_bus_GT": {"path": BUS, "n_pts": len(Q), "total_length_m": float(cum[-1])},
    "HH2_GT": {"path": HH2_GT, "n_pts": len(gt.positions_xyz),
               "total_length_m": plen(gt.positions_xyz)},
}
with open(DAT + "ablation_metrics.json", "w") as f:
    json.dump(dump, f, indent=2, ensure_ascii=False)

np.savez(DAT + "wt3_aligned.npz",
         bus_GT=Q, bus_cum=cum,
         fusion_Pa=wt3["CONSTRAINT-FUSION"]["Pa"],
         merge_Pa=wt3["POINT-CLOUD-MERGE"]["Pa"],
         fusion_resid=wt3["CONSTRAINT-FUSION"]["resid"],
         merge_resid=wt3["POINT-CLOUD-MERGE"]["resid"])
np.savez(DAT + "hh2_aligned.npz",
         gt=hh2["CONSTRAINT-FUSION"]["Qg"],
         fusion_Pa=hh2["CONSTRAINT-FUSION"]["Pa"],
         merge_Pa=hh2["POINT-CLOUD-MERGE"]["Pa"],
         fusion_err=hh2["CONSTRAINT-FUSION"]["err"],
         merge_err=hh2["POINT-CLOUD-MERGE"]["err"])

print("\nSaved figures to", FIG)
print("Saved data to", DAT)
print("ablation_metrics.json:\n", json.dumps(dump, indent=2, ensure_ascii=False))
