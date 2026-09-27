#!/usr/bin/env python3
"""CT1 (Central-Wan Chai Bypass 中環灣仔繞道) 约束级融合 vs 点云拼接 消融：指标 + 图 + dump。

对齐 CT1_18X_2_bus_grt_path.txt（巴士 2D GT），与 WT3 同一套评估管线。
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.tools import file_interface

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/CT1_18X_2_bus_grt_path.txt"
RES = "/home/clh/桌面/data/HKMEMS-results/"
FIG = "/home/clh/文章算法/experiments/figures/"
DAT = "/home/clh/文章算法/experiments/data/"

RUNS = {
    "CONSTRAINT-FUSION": RES + "CT1_dual_fusion.tum",
    "POINT-CLOUD-MERGE": RES + "CT1_dual_merge.tum",
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


bus = load_bus(BUS)
Q = bus[:, [1, 0]]                       # (east, north) -> (x, y)
dc = np.linalg.norm(np.diff(Q, axis=0), axis=1)
cum = np.concatenate([[0], np.cumsum(dc)])


def metrics(P, xyz):
    slen = plen(xyz)
    Pa = icp2d(P, Q)
    d = np.linalg.norm(Pa[:, None, :] - Q[None, :, :], axis=2)
    resid = d.min(1)
    idx = d.argmin(1)
    cov = cum[idx.max()] - cum[idx.min()]
    infl = slen / max(cov, 1.0)
    i_lo = np.clip(idx - 1, 0, len(Q) - 1)
    i_hi = np.clip(idx + 1, 0, len(Q) - 1)
    tang = Q[i_hi] - Q[i_lo]
    tn = np.linalg.norm(tang, axis=1, keepdims=True)
    tang = tang / np.maximum(tn, 1e-9)
    rvec = Pa - Q[idx]
    along = (rvec * tang).sum(1)
    cross = tang[:, 0] * rvec[:, 1] - tang[:, 1] * rvec[:, 0]
    diff = np.diff(idx)
    fwd = (diff > 0).mean() * 100
    back = (diff < 0).mean() * 100
    return dict(resid=float(resid.mean()), resid_std=float(resid.std()),
                cov=float(cov), slen=float(slen), infl=float(infl),
                fwd=float(fwd), back=float(back),
                along_mean=float(np.abs(along).mean()), along_std=float(along.std()),
                cross_mean=float(np.abs(cross).mean()), cross_std=float(cross.std()),
                n=len(P)), Pa, idx, resid, along, cross


ct1 = {}
for name, traj in RUNS.items():
    t = file_interface.read_tum_trajectory_file(traj)
    xyz = t.positions_xyz
    P = xyz[:, :2]
    m, Pa, idx, resid, along, cross = metrics(P, xyz)
    ct1[name] = dict(metrics=m, Pa=Pa, idx=idx, resid=resid,
                     along=along, cross=cross, xyz=xyz)
    print(f"[CT1] {name}: resid={m['resid']:.3f}  cov={m['cov']:.1f}  "
          f"infl={m['infl']:.2f}  fwd={m['fwd']:.0f}%/back={m['back']:.0f}%  "
          f"cross|mean|={m['cross_mean']:.2f}  along|mean|={m['along_mean']:.2f}")

# --- fig 1: trajectory (raw + aligned) ---
fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
for name in RUNS:
    ax[0].plot(ct1[name]["xyz"][:, 0], ct1[name]["xyz"][:, 1],
               COL[name], lw=0.7, label=name)
ax[0].set_title("CT1 raw local coords (m)")
ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
ax[0].axis("equal"); ax[0].legend(loc="best")

ax[1].plot(Q[:, 0], Q[:, 1], "k-", lw=0.5, alpha=0.7,
           label=f"bus GT ({cum[-1]/1000:.2f} km)")
for name in RUNS:
    ax[1].plot(ct1[name]["Pa"][:, 0], ct1[name]["Pa"][:, 1],
               COL[name], lw=0.8, label=name)
ax[1].set_title("CT1 ICP-aligned onto bus GT")
ax[1].set_xlabel("east (m)"); ax[1].set_ylabel("north (m)")
ax[1].axis("equal"); ax[1].legend(loc="best")
fig.suptitle("CT1 消融：约束级融合 vs 点云拼接（轨迹）")
fig.savefig(FIG + "ct1_ablation_trajectory.png"); plt.close(fig)

# --- fig 2: error vs along-track index ---
fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
for name in RUNS:
    ax[0].plot(ct1[name]["idx"], ct1[name]["resid"], COL[name], lw=0.6, label=name)
ax[0].set_title("residual vs bus along-track index")
ax[0].set_xlabel("bus GT index"); ax[0].set_ylabel("nearest distance (m)")
ax[0].legend()
for name in RUNS:
    ax[1].plot(np.arange(len(ct1[name]["resid"])), ct1[name]["resid"],
               COL[name], lw=0.6, label=name)
ax[1].set_title("residual vs trajectory sample")
ax[1].set_xlabel("sample #"); ax[1].set_ylabel("nearest distance (m)")
ax[1].legend()
fig.suptitle("CT1 残差时间序列")
fig.savefig(FIG + "ct1_ablation_error.png"); plt.close(fig)

# --- fig 3: cumulative progress ---
fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
for name in RUNS:
    P = ct1[name]["xyz"][:, :2]
    step = np.linalg.norm(np.diff(P, axis=0), axis=1)
    ax[0].plot(np.concatenate([[0], np.cumsum(step)]), COL[name], lw=1.2, label=name)
ax[0].set_title("cumulative SLAM path vs sample")
ax[0].set_xlabel("sample #"); ax[0].set_ylabel("cumulative path (m)")
ax[0].legend()
for name in RUNS:
    ax[1].plot(ct1[name]["idx"], COL[name], lw=0.7, label=name)
ax[1].set_title("bus GT index vs trajectory sample")
ax[1].set_xlabel("sample #"); ax[1].set_ylabel("bus GT index")
ax[1].legend()
fig.suptitle("CT1 进度（拼接停滞 vs 融合单调前进）")
fig.savefig(FIG + "ct1_ablation_progress.png"); plt.close(fig)

# --- fig 4: cross/along decomposition ---
fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
for name in RUNS:
    ax[0].plot(ct1[name]["cross"], COL[name], lw=0.6, label=name)
ax[0].set_title("cross-track error (signed, m)")
ax[0].legend()
for name in RUNS:
    ax[1].plot(ct1[name]["along"], COL[name], lw=0.6, label=name)
ax[1].set_title("along-track error (signed, m)")
ax[1].set_xlabel("sample #"); ax[1].legend()
fig.suptitle("CT1 横向/纵向误差分解")
fig.savefig(FIG + "ct1_ablation_decompose.png"); plt.close(fig)

# --- fig 5: CDF ---
fig, ax = plt.subplots(figsize=(7, 4.5))
for name in RUNS:
    r = np.sort(ct1[name]["resid"])
    ax.plot(r, np.linspace(0, 1, len(r)), COL[name], lw=1.4, label=name)
ax.set_title("CT1 残差 CDF")
ax.set_xlabel("residual (m)"); ax.set_ylabel("cumulative fraction")
ax.legend(); ax.set_xlim(0, 40)
fig.savefig(FIG + "ct1_ablation_cdf.png"); plt.close(fig)

# --- fig 6: summary bars ---
names = list(RUNS)
fig, ax = plt.subplots(1, 4, figsize=(14, 3.6))
ax[0].bar(names, [ct1[n]["metrics"]["resid"] for n in names],
          color=[COL[n] for n in names])
ax[0].set_title("ICP residual (m)"); ax[0].tick_params(axis="x", rotation=20)
ax[1].bar(names, [ct1[n]["metrics"]["cov"] for n in names],
          color=[COL[n] for n in names])
ax[1].set_title("bus coverage (m)"); ax[1].tick_params(axis="x", rotation=20)
ax[2].bar(names, [ct1[n]["metrics"]["infl"] for n in names],
          color=[COL[n] for n in names])
ax[2].set_title("scale inflation (x)"); ax[2].tick_params(axis="x", rotation=20)
ax[3].bar(names, [ct1[n]["metrics"]["cross_mean"] for n in names],
          color=[COL[n] for n in names])
ax[3].set_title("cross-track |mean| (m)"); ax[3].tick_params(axis="x", rotation=20)
fig.suptitle("CT1 消融汇总")
fig.savefig(FIG + "ct1_ablation_metrics.png"); plt.close(fig)

# --- dump ---
dump = {
    "CT1": {n: ct1[n]["metrics"] for n in RUNS},
    "CT1_bus_GT": {"path": BUS, "n_pts": len(Q), "total_length_m": float(cum[-1])},
}
with open(DAT + "ct1_ablation_metrics.json", "w") as f:
    json.dump(dump, f, indent=2, ensure_ascii=False)

np.savez(DAT + "ct1_aligned.npz",
         bus_GT=Q, bus_cum=cum,
         fusion_Pa=ct1["CONSTRAINT-FUSION"]["Pa"],
         merge_Pa=ct1["POINT-CLOUD-MERGE"]["Pa"],
         fusion_resid=ct1["CONSTRAINT-FUSION"]["resid"],
         merge_resid=ct1["POINT-CLOUD-MERGE"]["resid"])

print("\nSaved figures to", FIG)
print("Saved data to", DAT)
print("ct1_ablation_metrics.json:\n", json.dumps(dump, indent=2, ensure_ascii=False))
