#!/usr/bin/env python3
"""AT1 imuWeight 消融：验证"紧耦合 MEMS IMU 在 AT1 是否有害"。

对比 imuWeight=1.0(基线) / 0.5 / 0.1 / 0.0 的残差，参照 F-LOAM(纯激光单 M1)=3.59m。
对齐 AT1_77_1_bus_grt_path.txt（巴士 2D GT），与 plot_at1_ablation.py 同一套管线。
"""
import json
import numpy as np
from evo.tools import file_interface

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/AT1_77_1_bus_grt_path.txt"
RES = "/home/clh/桌面/data/HKMEMS-results/"
DAT = "/home/clh/文章算法/experiments/data/"

RUNS = {
    "imuW=1.0 (baseline fusion)": RES + "AT1_dual_imuw100.tum",
    "imuW=0.5": RES + "AT1_dual_imuw050.tum",
    "imuW=0.1": RES + "AT1_dual_imuw010.tum",
    "imuW=0.0 (lidar-only)": RES + "AT1_dual_imuw000.tum",
    "FAST-LIO2 (M1, IMU+lidar)": "/home/clh/视频/HK_MEMS_Dataset/evaluation_result/FAST-LIO2/all/fastlio_at1_path_grt.txt",
    "LIO-SAM (M1, IMU+lidar)": "/home/clh/视频/HK_MEMS_Dataset/evaluation_result/LIO-SAM/all/liosam_at1_path_grt.txt",
    "MEMS-LIO (M1, IMU+lidar)": "/home/clh/视频/HK_MEMS_Dataset/evaluation_result/memslio/all/memslio_at1_path_grt.txt",
    "Intensity-SLAM (M1, IMU+lidar)": "/home/clh/视频/HK_MEMS_Dataset/evaluation_result/Intensity-SLAM/all/intensity_at1_abe_path.txt",
}


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
                n=len(P))


print(f"{'method':34s} {'resid':>7s} {'cov':>7s} {'path':>8s} {'infl':>6s} "
      f"{'fwd':>5s} {'back':>5s} {'cross|m|':>8s} {'along|m|':>8s}  n")
out = {}
for name, traj in RUNS.items():
    try:
        t = file_interface.read_tum_trajectory_file(traj)
        xyz = t.positions_xyz
        m = metrics(xyz[:, :2], xyz)
        out[name] = m
        print(f"{name:34s} {m['resid']:7.3f} {m['cov']:7.1f} {m['slen']:8.1f} "
              f"{m['infl']:6.2f} {m['fwd']:5.0f} {m['back']:5.0f} "
              f"{m['cross_mean']:8.2f} {m['along_mean']:8.2f}  {m['n']}")
    except Exception as e:
        print(f"{name:34s} ERROR: {e}")

with open(DAT + "at1_imuw_metrics.json", "w") as f:
    json.dump(out, f, indent=2, ensure_ascii=False)
print("\nSaved", DAT + "at1_imuw_metrics.json")
