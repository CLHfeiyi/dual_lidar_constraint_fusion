#!/usr/bin/env python3
"""degeneracyAwareFeatures 消融评估：WT3（静态退化，目标场景）与 AT1（动态场景，验证无回归）。

WT3 对比 extfix 基线（3.83m，imuW=1.0）；AT1 对比 imuW=0.1 基线（8.48m）。
metric = ICP 对齐 bus 2D GT 后的 resid / path / cover / infl。
"""
import json
import numpy as np
from evo.tools import file_interface
from scipy.spatial import cKDTree

BUS_AT1 = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/AT1_77_1_bus_grt_path.txt"
BUS_WT3 = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"
RES = "/home/clh/桌面/data/HKMEMS-results/"
DAT = "/home/clh/文章算法/experiments/data/"


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
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, mq - R @ mp


def icp(P, Q, it=60):
    P = P.copy()
    tree = cKDTree(Q)
    for _ in range(it):
        _, idx = tree.query(P)
        R, t = umeyama(P, Q[idx])
        P = (R @ P.T).T + t
    return P


def analyze(name, traj, bus):
    Q = bus[:, [1, 0]]
    cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))])
    t = file_interface.read_tum_trajectory_file(traj)
    xyz = t.positions_xyz
    P = xyz[:, :2]
    slen = plen(xyz)
    Pa = icp(P, Q)
    dists, idx = cKDTree(Q).query(Pa)
    resid = float(dists.mean())
    cov = cum[idx.max()] - cum[idx.min()]
    infl = slen / max(cov, 1.0)
    print(f"{name:34s} resid={resid:7.3f}m  path={slen:8.1f}m  cover={cov:7.1f}m  infl={infl:5.2f}x  n={len(P)}")
    return dict(resid=resid, path=slen, cover=cov, infl=infl, n=int(len(P)))


bus_at1 = load_bus(BUS_AT1)
bus_wt3 = load_bus(BUS_WT3)
out = {}

print("== WT3 (static degenerate, target) ==")
for name, traj in [
    ("extfix baseline (imuW=1.0)", RES + "WT3_dual_extfix.tum"),
    ("degeneracyAwareFeatures", RES + "WT3_dual_degaware.tum"),
]:
    try:
        out["WT3|" + name] = analyze(name, traj, bus_wt3)
    except Exception as e:
        print(f"{name:34s} ERROR: {e}")

print("\n== AT1 (dynamic scene, no-regression) ==")
for name, traj in [
    ("imuW=0.1 baseline", RES + "AT1_dual_imuw010.tum"),
    ("degeneracyAwareFeatures", RES + "AT1_dual_degaware.tum"),
]:
    try:
        out["AT1|" + name] = analyze(name, traj, bus_at1)
    except Exception as e:
        print(f"{name:34s} ERROR: {e}")

with open(DAT + "degaware_metrics.json", "w") as f:
    json.dump(out, f, indent=2, ensure_ascii=False)
print("\nSaved", DAT + "degaware_metrics.json")
