#!/usr/bin/env python3
"""单雷达 baseline vs 双雷达融合（WT3，对齐巴士 GT）+ JSON dump。"""
import json
import numpy as np
from evo.tools import file_interface
from scipy.spatial import cKDTree

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"
RES = "/home/clh/桌面/data/HKMEMS-results/"
DAT = "/home/clh/文章算法/experiments/data/"

def load_bus(path):
    raw = open(path).read().strip().splitlines()
    pts = []
    for line in raw[1:]:
        parts = [x for x in line.replace(",", " ").split() if x]
        if len(parts) >= 2:
            pts.append([float(parts[0]), float(parts[1])])
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

def analyze(name, traj):
    t = file_interface.read_tum_trajectory_file(traj)
    xyz = t.positions_xyz
    P = xyz[:, :2]
    slen = plen(xyz)
    Pa = icp(P, Q)
    dists, idx = cKDTree(Q).query(Pa)
    resid = float(dists.mean())
    cov = cum[idx.max()] - cum[idx.min()]
    infl = slen / max(cov, 1.0)
    dp = np.diff(Pa, axis=0); L = np.linalg.norm(dp, axis=1)
    keep = L > 1e-6; dirs = dp[keep] / L[keep, None]
    turn = np.degrees(np.arccos(np.clip((dirs[:-1]*dirs[1:]).sum(1), -1, 1)).sum())
    d = np.diff(idx); fwd = (d > 0).mean()*100; back = (d < 0).mean()*100
    m = dict(resid=resid, slen=slen, cov=cov, infl=infl, turn=turn, fwd=fwd, back=back)
    print(f"{name:26s} resid={resid:9.3f}m  path={slen:9.1f}m  cover={cov:7.1f}m  "
          f"infl={infl:5.2f}x  turn={turn:7.0f}deg  fwd={fwd:3.0f}%/back={back:3.0f}%")
    return m

RUNS = [
    ("M1-only (5ch MEMS)",      RES + "WT3_single_m1.tum"),
    ("Ouster-only (32ch)",      RES + "WT3_single_ouster.tum"),
    ("DUAL fusion (Ouster+M1)", RES + "WT3_dual_extfix.tum"),
    ("DUAL merge (Ouster+M1)",  RES + "WT3_dual_merge.tum"),
]
out = {}
for name, traj in RUNS:
    try:
        out[name] = analyze(name, traj)
    except Exception as e:
        print(f"{name:26s} ERROR: {e}")

with open(DAT + "single_baseline_metrics.json", "w") as f:
    json.dump(out, f, indent=2, ensure_ascii=False)
print(f"\nSaved {DAT}single_baseline_metrics.json")
