import numpy as np
from evo.tools import file_interface
from scipy.spatial import cKDTree

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"

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
    Pc, Qc = P - mp, Q - mq
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    return R, mq - R @ mp

def icp(P, Q, it=60):
    P = P.copy()
    tree = cKDTree(Q)
    for _ in range(it):
        _, idx = tree.query(P)
        Qc = Q[idx]
        R, t = umeyama(P, Qc)
        P = (R @ P.T).T + t
    return P

bus = load_bus(BUS)
Q = bus[:, [1, 0]]
d = np.linalg.norm(np.diff(Q, axis=0), axis=1)
cum = np.concatenate([[0], np.cumsum(d)])

def analyze(name, traj):
    t = file_interface.read_tum_trajectory_file(traj)
    P = t.positions_xyz[:, :2]
    xyz = t.positions_xyz
    slen = plen(xyz)
    Pa = icp(P, Q)
    tree = cKDTree(Q)
    dists, idx = tree.query(Pa)
    resid = float(dists.mean())
    cov_min, cov_max = idx.min(), idx.max()
    bus_covered = cum[cov_max] - cum[cov_min]
    infl = slen / max(bus_covered, 1.0)
    dp = np.diff(Pa, axis=0)
    L = np.linalg.norm(dp, axis=1)
    keep = L > 1e-6
    dirs = dp[keep] / L[keep, None]
    ang = np.arccos(np.clip((dirs[:-1] * dirs[1:]).sum(1), -1, 1)).sum()
    turn = np.degrees(ang)
    diff = np.diff(idx)
    fwd = (diff > 0).mean() * 100
    back = (diff < 0).mean() * 100
    print(f"=== {name} ===")
    print(f"  poses={len(P)}  SLAM path={slen:.1f} m  ICP resid={resid:.3f} m")
    print(f"  bus idx coverage: {cov_min}->{cov_max} = {bus_covered:.1f} m along-track")
    print(f"  scale inflation = {infl:.2f}x   turn={turn:.0f} deg  (fwd {fwd:.0f}% / back {back:.0f}%)")

for name, traj in [
    ("FAST-LIO2 (official)", "/home/clh/视频/HK_MEMS_Dataset/evaluation_result/FAST-LIO2/all/fastlio_wt3_path_grt.txt"),
    ("LIO-SAM (official)",   "/home/clh/视频/HK_MEMS_Dataset/evaluation_result/LIO-SAM/all/liosam_wt3_path_grt.txt"),
    ("ours extfix (eps=1e-8)", "/home/clh/桌面/data/HKMEMS-results/WT3_dual_extfix.tum"),
]:
    try:
        analyze(name, traj)
    except Exception as e:
        print(f"=== {name} === ERROR: {e}")
