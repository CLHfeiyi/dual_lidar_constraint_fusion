import sys
import numpy as np
from evo.tools import file_interface

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"
TRAJ = sys.argv[1] if len(sys.argv) > 1 else "/home/clh/桌面/data/HKMEMS-results/WT3_dual_dcreg.tum"

def load_bus(path):
    raw = open(path).read().strip().splitlines()
    pts = []
    for line in raw[1:]:
        parts = [x for x in line.replace(",", " ").split() if x]
        if len(parts) >= 2:
            pts.append([float(parts[0]), float(parts[1])])
    return np.array(pts)

def umeyama_2d(P, Q):
    mp, mq = P.mean(0), Q.mean(0)
    Pc, Qc = P - mp, Q - mq
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mq - R @ mp
    return R, t

def icp(P, Q, iters=60):
    P = P.copy()
    for _ in range(iters):
        d = np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=2)
        Qc = Q[d.argmin(1)]
        R, t = umeyama_2d(P, Qc)
        P = (R @ P.T).T + t
    d = np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=2)
    return float(d.min(1).mean())

def path_length(xyz):
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()) if len(xyz) > 1 else 0.0

bus = load_bus(BUS)
Q = bus[:, [1, 0]]  # (east,north)->(x,y)
t = file_interface.read_tum_trajectory_file(TRAJ)
P = t.positions_xyz[:, :2]
plen = path_length(t.positions_xyz)
resid = icp(P, Q)
print(f"poses={len(P)} path_len={plen:.2f} m  icp_resid={resid:.3f} m")
