import numpy as np
from evo.tools import file_interface

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"

def load_bus(path):
    raw = open(path).read().strip().splitlines()
    pts = []
    for line in raw[1:]:
        parts = [x for x in line.replace(",", " ").split() if x]
        if len(parts) >= 2:
            pts.append([float(parts[0]), float(parts[1])])
    return np.array(pts)

def path_length(xyz):
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()) if len(xyz) > 1 else 0.0

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

def icp_align(P, Q, iters=60):
    P = P.copy()
    for _ in range(iters):
        d = np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=2)
        Qc = Q[d.argmin(1)]
        R, t = umeyama_2d(P, Qc)
        P = (R @ P.T).T + t
    return P

bus = load_bus(BUS)
Q = bus[:, [1, 0]]
bus_len = path_length(bus)
print(f"BUS GT: {len(bus)} pts, 2D path_len = {bus_len:.2f} m")
print(f"BUS extent: x[{Q[:,0].min():.1f},{Q[:,0].max():.1f}] y[{Q[:,1].min():.1f},{Q[:,1].max():.1f}]")

for name, traj in [("merger_v2", "/home/clh/桌面/data/HKMEMS-results/WT3_dual_v2.tum"),
                   ("extfix", "/home/clh/桌面/data/HKMEMS-results/WT3_dual_extfix.tum"),
                   ("dcreg", "/home/clh/桌面/data/HKMEMS-results/WT3_dual_dcreg.tum")]:
    t = file_interface.read_tum_trajectory_file(traj)
    P = t.positions_xyz[:, :2]
    plen = path_length(t.positions_xyz)
    Pa = icp_align(P, Q)
    d = np.linalg.norm(Pa[:, None, :] - Q[None, :, :], axis=2).min(1)
    idx = np.linalg.norm(Pa[:, None, :] - Q[None, :, :], axis=2).argmin(1)
    print(f"\n=== {name}: {len(P)} poses, path_len={plen:.2f} m ===")
    print(f"  extent: x[{P[:,0].min():.1f},{P[:,0].max():.1f}] y[{P[:,1].min():.1f},{P[:,1].max():.1f}]")
    print(f"  nearest-bus dist: mean={d.mean():.2f} median={np.median(d):.2f} max={d.max():.2f}")
    print(f"  bus-index coverage: min={idx.min()} max={idx.max()} (bus {len(bus)} pts)")
    n = len(d)
    print("  dist by 10% chunks:", " ".join(f"{d[int(n*k/10):int(n*(k+1)/10)].mean():.1f}" for k in range(10)))
