import numpy as np
from scipy.spatial import cKDTree

GT_LEN = 3273.0  # traj_legth(26) = WT3, hardcoded in official matlab

def load_tum(path):
    d = np.loadtxt(path)
    return d[:, 1:3].astype(float)  # tx, ty

def load_bus(path):
    raw = open(path).read().strip().splitlines()
    rows = [l.strip().rstrip(',').split() for l in raw[1:] if l.strip()]
    arr = np.array([[float(a), float(b)] for a, b in rows])  # north, east
    return np.stack([arr[:, 1], arr[:, 0]], axis=1)          # [east, north]

def preprocess(xy):
    # official: remove NaN, then greedy min-2m downsampling
    xy = xy[~np.isnan(xy).any(axis=1)]
    keep = [0]
    for j in range(1, len(xy)):
        if np.linalg.norm(xy[j] - xy[keep[-1]]) >= 2.0:
            keep.append(j)
    return xy[keep]

def downsample_equal(a, b):
    na, nb = len(a), len(b)
    if na > nb:
        idx = np.ceil(np.arange(1, na + 1, na / nb)).astype(int) - 1
        return a[idx], b
    if nb > na:
        idx = np.ceil(np.arange(1, nb + 1, nb / na)).astype(int) - 1
        return a, b[idx]
    return a, b

def rot_svd(source, target):
    source, target = downsample_equal(source, target)
    H = source.T @ target
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1; R = Vt.T @ U.T
    return R

def nn_rmse(source, target):
    d, _ = cKDTree(target).query(source)
    return float(np.sqrt(np.mean(d ** 2)))

def official_eval(name, traj, gt):
    slam = preprocess(traj)
    grt = gt - gt[0]                       # official: grt - grt(1,:)
    R = rot_svd(slam, grt)                 # move_slam_path=1: align slam -> grt
    aligned = (R @ slam.T).T
    for _ in range(10):                    # ICP, rotation-only (t forced 0)
        tree = cKDTree(grt)
        _, idx = tree.query(aligned)
        match = grt[idx]
        Ri = rot_svd(aligned, match)
        aligned = (Ri @ aligned.T).T
    rmse = (nn_rmse(aligned, grt) + nn_rmse(grt, aligned)) / 2.0
    err_pct = rmse / GT_LEN * 100.0 * 2.0  # official: rmse/traj_legth*100*2
    err_pct = min(100.0, max(0.0, err_pct))
    L = float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))
    rel_len = 100.0 * abs(L - GT_LEN) / GT_LEN
    print(f"{name:14s} 2D_RMSE={rmse:8.2f} m   official_err={err_pct:6.1f}%   path_len={L:9.0f} m   rel_len={rel_len:6.1f}%")
    return rmse, err_pct, L

bus = load_bus("/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt")
print(f"GT points={len(bus)}, GT length(announced)=3273m\n")

official_eval("FAST-LIO2", load_tum("/home/clh/视频/HK_MEMS_Dataset/evaluation_result/FAST-LIO2/all/fastlio_wt3_path_grt.txt"), bus)
official_eval("LIO-SAM",   load_tum("/home/clh/视频/HK_MEMS_Dataset/evaluation_result/LIO-SAM/all/liosam_wt3_path_grt.txt"), bus)

import os
for name, f in [("ours extfix", "/home/clh/桌面/data/HKMEMS-results/WT3_dual_extfix.tum")]:
    if os.path.exists(f):
        official_eval(name, load_tum(f), bus)
    else:
        print(f"{name}: MISSING {f}")
