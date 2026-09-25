import sys
import numpy as np
from evo.tools import file_interface

GT = "/home/clh/视频/HK_MEMS_Dataset/grt/gps_path/hh2_gps_path_grt_clean.txt"

def umeyama3d(P, Q):
    mp, mq = P.mean(0), Q.mean(0)
    Pc, Qc = P - mp, Q - mq
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    t = mq - R @ mp
    return R, t

def ape(traj_path, max_diff=0.2):
    gt = file_interface.read_tum_trajectory_file(GT)
    tr = file_interface.read_tum_trajectory_file(traj_path)
    # associate by timestamp (nearest within max_diff)
    gtt = gt.timestamps; trt = tr.timestamps
    gi = np.searchsorted(gtt, trt)
    gi = np.clip(gi, 0, len(gtt)-1)
    # keep pairs within max_diff
    mask = np.abs(gtt[gi] - trt) <= max_diff
    P = tr.positions_xyz[mask]
    Q = gt.positions_xyz[gi[mask]]
    if len(P) < 10:
        return None, None, 0
    R, t = umeyama3d(P, Q)
    Pa = (R @ P.T).T + t
    err = np.linalg.norm(Pa - Q, axis=1)
    rmse = float(np.sqrt(np.mean(err**2)))
    mean = float(err.mean())
    std = float(err.std())
    return rmse, mean, len(P)

if __name__ == "__main__":
    for name, f in [
        ("HH2 CONSTRAINT-FUSION", "/home/clh/桌面/data/HKMEMS-results/HH2_dual_fusion.tum"),
        ("HH2 POINT-CLOUD-MERGE", "/home/clh/桌面/data/HKMEMS-results/HH2_dual_merge.tum"),
    ]:
        try:
            rmse, mean, n = ape(f)
            if rmse is None:
                print(f"{name:24s} NO/too-few matched poses")
            else:
                print(f"{name:24s} APE rmse={rmse:.3f} m  mean={mean:.3f} m  matched={n}")
        except Exception as e:
            print(f"{name:24s} ERROR: {e}")
