import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.tools import file_interface

BUS = "/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt"
def load_bus(path):
    raw=open(path).read().strip().splitlines(); pts=[]
    for line in raw[1:]:
        p=[x for x in line.replace(","," ").split() if x]
        if len(p)>=2: pts.append([float(p[0]),float(p[1])])
    return np.array(pts)
def umeyama(P,Q):
    mp,mq=P.mean(0),Q.mean(0); Pc,Qc=P-mp,Q-mq; H=Pc.T@Qc
    U,S,Vt=np.linalg.svd(H); R=Vt.T@U.T
    if np.linalg.det(R)<0: Vt[-1]*=-1; R=Vt.T@U.T
    return R, mq-R@mp
def icp(P,Q,it=60):
    P=P.copy()
    for _ in range(it):
        Qc=Q[np.linalg.norm(P[:,None,:]-Q[None,:,:],axis=2).argmin(1)]
        R,t=umeyama(P,Qc); P=(R@P.T).T+t
    return P

bus=load_bus(BUS); Q=bus[:,[1,0]]
fig,ax=plt.subplots(1,2,figsize=(14,6))
# left: raw local coords
for name,traj,c in [("merger","/home/clh/桌面/data/HKMEMS-results/WT3_dual_v2.tum","b"),
                    ("extfix","/home/clh/桌面/data/HKMEMS-results/WT3_dual_extfix.tum","r"),
                    ("dcreg","/home/clh/桌面/data/HKMEMS-results/WT3_dual_dcreg.tum","g")]:
    t=file_interface.read_tum_trajectory_file(traj); P=t.positions_xyz[:,:2]
    ax[0].plot(P[:,0],P[:,1],c,label=f"{name}",lw=0.8)
ax[0].set_title("raw local coords (meters)"); ax[0].axis("equal"); ax[0].legend(); ax[0].grid(alpha=.3)
# right: ICP-aligned onto bus
ax[1].plot(Q[:,0],Q[:,1],'k-',lw=0.4,label="bus GT (3.27km)")
for name,traj,c in [("merger","/home/clh/桌面/data/HKMEMS-results/WT3_dual_v2.tum","b"),
                    ("extfix","/home/clh/桌面/data/HKMEMS-results/WT3_dual_extfix.tum","r"),
                    ("dcreg","/home/clh/桌面/data/HKMEMS-results/WT3_dual_dcreg.tum","g")]:
    t=file_interface.read_tum_trajectory_file(traj); P=t.positions_xyz[:,:2]
    Pa=icp(P,Q)
    ax[1].plot(Pa[:,0],Pa[:,1],c,label=name,lw=0.8)
ax[1].set_title("ICP-aligned onto bus GT"); ax[1].axis("equal"); ax[1].legend(); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig("/tmp/wt3_cmp.png",dpi=110)
print("saved")
