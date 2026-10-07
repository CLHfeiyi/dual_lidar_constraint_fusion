# lio_sam_paper — Dual-LiDAR Constraint-Level Fusion LIO

A tightly-coupled LiDAR-inertial odometry and mapping (LIO) system for **degenerate
environments** (e.g. long tunnels), extending [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
with **dual-LiDAR constraint-level fusion**.

The front sensor is a 32-channel Ouster OS1-32 and the back sensor is a sparse
RoboSense M1 (5-channel MEMS). Instead of merging the two point clouds, this work fuses
their constraints at the **information-matrix (Hessian) level**:

```
H = Hf + HbW ,   b = bf + bbW
```

where `Hf`/`Hb` are the per-lidar Hessians and `W` down-weights the back lidar's noisy
rotational DOFs (`backRotWeight`). In degenerate tunnels a single lidar diverges (the
along-track direction is unobservable); the second lidar supplies the missing constraint.

## Key contributions

- **Constraint-level fusion** — per-lidar Hessian accumulation + fusion (`H = Hf + HbW`),
  shown to beat point-cloud merging on degenerate sequences.
- **Degeneracy-aware directional weighting** — Schur-complement decoupling of rotation /
  translation, eigen-clamping (`adjustLidarHessian`).
- **Consistency suppression** — suppresses redundant weak-direction constraints between the
  two lidars (`consistencySuppression`).
- **Anti-weaving yaw prior** — adaptive yaw prior that damps lateral weaving in tunnels.
- **IEKF prediction + VGICP loop closure** — iterative Kalman filter for IMU preintegration
  and VGICP-based loop detection.

## Architecture

| File | Role |
|---|---|
| `src/paper_slam.cpp` | Main `PaperSLAM` class (feature extraction, scan-to-map, fusion) |
| `utility.h` | `ParamServer` (all parameters, per-sensor extrinsics) |
| `iekf_paper.h` | `IEKFPaper` — iterative Kalman filter prediction |
| `vgicp_loop.h` | `VGICPLoopDetector` — loop closure |
| `bfs_cluster.h` | BFS clustering for feature extraction |
| `src/dual_lidar_merger.cpp` | Optional point-cloud merging node (ablation baseline) |

Pipeline (callback-driven, `ros::spin`): IMU → IEKF prediction → per-lidar feature
extraction → `scan2MapOptimizationDual` (10 LM iterations of
corner/surface/direct-plane residual accumulation, Hessian fusion, degeneracy adjustment)
→ GTSAM factor-graph back-end → VGICP loop closure.

## Dependencies

- ROS Noetic (tested on Ubuntu 20.04)
- PCL, OpenCV
- GTSAM (Georgia Tech Smoothing and Mapping)
- Boost (timer)
- OpenMP

## Build

```bash
cd ~/catkin_ws
catkin_make -j4
source devel/setup.bash
```

## Run

```bash
# Dual-lidar constraint-level fusion (HK-MEMS dual-lidar config)
roslaunch lio_sam run_hkmems_dual.launch

# Or play a bag with /ouster/points (front), /rslidar_points (back), /imu/data
rosbag play --clock your_bag.bag
```

Trajectory is saved to a TUM file via `scripts/trajectory_saver.py`
(`traj_output` launch arg, default `/tmp/hkmems_dual_traj.tum`).

## Configuration

All parameters live in `config/params_*.yaml` under the `lio_sam` namespace.
Key dual-lidar parameters:

| Parameter | Meaning |
|---|---|
| `useDualLidar` | Enable dual-lidar mode |
| `pointCloudTopicFront` / `pointCloudTopicBack` | Front / back lidar topics |
| `mergeClouds` | `true` = point-cloud merge (baseline), `false` = constraint-level fusion |
| `extrinsicTransBack` / `extrinsicRotBack` | Offline back-lidar extrinsics `T_back_body` |
| `imuWeight` | IMU prior weight for yaw |
| `backRotWeight` | Down-weighting of back-lidar rotational DOFs |
| `degeneracyEnable` / `degeneracyEpsilon` | Degeneracy detection & directional weighting |

Experimental online-adaptive switches (`adaptiveImuWeight`, `dynamicOutlierRemoval`,
`degeneracyAwareFeatures`, `onlineExtrinsicCalib`) default to `false`. They were evaluated
as ablations and found not to improve the constraint-level fusion baseline; see
`experiments/RESULTS.md`.

## Results

Full quantitative ablations (single-lidar baselines, fusion vs. merge, IMU-weight sweep,
and the four adaptive directions) are in `experiments/RESULTS.md` with reproduction
scripts under `experiments/scripts/`.

## License & Acknowledgments

This project is derived from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
(Tixiao Shan, MIT) and is released under the BSD 3-Clause License (see `LICENSE`).
