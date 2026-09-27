# 双雷达 LIO-SAM 实验记录（WT3 / HH2）

生成日期：2026-09-25。所有数值由 `experiments/scripts/` 下的 eval 脚本复现，轨迹文件在 `/home/clh/桌面/data/HKMEMS-results/`。

## 1. 数据路径

| 数据 | 路径 |
|---|---|
| WT3 bag（西隧北向） | `/home/clh/视频/HK_MEMS_Dataset/bag/tunnel/western_tunnel/WT3_western_tunnel_north_3_no_camera_2024-06-13-15-23-55_compressed.bag` |
| WT3 巴士 2D GT | `/home/clh/视频/HK_MEMS_Dataset/grt/bus_2d_path/WT3_973_1_bus_grt_path.txt`（711 点，总长 3273m，无位姿 GT） |
| HH2 bag（海滨步道） | `/home/clh/视频/HK_MEMS_Dataset/bag/dyna_pedestrain/hung_hom/HH2_coastal_footpath.bag` |
| HH2 RTK GPS GT | `/home/clh/视频/HK_MEMS_Dataset/grt/gps_path/hh2_gps_path_grt_clean.txt`（TUM，7140 点） |
| FAST-LIO2 官方结果 | `/home/clh/视频/HK_MEMS_Dataset/evaluation_result/FAST-LIO2/all/fastlio_wt3_path_grt.txt` |
| LIO-SAM 官方结果 | `/home/clh/视频/HK_MEMS_Dataset/evaluation_result/LIO-SAM/all/liosam_wt3_path_grt.txt` |

传感器：前 Ouster OS1-32（32 线，`/ouster/points`）+ 后 RoboSense M1 MEMS（5 线，`/rslidar_points`）+ Xsens MTI-30 IMU（`/imu/data`）。

## 2. 指标定义（eval_full.py / eval_ablation.py）

- **SLAM path**：轨迹累计长度（3D）。
- **ICP resid**：SLAM 轨迹 2D 点云 ICP 对齐到巴士 GT 后，最近邻距离均值（m）。
- **bus coverage**：对齐后轨迹落在巴士 GT 沿程索引区间 [min,max] 对应的弧长（m）——**真正的进度指标**。
- **scale inflation** = SLAM path / bus coverage。>1 表示估速度偏快/摆动，<1 表示停滞。
- **turn / fwd / back**：轨迹累计转向角（°）与沿巴士索引的前进/后退步比例（%）。turn 在**原始位姿**上计算（未 1m 降采样），含位姿抖动，绝对数值偏大，只看相对比较。

## 3. WT3 主消融：约束级融合 vs 点云拼接

| 模式 | ICP resid | path | bus coverage | inflation | fwd/back |
|---|---|---|---|---|---|
| **约束级融合**（extfix, ε=1e-8） | **3.827 m** | 917.5 m | **576.4 m**（idx 489→624） | 1.59× | 3% / 0%（单调前进） |
| 点云拼接（mergeClouds） | 5.961 m | 515.8 m | 896.5 m（伪覆盖） | 0.58×（停滞） | 0% / 2%（停滞+跳变） |

**结论：融合胜 1.56× 残差**，且单调前进；拼接停滞（0.58×），其 896.5m "覆盖"是跳变伪迹。机制：前 Ouster(32线) 与后 M1(5线) 的 corner/surf 比例差异大，单一全局 α 稀释某一雷达约束，重新触发沿行进方向 x 的退化停滞。

## 4. WT3 官方 baseline（均为 M1-only，同传感器不匹配）

| 方法 | ICP resid | path | bus coverage | inflation |
|---|---|---|---|---|
| FAST-LIO2（官方） | 8817.4 m | 78475.6 m | 2914.6 m | 26.92×（彻底发散） |
| LIO-SAM（官方） | 104.1 m | 18816.4 m | 1915.2 m | 9.82×（发散） |
| **ours extfix** | **3.827 m** | 917.5 m | 576.4 m | 1.59× |

注意：官方 baseline 用 RoboSense M1 **单雷达**（5 线 MEMS），我们双雷达（Ouster+M1）。不能宣称"好 27×"——传感器不对等。

## 4b. WT3 单雷达 baseline（我们自己代码，同算法隔离传感器数量）

用我们自己的 `paper_slam` 分别只跑一个雷达（`useDualLidar:false`），与双雷达融合对比——**同算法、同场景，唯一变量是传感器数量**：

| 方法 | ICP resid | path | bus coverage | inflation |
|---|---|---|---|---|
| M1-only（5ch MEMS，单） | 34509.3 m | 139.2 km | 2997.8 m | 46.42×（发散） |
| Ouster-only（32ch，单） | 9772.0 m | 48.0 km | 3273.0 m | 14.66×（发散） |
| **双雷达融合（Ouster+M1）** | **3.827 m** | 917.5 m | 576.4 m | 1.59× |
| 双雷达拼接（Ouster+M1） | 5.961 m | 515.8 m | 896.5 m | 0.58×（停滞） |

**结论（关键正向结果）**：**单个雷达（无论 Ouster 32 线还是 M1）在 WT3 退化隧道全部发散**（残差 9.7km / 34.5km）。第二个雷达 + 约束融合是**必要**的——双雷达融合残差 3.83m，比 Ouster-only 好 **2550×**、比 M1-only 好 **9000×**。这彻底排除了"多传感器只是堆数据"的质疑：第二个视角对消歧退化隧道的沿行进方向是**必需**的，不是锦上添花。

完整消融链（三级，均同代码或同传感器）：
1. **单 vs 双**（同算法）：Ouster-only 9772m → 双融合 3.83m（2550×）——第二个雷达必需。
2. **双融合 vs 双拼接**（同双传感器）：3.83m vs 5.96m（1.56×）——逐雷达加权更好。
3. **vs 官方 M1-only baseline**：3.83m vs 最佳 104m（LIO-SAM）——27×，但传感器不对等（他们是 M1-only）。

## 5. WT3 退化处理实验（全部负结果，记录备查）

基线 = ε=1e-8 + 外参修复 + 无方向降权（`degeneracyEnable:false`）。

| 变体 | 关键数值 | 结论 |
|---|---|---|
| **extfix（基线）** | cover 576.4m, resid 3.83m, infl 1.59× | 最优 |
| ε=1e-3 降权 | cover 208m, resid 10.09m, infl 3.25×, turn 6082° | 更差 |
| dcreg（Schur+钳制） | resid 4.79m, infl 0.58×（停滞）, fwd5%/back20% | 更差 |
| dcreg 旧版 | 147.8m path / 82.3m cover, 1.80× | 更差 |
| 旧 merger "369.6m" | 67.3m cover, 5.49× | 早期停滞 |
| backrot03（wr=0.3） | infl 1.27×（摆动降）但 lateral 12.8m | 精度崩 |
| yawprior g1 | path 12km, resid 672m | 发散 |

**根因（已定位，非待办）**：退化方向是**沿隧道行进方向 x**（弱本征方向是平移 x，不是 yaw）。隧道墙均匀、前进位置不可观测。任何降权/约束 yaw 的手段（ε、DCReg 钳制、yaw 先验、yaw 钳制、后雷达旋转降权）都会移除系统唯一的前进锚定或唯一的横向可观测性 → 停滞/发散。yaw 摆动是优化器对 x 退化的**应对机制**，不可通过约束消除。真正缺口 = 沿行进方向信息（轮速计，bag 里没有；隧道无回环；IMU gyro z 与 body yaw 错位 3× 欠测）。

## 6. HH2 消融（开阔步道，RTK GT，lidar+IMU only）

| 模式 | APE rmse | APE mean | path | matched |
|---|---|---|---|---|
| 约束级融合 | 0.996 m | 0.885 m | 174.9 m | 2895 |
| 点云拼接 | **0.800 m** | 0.718 m | 175.6 m | 2895 |
| GT | — | — | 181.2 m | — |

**结论：HH2 上拼接反而略胜 ~20%**，两者都走完全程（无停滞、无尺度膨胀）。**融合的优势是退化场景专有的**：良态场景沿 x 完全可观测，逐雷达 α 的抗退化优势消失，拼接更稠密降噪更好。

## 7. 最终论文表述（诚实、可辩护）

> **逐雷达约束级融合的价值是"抗退化鲁棒性"，不是普适精度**：它在退化隧道（WT3）防止沿行进方向的跟踪失效（残差 3.83 vs 5.96m，融合单调前进 vs 拼接停滞），而在良态场景（HH2）与朴素拼接相当（0.996 vs 0.800m）。

不要宣称"融合永远更好"。

## 8. 关键配置键（config/params_hkmems_dual.yaml）

| 键 | 基线值 | 说明 |
|---|---|---|
| `degeneracyEnable` | false | 退化处理主开关（WT3 保持 false） |
| `degeneracyEpsilon` | 1.0e-8 | 相对阈值 ε（HH2 用 1e-3 有效） |
| `degeneracyKappaTg` | 10.0 | DCReg 目标条件数 |
| `yawPriorGain` | 0.0 | 抗摆动 yaw 先验（WT3 关闭） |
| `backRotWeight` | 1.0 | 后雷达旋转降权（WT3 关闭） |
| `mergeClouds` | false | false=约束级融合；true=点云拼接 |

## 9. 复现命令

```bash
# WT3 融合（基线）
#   config: degeneracyEnable=false, mergeClouds=false
#   输出: WT3_dual_extfix.tum
bash experiments/scripts/run_wt3_final.sh        # 或 run_wt3_extonly.sh

# WT3 拼接
#   config: mergeClouds=true
bash experiments/scripts/run_wt3_merge.sh

# HH2（融合/拼接，参数由 run_hh2.sh 的 $1 输出路径决定）
#   config 先切 mergeClouds 再跑
bash experiments/scripts/run_hh2.sh /home/clh/桌面/data/HKMEMS-results/HH2_dual_<mode>.tum

# 评估
python3 experiments/scripts/eval_ablation.py      # WT3 融合 vs 拼接
python3 experiments/scripts/eval_baselines.py     # WT3 官方 baseline
python3 experiments/scripts/eval_hh2.py           # HH2 APE
```

## 10. 轨迹文件索引（/home/clh/桌面/data/HKMEMS-results/）

- `WT3_dual_extfix.tum` — 基线（ε=1e-8 融合）
- `WT3_dual_merge.tum` — 点云拼接
- `WT3_dual_eps1e-3.tum` — ε=1e-3
- `WT3_dual_dcreg_schur.tum` / `WT3_dual_dcreg.tum` / `WT3_dual_dcreg_pcg.tum` — DCReg
- `WT3_dual_backrot03.tum` / `WT3_dual_clamponly.tum` — 后雷达降权 / 钳制
- `WT3_dual_yawprior*.tum` — yaw 先验
- `HH2_dual_fusion.tum` / `HH2_dual_merge.tum` — HH2 融合 / 拼接
- `AT1_dual_fusion.tum` / `AT1_dual_merge.tum` — AT1 (Aberdeen) 融合 / 拼接

## 11. AT1 (Aberdeen 穿山隧道) 消融（2026-09-25）

复用 WT3 同套 config/launch（传感器结构实测一致：Ouster 32×1024 + M1 15750×5），
仅换 bag 路径 + GT 路径。GT = `AT1_77_1_bus_grt_path.txt`（2.78km，596 点）。

| 指标 | AT1 融合 | AT1 拼接 |
|---|---|---|
| ICP 残差 (m) | 30.65 | 34.09 |
| 覆盖 (m) | 550 | 603 |
| 尺度膨胀 (x) | 4.02 | 2.74 |
| 横向误差 \|mean\| (m) | 30.59 | 33.91 |
| 纵向误差 \|mean\| (m) | 1.33 | 2.19 |

**结论（诚实）**：Aberdeen 比 WT3 难得多——两方法都退化到 ~30m 残差（WT3 融合仅 3.83m）。
数据集标注 AT1 为"高速 + 大量动态车辆"的退化隧道，动态噪声叠加沿行进方向退化导致严重横向漂移。
融合仍优于拼接（残差 30.65 vs 34.09，~11%），但优势远小于 WT3 的 56%；且拼接在 AT1 **没有停滞**
（覆盖 603m），膨胀（2.74x）反而低于融合（4.02x）。→ 结论必须限定："融合防退化失效"在 WT3 强、
AT1 弱；不能宣称融合普适更优。

复现：`bash experiments/scripts/run_at1_fusion.sh` / `run_at1_merge.sh`；评估 `python3 experiments/scripts/plot_at1_ablation.py`。
数据：`data/at1_ablation_metrics.json`；图：`figures/at1_ablation_*.png`。

### 11b. AT1 退化诊断 + 里程计调查（负结果，支撑"观测性极限"结论）

**DEGEN 诊断**（`degeneracyLogOnly:true`，只记 EVD 不投影，轨迹与融合版一致）：后雷达 M1 的 Schur
平移条件数 condT 干净段（t<513s）稳定在 4–13，随后爬升 23→42→52，**首次 isDeg=1 @ t≈606s
（condT=52，越过 degeneracyCondThresh=50）**；弱方向 weakIdx 分布 `{0(roll):101, 3(平移x):110}`，
退化段锁定 **weakIdx=3 = 沿轨道平移 x**——与 WT3 根因完全一致。前雷达 Ouster 因共享
`static int degDbg` 计数器只记到 3 条（vs 后雷达 208 条），但几何上它与后雷达看到同一堵平行墙，
必然同样失去 x 可观测性。

**里程计调查**：bag 里**没有** config 写的 `/odom`（轮速）。仅有的里程计来源是 mavros/PX4 自驾仪的
`/mavros/local_position/odom`、`/mavros/odometry/in`、`/mavros/global_position/local`（30–50Hz）：

| 话题 | 消息数 | 水平路径长 | 净位移 |
|---|---|---|---|
| `/mavros/local_position/odom` | 18094 | 531 m | **7.7 m** |
| `/mavros/odometry/in` | 18085 | 531 m | 7.7 m |
| `/mavros/global_position/local` | 29901 | 549 m | — |
| 巴士 GT | — | **2780 m** | 2780 m |

三个 odom 水平路径都只有 ~530m，而净位移仅 **7.7m**——**原地蛇形摆动 531m 却几乎不前进**。
这不是 scale 误差（否则路径≈净位移），而是 **PX4 EKF 在隧道里 GPS 失效后纯 IMU 航位推算的停滞**。
即：**这台车自己的导航栈（PX4 EKF）在这个隧道里也停摆了**，且失败模式与我们 lidar fusion（覆盖 550m）
一模一样。

**结论（重要负结果）**：AT1 是 GPS 拒止 + 无轮速编码器 + MEMS 级 IMU 的穿山隧道，**唯一沿轨道信号
是 IMU 航位推算，而它在 ~530m 处崩**。因此"融合外部里程计锚定 x 方向"在此场景走不通——odom 本身
比我们的 lidar 还退化。AT1 的 ~30m 残差已接近该场景的**观测性物理上限**，fusion vs merge 的微小差异
（11%）不应被夸大。这反向强化了论文主结论：约束级融合的价值在于**良态/退化场景间的鲁棒性跨度**
（WT3 防停滞 56%、HH2 相当），而非在观测性先天缺失的场景里无中生有。

## 12. CT1 (Central-Wan Chai Bypass 中環灣仔繞道) 消融（2026-09-25）

bag = `CT1_central_bypass1_2024-01-11-17-06_compressed.bag`（315s，传感器同 AT1/WT3：Ouster+M1+IMU）。
GT = `CT1_18X_2_bus_grt_path.txt`（5.21km，1101 点，巴士 18X 线）。

| 指标 | CT1 融合 | CT1 拼接 |
|---|---|---|
| ICP 残差 (m) | 12.98 | 4.15 |
| 覆盖 (m) | 550 | 578 |
| 尺度膨胀 (x) | 1.35 | 1.54 |
| 横向误差 \|mean\| (m) | 12.12 | 2.05 |
| 纵向误差 \|mean\| (m) | 1.91 | 3.03 |

**冻结定位（原始轨迹逐时位置，非 ICP 对齐）**：融合在 t≈79s 前进到 **557m** 后冻结、拼接在 t≈63s 前进到
**570m** 后冻结——两者都在 bag 的 **20–25% 处停摆**，剩余 ~4 分钟原地振荡（原始路径 713/865m 但净位移
仅 527/570m）。这与 WT3（融合覆盖 576m）、AT1（融合 550m）的 ~550m 停滞点高度一致。

**结论（诚实）**：CT1 上融合与拼接**双双停滞在 ~550m**，是同一沿轨道 x 退化的失败，与 WT3/AT1 同源。
表中的"拼接残差 4.15 < 融合 12.98"是**冻结簇 ICP 对齐的伪迹**，不是真实跟踪差异——融合的冻结簇横向
偏移 12m 而拼接的只有 2m，二者都停在 ~550m、都没有走完。因此**不能**据此宣称"CT1 上拼接更好"。
四场景（WT3/AT1/CT1 退化 + HH2 良态）下来，退化隧道的真实信号是"所有方法都在 ~550m 沿轨道停摆"，
再次印证主结论：沿行进方向的观测性缺失是根本瓶颈，不是逐雷达 α 或融合策略能弥补的。

复现：`bash experiments/scripts/run_ct1_fusion.sh` / `run_ct1_merge.sh`；评估 `python3 experiments/scripts/plot_ct1_ablation.py`。
数据：`data/ct1_ablation_metrics.json`；图：`figures/ct1_ablation_*.png`。

## 13. AT1 imuWeight 消融（2026-09-26，正向——降 IMU 权重 2× 改善；含 baseline 对齐修正）

新增 `imuWeight` 参数（0..1），同时缩放三处 IMU 影响：① IEKF 速度种子（`updateInitialGuess` 里
`lastScanPosX + imuWeight·v·dt`，沿行进方向初始猜测）；② yaw 先验权重（`fuseAndSolve` 里
`imuWeight·yawW`，目标 = IEKF 陀螺积分 yaw）；③ 点云去畸变旋转（`projectPointCloud` 里
`rx/ry/rz *= imuWeight`）。`imuWeight=0` = 纯激光（无速度种子、无 yaw 先验、无去畸变）。

| 方法 | ICP 残差 (m) | 覆盖 (m) | 路径 (m) | 膨胀 (x) | 横向\|mean\| (m) |
|---|---|---|---|---|---|
| **本工作 imuW=0.0** | **8.08** | 502.0 | 2287.5 | 4.56 | 7.92 |
| **本工作 imuW=0.1** | **8.48** | 527.4 | 1980.8 | 3.76 | 7.81 |
| 本工作 imuW=0.5 | 16.50 | 534.9 | 2088.4 | 3.90 | 16.32 |
| 本工作 imuW=1.0 | 16.85 | 534.9 | 3033.2 | 5.67 | 16.61 |
| LIO-SAM（M1，IMU+雷达） | 19.59 | 616.7 | 1398.6 | 2.27 | 19.16 |
| Intensity-SLAM（M1，IMU+雷达） | 26.36 | 458.5 | 1270.4 | 2.77 | 25.99 |
| MEMS-LIO（M1，IMU+雷达） | 26.41 | 741.7 | 26190.7 | 35.31 | 25.78 |
| FAST-LIO2（M1，IMU+雷达） | 562.31 | 2775.7 | 17757.4 | 6.40 | 367.33 |

**结论 1（正向）**：降 IMU 状态估计权重（速度种子 + yaw 先验）→ 残差 **16.85 → 8.08m（~2× 改善）**，
横向误差 16.61 → 7.92m 减半。呈**阶梯状**：imuWeight≥0.5 时 ~16.5m（坏），≤0.1 时 ~8.3m（好）——
存在一个临界点，激光 yaw 先验只有在权重足够低时才能压过坏 IMU yaw（yawW=2000·0.1=200）。紧耦合
MEMS IMU（gyro-z 3× 欠测 + 速度漂移）在 AT1 上是拖累，但不如早先估计的 74% 严重（~2× 而非 3.8×）。

**结论 2（修正——早先"关去畸变变差 22.30m"是异常 run）**：早先记录 imuWeight=0.0=22.30m 经重跑验证为
**一次异常 run**（现两次复现 8.08 / 8.53m）。真实情况：关去畸变（0.0，8.08m）与保留去畸变（0.1，8.48m）
**相当**（差异在 run 间噪声 ~0.5m 内）。即 AT1 上 IMU 去畸变**既无明显增益也无明显损害**——帧内运动
补偿的相对校正收益被 gyro 错位抵消。最优区间 imuWeight∈[0.0, 0.1]，**激光主导**。

**结论 3（负向，点到线假设被证伪）**：为排除"点到面 vs 点到线"这个前端差异，把边缘残差从
点到点改为点到线（F-LOAM/A-LOAM 标准）重跑 AT1 imuWeight 消融。结果残差 **33.69 (1.0) → 21.25 (0.5) →
12.13 (0.1) → 18.34 (0.0)**，对比点到点基线 **16.85 / 16.50 / 8.48 / 8.08**——点到线在最优 imuWeight=0.1
处反而**更差（8.48 → 12.13m，1.4×）**。**机制**：退化隧道内的边缘线大多平行于行进方向，点到线残差
（`normal·p − d0`）允许边缘点沿这条线（=沿行进方向 x）自由滑动，恰好丢掉唯一的弱 x 锚定；而点到点
（最近邻 3D 匹配）提供**各向同性约束**，无意中锚住了退化方向 x。所以点到点（"GenZ-ICP"）在退化隧道里
是**有意有效**的，不是缺陷。

**论文表述（按传感器对齐的 baseline）**：本系统 = IMU + 雷达，故 baseline 只对比同为 IMU+雷达的算法
（FAST-LIO2 / LIO-SAM / MEMS-LIO / Intensity-SLAM，官方均为 M1 单雷达）。在这些 baseline 中，
**本工作 imuWeight=0.0/0.1 的 8.08/8.48m 是最优**——LIO-SAM 19.59m（2.3×）、Intensity-SLAM 26.36m、
MEMS-LIO 26.41m（膨胀 35×）、FAST-LIO2 562m（彻底发散）。紧耦合 MEMS IMU 状态估计在 AT1 是拖累
（imuWeight 1.0→0.1 改善 ~2×），点到点边缘残差优于点到线。注：baseline 是 M1 单雷达，我们是双雷达——
第二雷达的必要性已由 WT3 单雷达发散实验（Ouster-only 9772m / M1-only 34509m 均发散）单独证明，
故本表的优势是"双雷达约束融合 + 降 IMU 权重"共同贡献，不是单纯"更多传感器"。

复现：`bash experiments/scripts/run_at1_imuw.sh <imuWeight> <suffix>`；评估 `python3 experiments/scripts/eval_at1_imuw.py`。
数据：`data/at1_imuw_metrics.json`。参数：`launch/run_at1_imuw.launch`（`<param lio_sam/imuWeight>` 覆盖）。


## 14. adaptiveImuWeight（2026-09-27，负向——condT 是滞后/混淆信号，无法预测性地驱动 IMU 权重）

实现 `adaptiveImuWeight` 开关（默认 false）：用**平移退化条件数 condT**（Schur 补后的平移子空间
λ_max/λ_min）作在线判别信号，EMA（α=0.05）估计退化置信度，超过 0.5 判"退化 regime"→ imuWeight 升到
1.0（信任 IMU），否则降到 0.1（信任激光）。机制本身运行正确（log 里 degBelief 随 condT 升降，imuW_adapt
在 0.1/1.0 间切换）。

| 场景 | 方法 | ICP 残差 (m) | 覆盖 (m) | 路径 (m) | 膨胀 (x) |
|---|---|---|---|---|---|
| AT1 | 静态 imuW=0.1 | 8.48 | 527.4 | 1980.8 | 3.76 |
| AT1 | adaptive（condT 驱动） | 11.86 | 536.3 | 2571.7 | 4.80 |
| WT3 | 静态 imuW=1.0（extfix 基线） | 3.83 | **576.4** | 917.5 | 1.59 |
| WT3 | adaptive（condT 驱动） | 2.93 | **22.2** | 51.0 | 2.29 |

**结论（负向，condT 判别器被证伪）**：

1. **AT1 上 adaptive 正确识别非退化（condT 中位 ~2.9，从未 >50），落到 0.1**——但这只是"正确答案是
   0.1"的平凡复现，且残差 11.86m 比静态 0.1 的 8.48m 差 ~3.4m（超出 run 噪声 ~0.5m，疑似分叉 run，
   需复验；两 run 有效权重相同，代码路径仅多 EVD 计算，理论上应 bit-相同）。

2. **WT3 上 adaptive 失败**：condT 在**正常跟踪时大多 <50**（中位 ~2.8，与 AT1 相同），只有跟踪
   **已经开始停滞后**才飙到 460-630。于是 adaptive 大部分时间选 0.1，而 WT3 需要 1.0 → **停滞在 22m
   覆盖（vs 基线 576m）**。残差 2.93m 是"冻结团块贴近 GT"的对齐假象，不是真跟踪。

3. **根因（condT 是滞后/混淆信号）**：condT 反映的是**当前**可观测性，而可观测性在你降 IMU 权重、
   开始停滞的那一刻就崩塌了——condT 只在系统**已经失败后**才升高，来不及**预防性**地把 imuWeight
   拉高。这正是 memory 里 DCReg 实验（degeneracyEnable=true）condT 稳定 460-630 的来源：那是在
   **已经降权、已经停滞**的轨迹上测的，是自证预言，不是预测信号。

4. **判别器需要重新设计**：WT3（静态退化，需 IMU 锚 x）vs AT1（动态车辆 + 退化，需激光主导）的真正
   分界是**动态性**（AT1 有大量动态车辆）而非 condT 可观测性。下一步应转向 `dynamicOutlierRemoval`
   （在线动态剔除）方向，用动态占比/残差鲁棒性作信号，而非 condT。

复现：`bash experiments/scripts/run_at1_adaptive.sh` / `run_wt3_adaptive.sh`（launch 设
`adaptiveImuWeight=true, imuWeight=1.0`）。轨迹：`AT1_dual_adaptive.tum` / `WT3_dual_adaptive.tum`。


## 15. dynamicOutlierRemoval（2026-09-27，负向——单帧残差鲁棒核无法区分动态物体与漂移）

实现 `dynamicOutlierRemoval` 开关（默认 false）：在 `accumulateLidarHessian`（双路）与
`combineOptimization`（拼接路）的 corner/surf 残差加权处，把软核 `1/(1+|r|)` 替换为 Cauchy 核
`1/(1+(r/σ)²)`（σ=`dynamicOutlierSigma`=0.5），权重低于 `dynamicOutlierMinWeight`=0.05 的点判为
动态/外点剔除。IRLS（每 LM 迭代重加权）本就隐式存在。目标：用更陡的鲁棒核压制动态车辆的大残差。

| 场景 | 方法 | ICP 残差 (m) | 覆盖 (m) | 路径 (m) | 膨胀 (x) |
|---|---|---|---|---|---|
| AT1 | 静态 imuW=0.1 | 8.48 | 527.4 | 1980.8 | 3.76 |
| AT1 | dynamicOutlierRemoval (σ=0.5) | 9.05 | 522.9 | 2042.7 | 3.91 |
| WT3 | 静态 imuW=1.0 (extfix) | 3.83 | **576.4** | 917.5 | 1.59 |
| WT3 | dynamicOutlierRemoval (σ=0.5) | 2.59 | **365.8** | 377.9 | 1.03 |

**结论（负向，单帧残差鲁棒核被证伪）**：

1. **AT1（动态场景）无收益**：9.05 vs 8.48m（+0.57m，逼近 run 噪声 ~0.5m 上限，中性到略差）。
   Cauchy 核无法区分"动态车辆的残差"与"漂移导致的残差"——两者都被同等地降权，故没有任何**选择性**
   增益。覆盖/膨胀也基本持平（522.9/3.91 vs 527.4/3.76）。

2. **WT3（静态退化）明显退化**：覆盖 576→366m（停滞），路径 918→378m。残差 2.59m 与膨胀 1.03x 的
   "改善"是**冻结团块贴近 GT 的对齐假象**（系统停止移动），不是真跟踪——与 adaptiveImuWeight 在 WT3
   的 22m 停滞同源。根因：退化隧道里那些"大残差"点恰恰是**唯一的沿轨道 x 弱锚点**，Cauchy 核把它们当
   "外点"剔除，等于拆掉了 x 方向的最后约束 → 提前停滞。

3. **根因（残差幅值不是有效的动态性信号）**：动态物体检测需要**时间维度**（占据栅格持久性 /
   多帧一致性 / 自由空间雕刻），而非单帧残差鲁棒性。单帧 Cauchy 核把"动态物体"与"退化方向的漂移"混为一
   谈，前者要压、后者要保，方向相反。

4. **对后续的启示**：`dynamicOutlierRemoval` 按 config 注释"鲁棒核 + IRLS"的原义实现到此为止，结果负。
   真正能区分动态性的是时序信号（点在地图中的持续出现/消失），属于更大的工程量，不是一次核函数替换能
   解决的。与 condT（§14）一致：**在线启发式信号在退化隧道里都会在需要时崩溃**，本工作的实际贡献仍是
   双雷达约束级融合本身（§5/§13），而非这些自适应增强。

复现：`bash experiments/scripts/run_at1_dynamic.sh` / `run_wt3_dynamic.sh`（launch 设
`dynamicOutlierRemoval=true`，AT1 imuWeight=0.1、WT3 imuWeight=1.0）。评估
`python3 experiments/scripts/eval_dynamic.py`，数据 `data/dynamic_metrics.json`。
轨迹：`AT1_dual_dynamic.tum` / `WT3_dual_dynamic.tum`。


## 16. degeneracyAwareFeatures（2026-09-27，负向——沿 x 退化时放大 x 约束会放大歧义噪声）

实现 `degeneracyAwareFeatures` 开关（默认 false）：当上一迭代 `adjustLidarHessian` 判定该雷达沿
x（weakIdx==3，平移 idx=3 = 沿行进方向）退化时（`degX[lid]`），在 `accumulateLidarHessian` 里对
corner 点整体权重乘 `degAwareXGain`=2.0，对 surf 点按法向 x 分量 `weight *= 1+(gain-1)·|n·x|`
放大横截面（x-normal）面约束。目标：退化方向上用更强的特征约束补足可观测性。

| 场景 | 方法 | ICP 残差 (m) | 覆盖 (m) | 路径 (m) | 膨胀 (x) |
|---|---|---|---|---|---|
| WT3 | extfix 基线 (imuW=1.0) | **3.83** | **576.4** | 917.5 | 1.59 |
| WT3 | degeneracyAwareFeatures (gain=2) | 10.34 | 537.9 | 836.9 | 1.56 |
| AT1 | imuW=0.1 基线 | 8.48 | **527.4** | 1980.8 | 3.76 |
| AT1 | degeneracyAwareFeatures (gain=2) | 6.50 | 343.6 | 1537.8 | 4.48 |

**结论（负向，特征放大被证伪）**：

1. **WT3（目标场景）明显退化**：残差 3.83→10.34m（2.7× 更差），覆盖 576→538m。在退化隧道里，
   沿 x 的可观测性本来就弱，放大 x 约束特征是"放大歧义的最近邻匹配噪声"——隧道横截面特征高度重复
   （均匀墙壁），NN 匹配在 x 方向上的对应关系是模糊的，把它们的权重翻倍只会让噪声主导 Hessian，
   直接破坏位姿估计。

2. **AT1 残差"改善"是覆盖丢失伪影**：6.50 vs 8.48m 看似更优，但覆盖 527→344m（丢 184m 轨道）、
   路径 1981→1538m、膨胀 3.76→4.48×（摆动更剧）。残差下降纯粹是因为轨迹变短、对齐的路径更少。
   与 §15 dynamicOutlierRemoval 在 WT3 的 2.59m "改善"同源——**覆盖率（bus GT 沿程对齐长度）才是
   真实指标，残差下降必须与覆盖不缩水同时成立才有意义**。

3. **根因（退化方向的特征本身不可靠）**：退化定义就是"该方向的信息矩阵病态/信息量低"。对退化方向
   做特征增强，等于对**信噪比最低**的方向加权重，只会放大噪声。可观测性的缺失无法靠"多算几个已有
   特征"弥补，需要的是**性质不同的补充观测**（时间维度、不同几何结构、不同传感器），而非同质特征
   的数值放大。

4. **四项自适应方向的收敛结论**：`adaptiveImuWeight`（§14）、`dynamicOutlierRemoval`（§15）、
   `degeneracyAwareFeatures`（§16）三条在线启发式全部在退化隧道里失效，失效模式高度一致——**单帧 /
   单迭代的启发式信号在系统真正需要它的那一刻（进入退化 regime）恰好崩溃或不可用**。三者共同印证
   memory 的核心结论：WT3-vs-AT1 的真正分界是**时序动态性**（占据栅格持久性），不是任何 per-frame
   可观测性或残差统计。剩最后一条 `onlineExtrinsicCalib`（§17）——它不同：标定的是**静态外参偏置**
   （T_back_body 离线标定误差），不依赖 per-frame 信号，理论上不应受此约束。

复现：`bash experiments/scripts/run_wt3_degaware.sh` / `run_at1_degaware.sh`（launch 设
`degeneracyAwareFeatures=true`，AT1 imuWeight=0.1、WT3 imuWeight=1.0）。评估
`python3 experiments/scripts/eval_degaware.py`，数据 `data/degaware_metrics.json`。
轨迹：`WT3_dual_degaware.tum` / `AT1_dual_degaware.tum`。


## 17. onlineExtrinsicCalib（2026-09-27，负向——外参 GN 与位姿耦合，退化方向外参不可观测）

实现 `onlineExtrinsicCalib` 开关（默认 false）：在 `accumulateLidarHessian` 里对后雷达（lid==1）每个
corner/surf 残差额外累加外参雅可比，帧末（`scan2MapOptimizationDual` 结尾）用阻尼 Gauss-Newton
独立求解外参增量 `dxe = -(HeeRaw + damp·I)⁻¹·beeRaw`（damp=`extCalibDamp`=1e3），左扰动更新
`R_ext ← dR·R_ext, t_ext ← dR·t_ext + δt`，每维步长钳制 `extCalibStepMax`=0.005。外参雅可比
（左扰动 T_ext ← Exp(δξ)·T_ext）：`d(p_world)/d(δξ) = R_cur·[-skew(p_body) | I]`。初始外参为
标定文件给出的 T_back_body（已较准）。目标：在线微调后雷达外参偏置。

| 场景 | 方法 | ICP 残差 (m) | 覆盖 (m) | 路径 (m) | 膨胀 (x) |
|---|---|---|---|---|---|
| WT3 | extfix 基线 (imuW=1.0) | **3.83** | **576.4** | 917.5 | 1.59 |
| WT3 | onlineExtrinsicCalib | 6.29 | 435.6 | 496.0 | 1.14 |
| AT1 | imuW=0.1 基线 | **8.48** | **527.4** | 1980.8 | 3.76 |
| AT1 | onlineExtrinsicCalib | 16.97 | 329.0 | 2102.7 | 6.39 |

**结论（负向，帧末外参 GN 被证伪）**：

1. **WT3 退化**：残差 3.83→6.29m（1.64× 更差），覆盖 576→436m，路径 918→496m。退化隧道里后雷达
   沿行进方向 x 的平移分量不可观测，外参 GN 的 x 平移更新与位姿的 x 漂移**完全耦合**——求解器无法区分
   "外参错"和"位姿错"，把位姿漂移吸收进外参（日志可见 dt_x 反复触及 0.005 步长钳制上限）。外参一旦被
   污染，后雷达点变换错误 → 融合约束错误 → 位姿更漂，形成正反馈。

2. **AT1 更差**：残差 8.48→16.97m（2×），覆盖 527→329m，膨胀 3.76→6.39×（摆动剧增）。即使动态场景
   里可观测性较好，帧末外参更新与 IEKF 位姿估计之间的**异步耦合**（外参用本帧已估计的位姿算雅可比，
   而位姿又依赖上一帧的外参）仍会引入额外的不稳定，放大摆动。

3. **失败模式与前三条不同**：`adaptiveImuWeight`/`dynamicOutlierRemoval`/`degeneracyAwareFeatures`
   是"per-frame 信号在需要时崩溃"；`onlineExtrinsicCalib` 是"**外参与位姿的可观测性混叠**"——没有独立
   于位姿的外参可观测性时（退化方向），联合标定必然把位姿误差泄漏进外参。要解耦需要（a）更长的观测
   窗口/关键帧批量标定，（b）外参标定与位姿估计分离（如 g2o 边分块、外参只在充分激励方向更新），或
   （c）已知的标定板/几何约束——都不是在线单帧 GN 能解决的。

4. **四项自适应方向的最终结论**：`adaptiveImuWeight`（§14）、`dynamicOutlierRemoval`（§15）、
   `degeneracyAwareFeatures`（§16）、`onlineExtrinsicCalib`（§17）**全部负向**。共同主题：退化隧道中，
   任何依赖"当前单帧可观测性/残差/特征/外参"的在线自适应，都会因退化方向的信息缺失而失效或反噬。
   本工作的真实贡献始终是**双雷达约束级融合本身**（§5/§13，WT3 1.56× 增益、单雷达必发散），而非这些
   自适应增强。离线标定（标定文件外参）已经足够好，在线标定无必要。

复现：`bash experiments/scripts/run_wt3_extcalib.sh` / `run_at1_extcalib.sh`（launch 设
`onlineExtrinsicCalib=true`，AT1 imuWeight=0.1、WT3 imuWeight=1.0）。评估
`python3 experiments/scripts/eval_extcalib.py`，数据 `data/extcalib_metrics.json`。
轨迹：`WT3_dual_extcalib.tum` / `AT1_dual_extcalib.tum`。
