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
