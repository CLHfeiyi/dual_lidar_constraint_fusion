# Experiments

本目录是论文的复现材料，**不属于核心算法**。核心算法位于仓库根目录
（`src/`、`utility.h`、`iekf_paper.h`、`vgicp_loop.h`、`config/`、`launch/`）。

| 内容 | 说明 |
|---|---|
| `RESULTS.md` | 完整消融实验记录与量化结果（§1–§17） |
| `scripts/` | 复现脚本（运行 / 评估 / 绘图） |
| `data/` | 评估指标 JSON 输出（`*.json`；二进制图与 npz 已 gitignore） |

## 关于脚本中的绝对路径

`scripts/` 与部分 `../scripts/` 下的绘图脚本**包含本机数据集的绝对路径**
（`/home/clh/...` 下的 bag、GT、地图文件）。这些数据集未随仓库分发，
复现时需将路径替换为你自己的数据路径：

```bash
grep -rn "/home/clh" experiments/scripts/ ../scripts/
```

复现命令见 `RESULTS.md` 各节末尾的 "复现" 一行。
