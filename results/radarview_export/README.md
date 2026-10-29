# RadarView 导出数据说明（2026-10-29）

由 `experiments/export_radarview_mat.m` 全帧（1950 帧）跑主链路后导出，
场景 `adsb_ns_fr24`（37 条 FR24 南北向真实航迹），布设 `params_coverage_online`（覆盖互补）。

## 文件与导入菜单的对应关系

| 文件 | 内容 | RadarView 菜单 | 变量名 |
| --- | --- | --- | --- |
| `R1_raw_meas.mat` | R1 原始量测点（Pd=0.6，含漏检空洞） | 文件 → 导入雷达原始测量数据 | `trackList` |
| `R2_raw_meas.mat` | R2 原始量测点（20 km / 0.60°，精度低于 R1） | 同上 | `trackList` |
| `R1_track_filtered.mat` | R1 单站 IMM-UKF 输出（逐帧外推，30 s 无空洞） | 文件 → 导入雷达数据 | `trackList` |
| `R2_track_filtered.mat` | R2 单站 IMM-UKF 输出 | 同上 | `trackList` |
| `fused_track.mat` | 融合中心系统航迹（两站互补，SCC） | 同上 | `trackList` |

## 格式

与 `D:/Desktop/RadarAgent/runs/tracks-shx/trackList.mat` 完全一致：

```
trackList(1×N) struct
  BatchNo          double   批号（原始量测文件里是目标真值编号；系统航迹是 CT 编号）
  Type             double   1
  asscPointList    1×M struct(time, lat, lon)   原始量测文件填这里
  smoothPointList  1×M struct(time, lat, lon)   跟踪/融合文件填这里
time = MATLAB datenum，基准取场景最早时间戳（2026-10-24 07:13:22）
```

## 本次导出实测

| 文件 | 航迹数 | 点数 | 中位帧间隔 | 最大空洞 |
| --- | --- | --- | --- | --- |
| R1 原始量测 | 37 | 1087 | 30 s | 240 s（Pd=0.6 漏检） |
| R2 原始量测 | 37 | 1088 | 30 s | 180 s |
| R1 跟踪输出 | 37 | 1838 | 30 s | 30 s（无空洞） |
| R2 跟踪输出 | 37 | 1910 | 30 s | 30 s |
| 系统航迹 | 41 | 2926 | 30 s | 30 s |

长度对比（`length_compare.csv`）：单站基线均值 61.7 帧 → 系统航迹 76.8 帧，
**37 条全部 ≥ 较长单站**，其中 10 条恰为 1.000（两站覆盖互相包含，几何上不可延长）。

模型集：`params_coverage_online` 已改为 **0.05 / 0.2 °/s**（2026-10-29）。
默认的 0.7/1.5 在 30 s 帧间隔下等于每帧转 21°/45°，纯外推帧误差被抬到 21.9 km；
换档后单站 18.1/20.1 → **14.2/12.2 km**，融合 18.8 → **7.92 km**。

## 复现

```bash
cd single_target_with-turn
"C:/Program Files/MATLAB/R2023a/bin/matlab.exe" -batch "addpath(pwd,'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization','validation','io','experiments'); p=params_coverage_online(); export_radarview_mat('scenario','adsb_ns_fr24','params',p);"
```
