# 双基地外辐射源雷达单目标跟踪仿真系统 — 项目文档

## 项目概述

本项目是一个完整的 **MATLAB 双基地外辐射源雷达（OTH-SWR）单目标跟踪仿真系统**。系统模拟两部异步采样的双基地雷达对空中飞机目标进行探测、跟踪与航迹融合的全流程。核心算法采用 **UKF（无迹卡尔曼滤波）+ PDA（概率数据关联）+ 模糊自适应 Q**，支持四种航迹融合算法（SCC/BC/CI/FCI），并包含拐弯机动检测与自适应处理。

### 技术栈

- MATLAB（纯.m脚本，无Simulink依赖）
- UKF（无迹卡尔曼滤波）
- 球面几何（Haversine / 大圆插值）
- 模糊推理系统（Sugeno-type）
- 概率数据关联（PDA）
- 航迹融合（Bar-Shalom-Campo / Covariance Intersection）

### 目录结构
```
single_target_with-turn/
├── config/              # 仿真参数配置
├── simulation/          # 场景仿真（航迹/雷达/量测）
├── tracker/             # 航迹管理（起始/维持/终止）
├── ukf/                 # UKF滤波器核心
├── fusion/              # 航迹融合算法
├── evaluation/          # 误差评估
├── registration/        # 空间/时间配准
├── utils/               # 球面几何工具
├── io/                  # 数据加载/保存
├── visualization/       # 可视化绘图
├── data/                # 数据文件
├── results/             # 结果输出
├── run_simulation.m     # 直线航迹仿真主程序
└── run_simulation_turn.m # 拐弯航迹仿真主程序
```

---

## 一、仿真流程总览

### 1.1 直线航迹仿真 (`run_simulation.m`)

```
Phase 0: 场景初始化（航迹生成 + 覆盖检查）
  │
Phase 1: ADS-B 系统偏差标定
  │
Phase 2: 原始点迹生成（含偏差，不做校正）
  │
Phase 3: 时间对齐策略声明（点迹级不对齐，延后到 Phase 6）
  │
Phase 4: 偏差校正 + 双基地几何反解
  │
Phase 5: 单目标航迹跟踪（UKF+PDA+模糊自适应Q）
  │
Phase 6: 航迹级时间对齐（R2→R1时间网格，CV模型外推）
  │
Phase 7: 航迹融合（SCC/BC/CI/FCI 四种算法）
  │
Phase 8: 定量误差评估
  │
Phase 9: 可视化 + 数据保存
```

### 1.2 拐弯航迹仿真 (`run_simulation_turn.m`)

在直线航迹仿真的基础上，增加了 **基础UKF vs 机动自适应UKF** 的对比：
- 基础UKF：使用 `ukf_fuzzy_adapt`（模糊自适应Q，连续平滑调节）
- 机动自适应UKF：使用 `ukf_maneuver_adapt`（趋势检测+离散提升Q）
- 两组结果分别进行融合和误差评估，产出一张综合对比图

---

## 二、推荐阅读顺序

按 **数据流** 顺序阅读，有助于理解完整仿真流水线：

### 第1层：参数与入口（先了解"跑什么"）

| 序号 | 文件 | 说明 |
|------|------|------|
| 1 | `config/simulation_params.m` | 所有仿真参数的集中定义（雷达位置、噪声、UKF参数等）|
| 2 | `run_simulation.m` | 直线航迹仿真主程序（完整9-Phase流水线）|

### 第2层：仿真场景生成（"真实世界"的模拟）

| 序号 | 文件 | 说明 |
|------|------|------|
| 3 | `simulation/aircraft_trajectory_create.m` | 航迹结构体创建（航段模型）|
| 4 | `simulation/aircraft_trajectory_locate.m` | 时间→航段定位 |
| 5 | `simulation/aircraft_trajectory_interpolate.m` | 单点线性插值 |
| 6 | `simulation/aircraft_trajectory_interpolate_batch.m` | 批量采样插值 |
| 7 | `simulation/aircraft_trajectory_generate.m` | 完整轨迹生成（封装）|
| 8 | `simulation/radar_coverage_check.m` | 雷达威力范围判定 |
| 9 | `simulation/generate_frame_detections.m` | 单帧点迹生成（目标+杂波）|

### 第3层：球面几何工具（底层数学库）

| 序号 | 文件 | 说明 |
|------|------|------|
| 10 | `utils/sphere_utils_get_earth_radius.m` | 地球半径常数 |
| 11 | `utils/sphere_utils_haversine_distance.m` | Haversine球面距离 |
| 12 | `utils/sphere_utils_azimuth.m` | 球面方位角 |
| 13 | `utils/sphere_utils_destination_point.m` | 大圆目的地点 |
| 14 | `utils/sphere_utils_interpolate_great_circle.m` | 大圆插值 |
| 15 | `utils/sphere_utils_radial_velocity.m` | 径向速度投影 |
| 16 | `utils/sphere_utils_seconds_to_datetime_str.m` | 时间格式化 |
| 17 | `utils/coord_systems_get_A.m` | WGS84长半轴 |
| 18 | `utils/coord_systems_get_E2.m` | WGS84第一偏心率平方 |
| 19 | `utils/coord_systems_lla_to_ecef.m` | LLA→ECEF转换 |
| 20 | `simulation/bistatic_inverse_solver.m` | 双基地反解（Rg,az→r1）|

### 第4层：UKF滤波器（核心算法）

| 序号 | 文件 | 说明 |
|------|------|------|
| 21 | `ukf/ukf_filter.m` | 滤波器构造函数（参数/Wm/Wc/Q/R/P）|
| 22 | `ukf/ukf_sigma_points.m` | Sigma点生成（Cholesky分解）|
| 23 | `ukf/ukf_state_transition.m` | CV模型状态转移 |
| 24 | `ukf/ukf_predict_step.m` | 预测步（Sigma点传播+加权统计）|
| 25 | `ukf/ukf_measurement_model.m` | 双基地量测模型 h(x) |
| 26 | `ukf/ukf_filter_init.m` | 首帧初始化（极坐标→经纬度）|
| 27 | `ukf/ukf_filter_update.m` | 标准UKF预测-更新循环 |
| 28 | `ukf/ukf_pda_update.m` | PDA概率数据关联更新 |
| 29 | `ukf/ukf_fuzzy_adapt.m` | 模糊自适应Q调节 |
| 30 | `ukf/ukf_maneuver_adapt.m` | 机动检测+Q动态提升 |
| 31 | `fusion/regularize_cov.m` | 协方差正则化（特征值裁剪法）|

### 第5层：航迹管理（检测→跟踪的桥梁）

| 序号 | 文件 | 说明 |
|------|------|------|
| 32 | `tracker/track_starter_mofn.m` | M/N逻辑航迹起始 |
| 33 | `tracker/jnn_association.m` | 全局最近邻点迹-航迹关联 |
| 34 | `tracker/manage_track_quality.m` | 航迹质量状态机 |
| 35 | `tracker/single_track_runner.m` | 单目标逐帧跟踪（基础版）|
| 36 | `tracker/single_track_runner_adaptive.m` | 单目标逐帧跟踪（机动自适应版）|
| 37 | `tracker/multi_track_manager.m` | 多目标跟踪管理器 |

### 第6层：融合与评估（结果产出）

| 序号 | 文件 | 说明 |
|------|------|------|
| 38 | `fusion/time_align_tracks.m` | 航迹级时间对齐 |
| 39 | `fusion/scc_fuse.m` | 简单凸组合融合 |
| 40 | `fusion/bc_fuse.m` | Bar-Shalom-Campo融合 |
| 41 | `fusion/ci_fuse.m` | 协方差交叉融合 |
| 42 | `fusion/fci_fuse.m` | 快速协方差交叉融合 |
| 43 | `fusion/run_track_fusion.m` | 融合主循环 |
| 44 | `evaluation/compute_tracking_errors.m` | UKF跟踪误差计算 |
| 45 | `evaluation/evaluate_fusion.m` | 融合误差评估 |

### 第7层：可视化（图表生成）

| 序号 | 文件 | 说明 |
|------|------|------|
| 46-62 | `visualization/plot_*.m` | 各种可视化函数 |

---

## 三、完整调用关系图

### 3.1 数据流程图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         simulation_params()                          │
│                     （参数配置，被所有模块引用）                        │
└──────────────────────────────────┬──────────────────────────────────┘
                                   │
    ┌──────────────────────────────┼──────────────────────────────┐
    │                              │                              │
    ▼                              ▼                              ▼
┌───────────┐              ┌──────────────┐              ┌──────────────┐
│ trajectory │              │ ADS-B数据    │              │  radar站点    │
│ create     │              │ (CSV加载)    │              │  (经纬度)     │
│ _create.m  │              │              │              │              │
└─────┬─────┘              └──────┬───────┘              └──────────────┘
      │                           │
      ▼                           ▼
┌───────────┐   ┌─────────────────────────────────────┐
│generate   │   │  Phase 1: ADS-B标定                  │
│_frame_    │   │  计算dr1_est, da1_est, dr2_est,     │
│detections │   │  da2_est                             │
│.m         │   └─────────────────────────────────────┘
└─────┬─────┘
      │
      ▼
┌───────────┐   ┌─────────────────────────────────────┐
│bistatic_  │   │  Phase 4: 偏差校正                    │
│inverse_   │   │  prange-dr_est → drange              │
│solver.m   │   │  paz-da_est → daz                    │
└───────────┘   │  drange+paz → bistatic_inverse_solver│
                └─────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │                   Phase 5: 航迹跟踪                        │
    │                                                          │
    │  ┌─────────────────────────────────────────────────┐     │
    │  │ single_track_runner(_adaptive).m                 │     │
    │  │   ├─ ukf_filter_init      (首帧初始化)           │     │
    │  │   ├─ ukf_predict_step     (预测)                 │     │
    │  │   │    ├─ ukf_sigma_points (Sigma点生成)         │     │
    │  │   │    └─ ukf_state_transition (CV传播)          │     │
    │  │   ├─ ukf_measurement_model (量测预测)             │     │
    │  │   ├─ ukf_pda_update       (PDA更新)              │     │
    │  │   └─ ukf_fuzzy_adapt / ukf_maneuver_adapt       │     │
    │  └─────────────────────────────────────────────────┘     │
    │                                                          │
    │  ┌────────────────────┐  ┌────────────────────────────┐  │
    │  │ track_starter_mofn │  │ manage_track_quality       │  │
    │  │ (M/N起始逻辑)       │  │ (航迹状态机 TYPE 1/2/6/7)  │  │
    │  └────────────────────┘  └────────────────────────────┘  │
    │                                                          │
    │  ┌─────────────────────────────┐                         │
    │  │ jnn_association             │                         │
    │  │ (全局最近邻点迹-航迹关联)     │                         │
    │  └─────────────────────────────┘                         │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 6: 航迹级时间对齐                       │
    │  time_align_tracks (R2航迹 CV模型回退到R1时间网格)         │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 7: 航迹融合                             │
    │  run_track_fusion                                        │
    │    ├─ scc_fuse   (简单凸组合)                             │
    │    ├─ bc_fuse    (Bar-Shalom-Campo, 维护互协方差)         │
    │    ├─ ci_fuse    (Covariance Intersection, 优化w)         │
    │    └─ fci_fuse   (Fast CI, 闭式解w)                       │
    │  └─ regularize_cov (融合前后协方差正则化)                  │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 8: 误差评估                             │
    │  compute_tracking_errors  (单站UKF位置RMSE)               │
    │  evaluate_fusion           (融合航迹 vs 真值 RMSE)        │
    │  compute_stage_rmse / compute_stitched_rmse_at_detections │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 9: 可视化                               │
    │  plot_scene_overview / plot_point_cloud_3d /              │
    │  plot_single_track_result / plot_single_fusion_result /   │
    │  plot_turn_* (拐弯场景专用)                                │
    └──────────────────────────────────────────────────────────┘
```

### 3.2 模块内调用关系

#### UKF模块内部调用链

```
ukf_filter.m ──► 创建ukf结构体（包含Q,R,P,Wm,Wc,lam等）
     │
     ├──► ukf_filter_init.m ──► 第一帧量测初始化
     │         └──► ukf_meas_to_latlon.m ──► bistatic逆解+球面正算
     │
     ├──► ukf_filter_update.m ──► 正常滤波更新（有量测时）
     │         ├──► ukf_predict_step.m
     │         │      ├──► ukf_sigma_points.m ──► Cholesky分解
     │         │      └──► ukf_state_transition.m ──► F*x (CV模型)
     │         └──► ukf_measurement_model.m ──► 双基地h(x)
     │
     ├──► ukf_pda_update.m ──► PDA多假设更新（多点迹时）
     │         ├──► ukf_filter_update.m （单量测退化时）
     │         └──► 内部计算 β_i 关联概率 + 加权新息
     │
     ├──► ukf_fuzzy_adapt.m ──► 模糊推理调节Q
     │         └──► trimf_val() ──► 三角形隶属函数
     │
     └──► ukf_maneuver_adapt.m ──► 机动检测+Q提升
               └──► trimf_val_mv() ──► 三角形隶属函数
```

#### Tracker模块调用链

```
single_track_runner.m / single_track_runner_adaptive.m
  │
  ├──► ukf_filter_init()           ──► 两点差分初始化UKF
  ├──► ukf_predict_step()          ──► UKF预测
  ├──► ukf_measurement_model()     ──► 量测预测
  ├──► ukf_pda_update()            ──► PDA关联更新
  ├──► ukf_fuzzy_adapt()           ──► 模糊自适应Q（基础版）
  └──► ukf_maneuver_adapt()        ──► 机动自适应（adaptive版）

multi_track_manager.m
  │
  ├──► find_active()               ──► 分离活跃/历史航迹
  ├──► ukf_predict_step()          ──► 批量UKF预测
  ├──► ukf_measurement_model()     ──► 量测预测(含P_zz)
  ├──► jnn_association()           ──► 全局最近邻关联
  ├──► ukf_pda_update()            ──► PDA更新已关联航迹
  ├──► ukf_fuzzy_adapt()           ──► 模糊自适应Q
  ├──► manage_track_quality()      ──► 航迹质量状态机
  └──► track_starter_mofn()        ──► 剩余点迹新起始

track_starter_mofn.m
  │
  ├──► ukf_filter_init()           ──► 两点差分初始化新航迹
  ├──► validate_candidate_sequence()──► 直线运动一致性验证
  └──► cleanup_stale_candidates()  ──► 清理过期候选
```

#### Fusion模块调用链

```
run_track_fusion.m
  │
  ├──► time_align_tracks()         ──► 时间对齐（外部先调用）
  ├──► scc_fuse()                   ──► 简单凸组合
  │     └──► regularize_cov()
  ├──► bc_fuse()                    ──► Bar-Shalom-Campo
  │     └──► regularize_cov()
  ├──► ci_fuse()                    ──► Covariance Intersection
  │     └──► regularize_cov() + fminbnd优化w
  └──► fci_fuse()                   ──► Fast CI（闭式解w）
        └──► regularize_cov()
```

---

## 四、各文件详细说明

### 4.1 根目录文件

#### `run_simulation.m` — 直线航迹仿真主程序
- **功能**：双基地雷达单目标直线飞行仿真的完整入口
- **输入/输出**：无命令行参数 / 生成 .mat 结果文件和可视化图片
- **9个Phase**：
  - Phase 0: 利用 `aircraft_trajectory_create` 生成单航段直线航迹
  - Phase 1: 从ADS-B CSV数据中统计估计两部雷达的系统偏差
  - Phase 2: 对每帧调用 `generate_frame_detections` 生成含噪点迹
  - Phase 3: 声明时间对齐策略（R1采样0s/30s/60s，R2采样13s/43s/73s）
  - Phase 4: 偏差校正 + `bistatic_inverse_solver` 解算经纬度
  - Phase 5: `single_track_runner` 执行UKF+PDA跟踪
  - Phase 6: `time_align_tracks` R2→R1时间对齐
  - Phase 7: `run_track_fusion` 四种算法融合
  - Phase 8: `compute_tracking_errors` + `evaluate_fusion` 误差评估
  - Phase 9: 可视化绘图 + 数据保存

#### `run_simulation_turn.m` — 拐弯航迹仿真主程序
- **功能**：对比基础UKF与机动自适应UKF在拐弯场景的表现
- **与`run_simulation.m`的区别**：
  - 使用 `aircraft_trajectory_create_turn` 生成带拐弯的3航路点航迹
  - 同时运行 `single_track_runner`（基础版）和 `single_track_runner_adaptive`（机动自适应版）
  - 两组结果分别融合（共8种融合结果：4算法×2跟踪器）
  - 产出对比图表：基础 vs 自适应的RMSE、轨迹、机动检测统计

### 4.2 config/ 目录

#### `simulation_params.m` — 仿真参数配置
- **功能**：集中管理所有仿真可调参数，返回一个结构体
- **包含12组参数**：
  1. 时间（dt=30s, duration=3600s, 时间偏移量）
  2. 站点几何（R1接收站113°E/33.5°N，R2接收站115°E/33°N，照射站109°E/33.5°N和111°E/33°N）
  3. 覆盖范围（波束中心92°/91°，宽度15°，距离1000-2000km）
  4. 目标航迹（waypoints定义）
  5. 量测噪声（R1精密站σ_range=7km/σ_az=0.35°，R2标准站σ_range=14km/σ_az=0.6°）
  6. 系统偏差（R1: +20km/-3°，R2: -15km/+3.5°）
  7. UKF滤波参数（α=1e-3, β=2, κ=0, Q_scale=2e4）
  8. 航迹管理（M/N=4/8, K_loss=15, gate_sigma=2.0）
  9. 检测/虚警（Pd=0.6, FAR=0.001）
  10. PDA加权（Pd_gate=0.8647, clutter_intensity）
  11. 模糊自适应Q（窗口8帧, Q因子0.6-1.5）
  12. 随机数种子（42）

### 4.3 simulation/ 目录 — 场景仿真模块

#### `aircraft_trajectory_create.m` — 航迹结构体创建
- **功能**：由航路点序列创建分段恒向线（Rhumb Line）航迹
- **数学模型**：Haversine距离 + 匀速运动 → 每个航段计算 `lon_rate`/`lat_rate`
- **调用者**：`run_simulation.m`、`aircraft_trajectory_create_turn.m`

#### `aircraft_trajectory_create_turn.m` — 拐弯航迹生成器
- **功能**：创建含约120°拐角的3航路点航迹
- **航路点**：(126.0,32.5)→(128.5,33.5)→(128.6,31.7)，航速140m/s
- **调用者**：`run_simulation_turn.m`

#### `aircraft_trajectory_locate.m` — 时间→航段定位
- **功能**：给定时间t，线性扫描查找所在的航段索引和段内偏移量
- **调用者**：`aircraft_trajectory_interpolate.m`

#### `aircraft_trajectory_interpolate.m` — 单点航迹插值
- **功能**：对给定时间t，在所属航段内线性插值出(lon, lat, lon_rate, lat_rate)
- **公式**：`lon = seg.start(1) + seg.lon_rate * t_seg`
- **调用者**：`aircraft_trajectory_interpolate_batch.m`

#### `aircraft_trajectory_interpolate_batch.m` — 批量采样插值
- **功能**：对时间数组逐点插值，输出N×5矩阵 `[lon, lat, lon_rate, lat_rate, time]`
- **调用者**：`aircraft_trajectory_generate.m`

#### `aircraft_trajectory_generate.m` — 完整轨迹生成
- **功能**：薄封装层，对`traj.time_array`中所有采样点生成完整轨迹
- **调用者**：`run_simulation.m`, `run_simulation_turn.m`

#### `radar_coverage_check.m` — 雷达威力范围判定
- **功能**：判断目标是否在雷达波束和距离范围内
- **条件**：r1∈[1000,2000]km 且 |az - beam_center| ≤ beam_width/2
- **调用者**：`generate_frame_detections.m`、Phase 1标定循环

#### `generate_frame_detections.m` — 单帧点迹生成
- **功能**：对单部雷达生成一帧的点迹列表（目标检测 + 虚警杂波）
- **流程**：覆盖率检查 → 检测概率（Pd=0.6）→ 加噪声 → 泊松虚警
- **调用者**：`run_simulation.m` Phase 2

#### `bistatic_inverse_solver.m` — 双基地几何反解
- **功能**：由群距离Rg和方位角az解算目标到接收站距离r1及经纬度
- **公式**：`r1 = 0.5*(Rg²-d²)/(Rg-d*cos(φ))`
- **调用者**：Phase 4偏差校正、`ukf_meas_to_latlon.m`

#### `radar_tracker.m` — 雷达内部跟踪器（简化版）
- **功能**：模拟雷达CFAR检测后的M/N起始+K_loss终止逻辑
- **状态机**：UNINITIATED(0) → INITIATING(1) → TRACKING(2)
- **调用者**：用于演示"真实雷达内部处理"概念

#### `stitch_tracks.m` — 同雷达航迹拼接
- **功能**：合并同一雷达的多段碎片航迹，填补短间隙（大圆插值）
- **调用者**：多目标场景（非单目标主流程）

### 4.4 ukf/ 目录 — UKF滤波器核心

#### `ukf_filter.m` — 滤波器构造函数
- **功能**：创建UKF结构体（状态空间n=4, 量测空间m=3）
- **状态向量**：`x = [lon; lon_dot; lat; lat_dot]`
- **量测向量**：`z = [range; azimuth; radial_vel]`
- **预计算**：Wm/Wc权重、R/Q/P矩阵

#### `ukf_sigma_points.m` — Sigma点生成
- **功能**：Cholesky分解生成2n+1=9个Sigma点
- **公式**：`X₀=x, Xᵢ=x+√((n+λ)P)ᵢ, Xᵢ₊ₙ=x-√((n+λ)P)ᵢ`
- **调用者**：`ukf_predict_step.m`

#### `ukf_state_transition.m` — CV模型状态转移
- **功能**：`x_next = F(dt)*x`，匀速运动模型
- **F矩阵**：块对角 `[1 dt; 0 1]; [1 dt; 0 1]`
- **调用者**：`ukf_predict_step.m`

#### `ukf_predict_step.m` — UKF预测步
- **功能**：生成Sigma点→状态传播→加权统计x_pred/P_pred
- **调用者**：`ukf_filter_update.m`、`single_track_runner.m`

#### `ukf_measurement_model.m` — 双基地非线性量测模型
- **功能**：`z=h(x)`，将状态映射到量测空间
- **包含**：Haversine群距离 + 球面方位角 + 双基地径向速度
- **调用者**：`ukf_filter_update.m`、`single_track_runner.m`

#### `ukf_filter_init.m` — UKF初始化
- **功能**：首帧量测→双基地反解→零速初始化
- **支持**：单点初始化（零速）和两点初始化（差分估计速度）
- **调用者**：`single_track_runner.m`、`track_starter_mofn.m`

#### `ukf_filter_update.m` — 标准UKF更新
- **功能**：预测+新息+卡尔曼增益+状态/协方差更新+方位角包裹
- **调用者**：`single_track_runner.m`

#### `ukf_pda_update.m` — PDA概率数据关联更新
- **功能**：βᵢ关联概率计算→加权新息→PDA状态/协方差更新
- **优势**：不"孤注一掷"选最近点，杂波鲁棒
- **调用者**：`single_track_runner.m`、`multi_track_manager.m`

#### `ukf_fuzzy_adapt.m` — 模糊自适应Q
- **功能**：NIS滑动均值→模糊隶属度(VS/S/M/L/VL)→Sugeno解模糊→EMA平滑
- **原理**：NIS偏高→Q↑信任量测；NIS偏低→Q↓信任模型
- **调用者**：`single_track_runner.m`、`multi_track_manager.m`

#### `ukf_maneuver_adapt.m` — 机动自适应UKF
- **功能**：短时/长时NIS趋势比较→机动检测→Q渐进提升(1.5→3.5倍)
- **与fuzzy_adapt的区别**：趋势检测+离散大幅提升（vs 连续平滑调节）
- **调用者**：`single_track_runner_adaptive.m`

### 4.5 tracker/ 目录 — 航迹管理

#### `single_track_runner.m` — 单目标航迹跟踪器（基础版）
- **功能**：M/N滑窗起始 + NN关联 + PDA更新 + K_loss终止
- **状态机**：INITIATING → TRACKING → LOST → INITIATING（循环）
- **M/N逻辑**：滑窗内多点迹配对→速度检验→共识评分→两点差分初始化
- **调用者**：`run_simulation.m`

#### `single_track_runner_adaptive.m` — 单目标航迹跟踪器（机动自适应版）
- **功能**：与基础版相同框架，但使用`ukf_maneuver_adapt`替代`ukf_fuzzy_adapt`
- **新增**：机动预扫描（渐进放宽门限）、新息历史记录、机动检测统计
- **调用者**：`run_simulation_turn.m`

#### `jnn_association.m` — 全局最近邻关联
- **功能**：基于马氏距离的贪心全局点迹-航迹分配
- **流程**：代价矩阵→迭代选最小→移除行列→保证1对1
- **特点**：地理预筛选（120→80km可变） + 自适应波门放宽
- **调用者**：`multi_track_manager.m`

#### `manage_track_quality.m` — 航迹质量状态机
- **功能**：对称计分（±1）+ 降级/升级阈值
- **航迹类型**：TEMPORARY(6)→RELIABLE(1)↔MAINTAIN(2)→HISTORY(7)
- **阈值**：TEMPORARY quality≥10升级，<3降级；RELIABLE quality<8降MAINTAIN
- **调用者**：`multi_track_manager.m`、`single_track_runner.m`

#### `track_starter_mofn.m` — M/N逻辑航迹起始
- **功能**：维护tempPool→回溯匹配→M/N条件→两点初始化
- **高级特性**：直线运动一致性验证（鲁棒回归）、航迹复活（死亡航迹再续接）
- **调用者**：`multi_track_manager.m`

#### `multi_track_manager.m` — 多目标航迹管理器
- **功能**：单帧分发引擎，串联预测→关联→更新→质量管理→新起始
- **8个步骤**：分离活跃航迹→批量UKF预测→JNN关联→PDA更新→纯预测→质量状态机→新起始
- **调用者**：多目标场景（非单目标主流程）

### 4.6 fusion/ 目录 — 航迹融合算法

#### `time_align_tracks.m` — 航迹级时间对齐
- **功能**：将R2航迹用CV模型外推到R1时间网格
- **方法**：`x(t-Δt) = F(-Δt)·x(t)`, `P(t-Δt) = F·P·F' + Q(|Δt|)`
- **参数**：R2滞后R1 13秒
- **调用者**：Phase 6（融合前必须先对齐）

#### `scc_fuse.m` — 简单凸组合（Simple Convex Combination）
- **功能**：假设两源误差独立，信息矩阵直接相加
- **公式**：`P⁻¹ = P₁⁻¹+P₂⁻¹`, `x = P*(P₁⁻¹x₁+P₂⁻¹x₂)`
- **等效权重**：w=0.5（隐式）
- **调用者**：`run_track_fusion.m`

#### `bc_fuse.m` — Bar-Shalom-Campo融合
- **功能**：考虑互协方差P12的精确融合
- **公式**：`S=P₁+P₂-P₁₂-P₁₂'`, `x=x₁+(P₁-P₁₂)S⁻¹(x₂-x₁)`
- **互协方差维护**：P12预测（F·P12·F'+Q/2）+ P12更新（迹收缩比近似）
- **调用者**：`run_track_fusion.m`

#### `ci_fuse.m` — 协方差交叉（Covariance Intersection）
- **功能**：无需互协方差，用fminbnd优化w最小化det(P_fused)
- **公式**：`P⁻¹ = w*P₁⁻¹+(1-w)*P₂⁻¹`
- **调用者**：`run_track_fusion.m`

#### `fci_fuse.m` — 快速协方差交叉（Fast CI）
- **功能**：无需迭代优化，用迹的倒数闭式解计算权重
- **公式**：`w = tr(P₁)⁻¹/(tr(P₁)⁻¹+tr(P₂)⁻¹)`
- **调用者**：`run_track_fusion.m`

#### `regularize_cov.m` — 协方差正则化
- **功能**：特征值分解 + 裁剪负/小特征值→重构对称正定矩阵
- **双阈值**：绝对1e-12 + 相对1e-6×max_λ
- **调用者**：所有融合函数、`ukf_filter_update.m`

#### `run_track_fusion.m` — 融合主循环
- **功能**：逐帧遍历匹配航迹对，调用指定融合算法
- **支持**：四种算法（SCC/BC/CI/FCI）+ 单源降级（R1_only/R2_only）
- **调用者**：`run_simulation.m` Phase 7

### 4.7 utils/ 目录 — 球面几何工具

| 文件 | 功能 | 关键公式/参数 |
|------|------|---------------|
| `sphere_utils_get_earth_radius.m` | 地球半径 | R=6371000.0m |
| `sphere_utils_haversine_distance.m` | 球面大圆距离 | a=sin²(Δlat/2)+cos(lat₁)cos(lat₂)sin²(Δlon/2)，c=2·atan2(√a,√(1-a)) |
| `sphere_utils_azimuth.m` | 大圆初始方位角 | α=atan2(sin(Δlon)cos(lat₂), cos(lat₁)sin(lat₂)-sin(lat₁)cos(lat₂)cos(Δlon)) |
| `sphere_utils_destination_point.m` | 大圆目的地点（正算） | lat₂=asin(sin(lat₁)cos(δ)+cos(lat₁)sin(δ)cos(α)) |
| `sphere_utils_interpolate_great_circle.m` | 大圆插值 | 通过distance+azimuth→destination走fraction*dist |
| `sphere_utils_radial_velocity.m` | 径向速度投影 | v_east=lon_rate*(π/180)*R*cos(lat)，rv=v_east*sin(az)+v_north*cos(az) |
| `sphere_utils_seconds_to_datetime_str.m` | 时间格式化 | t=ref_time+seconds(secs)→datestr |
| `coord_systems_get_A.m` | WGS84长半轴 | A=6378137.0m |
| `coord_systems_get_E2.m` | WGS84第一偏心率平方 | E2=2f-f², f=1/298.257223563 |
| `coord_systems_lla_to_ecef.m` | LLA→ECEF | N=A/√(1-E2·sin²(lat)), x=(N+alt)cos(lat)cos(lon) |
| `compute_stage_rmse.m` | 阶段RMSE计算 | Haversine距离误差的RMSE |
| `compute_stitched_rmse_at_detections.m` | 拼接航迹RMSE | 仅在原始检测时刻比较，避免外推值污染 |

### 4.8 evaluation/ 目录 — 误差评估

#### `compute_tracking_errors.m` — UKF跟踪误差计算
- **功能**：逐帧计算UKF滤波位置、校准后点迹、校准前原始点迹 vs 真值的误差
- **统计量**：中位数、均值、标准差、RMSE、95th百分位
- **输出**：`errorStats`结构体（含逐帧误差+汇总统计）

#### `evaluate_fusion.m` — 融合误差评估
- **功能**：对比四种融合算法和两个单站的RMSE
- **流程**：匹配对→飞机映射→逐帧融合误差→单站误差→汇总统计
- **输出**：`fusion_eval`结构体（含`overall`对比表）

### 4.9 registration/ 目录 — 空间/时间配准

| 文件 | 功能 |
|------|------|
| `create_unified_grid.m` | 创建统一时间网格 |
| `align_radar_to_grid.m` | 球面大圆插值时间对齐 |
| `spherical_interpolate_.m` | 球面大圆单点插值 |
| `cost_fcn_with_params.m` | 空间配准EML代价函数 |
| `estimate_biases.m` | 空间偏差估计（最大似然） |

### 4.10 io/ 目录 — 数据I/O

| 文件 | 功能 |
|------|------|
| `extract_measurement_field.m` | 从量测cell数组提取指定字段 |
| `load_adsb.m` | 加载ADS-B CSV数据，提取航迹并重采样到仿真网格 |
| `save_all.m` | 批量保存CSV+MAT+JSON格式结果 |

### 4.11 visualization/ 目录 — 可视化

**通用可视化（直线航迹）：**

| 文件 | 功能 |
|------|------|
| `plot_scene_overview.m` | 场景总览（真值航迹+雷达位置+覆盖范围） |
| `plot_point_cloud_3d.m` | 3D点云图（距离×方位×径向速度） |
| `plot_single_track_result.m` | 单目标跟踪综合图（6子图） |
| `plot_single_fusion_result.m` | 融合结果综合图（8子图） |
| `plot_combined_tracks.m` | 多航迹并排对比 |
| `plot_error_timeline.m` | 误差时间序列 |
| `plot_tracks_vs_truth.m` | 航迹 vs 真值对比 |
| `plot_tracker_result.m` | 跟踪器状态统计 |

**拐弯航迹专用可视化：**

| 文件 | 功能 |
|------|------|
| `plot_turn_point_clouds.m` | R1/R2点云+基础UKF(虚线)+自适应UKF(实线) |
| `plot_turn_radar_compare.m` | 单站对比（地图+拐弯放大+误差时间线+RMSE柱状图） |
| `plot_turn_fusion_map.m` | 融合地图对比（基础+自适应+拐弯放大+信息面板） |
| `plot_turn_rmse_bars.m` | RMSE柱状图总览（基础灰vs自适应绿+数值汇总） |
| `plot_turn_comparison.m` | 轨迹对比图（真值+基础+自适应） |
| `plot_turn_comprehensive.m` | 全图层综合对比（地图+按钮控制显隐） |
| `plot_turn_fusion_compare.m` | 融合轨迹对比（四种算法×两种跟踪器） |
| `plot_turn_single_compare.m` | 单站跟踪结果（6子图版） |
| `plot_error_timeline_turn.m` | 拐弯误差时间线 |
| `plot_turn_fusion_map.m` | 融合地图总览 |

---

## 五、关键算法说明

### 5.1 UKF（无迹卡尔曼滤波）

**状态空间**：4维 `[lon; lon_dot; lat; lat_dot]`
**量测空间**：3维 `[bistatic_range; azimuth; radial_velocity]`

**每帧滤波循环**：
1. **Sigma点生成**（`ukf_sigma_points`）：Cholesky分解 `(n+λ)P=LL'`，生成9个Sigma点
2. **预测步**（`ukf_predict_step`）：CV模型传播→加权统计x_pred/P_pred
3. **量测传播**（`ukf_measurement_model`）：每个Sigma点通过非线性h(x)
4. **更新步**（`ukf_filter_update`）：卡尔曼增益 `K=Pxz/Pzz` → 状态修正
5. **PDA增强**（`ukf_pda_update`）：波门内多点迹按Gaussian似然加权

### 5.2 航迹起始与终止

**M/N逻辑**：N帧滑动窗口内至少M帧有点迹 → 触发起始
**两点差分初始化**：velocity = Δposition / Δt，速度合理性检验 [30, 600] m/s
**K_loss终止**：连续K帧漏检→航迹丢失

### 5.3 航迹质量状态机

```
TEMPORARY(6) ──── quality≥10 ────► RELIABLE(1)
                                      │
                    quality<8          │
                      ◄────────────────┘
                                      │
                  quality≥10           │
                      ────────────────►
                                      │
RELIABLE/MainTAIN ─ quality<5 ──► HISTORY(7) (死亡)
```

### 5.4 航迹融合算法对比

| 算法 | 互协方差需求 | 权重计算 | 特点 |
|------|-------------|----------|------|
| SCC | 不需要 | w=0.5(等效) | 最简单，假设完全独立 |
| BC | 需要维护P12 | 动态计算 | 最精确（如果P12准确） |
| CI | 不需要 | fminbnd优化 | 保守融合，不低估不确定性 |
| FCI | 不需要 | tr(P)⁻¹闭式解 | CI的快速近似，无需迭代 |

### 5.5 机动检测策略

**基础模糊自适应**（`ukf_fuzzy_adapt`）：
- 基于NIS滑动平均值
- 5级模糊隶属度(VS/S/M/L/VL)→Sugeno解模糊
- Q调节因子 [0.6, 3.0]，EMA平滑(η=0.35)

**机动自适应**（`ukf_maneuver_adapt`）：
- 短时(3帧) vs 长时NIS趋势比较
- 机动判定：短时NIS > 长时NIS×1.25 且 短时NIS > 2.8
- Q渐进提升：1.5→2.3→3.1→3.5（而非瞬间跳跃）
- 恢复：连续4帧正常→结束机动

---

## 六、配置文件与数据说明

### 仿真参数默认值
- 时间步长：30 s
- 仿真总时长：3600 s (1小时)
- 检测概率：60%
- 虚警率：0.1%
- 雷达距离：1000-2000 km
- 波束宽度：15°

### ADS-B数据格式
CSV文件，19列，无表头，包含ICAO地址、经纬度、航向、高度、地速、时间戳等。

### 结果文件格式
- `.mat`：MATLAB原生格式，完整数据
- 可视化保存到 `results/` 目录

---

## 七、扩展与修改指南

1. **修改雷达站点**：编辑 `config/simulation_params.m` 中的 `radar1_*` / `radar2_*` 参数
2. **修改航迹**：修改 waypoints 或调用 `aircraft_trajectory_create_turn` 调整拐弯点
3. **修改噪声水平**：编辑 `params.radar*_noise_std_m` 和 `radar*_azimuth_noise_std_deg`
4. **调整UKF参数**：修改 `ukf_alpha/beta/kappa/Q_scale/P_pos_std/P_vel_std`
5. **切换融合算法**：修改 `method_names` 列表
6. **增加新融合算法**：在 `fusion/` 创建新函数，加入 `run_track_fusion.m` 的 switch 分支
7. **换用其他运动模型**：修改 `ukf_state_transition.m` 中的F矩阵（如改用CTRV模型）
