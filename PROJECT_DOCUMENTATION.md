# 双基地OTH-SWR单目标跟踪仿真系统 — 项目文档

## 项目概述

本项目是一个完整的 **MATLAB 双基地高频天波超视距雷达（OTH-SWR）单目标跟踪仿真系统**。系统模拟两部异步采样的双基地雷达对空中飞机目标进行探测、跟踪与航迹融合的全流程。核心算法采用 **UKF（无迹卡尔曼滤波）+ PDA（概率数据关联）+ 模糊自适应 Q**，支持四种航迹融合算法（SCC/BC/CI/FCI），并包含拐弯机动检测与自适应处理。

### 技术栈

- MATLAB（纯 .m 脚本，无 Simulink 依赖，纯函数式编程）
- UKF（无迹卡尔曼滤波，action dispatcher 模式）
- 球面几何（Haversine / 大圆插值 / 双基地反解）
- 模糊推理系统（Sugeno-type，三角形隶属函数）
- 概率数据关联（PDA，β 权重计算）
- 航迹融合（SCC / Bar-Shalom-Campo / Covariance Intersection / Fast CI）

### 架构特点（v2.0 精简重构）

本版本经过大幅精简重构，核心变化：
- **UKF 模块合并**：原来 10 个独立文件合并为 2 个 dispatcher 文件（`ukf_jichu.m` + `ukf_zishiying.m`），通过 action 字符串分发
- **关联模块独立**：NN 关联和 PDA 权重计算提取为独立纯函数，与 UKF 数学解耦
- **航迹起始独立**：M/N 滑窗起始逻辑提取为 `initiation/track_initiation.m`
- **可视化聚合**：原来 17 个独立绘图文件合并为 3 个 dispatcher 文件
- **评估聚合**：跟踪误差评估和融合评估合并为 `evaluate_all.m`
- **冗余消除**：删除了可内联的常量文件、重复的工具函数

### 目录结构

```
single_target_with-turn/
├── config/                  # 仿真参数配置
│   └── simulation_params.m  #   唯一参数入口，13 个模块
├── simulation/              # 场景仿真（航迹/雷达/量测）
│   ├── aircraft_trajectory_create.m      # 航迹结构体创建
│   ├── aircraft_trajectory_locate.m      # 时间→航段定位
│   ├── aircraft_trajectory_interpolate.m # 航迹插值（含批量生成）
│   ├── radar_coverage_check.m            # 雷达威力范围判定
│   ├── radar_station_true_polar.m        # 双基地真实极坐标计算
│   ├── bistatic_inverse_solver.m         # 双基地几何反解
│   ├── measurement_simulator.m           # 量测仿真器（创建+量测）
│   ├── generate_frame_detections.m       # 单帧点迹生成（目标+杂波）
│   └── tracker_utils.m                  # 雷达内部跟踪器+航迹拼接
├── ukf/                     # UKF 滤波器核心（2 文件）
│   ├── ukf_jichu.m          #   基础 UKF（create/init/prepare/update/predict/measurement）
│   └── ukf_zishiying.m      #   自适应 UKF（机动检测 + 模糊自适应 Q）
├── association/             # 点迹-航迹关联
│   ├── nn_associate.m       #   最近邻（NN）关联
│   └── pda_weight.m         #   PDA β 权重计算
├── initiation/              # 航迹起始
│   └── track_initiation.m   #   M/N 滑窗航迹起始器
├── tracker/                 # 航迹管理
│   ├── single_track_runner.m           # 单目标逐帧跟踪（基础版）
│   ├── single_track_runner_adaptive.m  # 单目标逐帧跟踪（机动自适应版）
│   ├── multi_track_manager.m           # 多目标跟踪管理器
│   └── track_management.m             # JNN 关联 + 航迹质量状态机
├── fusion/                  # 航迹融合算法
│   ├── time_align_tracks.m        # 航迹级时间对齐（CV 模型外推）
│   ├── track_fusion_algorithms.m  # 四种融合算法合集（SCC/BC/CI/FCI）
│   ├── run_track_fusion.m         # 融合主循环
│   └── regularize_cov.m           # 协方差正则化（特征值裁剪）
├── evaluation/              # 误差评估
│   └── evaluate_all.m       #   跟踪误差 + 融合评估
├── registration/            # 空间/时间配准
│   ├── align_radar_to_grid.m      # 球面大圆插值时间对齐
│   ├── cost_fcn_with_params.m     # 空间配准 EML 代价函数
│   └── estimate_biases.m          # 空间偏差估计（最大似然）
├── utils/                   # 球面几何工具
│   ├── sphere_utils_haversine_distance.m   # Haversine 球面距离
│   ├── sphere_utils_azimuth.m              # 球面方位角
│   ├── sphere_utils_destination_point.m    # 大圆目的地点（正算）
│   ├── sphere_utils_interpolate_great_circle.m  # 大圆插值
│   ├── sphere_utils_radial_velocity.m      # 径向速度投影
│   ├── sphere_utils_seconds_to_datetime_str.m  # 时间格式化
│   └── coord_systems_lla_to_ecef.m         # LLA→ECEF 坐标转换
├── io/                      # 数据 I/O
│   ├── load_adsb.m                 # 加载 ADS-B CSV 数据
│   ├── save_all.m                  # 批量保存结果
│   └── extract_measurement_field.m # 量测字段提取
├── visualization/           # 可视化（3 个 dispatcher 文件）
│   ├── plot_scene_overview.m   # 场景总览
│   ├── plot_point_cloud_3d.m   # 3D 点云图
│   ├── plot_results.m          # 直线航迹结果（7 种模式）
│   ├── plot_turn_spatial.m     # 拐弯空间可视化（4 种模式）
│   └── plot_turn_stats.m       # 拐弯统计分析（4 种模式）
├── data/                    # 数据文件（ADS-B CSV 等）
├── results/                 # 结果输出
├── run_simulation.m         # 直线航迹仿真主程序（9-Phase 流水线）
└── run_simulation_turn.m    # 拐弯航迹仿真主程序（基础 vs 自适应对比）
```

---

## 一、仿真流程总览

### 1.1 直线航迹仿真（`run_simulation.m`）

```
Phase 0: 场景初始化（航迹生成 + 覆盖检查）
  │
Phase 1: ADS-B 系统偏差标定（统计 dr_est, da_est）
  │
Phase 2: 原始点迹生成（含偏差，不做校正）
  │
Phase 3: 时间对齐策略声明（点迹级不对齐，延后到 Phase 6）
  │
Phase 4: 偏差校正 + 双基地几何反解（prange - dr_est → drange）
  │
Phase 5: 单目标航迹跟踪（UKF + PDA + 模糊自适应 Q）
  │        └── prepare → NN 关联 → PDA 加权 → UKF 更新
  │
Phase 6: 航迹级时间对齐（R2 → R1 时间网格，CV 模型外推）
  │
Phase 7: 航迹融合（SCC / BC / CI / FCI 四种算法，直接 1 对 1）
  │
Phase 8: 定量误差评估（融合 + 单站 RMSE）
  │
Phase 9: 可视化 + 数据保存
```

### 1.2 拐弯航迹仿真（`run_simulation_turn.m`）

在直线航迹仿真的基础上，增加了**基础 UKF vs 机动自适应 UKF** 的对比：
- **基础 UKF**：使用 `ukf_zishiying`（模糊自适应 Q + 机动检测，连续平滑调节）
- **机动自适应 UKF**：同样使用 `ukf_zishiying`，但启用了机动预检测和渐进 Q 提升
- 两组结果分别进行融合和误差评估，产出一张综合对比图

---

## 二、推荐阅读顺序

按**数据流**顺序阅读，有助于理解完整仿真流水线：

### 第 1 层：参数与入口（先了解"跑什么"）

| 序号 | 文件 | 说明 |
|------|------|------|
| 1 | `config/simulation_params.m` | 所有仿真参数的集中定义（13 个模块：时间、几何、覆盖、航迹、噪声、偏差、UKF、航迹管理、检测/虚警、PDA、模糊自适应 Q、ADS-B 路径、随机种子） |
| 2 | `run_simulation.m` | 直线航迹仿真主程序（完整 9-Phase 流水线） |

### 第 2 层：仿真场景生成（"真实世界"的模拟）

| 序号 | 文件 | 说明 |
|------|------|------|
| 3 | `simulation/aircraft_trajectory_create.m` | 航迹结构体创建（航段模型，Haversine + 匀速） |
| 4 | `simulation/aircraft_trajectory_locate.m` | 时间 → 航段定位（线性扫描） |
| 5 | `simulation/aircraft_trajectory_interpolate.m` | 航迹插值（单点 + 批量生成，含 `'generate'` action） |
| 6 | `simulation/radar_coverage_check.m` | 雷达威力范围判定（距离 + 方位） |
| 7 | `simulation/radar_station_true_polar.m` | 双基地真实极坐标计算（群距离 + 方位角 + 径向速度） |
| 8 | `simulation/measurement_simulator.m` | 量测仿真器（`'create'` + `'measure'` action dispatcher） |
| 9 | `simulation/generate_frame_detections.m` | 单帧点迹生成（目标检测 + 泊松虚警） |
| 10 | `simulation/bistatic_inverse_solver.m` | 双基地反解（Rg, az → r1 → 经纬度） |
| 11 | `simulation/tracker_utils.m` | 雷达内部航迹管理 + 碎片航迹拼接（`'init'/'process'/'finalize'` + `'stitch'`） |

### 第 3 层：球面几何工具（底层数学库）

| 序号 | 文件 | 说明 |
|------|------|------|
| 12 | `utils/sphere_utils_haversine_distance.m` | Haversine 球面大圆距离 |
| 13 | `utils/sphere_utils_azimuth.m` | 大圆初始方位角 |
| 14 | `utils/sphere_utils_destination_point.m` | 大圆目的地点（正算） |
| 15 | `utils/sphere_utils_interpolate_great_circle.m` | 大圆插值（按比例） |
| 16 | `utils/sphere_utils_radial_velocity.m` | 径向速度投影（v_east × sin(az) + v_north × cos(az)） |
| 17 | `utils/sphere_utils_seconds_to_datetime_str.m` | 秒数 → 日期字符串 |
| 18 | `utils/coord_systems_lla_to_ecef.m` | LLA → ECEF 坐标转换 |

### 第 4 层：UKF 滤波器 + 关联（核心算法）

| 序号 | 文件 | 说明 |
|------|------|------|
| 19 | `ukf/ukf_jichu.m` | 基础 UKF dispatcher（`'create'/'init'/'prepare'/'update'/'predict'/'measurement'`），内含 Sigma 点生成、CV 状态转移、双基地量测模型、协方差正则化等全部局部函数 |
| 20 | `ukf/ukf_zishiying.m` | 自适应 UKF dispatcher（`'create'/'init'/'update'`），委托 `ukf_jichu` 完成 Kalman 数学后施加机动检测 + 模糊自适应 Q |
| 21 | `association/nn_associate.m` | NN 点迹-航迹关联（地理预筛 120/60km + 马氏距离精筛） |
| 22 | `association/pda_weight.m` | PDA β 权重计算（门内多量测 → 加权新息向量，3D 完整处理） |
| 23 | `fusion/regularize_cov.m` | 协方差正则化（特征值分解 + 双阈值裁剪） |

### 第 5 层：航迹管理（检测 → 跟踪的桥梁）

| 序号 | 文件 | 说明 |
|------|------|------|
| 24 | `initiation/track_initiation.m` | M/N 滑窗航迹起始器（`'init'/'process'/'reset'` action dispatcher，多假设配对 + 共识评分） |
| 25 | `tracker/single_track_runner.m` | 单目标逐帧跟踪（基础版，`ukf_jichu` + `nn_associate` + `pda_weight` 流水线） |
| 26 | `tracker/single_track_runner_adaptive.m` | 单目标逐帧跟踪（机动自适应版，使用 `ukf_zishiying`） |
| 27 | `tracker/track_management.m` | JNN 全局关联 + 航迹质量状态机（`'associate'/'quality'` dispatcher） |
| 28 | `tracker/multi_track_manager.m` | 多目标跟踪管理器（批量预测 → JNN → PDA → 质量 → 新起始） |

### 第 6 层：融合与评估（结果产出）

| 序号 | 文件 | 说明 |
|------|------|------|
| 29 | `fusion/time_align_tracks.m` | 航迹级时间对齐（R2 航迹 CV 模型外推到 R1 时间网格） |
| 30 | `fusion/track_fusion_algorithms.m` | 四种融合算法合集（SCC / BC / CI / FCI，含 `ci_cost` 局部函数） |
| 31 | `fusion/run_track_fusion.m` | 融合主循环（逐帧匹配航迹对，调用指定融合算法） |
| 32 | `evaluation/evaluate_all.m` | 评估 dispatcher（`'tracking_errors'/'fusion'`，含 UKF 跟踪误差 + 融合误差评估） |

### 第 7 层：可视化（图表生成）

| 序号 | 文件 | 说明 |
|------|------|------|
| 33 | `visualization/plot_scene_overview.m` | 场景总览（真值航迹 + 雷达位置 + 覆盖范围） |
| 34 | `visualization/plot_point_cloud_3d.m` | 3D 点云图（距离 × 方位 × 径向速度） |
| 35 | `visualization/plot_results.m` | 直线航迹结果调度器（7 种模式：single_track / single_fusion / combined_tracks / tracks_vs_truth / tracker / error_timeline / error_timeline_turn） |
| 36 | `visualization/plot_turn_spatial.m` | 拐弯空间可视化调度器（4 种模式：point_clouds / radar_compare / fusion_map / comprehensive） |
| 37 | `visualization/plot_turn_stats.m` | 拐弯统计分析调度器（4 种模式：comparison / fusion_compare / rmse_bars / single_compare） |

---

## 三、完整调用关系图

### 3.1 数据流程图

```
┌─────────────────────────────────────────────────────────────────────┐
│                      simulation_params()                             │
│                  （参数配置，被所有模块引用）                           │
└──────────────────────────────────┬──────────────────────────────────┘
                                   │
    ┌──────────────────────────────┼──────────────────────────────┐
    │                              │                              │
    ▼                              ▼                              ▼
┌───────────┐              ┌──────────────┐              ┌──────────────┐
│ trajectory │              │ ADS-B 数据   │              │  radar 站点   │
│ create     │              │ (CSV 加载)   │              │  (经纬度)     │
└─────┬─────┘              └──────┬───────┘              └──────────────┘
      │                           │
      ▼                           ▼
┌───────────┐   ┌─────────────────────────────────────┐
│generate   │   │  Phase 1: ADS-B 标定                  │
│_frame_    │   │  计算 dr1_est, da1_est, dr2_est,     │
│detections │   │  da2_est                             │
└─────┬─────┘   └─────────────────────────────────────┘
      │
      ▼
┌───────────┐   ┌─────────────────────────────────────┐
│bistatic_  │   │  Phase 4: 偏差校正                    │
│inverse_   │   │  prange - dr_est → drange            │
│solver     │   │  paz - da_est → daz                  │
└───────────┘   │  drange + daz → bistatic_inverse     │
                └─────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │                   Phase 5: 航迹跟踪                        │
    │                                                          │
    │  ┌─────────────────────────────────────────────────┐     │
    │  │ single_track_runner(_adaptive).m                 │     │
    │  │   ├─ track_initiation('process', ...)            │     │
    │  │   │    M/N 滑窗 → 多假设配对 → 共识评分          │     │
    │  │   ├─ ukf_jichu('init', ...)     (两点差分初始化) │     │
    │  │   ├─ ukf_jichu('prepare', ...)  (预测+量测统计)  │     │
    │  │   ├─ nn_associate(...)          (NN 关联)        │     │
    │  │   ├─ pda_weight(...)            (PDA β 加权)     │     │
    │  │   └─ ukf_jichu('update', ...)   (Kalman 更新)    │     │
    │  │   或 ukf_zishiying('update', ...) (自适应更新)   │     │
    │  └─────────────────────────────────────────────────┘     │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 6: 航迹级时间对齐                       │
    │  time_align_tracks (R2 航迹 CV 模型回退到 R1 时间网格)     │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 7: 航迹融合                             │
    │  run_track_fusion                                        │
    │    └─ track_fusion_algorithms (调度 fuse_scc/fuse_bc/   │
    │        fuse_ci/fuse_fci)                                 │
    │    └─ regularize_cov (融合前后协方差正则化)                │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 8: 误差评估                             │
    │  evaluate_all('tracking_errors', ...)  (单站 UKF 误差)    │
    │  evaluate_all('fusion', ...)           (融合 vs 真值)     │
    └──────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────┐
    │              Phase 9: 可视化                               │
    │  plot_scene_overview / plot_point_cloud_3d /              │
    │  plot_results('single_track', ...) /                      │
    │  plot_results('single_fusion', ...) /                     │
    │  plot_turn_spatial(...) / plot_turn_stats(...)            │
    └──────────────────────────────────────────────────────────┘
```

### 3.2 UKF 模块内部调用链（v2.0 精简后）

```
ukf_jichu.m  (action dispatcher，6 个公共 action)
  │
  ├── 'create' → create_ukf()
  │     设置 n=4, m=3, lam, Wm/Wc, R, Q, P, R_EARTH
  │
  ├── 'init' → init_ukf()
  │     ├── meas_to_latlon_ukf()  (双基地反解 + 球面正算)
  │     └── 两点差分速度估计 + 速度合理性检验 (10-2000 m/s)
  │
  ├── 'prepare' → prepare_ukf()
  │     ├── predict_step_ukf()
  │     │     ├── sigma_points_ukf()  (Cholesky 分解，2n+1 个 Sigma 点)
  │     │     └── state_transition_ukf()  (CV 模型 F 矩阵)
  │     ├── measurement_ukf()  (双基地群距离 + 方位角 + 径向速度)
  │     └── P_zz 计算 (Sigma 点量测传播 + 加权协方差)
  │
  ├── 'update' → update_with_innov()
  │     ├── P_xz 互协方差计算
  │     ├── K = P_xz / P_zz  (卡尔曼增益)
  │     ├── x = x_pred + K * innov  (状态更新)
  │     ├── P = P_pred - K * P_zz * K'  (协方差更新)
  │     └── regularize_cov_ukf()  (特征值裁剪)
  │
  ├── 'predict' → predict_step_ukf()  (仅预测，不含量测)
  │
  └── 'measurement' → measurement_ukf()  (h(x) 非线性量测函数)


ukf_zishiying.m  (action dispatcher，3 个公共 action)
  │
  ├── 'create' → ukf_jichu('create', ...)  (委托基础 UKF)
  ├── 'init'   → ukf_jichu('init', ...)    (委托基础 UKF)
  └── 'update' → ukf_jichu('update', ...)  (委托基础 UKF)
        └── apply_maneuver_adapt_post()
              ├── 机动检测: 短时(3帧) vs 长时 NIS 趋势比较
              │    判定: nis_short > nis_long × 1.25 && nis_short > 2.8
              ├── 机动预检测: 渐进放宽波门 (suspect_counter)
              ├── 模糊自适应 Q: 5 级隶属度(VS/S/M/L/VL) → Sugeno 解模糊
              │    Q 因子 [0.6, 1.0, 1.8, 3.0]
              ├── 机动 Q 渐进提升: 1.5 → 2.3 → 3.1 → 3.5
              └── EMA 平滑 (η=0.20)
```

### 3.3 Tracker 模块调用链

```
single_track_runner.m  (基础版)
  │
  ├── track_initiation('init', params)     ← 初始化 M/N 滑窗
  ├── track_initiation('process', ...)     ← 逐帧 M/N 起始尝试
  ├── ukf_jichu('init', ukf_tpl, det1, det2)  ← 两点差分初始化
  ├── ukf_jichu('prepare', ukf)           ← 预测 + 量测统计
  ├── nn_associate(x_pred, z_pred, P_zz_2d, dets, params, life)  ← NN 关联
  ├── pda_weight(dets_in_gate, z_pred, P_zz, params)  ← PDA β 加权
  └── ukf_jichu('update', ukf, innov, ...)  ← Kalman 状态/协方差更新

single_track_runner_adaptive.m  (机动自适应版)
  │
  ├── [同上起始和 prepare 流程]
  ├── nn_associate(...)                   ← NN 关联
  ├── pda_weight(...)                     ← PDA β 加权
  └── ukf_zishiying('update', ukf, innov, ..., params)
        └── ukf_jichu('update', ...) + apply_maneuver_adapt_post()

multi_track_manager.m  (多目标版)
  │
  ├── ukf_jichu('prepare', ...)           ← 批量 UKF 预测
  ├── track_management('associate', ...)  ← JNN 全局最近邻关联
  ├── pda_weight(...) / ukf_jichu('update', ...)  ← PDA 更新已关联航迹
  ├── track_management('quality', ...)    ← 航迹质量状态机
  └── track_initiation('process', ...)    ← 剩余点迹新起始
```

### 3.4 Fusion 模块调用链

```
run_track_fusion.m
  │
  ├── time_align_tracks()               ← 时间对齐（外部先调用）
  └── track_fusion_algorithms()         ← 调度四种融合算法
        ├── fuse_scc()                   ← 简单凸组合（P⁻¹ = P₁⁻¹ + P₂⁻¹）
        ├── fuse_bc()                    ← Bar-Shalom-Campo（考虑互协方差 P₁₂）
        ├── fuse_ci()                    ← Covariance Intersection（fminbnd 优化 w）
        └── fuse_fci()                   ← Fast CI（tr(P)⁻¹ 闭式解 w）
        └── regularize_cov()             ← 融合后协方差正则化
```

---

## 四、各文件详细说明

### 4.1 根目录文件

#### `run_simulation.m` — 直线航迹仿真主程序

- **功能**：双基地雷达单目标直线飞行仿真的完整入口
- **9 个 Phase**：
  - Phase 0: 场景初始化（`aircraft_trajectory_create` + `radar_coverage_check` + 时间网格）
  - Phase 1: ADS-B 系统偏差标定（统计 `dr_est`, `da_est`）
  - Phase 2: 原始点迹生成（`generate_frame_detections`，含偏差含噪声）
  - Phase 3: 声明时间对齐策略（R1: 0s/30s/60s，R2: 13s/43s/73s）
  - Phase 4: 偏差校正 + `bistatic_inverse_solver` 解算经纬度
  - Phase 5: `single_track_runner` 执行 UKF+PDA 跟踪
  - Phase 6: `time_align_tracks` R2 → R1 时间对齐
  - Phase 7: `run_track_fusion` 四种算法融合
  - Phase 8: `evaluate_all` 误差评估
  - Phase 9: 可视化绘图 + 数据保存

#### `run_simulation_turn.m` — 拐弯航迹仿真主程序

- **功能**：对比基础 UKF 与机动自适应 UKF 在拐弯场景的表现
- **特点**：同时运行 `single_track_runner`（基础版）和 `single_track_runner_adaptive`（机动自适应版），两组结果分别融合（共 8 种融合结果：4 算法 × 2 跟踪器）

### 4.2 config/ 目录

#### `simulation_params.m` — 仿真参数配置

- **功能**：集中管理所有仿真可调参数，返回一个结构体，是仿真的唯一配置入口
- **包含 13 组参数**：
  1. 时间设定（dt=30s, duration=3600s, 时间偏移量 R1=0s / R2=13s）
  2. 站点几何（R1 Rx: 113°E/33.5°N, R2 Rx: 115°E/33°N, Tx: 109°E/33.5°N / 111°E/33°N）
  3. 威力覆盖（波束中心 92°/91°，宽度 15°，距离 1000-2000 km）
  4. 目标航迹（waypoints: (127.5,31) → (130.5,33)，航速 230 m/s，straight 模式）
  5. 量测噪声（R1 精度站: σ_range=7km / σ_az=0.35°；R2 标准站: σ_range=14km / σ_az=0.6°）
  6. 系统偏差（R1: +20km / -3.0°；R2: -15km / +3.5°，符号相反以利互校正）
  7. UKF 滤波参数（α=1e-3, β=2, κ=0, Q_scale=2e4, P_pos_std=0.2°, P_vel_std=0.003°/s）
  8. 航迹管理（M/N=4/8, K_loss=15, gate_sigma=2.0）
  9. 检测/虚警（P_d=0.6, P_fa=0.001, Δr=10km, Δaz=1°）
  10. PDA 加权（P_G=0.8647, clutter_intensity）
  11. 模糊自适应 Q（窗口 8 帧, Q 因子 [0.6, 1.5]）
  12. ADS-B 标定数据路径
  13. 随机种子（42）

### 4.3 simulation/ 目录 — 场景仿真模块

| 文件 | 功能 | 关键算法 |
|------|------|----------|
| `aircraft_trajectory_create.m` | 航迹结构体创建 | Haversine 距离 + 匀速运动 → 航段 `lon_rate`/`lat_rate` |
| `aircraft_trajectory_locate.m` | 时间 → 航段定位 | 线性扫描累积时长，确定所属航段和段内偏移 |
| `aircraft_trajectory_interpolate.m` | 航迹插值 | `'single'` 单点插值 + `'generate'` 批量生成，线性插值 `lon + lon_rate × t_seg` |
| `radar_coverage_check.m` | 威力范围判定 | r1 ∈ [1000,2000] km 且 \|az - beam_center\| ≤ beam_width/2 |
| `radar_station_true_polar.m` | 双基地真实极坐标 | 群距离 r0+r1 + 球面方位角 + 双基地径向速度（Tx+Rx 双向投影） |
| `bistatic_inverse_solver.m` | 双基地几何反解 | r1 = 0.5(Rg²-d²)/(Rg-d·cos(φ)) → 球面正算经纬度 |
| `measurement_simulator.m` | 量测仿真器 | `'create'` 封装雷达定义 + 噪声参数 + 随机种子；`'measure'` 执行 P_d 判定 + 噪声注入 + 反解经纬度 |
| `generate_frame_detections.m` | 单帧点迹生成 | 覆盖率检查 → 检测概率（P_d=0.6）→ 高斯噪声 → 泊松虚警 |
| `tracker_utils.m` | 雷达内部跟踪 + 拼接 | `'init'/'process'/'finalize'` 三状态 M/N+K_loss 状态机；`'stitch'` 碎片择优合并 + 短间隙大圆插值 |

### 4.4 ukf/ 目录 — UKF 滤波器核心（2 文件）

#### `ukf_jichu.m` — 基础 UKF dispatcher

- **功能**：纯滤波数学，不含任何关联逻辑。采用 action dispatcher 模式。
- **状态空间**：4 维 `[lon; lon_dot; lat; lat_dot]`（地理坐标）
- **量测空间**：3 维 `[bistatic_range; azimuth; radial_velocity]`
- **6 个 action**：
  - `'create'` — 创建 UKF 模板（n, m, lam, Wm/Wc, R, Q, P, R_EARTH）
  - `'init'` — 单点/两点初始化（双基地反解 + 差分速度估计 + 合理性检验）
  - `'prepare'` — 预测 + 量测统计（Sigma 点传播 → x_pred/P_pred/z_pred/Z_pred/P_zz），供上层关联模块使用
  - `'update'` — 纯 Kalman 数学（P_xz → K → x_new → P_new → 协方差正则化）
  - `'predict'` — 仅预测步（无量测更新，用于纯预测帧）
  - `'measurement'` — 双基地非线性量测函数 h(x)
- **内置局部函数**（原独立文件，现全部内联）：
  - `sigma_points_ukf` — Cholesky 分解，2n+1=9 个 Sigma 点
  - `state_transition_ukf` — CV 匀速模型 F 矩阵（块对角）
  - `measurement_ukf` — 双基地群距离 + 方位角 + 径向速度
  - `meas_to_latlon_ukf` — 极坐标 → 经纬度球面反算
  - `regularize_cov_ukf` — 协方差正则化（特征值双阈值裁剪）
  - `haversine_ukf` — 球面大圆距离

#### `ukf_zishiying.m` — 自适应 UKF dispatcher

- **功能**：封装 `ukf_jichu` + 机动自适应 Q。`'update'` 动作先委托 `ukf_jichu('update', ...)` 完成纯 Kalman 数学，再施加机动检测和模糊自适应 Q。
- **3 个 action**：`'create'`, `'init'`, `'update'`
- **自适应机制**（`apply_maneuver_adapt_post`）：
  - 机动检测：短时（3 帧）vs 长时 NIS 趋势比较
  - 机动预检测：渐进放宽波门（`suspect_counter`）
  - 模糊自适应 Q：5 级隶属度（VS/S/M/L/VL）→ Sugeno 解模糊
  - 机动 Q 渐进提升：1.5 → 2.3 → 3.1 → 3.5
  - EMA 平滑（η=0.20）

### 4.5 association/ 目录 — 点迹-航迹关联

#### `nn_associate.m` — 最近邻关联

- **功能**：两步筛选（地理距离预筛 + 马氏距离精筛），返回最佳点迹和波门内所有点迹
- **地理波门**：初始 120 km，UKF 收敛后（life>15）缩小到 60 km
- **马氏门限**：`gate_sigma² × 2`（2D 卡方）
- **输出**：`best_det`（最小马氏距离点迹）+ `dets_in_gate`（波门内所有点迹 cell 数组）

#### `pda_weight.m` — PDA β 权重计算

- **功能**：根据波门内点迹集计算关联概率 βᵢ，构造 3D 加权新息向量
- **纯数学**：不含任何 UKF K/P 更新，结果传给 `ukf_jichu('update', ...)`
- **算法**：`eᵢ = exp(-0.5 × mahal_2d(i))` → `βᵢ = eᵢ / (b + Σe)` → `innov_weighted = Σβᵢ × innov_3d(i)`
- **退化处理**：单量测直接返回简单新息；P_zz 非 3×3 时兜底

### 4.6 initiation/ 目录 — 航迹起始

#### `track_initiation.m` — M/N 滑窗航迹起始器

- **功能**：纯过程化 M/N 滑窗起始器，action dispatcher 模式（`'init'/'process'/'reset'`）
- **算法**：
  1. 维护长度为 N 的滑窗，记录每帧点迹
  2. 当滑窗内 ≥ M 帧有点迹且当前帧有点迹时，触发起始尝试
  3. 多假设配对：当前帧 × 历史帧所有点迹对
  4. Haversine 距离 → 速度检验（30-600 m/s）
  5. 共识评分：其他含检测帧中点迹是否靠近配对轨迹（80 km 门限）
  6. 选择最高评分对，评分 ≥ 1 即成功

### 4.7 tracker/ 目录 — 航迹管理

| 文件 | 功能 | 核心逻辑 |
|------|------|----------|
| `single_track_runner.m` | 单目标跟踪（基础版） | M/N 起始 → prepare → NN 关联 → PDA 加权 → `ukf_jichu.update`，三状态 INITIATING/TRACKING/LOST |
| `single_track_runner_adaptive.m` | 单目标跟踪（机动自适应版） | 同上框架，但使用 `ukf_zishiying('update', ...)`，含机动预扫描和自适应波门 |
| `track_management.m` | JNN 关联 + 质量状态机 | `'associate'` 贪心全局最近邻（代价矩阵 → 迭代选最小 → 移除行列 → 1 对 1）；`'quality'` 对称计分（±1）+ TYPE 1/2/6/7 状态机 |
| `multi_track_manager.m` | 多目标跟踪管理器 | 单帧分发引擎：分离活跃/历史航迹 → 批量 UKF 预测 → JNN 关联 → PDA 更新 → 质量状态机 → M/N 新起始 |

#### 航迹质量状态机

```
TEMPORARY(6) ──── quality ≥ 10 ────► RELIABLE(1)
                                           │
                        quality < 8         │
                          ◄─────────────────┘
                                           │
                      quality ≥ 10          │
                          ─────────────────►
                                           │
RELIABLE / MAINTAIN ── quality < 3 ──► HISTORY(7) (死亡)
```

### 4.8 fusion/ 目录 — 航迹融合算法

| 文件 | 功能 | 关键公式 |
|------|------|----------|
| `time_align_tracks.m` | 航迹级时间对齐 | R2 航迹用 CV 模型外推到 R1 时间网格：`x(t-Δt) = F(-Δt)·x(t)`, `P(t-Δt) = F·P·F' + Q(|Δt|)` |
| `track_fusion_algorithms.m` | 四种融合算法合集 | SCC / BC / CI / FCI，通过调度函数统一调用 |
| `run_track_fusion.m` | 融合主循环 | 逐帧遍历匹配航迹对，调用指定融合算法，支持单源降级 |
| `regularize_cov.m` | 协方差正则化 | 特征值分解 + 双阈值裁剪（绝对 1e-12 + 相对 1e-6 × max_λ） |

#### 四种融合算法对比

| 算法 | 互协方差需求 | 权重计算 | 特点 |
|------|-------------|----------|------|
| SCC | 不需要 | w=0.5（等效） | 最简单，假设完全独立，P⁻¹ = P₁⁻¹ + P₂⁻¹ |
| BC | 需要维护 P₁₂ | 动态计算 | 最精确（如果 P₁₂ 准确），x = x₁ + (P₁-P₁₂)S⁻¹(x₂-x₁) |
| CI | 不需要 | fminbnd 优化 | 保守融合，不低估不确定性，min det(P_fused) |
| FCI | 不需要 | tr(P)⁻¹ 闭式解 | CI 的快速近似，w = tr(P₁)⁻¹/(tr(P₁)⁻¹+tr(P₂)⁻¹) |

### 4.9 utils/ 目录 — 球面几何工具

| 文件 | 功能 | 关键公式 |
|------|------|----------|
| `sphere_utils_haversine_distance.m` | 球面大圆距离 | a = sin²(Δlat/2) + cos(lat₁)cos(lat₂)sin²(Δlon/2), c = 2·atan2(√a, √(1-a)) |
| `sphere_utils_azimuth.m` | 大圆初始方位角 | az = atan2(sin(Δlon)cos(lat₂), cos(lat₁)sin(lat₂) - sin(lat₁)cos(lat₂)cos(Δlon)) |
| `sphere_utils_destination_point.m` | 大圆目的地点（正算） | 给定起点 + 距离 + 方位角 → 目标经纬度 |
| `sphere_utils_interpolate_great_circle.m` | 大圆插值 | distance + azimuth → destination 走 fraction × dist |
| `sphere_utils_radial_velocity.m` | 径向速度投影 | v_east × sin(az) + v_north × cos(az) |
| `sphere_utils_seconds_to_datetime_str.m` | 时间格式化 | t = ref_time + seconds(secs) → datestr |
| `coord_systems_lla_to_ecef.m` | LLA → ECEF | N = A/√(1-E2·sin²(lat)), x = (N+alt)cos(lat)cos(lon) |

### 4.10 evaluation/ 目录 — 误差评估

#### `evaluate_all.m` — 评估 dispatcher

- **功能**：合并原 `compute_tracking_errors.m` 和 `evaluate_fusion.m`，通过 action 字符串分发
- **`'tracking_errors'`** — UKF 跟踪误差计算（逐帧 UKF / 校准点迹 / 原始点迹 vs 真值，统计中位数/均值/标准差/RMSE/95th 百分位）
- **`'fusion'`** — 融合误差评估（匹配对 → 真值飞机映射 → 逐帧融合误差 → 单站误差 → 汇总对比表）

### 4.11 registration/ 目录 — 空间/时间配准

| 文件 | 功能 |
|------|------|
| `align_radar_to_grid.m` | 球面大圆插值时间对齐 |
| `cost_fcn_with_params.m` | 空间配准 EML 代价函数 |
| `estimate_biases.m` | 空间偏差估计（最大似然） |

### 4.12 io/ 目录 — 数据 I/O

| 文件 | 功能 |
|------|------|
| `load_adsb.m` | 加载 ADS-B CSV 数据，提取航迹并重采样到仿真网格 |
| `save_all.m` | 批量保存 CSV + MAT + JSON 格式结果 |
| `extract_measurement_field.m` | 从量测 cell 数组提取指定字段 |

### 4.13 visualization/ 目录 — 可视化

#### 独立文件

| 文件 | 功能 |
|------|------|
| `plot_scene_overview.m` | 场景总览（真值航迹 + 雷达位置 + 覆盖范围） |
| `plot_point_cloud_3d.m` | 3D 点云图（距离 × 方位 × 径向速度） |

#### `plot_results.m` — 直线航迹结果调度器（7 种模式）

| mode | 功能 | 来源原文件 |
|------|------|-----------|
| `'single_track'` | 单目标跟踪综合图（地图 + 图层控制） | plot_single_track_result.m |
| `'single_fusion'` | 融合结果综合图（地图 + 误差收敛 + CDF） | plot_single_fusion_result.m |
| `'combined_tracks'` | 多航迹并排对比（原始/校准/UKF 滤波） | plot_combined_tracks.m |
| `'tracks_vs_truth'` | UKF 航迹 vs 真值对比（R1/R2 分列） | plot_tracks_vs_truth.m |
| `'tracker'` | 跟踪器碎片化与拼接交互式图（9 个图层控制按钮） | plot_tracker_result.m |
| `'error_timeline'` | 误差时间序列 + 检测/关联事件图 | plot_error_timeline.m |
| `'error_timeline_turn'` | 拐弯误差时间线对比（基础 vs 自适应） | plot_error_timeline_turn.m |

#### `plot_turn_spatial.m` — 拐弯空间可视化调度器（4 种模式）

| mode | 功能 | 来源原文件 |
|------|------|-----------|
| `'point_clouds'` | R1/R2 点云 + 基础 UKF（虚线）+ 自适应 UKF（实线） | plot_turn_point_clouds.m |
| `'radar_compare'` | 单站对比（地图 + 拐弯放大 + 误差时间线 + RMSE 柱状图） | plot_turn_radar_compare.m |
| `'fusion_map'` | 融合地图对比（基础 + 自适应 + 拐弯放大 + 信息面板） | plot_turn_fusion_map.m |
| `'comprehensive'` | 全图层综合对比（原始量测 → 校准 → UKF → 融合，含图层控制） | plot_turn_comprehensive.m |

#### `plot_turn_stats.m` — 拐弯统计分析调度器（4 种模式）

| mode | 功能 | 来源原文件 |
|------|------|-----------|
| `'comparison'` | 轨迹对比图（真值 + 基础 + 自适应 + 融合，含图层控制） | plot_turn_comparison.m |
| `'fusion_compare'` | 融合对比综合图（6 子图：全图 + 放大 + RMSE 柱状图 + 误差 + 精度链 + 汇总） | plot_turn_fusion_compare.m |
| `'rmse_bars'` | RMSE 柱状图总览（基础灰 vs 自适应绿 + 数值汇总面板） | plot_turn_rmse_bars.m |
| `'single_compare'` | 单站对比综合图（6 子图：R1/R2 全图 + 放大 + 误差 + RMSE 柱状图） | plot_turn_single_compare.m |

---

## 五、关键算法说明

### 5.1 UKF（无迹卡尔曼滤波）

**状态空间**：4 维 `[lon; lon_dot; lat; lat_dot]`
**量测空间**：3 维 `[bistatic_range; azimuth; radial_velocity]`

**每帧滤波循环（模块化流水线）**：
1. **prepare**（`ukf_jichu.prepare`）：Sigma 点生成 → CV 模型传播 → x_pred/P_pred → Z_pred → z_pred/P_zz
2. **关联**（`nn_associate`）：地理预筛 + 马氏距离 NN 关联 → best_det + dets_in_gate
3. **PDA 加权**（`pda_weight`）：βᵢ 关联概率 → 加权新息向量 innov_weighted
4. **更新**（`ukf_jichu.update`）：P_xz → K → x_new → P_new → 协方差正则化
5. **自适应**（`ukf_zishiying.update` 额外步骤）：NIS 记录 → 机动检测 → 模糊自适应 Q

### 5.2 航迹起始（M/N 滑窗 + 共识评分）

1. 维护长度为 N 的滑窗，记录每帧点迹
2. 当滑窗内 ≥ M 帧有点迹时，触发起始尝试
3. 多假设配对：当前帧 × 历史帧所有点迹对，Haversine 距离 → 速度检验（30-600 m/s）
4. 共识评分：其他含检测帧中点迹是否靠近配对轨迹（80 km 门限）
5. 选择最高评分对，评分 ≥ 1 即成功，两点差分初始化 UKF

### 5.3 机动检测策略（`ukf_zishiying`）

**模糊自适应 Q**：
- 基于 NIS 滑动平均值（默认 3 帧短窗口）
- 5 级模糊隶属度（VS/S/M/L/VL）→ Sugeno 解模糊
- Q 输出等级：Decrease(0.6) / SlightDecrease(0.8) / Maintain(1.0) / Increase(1.8) / RapidIncrease(3.0)

**机动检测**：
- 短时（3 帧）vs 长时 NIS 趋势比较
- 判定条件：`nis_short > nis_long × 1.25 && nis_short > 2.8` 或 `nis_long > 3.2`
- Q 渐进提升：1.5 → 2.3 → 3.1 → 3.5（而非瞬间跳跃）
- 恢复：连续 4 帧正常 → 结束机动

**机动预检测**：
- 非机动状态下，用 1.8× 宽波门扫描备选量测
- suspect_counter ≥ 2 → 提前激活机动状态

### 5.4 航迹融合算法对比

| 算法 | 互协方差 | 权重 | 特点 |
|------|---------|------|------|
| SCC | 不需要 | w=0.5（等效） | 最简单，假设完全独立 |
| BC | 需维护 P₁₂ | 动态计算 | 最精确（若 P₁₂ 准确） |
| CI | 不需要 | fminbnd 优化 | 保守融合，不低估不确定性 |
| FCI | 不需要 | tr(P)⁻¹ 闭式解 | CI 的快速近似，无需迭代 |

---

## 六、配置文件与数据说明

### 仿真参数默认值

- 时间步长：30 s
- 仿真总时长：3600 s（1 小时，120 帧）
- 检测概率：60%（OTH-SWR 典型值，电离层衰落损耗大）
- 虚警率：0.1%（每分辨单元）
- 雷达距离：1000-2000 km（一跳天波传播）
- 波束宽度：15°
- R1（精度站）噪声：σ_range=7km / σ_az=0.35°
- R2（标准站）噪声：σ_range=14km / σ_az=0.6°
- 航速：230 m/s（~828 km/h，民航巡航速度）
- M/N 起始：4/8
- K_loss 终止：15（450 秒 = 7.5 分钟）

### ADS-B 数据格式

CSV 文件，19 列，无表头，包含 ICAO 地址、经纬度、航向、高度、地速、时间戳等。

### 结果文件格式

- `.mat`：MATLAB 原生格式，完整数据
- `.png`：可视化图表保存到 `results/` 目录

---

## 七、扩展与修改指南

1. **修改雷达站点**：编辑 `config/simulation_params.m` 中的 `radar1_*` / `radar2_*` 参数
2. **修改航迹**：修改 `aircraft_waypoints` 数组，调整 `aircraft_speed_ms`
3. **修改噪声水平**：编辑 `radar*_range_noise_std_m` 和 `radar*_azimuth_noise_std_deg`
4. **调整 UKF 参数**：修改 `ukf_alpha/beta/kappa/Q_scale/P_pos_std/P_vel_std`
5. **切换融合算法**：修改 `run_simulation.m` 中的 `method_names` 列表
6. **增加新融合算法**：在 `fusion/track_fusion_algorithms.m` 添加新函数，加入 `run_track_fusion.m` 的调度分支
7. **换用其他运动模型**：修改 `ukf_jichu.m` 中的 `state_transition_ukf` 局部函数的 F 矩阵（如改用 CTRV 模型）
8. **调整机动检测灵敏度**：修改 `ukf_zishiying.m` 中 `apply_maneuver_adapt_post` 的检测阈值和 Q 提升曲线
9. **添加新的可视化模式**：在 `plot_results.m` / `plot_turn_spatial.m` / `plot_turn_stats.m` 中添加新的 mode case 和对应子函数
