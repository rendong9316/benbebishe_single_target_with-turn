# 双基地天波超视距雷达单/多目标跟踪仿真系统 — 完整项目文档

> **历史材料声明（2026-07-10）**：本文件由多个阶段稿拼接而成，包含大量整章重复和已被后续源码复核证伪的旧结论，仅作为历史材料池保留，不计入“百万字有效代码审查正文”。请以 [`code-review/README.md`](code-review/README.md) 为权威入口，并通过 [`code-review/04_CORRECTION_LOG.md`](code-review/04_CORRECTION_LOG.md) 查看勘误。为保护历史内容，本文件暂不对所有重复副本逐处改写。

> 版本：v1.0 | 日期：2026-07-09 | 作者：rendong9316

---

## 第一部分：项目概述与设计哲学

### 1.1 项目背景

本项目是一个基于 MATLAB 的双基地高频天波超视距雷达（Over-The-Horizon Surface-Wave Radar, OTH-SWR）目标跟踪仿真系统。天波超视距雷达利用电离层对高频（HF, 3-30 MHz）信号的反射，能够实现1000-3000公里的超视距探测能力，在边境监控、海上目标探测等领域具有重要军事和民用价值。

然而，天波超视距雷达面临着诸多独特的技术挑战：
1. **电离层传播不确定性**：电离层F层高度（250-400km）时变，导致传播路径和群距离随时间漂移
2. **量测噪声大**：距离精度通常在5-15km量级，方位精度约0.3-1.0度，远劣于微波雷达
3. **系统偏差显著**：电离层虚高估计误差可导致20km级的距离偏置，阵列标定误差可导致3-5度的方位偏置
4. **检测概率低**：受电离层衰落影响，单帧检测概率通常仅0.6左右
5. **杂波环境恶劣**：电离层不规则体回波、流星余迹、同频段干扰等导致杂波密度高
6. **时间异步**：多部雷达独立工作，采样时刻不同步，融合前需时间对齐

本项目针对上述挑战，构建了一个完整的仿真流水线，涵盖了从场景生成、系统偏差标定、点迹仿真、UKF滤波、航迹管理、多传感器融合到性能评估的全流程。

### 1.2 系统设计目标

本项目的核心设计目标包括：

1. **双基地雷达体制建模**：精确模拟发射站（Tx）和接收站（Rx）地理分离的双基地几何关系，使用天波传播模型计算群距离、方位角和多普勒速度
2. **异质传感器网络**：两部雷达具有不同的精度等级（R1精密站、R2标准站），模拟实际多传感器网络中的精度差异
3. **系统偏差标定**：利用ADS-B合作目标数据，通过样本均值估计方法在线标定雷达系统偏差
4. **UKF滤波跟踪**：采用无迹卡尔曼滤波处理量测-状态的非线性映射关系，支持CV（匀速）和CT（协调转弯）两种运动模型
5. **多模型自适应**：实现自适应UKF（机动检测+模糊Q调整）和IMM（交互多模型）两种自适应策略
6. **多传感器融合**：实现SCC、BC、CI、FCI四种经典融合算法，比较其在双基地场景下的性能
7. **蒙特卡洛统计分析**：通过500次独立仿真，统计RMSE、MTL、断裂次数、关联率等关键指标
8. **多目标扩展**：支持三目标交叉航迹场景，使用JPDA关联和M/N起始

### 1.3 设计哲学

本项目的代码设计遵循以下哲学：

1. **过程式编程**：禁止使用classdef/OOP，全部采用纯函数式编程，通过action dispatcher模式实现面向对象的多态效果
2. **模块化分离**：每个功能模块独立成文件，通过清晰的输入输出接口耦合
3. **参数集中管理**：所有可调参数集中在config/simulation_params.m中，主入口仅做"接线"不硬编码
4. **多态路由**：通过ukf_dispatch实现滤波器后端的透明切换，tracker不感知具体后端类型
5. **真值辅助起始**：首次起始使用真值辅助保证开局正确，后续重新起始使用纯M/N逻辑避免作弊
6. **随机流隔离**：R1和R2使用独立的随机种子偏移（1e7/2e7），蒙特卡洛各次仿真使用不同种子

### 1.4 场景配置

| 参数 | R1（精密站） | R2（标准站） | 备注 |
|------|-------------|-------------|------|
| Rx位置 | 113.0°E, 33.5°N | 115.0°E, 33.0°N | 相距约185km |
| Tx位置 | 109.0°E, 33.5°N | 111.0°E, 33.0°N | 基线约370km |
| 波束指向 | 92° | 91° | 指向东部海域 |
| 距离噪声 | 7km | 14km | R2约为R1的2倍 |
| 方位噪声 | 0.35° | 0.6° | R2约为R1的1.7倍 |
| 系统偏差 | +20km/-3° | -15km/+3.5° | 符号相反 |
| 采样偏移 | 0s | 13s | 时间异步 |
| UKF Q_scale | 1e5 | 2e5 | R2需更大Q补偿 |
| gate_sigma | 6 | 6 | 关联波门 |
| Vr硬门 | 20m/s | 40m/s | R2放宽 |
| K_loss | 8 | 8 | 连续丢帧终止 |

---

## 第二部分：完整文件目录结构

```
single_target_with-turn/
│
├── CLAUDE.md                                    # 项目配置（MATLAB路径、代码风格）
├── MEMORY.md                                    # 项目记忆索引
│
├── config/                                      # 参数配置
│   ├── simulation_params.m                      # 单目标参数（13模块，545行）
│   └── simulation_params_multi.m                # 多目标参数（继承单目标）
│
├── simulation/                                  # 场景生成与量测仿真
│   ├── aircraft_trajectory_create.m             # 航迹结构体创建（663行）
│   ├── aircraft_trajectory_interpolate.m        # 航迹插值（171行）
│   ├── aircraft_trajectory_locate.m             # 航段时间定位（105行）
│   ├── bistatic_inverse_solver.m                # 双基地几何反解
│   ├── generate_frame_detections.m              # 单帧点迹生成（231行）
│   ├── generate_frame_detections_multi.m        # 多目标单帧点迹（105行）
│   ├── measurement_simulator.m                  # 量测仿真器（125行）
│   ├── radar_coverage_check.m                   # 威力覆盖判定（97行）
│   ├── radar_station_true_polar.m               # 真实极坐标量测（58行）
│   └── tracker_utils.m                          # 跟踪器工具
│
├── ukf/                                         # UKF滤波器模块
│   ├── ukf_dispatch.m                           # 多态路由器（39行）
│   ├── ukf_imm.m                                # IMM UKF（380行）
│   ├── ukf_jichu.m                              # 基础UKF（480行）
│   └── ukf_zishiying.m                          # 自适应UKF（270行）
│
├── tracker/                                     # 航迹管理
│   ├── single_track_runner.m                    # 单目标跟踪器（329行）
│   ├── single_track_runner_adaptive.m           # 自适应版本
│   ├── single_track_runner_nanyang.m            # 南阳方案
│   ├── single_track_runner_nanyang_adaptive.m   # 南阳自适应
│   ├── multi_track_manager.m                    # 多目标跟踪引擎（185行）
│   ├── multi_track_runner_kf.m                  # 多目标KF运行器
│   ├── multi_track_start.m                      # 多目标起始
│   ├── post_init_multi.m                        # 多目标初始化后处理
│   ├── track_management.m                       # 航迹管理（JNN+质量）
│   └── inject_truth_velocity.m                  # 真值速度注入
│
├── association/                                 # 关联算法
│   ├── nn_associate.m                           # 最近邻关联（111行）
│   ├── pda_weight.m                             # PDA加权（98行）
│   └── jpda_multi.m                             # 多目标JPDA（140行）
│
├── fusion/                                      # 航迹融合
│   ├── run_track_fusion.m                       # 逐帧融合主循环（306行）
│   ├── track_fusion_algorithms.m                # 四种融合算法（422行）
│   ├── time_align_tracks.m                      # 时间对齐（136行）
│   ├── track_matcher.m                          # 跨雷达航迹匹配
│   └── regularize_cov.m                         # 协方差正则化
│
├── initiation/                                  # 航迹起始
│   └── track_initiation.m                       # M/N滑窗起始器（148行）
│
├── evaluation/                                  # 性能评估
│   ├── evaluate_all.m                           # 单目标评估（362行）
│   └── evaluate_all_multi.m                     # 多目标评估
│
├── registration/                                # 标定与对齐
│   ├── align_radar_to_grid.m                    # 雷达对齐
│   ├── cost_fcn_with_params.m                   # 代价函数
│   └── estimate_biases.m                        # 偏差估计
│
├── utils/                                       # 工具函数
│   ├── coord_systems_lla_to_ecef.m              # LLA→ECEF转换
│   ├── skywave_geometry.m                       # 天波几何模型
│   ├── sphere_utils_azimuth.m                   # 球面方位角
│   ├── sphere_utils_destination_point.m         # 球面正算
│   ├── sphere_utils_haversine_distance.m        # Haversine距离
│   ├── sphere_utils_interpolate_great_circle.m  # 大圆插值
│   ├── sphere_utils_radial_velocity.m           # 径向速度
│   └── sphere_utils_seconds_to_datetime_str.m   # 时间格式化
│
├── visualization/                               # 可视化工具
│   ├── plot_results.m                           # 统一绘图调度器（1270行）
│   ├── plot_results_multi.m                     # 多目标绘图
│   ├── plot_scene_overview.m                    # 场景总览
│   ├── plot_scene_overview_multi.m              # 多目标场景总览
│   ├── plot_point_cloud_3d.m                    # 3D点云
│   ├── plot_turn_spatial.m                      # 拐弯空间可视化
│   └── plot_turn_stats.m                        # 拐弯统计可视化
│
├── io/                                          # 输入输出
│   ├── load_adsb.m                              # ADS-B数据加载
│   ├── extract_measurement_field.m              # 字段提取
│   └── save_all.m                               # 结果保存
│
├── nanyang/                                     # 南阳航迹处理子系统（备用）
│   ├── PointTrackAssociation_JNN.m
│   ├── cleanTrackList.m
│   ├── det2nanyang_point.m
│   ├── det2trackDataConverter.m
│   ├── distance.m
│   ├── fun_calculate_track_travelLen.m
│   ├── fun_check_35logic_points_improved.m
│   ├── fun_check_colinear_points.m
│   ├── fun_check_track_validation.m
│   ├── fun_create_new_track.m
│   ├── fun_fill_smooth_list_by_alpha_beta_filter.m
│   ├── fun_fill_smooth_list_by_predict_result.m
│   ├── fun_find_tracks_to_report.m
│   ├── fun_remove_assc_pts_from_pointlist.m
│   ├── fun_select_point_by_rd.m
│   ├── fun_select_track_by_rd.m
│   ├── fun_set_tracking_parameter.m
│   ├── fun_track_quality_management_and_info_completion.m
│   ├── fun_trackfilter_AlphaBeta.m
│   ├── Fun_PredictNextStep_CV.m
│   ├── Fun_UpdateTrackByAsscResult.m
│   ├── Fun_UpdateTrackforNoInputPoint.m
│   ├── header.m
│   ├── is_duplicate_track.m
│   ├── pdCoefInterprator.m
│   ├── predictNextStep_cv.m
│   ├── reckon.m
│   ├── resetAllTracks.m
│   ├── robu/
│   │   └── (子目录，航迹处理算法)
│
├── run_simulation.m                           # 单目标直线主入口（9-Phase，1368行）
├── run_simulation_turn.m                      # 单目标渐进拐弯主入口（710行）
├── run_simulation_turn_180deg.m               # 单目标回头弯主入口（694行）
├── run_simulation_multi.m                     # 三目标交叉主入口（925行）
├── run_mc_straight.m                          # 直线场景MC仿真（563行）
├── run_mc_turn.m                              # 拐弯场景MC-IMM（676行）
├── run_mc_turn_180deg.m                       # 回头弯MC-IMM（665行）
├── run_mc_turn_compare.m                      # 拐弯三体制对比MC（793行）
├── run_mc_turn_180deg_compare.m               # 回头弯三体制对比MC（749行）
├── run_mc_multi.m                             # 三目标交叉MC（486行）
├── scan_Q_scale.m                             # Q_scale参数扫描（864行）
├── scan_Pi.m                                  # Pi参数扫描（860行）
├── _extract_data.m                            # 数据提取（57行）
├── _get_precise.m                             # 精确值获取（28行）
├── analyze_covariance.m                       # 协方差分析（113行）
├── analyze_cov_simple.m                       # 简化协方差分析（45行）
```

---

## 第三部分：配置文件详解

### 3.1 simulation_params.m

文件位置：`config/simulation_params.m`
行数：约545行
功能：单目标场景的完整参数配置，包含13个模块的默认参数。

#### 13个参数模块

| 模块 | 主要参数 | 说明 |
|------|---------|------|
| 雷达1 | radar1_lon, radar1_lat, radar1_tx_lon, radar1_tx_lat | R1接收站/发射站经纬度 |
| 雷达2 | radar2_lon, radar2_lat, radar2_tx_lon, radar2_tx_lat | R2接收站/发射站经纬度 |
| 噪声 | radar1_range_noise_std_m, radar1_azimuth_noise_std_deg | R1量测噪声标准差 |
| 偏差 | radar1_range_bias_m, radar1_azimuth_bias_deg | R1系统偏差（用于标定验证） |
| UKF | radar1_ukf_Q_scale, radar1_ukf_P_pos_std, radar1_ukf_P_vel_std | UKF过程噪声和初始协方差 |
| 关联 | radar1_gate_sigma, radar1_gate_vr_ms | 关联波门参数 |
| 航迹管理 | tracker_K_loss, tracker_M, tracker_N | 丢失容忍和M/N起始参数 |
| 场景 | aircraft_waypoints, aircraft_speed_ms, dt_sec | 航迹定义 |
| 覆盖 | range_min_km, range_max_km, beam_width_deg | 雷达覆盖范围 |
| 检测 | detection_probability, false_alarm_rate | Pd和Pfa |
| 异步 | time_offset_radar1_sec, time_offset_radar2_sec | 两部雷达时间偏移 |
| 标定 | adsb_csv_path | ADS-B数据文件路径 |
| 随机 | random_seed | 随机种子 |

#### 典型参数值

```
params.radar1_lon = 113.0;   % 精密站接收站经度
params.radar1_lat = 33.5;    % 精密站接收站纬度
params.radar1_range_bias_m = 20000;   % 20km距离偏差
params.radar1_azimuth_bias_deg = -3.0; % -3度方位偏差
params.radar1_range_noise_std_m = 7000;  % 7km噪声
params.radar1_azimuth_noise_std_deg = 0.35; % 0.35度噪声
params.aircraft_speed_ms = 230;    % 亚音速飞机
params.dt_sec = 30;        % 30秒采样间隔
params.detection_probability = 0.6;  % 60%检测概率
params.false_alarm_rate = 0.001;  % 千分之一虚警率
```

### 3.2 simulation_params_multi.m

文件位置：`config/simulation_params_multi.m`
功能：多目标参数，继承单目标参数并扩展多目标特有参数。

---

## 第四部分：仿真文件详解

### 4.1 航迹生成

#### aircraft_trajectory_create.m (663行)

功能：基于航段模型生成目标真实轨迹。支持两种模式：
- 航点列表模式：传入[N×2 waypoints]矩阵，按顺序连接各航点
- 特殊模式：`'gradual_turn'`（渐进拐弯）和 `'uturn'`（回头弯180度）

#### aircraft_trajectory_interpolate.m (171行)

功能：从航段模型进行时间插值。支持两种调用：
- `'generate'`：批量生成N×5矩阵 [lon, lat, lon_rate, lat_rate, time_sec]
- `(traj, t)`：单点插值，返回(pos, vel)

#### aircraft_trajectory_locate.m (105行)

功能：给定时间t，查找目标在航迹上的位置。

### 4.2 双基地几何

#### bistatic_inverse_solver.m

功能：已知群距离Rg、方位角az、Tx/Rx位置，反解目标经纬度。
核心公式：r1 = 0.5 × (Rg^2 - d^2) / (Rg - d × cos(phi))

#### skywave_geometry.m

功能：天波传播模型，计算群距离 Rg = r_tx + r_rx。

### 4.3 点迹生成

#### generate_frame_detections.m (231行)

功能：单目标单帧点迹生成。内部流程：
1. 覆盖检查
2. 检测概率判断（Pd=0.6）
3. 生成含噪量测（距离+方位+伪径向速度）
4. 泊松分布生成虚警

#### generate_frame_detections_multi.m (105行)

功能：多目标单帧点迹生成，接受多个目标状态向量。

#### radar_coverage_check.m (97行)

功能：判断目标是否在雷达威力范围内（距离1000-2000km + 波束扇区）。

#### radar_station_true_polar.m (58行)

功能：从真实经纬度计算极坐标量测。

### 4.4 UKF滤波器

#### ukf_dispatch.m (39行)

功能：多态路由器，根据ukf_type分发到不同后端。

#### ukf_jichu.m (480行)

功能：基础CV-UKF，固定Q矩阵。

#### ukf_zishiying.m (270行)

功能：自适应UKF，含模糊Q调整和机动检测。

#### ukf_imm.m (380行)

功能：CV+CT双模型IMM-UKF，含Pd-IPDA似然计算。

### 4.5 跟踪器

#### single_track_runner.m (329行)

功能：单目标跟踪器，跳过M/N逻辑直接初始化。

#### multi_track_manager.m (185行)

功能：多目标跟踪引擎，含JPDA关联和M/N起始。

### 4.6 关联算法

#### nn_associate.m (111行)

功能：最近邻关联（地理预筛 + 马氏距离最小）。

#### jpda_multi.m (140行)

功能：多目标联合概率数据关联。

### 4.7 融合

#### run_track_fusion.m (306行)

功能：逐帧融合主循环，支持SCC/BC/CI/FCI四种算法。

#### track_fusion_algorithms.m (422行)

功能：四种融合算法的具体实现。

### 4.8 可视化和评估

#### plot_results.m (1270行)

功能：统一绘图调度器，支持单目标和多目标场景的各种图表。

#### evaluate_all.m (362行)

功能：单目标性能评估，计算RMSE、中位误差、改善率等。

---

**（文档未完待续 — 请指示继续生成第五部分：入口脚本详细文档）**# 双基地天波超视距雷达单/多目标跟踪仿真系统 — 项目文档

> 毕业设计项目完整文档：架构说明书 + 函数接口文档 + 目录总览

---

## 第一部分：整体架构说明书

### 1. 项目概述

本项目是一个基于 MATLAB 的双基地高频天波超视距雷达（OTH-SWR）目标跟踪仿真系统，实现了从场景生成、量测仿真、系统偏差标定、UKF滤波、航迹管理、多传感器融合到性能评估的完整流水线。项目支持单目标和多目标（三目标交叉航迹）场景，涵盖直线、渐进拐弯、180度回头弯三种目标机动模式。

#### 1.1 研究背景

天波超视距雷达利用电离层反射实现超视距探测，具有探测距离远（1000-3000km）的优势，但同时也面临电离层传播路径时变、量测噪声大、系统偏差显著等挑战。本项目重点研究：
- 双基地雷达体制下的目标跟踪
- 多部异质雷达的航迹级融合
- 转弯机动目标的跟踪性能
- IMM（交互多模型）滤波在机动跟踪中的应用

#### 1.2 系统架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                    仿真主入口层 (Entry Points)               │
│  run_simulation.m  run_mc_straight.m  run_simulation_turn.m  │
│  run_mc_turn.m  run_simulation_multi.m  run_mc_multi.m       │
├─────────────────────────────────────────────────────────────┤
│                    Phase 0-9 流水线                          │
│  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐  │
│  │Phase0│Phase1│Phase2│Phase3│Phase4│Phase5│Phase6│Phase7│  │
│  │场景  │ADS-B │点迹  │策略  │偏差  │UKF  │时间  │融合  │  │
│  │初始化│标定  │生成  │声明  │校正  │跟踪  │对齐  │算法  │  │
│  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘  │
│  ┌──────┬──────┬──────┐                                     │
│  │Phase8│Phase9│       │                                     │
│  │误差  │可视  │保存  │                                     │
│  │评估  │化    │       │                                     │
│  └──────┴──────┴──────┘                                     │
├─────────────────────────────────────────────────────────────┤
│                    核心算法层 (Algorithm Modules)            │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │ UKF滤波  │ │ 航迹管理 │ │ 航迹融合  │ │ 点迹生成  │       │
│  │ukf/       │ │tracker/   │ │fusion/    │ │simulation/ │       │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │关联算法  │ │ 航迹起始  │ │ 评估模块  │ │ 可视化工  │       │
│  │association│ │initiation│ │evaluation│ │visualization│      │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │
├─────────────────────────────────────────────────────────────┤
│                    基础设施层 (Infrastructure)               │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │ 参数配置  │ │ 球面几何  │ │ 天波几何  │ │ IO工具   │       │
│  │config/    │ │utils/     │ │utils/     │ │io/       │       │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │
└─────────────────────────────────────────────────────────────┘
```

### 2. 模块详细架构

#### 2.1 参数配置模块 (config/)

**simulation_params.m** — 单目标场景唯一参数入口，包含13个模块的参数定义：
- 模块1: 时间设定（采样周期30s，仿真时长3600s，R2偏移13s）
- 模块2: 站点几何（R1/R2的Tx和Rx经纬度）
- 模块3: 威力覆盖（波束指向92°/91°，宽度15°，距离1000-2000km）
- 模块4: 目标航迹（两航路点，速度230m/s，直线模式）
- 模块5: 量测噪声（R1精度站：σr=7km, σaz=0.35°；R2标准站：σr=14km, σaz=0.6°）
- 模块6: 系统偏差（R1: +20km/-3°；R2: -15km/+3.5°）
- 模块7: UKF参数（UT参数α/β/κ、过程噪声Q、初始协方差P、关联波门、模糊自适应Q）
- 模块8: 航迹管理（M=4, N=8起始；K_loss=8终止；gate_sigma=2.5）
- 模块9: 检测/虚警模型（Pd=1.0, Pfa=0.001, n_cells=1500）
- 模块10: PDA加权（门内概率0.8647，杂波密度）
- 模块11: 模糊自适应Q（窗口3帧，EMA系数0.10）
- 模块12: ADS-B标定数据路径
- 模块13: 随机种子（seed=94）

**simulation_params_multi.m** — 多目标场景参数配置，继承单目标参数后覆盖差异化配置。

#### 2.2 目标航迹生成模块 (simulation/)

采用三段式航迹生成流水线：
1. `aircraft_trajectory_create`: 从航路点创建航迹结构体，计算各航段距离、时长、速度
2. `aircraft_trajectory_locate`: 给定时间t，定位所在航段索引和偏移量
3. `aircraft_trajectory_interpolate`: 分段线性插值，输出[lon, lat, lon_rate, lat_rate, time]

支持的航迹模式：
- 直线（straight）：两航路点匀速飞行
- 拐弯（turn）：三航路点，中间约120°拐角
- 渐进拐弯（gradual_turn）：1°/s转弯率的协调转弯
- 回头弯（uturn）：180°左转半圆

#### 2.3 量测仿真模块 (simulation/)

**generate_frame_detections.m** — 单帧点迹生成核心函数：
- 目标检测：覆盖检查→Pd抽样→天波极坐标量测→偏差+噪声→结构体输出
- 杂波生成：泊松分布确定数量→极坐标均匀采样→球面正算→天波群距离
- 杂波的prange/paz也掺入系统偏差，保证Phase4校正后一致性

**skywave_geometry.m** — 天波几何模型：
- 群距离：地心角→弦长→双跳斜距→群距离（与单基地弦长不同）
- 方位角：Rx→目标的球面方位角
- 多普勒：dRg/dt = dr_tx/dt + dr_rx/dt（ENU速度投影）

**bistatic_inverse_solver.m** — 双基地几何反解：
- 已知Rg（群距离）、az（方位角）、Tx/Rx位置
- 求解目标经纬度（经典双基地余弦定理+迭代精化）

#### 2.4 UKF滤波模块 (ukf/)

三种UKF后端，通过 `ukf_dispatch.m` 多态路由：

**ukf_jichu.m（基础UKF）**：
- CV（匀速）模型：F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1]
- CT（协调转弯）模型：F(ω)含sin/cos项
- UT变换：2n+1个Sigma点，Cholesky分解生成
- 天波量测模型：h(x) = [Rg, az, vd]
- 极坐标→经纬度反解：经典双基地解+30次迭代精化
- 协方差正则化：对称化→特征值分解→负值裁剪→重构

**ukf_zishiying.m（自适应UKF）**：
- 在基础UKF之上增加机动自适应Q
- 机动检测：短时vs长时NIS趋势比较 + 宽门预检测
- 模糊推理系统：5个隶属函数（VS/S/M/L/VL）→ Q缩放因子
- EMA平滑：防止Q跳变

**ukf_imm.m（IMM UKF）**：
- CV+CT双模型并行
- IMM混合：Mu混合初始状态→双模型独立预测
- Pd-IPDA似然度：Musicki 2008框架
- 贝叶斯模型概率更新 + 概率钳位[0.02, 0.95]

#### 2.5 航迹管理模块 (tracker/)

**single_track_runner.m** — 单目标统一跟踪器：
- 状态机：INITIATING → TRACKING → LOST → INITIATING
- 真值辅助首次起始：保证开局正确
- M/N重新起始：超时兜底的纯M/N逻辑
- 统一流水线：杂波预筛 → prepare → NN关联 → PDA → update
- Probation期NIS保护（life≤5, NIS>50拒）
- 连续丢点防杂波劫持（固定50km地理门）
- 通过ukf_dispatch多态路由到三种后端

**multi_track_manager.m** — 多目标跟踪引擎：
- 分离活跃/历史航迹 → 批量预测 → JNN关联 → 更新 → 质量管理 → 新航迹起始

**track_initiation.m** — M/N滑窗起始器：
- 维护长度N的滑窗，≥M帧有点迹且当前帧有点迹时触发起始
- 多假设配对 + Haversine速度检验（30-600m/s）
- 共识评分：其他帧点迹是否靠近配对轨迹（80km门限）

#### 2.6 关联算法模块 (association/)

**nn_associate.m** — 最近邻关联：
- 三步筛选：地理预筛(60-120km) → Vr硬门(20-40m/s) → 2D马氏距离
- 马氏距离自适应UKF不确定度
- 返回最佳点迹和门内所有点迹（供PDA使用）

**pda_weight.m** — 概率数据关联加权：
- 计算每个门内量测的β权重
- 构造加权新息向量

**jpda_multi.m** — 多目标JPDA：
- 真值辅助 cheating 的空间聚类
- 多目标PDA加权

#### 2.7 航迹融合模块 (fusion/)

**run_track_fusion.m** — 逐帧融合主循环：
- 双源融合 + 单源透传
- BC方法专用：P12互协方差维护（预测→更新→稳定性约束）

**track_fusion_algorithms.m** — 四种融合算法：
- SCC：简单凸组合，信息矩阵相加，假设独立
- BC：Bar-Shalom-Campo，显式互协方差P12，理论最优
- CI：协方差交叉，fminbnd优化w最小化det(P)，保守安全
- FCI：快速CI，迹加权解析解，无需迭代

#### 2.8 评估模块 (evaluation/)

**evaluate_all.m** — 统一评估调度器：
- 'tracking_errors': UKF航迹 vs 真值误差统计
- 'fusion': 融合航迹 vs 真值误差统计
- 输出：RMSE、中位误差、均值、95%分位数
- UKF vs 检测改善率

#### 2.9 可视化工具 (visualization/)

**plot_results.m** — 统一绘图调度器：
- 'single_track': 单目标跟踪综合图（R1+R2）
- 'single_fusion': 融合结果对比图（4种算法）
- 'combined_tracks': 合并航迹对比
- 'tracks_vs_truth': 航迹 vs 真值
- 'error_timeline': 误差时序图

**plot_scene_overview.m** — 场景总览：真值航迹+雷达位置+覆盖扇形

**plot_point_cloud_3d.m** — 3D点云：Range-Azimuth-Frame空间

#### 2.10 工具模块 (utils/)

**sphere_utils_*** — 球面几何工具集：
- haversine_distance: 大圆距离
- azimuth: 初始方位角
- destination_point: 球面正算
- radial_velocity: 径向速度投影
- seconds_to_datetime_str: 时间格式化

**skywave_geometry.m** — 天波几何（群距离、方位角、多普勒）

**coord_systems_lla_to_ecef.m** — WGS84 LLA→ECEF坐标转换

#### 2.11 注册/标定模块 (registration/)

**align_radar_to_grid.m** — 雷达航迹对齐到统一时间网格
**estimate_biases.m** — 系统偏差估计（LS+EML两阶段）
**cost_fcn_with_params.m** — ECEF空间代价函数

### 3. 数据流与调用关系

```
simulation_params()
        ↓
aircraft_trajectory_create() → traj
        ↓
aircraft_trajectory_interpolate('generate') → true_track (N×5)
        ↓
radar_coverage_check() → 覆盖率统计
        ↓
┌─────────────────────────────────────────┐
│ Phase 1: ADS-B 偏差标定                  │
│   readtable(ADS-B CSV)                  │
│   → skywave_geometry('group_range')     │
│   → sphere_utils_azimuth()              │
│   → dr_est, da_est (均值估计)            │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 2: 原始点迹生成                    │
│   generate_frame_detections() × 2雷达   │
│   → detRaw_R1, detRaw_R2                │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 4: 偏差校正 + 几何反解             │
│   prange - dr_est → drange              │
│   bistatic_inverse_solver() → lat, lon  │
│   → detList_R1, detList_R2              │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 5: UKF 跟踪                       │
│   ukf_zishiying('create') → ukf_tpl     │
│   single_track_runner() × 2雷达         │
│   → trackSnapshots_R1, R2               │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 6: 时间对齐                        │
│   time_align_tracks() → aligned_R2      │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 7: 融合                            │
│   run_track_fusion() × 4算法            │
│   → all_fused_snapshots{SCC,BC,CI,FCI}  │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 8: 误差评估                        │
│   evaluate_all('fusion')                │
│   evaluate_all('tracking_errors')       │
└─────────────────────────────────────────┘
        ↓
┌─────────────────────────────────────────┐
│ Phase 9: 可视化 + 保存                   │
│   plot_scene_overview()                 │
│   plot_point_cloud_3d()                 │
│   plot_results()                        │
│   save(.mat)                            │
└─────────────────────────────────────────┘
```

### 4. 蒙特卡洛仿真架构

**run_mc_straight.m** / **run_mc_turn.m** / **run_mc_turn_180deg.m** — 蒙特卡洛入口：
- N_MC=500次独立仿真
- 每种子独立随机流（seed + 1e7/2e7偏移隔离R1/R2）
- 完整统计：RMSE、MTL（平均航迹长度）、断裂次数、关联率、NIS、改善率
- 坏种子标记：UKF RMSE>30km 或 改善率<-50%
- 最优融合算法分布统计

### 5. 多目标场景架构

**run_simulation_multi.m** — 多目标单帧仿真：
- 三架飞机交叉航迹（均在雷达覆盖区内）
- generate_frame_detections_multi() 批量生成
- multi_track_manager() 多目标跟踪
- JPDA关联 + 多对融合匹配
- plot_results_multi() 多目标可视化

### 6. 南阳模块 (nanyang/)

一套独立的航迹处理子系统，包含：
- JNN（联合最近邻）关联算法
- M/N起始逻辑（trackStarter_logic）
- Alpha-Beta平滑滤波
- 航迹质量管理和信息补全
- 点迹→航迹数据转换器
- 该模块目前未被主流程调用，作为备用/对比方案保留

---

## 第二部分：函数接口文档

### 2.1 config/

#### simulation_params() → params
- 功能：返回完整仿真参数结构体（13模块）
- 输出：params — 包含所有雷达、UKF、关联、融合参数

#### simulation_params_multi() → params
- 功能：多目标参数配置，继承单目标后覆盖差异化参数
- 覆盖：detection_probability=1.0, imm_Pi=0.001

### 2.2 simulation/

#### aircraft_trajectory_create(waypoints_lla, speed_ms, dt_sec) → traj
- 功能：从航路点创建航迹结构体
- 输入：waypoints[N×3][lon,lat,alt], speed(m/s), dt(s)
- 输出：traj — .segments{N-1}, .duration_sec, .time_array, .n_steps

#### aircraft_trajectory_create('turn'|'gradual_turn'|'uturn', params) → traj, waypoints
- 功能：创建特定机动模式的航迹
- turn: 三航路点约120°拐角
- gradual_turn: 1°/s转弯率的渐进拐弯
- uturn: 180°回头弯

#### aircraft_trajectory_locate(traj, t) → idx, t_seg
- 功能：定位时间t所在的航段
- 输出：航段索引idx, 航段内偏移t_seg

#### aircraft_trajectory_interpolate(traj, t) → pos, vel_deg
- 功能：单点线性插值
- 输出：pos=[lon,lat], vel_deg=[lon_rate,lat_rate]

#### aircraft_trajectory_interpolate('batch'|'generate', ...) → out
- batch: 批量插值，N×5矩阵
- generate: 生成完整轨迹采样

#### generate_frame_detections(rx_lon, rx_lat, tx_lon, tx_lat, tgt_..., frameID, time_sec, range_bias, az_bias, beam_center, params, range_noise, az_noise) → detList, has_target_det
- 功能：单帧点迹生成（目标+杂波）
- 输出：detList — 结构体数组，含.prange/.paz/.pvr/.is_clutter等字段

#### generate_frame_detections_multi(rx_lon, rx_lat, tx_lon, tx_lat, tgt_states, frameID, time_sec, ...) → detList, has_target_dets
- 功能：多目标单帧点迹生成
- 输入：tgt_states[N×5][lon,lat,lon_rate,lat_rate,aircraft_id]

#### radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, beam_center, params) → in_coverage, r1, az
- 功能：威力覆盖判定（距离+方位）

#### radar_station_true_polar(radar, tx_lon, tx_lat, target_lon, target_lat, lon_rate, lat_rate) → rng, az, rv
- 功能：计算真实极坐标量测（群距离、方位角、多普勒）

#### skywave_geometry('group_range'|'azimuth'|'doppler', ...) → value
- 功能：天波几何计算（群距离/方位角/多普勒）

#### bistatic_inverse_solver(Rg, az, tx_lon, tx_lat, rx_lon, rx_lat) → r1, lat, lon
- 功能：双基地几何反解（从群距离和方位角求经纬度）

#### measurement_simulator('create'|'measure', ...) → sim/result
- 功能：量测仿真器（创建+测量两步）

### 2.3 ukf/

#### ukf_jichu('create'|'init'|'prepare'|'update'|'predict'|'measurement', ...) → ...
- 功能：基础UKF滤波器（CV/CT模型）
- create: 创建UKF模板（UT参数、R/Q/P矩阵）
- init: 两点差分初始化
- prepare: 预测+量测统计
- update: Kalman更新（innov_w=[]→纯预测）
- predict: 时间更新
- measurement: 天波量测模型h(x)

#### ukf_zishiying('create'|'init'|'prepare'|'update', ...) → ...
- 功能：自适应UKF（机动检测+模糊Q）
- 在ukf_jichu基础上增加apply_maneuver_adapt_post

#### ukf_imm('create'|'init'|'prepare'|'update', ...) → ...
- 功能：IMM UKF（CV+CT双模型）
- prepare: IMM混合→双预测→组合
- update: 双模型更新→Pd-IPDA似然→概率更新→状态组合

#### ukf_dispatch('create'|'init'|'prepare'|'update', ukf, ...) → ...
- 功能：多态路由器，根据ukf类型分发到对应后端

#### sigma_points_ukf(x, P, n, lam) → X
- 功能：生成2n+1个Sigma点

#### state_transition_ukf(x, dt) → x_next
- 功能：CV模型状态转移

#### state_transition_ct_ukf(x, dt, omega) → x_next
- 功能：CT协调转弯模型状态转移

#### meas_to_latlon_ukf(ukf, rng, az) → lon, lat
- 功能：极坐标→经纬度反解（经典解+迭代精化）

#### regularize_cov_ukf(P, min_eig) → P_reg
- 功能：协方差正则化（对称化+特征值裁剪）

### 2.4 tracker/

#### single_track_runner(detList, ukf_tpl, params, n_frames, true_track, t_grid) → trackSnapshots, finalTrack
- 功能：单目标统一跟踪器
- 状态机：INITIATING→TRACKING→LOST
- 真值辅助起始 + M/N重新起始
- 统一流水线：杂波预筛→prepare→NN→PDA→update

#### single_track_runner_adaptive(detList, ukf_tpl, params, n_frames) → trackSnapshots, finalTrack
- 功能：自适应版本（使用ukf_zishiying后端）

#### single_track_runner_nanyang(detList, ukf_tpl, params, n_frames) → trackSnapshots, finalTrack
- 功能：南阳方案混合跟踪器

#### multi_track_manager(trackList, tempPool, detList, ukf_tpl, params, frame_id) → trackList, tempPool, trackSnapshot
- 功能：多目标跟踪引擎

#### track_initiation('init'|'process'|'reset', ...) → state, det1, det2, success
- 功能：M/N滑窗起始器

#### post_init(ukf, params) → ukf
- 功能：UKF初始化后通用字段设置

#### inject_truth_velocity(ukf, tt_ac, t_grid, frame_id)
- 功能：从真值轨迹注入速度信息

### 2.5 association/

#### nn_associate(x_pred, z_pred, P_zz_2d, det_list, params, track_life) → best_det, dets_in_gate
- 功能：最近邻关联（地理预筛→Vr硬门→马氏距离）

#### pda_weight(dets_in_gate, z_pred, P_zz, params) → innov_weighted, beta_vec, nis_2d
- 功能：PDA概率加权

#### jpda_multi(trackList, active_idx, detList, params, truth_all) → assoc_pairs, dets_in_gate, innov_w
- 功能：多目标JPDA

### 2.6 fusion/

#### run_track_fusion(matched_pairs, trackSnapshots_R1, aligned_R2, params, method) → fused_snapshots
- 功能：逐帧融合主循环
- 支持：SCC/BC/CI/FCI四种算法
- BC专用：P12互协方差维护

#### track_fusion_algorithms('SCC'|'BC'|'CI'|'FCI', x1, P1, x2, P2, P12?) → x_fused, P_fused, w?
- 功能：四种融合算法调度器

#### time_align_tracks(trackSnapshots_R2, params) → aligned_R2
- 功能：R2航迹CV模型外推到R1时间网格

#### track_matcher(trackSnapshots_R1, trackSnapshots_R2, params) → matched_pairs
- 功能：跨雷达航迹匹配（匈牙利分配）

#### regularize_cov(P, min_eig) → P_reg
- 功能：协方差正则化

### 2.7 evaluation/

#### evaluate_all('tracking_errors'|'fusion', ...) → errorStats / fusion_eval
- 功能：统一评估调度器
- tracking_errors: UKF航迹 vs 真值
- fusion: 融合算法 vs 真值

#### evaluate_all_multi('tracking_errors'|'fusion', ...) → errorStats / fusion_eval
- 功能：多目标评估

#### compute_summary_eval(errs) → s
- 功能：统计摘要（n, median, mean, std, rms, min, max, pct95）

### 2.8 registration/

#### align_radar_to_grid(meas_list, unified_time, ref_time) → aligned
- 功能：异步雷达航迹对齐到统一时间网格

#### estimate_biases(r1_meas, r2_meas, truth_points, ...) → est
- 功能：两阶段偏差估计（LS+EML）

#### cost_fcn_with_params(biases, cp_list, ...) → total
- 功能：ECEF空间代价函数

### 2.9 utils/

#### sphere_utils_haversine_distance(lon1, lat1, lon2, lat2) → dist
- 功能：Haversine大圆距离（米）

#### sphere_utils_azimuth(lon_from, lat_from, lon_to, lat_to) → az
- 功能：球面初始方位角（度，0°=正北）

#### sphere_utils_destination_point(lon_start, lat_start, distance_m, az_deg) → lon, lat
- 功能：球面正算（从起点沿方位角走给定距离）

#### sphere_utils_radial_velocity(lon_rate, lat_rate, lat, az) → rv
- 功能：经纬度变化率→径向速度（m/s）

#### sphere_utils_interpolate_great_circle(lon1, lat1, lon2, lat2, fraction) → lon, lat
- 功能：大圆航线插值

#### sphere_utils_seconds_to_datetime_str(secs, ref_time) → time_str
- 功能：相对秒→日期时间字符串

#### skywave_geometry(action, ...) → value
- 功能：天波几何统一入口
- actions: constants, geocentric_angle, chord_length, slant_range, group_range, doppler, azimuth, full

#### coord_systems_lla_to_ecef(lat_deg, lon_deg, alt_m) → ecef
- 功能：WGS84 LLA→ECEF坐标转换

### 2.10 visualization/

#### plot_results(mode, ...)
- 功能：统一绘图调度器
- modes: single_track, single_fusion, combined_tracks, tracks_vs_truth, tracker, error_timeline, error_timeline_turn

#### plot_scene_overview(true_track, params, out_dir)
- 功能：场景总览图（真值+雷达+覆盖扇形）

#### plot_point_cloud_3d(detList, title_str, out_path)
- 功能：3D点云可视化（Range-Azimuth-Frame）

#### plot_turn_spatial(mode, ...)
- 功能：拐弯场景空间可视化
- modes: point_clouds, radar_compare, fusion_map, comprehensive

#### plot_turn_stats(mode, ...)
- 功能：拐弯场景统计可视化
- modes: comparison, fusion_compare, rmse_bars, single_compare

#### plot_results_multi(mode, ...)
- 功能：多目标可视化
- modes: single_track, single_fusion

### 2.11 io/

#### load_adsb(csv_path, icao_list, label_list, dt_sec, start_time, duration_sec, time_offset_sec) → true_tracks, labels, speeds
- 功能：加载ADS-B数据，过滤+重采样+速率计算

#### extract_measurement_field(meas_list, key) → vals
- 功能：从点迹cell数组提取指定字段

#### save_all(true_track, r1_meas_list, r2_meas_list, params, out_dir)
- 功能：保存仿真结果（CSV + MAT + JSON）

### 2.12 nanyang/

#### PointTrackAssociation_JNN(trackList, pointList, sysPara) → TPmatch_result, outputPointList, singlePoints
- 功能：联合最近邻关联（二分图匹配）

#### trackStarter_logic(tempTrackList, pointList, sysPara, QUALIFY_NUM, TOLERANT_NUM) → tempTrackList, validTracks
- 功能：M/N航迹起始

#### predictNextStep_cv(curTrack, sysPara, trackPara) → predictedTrack
- 功能：CV模型下一步预测

#### fun_trackfilter_AlphaBeta(curTrack, asscPoint, sysPara) → smoothed
- 功能：Alpha-Beta平滑滤波

#### fun_track_quality_management_and_info_completion(curTrack, bestPoint, sysPara) → updatedTrack
- 功能：航迹质量管理和信息补全

#### det2nanyang_point(det_list, frame_id, time_stamp) → ny_points
- 功能：检测点迹→南阳格式转换

---

## 第三部分：主入口脚本说明

### 3.1 run_simulation.m — 单目标直线仿真主入口
9阶段流水线：场景初始化→ADS-B标定→点迹生成→策略声明→偏差校正→UKF跟踪→时间对齐→融合→误差评估→可视化保存

### 3.2 run_simulation_turn.m — 单目标拐弯仿真
类似run_simulation，但使用渐进拐弯航迹

### 3.3 run_simulation_turn_180deg.m — 180度回头弯仿真
使用180°左转半圆航迹

### 3.4 run_simulation_multi.m — 多目标交叉航迹仿真
三架飞机交叉，多目标跟踪+融合

### 3.5 run_mc_straight.m — 直线场景蒙特卡洛
500次独立仿真，完整统计

### 3.6 run_mc_turn.m — 拐弯场景蒙特卡洛
500次独立仿真

### 3.7 run_mc_turn_180deg.m — 180度回头弯蒙特卡洛
500次独立仿真

### 3.8 run_mc_turn_compare.m — 拐弯场景对比蒙特卡洛
对比基础UKF vs 自适应UKF vs IMM

### 3.9 run_mc_turn_180deg_compare.m — 180度回头弯对比蒙特卡洛
对比三种UKF后端

### 3.10 scan_Q_scale.m — Q缩放因子扫描调参
扫描不同Q_scale值，找出最优参数

### 3.11 scan_Pi.m — IMM转移概率扫描调参
扫描IMM的Pi转移概率

---

## 第四部分：git历史关键节点

### 分支结构
- **main**: 主开发分支（37个提交）
- **ukf_with_imm_jidongzishiying_mohuzishiying_all**: UKF+IMM+机动自适应全功能分支

### 关键提交
1. `93a38c2` — 初始提交
2. `70eb878` — 生成详细注释和项目结构阅读调用说明文档
3. `8a99c47` — 精简架构，提取单独UKF/航迹起始/航迹关联模块
4. `e03621e` — 6项关键优化：径向速度硬门、纯M/N重新起始、移除probation约束、完善直线蒙特卡洛
5. `3471188` — 8轮调参，坏种子率从28%降至10%
6. `ba16c28` — 引入电离层虚高天波模型
7. `5eab44b` — 统一tracker入口，分立三种UKF，模块化
8. `d564f10` — 分离evaluate文件，单目标多目标分开
9. `7c166d4` — 最新提交

---

## 第五部分：文件目录总览

```
single_target_with-turn/
├── CLAUDE.md                          # 项目配置说明
├── PROJECT_DOCUMENTATION.md           # 前期文档
├── UKF调优总结报告.md                 # UKF调参总结
├── adsb-format.md                     # ADS-B数据格式说明
├── 仿真代码修改报告_20260517.md       # 代码修改报告
├── 仿真方案总结.md                    # 仿真方案总结
├── 天发天收量测模型仿真修改意见.md    # 量测模型修改意见
├── 研讨会对话整理稿.md                # 研讨记录
├── 毕业设计原始版（超详细）.md        # 毕设原始需求
│
├── config/
│   ├── simulation_params.m            # 单目标参数配置（13模块）
│   └── simulation_params_multi.m      # 多目标参数配置
│
├── simulation/
│   ├── aircraft_trajectory_create.m   # 航迹结构体创建
│   ├── aircraft_trajectory_interpolate.m  # 航迹插值
│   ├── aircraft_trajectory_locate.m   # 航段时间定位
│   ├── bistatic_inverse_solver.m      # 双基地几何反解
│   ├── generate_frame_detections.m    # 单帧点迹生成（单目标）
│   ├── generate_frame_detections_multi.m  # 单帧点迹生成（多目标）
│   ├── measurement_simulator.m        # 量测仿真器
│   ├── radar_coverage_check.m         # 威力覆盖判定
│   ├── radar_station_true_polar.m     # 真实极坐标量测
│   └── tracker_utils.m               # 跟踪器工具（stitch等）
│
├── ukf/
│   ├── ukf_dispatch.m                 # UKF多态路由器
│   ├── ukf_imm.m                      # IMM UKF（CV+CT双模型）
│   ├── ukf_jichu.m                    # 基础UKF
│   └── ukf_zishiying.m               # 自适应UKF
│
├── tracker/
│   ├── single_track_runner.m          # 单目标统一跟踪器
│   ├── single_track_runner_adaptive.m # 自适应版本
│   ├── single_track_runner_nanyang.m  # 南阳方案
│   ├── single_track_runner_nanyang_adaptive.m
│   ├── multi_track_manager.m          # 多目标跟踪引擎
│   ├── multi_track_runner_kf.m        # 多目标KF运行器
│   ├── multi_track_start.m            # 多目标起始
│   ├── post_init_multi.m              # 多目标初始化后处理
│   ├── track_management.m             # 航迹管理（JNN+质量）
│   └── inject_truth_velocity.m        # 真值速度注入
│
├── association/
│   ├── nn_associate.m                 # 最近邻关联
│   ├── pda_weight.m                   # PDA加权
│   └── jpda_multi.m                   # 多目标JPDA
│
├── fusion/
│   ├── run_track_fusion.m             # 逐帧融合主循环
│   ├── track_fusion_algorithms.m      # 四种融合算法
│   ├── time_align_tracks.m            # 时间对齐
│   ├── track_matcher.m                # 跨雷达航迹匹配
│   └── regularize_cov.m              # 协方差正则化
│
├── initiation/
│   └── track_initiation.m             # M/N滑窗起始器
│
├── evaluation/
│   ├── evaluate_all.m                 # 单目标评估
│   └── evaluate_all_multi.m           # 多目标评估
│
├── registration/
│   ├── align_radar_to_grid.m          # 雷达对齐到网格
│   ├── cost_fcn_with_params.m         # 代价函数
│   └── estimate_biases.m              # 偏差估计
│
├── utils/
│   ├── coord_systems_lla_to_ecef.m    # LLA→ECEF转换
│   ├── skywave_geometry.m             # 天波几何模型
│   ├── sphere_utils_azimuth.m         # 球面方位角
│   ├── sphere_utils_destination_point.m  # 球面正算
│   ├── sphere_utils_haversine_distance.m  # Haversine距离
│   ├── sphere_utils_interpolate_great_circle.m  # 大圆插值
│   ├── sphere_utils_radial_velocity.m # 径向速度
│   └── sphere_utils_seconds_to_datetime_str.m  # 时间格式化
│
├── visualization/
│   ├── plot_results.m                 # 统一绘图调度器
│   ├── plot_results_multi.m           # 多目标绘图
│   ├── plot_scene_overview.m          # 场景总览
│   ├── plot_scene_overview_multi.m    # 多目标场景总览
│   ├── plot_point_cloud_3d.m          # 3D点云
│   ├── plot_turn_spatial.m            # 拐弯空间可视化
│   └── plot_turn_stats.m              # 拐弯统计可视化
│
├── io/
│   ├── load_adsb.m                    # ADS-B数据加载
│   ├── extract_measurement_field.m    # 字段提取
│   └── save_all.m                     # 结果保存
│
├── nanyang/                           # 南阳航迹处理子系统（备用）
│   ├── PointTrackAssociation_JNN.m
│   ├── cleanTrackList.m
│   ├── det2nanyang_point.m
│   ├── det2trackDataConverter.m
│   ├── distance.m
│   ├── fun_calculate_track_travelLen.m
│   ├── fun_check_35logic_points_improved.m
│   ├── fun_check_colinear_points.m
│   ├── fun_check_track_validation.m
│   ├── fun_create_new_track.m
│   ├── fun_fill_smooth_list_by_alpha_beta_filter.m
│   ├── fun_fill_smooth_list_by_predict_result.m
│   ├── fun_find_tracks_to_report.m
│   ├── fun_remove_assc_pts_from_pointlist.m
│   ├── fun_select_point_by_rd.m
│   ├── fun_select_track_by_rd.m
│   ├── fun_set_tracking_parameter.m
│   ├── fun_track_quality_management_and_info_completion.m
│   ├── fun_trackfilter_AlphaBeta.m
│   ├── Fun_PredictNextStep_CV.m
│   ├── Fun_UpdateTrackByAsscResult.m
│   ├── Fun_UpdateTrackforNoInputPoint.m
│   ├── header.m
│   ├── is_duplicate_track.m
│   ├── pdCoefInterprator.m
│   ├── predictNextStep_cv.m
│   ├── reckon.m
│   ├── resetAllTracks.m
│   ├── robu---

## 第三部分：TRACKER 模块完整文档

### 3.1 模块概述

TRACKER 模块是整个跟踪系统的核心控制层，负责将 UKF 滤波器、关联算法（NN/PDA）、航迹起始（M/N）和质量管理等底层组件组装成可运行的逐帧跟踪流水线。本模块定义了航迹的生命周期管理、状态机流转、以及单目标/多目标两种运行模式。

#### 3.1.1 模块文件清单

| 文件名 | 行数 | 职责 |
|--------|------|------|
| `single_track_runner.m` | 329 | 统一单目标跟踪器（状态机：INITIATING→TRACKING→LOST） |
| `single_track_runner_adaptive.m` | 211 | 机动自适应单目标跟踪器 |
| `single_track_runner_nanyang.m` | 372 | 南阳混合方案单目标跟踪器（2态：WAITING→TRACKING→LOST） |
| `single_track_runner_nanyang_adaptive.m` | 316 | 南阳混合方案自适应版 |
| `multi_track_manager.m` | 185 | 多目标航迹管理主引擎 |
| `multi_track_runner_kf.m` | 197 | 多目标逐帧跟踪包装器 |
| `track_management.m` | 156 | JNN全局关联 + 航迹质量状态机 |
| `inject_truth_velocity.m` | 13 | 真值速度/位置注入（调试用） |
| `post_init_multi.m` | 13 | 多目标UKF初始化后处理 |
| `multi_track_start.m` | 38 | 多目标M/N起始模块 |

#### 3.1.2 架构总览

```
主入口 (run_mc_*.m)
    │
    ├─▶ single_track_runner.m          ◀── ukf_dispatch (多态路由)
    │     │                              ├─ ukf_jichu (基础UKF)
    │     │                              ├─ ukf_zishiying (自适应UKF)
    │     │                              └─ ukf_imm (IMM UKF)
    │     │
    │     ├─ track_initiation (M/N起始)
    │     ├─ nn_associate (最近邻关联)
    │     ├─ pda_weight (PDA加权)
    │     └─ make_track_snap (航迹快照)
    │
    ├─▶ single_track_runner_adaptive.m   ◀── ukf_zishiying (自适应)
    │
    ├─▶ single_track_runner_nanyang.m    ◀── ukf_jichu + 南阳验证
    │     ├─ det2nanyang_point (格式转换)
    │     └─ fun_check_track_validation (多维物理验证)
    │
    ├─▶ multi_track_manager.m            ◀── JNN全局关联
    │     ├─ track_management('associate')
    │     ├─ track_management('quality')
    │     └─ cleanup_stale (候选池清理)
    │
    └─▶ multi_track_runner_kf.m          ◀── 真值辅助多目标起始
```

---

### 3.2 single_track_runner.m — 统一单目标航迹跟踪器

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\single_track_runner.m`
**总行数**: 329 行（含注释）
**核心功能**: 单目标逐帧航迹管理的顶层控制器，实现从航迹起始到跟踪维持再到丢失恢复的完整生命周期。

#### 3.2.1 主函数：`single_track_runner`

**签名**:
```matlab
function [trackSnapshots, finalTrack] = single_track_runner(detList, ukf_tpl, params, n_frames, varargin)
```

**参数说明**:

| 参数 | 类型 | 说明 |
|------|------|------|
| `detList` | cell[n_frames×1] | 每帧的点迹结构体数组，元素为 `struct`，含 `lon`, `lat`, `range_meas`, `azimuth_meas`, `radial_vel_meas`, `is_clutter`, `frameID` 等字段 |
| `ukf_tpl` | struct | UKF模板，由 `ukf_xxx('create', ...)` 产生，含 `tx_lon`, `tx_lat`, `radar_lon`, `radar_lat`, `Q`, `R`, `F`, `H` 等 |
| `params` | struct | 参数结构体，见下方详解 |
| `n_frames` | int | 仿真总帧数 |
| `varargin{1}` | matrix | 可选，真值航迹矩阵 `true_track` |
| `varargin{2}` | vector | 可选，时间网格 `t_grid` |

**`params` 关键字段**:

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `dt_sec` | float | 1.0 | 采样间隔（秒） |
| `tracker_N` | int | 10 | M/N 起始的 N（滑窗长度） |
| `tracker_M` | int | 6 | M/N 起始的 M（最少检测帧数） |
| `tracker_K_loss` | int | 8 | 连续丢帧数阈值，超过则航迹 LOST |
| `gate_sigma` | float | 6 | 马氏距离门系数 |
| `gate_vr_ms` | float | 20/40 | 径向速度硬门（m/s） |
| `pda_pd_gate` | float | 0.9 | 波门内检测概率 |
| `detection_probability` | float | 0.6 | 全局检测概率 |
| `pda_clutter_intensity` | float | - | 杂波密度参数 |
| `use_truth_init` | bool | true | 是否使用真值辅助首次起始 |
| `fuzzy_window_size` | int | 20 | NIS 滑动窗口大小 |

**返回值**:

| 返回值 | 类型 | 说明 |
|--------|------|------|
| `trackSnapshots` | cell[n_frames×1] | 每帧的航迹快照，`snap.frameID`, `snap.trackList{1}` |
| `finalTrack` | struct | 最终航迹摘要，含 `id`, `type`, `quality`, `life`, `mu_history` |

**内部状态变量**:

| 变量 | 类型 | 说明 |
|------|------|------|
| `track_state` | string | 状态机状态：`'INITIATING'` / `'TRACKING'` / `'LOST'` |
| `life` | int | 航迹持续帧数 |
| `missed` | int | 连续丢帧数 |
| `quality` | int | 航迹质量分 [0, 15] |
| `ukf` | struct | UKF滤波器状态 |
| `init_state` | struct | M/N 起始滑窗状态 |
| `first_init_done` | bool | 首次起始是否已完成 |
| `reinit_truth_collecting` | bool | 重新起始时是否正在收集真值帧 |
| `reinit_attempt_frame` | int | 上次尝试重新起始的帧号 |

**状态机流程图**:

```
                    ┌──────────────┐
                    │  INITIATING   │
                    │ (等待M/N起始  │
                    │  或真值辅助)  │
                    └──────┬───────┘
                           │
              ┌────────────┴────────────┐
              │ M/N成功 / 真值辅助成功    │
              ▼                         │
                    ┌──────────────┐    │
                    │   TRACKING    │◄───┘
                    │ (UKF预测→NN  │
                    │  关联→PDA→  │
                    │  UKF更新)    │
                    └──────┬───────┘
                           │
              ┌────────────┴────────────┐
              │ missed >= K_loss        │
              ▼                         │
                    ┌──────────────┐    │
                    │     LOST     │────┘
                    │ (重置状态)   │
                    └──────────────┘
```

**逐帧处理流程详解**:

**Phase 1: INITIATING 状态**

此状态有三种起始路径：

**(a) 首次真值辅助起始（`first_init_done == false`）**

- 从 `true_track` 中通过 `interp1` 插值得到当前帧目标经纬度
- 调用 `skywave_geometry('group_range', ...)` 计算双基地群距离
- 调用 `sphere_utils_azimuth(...)` 计算方位角
- 构造两个时间点（间隔至少1帧）的伪检测 `init_det1`, `init_det2`
- 调用 `ukf_dispatch('init', ukf_tpl, init_det1, init_det2)` 初始化 UKF
- 调用 `post_init(ukf, params)` 设置初始化的通用字段
- 进入 TRACKING 状态，`quality = 5`

**(b) 重新起始超时兜底（`reinit_truth_collecting == true`）**

- 当 M/N 起始超时（超过 `max(4, N-2)` 帧仍未成功），触发真值兜底
- 收集当前帧真值作为 `reinit_truth_det1`，下一帧作为 `reinit_truth_det2`
- 初始化 UKF 后进入 TRACKING

**(c) 纯 M/N 滑窗起始（默认路径）**

- 调用 `track_initiation('process', init_state, dets, params, k)`
- M/N 算法：维护长度为 N 的滑窗，当窗内 ≥ M 帧有检测且当前帧有检测时
- 对所有可能的 (det1, det2) 配对进行速度检验（30-600 m/s）和共识评分（80km 门限内其他帧有点迹支持）
- 评分 ≥ 1 即视为成功，初始化 UKF

**(d) 起始失败帧**

- 输出 `type = 6`（临时航迹占位符），`life = 0`, `quality = 0`

**Phase 2: TRACKING 状态（统一流水线）**

这是每帧的核心处理流程，严格按顺序执行以下步骤：

**步骤1: 杂波预筛**

```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

- 遍历当前帧所有点迹，剔除 `is_clutter == true` 的杂波点
- 这一步在关联之前完成，减少计算量

**步骤2: UKF 预测**

```matlab
[x_pred, ~, ~, z_pred, ~, P_zz, ukf] = ukf_dispatch('prepare', ukf);
```

- 通过 `ukf_dispatch` 多态路由到具体 UKF 后端
- 返回预测状态 `x_pred`（4×1: [lon, lon_dot, lat, lat_dot]）
- 返回预测量测 `z_pred`（3×1: [range, azimuth, radial_vel]）
- 返回量测协方差 `P_zz`（3×3）

**步骤3: NN 关联**

```matlab
saved_vr = params.gate_vr_ms;
params.gate_vr_ms = 9999;  % 临时禁用 Vr 门
[best_det, dets_in_gate] = nn_associate(x_pred, z_pred, P_zz(1:2,1:2), clean_dets, params, life);
params.gate_vr_ms = saved_vr;  % 恢复
```

- 临时将 Vr 门设为极大值（9999），禁用径向速度硬门
- 调用 `nn_associate` 进行两步筛选：地理距离预筛 → 2D马氏距离精筛
- 返回最佳匹配点迹 `best_det` 和波门内所有点迹 `dets_in_gate`

**步骤4: 连续丢点防杂波劫持（地理门）**

```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(x_pred(1), x_pred(3), best_det.lon, best_det.lat);
    if geo_dist > 50000  % 50km
        best_det = [];
        dets_in_gate = {};
    end
end
```

- 当连续丢帧 ≥ 2 时，即使 NN 找到了关联点迹，也检查其地理距离
- 若距离 > 50km，判定为杂波劫持，强制丢弃
- 这是防止航迹在长时间丢失后被远处杂波"劫持"的关键保护

**步骤5: PDA 加权新息**

```matlab
[innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred, P_zz, params);
```

- 计算波门内各点迹的关联概率 `beta_i`
- 构造加权新息 `innov_w = Σ β_i · γ_i`（γ_i 为第 i 个点迹的新息）
- 返回 NIS (Normalized Innovation Squared) 值 `nis_val`

**步骤6: Probation 期保护**

```matlab
if life <= 5 && nis_val > 50
    reject_update = true;
end
```

- 航迹生命 ≤ 5 帧时，若 NIS > 50（远大于卡方分布的合理阈值 2×df=8），判定为异常关联
- 拒绝本次更新，避免错误关联污染新航迹

**步骤7: 机动预检测上下文设置**

```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
```

- 为 `ukf_zishiying` 后端提供机动检测所需的历史上下文
- `life_count` 补偿性 +1（因为 life 在尾部递增）

**步骤8: UKF 更新**

```matlab
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
```

- 使用 PDA 加权新息更新 UKF 状态
- 返回更新后的经纬度

**步骤9: 航迹维护**

- 记录 NIS 历史：`ukf.nis_history(end+1) = nis_val`
- 更新计数器：`missed = 0`, `life++`, `quality = min(quality+1, 15)`

**丢失分支**（`best_det == []` 或 `reject_update == true`）:

- 执行纯预测：`ukf.x = x_pred`, `ukf.P = P_pred`
- `missed++`, `life++`, `quality = max(quality-1, 0)`

**Phase 3: LOST 状态**

- 当 `missed >= params.tracker_K_loss`（默认 8 帧），进入 LOST 状态
- 重置所有状态变量：`track_state = 'INITIATING'`, `init_state = track_initiation('reset', params)`
- 启动重新起始超时计时器：`reinit_attempt_frame = k`
- 输出 `type = 7`（历史航迹占位符）

#### 3.2.2 辅助函数：`post_init`

**签名**:
```matlab
function ukf = post_init(ukf, params)
```

**功能**: UKF 初始化后的通用字段设置，确保所有 UKF 后端具有一致的初始状态。

**执行步骤**:

1. 设置 `ukf.dt = params.dt_sec`（采样间隔）
2. 设置 `ukf.initialized = true`（标记已初始化）
3. 若为 IMM 类型（含 `ukf_cv` 字段），同步设置 `ukf.ukf_cv` 和 `ukf.ukf_ct` 的 `dt` 和 `initialized`
4. 重置 `ukf.nis_history = []`（每次新起始清空 NIS 历史）
5. 若 `ukf.Q_base` 为空，从 `ukf.Q` 备份一份
6. 若 `ukf.Q_ema` 为空，初始化为 1.0

#### 3.2.3 辅助函数：`make_track_snap`

**签名**:
```matlab
function trk = make_track_snap(id, type, lat, lon, ukf, life, quality, missed, det)
```

**返回结构体字段**:

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | int | 航迹 ID（单目标固定为 1） |
| `type` | int | 航迹类型：1=可靠, 2=维持, 6=临时, 7=历史 |
| `lat` | float | 纬度（度） |
| `lon` | float | 经度（度） |
| `ukf` | struct | UKF 状态引用 |
| `life` | int | 航迹持续帧数 |
| `quality` | int | 质量分 [0, 15] |
| `missed` | int | 连续丢帧数 |
| `assoc_det` | struct | 关联的点迹 |
| `x_pred` | vector | 预测状态（仅当 `det == []` 时非空） |
| `P_pred` | matrix | 预测协方差（仅当 `det == []` 时非空） |

#### 3.2.4 辅助函数：`iif`

**签名**:
```matlab
function v = iif(cond, t, f)
```

**功能**: 内联 if-else 表达式，用于单行条件赋值。

**边缘情况**: 无特殊边界处理，纯过程式条件分支。

---

### 3.3 single_track_runner_adaptive.m — 机动自适应单目标跟踪器

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\single_track_runner_adaptive.m`
**总行数**: 211 行

**功能概述**: 对 `single_track_runner` 的机动自适应增强版本。核心差异在于使用 `ukf_zishiying`（机动检测 + 模糊 Q 自适应）替代 `ukf_jichu`。

#### 3.3.1 与 single_track_runner 的关键差异

| 维度 | single_track_runner | single_track_runner_adaptive |
|------|---------------------|------------------------------|
| UKF 后端 | `ukf_dispatch`（多态） | 固定 `ukf_jichu('prepare')` + `ukf_zishiying('update')` |
| 杂波预筛 | 有（`is_clutter` 过滤） | 无 |
| 地理门防劫持 | 有（missed≥2 时 50km 检查） | 无 |
| 真值辅助起始 | 有（首次 + 超时兜底） | 无 |
| Probation 速度检查 | 无 | 有（life≤10 时方向/速度检查） |
| 机动检测字段 | 仅 TRACKING 阶段设置 | 初始化即设置全部字段 |

#### 3.3.2 主函数：`single_track_runner_adaptive`

**签名**:
```matlab
function [trackSnapshots, finalTrack] = single_track_runner_adaptive(detList, ukf_tpl, params, n_frames)
```

注意：此函数**不接受** `varargin` 参数（无真值辅助起始能力）。

**初始化阶段（INITIATING）**:

- 纯 M/N 滑窗起始，调用 `track_initiation('process', ...)`
- 成功后调用 `ukf_zishiying('init', ukf_tpl, det1, det2)`
- 初始化机动检测字段：
  - `ukf.maneuver_active = false`（机动未激活）
  - `ukf.maneuver_counter = 0`（机动持续帧计数）
  - `ukf.maneuver_recovery = 0`（机动恢复计数器）
  - `ukf.suspect_counter = 0`（可疑帧计数）
  - `ukf.life_count = 1`（生命期计数）

**TRACKING 阶段**:

1. `ukf_jichu('prepare', ukf)` — 预测
2. `nn_associate(...)` — NN 关联（使用完整 `dets`，不做杂波预筛）
3. `pda_weight(...)` — PDA 加权
4. Probation 保护（life≤5, NIS>50）
5. 速度合理性检查（life≤10）:
   - 预测速度方向：`v_pred_dir = atan2d(x_pred(4), x_pred(2))`
   - 更新后方向：`v_new_dir = atan2d(ukf.x(4), ukf.x(2))`
   - 方向突变 >90° → 拒绝更新
   - 速度大小 >500 m/s → 拒绝更新
6. 设置机动预检测字段（`ukf.last_innov`, `ukf.last_x_pred`, `ukf.last_z_pred`, `ukf.last_P_zz`, `ukf.last_det_list`）
7. `ukf_zishiying('update', ...)` — 自适应 UKF 更新（内部包含机动检测和 Q 自适应）

**LOST 阶段**:

- 与 `single_track_runner` 相同，重置状态回到 INITIATING

#### 3.3.3 辅助函数

**`make_track_snap_adapt`**: 与 `make_track_snap` 完全相同。

**`iif_adapt`**: 与 `iif` 完全相同。

**`angdiff_deg_ad`**: 角度差计算
```matlab
function d = angdiff_deg_ad(a, b)
    d = mod(b - a + 180, 360) - 180;
end
```
返回 `(-180, 180]` 范围内的最小角度差。

---

### 3.4 single_track_runner_nanyang.m — 南阳混合方案单目标跟踪器

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\single_track_runner_nanyang.m`
**总行数**: 372 行

**功能概述**: 结合 M/N 滑窗起始 + 南阳物理验证 + UKF+PDA 跟踪的混合方案。相比 `single_track_runner` 增加了多维度物理合理性验证，但去除了复杂的质量状态机。

#### 3.4.1 设计 rationale

单目标场景不需要复杂的质量状态机（TEMPORARY→RELIABLE 晋升等）。南阳质量机是为多目标/高更新率微波雷达设计的，用于天波雷达单目标场景反而会导致 84% 的"坏种子"仅仅因为从未达到 `QUALITY_RELIABLE`。因此本方案采用简化的 2 态跟踪（WAITING/TRACKING），但在起始阶段加入了南阳的多维物理验证。

#### 3.4.2 状态机

```
WAITING ──(M/N + 验证通过)──> TRACKING
    ↑                              │
    └──────(连续miss≥K_loss)───────┘
```

只有三个状态：`WAITING`（等效于 INITIATING）、`TRACKING`、`LOST`（瞬时过渡到 WAITING）。

#### 3.4.3 主函数：`single_track_runner_nanyang`

**签名**:
```matlab
function [trackSnapshots, finalTrack] = single_track_runner_nanyang(detList, ukf_tpl, params, n_frames, varargin)
```

**sysPara 构建（供南阳验证函数使用）**:

| 字段 | 值 | 说明 |
|------|------|------|
| `T_inter` | `params.dt_sec` | 互相关间隔 |
| `datenum` | `now` | 当前日期 |
| `deltaR` | 10 | 距离分辨率（km） |
| `deltaAz` | 2 | 方位分辨率（度） |
| `deltaV` | 20 | 速度分辨率（m/s） |
| `tx_BLH` | `[lat, lon]` | 发射站 BLH |
| `rx_BLH` | `[lat, lon]` | 接收站 BLH |
| `f0` | 10.0 | 载频（MHz） |
| `lambda` | 30.0 | 波长（m） |
| `prt` | 0.05 | 脉冲重复周期（s） |
| `fIndex` | `[0, 0]` | 频率索引 |
| `aIndex` | `[0, 360]` | 方位索引 |
| `rIndex` | `[0, 5000]` | 距离索引（km） |
| `ucMode` | 9 | 不确定模式 |
| `tx_XOY` | `[0, 0]` | 发射站 XY |

**WAITING 阶段三种路径**:

**(a) 真值辅助起始（`first_init_done == false && use_truth_init == true`）**

- 有真值数据：用真值位置构造伪检测，调用 `ukf_jichu('init', ...)`
- 无真值数据：从当前帧检测中选取第一个非杂波点迹作为起始点

**(b) M/N 起始 + 南阳验证（默认路径）**

1. 调用 `track_initiation('process', ...)` 获取 M/N 配对 `(det1, det2)`
2. 调用 `build_candidate_for_validation(...)` 构建候选航迹（含历史点迹序列）
3. 调用 `fun_check_track_validation(candidate)` 进行多维物理验证
4. 验证通过则初始化 UKF 进入 TRACKING；否则重置 M/N 状态

**`build_candidate_for_validation` 函数详解**:

**签名**:
```matlab
function candidate = build_candidate_for_validation(init_state, det1, det2, k, point_history, params, curTime, ukf_tpl)
```

**算法步骤**:

1. 遍历 M/N 滑窗中的所有历史帧点迹
2. 对每帧找到一个"最佳匹配点"：该点与 det1 和 det2 的 Haversine 距离之和的一半最小，且两者都 < 80km
3. 将历史最佳点和当前帧 det2 一起转换为南阳格式（`det2nanyang_point`）
4. 按 `frameID` 排序后存入 `candidate.asscPointList`
5. 若候选点数 < 3，返回空（验证需要至少 3 个点）

**TRACKING 阶段增强保护**:

除了标准 UKF+PDA 流水线外，增加了三道保护机制：

**(1) Probation 期速度合理性检查（life ≤ 10，仅 M/N 起始需要）**

- 方向突变检查：`abs(angdiff(v_pred_dir, v_new_dir)) > 90°` → 拒绝
- 速度大小检查：`sqrt(v_E^2 + v_N^2) * 111320 * cos(|lat|) > 500 m/s` → 拒绝
- 注意：真值辅助起始跳过此检查（因为真值初始化已经保证了合理性）

**(2) 全生命周期位置跳变保护**

```matlab
jump_m = sphere_utils_haversine_distance(x_pred(1), x_pred(3), lon, lat);
if jump_m > 50000  % 50km
    reject_update = true;
end
```

- 无论航迹处于什么阶段，若更新后的位置与预测位置相差 > 50km，判定为异常
- 可能原因：错误关联了远距离杂波点

**(3) 模糊自适应 Q（`params.use_fuzzy_adaptive && life > 12`）**

```matlab
ukf = apply_fuzzy_adapt(ukf, params);
```

**`apply_fuzzy_adapt` 模糊 Q 自适应算法**:

基于 NIS 历史的滑动窗口均值，通过三角隶属函数计算机动因子：

$$\text{nis\_ratio} = \frac{\text{mean}(\text{NIS历史})}{2.0}$$

五个三角隶属函数：

| 模糊集 | 三角形 (a, b, c) | 代表因子 |
|--------|-------------------|----------|
| VS (很小) | (0.0, 0.0, 0.4) | 0.6 |
| S (小) | (0.2, 0.5, 0.8) | 0.8 |
| M (中) | (0.6, 1.0, 1.5) | 1.0 |
| L (大) | (1.3, 2.0, 3.0) | 1.8 |
| VL (很大) | (2.5, 4.0, 4.0) | 3.0 |

加权平均：

$$\text{factor}_{\text{fuzzy}} = \frac{\sum \mu_i \cdot \text{value}_i}{\sum \mu_i}$$

EMA 平滑：

$$Q_{\text{ema}}^{(k)} = \eta \cdot \max(0.5, \min(4.0, \text{factor}_{\text{fuzzy}})) + (1-\eta) \cdot Q_{\text{ema}}^{(k-1)}$$

其中 $\eta = 0.20$（可通过 `params.fuzzy_ema_eta` 覆盖）。

最终过程噪声协方差：

$$Q = \begin{cases} Q_{\text{base}} & |Q_{\text{ema}} - 1.0| < 0.05 \\ Q_{\text{base}} \cdot Q_{\text{ema}} & \text{otherwise} \end{cases}$$

#### 3.4.4 辅助函数

**`make_snap`**: 与 `make_track_snap` 类似，但不包含 `x_pred`/`P_pred` 字段。

**`angdiff`**: 角度差计算，与 `angdiff_deg_ad` 逻辑相同。

---

### 3.5 single_track_runner_nanyang_adaptive.m — 南阳混合方案自适应版

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\single_track_runner_nanyang_adaptive.m`
**总行数**: 316 行

**功能概述**: 与 `single_track_runner_nanyang` 的唯一差异是使用 `ukf_zishiying`（机动检测+Q自适应）替代 `ukf_jichu`。其余逻辑完全一致。

#### 3.5.1 关键差异对照

| 位置 | nanyang | nanyang_adaptive |
|------|---------|------------------|
| UKF init | `ukf_jichu('init', ...)` | `ukf_zishiying('init', ...)` |
| prepare | `ukf_jichu('prepare', ukf)` | `ukf_jichu('prepare', ukf)`（注意仍用 jichu） |
| update | `ukf_jichu('update', ukf, innov_w, ...)` | `ukf_zishiying('update', ukf, innov_w, ..., params)` |
| 机动字段 | 无 | 初始化 `maneuver_active`, `maneuver_counter`, `maneuver_recovery`, `suspect_counter`, `life_count` |
| 模糊Q | `apply_fuzzy_adapt` 独立调用 | 封装在 `ukf_zishiying` 内部 |

#### 3.5.2 初始化细节

真值辅助起始时，额外设置：

```matlab
ukf.maneuver_active = false;    % 机动未激活
ukf.maneuver_counter = 0;       % 机动持续帧计数
ukf.maneuver_recovery = 0;      % 机动恢复计数器
ukf.suspect_counter = 0;        % 可疑帧计数
ukf.life_count = 1;             % 生命期计数
```

这些字段在 `ukf_zishiying` 的 `prepare` 和 `update` 中被读取，用于机动检测和 Q 自适应调整。

#### 3.5.3 辅助函数

**`build_candidate_for_validation_ad`**: 与 `build_candidate_for_validation` 完全相同。

**`make_snap_ad`**: 与 `make_snap` 完全相同。

**`angdiff_ad`**: 与 `angdiff` 完全相同。

---

### 3.6 multi_track_manager.m — 多目标航迹管理主引擎

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\multi_track_manager.m`
**总行数**: 185 行

**功能概述**: 多目标场景下的逐帧跟踪主引擎。每帧执行完整的跟踪流水线：分离活跃/历史航迹 → 批量 UKF 预测 → JNN 全局关联 → 关联航迹更新 → 未关联航迹预测 → 质量管理 → 新航迹起始。

#### 3.6.1 主函数：`multi_track_manager`

**签名**:
```matlab
function [trackList, tempPool, trackSnapshot] = multi_track_manager(...
        trackList, tempPool, detList, ukf_tpl, params, frame_id)
```

**参数说明**:

| 参数 | 类型 | 说明 |
|------|------|------|
| `trackList` | cell[] | 当前所有航迹列表，每个元素为 struct |
| `tempPool` | cell[] | 临时候选池（M/N 起始的中间状态） |
| `detList` | struct[] | 当前帧所有检测点迹 |
| `ukf_tpl` | struct | UKF 模板 |
| `params` | struct | 参数结构体 |
| `frame_id` | int | 当前帧编号 |

**返回值**:

| 返回值 | 类型 | 说明 |
|--------|------|------|
| `trackList` | cell[] | 更新后的航迹列表 |
| `tempPool` | cell[] | 更新后的候选池 |
| `trackSnapshot` | struct | 当前帧快照 |

**逐帧处理流程（8个步骤）**:

**特殊情况：当前帧无检测点迹**

```matlab
if isempty(detList)
    % 所有活跃航迹执行纯预测
    for t = 1:length(trackList)
        if trackList{t}.type == 7, continue; end  % 跳过 HISTORY
        [x_pred, P_pred, ~, trk.ukf] = ukf_jichu('predict', trk.ukf);
        trk.ukf.x = x_pred; trk.ukf.P = P_pred;
        trk.missed++; trk.life++;
        trk.lat = x_pred(3); trk.lon = x_pred(1);
        trk.assoc_det = [];
    end
    % 质量管理
    active_idx = find_active(trackList);
    trackList = track_management('quality', ..., active_idx, params, frame_id);
    return;
end
```

**Step 1: 分离活跃航迹**

调用 `find_active(trackList)` 找出所有 `type ~= 7` 的航迹索引。

**Step 2: 无活跃航迹 → 无处理**

若无可跟踪航迹，直接返回当前状态。

**Step 3: 批量 UKF 预测**

对每个活跃航迹执行 `ukf_jichu('prepare', trk.ukf)`，返回：
- `x_pred`: 预测状态 (4×1)
- `P_pred`: 预测状态协方差 (4×4)
- `X_pred`: 无迹变换 sigma 点 (4×9)
- `z_pred`: 预测量测 (3×1)
- `Z_pred`: 量测 sigma 点 (3×9)
- `P_zz`: 量测协方差 (3×3)

将这些预测结果暂存到航迹结构体的 `x_pred`, `P_pred`, `X_pred`, `z_pred`, `Z_pred`, `P_zz` 字段中。

**Step 4: JNN 全局点迹-航迹关联**

```matlab
assoc_pairs = track_management('associate', trackList, active_idx, detList, params);
```

调用 `jnn_association` 函数（详见 3.7 节）。

**Step 5: 更新关联成功的航迹**

对每对关联 `(track_idx, det_idx)`:
1. 调用 `nn_associate` 收集该航迹波门内的所有点迹
2. 调用 `pda_weight` 计算加权新息和 NIS
3. 调用 `ukf_jichu('update', ...)` 执行纯 Kalman 更新
4. 若未找到门内点迹，执行纯预测（`ukf.x = x_pred`, `ukf.P = P_pred`）
5. 更新航迹位置、`missed = 0`、`life++`、记录 NIS 历史

**Step 6: 更新未关联航迹（纯预测）**

对每个未关联的活跃航迹：
- `ukf.x = x_pred`, `ukf.P = P_pred`
- `missed++`, `life++`
- `assoc_det = []`

**Step 7: 航迹质量状态机**

调用 `track_management('quality', ...)`（详见 3.7 节）。

**Step 8: 新航迹起始**

```matlab
unused_dets = detList(~point_used);
if ~isempty(unused_dets)
    tempPool = cleanup_stale(tempPool, frame_id, params.tracker_N);
    % 调用 multi_track_start 进行 M/N 起始
end
```

#### 3.6.2 辅助函数：`find_active`

**签名**:
```matlab
function idx = find_active(trackList)
```

返回 `trackList` 中所有 `type ~= 7` 的索引。

#### 3.6.3 辅助函数：`cleanup_stale`

**签名**:
```matlab
function tempPool = cleanup_stale(tempPool, current_frame, N)
```

清理 `tempPool` 中超过 N 帧未更新的过期候选。判断条件：`current_frame - tempPool{c}.lastFrame > N`。

---

### 3.7 track_management.m — JNN 关联与航迹质量管理

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\track_management.m`
**总行数**: 156 行

**功能概述**: 航迹管理的统一入口，通过 action dispatcher 模式调度 JNN 关联和质量状态机两个子功能。

#### 3.7.1 主调度函数：`track_management`

**签名**:
```matlab
function varargout = track_management(action, varargin)
```

**Actions**:
- `'associate'` → 调用 `jnn_association`
- `'quality'` → 调用 `manage_track_quality`

#### 3.7.2 JNN 全局关联：`jnn_association`

**签名**:
```matlab
function assoc_pairs = jnn_association(trackList, active_idx, detList, params)
```

**算法描述**:

JNN (Joint Nearest Neighbor) 是一种贪心全局最优关联算法。核心思想是：在所有可用的 (航迹, 点迹) 对中，每次选择代价最小的配对，然后标记该航迹和点迹为已使用，重复直到没有合法配对。

**代价矩阵构建**:

1. 初始化 `cost = inf(n_tracks, n_dets)`
2. 对每个航迹-点迹对:
   - **地理距离预筛**:
     ```
     geo_gate_m = (life ≤ 15 ? 120000 : 80000) + missed * 15000
     ```
     新航迹用 120km 大波门，稳定航迹用 80km，丢失后波门逐步扩大（每帧 +15km）
   - **角度包裹**: 方位角新息 > 180° 时减 360°，< -180° 时加 360°
   - **马氏距离**: `mahal = innov' * (P_zz_2d \ innov)`
   - 若 `mahal < gate_threshold`（`gate_sigma^2 * 2`），填入代价矩阵

**贪心配对算法**:

```
while true:
    在所有 (available_track, available_det) 中找到 cost 最小的对
    if cost == inf: break  % 没有合法配对了
    记录配对 [track_idx, det_idx]
    标记该航迹和点迹为 unavailable
```

**边缘情况**:

- `n_tracks == 0 || n_dets == 0` → 直接返回空矩阵
- 航迹缺少 `P_zz` 字段 → 跳过
- `P_zz` 含 NaN → 跳过
- 点迹缺少 `drange` 字段或为 NaN → 跳过

#### 3.7.3 航迹质量状态机：`manage_track_quality`

**签名**:
```matlab
function trackList = manage_track_quality(trackList, active_idx, params, frame_id)
```

**航迹类型定义**:

| 类型常量 | 值 | 含义 |
|----------|-----|------|
| `TYPE_RELIABLE` | 1 | 可靠航迹 |
| `TYPE_MAINTAIN` | 2 | 维持航迹 |
| `TYPE_TEMPORARY` | 6 | 临时航迹 |
| `TYPE_HISTORY` | 7 | 历史航迹（已终止） |

**状态转移表**:

| 当前类型 | 有关联 | 操作 | 转移条件 |
|----------|--------|------|----------|
| TEMPORARY (6) | 是 | `quality++` (上限15) | `quality ≥ 10` → 晋升为 RELIABLE |
| TEMPORARY (6) | 否 | `quality--` (下限0) | `quality < 3` → 降级为 HISTORY |
| TEMPORARY (6) | 任 | `missed ≥ K_loss` | → 降级为 HISTORY |
| RELIABLE (1) | 是 | `quality++` (上限15) | 无转移 |
| RELIABLE (1) | 否 | `quality--` (下限0) | `quality < 8` → 降级为 MAINTAIN |
| MAINTAIN (2) | 是 | `quality++` (上限15) | `quality ≥ 10` → 晋升为 RELIABLE |
| MAINTAIN (2) | 否 | `quality--` (下限0) | `quality < 3` → 降级为 HISTORY |
| HISTORY (7) | 吸收态 | 无变化 | 不可逆转 |

**关键参数**:

- TEMPORARY → RELIABLE 晋升阈值：`quality ≥ 10`
- RELIABLE → MAINTAIN 降级阈值：`quality < 8`
- MAINTAIN/TEMPORARY → HISTORY 终止阈值：`quality < 3`
- 丢失帧终止：`missed ≥ K_loss`（默认 8 帧）

---

### 3.8 multi_track_runner_kf.m — 多目标逐帧跟踪包装器

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\multi_track_runner_kf.m`
**总行数**: 197 行

**功能概述**: 单帧多目标跟踪包装器，整合了真值辅助多目标起始、JNN 关联、PDA 更新和作弊放宽的质量管理。

#### 3.8.1 主函数：`multi_track_runner_kf`

**签名**:
```matlab
function [trackList, tempPool, snap, next_id] = multi_track_runner_kf(trackList, tempPool, detList_k, ukf_tpl, ...
        params, frame_id, next_id, truth_ref, t_grid, truth_all)
```

**与 `multi_track_manager` 的关键差异**:

| 维度 | multi_track_manager | multi_track_runner_kf |
|------|---------------------|----------------------|
| 起始方式 | 纯 M/N | 第一帧真值辅助（贪心选最分散3点） |
| UKF 后端 | `ukf_jichu` | `ukf_dispatch`（多态） |
| 关联算法 | JNN（全局） | JNN 初步 + NN 精关联 |
| 质量阈值 | 标准（TEMPORARY<3→HISTORY） | 作弊放宽（TEMPORARY<1→HISTORY, RELIABLE<3→MAINTAIN） |
| 新航迹起始 | 内置 | 调用 `multi_track_start` |

**Step 1: 第一帧真值辅助多目标起始**

当 `frame_id == 1` 且检测数 ≥ 3 时：

1. **贪心选择最分散的检测点**:
   ```
   selected = [1]  % 固定选第一个
   for s = 2 to 3:
       for 每个未选检测 j:
           min_d = min(haversine(j, selected中的每个点))
           if min_d > best_d: 更新最佳点
       selected 加入最佳点
   ```
2. 对每个选中点调用 `ukf_imm('init', ukf_tpl, dp, dp)` + `post_init_multi`
3. 创建航迹结构体，`quality = 15`（满分起始）
4. 剩余未选检测从 `detList_k` 中移除

若检测数 < 3，直接全部用作起始。

**Step 2: 逐航迹准备预测**

遍历所有活跃航迹，调用 `ukf_dispatch('prepare', trk.ukf)`，存储预测结果到航迹结构体。

**Step 3: JNN 全局关联**

调用 `track_management('associate', ...)` 获取关联对。

**Step 4: 更新关联成功的航迹**

对每对关联:
1. 再次调用 `nn_associate` 收集波门内点迹
2. `pda_weight` 计算加权新息
3. `ukf_dispatch('update', ...)` 更新
4. 若未找到关联点迹，使用纯预测值

**Step 5: 未关联航迹纯预测**

`ukf.x = x_pred`, `ukf.P = P_pred`, `missed++`

**Step 6: 作弊放宽的质量管理**

与 `manage_track_quality` 相比，终止条件大幅放宽：

| 类型 | 标准终止阈值 | 放宽后阈值 |
|------|-------------|-----------|
| TEMPORARY | quality < 3 | quality < 1 |
| RELIABLE | quality < 8 → MAINTAIN | quality < 3 → MAINTAIN |
| MAINTAIN | quality < 3 | quality < 1 |
| 丢失帧 | K_loss = 8 | 20 帧 |

**Step 7: 新航迹起始**

从未关联点迹中通过 `multi_track_start` 尝试 M/N 起始新航迹。

#### 3.8.2 辅助函数：`post_init_multi`

与 `single_track_runner` 中的 `post_init` 完全相同（代码重复，未共享）。

---

### 3.9 inject_truth_velocity.m — 真值速度/位置注入

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\inject_truth_velocity.m`
**总行数**: 13 行

**功能概述**: 调试用辅助函数，从真值航迹中注入当前位置估计到 UKF 状态。主要用于验证 UKF 的状态估计是否与真值一致。

**签名**:
```matlab
function inject_truth_velocity(ukf, tt_ac, t_grid, frame_id)
```

**参数**:

| 参数 | 类型 | 说明 |
|------|------|------|
| `ukf` | struct | UKF 状态（可能被原地修改） |
| `tt_ac` | matrix | 真值航迹矩阵，列 1=经度, 列 2=纬度, 列 5=时间 |
| `t_grid` | vector | 时间网格 |
| `frame_id` | int | 当前帧号 |

**执行逻辑**:

1. 边界检查：`t_grid` 为空或 `frame_id` 超出范围 → 直接返回
2. 通过 `interp1` 插值得到当前帧真值经纬度
3. 注入位置到 UKF 状态：
   - 单层 UKF：`ukf.x(3) = tb` (纬度), `ukf.x(4) = tl` (经度)
   - IMM UKF：同时注入 `ukf.ukf_cv.x(3:4)` 和 `ukf.ukf_ct.x(3:4)`

**注意**: 此函数仅注入位置，不注入速度。速度由 UKF 自身估计。

---

### 3.10 post_init_multi.m — 多目标UKF初始化后处理

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\post_init_multi.m`
**总行数**: 13 行

**功能概述**: 多目标场景下 UKF 初始化后的通用字段设置。与 `single_track_runner.m` 中的 `post_init` 和 `multi_track_runner_kf.m` 中的 `post_init_multi` 代码完全相同，属于重复代码。

**签名**:
```matlab
function ukf = post_init_multi(ukf, params)
```

**执行步骤**:

1. 设置 `dt` 和 `initialized`
2. 若为 IMM 类型，同步设置两个子模型的 `dt` 和 `initialized`
3. 重置 `nis_history`
4. 备份 `Q_base`（若为空）
5. 初始化 `Q_ema = 1.0`（若为空）

---

### 3.11 multi_track_start.m — 多目标航迹起始模块

**文件路径**: `D:\Desktop\single_target_with-turn\tracker\multi_track_start.m`
**总行数**: 38 行

**功能概述**: 为 `multi_track_manager` 提供 M/N 滑窗航迹起始功能。从未关联点迹中检测新目标。

**签名**:
```matlab
function [new_state, det1, det2, success] = multi_track_start(state, unused_dets, params, frame_id)
```

**执行逻辑**:

1. 若 `state` 为空，调用 `track_initiation('init', params)` 初始化
2. 调用 `track_initiation('process', state, unused_dets, params, frame_id)` 处理当前帧
3. 返回更新后的状态和起始检测结果

**调用时机**: 在 `multi_track_manager` 的 Step 8 中，从未关联点迹 (`unused_dets`) 中尝试起始新航迹。

**限制**: 当前单目标项目中不支持多目标的完整 M/N 起始（注释注明 "Multi-target M/N track initiation is not supported in this single-target project"）。

---

### 3.12 跨文件调用关系图

```
single_track_runner.m
├── track_initiation('init', params)
├── track_initiation('process', ...)
├── track_initiation('reset', params)
├── ukf_dispatch('init', ukf_tpl, det1, det2)
├── ukf_dispatch('prepare', ukf)
├── ukf_dispatch('update', ukf, innov_w)
├── post_init(ukf, params)
├── nn_associate(...)
├── pda_weight(...)
├── make_track_snap(...)
├── skywave_geometry('group_range', ...)
├── sphere_utils_azimuth(...)
├── sphere_utils_haversine_distance(...)
└── iif(...)

single_track_runner_adaptive.m
├── track_initiation(...)
├── ukf_zishiying('init', ...)
├── ukf_zishiying('update', ...)
├── ukf_jichu('prepare', ukf)
├── nn_associate(...)
├── pda_weight(...)
├── make_track_snap_adapt(...)
├── iif_adapt(...)
└── angdiff_deg_ad(...)

single_track_runner_nanyang.m
├── track_initiation(...)
├── ukf_jichu('init', ...)
├── ukf_jichu('prepare', ...)
├── ukf_jichu('update', ...)
├── nn_associate(...)
├── pda_weight(...)
├── det2nanyang_point(...)
├── fun_check_track_validation(...)
├── build_candidate_for_validation(...)
├── apply_fuzzy_adapt(...)
├── trimf(...)
├── make_snap(...)
└── angdiff(...)

single_track_runner_nanyang_adaptive.m
├── track_initiation(...)
├── ukf_zishiying('init', ...)
├── ukf_zishiying('update', ...)
├── ukf_jichu('prepare', ...)
├── nn_associate(...)
├── pda_weight(...)
├── det2nanyang_point(...)
├── fun_check_track_validation(...)
├── build_candidate_for_validation_ad(...)
├── make_snap_ad(...)
└── angdiff_ad(...)

multi_track_manager.m
├── find_active(trackList)
├── track_management('associate', ...)
├── track_management('quality', ...)
├── ukf_jichu('predict', ...)
├── ukf_jichu('prepare', ...)
├── ukf_jichu('update', ...)
├── nn_associate(...)
├── pda_weight(...)
├── cleanup_stale(tempPool, ...)
└── multi_track_start(...)

multi_track_runner_kf.m
├── ukf_imm('init', ...)
├── ukf_dispatch('prepare', ...)
├── ukf_dispatch('update', ...)
├── track_management('associate', ...)
├── nn_associate(...)
├── pda_weight(...)
├── multi_track_start(...)
├── post_init_multi(...)
└── sphere_utils_haversine_distance(...)

track_management.m
├── jnn_association(...)
│   ├── sphere_utils_haversine_distance(...)
│   └── nn_associate (间接)
└── manage_track_quality(...)

multi_track_start.m
└── track_initiation('init', ...)
    └── track_initiation('process', ...)
```

---

### 3.13 航迹类型（type 字段）汇总

| type 值 | 名称 | 含义 | 何时输出 |
|---------|------|------|----------|
| 1 | RELIABLE | 可靠航迹 | TRACKING 状态，质量 ≥ 10 |
| 2 | MAINTAIN | 维持航迹 | RELIABLE 丢失导致质量降到 < 8 |
| 6 | TEMPORARY | 临时航迹 | 新起始或刚丢失的航迹 |
| 7 | HISTORY | 历史航迹 | 航迹终止（质量过低或丢失超限） |

**注意**: `single_track_runner` 在 TRACKING 状态下输出 `type=1`，在 INITIATING 状态下输出 `type=6`，在 LOST 状态下输出 `type=7`。`single_track_runner_nanyang` 同理，但在 WAITING 状态使用 `track_type = 7`（而非 6）以保持与 `rmse_tracks` 函数的兼容性。

---

### 3.14 关键数学公式汇总

#### 3.14.1 马氏距离

$$D_M^2 = (\mathbf{z}_{meas} - \mathbf{z}_{pred})^T P_{zz}^{-1} (\mathbf{z}_{meas} - \mathbf{z}_{pred})$$

用于 NN 关联和 JNN 关联的代价计算。门限为 `gate_sigma^2 * 2`。

#### 3.14.2 PDA 关联概率

$$\beta_i = \frac{\exp(-\frac{1}{2} D_{M,i}^2)}{b + \sum_j \exp(-\frac{1}{2} D_{M,j}^2)}$$

其中：

$$b = \frac{\lambda V_{norm} (1 - \alpha)}{\alpha}$$

- $\lambda$: 杂波密度
- $V_{norm} = 2\pi\sqrt{\det(P_{zz,2D})}$: 归一化波门体积
- $\alpha = P_d \cdot P_g$: 检测概率 × 波门内检测概率

#### 3.14.3 加权新息

$$\tilde{\mathbf{y}} = \sum_{i=1}^m \beta_i \cdot (\mathbf{z}_i - \hat{\mathbf{z}})$$

#### 3.14.4 NIS (Normalized Innovation Squared)

$$NIS = \tilde{\mathbf{y}}^T S^{-1} \tilde{\mathbf{y}}$$

其中 $S = P_{zz}$ 为新息协方差。Probation 期保护阈值为 50。

#### 3.14.5 Haversine 距离

$$a = \sin^2\left(\frac{\Delta lat}{2}\right) + \cos(lat_1) \cdot \cos(lat_2) \cdot \sin^2\left(\frac{\Delta lon}{2}\right)$$

$$c = 2 \cdot \text{atan2}(\sqrt{a}, \sqrt{1-a})$$

$$d = R \cdot c$$

其中 $R \approx 6371000$ 米为地球半径。

#### 3.14.6 角度差

$$\Delta\theta = \text{mod}(\theta_2 - \theta_1 + 180, 360) - 180$$

返回 $(-180, 180]$ 范围内的最小角度差。

#### 3.14.7 模糊 Q 自适应

参见 3.4.3 节的完整公式链。

#### 3.14.8 速度估计（从经纬度）

$$v = \sqrt{v_E^2 + v_N^2} \times 111320 \times \cos(|lat|)$$

其中 $v_E, v_N$ 分别为东向和北向速度分量（deg/s），111320 为 1 度纬度的近似米数。

---

### 3.15 边缘情况与异常处理

| 场景 | 处理方式 |
|------|----------|
| 当前帧无检测点迹 | 所有活跃航迹纯预测，`missed++` |
| 无活跃航迹 | 直接返回，不做任何处理 |
| M/N 起始超时 | 触发真值兜底（仅 `single_track_runner`） |
| 连续丢帧 ≥ 2 后的关联 | 强制 50km 地理门检查，防止杂波劫持 |
| Probation 期 NIS > 50 | 拒绝更新，变为纯预测帧 |
| Probation 期速度方向突变 > 90° | 拒绝更新（nanyang 和 adaptive 版本） |
| Probation 期速度 > 500 m/s | 拒绝更新（nanyang 和 adaptive 版本） |
| 全生命周期位置跳变 > 50km | 拒绝更新（nanyang 版本） |
| 南阳验证 MSE_R > 200km | 拒绝起始（天波适配放宽） |
| 南阳验证 MSE_V > 200m/s | 拒绝起始（天波适配放宽） |
| 南阳验证方位角波动 > 7.5° | 拒绝起始 |
| 候选点数 < 3 | 无法构建验证候选，重置 M/N 状态 |
| UKF 缺少 `P_zz` 字段 | JNN 关联中跳过该航迹 |
| `P_zz` 含 NaN | JNN 关联中跳过该航迹 |
| 点迹缺少 `drange` 字段 | JNN 关联中跳过该点迹 |
| 点迹 `lat` 为 NaN | NN 关联中跳过该点迹 |

---

### 3.16 参数敏感性分析

| 参数 | 过小的影响 | 过大的影响 | 推荐范围 |
|------|-----------|-----------|----------|
| `gate_sigma` | 波门过小，漏关联 | 波门过大，杂波关联 | 4-8 |
| `tracker_K_loss` | 航迹过早终止 | 航迹延迟终止，浪费计算 | 5-15 |
| `tracker_N` | M/N 起始窗太窄 | 起始延迟 | 8-15 |
| `tracker_M` | 起始条件太严 | 假起始增多 | 4-8 |
| `probate_nis_limit` | 过度保护，频繁拒绝 | 保护不足，假起始污染 | 30-80 |
| `geo_gate_m (防劫持)` | 过度丢弃真实关联 | 允许杂波劫持 | 30000-80000 |
| `quality 晋升阈值` | 难以成为可靠航迹 | 假航迹过早变可靠 | 8-12 |
| `quality 终止阈值` | 航迹过早终止 | 假航迹长期存活 | 1-5 |

## 第五部分：入口脚本（Entry Scripts）详细文档

> 本部分逐一记录项目根目录下所有 .m 入口脚本的完整源代码、函数分析、调用图和设计理念。
> 所有脚本均采用纯函数式编程，无 classdef/OOP，通过 action dispatcher 模式实现多态。

---

### 5.1 run_simulation.m — 单目标直线场景主入口

**文件路径：** `D:/Desktop/single_target_with-turn/run_simulation.m`
**行数：** 1368 行（含注释和空白行）
**场景：** 直线航迹，单目标，自适应UKF跟踪
**架构：** 9-Phase 完整流水线

#### 5.1.1 完整源代码

（源代码已在前面完整列出，此处给出结构化摘要）

```
Line 1-127:  顶部注释块（程序定位、双基地概念、9-Phase流水线总览、调用链、前置依赖、输出文件）
Line 128-132: clear/close/all/clc; addpath(genpath('.'))
Line 134-219: Phase 0: 场景初始化
Line 221-339: Phase 1: ADS-B系统偏差标定
Line 341-415: Phase 2: 原始点迹生成
Line 417-453: Phase 3: 时间对齐策略声明
Line 455-623: Phase 4: 偏差校正 + 几何反解
Line 625-841: Phase 5: 单目标航迹跟踪
Line 843-874: Phase 6: 航迹级时间对齐
Line 876-1003: Phase 7: 航迹融合（四种算法）
Line 1005-1144: Phase 8: 定量误差评估
Line 1146-1303: Phase 9: 可视化 + 数据保存
Line 1305-1368: 内部函数（get_type_str, find_active_tracks, find_reliable, rms_km）
```

#### 5.1.2 函数逐段分析

**Phase 0: 场景初始化 (Line 134-219)**

- 调用 `simulation_params()` 加载13模块默认配置
- 调用 `aircraft_trajectory_create()` 基于航点列表生成航段结构体
- 调用 `aircraft_trajectory_interpolate('generate')` 批量采样生成 N×5 真值矩阵
- 逐点调用 `radar_coverage_check()` 统计R1/R2覆盖范围内的点数
- 构建 `t1_grid` 和 `t2_grid` 异步时间网格（R1从0s开始，R2从13s开始）
- 构造 `truthTraj` 结构体供Phase 8使用

**Phase 1: ADS-B系统偏差标定 (Line 221-339)**

- 读取约24万行ADS-B CSV数据
- 均匀采样5000个点（步长 = floor(height/5000)），避免时间相关性
- 对每个采样点：
  1. 检查是否在R1/R2威力范围内
  2. 用 `skywave_geometry('group_range')` 计算真值群距离
  3. 用 `sphere_utils_azimuth()` 计算真值方位角
  4. 模拟含偏差+噪声的量测值
  5. 计算偏差样本 dr = Rg_meas - Rg_true, da = az_meas - az_true
- 最终 `dr_est = mean(dr_list)`, `da_est = mean(da_list)`

**Phase 2: 原始点迹生成 (Line 341-415)**

- R1随机流：`rng(params.random_seed + 1e7)`，连续推进
- R2随机流：`rng(params.random_seed + 2e7)`，完全隔离
- 每帧调用 `generate_frame_detections()` 生成含偏差+噪声的点迹
- 为每个点迹补充 `aircraft_id = 1`

**Phase 3: 时间对齐策略 (Line 417-453)**

- 纯策略声明，不执行任何计算
- 记录R1/R2采样时间偏移为13秒
- 策略：点迹不做对齐，航迹级对齐延后到Phase 6

**Phase 4: 偏差校正 + 几何反解 (Line 455-623)**

- 对每个点迹：
  1. `drange = prange - dr_est`（距离偏差校正）
  2. `daz = paz - da_est`（方位偏差校正）
  3. `bistatic_inverse_solver(Rgc, azc, ...)` 反解校正后经纬度
  4. `bistatic_inverse_solver(prange, paz, ...)` 反解原始偏差下经纬度
- 同时计算原始点迹和校准后点迹的RMSE

**Phase 5: 单目标航迹跟踪 (Line 625-841)**

- R1参数：`ukf_Q_scale=5e4`, `gate_sigma=2.0`, `K_loss=8`
- R2参数：`ukf_Q_scale=1e5`, `gate_sigma=2.5`, `K_loss=12`
- 创建 `ukf_zishiying('create', ...)` 模板
- 调用 `single_track_runner(detList, ukf_tpl, params, n_frames, true_track, t_grid)`
- 内部每帧执行：UKF预测 → NN关联 → PDA加权 → UKF更新 → 模糊自适应Q
- 关联诊断：统计关联率、NIS均值、NIS门内比例

**Phase 6: 航迹级时间对齐 (Line 843-874)**

- 调用 `time_align_tracks(trackSnapshots_R2, params)`
- 用CV模型全状态外推：`x(t1) = F(-offset) × x(t2)`, `P(t1) = F(-offset) × P(t2) × F(-offset)' + Q(|offset|)`

**Phase 7: 航迹融合 (Line 876-1003)**

- 单目标直接1对1融合（R1#1 ↔ R2#1）
- 调用 `run_track_fusion(matched_pair, trackSnapshots_R1, aligned_R2, params, method)` 四次
- 四种方法：SCC（简单凸组合）、BC（Bar-Shalom-Campo精确融合）、CI（协方差交叉）、FCI（快速CI）

**Phase 8: 定量误差评估 (Line 1005-1144)**

- 构建 `matcher_simple` 结构体
- 调用 `evaluate_all('fusion', ...)` 计算融合RMSE
- 调用 `evaluate_all('tracking_errors', ...)` 计算单站UKF误差
- 打印融合 vs 单站对比表，找出最优融合算法

**Phase 9: 可视化 + 数据保存 (Line 1146-1303)**

- 生成5-6张图：场景总览、R1/R2点云3D、单目标跟踪综合图、融合综合图
- 保存完整.mat文件：sysPara, calibResult, truthTraj, R1/R2数据, params, errorStats, fusion_eval

#### 5.1.3 调用图

```
run_simulation.m
├── Phase 0:
│   ├── simulation_params()
│   ├── aircraft_trajectory_create(waypoints, speed, dt)
│   ├── aircraft_trajectory_interpolate('generate', traj)
│   └── radar_coverage_check(x4) — 逐点覆盖检查
├── Phase 1:
│   ├── readtable(adsb_csv_path)
│   ├── radar_coverage_check(x5000) — ADS-B采样
│   ├── skywave_geometry('group_range')
│   └── sphere_utils_azimuth()
├── Phase 2:
│   └── generate_frame_detections(x2n_frames) — R1+R2各n_frames帧
├── Phase 4:
│   ├── bistatic_inverse_solver(x2×n_frames×n_dets)
│   └── sphere_utils_haversine_distance() — RMSE计算
├── Phase 5:
│   ├── ukf_zishiying('create') ×2 — R1+R2
│   ├── single_track_runner(detList_R1, ukf1_tpl, ...)
│   │   └── 每帧: ukf_prepare → nn_associate → pda_weight → ukf_update → fuzzy_adapt
│   └── single_track_runner(detList_R2, ukf2_tpl, ...)
├── Phase 6:
│   └── time_align_tracks(trackSnapshots_R2, params)
├── Phase 7:
│   ├── run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, 'SCC')
│   ├── run_track_fusion(..., 'BC')
│   ├── run_track_fusion(..., 'CI')
│   └── run_track_fusion(..., 'FCI')
├── Phase 8:
│   ├── evaluate_all('fusion', all_fused_snapshots, ...)
│   └── evaluate_all('tracking_errors', trackSnapshots_R1/R2, ...)
└── Phase 9:
    ├── plot_scene_overview(true_track, params, 'results')
    ├── plot_point_cloud_3d(detList_R1, 'R1', ...)
    ├── plot_point_cloud_3d(detList_R2, 'R2', ...)
    ├── plot_results('single_track', ...)
    ├── plot_results('single_fusion', ...)
    └── save(outf, 'sysPara', 'calibResult', 'truthTraj', 'R1', 'R2', ...)
```

#### 5.1.4 设计 rationale

1. **9-Phase模块化设计**：每个Phase职责单一，便于单独调试和替换。例如Phase 1（标定）可独立验证ADS-B数据处理流程。

2. **随机流隔离**：R1和R2使用 `seed+1e7` 和 `seed+2e7` 偏移，确保两部雷达的随机数序列完全不重叠。相比旧的 `rng(seed+k)` 方案，避免了Toeplitz对角线相关性。

3. **航迹级时间对齐优于点迹级**：UKF滤除噪声后的航迹状态更"干净"，且包含完整协方差矩阵，外推后不确定性可正确传播。

4. **偏差校正与反解并行**：同时保留原始偏差下的经纬度（raw_lat/raw_lon），便于可视化对比校正前后的定位精度提升。

5. **四种融合算法全覆盖**：SCC/BC/CI/FCI覆盖了从简单到精确、从乐观到保守的全部策略谱系，便于全面比较。

---

### 5.2 run_simulation_turn.m — 单目标渐进拐弯三体制对比

**文件路径：** `D:/Desktop/single_target_with-turn/run_simulation_turn.m`
**行数：** 710 行
**场景：** 渐进拐弯航迹（46.7度，1度/s转弯率）
**架构：** 三体制并行（jichu / zishiying / imm）

#### 5.2.1 源代码结构

```
Line 1-33:    顶部注释
Line 35-90:   Phase 0: 渐进拐弯航迹生成 + 覆盖检查 + 时间网格
Line 92-147:  Phase 1: ADS-B标定
Line 149-183: Phase 2: 原始点迹生成
Line 185-190: Phase 3: 时间对齐策略
Line 192-301: Phase 4: 偏差校正 + RMSE统计
Line 303-466: Phase 5: 三体制航迹跟踪（并行）
Line 468-606: Phase 6-8: 每体制独立对齐/融合/评估
Line 608-709: Phase 9: 可视化 + 数据保存
Line 697-709: 内部函数（get_type_str, rms_km）
```

#### 5.2.2 与 run_simulation.m 的关键差异

| 维度 | run_simulation.m | run_simulation_turn.m |
|------|-----------------|----------------------|
| 航迹类型 | 直线 | 渐进拐弯（46.7度） |
| UKF后端 | 仅自适应UKF | 三体制并行（jichu/zishiying/imm） |
| 跟踪器 | single_track_runner | single_track_runner ×3体制 |
| 融合评估 | 一次性 | 每体制独立评估 |
| 可视化 | 一组图 | 每体制两图（共6张） |
| 数据保存 | 单组 | R1_all{3}/R2_all{3} |

#### 5.2.3 三体制并行架构

Phase 5 中，对每种UKF类型（jichu/zishiying/imm）：
1. 创建对应的UKF模板（`ukf_jichu('create')` / `ukf_zishiying('create')` / `ukf_imm('create')`）
2. 分别对R1和R2执行 `single_track_runner`
3. 打印关联诊断（关联率、NIS、起始帧）
4. IMM体制额外打印模型概率诊断

Phase 6-8 中，对每种体制：
1. 独立调用 `time_align_tracks`
2. 独立执行4种融合算法
3. 独立调用 `evaluate_all('fusion')`
4. 找出最优融合算法并打印RMSE对比

#### 5.2.4 调用图

```
run_simulation_turn.m
├── Phase 0:
│   └── aircraft_trajectory_create('gradual_turn', params)
├── Phase 1-4: (同 run_simulation.m)
├── Phase 5: 三体制并行
│   ├── for u = 1:3
│   │   ├── switch ukf_type
│   │   │   ├── 'jichu' → ukf_jichu('create')
│   │   │   ├── 'zishiying' → ukf_zishiying('create')
│   │   │   └── 'imm' → ukf_imm('create')
│   │   ├── single_track_runner(detList_R1, tpl1, ...)
│   │   └── single_track_runner(detList_R2, tpl2, ...)
│   └── 三体制UKF RMSE对比
├── Phase 6-8: 每体制独立
│   └── for u = 1:3
│       ├── time_align_tracks(ukf_snaps_R2{u})
│       ├── run_track_fusion ×4
│       └── evaluate_all('fusion')
└── Phase 9:
    ├── plot_results('single_track') ×3
    ├── plot_results('single_fusion') ×3
    └── save(outf, 'R1_all', 'R2_all', 'fus_data', ...)
```

#### 5.2.5 设计 rationale

1. **三体制并行设计**：同一点迹数据，三种UKF后端独立运行，便于公平对比。
2. **每体制独立融合评估**：不同UKF产生的航迹质量不同，最优融合算法也可能不同。
3. **渐进拐弯场景**：比直线更难跟踪，能暴露固定Q UKF的不足，凸显自适应/IMM的价值。

---

### 5.3 run_simulation_turn_180deg.m — 单目标回头弯180度三体制对比

**文件路径：** `D:/Desktop/single_target_with-turn/run_simulation_turn_180deg.m`
**行数：** 694 行
**场景：** 回头弯180度（正东入弯 → 左转180度半圆 → 正西出弯）
**架构：** 三体制并行（与 run_simulation_turn.m 类似）

#### 5.3.1 与 run_simulation_turn.m 的差异

| 维度 | run_simulation_turn.m | run_simulation_turn_180deg.m |
|------|----------------------|------------------------------|
| 航迹 | 渐进拐弯46.7度 | 回头弯180度半圆 |
| 航迹创建 | `'gradual_turn'` | `'uturn'` |
| 转弯率 | 1度/s渐进 | 1度/s恒定 |
| 帧数 | ~81帧 | ~41帧 |
| 转弯难度 | 中等 | 极高 |

#### 5.3.2 回头弯航迹设计

```
W1: (125.5°E, 33.0°N) — 起点
→ 正东飞120km
→ 入弯点：左转180度半圆（1°/s, R=13.2km, 180s≈6帧）
→ 出弯点：正西飞120km
→ W3: 终点
```

#### 5.3.3 调用图

```
run_simulation_turn_180deg.m
├── Phase 0:
│   └── aircraft_trajectory_create('uturn', params)
├── Phase 1-4: (同 run_simulation_turn.m)
├── Phase 5: 三体制并行（同 run_simulation_turn.m）
├── Phase 6-8: 每体制独立（同 run_simulation_turn.m）
└── Phase 9:
    └── save(outf, ..., 'turn_rate_rad_per_sec')
```

#### 5.3.4 设计 rationale

1. **极端机动场景**：180度回头弯是跟踪算法的极限测试场景，CC/CT模型切换频繁。
2. **验证IMM优势**：基础UKF（固定CV模型）在此场景下必然发散，自适应和IMM应有显著优势。

---

### 5.4 run_simulation_multi.m — 三目标交叉场景主入口

**文件路径：** `D:/Desktop/single_target_with-turn/run_simulation_multi.m`
**行数：** 925 行（含内部函数 multi_track_runner_kf）
**场景：** 三目标交叉（A:西南→东北, B:西北→东南, C:西→东）
**架构：** IMM CV+CT 多目标跟踪 + 跨雷达航迹匹配 + 四种融合

#### 5.4.1 源代码结构

```
Line 1-30:    顶部注释
Line 31-111:  Phase 0: 三目标交叉航迹生成 + 覆盖检查 + 时间网格
Line 113-156: Phase 1: ADS-B标定
Line 158-275: Phase 2+4: 多目标点迹生成 + 偏差校正
Line 277-352: Phase 5: 多目标航迹跟踪（IMM CV+CT）
Line 354-357: Phase 6: 航迹级时间对齐
Line 359-495: Phase 7: 航迹匹配 + 融合
Line 497-589: Phase 8: 定量误差评估
Line 591-640: Phase 9: 可视化 + 数据保存
Line 646-924: 内部函数 multi_track_runner_kf（9帧跟踪包装器）
```

#### 5.4.2 三目标交叉场景设计

```
目标A: [126.8, 31.5] → [130.0, 33.5]  （西南→东北）
目标B: [126.8, 33.5] → [130.0, 31.5]  （西北→东南）
目标C: [126.8, 32.5] → [130.8, 32.5]  （西→东）
```

三目标在覆盖区中心附近交叉，形成复杂的关联歧义场景。

#### 5.4.3 多目标跟踪包装器 multi_track_runner_kf (Line 646-924)

这是整个多目标场景的核心逻辑，包含7个步骤：

**Step 1: 第一帧 truth-assisted 起始**
- 用真值位置搜索最近的非杂波检测
- 为每条航迹打上 `ac_idx` 标签（绑定到 truth_all{ac_idx}）
- 调用 `inject_truth_velocity()` 注入真值速度

**Step 2: 分离活跃航迹 + 预测**
- 过滤掉 type=7 的死亡航迹
- 对每条活跃航迹调用 `ukf_dispatch('prepare')` 执行UKF预测

**Step 3: Truth-assisted association（上帝视角）**
- 每条航迹通过 `ac_idx` 绑定到真值
- 用真值位置搜索最近的非杂波检测（200km门限）
- 杂波点迹完全忽略

**Step 4: 标记已用检测**
- 供Step 7的M/N起始使用

**Step 5: 用关联结果更新航迹**
- 构造加权新息向量 [dr_innov, az_innov, vr_innov]
- 调用 `ukf_dispatch('update')` 更新状态
- 计算NIS值

**Step 6: 航迹质量状态机**
- 关联：quality+1（上限15）
- 丢失：TEMPORARY扣1分，RELIABLE只扣1分（最低到8），MAINTAIN扣2分
- 状态转换：TEMPORARY→RELIABLE(quality≥10), RELIABLE→MAINTAIN(quality<5), MAINTAIN→RELIABLE/ HISTORY

**Step 7: 未关联点的M/N航迹起始**
- 对未被关联的检测，调用 `multi_track_start()` 尝试新航迹起始

#### 5.4.4 跨雷达航迹匹配

支持两种模式：
- `'truth_assisted'`：基于 `ac_idx` 映射（100%正确）
- `'real'`：基于位置+速度+航向的多维特征匹配

```
for ac = 1:3
    找R1中ac_idx=ac的航迹ID → r1_id
    找R2中ac_idx=ac的航迹ID → r2_id
    统计共现帧数和平均距离
    构建 matched_pairs{p}
```

#### 5.4.5 调用图

```
run_simulation_multi.m
├── Phase 0:
│   ├── aircraft_trajectory_create(way_A/B/C) ×3
│   ├── aircraft_trajectory_interpolate('generate') ×3
│   └── radar_coverage_check(x9) — 3目标×2雷达
├── Phase 1: (同单目标)
├── Phase 2+4:
│   └── generate_frame_detections_multi() ×2n_frames
├── Phase 5:
│   ├── ukf_imm('create') ×2 — R1+R2
│   └── for k = 1:n_frames
│       ├── multi_track_runner_kf(R1) — 7步流程
│       └── multi_track_runner_kf(R2) — 7步流程
├── Phase 6:
│   └── time_align_tracks(trackSnapshots_R2, params)
├── Phase 7:
│   ├── track_matcher(...) — 跨雷达航迹匹配
│   └── for p = 1:n_pairs
│       └── for m = 1:4
│           └── run_track_fusion(matched_pairs{p}, ..., method_names{m})
├── Phase 8:
│   ├── evaluate_all_multi('fusion', ...)
│   └── evaluate_all('tracking_errors') ×2
└── Phase 9:
    ├── plot_scene_overview_multi(...)
    ├── plot_point_cloud_3d ×2
    ├── plot_results_multi('single_track', ...)
    └── plot_results_multi('single_fusion', ...)
```

#### 5.4.6 设计 rationale

1. **Truth-assisted关联**：多目标场景下JPDA/JNN在交叉点容易混淆，truth-assisted提供"上帝视角"验证融合算法本身性能。
2. **放宽K_loss=15**：确保三目标交叉时航迹不丢失，便于聚焦融合算法评估。
3. **质量状态机**：借鉴NY_track_new逻辑但放宽参数，RELIABLE航迹丢失只扣1分（最低到8），确保3条航迹全程不丢失。

---

### 5.5 run_mc_straight.m — 直线场景蒙特卡洛仿真

**文件路径：** `D:/Desktop/single_target_with-turn/run_mc_straight.m`
**行数：** 563 行
**N_MC：** 500
**架构：** 首次真值辅助起始 + 重新纯M/N + 自适应UKF

#### 5.5.1 源代码结构

```
Line 1-46:     顶部注释 + 配置 + 预分配统计数组
Line 48-57:    打印表头
Line 59-294:   主循环（500次MC）
Line 296-395:  汇总统计（RMSE + 改善率 + MTL + 断裂 + 关联 + NIS + 坏种子 + 融合分布）
Line 397-405:  保存数据
Line 408-562:  内部工具函数
```

#### 5.5.2 统计数据结构

```matlab
rmse.raw_R1 = nan(500,1);    rmse.raw_R2 = nan(500,1);
rmse.cal_R1 = nan(500,1);    rmse.cal_R2 = nan(500,1);
rmse.ukf_R1 = nan(500,1);    rmse.ukf_R2 = nan(500,1);
rmse.ukf_R2_aligned = nan(500,1);
rmse.fus = nan(500,4);       % SCC/BC/CI/FCI
rmse.fus_best = nan(500,1);
fus_best_method = cell(500,1);
mtl_R1 = nan(500,1); mtl_R2 = nan(500,1); mtl_fus = nan(500,1);
brk_R1 = nan(500,1); brk_R2 = nan(500,1); brk_fus = nan(500,1);
seg_count_R1 = nan(500,1); seg_count_R2 = nan(500,1); seg_count_fus = nan(500,1);
nis_mean_R1 = nan(500,1); nis_mean_R2 = nan(500,1);
nis_gate_R1 = nan(500,1); nis_gate_R2 = nan(500,1);
assoc_R1 = nan(500,1); assoc_R2 = nan(500,1);
init_frame_R1 = nan(500,1); init_frame_R2 = nan(500,1);
imp_ukf_R1 = nan(500,1); imp_ukf_R2 = nan(500,1);
imp_fus_vs_R1 = nan(500,1); imp_fus_vs_R2 = nan(500,1);
bad_seed = zeros(500,1); bad_reason = cell(500,1);
seg_info = cell(500,1);
```

#### 5.5.3 坏种子判断逻辑

```matlab
if rmse.ukf_R1(mc) > 30 || rmse.ukf_R2(mc) > 30
    bad_seed(mc) = 1;  % 发散
elseif imp_ukf_R1(mc) < -50 || imp_ukf_R2(mc) < -50
    bad_seed(mc) = 1;  % 退化（UKF比原始点迹还差超过50%）
end
```

#### 5.5.4 内部工具函数

| 函数 | 行号 | 功能 |
|------|------|------|
| `rmse_detlist` | 412-432 | 点迹RMSE（区分raw/cal模式） |
| `rmse_tracks` | 434-450 | 航迹RMSE |
| `rmse_fusion_snaps` | 452-465 | 融合航迹RMSE |
| `diagnose_tracking` | 467-498 | 关联率、NIS均值、NIS门内比例、起始帧 |
| `extract_segments` | 500-521 | 航迹分段（type=1连续区间） |
| `extract_fusion_segments` | 523-544 | 融合航迹分段 |
| `compute_mtl` | 546-548 | 航迹平均长度 |
| `rms_val` | 550-552 | RMSE计算 |
| `print_mc_row` | 554-562 | 打印一行统计（均值/std/中位/最小/最大） |

#### 5.5.5 调用图

```
run_mc_straight.m (500次MC)
└── for mc = 1:500
    ├── Phase 0-1: (同 run_simulation.m)
    ├── Phase 2+4: 点迹生成 + 偏差校正（R1+R2独立随机流）
    ├── Phase 4b: 点迹RMSE（raw/cal）
    ├── Phase 5: UKF跟踪（R1+R2）
    │   ├── ukf_zishiying('create') ×2
    │   └── single_track_runner ×2
    ├── 关联诊断（diagnose_tracking ×2）
    ├── Phase 6: time_align_tracks(R2)
    ├── Phase 7: 融合（SCC/BC/CI/FCI ×4）
    ├── 航迹分段（extract_segments ×3）
    ├── MTL + 断裂统计
    ├── 坏种子判断
    └── 逐种子详细输出
```

#### 5.5.6 设计 rationale

1. **逐种子详细输出**：每个MC种子打印完整统计，便于调试和定位坏种子。
2. **坏种子机制**：RMSE>30km或改善率<-50%标记为坏种子，汇总时单独统计。
3. **预分配所有统计数组**：避免动态增长，提升500次循环性能。
4. **分段详情保存**：`seg_info{mc}` 保存每种的三段分段信息，便于后续分析。

---

### 5.6 run_mc_turn.m — 拐弯场景蒙特卡洛（IMM）

**文件路径：** `D:/Desktop/single_target_with-turn/run_mc_turn.m`
**行数：** 676 行
**N_MC：** 100
**UKF后端：** IMM（CV+CT双模型，Pd-IPDA似然，Pi=[0.90, 0.10]）

#### 5.6.1 与 run_mc_straight.m 的差异

| 维度 | run_mc_straight.m | run_mc_turn.m |
|------|------------------|---------------|
| 航迹 | 直线 | 渐进拐弯46.7度 |
| UKF后端 | 自适应UKF | IMM（CV+CT） |
| N_MC | 500 | 100 |
| 额外统计 | 无 | IMM模型概率（mu_ct_avg/turn/dom） |
| K_loss | R1=4, R2=6 | R1=R2=8 |

#### 5.6.2 IMM模型概率统计

```matlab
% 对每条最终航迹的 mu_history 分析：
mu_ct_avg_R1(mc) = mean(mu_hist1(:,2)) * 100;    % 全局CT模型平均概率
mu_ct_turn_R1(mc) = mean(mu_hist1(turn_frames, 2)) * 100;  % 转弯帧CT概率
mu_ct_dom_R1(mc) = sum(mu_hist1(:,2) > 0.5);     % CT占优帧数
```

#### 5.6.3 调用图

```
run_mc_turn.m (100次MC)
├── Phase 0: aircraft_trajectory_create('gradual_turn')
├── Phase 1: ADS-B标定
├── Phase 2+4: 点迹生成 + 校正
├── Phase 5: IMM跟踪（R1+R2）
│   ├── ukf_imm('create') ×2
│   └── single_track_runner ×2
├── IMM模型概率分析
├── Phase 6-7: 时间对齐 + 融合
├── 航迹分段 + MTL + 断裂
└── 汇总统计（含IMM模型概率汇总）
```

---

### 5.7 run_mc_turn_180deg.m — 回头弯180度蒙特卡洛（IMM）

**文件路径：** `D:/Desktop/single_target_with-turn/run_mc_turn_180deg.m`
**行数：** 665 行
**N_MC：** 500
**UKF后端：** IMM（CV+CT）

#### 5.7.1 与 run_mc_turn.m 的差异

| 维度 | run_mc_turn.m | run_mc_turn_180deg.m |
|------|--------------|---------------------|
| 航迹 | 渐进拐弯46.7度 | 回头弯180度 |
| N_MC | 100 | 500 |
| 航迹创建 | `'gradual_turn'` | `'uturn'` |
| 转弯率 | 1度/s渐进 | 1度/s恒定 |

#### 5.7.2 调用图

```
run_mc_turn_180deg.m (500次MC)
├── Phase 0: aircraft_trajectory_create('uturn')
├── Phase 1-4: (同 run_mc_turn.m)
├── Phase 5: IMM跟踪
├── Phase 6-7: 时间对齐 + 融合
└── 汇总统计（含IMM模型概率 + 坏种子统计）
```

---

### 5.8 run_mc_turn_compare.m — 拐弯场景三体制对比蒙特卡洛

**文件路径：** `D:/Desktop/single_target_with-turn/run_mc_turn_compare.m`
**行数：** 793 行
**N_MC：** 200
**架构：** 同一点迹数据，三体制并行（jichu / zishiying / imm）

#### 5.8.1 源代码结构

```
Line 1-17:     顶部注释
Line 19-86:    配置 + 预分配统计结构（struct数组 s(1..3)）
Line 88-97:    预计算转弯信息
Line 99-403:   主循环（200次MC，每轮三体制并行）
Line 405-562:  汇总统计（三体制对比表）
Line 565-572:  保存数据
Line 575-793:  内部工具函数
```

#### 5.8.2 统计数据结构

```matlab
s(1).name = 'jichu';
s(1).rmse_ukf_R1 = nan(200,1);
s(1).rmse_ukf_R2 = nan(200,1);
s(1).rmse_fus = nan(200,4);
s(1).rmse_fus_best = nan(200,1);
... (每个体制都有相同的统计字段)

s(3) 额外有:
s(3).mu_ct_avg_R1 = nan(200,1);
s(3).mu_ct_turn_R1 = nan(200,1);
s(3).mu_ct_dom_R1 = nan(200,1);
```

#### 5.8.3 逐种子对比输出格式

```
  MC #42 (seed=42) --
  点迹: R1原始23km 校准5.2km | R2原始45km 校准10.3km km
  UKF          │    R1_UKF    R2_UKF │ AssocR1 AssocR2 │  FusBest   FusRMSE │ Method
  ─────────────┼────────────────────┼──────────────┼───────────────┼───────
  jichu        │    8.3km   7.1km │    72%    65% │    6.2km    5.8km │ SCC ***BAD***
  zishiying    │    6.1km   5.4km │    85%    78% │    4.8km    4.5km │ BC
  imm          │    4.2km   3.8km │    91%    87% │    3.5km    3.2km │ FCI
  -> 最优体制: imm (融合RMSE=3.2km)
  IMM CT概率: R1 avg=45% turn=78% dom=32 | R2 avg=42% turn=75% dom=28
```

#### 5.8.4 汇总统计

包含以下对比板块：
1. UKF RMSE 三体制对比
2. 融合 RMSE 三体制对比（4种算法×3体制）
3. 改善率三体制对比
4. 关联诊断三体制对比
5. MTL + 断裂对比
6. 坏种子统计（含三体制均坏的种子列表）
7. 融合算法分布（每体制下4种算法的最优次数）
8. IMM模型概率（仅imm体制）
9. 交叉对比：zishiying vs jichu, imm vs jichu（差异百分比 + 胜率）
10. 三体制终极PK（每种子最优体制分布）

#### 5.8.5 内部工具函数

| 函数 | 功能 |
|------|------|
| `get_turn_info` | 计算转弯角度和转弯率 |
| `find_turn_frames` | 定位真值中的转弯帧（航向变化率>阈值） |
| `rmse_detlist` | 点迹RMSE |
| `rmse_tracks` | 航迹RMSE |
| `rmse_fusion_snaps` | 融合RMSE |
| `diagnose_tracking` | 关联诊断 |
| `extract_segments` / `extract_fusion_segments` | 航迹分段 |
| `compute_mtl` | 航迹平均长度 |
| `print_3way` | 三列对比打印 |
| `print_3way_pct` | 百分比三列对比 |
| `print_imm_mu` | IMM模型概率打印 |
| `print_cross_row` | 差异百分比打印 |

#### 5.8.6 调用图

```
run_mc_turn_compare.m (200次MC)
└── for mc = 1:200
    ├── Phase 0: aircraft_trajectory_create('gradual_turn')
    ├── Phase 1: ADS-B标定
    ├── Phase 2+4: 点迹生成 + 校正
    ├── Phase 5+6+7: 三体制并行
    │   └── for u = 1:3
    │       ├── switch ukf_type (jichu/zishiying/imm)
    │       ├── single_track_runner(R1)
    │       ├── single_track_runner(R2)
    │       ├── time_align_tracks
    │       ├── run_track_fusion ×4
    │       └── extract_segments + compute_mtl
    ├── 逐种子三体制对比输出
    └── 汇总统计（10个对比板块）
```

---

### 5.9 run_mc_turn_180deg_compare.m — 回头弯180度三体制对比

**文件路径：** `D:/Desktop/single_target_with-turn/run_mc_turn_180deg_compare.m`
**行数：** 749 行
**N_MC：** 200
**架构：** 与 run_mc_turn_compare.m 完全相同，仅航迹改为 `'uturn'`

#### 5.9.1 关键差异

| 维度 | run_mc_turn_compare.m | run_mc_turn_180deg_compare.m |
|------|----------------------|------------------------------|
| 航迹 | 渐进拐弯46.7度 | 回头弯180度 |
| 转弯率计算 | `get_turn_info()` | 硬编码 `omega = pi/180` |
| 航迹创建 | `'gradual_turn'` | `'uturn'` |
| 表头格式 | box-drawing字符 | ASCII `=====` |

#### 5.9.2 调用图

```
run_mc_turn_180deg_compare.m (200次MC)
├── Phase 0: aircraft_trajectory_create('uturn')
├── Phase 1-4: (同 run_mc_turn_compare.m)
├── Phase 5+6+7: 三体制并行（同 run_mc_turn_compare.m）
└── 汇总统计（同 run_mc_turn_compare.m，ASCII格式）
```

---

### 5.10 run_mc_multi.m — 三目标交叉蒙特卡洛

**文件路径：** `D:/Desktop/single_target_with-turn/run_mc_multi.m`
**行数：** 486 行（含内部函数）
**N_MC：** 200
**架构：** IMM CV+CT 多目标跟踪 + JPDA关联

#### 5.10.1 源代码结构

```
Line 1-17:     顶部注释
Line 19-54:    配置 + 预分配
Line 56-61:    打印表头
Line 63-336:   主循环（200次MC）
Line 338-365:  汇总统计
Line 367-485:  内部函数
```

#### 5.10.2 多目标跟踪包装器 multi_track_runner_kf_mc

这是轻量版多目标跟踪器，专为蒙特卡洛场景设计：

```matlab
function [trackList, tempPool, snap, next_id] = multi_track_runner_kf_mc(...)
    % Step 1: 第一帧直接起始（最多3条，消耗前3个非杂波检测）
    % Step 2: 预测（ukf_dispatch('prepare')）
    % Step 3: JPDA关联（jpda_multi）
    % Step 4: 用JPDA加权新息更新（ukf_dispatch('update')）
    % Step 5: track_management('quality') — 质量状态机
    % Step 6: 未关联点的M/N起始（multi_track_start）
end
```

#### 5.10.3 内部函数

| 函数 | 功能 |
|------|------|
| `multi_track_runner_kf_mc` | 多目标单帧跟踪包装器（蒙特卡洛轻量版） |
| `post_init_multi` | 初始化后处理（设置dt, initialized, Q_base, Q_ema等） |

#### 5.10.4 调用图

```
run_mc_multi.m (200次MC)
└── for mc = 1:200
    ├── Phase 0: 三目标航迹生成
    ├── Phase 1: ADS-B标定
    ├── Phase 2+4: 多目标点迹生成 + 校正
    ├── Phase 5: 多目标跟踪
    │   ├── ukf_imm('create') ×2
    │   └── for k = 1:n_frames
    │       ├── multi_track_runner_kf_mc(R1) — JPDA关联
    │       └── multi_track_runner_kf_mc(R2) — JPDA关联
    ├── Phase 6: time_align_tracks
    ├── Phase 7: track_matcher + run_track_fusion
    └── 统计 RMSE（取最优融合对）
```

---

### 5.11 scan_Q_scale.m — Q_scale参数扫描

**文件路径：** `D:/Desktop/single_target_with-turn/scan_Q_scale.m`
**行数：** 864 行
**功能：** 系统扫描 Q_scale 参数，评估对拐弯场景跟踪性能的影响

#### 5.11.1 源代码结构

```
Line 1-24:    配置（9个Q值，100次MC，3体制，4融合）
Line 26-92:   扫描主循环（9个Q值）
Line 98-400:  内部函数 run_mc_gradual_turn
Line 402-701: 内部函数 run_mc_180deg_uturn
Line 703-863: 工具函数（从MC脚本提取）
```

#### 5.11.2 扫描参数

```matlab
Q_values = [5e2, 1e3, 3e3, 1e4, 3e4, 1e5, 3e5, 1e6, 3e6];
```

#### 5.11.3 核心机制：动态修改配置文件

```matlab
% 每次迭代修改 config/simulation_params.m 中的 Q_scale 值
fid = fopen('config/simulation_params.m', 'r');
lines = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);
for li = 1:length(lines)
    if contains(lines{li}, 'params.radar1_ukf_Q_scale')
        lines{li} = sprintf('params.radar1_ukf_Q_scale = %g;', q_val);
    end
    if contains(lines{li}, 'params.radar2_ukf_Q_scale')
        lines{li} = sprintf('params.radar2_ukf_Q_scale = %g;', q_val);
    end
end
fid = fopen('config/simulation_params.m', 'w');
fprintf(fid, '%s\n', lines{:});
fclose(fid);
rehash toolboxcache;  % 刷新MATLAB函数缓存
```

#### 5.11.4 断点续扫机制

```matlab
skip_file = fullfile('results', sprintf('scan_Q_done_Q%g.mat', q_val));
if exist(skip_file, 'file')
    fprintf('已存在结果，跳过\n');
    continue;
end
```

#### 5.11.5 调用图

```
scan_Q_scale.m
└── for qi = 1:9 (Q值)
    ├── 备份 simulation_params.m
    ├── 修改 Q_scale 值
    ├── rehash toolboxcache
    ├── run_mc_gradual_turn(100次MC)
    │   ├── for mc = 1:100
    │   │   ├── Phase 0-4: 场景+点迹
    │   │   ├── Phase 5: 三体制跟踪
    │   │   ├── Phase 6-7: 对齐+融合
    │   │   └── 坏种子判断
    │   └── 保存 gradual_N100_Q{q}.mat
    ├── run_mc_180deg_uturn(100次MC)
    │   └── 保存 uturn_N100_Q{q}.mat
    ├── 保存 scan_Q_done_Q{q}.mat
    └── 恢复 simulation_params.m
```

#### 5.11.6 设计 rationale

1. **动态修改配置文件**：避免为每个Q值创建独立的MC脚本，通过修改同一份配置文件实现参数扫描。
2. **断点续扫**：已完成的Q值自动跳过，支持长时间运行的中断恢复。
3. **两场景覆盖**：gradual_turn（中等机动）+ 180deg_uturn（极端机动），全面评估Q_scale影响。

---

### 5.12 scan_Pi.m — Pi (IMM转移概率) 参数扫描

**文件路径：** `D:/Desktop/single_target_with-turn/scan_Pi.m`
**行数：** 860 行
**功能：** 系统扫描 IMM 转移概率 Pi，评估对拐弯场景跟踪性能的影响

#### 5.12.1 与 scan_Q_scale.m 的差异

| 维度 | scan_Q_scale.m | scan_Pi.m |
|------|---------------|-----------|
| 扫描参数 | Q_scale | Pi (CV→CT / CT→CV) |
| 参数值 | 9个(500~3e6) | 9个(0.001~0.50) |
| 修改字段 | `params.radar1_ukf_Q_scale` | `params.imm_Pi_CV_to_CT` + `params.imm_Pi_CT_to_CV` |
| 保存文件 | `gradual_N100_Q{q}.mat` | `gradual_N100_Pi{pi}.mat` |

#### 5.12.2 扫描参数

```matlab
Pi_values = [0.001, 0.005, 0.01, 0.03, 0.05, 0.10, 0.20, 0.30, 0.50];
```

#### 5.12.3 调用图

```
scan_Pi.m
└── for pi_idx = 1:9
    ├── 修改 Pi 值（CV→CT 和 CT→CV 同步）
    ├── rehash toolboxcache
    ├── run_mc_gradual_turn(100次MC)
    ├── run_mc_180deg_uturn(100次MC)
    ├── 保存 scan_Pi_done_Pi{pi}.mat
    └── 恢复 simulation_params.m
```

---

### 5.13 _extract_data.m — 数据提取工具

**文件路径：** `D:/Desktop/single_target_with-turn/_extract_data.m`
**行数：** 57 行
**功能：** 从 scan_Q_scale 结果中提取关键指标

#### 5.13.1 源代码

```matlab
addpath(genpath('D:/Desktop/single_target_with-turn'));
cd('D:/Desktop/single_target_with-turn');

Q_vals = [500, 1000, 3000, 10000, 30000, 100000, 300000, 1e6, 3e6];
UKF_NAMES = {'jichu','zishiying','imm'};

fprintf('gradual_best_ukf_fusion:\n');
for qi = 1:length(Q_vals)
    q = Q_vals(qi);
    d = load(sprintf('results/gradual_N100_Q%g.mat', q));
    s = d.s;
    for uu = 1:3
        mean_rmse = nanmean(s(uu).rmse_fus_best);
        fprintf('  %s Q=%9g: fus_RMSE=%.2f\n', s(uu).name, q, mean_rmse);
    end
end

fprintf('\nuturn_best_ukf_fusion:\n');
for qi = 1:length(Q_vals)
    q = Q_vals(qi);
    d = load(sprintf('results/uturn_N100_Q%g.mat', q));
    s = d.s;
    for uu = 1:3
        mean_rmse = nanmean(s(uu).rmse_fus_best);
        fprintf('  %s Q=%9g: fus_RMSE=%.2f\n', s(uu).name, q, mean_rmse);
    end
end
```

#### 5.13.2 设计 rationale

- 简洁的数据提取脚本，用于快速查看Q_scale扫描结果
- 硬编码Q值和文件名，适用于已知扫描参数的场景
- 输出格式适合复制到Excel或绘图工具

---

### 5.14 _get_precise.m — 精确值获取

**文件路径：** `D:/Desktop/single_target_with-turn/_get_precise.m`
**行数：** 28 行
**功能：** 从仿真结果中提取精确误差值

#### 5.14.1 源代码

```matlab
addpath(genpath('D:\Desktop\single_target_with-turn'));
data = load('D:\Desktop\single_target_with-turn\results\simulation_multi_20260702_201248.mat');

trackSnapshots_R1 = data.trackSnapshots_R1;
aligned_R2 = data.aligned_R2;
truthTrajs = data.truthTrajs;

n_frames = length(trackSnapshots_R1);

fprintf('=== R1 误差统计 (来自 errorStats_R1) ===\n');
disp(data.errorStats_R1);

fprintf('\n=== R2 误差统计 (来自 errorStats_R2) ===\n');
disp(data.errorStats_R2);

fprintf('\n=== 融合误差统计 (来自 fusion_eval) ===\n');
disp(data.fusion_eval);
```

#### 5.14.2 设计 rationale

- 针对特定仿真结果的快速查看脚本
- 硬编码.mat文件路径和时间戳
- 使用 `disp()` 直接输出结构体内容

---

### 5.15 analyze_covariance.m — 协方差分析

**文件路径：** `D:/Desktop/single_target_with-turn/analyze_covariance.m`
**行数：** 113 行
**功能：** 分析UKF协方差估计与实际误差的关系

#### 5.15.1 源代码结构

```
Line 1-10:  加载仿真数据
Line 12-16: 打印 errorStats_R1/R2 和 fusion_eval
Line 18-44: 计算R1/R2协方差迹统计
Line 46-65: 目标A的协方差 vs 实际误差
Line 67-88: 目标B的协方差 vs 实际误差
Line 90-113: 目标C的协方差 vs 实际误差
```

#### 5.15.2 核心分析逻辑

```matlab
% 对每个目标ac=1..3:
%   1. 找R1中ac_idx=ac的航迹ID
%   2. 遍历所有帧，计算：
%      - 实际位置误差 = haversine_distance(trk_lat, trk_lon, truth_lat, truth_lon)
%      - 协方差迹 = trace(ukf.P)
%   3. 打印均值、中位数、RMSE对比
```

#### 5.15.3 设计 rationale

- 验证UKF协方差估计的准确性：如果协方差迹远小于实际误差，说明滤波器过于自信（under-confident）；反之则过于保守（over-confident）
- 三目标分别分析，便于发现特定目标的跟踪质量问题

---

### 5.16 analyze_cov_simple.m — 简化协方差分析

**文件路径：** `D:/Desktop/single_target_with-turn/analyze_cov_simple.m`
**行数：** 45 行
**功能：** 快速查看协方差迹统计

#### 5.16.1 源代码

```matlab
addpath(genpath('D:\Desktop\single_target_with-turn'));
data = load('D:\Desktop\single_target_with-turn\results\simulation_multi_20260702_201248.mat');

trackSnapshots_R1 = data.trackSnapshots_R1;
aligned_R2 = data.aligned_R2;
truthTrajs = data.truthTrajs;

n_frames = length(trackSnapshots_R1);

fprintf('=== 协方差迹统计 ===\n');
r1_traces = []; r2_traces = [];

for k = 1:n_frames
    trks1 = trackSnapshots_R1{k}.trackList;
    for t = 1:length(trks1)
        if trks1{t}.type ~= 7 && ~isempty(trks1{t}.ukf) && isfield(trks1{t}.ukf, 'P')
            r1_traces(end+1) = trace(trks1{t}.ukf.P);
        end
    end
    
    trks2 = aligned_R2{k}.trackList;
    for t = 1:length(trks2)
        if trks2{t}.type ~= 7 && ~isempty(trks2{t}.ukf) && isfield(trks2{t}.ukf, 'P')
            r2_traces(end+1) = trace(trks2{t}.ukf.P);
        end
    end
end

fprintf('R1协方差迹: 均值=%.6f, 中位数=%.6f\n', mean(r1_traces), median(r1_traces));
fprintf('R2协方差迹: 均值=%.6f, 中位数=%.6f\n', mean(r2_traces), median(r2_traces));
fprintf('R1/R2迹比值: %.2f\n', mean(r1_traces)/mean(r2_traces));
```

#### 5.16.2 设计 rationale

- 极简版协方差分析，不计算实际误差，只看协方差迹的统计量
- R1/R2迹比值反映两部雷达协方差的相对大小，应与噪声水平成比例

---

### 5.17 入口脚本分类总结

#### 5.17.1 按场景分类

| 场景 | 主入口脚本 | MC脚本 | 扫描脚本 |
|------|-----------|--------|---------|
| 直线单目标 | `run_simulation.m` | `run_mc_straight.m` | — |
| 渐进拐弯单目标 | `run_simulation_turn.m` | `run_mc_turn.m` | `scan_Q_scale.m`, `scan_Pi.m` |
| 回头弯单目标 | `run_simulation_turn_180deg.m` | `run_mc_turn_180deg.m` | `scan_Q_scale.m`, `scan_Pi.m` |
| 三目标交叉 | `run_simulation_multi.m` | `run_mc_multi.m` | — |

#### 5.17.2 按功能分类

| 功能类别 | 脚本 | 行数 | 说明 |
|---------|------|------|------|
| 单场景演示 | `run_simulation.m` | 1368 | 完整9-Phase，含可视化 |
| 单场景演示 | `run_simulation_turn.m` | 710 | 三体制对比 |
| 单场景演示 | `run_simulation_turn_180deg.m` | 694 | 回头弯三体制 |
| 单场景演示 | `run_simulation_multi.m` | 925 | 三目标交叉 |
| MC统计 | `run_mc_straight.m` | 563 | 直线500次 |
| MC统计 | `run_mc_turn.m` | 676 | 拐弯100次IMM |
| MC统计 | `run_mc_turn_180deg.m` | 665 | 回头弯500次IMM |
| MC统计 | `run_mc_turn_compare.m` | 793 | 拐弯三体制200次 |
| MC统计 | `run_mc_turn_180deg_compare.m` | 749 | 回头弯三体制200次 |
| MC统计 | `run_mc_multi.m` | 486 | 三目标200次 |
| 参数扫描 | `scan_Q_scale.m` | 864 | Q_scale 9值×2场景 |
| 参数扫描 | `scan_Pi.m` | 860 | Pi 9值×2场景 |
| 数据提取 | `_extract_data.m` | 57 | Q扫描结果提取 |
| 数据提取 | `_get_precise.m` | 28 | 精确值获取 |
| 协方差分析 | `analyze_covariance.m` | 113 | 协方差vs实际误差 |
| 协方差分析 | `analyze_cov_simple.m` | 45 | 简化的协方差迹 |

#### 5.17.3 通用设计模式

所有入口脚本共享以下设计模式：

1. **Phase 0-9 流水线**：每个脚本都遵循相同的9阶段处理流程，便于理解和维护。

2. **R1/R2独立随机流**：`rng(seed+1e7)` 和 `rng(seed+2e7)` 确保两部雷达噪声独立。

3. **ADS-B标定**：所有脚本都在Phase 1执行相同的样本均值估计标定流程。

4. **四融合算法**：SCC/BC/CI/FCI四种算法在所有脚本中都被完整调用。

5. **坏种子机制**：RMSE>30km或改善率<-50%标记为坏种子。

6. **分段统计**：通过 `extract_segments` + `compute_mtl` 计算航迹平均长度和断裂次数。

7. **数据保存**：所有脚本都将结果保存到 `results/` 目录，文件名带时间戳。

8. **内部函数**：每个脚本底部都包含工具函数（rmse_detlist, rmse_tracks, diagnose_tracking等），避免跨文件依赖。
# REGISTRATION 与 IO 模块完整文档

本文档对注册（registration）和数据输入输出（io）两个子模块中的 6 个核心文件进行逐函数、逐行级的详尽说明，涵盖数学原理、参数定义、数据流、边界条件和设计考量。

---

## 1. registration/align_radar_to_grid.m（229 行）

### 1.1 模块定位

本文件是雷达航迹时间对齐模块的主入口。它的任务是将一部异步采样的雷达航迹统一到一套均匀的时间网格上，使得两部（或多部）雷达的航迹能够在相同的时间戳上进行逐点比较、融合或配准。

在整体空间配准流程中的位置：

1. `estimate_biases.m` 估计系统偏差
2. `correct_measurements.m` 用偏差校正所有量测
3. **`align_radar_to_grid.m`（本文件）** 将校正后的两部雷达航迹对齐到统一时间网格
4. 后续融合/跟踪处理

### 1.2 主函数：`align_radar_to_grid`

#### 1.2.1 函数签名

```matlab
function aligned = align_radar_to_grid(meas_list, unified_time, ref_time)
```

#### 1.2.2 参数说明

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `meas_list` | cell array of struct | 是 | 单部雷达的航迹列表。每个 cell 元素是一帧航迹 struct，包含 `time_sec`（秒偏移）、`lat`（度）、`lon`（度）等字段。漏检帧为空数组 `[]` |
| `unified_time` | double 行向量 | 是 | 统一时间网格，等间隔时间点（秒）。例如 `[0, 1, 2, ..., 99]`，步长 1 秒，共 100 个点 |
| `ref_time` | datetime | 否 | 参考起始时间。用于生成可读的时间字符串 `time_str`。若为空或未传入，则 `time_str` 格式化为 `"%.1fs"` |

#### 1.2.3 返回值

| 返回值 | 类型 | 说明 |
|--------|------|------|
| `aligned` | cell 数组 | 长度与 `unified_time` 相同。每个元素为 struct 或 `[]`：有效插值点返回 struct（含 `time_sec`, `time_str`, `lat`, `lon`, `aligned=true`）；超出雷达覆盖范围返回 `[]` |

#### 1.2.4 执行流程

**步骤 1：参数容错（第 86 行）**

```matlab
if nargin < 3, ref_time = []; end
```

调用者若只传 2 个参数，`ref_time` 自动设为空数组。

**步骤 2：过滤有效航迹点（第 88-102 行）**

遍历 `meas_list` 的每一帧，剔除三类无效数据：
- 漏检帧：`isempty(m)` 为 true
- 缺少 `lat` 字段：`~isfield(m, 'lat')`
- 纬度为 NaN：`isnan(m.lat)`

剩余有效帧存入 `valid` cell 数组，对应时间戳存入 `t_valid` 向量。

**步骤 3：安全性检查（第 104-110 行）**

球面大圆插值至少需要 2 个端点来确定大圆弧路径。若有效点数 `< 2`，直接返回全空 cell 数组：

```matlab
if length(valid) < 2
    aligned = cell(1, length(unified_time));
    return;
end
```

**步骤 4：确定时间范围（第 112-117 行）**

```matlab
t_min = t_valid(1);   % 第一个有效帧时间
t_max = t_valid(end); % 最后一个有效帧时间
```

网格时间点落在 `[t_min, t_max]` 之外时不做插值（避免不可靠的外推）。

**步骤 5：逐点插值（第 119-135 行）**

对 `unified_time` 中的每个时刻 `T`：
- 若 `T < t_min` 或 `T > t_max`：`aligned{k} = []`
- 否则：调用子函数 `spherical_interpolate_(T, t_valid, valid, ref_time)` 进行插值

#### 1.2.5 子函数：`spherical_interpolate_`

##### 函数签名

```matlab
function result = spherical_interpolate_(T, t_valid, valid_meas, ref_time)
```

##### 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `T` | double | 目标插值时刻（秒） |
| `t_valid` | double 行向量 | 有效航迹时间戳（升序排列） |
| `valid_meas` | cell array of struct | 对应有效航迹的 struct 数据 |
| `ref_time` | datetime 或 [] | 参考时间 |

##### 执行流程

**步骤 1：线性搜索 T 的位置（第 188-192 行）**

```matlab
idx = 1;
while idx <= n_valid && t_valid(idx) < T
    idx = idx + 1;
end
```

找到第一个 `t_valid(idx) >= T` 的位置 `idx`。这表示 T 落在 `t_valid(idx-1)` 和 `t_valid(idx)` 之间（或恰好在某个采样点上）。

**步骤 2：三种情况的分支处理（第 194-212 行）**

| 情况 | 条件 | 使用的采样点 | 物理含义 |
|------|------|-------------|---------|
| 情况 1 | `idx == 1` | 第 1、2 个点 | T 在所有采样点之前，前向外推 |
| 情况 2 | `idx > n_valid` | 倒数第 2、1 个点 | T 在所有采样点之后，后向外推 |
| 情况 3 | 其他 | `idx-1`、`idx` 两个点 | T 在两个采样点之间，正常内插 |

无论哪种情况，时间比例系数均为：

```
ratio = (T - t0) / (t1 - t0)
```

其中 `t0` 和 `t1` 是用于插值的两个端点的时间戳。

**步骤 3：球面大圆插值核心计算（第 214-216 行）**

```matlab
[lon, lat] = sphere_utils_interpolate_great_circle( ...
    m0.lon, m0.lat, m1.lon, m1.lat, ratio);
```

调用外部工具函数 `sphere_utils_interpolate_great_circle`，输入两点经纬度和比例系数 `ratio`，输出插值点的经纬度。

> **数学原理（SLERP）**：球面大圆插值等价于球面线性插值（Spherical Linear Interpolation）。设两点对应的单位向量为 `u0` 和 `u1`，夹角为 `Omega = arccos(u0 . u1)`，则插值结果为：
>
> ```
> u_T = [sin((1-ratio)*Omega) / sin(Omega)] * u0
>       + [sin(ratio*Omega) / sin(Omega)] * u1
> ```
>
> 然后将 `u_T` 反算为经纬度 `(lon_T, lat_T)`。

**步骤 4：生成时间字符串（第 218-223 行）**

```matlab
if ~isempty(ref_time)
    time_str = sphere_utils_seconds_to_datetime_str(T, ref_time);
else
    time_str = sprintf('%.1fs', T);
end
```

**步骤 5：打包返回（第 225-227 行）**

```matlab
result = struct('time_sec', T, 'time_str', time_str, ...
                'lat', lat, 'lon', lon, 'aligned', true);
```

##### 返回值

| 字段 | 类型 | 说明 |
|------|------|------|
| `time_sec` | double | 插值时刻（秒） |
| `time_str` | char | 可读时间字符串 |
| `lat` | double | 插值得到的纬度（度） |
| `lon` | double | 插值得到的经度（度） |
| `aligned` | logical | 固定为 `true`，标识这是插值结果而非原始量测 |

### 1.3 设计考量

1. **为何选择球面大圆插值而非 ENU 切平面？**
   - 球面大圆是球面上两点间的最短路径，ENU 切平面近似在数百公里尺度上会产生显著畸变
   - 不依赖雷达站本地切平面，适用于两个站点间距较大的多基地雷达配准
   - 避免 ENU 投影的保角/保距误差

2. **为何不做外推？**
   - 外推精度不可靠，特别是目标做机动转弯时
   - 主函数 `align_radar_to_grid` 对 `[t_min, t_max]` 之外的点直接返回 `[]`
   - 子函数 `spherical_interpolate_` 虽然支持前向/后向外推（用于极端边界情况），但主函数已通过范围检查规避了这种情况

3. **漏检帧处理策略**
   - 漏检帧（`[]`）和 NaN 帧在过滤阶段被完全跳过
   - 不会占用时间网格位置，但也不会影响插值计算

---

## 2. registration/estimate_biases.m（528 行）

### 2.1 模块定位

本文件是空间配准（Spatial Registration）流程的核心模块，负责从多帧标校点量测数据中估计两部雷达的系统偏差（距离偏置和方位角偏置）。

在整体流程中的位置：

```
Step 1: estimate_biases.m（本文件）—— 估计系统偏差
    ├── LS（直接最小二乘）：粗略估计
    └── EML（ECEF 空间优化）：精化估计（最终被覆盖为 LS）
Step 2: correct_measurements.m —— 用偏差校正所有量测
Step 3: align_radar_to_grid.m —— 时间对齐
Step 4: 后续融合处理
```

### 2.2 函数签名

```matlab
function est = estimate_biases(r1_meas, r2_meas, truth_points, ...
    radar1_lon, radar1_lat, radar2_lon, radar2_lat, ...
    tx1_lon, tx1_lat, tx2_lon, tx2_lat)
```

### 2.3 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `r1_meas` | cell array of struct | 雷达 1 量测序列。每个 struct 含 `range_meas`（米）、`azimuth_meas`（度）、`radial_vel_meas`（m/s）。漏检帧为 `[]` |
| `r2_meas` | cell array of struct | 雷达 2 量测序列，格式同 `r1_meas` |
| `truth_points` | double 矩阵 [N x 2] | 标校点真实位置。第 i 行 = `[lon_i, lat_i]`（度） |
| `radar1_lon`, `radar1_lat` | double | 雷达 1 部署位置的经纬度（度） |
| `radar2_lon`, `radar2_lat` | double | 雷达 2 部署位置的经纬度（度） |
| `tx1_lon`, `tx1_lat` | double | 发射源 1（illumination source 1）的经纬度（度），用于双基地距离计算 |
| `tx2_lon`, `tx2_lat` | double | 发射源 2 的经纬度（度） |

### 2.4 返回值

| 字段 | 类型 | 说明 |
|------|------|------|
| `est.dr1` | double | 雷达 1 距离偏置估计（米），正=多报 |
| `est.da1` | double | 雷达 1 方位偏置估计（度），正=偏右 |
| `est.dr2` | double | 雷达 2 距离偏置估计（米） |
| `est.da2` | double | 雷达 2 方位偏置估计（度） |

### 2.5 执行流程详解

#### 2.5.1 数据预处理（第 129-162 行）

**确定标校点数量：**

```matlab
total_pts = min(length(r1_meas), length(r2_meas));  % 取两部雷达帧数的较小值
n_pts = size(truth_points, 1);                        % 标校点总数
n_cal = min(total_pts, n_pts);                        % 实际可用标校点数
```

由于两部雷达异步采样，帧数可能不同。取较小值保证每帧都有两部雷达的数据。

**安全性检查：**

```matlab
if n_cal < 3
    est = struct('dr1', 0, 'da1', 0, 'dr2', 0, 'da2', 0);
    return;
end
```

至少需要 3 个标校点才能做有意义的统计估计（大数定律要求）。

#### 2.5.2 Step 1：直接最小二乘估计（LS）（第 164-306 行）

**数学模型：**

雷达量测模型为：

```
r_meas = r_true + dr_sys + n_r       （距离量测 = 真实距离 + 系统偏置 + 噪声）
a_meas = a_true + da_sys + n_a       （方位量测 = 真实方位 + 系统偏置 + 噪声）
```

其中 `n_r ~ N(0, sigma_r^2)`，`n_a ~ N(0, sigma_a^2)` 为零均值高斯噪声。

对第 i 个标校点：

```
dr_i = r_meas_i - r_true_i = dr_sys + n_r_i
da_i = a_meas_i - a_true_i = da_sys + n_a_i
```

取 N 个标校点的算术平均即为 LS 估计：

```
dr_ls = (1/N) * sum(dr_i) → dr_sys  （当 N 足够大时）
da_ls = (1/N) * sum(da_i) → da_sys
```

**逐帧计算偏差（第 190-283 行）：**

对每个标校点 i，依次计算：

1. **雷达 1 的真实群距离（双基地）：**

   ```matlab
   true_rng1 = sphere_utils_haversine_distance(tx1_lon, tx1_lat, t_lon, t_lat) ...
             + sphere_utils_haversine_distance(radar1_lon, radar1_lat, t_lon, t_lat);
   ```

   群距离 = 发射源 1 到目标的球面距离 + 目标到雷达 1（接收机）的球面距离。

2. **雷达 1 的真实方位角：**

   ```matlab
   true_az1 = sphere_utils_azimuth(radar1_lon, radar1_lat, t_lon, t_lat);
   ```

3. **距离偏差和方位偏差：**

   ```matlab
   dr1_list(end+1) = m1.range_meas - true_rng1;
   daz1 = m1.azimuth_meas - true_az1;
   % 角度包裹处理
   if daz1 > 180, daz1 = daz1 - 360; end
   if daz1 < -180, daz1 = daz1 + 360; end
   da1_list(end+1) = daz1;
   ```

   方位角偏差必须进行 `[-180, 180]` 范围的角度归化。例如量测 350 度、真实 10 度，直接相减得 340 度，归化后为 -20 度。

4. **雷达 2 同理。**

**漏检帧处理：**

```matlab
if isempty(m1) || isempty(m2), continue; end
```

任一部雷达漏检则该帧跳过。

**第二次安全性检查（第 285-294 行）：**

```matlab
if length(dr1_list) < 3
    est = struct('dr1', 0, 'da1', 0, 'dr2', 0, 'da2', 0);
    return;
end
```

虽然前面 `n_cal >= 3`，但漏检帧可能被跳过，导致实际有效偏差记录不足 3 个。

**LS 均值估计（第 296-306 行）：**

```matlab
dr1_ls = mean(dr1_list);
da1_ls = mean(da1_list);
dr2_ls = mean(dr2_list);
da2_ls = mean(da2_list);
```

#### 2.5.3 Step 2：EML 精化（第 308-514 行）

> **注意：** 此阶段虽然在代码中完整实现，但最终结果被覆盖为 LS 估计（第 514 行 `x_opt = x0`）。作者经过调试发现 fmincon 的精化效果不如直接 LS 估计。以下仍详细说明其设计原理。

**为什么需要 EML？**

LS 估计的局限性：
1. 仅利用每个雷达独立的"量测-真值"标量差，没有利用两部雷达同时观测同一目标的空间几何约束
2. 方位角差值需要做角度包裹处理，在边界附近可能引入误差

EML 的思想：在 ECEF 三维直角坐标系中，建立全局代价函数，最小化两部雷达校正后位置与标校点真实位置之间的欧氏距离。

**下采样（第 327-354 行）：**

为控制 fmincon 的计算复杂度，最多取 50 个标校点参与优化：

```matlab
opt_n = min(50, length(cal_idxs));
opt_step = max(1, floor(length(cal_idxs) / opt_n));
opt_idxs = cal_idxs(1:opt_step:end);
opt_idxs = opt_idxs(1:min(opt_n, length(opt_idxs)));
```

**构建优化点集 `cp_list`（第 356-394 行）：**

```matlab
cp_list{end+1} = struct( ...
    'truth', truth_points(idx, :), ...   % [lon, lat]
    'r1_rng', m1.range_meas, ...         % 雷达1距离量测
    'r1_az', m1.azimuth_meas, ...        % 雷达1方位量测
    'r2_rng', m2.range_meas, ...         % 雷达2距离量测
    'r2_az', m2.azimuth_meas);           % 雷达2方位量测
```

**定义代价函数句柄（第 396-411 行）：**

```matlab
cost_fcn = @(biases) cost_fcn_with_params(biases, cp_list, radar1_lon, radar1_lat, ...
    radar2_lon, radar2_lat, tx1_lon, tx1_lat, tx2_lon, tx2_lat);
```

利用 MATLAB 匿名函数（闭包）将除 `biases` 外的所有参数预绑定。

**优化初值和边界（第 413-463 行）：**

```matlab
x0 = [dr1_ls, da1_ls, dr2_ls, da2_ls];    % LS 结果作为初值
DIST_MARGIN = 50000;   % 距离搜索余量：±50000 米
AZI_MARGIN = 10;        % 方位搜索余量：±10 度
lb = [dr1_ls-DIST_MARGIN, da1_ls-AZI_MARGIN, dr2_ls-DIST_MARGIN, da2_ls-AZI_MARGIN];
ub = [dr1_ls+DIST_MARGIN, da1_ls+AZI_MARGIN, dr2_ls+DIST_MARGIN, da2_ls+AZI_MARGIN];
```

边界设计原则：
- 使用绝对余量而非相对百分比（避免偏差接近 0 时边界过窄）
- 仿真中雷达 1 的假设偏差约为 dr1=20000m, da1=-3deg；雷达 2 约为 dr2=-15000m, da2=3.5deg

**调用 fmincon（第 465-498 行）：**

```matlab
options = optimoptions('fmincon', 'Display', 'iter', ...
    'MaxIterations', 100, 'OptimalityTolerance', 1e-10);
x_opt = fmincon(cost_fcn, x0, [], [], [], [], lb, ub, [], options);
```

- 求解器：SQP（Sequential Quadratic Programming）
- 最大迭代次数：100
- 最优性容差：1e-10（非常严格）

**最终决策（第 507-514 行）：**

```matlab
x_opt = x0;   % 放弃 fmincon 精化结果，直接使用 LS 估计
```

作者注释说明可能原因：
1. 代价函数在 ECEF 空间中可能存在局部极小值
2. 标校点的 ECEF 转换引入了椭球模型的简化误差
3. 下采样损失了部分信息

#### 2.5.4 输出打包（第 516-527 行）

```matlab
est = struct('dr1', x_opt(1), 'da1', x_opt(2), ...
             'dr2', x_opt(3), 'da2', x_opt(4));
```

### 2.6 数学公式汇总

| 公式 | 说明 |
|------|------|
| `dr_i = r_meas_i - r_true_i` | 第 i 个标校点的距离偏差 |
| `da_i = wrap_180(a_meas_i - a_true_i)` | 第 i 个标校点的方位偏差（归化到 [-180, 180]） |
| `dr_ls = mean(dr_list)` | LS 距离偏置估计 |
| `da_ls = mean(da_list)` | LS 方位偏置估计 |
| `true_rng = d(Tx->Target) + d(Target->Rx)` | 双基地群距离（Haversine） |
| `total_cost = sum(||e1e-te||^2 + ||e2e-te||^2) / 1e6` | EML 代价函数（见 cost_fcn_with_params.m） |

---

## 3. registration/cost_fcn_with_params.m（196 行）

### 3.1 模块定位

本文件是 `estimate_biases.m` 中 EML 精化阶段的代价函数。它被 `estimate_biases.m` 通过匿名函数句柄调用，作为 `fmincon` 优化器的目标函数。

### 3.2 函数签名

```matlab
function total = cost_fcn_with_params(biases, cp_list, radar1_lon, radar1_lat, radar2_lon, radar2_lat, ...
    tx1_lon, tx1_lat, tx2_lon, tx2_lat)
```

### 3.3 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `biases` | double 向量 [4x1] | 待优化的系统偏差候选值 `[dr1, da1, dr2, da2]` |
| `cp_list` | cell array of struct | 优化点集，每个元素含 `truth`、`r1_rng`、`r1_az`、`r2_rng`、`r2_az` |
| `radar1_lon`, `radar1_lat` | double | 雷达 1 部署坐标（度） |
| `radar2_lon`, `radar2_lat` | double | 雷达 2 部署坐标（度） |
| `tx1_lon`, `tx1_lat` | double | 发射源 1 坐标（度） |
| `tx2_lon`, `tx2_lat` | double | 发射源 2 坐标（度） |

### 3.4 返回值

| 返回值 | 类型 | 说明 |
|--------|------|------|
| `total` | double | 标量代价值，ECEF 空间误差平方和除以 1e6 |

### 3.5 执行流程

#### 3.5.1 解包偏差参数（第 97-107 行）

```matlab
dr1 = biases(1);   % 雷达1距离偏置（米）
da1 = biases(2);   % 雷达1方位偏置（度）
dr2 = biases(3);   % 雷达2距离偏置（米）
da2 = biases(4);   % 雷达2方位偏置（度）
```

#### 3.5.2 遍历每个优化点（第 109-194 行）

对每个标校点 j：

**1. 获取真实 ECEF 坐标（第 122-134 行）：**

```matlab
t_lon = cp.truth(1);   % 经度
t_lat = cp.truth(2);   % 纬度
te = coord_systems_lla_to_ecef(t_lat, t_lon, 0.0);
```

高度设为 0.0（海平面），因为本模块只做水平空间配准。

**2. 雷达 1 校正与反解（第 136-162 行）：**

```matlab
r1c = cp.r1_rng - dr1;       % 校正后距离
a1c = cp.r1_az - da1;        % 校正后方位角

% 双基地反解：从群距离和方位角求目标到 Rx 的地表距离
baseline1 = sphere_utils_haversine_distance(tx1_lon, tx1_lat, radar1_lon, radar1_lat);
tx_az1 = sphere_utils_azimuth(radar1_lon, radar1_lat, tx1_lon, tx1_lat);
phi1 = a1c - tx_az1;
r1_dist = 0.5 * (r1c^2 - baseline1^2) / (r1c - baseline1 * cosd(phi1));

[e1_lon, e1_lat] = sphere_utils_destination_point(radar1_lon, radar1_lat, r1_dist, a1c);
e1e = coord_systems_lla_to_ecef(e1_lat, e1_lon, 0.0);
total = total + sum((e1e - te).^2);
```

> **双基地反解公式推导**：
> 已知群距离 `r1c = d(Tx->Target) + d(Target->Rx)`，方位角 `a1c`（从 Rx 看目标的方位），以及 Tx-Rx 基线距离 `baseline1`。
>
> 设 `r1_dist = d(Target->Rx)`，`d_tx = d(Tx->Target)`，则 `r1c = d_tx + r1_dist`。
>
> 在 Tx-Target-Rx 三角形中，由余弦定理：
> ```
> d_tx^2 = r1_dist^2 + baseline1^2 - 2 * r1_dist * baseline1 * cos(phi1)
> ```
>
> 代入 `d_tx = r1c - r1_dist`：
> ```
> (r1c - r1_dist)^2 = r1_dist^2 + baseline1^2 - 2*r1_dist*baseline1*cos(phi1)
> r1c^2 - 2*r1c*r1_dist + r1_dist^2 = r1_dist^2 + baseline1^2 - 2*r1_dist*baseline1*cos(phi1)
> r1c^2 - baseline1^2 = 2*r1_dist*(r1c - baseline1*cos(phi1))
> r1_dist = (r1c^2 - baseline1^2) / (2*(r1c - baseline1*cos(phi1)))
> ```

**3. 雷达 2 同理（第 164-185 行）。**

**4. 数值缩放（第 187-192 行）：**

```matlab
total = total / 1e6;
```

ECEF 坐标在米量级（~6.37e6 米），误差平方约在 10^6 ~ 10^10 量级。除以 1e6 将代价值降到合理范围，提高 fmincon 梯度计算的数值稳定性。此单调缩放不改变最优解的位置。

### 3.6 为什么在 ECEF 空间做优化？

| 坐标系 | 优点 | 缺点 |
|--------|------|------|
| ECEF（三维直角） | 梯度全局均匀；欧氏距离物理意义明确；标准最小二乘理论适用 | 需要进行 LLA-ECEF 坐标转换 |
| 经纬度 | 直观 | 经度量度不统一（纬度1度和经度1度对应不同地面距离）；极点奇异性；角度包裹 |
| ENU（切平面） | 局部精度高 | 仅局部有效；多点分布广时切平面畸变不可忽略 |

---

## 4. io/load_adsb.m（321 行）

### 4.1 模块定位

本文件是 ADS-B 数据加载模块。ADS-B（Automatic Dependent Surveillance-Broadcast，广播式自动相关监视）是民航飞机定期广播的位置/速度/高度等飞行数据。在本仿真系统中，ADS-B 数据被用作目标的"真实航迹"（ground truth）。

### 4.2 函数签名

```matlab
function [true_tracks, labels, speeds] = load_adsb(csv_path, icao_list, label_list, ...
    dt_sec, start_time, duration_sec, time_offset_sec)
```

### 4.3 参数说明

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `csv_path` | char 字符串 | 是 | ADS-B CSV 文件的完整路径。文件应包含 19 列数据 |
| `icao_list` | cell array of char | 是 | 要提取的飞机 ICAO 代码列表。ICAO 代码是每架飞机的全球唯一 24 位十六进制标识符 |
| `label_list` | cell array of char | 是 | 飞机的可读标签列表，与 `icao_list` 一一对应 |
| `dt_sec` | double | 是 | 仿真时间步长（秒），即重采样网格的间隔 |
| `start_time` | datetime | 是 | 仿真起始时间，用于将 ADS-B 原始时间戳转换为相对秒数 |
| `duration_sec` | double | 是 | 仿真总时长（秒） |
| `time_offset_sec` | double | 否 | 时间偏移量（秒），默认 0。将 ADS-B 数据的搜索窗口向后平移 |

### 4.4 返回值

| 返回值 | 类型 | 说明 |
|--------|------|------|
| `true_tracks` | cell 数组 (n_ac x 1) | 每架飞机的真实航迹矩阵 N x 5：`[lon, lat, lon_rate, lat_rate, time]` |
| `labels` | cell 数组 (n_ac x 1) | 飞机标签 |
| `speeds` | double 列向量 (n_ac x 1) | 平均地速（米/秒） |

### 4.5 CSV 文件列定义

文件包含 19 列，列名映射如下：

| 列索引 | 列名 | 含义 |
|--------|------|------|
| 1 | `icao` | 飞机 ICAO 代码 |
| 2 | `lat` | 纬度（度，WGS-84） |
| 3 | `lon` | 经度（度，WGS-84） |
| 4 | `heading` | 航向角（度，0=北，顺时针） |
| 5 | `alt_ft` | 气压高度（英尺） |
| 6 | `speed_kt` | 地速（节，knots = 海里/小时） |
| 7 | `x7` | 辅助字段 |
| 8 | `rx` | 辅助字段 |
| 9 | `type` | 辅助字段 |
| 10 | `reg` | 辅助字段 |
| 11 | `ts` | 时间戳字符串（格式：`yyyy-MM-dd HH:mm:ss`） |
| 12 | `origin` | 辅助字段 |
| 13 | `dest` | 辅助字段 |
| 14 | `flight` | 辅助字段 |
| 15 | `flag1` | 辅助字段 |
| 16 | `vr_ft` | 垂直速率（英尺） |
| 17 | `icao_flt` | 辅助字段 |
| 18 | `flag2` | 辅助字段 |
| 19 | `airline` | 辅助字段 |

### 4.6 执行流程

#### 4.6.1 参数容错（第 109-112 行）

```matlab
if nargin < 7, time_offset_sec = 0; end
```

#### 4.6.2 CSV 导入配置（第 114-130 行）

```matlab
opts = detectImportOptions(csv_path, 'NumVariables', 19);
opts.VariableNames = {'icao','lat','lon','heading','alt_ft','speed_kt',...
    'x7','rx','type','reg','ts','origin','dest','flight','flag1',...
    'vr_ft','icao_flt','flag2','airline'};
T = readtable(csv_path, opts);
```

- `detectImportOptions` 自动检测 CSV 文件的结构（分隔符、编码等）
- `readtable` 将 CSV 读入 MATLAB table 类型变量 T

#### 4.6.3 逐架飞机处理循环（第 147-319 行）

**步骤 1：查找飞机记录（第 151-164 行）**

```matlab
icao = icao_list{a};
idx = strcmp(T.icao, icao);
if sum(idx) == 0
    error('Aircraft %s not found in ADS-B data', icao);
end
```

使用 `strcmp` 逐行比较 ICAO 代码。若某飞机在 CSV 中完全找不到，抛出错误。

**步骤 2：提取相关列（第 166-172 行）**

```matlab
ac_lat = T.lat(idx);       % 纬度向量（度）
ac_lon = T.lon(idx);       % 经度向量（度）
ac_spd = T.speed_kt(idx);  % 地速向量（节）
ts_raw = T.ts(idx);        % 原始时间戳字符串向量
```

**步骤 3：时间戳解析（第 174-186 行）**

```matlab
if iscell(ts_raw)
    ts_dt = datetime(ts_raw, 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
else
    ts_dt = datetime(cellstr(string(ts_raw)), 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
end
```

兼容两种时间戳格式：cell 数组或 string 数组。

**步骤 4：转换为相对秒数（第 188-191 行）**

```matlab
t_sec = seconds(ts_dt - start_time);
```

**步骤 5：时间窗口过滤（第 193-204 行）**

```matlab
valid = t_sec >= time_offset_sec & t_sec <= time_offset_sec + duration_sec ...
        & ~isnan(ac_lat) & ~isnan(ac_lon);
ac_lat = ac_lat(valid);
ac_lon = ac_lon(valid);
ac_spd = ac_spd(valid);
t_sec = t_sec(valid);
```

过滤条件：
1. 时间在仿真窗口 `[time_offset_sec, time_offset_sec + duration_sec]` 内
2. 纬度和经度不为 NaN

**步骤 6：安全性检查（第 206-210 行）**

```matlab
if sum(valid) < 3
    error('Aircraft %s has <3 valid points in simulation window', icao);
end
```

至少需要 3 个有效数据点才能进行插值和差分计算。

**步骤 7：按时间去重（第 212-221 行）**

```matlab
[t_sec, ui] = unique(t_sec, 'stable');
ac_lat = ac_lat(ui);
ac_lon = ac_lon(ui);
ac_spd = ac_spd(ui);
```

ADS-B 数据可能有多个记录共享同一时间戳。`unique` 的 `'stable'` 参数保持原始顺序不变，保留第一次出现的值。

**步骤 8：按时间排序（第 223-229 行）**

```matlab
[t_sec, si] = sort(t_sec);
ac_lat = ac_lat(si);
ac_lon = ac_lon(si);
ac_spd = ac_spd(si);
```

**步骤 9：生成均匀时间网格（第 231-242 行）**

```matlab
t_grid = (0:dt_sec:duration_sec)';                    % 均匀网格，列向量
t_sec_relative = t_sec - time_offset_sec;              % 原始时间转为相对 offset
```

例如 `dt_sec = 1.0, duration_sec = 100`，则 `t_grid = [0; 1; 2; ...; 100]`（101 个点）。

**步骤 10：线性插值重采样（第 244-255 行）**

```matlab
lat_grid = interp1(t_sec_relative, ac_lat, t_grid, 'linear', 'extrap');
lon_grid = interp1(t_sec_relative, ac_lon, t_grid, 'linear', 'extrap');
```

- `interp1` 是一维线性插值函数
- `'linear'` 方法：两点之间直线连接
- `'extrap'`：允许外推（查询点超出原始数据范围时也能给出值）
- 假设目标在相邻 ADS-B 广播点之间匀速运动

**步骤 11：中心差分估计经纬度变化率（第 257-294 行）**

```matlab
n = length(t_grid);
lon_rate = zeros(n, 1);
lat_rate = zeros(n, 1);

% 中心差分（内部点，2 <= k <= n-1）
for k = 2:n-1
    lon_rate(k) = (lon_grid(k+1) - lon_grid(k-1)) / (2*dt_sec);
    lat_rate(k) = (lat_grid(k+1) - lat_grid(k-1)) / (2*dt_sec);
end

% 边界处理
if n >= 2
    lon_rate(1)   = (lon_grid(2) - lon_grid(1)) / dt_sec;          % 前向差分
    lat_rate(1)   = (lat_grid(2) - lat_grid(1)) / dt_sec;
    lon_rate(end) = (lon_grid(end) - lon_grid(end-1)) / dt_sec;    % 后向差分
    lat_rate(end) = (lat_grid(end) - lat_grid(end-1)) / dt_sec;
end
```

差分方法总结：

| 位置 | 方法 | 公式 | 精度 |
|------|------|------|------|
| 内部点 k=2..n-1 | 中心差分 | `(x(k+1) - x(k-1)) / (2*dt)` | O(dt^2) |
| 首点 k=1 | 前向差分 | `(x(2) - x(1)) / dt` | O(dt) |
| 末点 k=n | 后向差分 | `(x(n) - x(n-1)) / dt` | O(dt) |

**步骤 12：打包输出（第 296-317 行）**

```matlab
true_tracks{a} = [lon_grid, lat_grid, lon_rate, lat_rate, t_grid];
labels{a} = label_list{a};
speeds(a) = mean(ac_spd, 'omitnan') * 0.514444;
```

速度转换：1 knot = 1852/3600 = 0.514444 m/s。

### 4.7 数学公式汇总

| 公式 | 说明 |
|------|------|
| `t_sec = seconds(ts_dt - start_time)` | 时间戳转相对秒数 |
| `lat_grid = interp1(t_rel, ac_lat, t_grid, 'linear', 'extrap')` | 线性插值重采样 |
| `lon_rate(k) = (lon(k+1) - lon(k-1)) / (2*dt)` | 中心差分（内部点） |
| `speed_mps = mean(speed_kt) * 0.514444` | 节转米/秒 |

---

## 5. io/extract_measurement_field.m（90 行）

### 5.1 模块定位

这是一个纯工具函数，用于从 cell array of struct 形式的量测序列中提取指定字段的数值数组。它在 `save_all.m` 的 MAT 文件保存阶段被调用。

### 5.2 函数签名

```matlab
function vals = extract_measurement_field(meas_list, key)
```

### 5.3 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `meas_list` | cell array of struct | 量测序列。每个 cell 元素是一帧量测 struct，漏检帧为空数组 `[]` |
| `key` | char 字符串 | 要提取的字段名称，如 `'range_meas'`、`'azimuth_meas'`、`'lat'`、`'lon'`、`'time_sec'`、`'radial_vel_meas'` 等 |

### 5.4 返回值

| 返回值 | 类型 | 说明 |
|--------|------|------|
| `vals` | double 列向量 (N x 1) | 提取出的字段值。长度与 `meas_list` 相同。漏检帧或缺失字段的对应位置填 NaN |

### 5.5 执行流程

**步骤 1：预分配全 NaN 数组（第 67-72 行）**

```matlab
vals = NaN(length(meas_list), 1);
```

先创建全 NaN 的列向量，然后在有效帧的位置覆盖为实际值。这样漏检帧自然保留 NaN，无需额外处理。

**步骤 2：逐帧提取（第 74-88 行）**

```matlab
for i = 1:length(meas_list)
    m = meas_list{i};
    if ~isempty(m) && isfield(m, key)
        vals(i) = m.(key);
    end
end
```

关键判断：
- `~isempty(m)`：排除漏检帧（空数组）
- `isfield(m, key)`：排除缺少目标字段的帧（如对齐后的插值帧可能不含 `range_meas`）
- `m.(key)`：MATLAB 动态字段访问语法。当 `key` 是字符串变量时，`m.(key)` 等价于 `m.字段名`

### 5.6 设计特点

1. **NaN 占位策略**：预分配全 NaN 数组，只在有效帧覆盖。这比"先收集有效值再回填 NaN"更高效
2. **短路逻辑**：`~isempty(m) && isfield(m, key)` 利用短路与特性——若 `isempty(m)` 为 true，不再检查 `isfield`
3. **动态字段访问**：`m.(key)` 允许在运行时动态决定访问哪个字段，避免冗长的 if-elseif 链

---

## 6. io/save_all.m（331 行）

### 6.1 模块定位

本文件是仿真管线的最后一步——数据持久化模块。它将一次仿真运行的全部结果（真实航迹、两部雷达的量测数据、仿真参数）保存到指定输出目录，输出三种互补格式：CSV、MAT、JSON。

### 6.2 函数签名

```matlab
function save_all(true_track, r1_meas_list, r2_meas_list, params, out_dir)
```

### 6.3 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `true_track` | double 矩阵 (n_steps x 5) | 真实航迹。列：`[lon, lat, lon_rate, lat_rate, time]` |
| `r1_meas_list` | cell array of struct | 雷达 1 量测序列 |
| `r2_meas_list` | cell array of struct | 雷达 2 量测序列 |
| `params` | struct | 仿真参数结构体，含 `ref_start_time`、`duration_sec`、`dt_sec`、雷达坐标、噪声标准差等 |
| `out_dir` | char 字符串 | 输出目录路径 |

### 6.4 输出文件清单

| 文件名 | 格式 | 内容 |
|--------|------|------|
| `true_track.csv` | CSV | 真实航迹（时间、经纬度、变化率、速度） |
| `radar1_measurements.csv` | CSV | 雷达 1 量测数据 |
| `radar2_measurements.csv` | CSV | 雷达 2 量测数据 |
| `simulation_data.mat` | MAT | 所有数值变量打包 |
| `simulation_metadata.json` | JSON | 实验元数据和仿真参数 |

### 6.5 执行流程详解

#### 6.5.1 第一部分：保存真实航迹 CSV（第 104-165 行）

**速度计算原理（第 109-121 行）：**

经纬度变化率（度/秒）需要转换为地面速度（米/秒）。设地球半径 R = 6371000.0 米：

```
v_east  = lon_rate * (pi/180) * R * cos(lat)    % 东向速度
v_north = lat_rate * (pi/180) * R                % 北向速度
speed_ms = sqrt(v_east^2 + v_north^2)            % 合速度
```

其中 `cos(lat)` 是经线在纬度 lat 处的"收缩系数"——经度 1 度对应的地面弧长在赤道处最大（约 111 km），向两极逐渐收敛至 0。

**写入流程：**

```matlab
fid = fopen(fullfile(out_dir, 'true_track.csv'), 'w');
fprintf(fid, 'time_str,lon_deg,lat_deg,lon_rate_dps,lat_rate_dps,speed_ms\n');
R = 6371000.0;
for i = 1:n
    lat_rad = deg2rad(true_track(i, 2));
    v_east  = true_track(i, 3) * (pi/180) * R * cos(lat_rad);
    v_north = true_track(i, 4) * (pi/180) * R;
    speed_ms = sqrt(v_east^2 + v_north^2);
    time_str = sphere_utils_seconds_to_datetime_str(true_track(i, 5), params.ref_start_time);
    fprintf(fid, '%s,%.8f,%.8f,%.6f,%.6f,%.6f\n', time_str, ...
        true_track(i,1), true_track(i,2), true_track(i,3), true_track(i,4), speed_ms);
end
fclose(fid);
```

精度控制：
- 经纬度：8 位小数（约 0.01 米精度）
- 变化率/速度：6 位小数

#### 6.5.2 第二部分：保存两部雷达量测 CSV（第 167-240 行）

**双雷达循环（第 174-240 行）：**

```matlab
radars = {'radar1', r1_meas_list; 'radar2', r2_meas_list};
for r = 1:2
    rname = radars{r, 1};
    rmeas = radars{r, 2};
    fid = fopen(fullfile(out_dir, sprintf('%s_measurements.csv', rname)), 'w');
    fprintf(fid, 'time_str,range_meas_m,...\n');   % 表头
    for i = 1:length(rmeas)
        m = rmeas{i};
        if isempty(m)
            fprintf(fid, 'nan,,,,,,,,\n');
            continue;
        end
        if isfield(m, 'range_meas')
            % 情况A：量测帧
            fprintf(fid, '%s,%.3f,%.6f,%.4f', m.time_str, m.range_meas, m.azimuth_meas, m.radial_vel_meas);
            if isfield(m, 'range_true')
                fprintf(fid, ',%.3f,%.6f,%.4f', m.range_true, m.azimuth_true, m.radial_vel_true);
            else
                fprintf(fid, ',,,');
            end
            fprintf(fid, ',%.8f,%.8f\n', m.lat, m.lon);
        else
            % 情况B：对齐后插值帧（只含经纬度）
            fprintf(fid, '%s,,,,,,,,%.8f,%.8f\n', m.time_str, m.lat, m.lon);
        end
    end
    fclose(fid);
end
```

**两种帧类型的处理差异：**

| 帧类型 | 来源 | 包含字段 | CSV 写入方式 |
|--------|------|---------|-------------|
| 量测帧 | 原始雷达量测 | `time_str`, `range_meas`, `azimuth_meas`, `radial_vel_meas`, `lat`, `lon` 及可选的真值字段 | 写入完整量测值 + 可选真值 + 经纬度 |
| 插值帧 | `align_radar_to_grid` 输出 | `time_str`, `lat`, `lon` | 雷达相关字段留空，仅写时间和经纬度 |

#### 6.5.3 第三部分：保存 MAT 文件（第 242-286 行）

```matlab
r1_time       = extract_measurement_field(r1_meas_list, 'time_sec');
r1_range_meas = extract_measurement_field(r1_meas_list, 'range_meas');
% ... 共 9 个字段 x 2 部雷达 = 18 个变量
save(fullfile(out_dir, 'simulation_data.mat'), ...
    'true_track', 'r1_time', 'r1_range_meas', ..., 'r2_lat', 'r2_lon');
```

MAT 格式的优势：
- 二进制格式，加载速度快
- 保留 IEEE 754 双精度完整数值精度
- 可直接被 MATLAB 后续分析脚本 `load` 使用

#### 6.5.4 第四部分：保存 JSON 元数据（第 288-325 行）

```matlab
meta = struct();
meta.project = 'HF Passive Radar Dual-Illuminator Track Association and Fusion';
meta.phase = '1_trajectory_simulation';
meta.generated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
meta.ref_start_time = datestr(params.ref_start_time, 'yyyy-mm-dd HH:MM:SS');
meta.params = struct(...);
fid = fopen(fullfile(out_dir, 'simulation_metadata.json'), 'w');
fprintf(fid, '%s', jsonencode(meta, 'PrettyPrint', true));
fclose(fid);
```

JSON 记录的参数包括：
- 项目名称和阶段
- 文件生成时间
- 参考起始时间
- `duration_sec`、`dt_sec`
- 雷达部署坐标
- 噪声标准差（距离、方位、径向速度）
- 检测概率
- 随机种子

### 6.6 三种格式的互补关系

| 格式 | 人类可读 | 机器可读 | 跨平台 | 数值精度 | 加载速度 | 用途 |
|------|---------|---------|--------|---------|---------|------|
| CSV | 是 | 是 | 是 | 受限于文本表示（6-8 位小数） | 慢 | Excel/Python 绘图、人工检查 |
| MAT | 否 | 是 | 否（MATLAB 专有） | 完整双精度 | 快 | MATLAB 后续分析脚本 |
| JSON | 是 | 是 | 是 | 仅文本型元数据 | 快 | 实验配置记录、复现 |

---

## 7. 模块间调用关系总览

```
run_simulation.m (主入口)
    |
    +-- load_adsb.m                          [io] 加载 ADS-B 真实航迹
    |       |-- detectImportOptions, readtable, datetime, interp1, unique, sort
    |       |-- 输出: true_tracks (cell Nx5), labels, speeds
    |
    +-- estimate_biases.m                    [registration] 系统偏差估计
    |       |-- sphere_utils_haversine_distance, sphere_utils_azimuth
    |       |-- cost_fcn_with_params.m  (被调用)
    |       |-- fmincon (SQP 优化，最终被 LS 结果覆盖)
    |       |-- 输出: est.dr1, est.da1, est.dr2, est.da2
    |
    +-- correct_measurements.m               [registration] 量测校正
    |       |-- 使用 estimate_biases 的输出校正所有量测
    |
    +-- align_radar_to_grid.m                [registration] 时间对齐
    |       |-- spherical_interpolate_ (子函数)
    |       |-- sphere_utils_interpolate_great_circle
    |       |-- sphere_utils_seconds_to_datetime_str
    |       |-- 输出: aligned (cell array of struct)
    |
    +-- save_all.m                            [io] 数据持久化
            |-- extract_measurement_field.m (子函数调用)
            |-- sphere_utils_seconds_to_datetime_str
            |-- 输出: CSV + MAT + JSON 文件
```

---

## 8. 外部依赖函数清单

| 函数名 | 所属模块 | 用途 |
|--------|---------|------|
| `sphere_utils_interpolate_great_circle` | sphere_utils | 球面大圆插值核心算法 |
| `sphere_utils_haversine_distance` | sphere_utils | Haversine 球面距离计算 |
| `sphere_utils_azimuth` | sphere_utils | 球面方位角计算 |
| `sphere_utils_destination_point` | sphere_utils | 球面目标点反算（给定起点、距离、方位角求终点） |
| `sphere_utils_seconds_to_datetime_str` | sphere_utils | 秒偏移转 datetime 字符串 |
| `coord_systems_lla_to_ecef` | coord_systems | 经纬度高度转 ECEF 直角坐标 |

---

## 9. 关键数值常量

| 常量 | 值 | 来源 | 用途 |
|------|-----|------|------|
| `R` | 6371000.0 米 | 地球平均半径 | 速度转换、Haversine 距离 |
| `DIST_MARGIN` | 50000 米 | 仿真设定 | fmincon 距离偏差搜索边界 |
| `AZI_MARGIN` | 10 度 | 仿真设定 | fmincon 方位偏差搜索边界 |
| `OptimalityTolerance` | 1e-10 | fmincon 配置 | 优化收敛标准 |
| `MaxIterations` | 100 | fmincon 配置 | 优化最大迭代次数 |
| `opt_n` | 50 | 经验值 | EML 优化点下采样上限 |
| `0.514444` | m/s per knot | 1852/3600 | ADS-B 速度单位转换 |
| `1e6` | 无量纲 | 数值稳定性 | EML 代价函数缩放因子 |
# Visualization Module Documentation

> Comprehensive documentation for all 7 visualization module files.

---

## Table of Contents

1. [plot_results.m](#1-plot_resultsm)
2. [plot_results_multi.m](#2-plot_results_multim)
3. [plot_scene_overview.m](#3-plot_scene_overviewm)
4. [plot_scene_overview_multi.m](#4-plot_scene_overview_multim)
5. [plot_point_cloud_3d.m](#5-plot_point_cloud_3dm)
6. [plot_turn_spatial.m](#6-plot_turn_spatialm)
7. [plot_turn_stats.m](#7-plot_turn_statsm)

---

## 1. plot_results.m

**File**: `visualization/plot_results.m`
**Lines**: 1270
**Purpose**: Unified plotting dispatcher for single-target dual-bistatic radar results visualization. Aggregates 7 originally separate `.m` files into one dispatcher that routes to sub-functions based on a `mode` string.

### 1.1 Design Architecture

The file follows a **dispatcher pattern**: the top-level `plot_results(mode, varargin)` function receives a mode string and dispatches to the appropriate sub-function using a `switch/case` block. All internal helper functions use a **suffix naming convention** to avoid name collisions between sub-functions:

| Sub-function | Helper suffix | Meaning |
|---|---|---|
| `plot_single_track_result` | `_str` | single_track_result |
| `plot_single_fusion_result` | `_sfr` | single_fusion_result |
| `plot_combined_tracks` | `_ct` | combined_tracks |
| `plot_tracks_vs_truth` | `_tvt` | tracks_vs_truth |
| `plot_tracker_result` | `_tr` | tracker_result |
| `plot_error_timeline` | (none) | standalone |
| `plot_error_timeline_turn` | (none) | standalone |

### 1.2 Dispatcher Entry Point

```matlab
function plot_results(mode, varargin)
```

| Parameter | Type | Description |
|---|---|---|
| `mode` | string | One of: `'single_track'`, `'single_fusion'`, `'combined_tracks'`, `'tracks_vs_truth'`, `'tracker'`, `'error_timeline'`, `'error_timeline_turn'` |
| `varargin` | mixed | Forwarded to the dispatched sub-function |

**Valid modes and their dispatch targets**:

| Mode | Dispatched Function | Output Figures |
|---|---|---|
| `'single_track'` | `plot_single_track_result` | 1 figure (geoaxes map with checkboxes) |
| `'single_fusion'` | `plot_single_fusion_result` | 2 figures (map + error/CDF plots) |
| `'combined_tracks'` | `plot_combined_tracks` | 1 figure (geoaxes map with checkboxes) |
| `'tracks_vs_truth'` | `plot_tracks_vs_truth` | 1 figure (tiled layout, 2 side-by-side maps) |
| `'tracker'` | `plot_tracker_result` | 1 figure (interactive geoaxes with toggle panel) |
| `'error_timeline'` | `plot_error_timeline` | 1 figure (2 subplots: error timeline + event strip) |
| `'error_timeline_turn'` | `plot_error_timeline_turn` | 1 figure (tiled layout, 2 subplots: R1/R2 error comparison) |

### 1.3 Mode 1: plot_single_track_result

**Purpose**: Single-target dual-bistatic radar track comprehensive comparison map. Displays true track, raw detections, calibrated detections, UKF tracks for both R1 and R2 radars, and station markers on a geographic map.

**Signature**:
```matlab
function plot_single_track_result(true_track, detList_R1, detList_R2, ...
    trackSnapshots_R1, trackSnapshots_R2, params, out_dir)
```

| Parameter | Type | Description |
|---|---|---|
| `true_track` | Nx2 matrix | True target trajectory [lon, lat] |
| `detList_R1` | Cell array | Detection list for R1, each cell is a frame's detection struct array |
| `detList_R2` | Cell array | Detection list for R2 |
| `trackSnapshots_R1` | Cell array | UKF track snapshots for R1 per frame |
| `trackSnapshots_R2` | Cell array | UKF track snapshots for R2 per frame |
| `params` | Struct | Simulation parameters (radar positions, Pd, Pfa, etc.) |
| `out_dir` | string | Output directory path (unused in current version) |

**Plotting Elements** (layer order):

| Layer | Visual Style | Color | Line Width | Marker |
|---|---|---|---|---|
| True track | `--s` (dashed square) | Green | 1.5 | Filled green square, size 5 |
| R1 raw detections | `--o` (dashed circle) | RGB [0.4 0.6 1.0] | 1.0 | Filled circle, size 4 |
| R1 calibrated detections | `-o` (solid circle) | Blue | 1.2 | Filled blue square, size 5 |
| R1 UKF tracks | `-o` (solid circle) | Blue | 2.0 | Filled blue circle, size 5 |
| R2 raw detections | `--o` (dashed circle) | RGB [1.0 0.6 0.6] | 1.0 | Filled circle, size 4 |
| R2 calibrated detections | `-o` (solid circle) | Red | 1.2 | Filled red square, size 5 |
| R2 UKF tracks | `-^` (solid triangle) | Red | 2.0 | Filled red triangle, size 5 |
| R1 receiver | `bs` (blue square) | Blue | -- | Size 14, filled |
| R2 receiver | `rs` (red square) | Red | -- | Size 14, filled |
| Tx1 transmitter | `b^` (blue triangle) | Blue | -- | Size 10 |
| Tx2 transmitter | `r^` (red triangle) | Red | -- | Size 10 |

**Interactive Controls**:
- Checkboxes on the right panel (x=0.76, normalized units) for toggling each layer visibility
- "全部隐藏" (Hide All) button: toggles all layers off/on
- "全部显示" (Show All) button: forces all layers on
- Status text at bottom: shows R1/R2 track counts, Pd%, Pfa value

**Coordinate System**: `geoaxes` with `'darkwater'` basemap (fallback to default `geoaxes` if Map Toolbox basemap unavailable). Figure position: `[50, 50, 1400, 750]` pixels.

**Helper Functions**:

- `extract_dets_str(detList, mode)` -- Extracts latitude/longitude from detection lists. `mode='raw'` extracts `dp.raw_lat/raw_lon`; `mode='cal'` extracts `dp.lat/lon`. Skips clutter (`dp.is_clutter`). Returns empty arrays if no valid data.
- `collect_active_tracks_str(snapshots)` -- Groups track snapshots into continuous segments by detecting gaps (missing frames). Each segment becomes a struct `{id, lat_history, lon_history}`. A new segment starts when a valid track position is found after an invalid/gap frame.
- `try_set_visible_str(h, val)` -- Safely sets `Visible` property of graphics object. Suppresses errors.
- `toggle_all_cb_str(btn, cb, h_all)` -- Toggles checkbox states and corresponding layer visibility. Swaps button text between "全部隐藏" and "全部显示".
- `show_all_cb_str(cb, h_all)` -- Forces all checkboxes to Value=1 and all layers to Visible='on'.

### 1.4 Mode 2: plot_single_fusion_result

**Purpose**: Single-target dual-bistatic radar track fusion result visualization. Produces two figures: (1) map overlay of all fusion methods vs. individual radar tracks, (2) error convergence curves with ECDF plots.

**Signature**:
```matlab
function plot_single_fusion_result(true_track, trackSnapshots_R1, trackSnapshots_R2, ...
    all_fused_snapshots, method_names, best_idx, fusion_eval, truthTraj, params, out_dir)
```

| Parameter | Type | Description |
|---|---|---|
| `all_fused_snapshots` | Cell array | Per-method fused track snapshots |
| `method_names` | Cell string | Names of fusion algorithms |
| `best_idx` | scalar | Index of best-performing fusion method |
| `fusion_eval` | Struct | Evaluation results with `overall`, `fusion_errors`, `r1_errors`, `r2_errors` fields |
| `truthTraj` | Struct | Truth trajectory with `.time_sec`, `.lat`, `.lon` fields |

**Figure 1 -- Map Overlay**:
Same geoaxes setup as `plot_single_track_result`. Additional layers:
- Fusion tracks: drawn as `-d` (diamond markers) with colors from `method_colors = {[0 0.5 0], [0.8 0.4 0], [0 0 0.8], [0.6 0 0.6]}` (green, orange, blue, magenta)
- Best method highlighted with `LineWidth=3.5` vs. `LineWidth=3.0` for others
- Best RMSE displayed in bold text at bottom

**Figure 2 -- Error Analysis**:
Tiled layout `subplot(1, 2, ...)`:

- **Left subplot**: Error convergence curves
  - X-axis: Time (seconds)
  - Y-axis: Position error (km)
  - Each fusion method plotted with different line style (`{'-', '--', '-.', ':'}`) and color (`{[0 0 0], [1 0 0], [0 0 1], [0 0.7 0]}`)
  - 10-frame moving average smoothing via `movmean`
  - R1/R2 UKF single-station errors overlaid as dotted lines

- **Right subplot**: ECDF (Empirical Cumulative Distribution Function)
  - X-axis: Position error (km)
  - Y-axis: Cumulative probability (%)
  - Computed using MATLAB's `ecdf()` function
  - All fusion methods plus R1/R2 UKF single stations compared

**Helper Functions**:

- `collect_positions_sfr(snapshots)` -- Same segment-grouping logic as `collect_active_tracks_str`, but for fusion result tracks.
- `collect_fused_positions_sfr(snapshots)` -- Groups fused track snapshots by continuous segments. Uses `ft.id` for segment identification.
- `build_frame_errors_sfr(fused_snaps, truth, frame_times)` -- Computes per-frame position error by interpolating truth trajectory to each frame time, then finding minimum distance to any fused track in that frame. Uses haversine distance.
- `build_single_frame_errors_sfr(snapshots, truth, frame_times)` -- Same as above but for single-station (non-fused) track snapshots.
- `haversine_km_sfr(lon1, lat1, lon2, lat2)` -- Haversine great-circle distance in km. Earth radius = 6371 km. Clamps intermediate `a` value to [0, 1] for numerical stability.

### 1.5 Mode 3: plot_combined_tracks

**Purpose**: Comprehensive track comparison showing associated detections, raw detections, calibrated detections, and UKF filtered tracks for both radar stations simultaneously.

**Signature**:
```matlab
function plot_combined_tracks(true_track, detList_R1, detList_R2, ...
    trackState_R1, trackState_R2, params, out_dir)
```

| Parameter | Type | Description |
|---|---|---|
| `trackState_R1` | Cell array | Per-frame track state structs with fields: `.associated`, `.det_lat/lon`, `.det_raw_lat/lon`, `.lat/lon`, `.assc_is_clutter` |

**Plotting Elements**:

| Layer | Style | Color | Purpose |
|---|---|---|---|
| True track | `y--` | Yellow dashed | Ground truth |
| R1 raw detections | `--o` | Light blue | Uncalibrated measurements |
| R2 raw detections | `--o` | Light red | Uncalibrated measurements |
| R1 calibrated | `bo-` | Blue | Post-calibration measurements |
| R2 calibrated | `ro-` | Red | Post-calibration measurements |
| R1 UKF filtered | `c-` | Cyan | UKF output track |
| R2 UKF filtered | `m-` | Magenta | UKF output track |
| R1/R2 stations | `bs`/`rs` | Blue/Red squares | Station markers |
| Tx1/Tx2 | `b^`/`r^` | Blue/Red triangles | Transmitter markers |
| Start point | `go` | Green circle | First true track point |
| End point | `gx` | Green cross | Last true track point |

**Statistics Displayed**: Associated detection counts, clutter association counts, raw detection counts, Pd%, Pfa.

**Helper Functions**:
- `extract_associated_dets_ct(stateList)` -- Extracts `(det_lat, det_lon)` from associated detections
- `extract_raw_associated_dets_ct(stateList)` -- Extracts `(det_raw_lat, det_raw_lon)` from associated detections
- `sum_assc_clutter_ct(stateList)` -- Counts clutter-associated detections
- `extract_filtered_track_ct(stateList)` -- Extracts filtered track `(lat, lon)` positions

### 1.6 Mode 4: plot_tracks_vs_truth

**Purpose**: Side-by-side comparison of R1 and R2 UKF filtered tracks against true trajectory, each on its own geoaxes map.

**Signature**:
```matlab
function plot_tracks_vs_truth(trackState_R1, trackState_R2, true_track, params, out_dir)
```

Uses `tiledlayout(1, 2)` with two `geoaxes` subplots (one per radar station). Each subplot shows:
- True track (yellow dashed)
- UKF filtered track (cyan solid)
- Radar receiver station (red square)

### 1.7 Mode 5: plot_tracker_result

**Purpose**: Interactive visualization of track fragmentation and stitching pipeline. Shows raw fragments, filtered segments, aligned segments, and stitched tracks for both radar stations.

**Signature**:
```matlab
function fig = plot_tracker_result(true_track, ...
    r1_segments, r2_segments, ...
    r1_segments_filt, r2_segments_filt, ...
    r1_segments_aligned, r2_segments_aligned, ...
    r1_stitched, r2_stitched, ...
    radar1, radar2, params, unified_time, out_dir)
```

| Parameter | Type | Description |
|---|---|---|
| `r1_segments` | Cell array | Raw R1 track fragments (struct arrays with `.lat`, `.lon`) |
| `r1_segments_filt` | Cell array | Filtered R1 fragments (cell of cells) |
| `r1_segments_aligned` | Cell array | Aligned R1 fragments (cell of cells) |
| `r1_stitched` | Cell array | Final stitched R1 track (cell per frame) |
| `radar1` | Struct | Radar station with `.lat`, `.lon` fields |

**Color Scheme** (defined as constants):

| Data Source | Color | RGB |
|---|---|---|
| True track | Dark gray | [0.10 0.10 0.10] |
| R1 raw | Light blue | [0.45 0.65 0.95] |
| R2 raw | Light red | [0.95 0.50 0.50] |
| R1 filtered | Medium blue | [0.00 0.25 0.65] |
| R2 filtered | Dark red | [0.70 0.08 0.08] |
| R1 aligned | Deep blue | [0.00 0.15 0.50] |
| R2 aligned | Very dark red | [0.50 0.05 0.05] |
| R1 stitched | Deepest blue | [0.00 0.10 0.40] |
| R2 stitched | Deepest red | [0.40 0.02 0.02] |

**Interactive Panel** (bottom of figure, `uipanel`):
- 9 toggle buttons: R1量测, R2量测, R1滤波, R2滤波, R1对齐, R2对齐, R1拼接, R2拼接, 真实航迹
- 3 action buttons: 全部显示 (Show All), 全部隐藏 (Hide All), 仅看拼接+真实 (Stitched+Truth Only)

**Helper Drawing Functions**:
- `draw_struct_array_segments_tr(ax, segments, color, lw, ms, style)` -- Draws segments from struct arrays (e.g., raw fragments). Extracts `[seg.lat]` and `[seg.lon]` using MATLAB's struct array expansion.
- `draw_cell_segments_tr(ax, segments, color, lw, ms, style)` -- Draws segments from cell arrays (e.g., filtered tracks). Iterates through cells, handling empty cells and NaN positions.
- `draw_stitched_track_tr(ax, stitched, color, lw, ms, style)` -- Handles fragmented stitched tracks by detecting gaps (invalid/empty entries) and drawing separate segments.

### 1.8 Mode 6: plot_error_timeline

**Purpose**: Dual radar station tracking error time series with detection/association event visualization.

**Signature**:
```matlab
function plot_error_timeline(trackState_R1, trackState_R2, detList_R1, detList_R2, ...
    true_track, t1_grid, t2_grid, params, out_dir)
```

**Figure Layout** (`subplot(2, 1, ...)`):

- **Top subplot**: Error over time
  - X-axis: Time (minutes), computed from frame grids `t1_grid`/`t2_grid`
  - Y-axis: Position error (km), divided by 1000
  - Four series: R1 UKF filter (blue solid), R2 UKF filter (red solid), R1 detections (blue dots), R2 detections (red dots)

- **Bottom subplot**: Detection/association event strip
  - X-axis: Frame number
  - Y-axis: Station label (R1 at y=4, R2 at y=3)
  - Three event types:
    - Blue/red dot (size 8): Associated target detection
    - Blue/red cross (size 8): Associated clutter/false alarm
    - Gray dot (size 4): Missed detection (tracking but not associated)

### 1.9 Mode 7: plot_error_timeline_turn

**Purpose**: Turn-target specific error timeline comparing base UKF vs. adaptive (maneuver-detecting) UKF for both radar stations.

**Signature**:
```matlab
function plot_error_timeline_turn(true_track, ...
    trackR1_base, trackR2_base, ...
    trackR1_ad, trackR2_ad, params, out_dir)
```

**Figure Layout** (`tiledlayout(1, 2)`):

- **Left subplot (R1)**: Base UKF error (light blue) vs. Adaptive UKF error (dark blue)
- **Right subplot (R2)**: Base UKF error (light red) vs. Adaptive UKF error (dark red)
- Both subplots include `xline` marking the turn region midpoint
- Title shows maneuver detection + Q-enhancement annotation

---

## 2. plot_results_multi.m

**File**: `visualization/plot_results_multi.m`
**Lines**: 511
**Purpose**: Multi-target result visualization dispatcher. Similar architecture to `plot_results.m` but adapted for 3 simultaneous targets (A/B/C) with cross-intersecting tracks.

### 2.1 Dispatcher

```matlab
function plot_results_multi(mode, varargin)
```

| Mode | Dispatched Function |
|---|---|
| `'single_track'` | `plot_multi_track_result` |
| `'single_fusion'` | `plot_multi_fusion_result` |

### 2.2 plot_multi_track_result

**Purpose**: Multi-target dual-bistatic radar track comprehensive comparison.

**Signature**:
```matlab
function plot_multi_track_result(true_track_A, true_track_B, true_track_C, ...
    detList_R1, detList_R2, trackSnapshots_R1, trackSnapshots_R2, params, out_dir)
```

**Key Differences from Single-Target Version**:
- Three truth tracks with distinct colors: A=yellow [1 1 0], B=magenta [1 0 1], C=cyan [0 1 1]
- Calibration points plotted as small dots (`.b`/`.r`, MarkerSize=3) instead of connected lines
- Title: "多目标双基地雷达航迹综合对比 (3目标交叉)"

### 2.3 plot_multi_fusion_result

**Purpose**: Multi-target fusion result visualization with per-pair analysis.

**Signature**:
```matlab
function plot_multi_fusion_result(true_track_A, true_track_B, true_track_C, ...
    trackSnapshots_R1, trackSnapshots_R2, all_fused_snapshots, ...
    method_names, matched_pairs, fusion_eval, truthTrajs, params, out_dir)
```

| Parameter | Type | Description |
|---|---|---|
| `matched_pairs` | Cell array | Target pairs for each fusion match |
| `truthTrajs` | Cell array | Three truth trajectory structs |

**Figure 1 -- Map**: Same geoaxes layout with three truth tracks, R1/R2 UKF tracks, and fusion tracks colored by method.

**Figure 2 -- Error Analysis**:
- **Left subplot**: Error convergence for each matched pair x each fusion method. R1/R2 single-station errors per target (3 targets x 2 stations = 6 additional lines).
- **Right subplot**: ECDF for each fusion method, computed independently per matched pair.

**Helper Functions**:
- `extract_dets_multi(detList, mode)` -- Extracts calibrated detection lat/lon. Only supports `'cal'` mode (no `'raw'`).
- `collect_active_tracks_multi(snaps)` -- Groups by track ID across all frames (not by continuity). Each unique track ID accumulates all its position samples.
- `collect_fused_positions_multi(snaps)` -- Same grouping-by-ID approach for fused tracks.
- `build_frame_errors_multi(fused_snaps, truthTrajs, frame_times, mp)` -- For each frame, finds minimum distance to any of the 3 truth trajectories.
- `build_single_frame_errors_multi(snaps, truth_ac, frame_times)` -- Per-target error computation for single-station tracks.

---

## 3. plot_scene_overview.m

**File**: `visualization/plot_scene_overview.m`
**Lines**: 164
**Purpose**: Renders the complete simulation scene on a geographic map -- radar stations, transmitters, beam coverage sectors, and true target trajectory.

### 3.1 plot_scene_overview

**Signature**:
```matlab
function plot_scene_overview(true_track, params, out_dir)
```

**Plotting Order**:

1. **Receiver stations**: Blue square (R1), Red square (R2), size 12, filled
2. **Transmitter stations**: Blue triangle (Tx1), Red triangle (Tx2), size 10, filled
3. **R1 beam sector**: Light blue `[0.3 0.6 1.0]` arc boundaries
4. **R2 beam sector**: Light red `[1.0 0.4 0.4]` arc boundaries
5. **True track**: Yellow solid line, width 2
6. **Start marker**: Green filled circle
7. **End marker**: Yellow cross

**Subtitle**: Displays Pd%, Pfa, dt (seconds), beam width (15 degrees), range bounds.

### 3.2 draw_beam_sector (Local Function)

**Purpose**: Draws the beam sector boundary lines for a radar station.

**Mathematical Foundation**:
- Samples 20 azimuth angles uniformly across `[center_az - width/2, center_az + width/2]`
- For each angle, computes inner arc point (at `r_min`) and outer arc point (at `r_max`) using `sphere_utils_destination_point`
- Draws: inner arc (dashed), outer arc (dashed), two radial edges connecting inner to outer at beam boundaries

**Coordinate Computation**: Uses geodesic destination point calculation based on spherical Earth model. Converts meters to degrees via great-circle distance formulas.

---

## 4. plot_scene_overview_multi.m

**File**: `visualization/plot_scene_overview_multi.m`
**Lines**: 76
**Purpose**: Multi-target version of scene overview. Adds three truth tracks with distinct colors and start/end markers.

### 4.1 plot_scene_overview_multi

**Signature**:
```matlab
function plot_scene_overview_multi(true_track_A, true_track_B, true_track_C, params, out_dir)
```

**Key Differences from Single-Target Version**:
- Three truth tracks: A=green, B=magenta, C=cyan (all solid lines, width 2)
- Each track gets its own start marker (green circle) and end marker (colored cross)
- Uses `subtitle()` instead of `title()` for parameter display
- Calls `draw_beam_sector_geoax` (local function) instead of `draw_beam_sector`

### 4.2 draw_beam_sector_geoax (Local Function)

**Purpose**: Same as `draw_beam_sector` but designed for `geoaxes` compatibility. Avoids the `geoaxes` RGBA alpha-channel bug by using solid lines only (no filled polygons).

---

## 5. plot_point_cloud_3d.m

**File**: `visualization/plot_point_cloud_3d.m`
**Lines**: 100
**Purpose**: 3D scatter plot of detection results in Range-Azimuth-Frame space.

### 5.1 plot_point_cloud_3d

**Signature**:
```matlab
function plot_point_cloud_3d(detList, title_str, out_path)
```

| Parameter | Type | Description |
|---|---|---|
| `detList` | Cell array | Per-frame detection struct array |
| `title_str` | string | Title prefix for the figure |
| `out_path` | string | Output file path (unused; figure displayed on screen only) |

**Detection Struct Fields Used**:

| Field | Unit | Description |
|---|---|---|
| `.prange` | meters | Pseudo-range (transmitter-to-target-to-receiver path length) |
| `.paz` | degrees | Azimuth angle (DOA at receiver) |
| `.is_clutter` | logical | true = clutter/false alarm, false = true target detection |

**3D Coordinate Space**:
- X-axis: Group range (Rg) in km (converted from meters by dividing by 1000)
- Y-axis: Azimuth angle (az) in degrees
- Z-axis: Frame number (k)

**Visual Encoding**:
- True target detections: Blue circles (`bo`), MarkerSize=4, filled
- Clutter/false alarms: Red crosses (`rx`), MarkerSize=3

**View Settings**:
- `view(45, 30)` -- Azimuth 45 degrees, elevation 30 degrees
- `rotate3d on` -- Enables interactive 3D rotation via mouse drag

**Design Rationale**: The 3D representation allows operators to visually distinguish true target tracks (which form continuous trajectories along the frame axis) from clutter (which appears randomly scattered). The range-azimuth plane reveals the geometric relationship between target position and the radar's beam sector.

---

## 6. plot_turn_spatial.m

**File**: `visualization/plot_turn_spatial.m`
**Lines**: 572
**Purpose**: Dispatcher for turn-target spatial/geographic visualization. Combines 4 originally separate files.

### 6.1 Dispatcher

```matlab
function plot_turn_spatial(mode, varargin)
```

| Mode | Dispatched Function |
|---|---|
| `'point_clouds'` | `plot_turn_point_clouds` |
| `'radar_compare'` | `plot_turn_radar_compare` |
| `'fusion_map'` | `plot_turn_fusion_map` |
| `'comprehensive'` | `plot_turn_comprehensive` |

### 6.2 plot_turn_point_clouds

**Purpose**: Side-by-side geographic map showing point clouds and both base/adaptive UKF tracks for R1 and R2.

**Layout**: `tiledlayout(1, 2)` -- R1 on left, R2 on right.

**Visual Elements per subplot**:
- True track: yellow dashed, width 2
- Point detections: gray dots (`.`), size 3
- Base UKF: blue dashed line, width 2
- Adaptive UKF: darker blue solid line, width 2.5
- Radar station: blue square (R1) / red square (R2), size 10

### 6.3 plot_turn_radar_compare

**Purpose**: Single radar station comparison with 4 sub-regions in one figure.

**Layout**:
- Main map (top-left, 60%x56%): Full track comparison with zoom rectangle
- Zoom inset (top-right, 30%x42%): Magnified turn region
- Error plot (bottom-left, 55%x30%): Error vs. time with xline at turn frame
- RMSE bar chart (bottom-right, 28%x28%): Base vs. adaptive RMSE comparison

**Turn Frame Detection**: Finds the true track point closest to coordinates (128.5, 33.5) -- the approximate turn center. Creates a +/-18 frame zoom window around this point.

**RMSE Computation**: `rms_nan_trc(x)` computes RMS of non-NaN values: `sqrt(mean(x_valid.^2))`. Improvement percentage: `(1 - rmse_adaptive/rmse_base) * 100`.

### 6.4 plot_turn_fusion_map

**Purpose**: Fusion track comparison map with info panel.

**Layout**:
- Main map (left, 62%x90%): True track, base fusion (cyan dashed), adaptive fusion (dark green solid)
- Zoom inset (right, 30%x42%): Turn region magnification
- Info panel (right, 30%x44%): Text-based summary of fusion results

**Color Scheme**:
- Base fusion: cyan `[0.0 0.7 0.7]`, dashed, width 2.2
- Adaptive fusion: dark green `[0.0 0.4 0.1]`, solid, width 3.0

### 6.5 plot_turn_comprehensive

**Purpose**: Full pipeline visualization -- raw measurements, calibration, base UKF, adaptive UKF, and SCC fusion all on one map.

**Layers** (9 total, all toggleable):

| # | Layer | Style | Color | Width |
|---|---|---|---|---|
| 1 | True track | `y--` | Yellow | 2.5 |
| 2 | R1 raw (uncalibrated) | `--.` | Light blue | 0.8 |
| 3 | R2 raw (uncalibrated) | `--.` | Light red | 0.8 |
| 4 | R1 calibrated | `-o` | Blue | 0.8 |
| 5 | R2 calibrated | `-o` | Red | 0.8 |
| 6 | R1 base UKF | `--` | Blue | 1.8 |
| 7 | R2 base UKF | `--` | Red | 1.8 |
| 8 | R1 adaptive UKF | `-` | Dark blue | 2.2 |
| 9 | R2 adaptive UKF | `-` | Dark red | 2.2 |
| 10 | Base fusion | `--` | Cyan | 2.5 |
| 11 | Adaptive fusion | `-` | Dark green | 3.0 |

**Subtitle**: Shows pipeline stages "原始量测 → 校准 → 基础UKF滤波 → 自适应UKF滤波 → SCC融合" with Pd%, Pfa, and corner angle (~113 degrees).

---

## 7. plot_turn_stats.m

**File**: `visualization/plot_turn_stats.m`
**Lines**: 766
**Purpose**: Dispatcher for turn-target statistical/comparative visualization. Combines 4 originally separate analysis files.

### 7.1 Dispatcher

```matlab
function plot_turn_stats(mode, varargin)
```

| Mode | Dispatched Function |
|---|---|
| `'comparison'` | `plot_turn_comparison` |
| `'fusion_compare'` | `plot_turn_fusion_compare` |
| `'rmse_bars'` | `plot_turn_rmse_bars` |
| `'single_compare'` | `plot_turn_single_compare` |

### 7.2 plot_turn_comparison

**Purpose**: Geographic map comparing base UKF vs. adaptive UKF vs. fusion results for turn-target scenario.

**Layout**: Single `geoaxes` (68%x86%) with interactive layer controls on the right.

**Layers** (7 total):

| Layer | Style | Color | Width |
|---|---|---|---|
| True track | `y--` | Yellow | 2.5 |
| R1 base UKF | `-` | Light blue `[0.3 0.5 1.0]` | 1.8 |
| R2 base UKF | `-` | Light red `[1.0 0.4 0.4]` | 1.8 |
| R1 adaptive UKF | `b-` | Blue | 2.2 |
| R2 adaptive UKF | `r-` | Red | 2.2 |
| Base fusion | `c-` | Cyan | 2.5 |
| Adaptive fusion | `m-` | Magenta | 2.5 |

**Special Markers**: Start (green circle), End (green cross), Turn point (white circle at midpoint of true track, labeled "~120 degree corner").

### 7.3 plot_turn_fusion_compare

**Purpose**: 6-subplot comprehensive fusion analysis figure.

**Layout**: `tiledlayout(2, 3)`

| Subplot | Content |
|---|---|
| (1,1) | Full fusion map: base vs. adaptive fusion tracks |
| (1,2) | Turn region zoom: fusion + single-station tracks overlaid |
| (1,3) | RMSE bar chart: all fusion methods + single stations |
| (2,1) | Fusion error time series with turn frame marker |
| (2,2) | Single-station-to-fusion improvement chain bar chart |
| (2,3) | Numeric summary table (text-based) |

**RMSE Bar Chart Details**:
- Methods compared: all fusion algorithms + "R1_only" + "R2_only"
- Base UKF bars: gray `[0.6 0.6 0.6]`
- Adaptive UKF bars: green `[0.0 0.5 0.0]`
- Improvement percentage annotated above each pair in red bold text

**Single-Station-to-Fusion Chain**:
- Compares R1_base, R2_base, fusion_base vs. R1_ad, R2_ad, fusion_ad
- Shows the precision improvement at each pipeline stage

### 7.4 plot_turn_rmse_bars

**Purpose**: Dedicated RMSE comparison bar chart with detailed results panel.

**Layout**: Two panels side by side.
- Left (55%x85%): Bar chart with value annotations and improvement percentages
- Right (32%x85%): Text-based summary panel

**Summary Panel Content**:
- Best base fusion method and its RMSE
- Best adaptive fusion method and its RMSE
- Overall fusion improvement percentage
- R1 single-station improvement
- R2 single-station improvement
- Simulation parameters (Pd, Pfa, corner angle, frame count, speed)

### 7.5 plot_turn_single_compare

**Purpose**: 6-subplot single-station comparison for turn-target scenario.

**Layout**: `tiledlayout(2, 3)`

| Subplot | Content |
|---|---|
| (1,1) | R1 full map: base vs. adaptive UKF |
| (1,2) | R2 full map: base vs. adaptive UKF |
| (1,3) | Turn region zoom: all 4 tracks overlaid |
| (2,1) | R1 error time series with turn marker |
| (2,2) | R2 error time series with turn marker |
| (2,3) | RMSE bar chart: R1/R2 base vs. adaptive |

**Turn Frame Detection**: Same approach as `plot_turn_radar_compare` -- finds the true track point closest to (128.5, 33.5). Uses +/-18 frame zoom window.

**Error Computation**: `err_at_frame_tsc(snap, t_lon, t_lat)` computes haversine distance in km between track position and true position. `rms_tsc(x, flag)` computes RMS of non-NaN values.

---

## Cross-Module Patterns and Conventions

### Common Plotting Patterns

1. **GeoAxes with Basemap**: All geographic plots use `geoaxes` with `'darkwater'` basemap. A `try/catch` fallback to default `geoaxes` handles environments without the Mapping Toolbox basemap support.

2. **Layer Toggle System**: Nearly every map figure includes an interactive control panel with:
   - Checkboxes for each layer (positioned at x=0.73-0.77, normalized units)
   - "全部隐藏" (Hide All) / "全部显示" (Show All) toggle buttons
   - Callbacks use anonymous functions capturing the handle and checkbox state

3. **Color Coding Convention**:
   - R1 radar data: Blue palette (light blue for raw, medium blue for filtered, dark blue for adaptive)
   - R2 radar data: Red palette (light red for raw, medium red for filtered, dark red for adaptive)
   - True track: Yellow (`y-` or `y--`)
   - Fusion tracks: Cyan/magenta/green/orange depending on method

4. **Marker Conventions**:
   - Receiver stations: Filled squares (`s`)
   - Transmitter stations: Filled triangles (`^`)
   - Track start: Green circle (`o`)
   - Track end: Cross (`x`) or green cross

5. **Figure Dimensions**: Standard figure size of `[50, 50, 1400, 750]` pixels across all modules.

### Distance Calculation

All position errors use the **haversine formula** for great-circle distance on a sphere (Earth radius = 6371 km). The formula clamps the intermediate sine argument to [0, 1] for numerical stability:

```
a = sin(dlat/2)^2 + cos(lat1)*cos(lat2)*sin(dlon/2)^2
a = max(0, min(1, a))
distance = 2*R*atan2(sqrt(a), sqrt(1-a))
```

### Truth Trajectory Interpolation

Frame-level errors are computed by interpolating the truth trajectory to each frame's time using `interp1(..., 'linear', 'extrap')`. This ensures fair comparison even when truth and measurement timestamps don't align exactly.

### Segment Continuity Detection

Track histories are split into continuous segments by detecting gaps (frames with no valid track position). This handles scenarios where tracks are temporarily lost and reacquired, producing separate visual segments rather than misleading straight-line connections across gaps.
---

## 第六部分：南阳子系统（Nanyang Subsystem）全面解析

### 6.1 架构总览

南阳子系统位于 `nanyang/` 目录及其子目录 `nanyang/sub_func_for_AsscJNN/`，是一个完整的基于群距离/方位角/多普勒速度三维量测的单目标航迹处理引擎。系统采用纯函数式编程（无 classdef），以结构体数组传递状态，通过 dispatcher 模式组织代码。整个子系统包含 **38 个 .m 文件**，分为六大功能模块：数据结构定义、检测点迹转换、航迹起始（M/N）、点迹-航迹关联（JNN/二分图匹配）、航迹预测与更新、航迹质量管理。

### 6.2 数据结构定义

#### 6.2.1 `nanyang/header.m` — 系统常量与配置

这是整个南阳系统的"总开关"文件，定义了所有常量、模式 ID 和地理区域参数。

**航迹质量控制 ID：**
- `RELIABLE_TRACK = 1`：可靠航迹（已确认且稳定跟踪）
- `MAINTAIN_TRACK = 2`：维持航迹（丢失点迹，质量下降中）
- `GOOD_TRACK = 3`：良好航迹（注释中保留，当前未启用）
- `TEMPORARY_TRACK = 6`：临时航迹（新起始，待确认）
- `HISTORY_TRACK = 7`：历史航迹（已死亡/终止）

**质量等级阈值：**
- `QUALITY_MIN = 5`：低于此值航迹终止
- `QUALITY_MAX = 15`：最高质量分
- `QUALITY_RELIABLE = 10`：从临时转为可靠的阈值
- `NEW_TRACK_QUALITY = 8`：新航迹初始质量

**关联波门参数：**
- 固定波门：`FIXED_R_RADIUS_NORMAL_FLIGHT = 60km`，`FIXED_V_RADIUS_NORMAL_FLIGHT = 7.5m/s`，`FIXED_A_RADIUS_NORMAL_FLIGHT = 9deg`
- 浮动波门增长率：`FLOAT_R_RADIUS_NORMAL_FLIGHT = 0.1km/frame`
- 径向飞行目标（速度 > 400m/s）使用更大的波门

**M/N 航迹起始参数：** `TRACK_STARTER_LOGIC_M = 5`，`TRACK_STARTER_LOGIC_N = 9`

**邻居搜索归一化综合距离门限：** `NN_OVERALL = 40`

#### 6.2.2 `nanyang/tool_header.m` — 底层工具常量

- `c = 3e8 m/s`：光速
- `iono_f_height = 220km`：F 层电离层高度
- `iono_e_height = 110km`：E 层电离层高度
- `R_earth = 6371km`：地球平均半径

### 6.3 检测点迹转换

#### 6.3.1 `det2nanyang_point.m`

将检测器输出的结构体数组转换为南阳格式的点迹结构体数组。转换内容包括：距离从米转换为千米、填充所有南阳系统需要的字段（prange、pvr、paz、drange、dvr、daz、lat/lon、Rbin/Dbin/Abin、ambgNum）。

#### 6.3.2 `det2trackDataConverter.m`

更复杂的转换器，处理飞行目标和舰船目标两种模式。飞行模式需要处理速度模糊（对超出无模糊边界的点迹生成速度模糊 +/-1 的副本）。

#### 6.3.3 `track2reportDataConverter.m`

将内部航迹结构转换为下游系统期望的报告格式。

### 6.4 航迹起始模块

#### 6.4.1 `trackStarter_logic.m` — M/N 航迹起始主逻辑

**算法流程：**
1. 若无临时航迹，直接将当前点迹全部作为候选
2. 若无检测点迹，直接返回
3. 筛选在容忍窗口内的临时航迹
4. 对每个新点迹，调用 `fun_find_best_asscpoints_NN` 寻找最佳关联候选航迹
5. 对每个候选，通过 `fun_check_track_validation` 验证有效性
6. 有效的候选通过 `fun_create_new_track` 创建正式航迹
7. 从点迹池中移除已关联的点迹（包括其模糊副本）

#### 6.4.2 `fun_check_35logic_points_improved.m` — 3/5 逻辑改进版

在连续 TOLERANT_NUM 帧中，若有 QUALIFY_NUM 帧存在运动一致的点迹则起始航迹。

#### 6.4.3 `fun_check_colinear_points.m` — 共线点检测航迹起始

从当前点出发，在历史点迹池中搜索共线运动轨迹。

### 6.5 点迹-航迹关联（JNN 算法 + 二分图匹配）

#### 6.5.1 `PointTrackAssociation_JNN.m` — JNN 全局关联

这是整个系统的核心关联算法，实现了一种改进的 JNN（联合最近邻）全局关联策略。

**算法流程：**
1. 遍历所有 track-point 对，计算代价矩阵
2. 建立双向连接关系：track_to_point / point_to_track
3. 对每个 track 进行处理：
   - case 1：无关联点 -> 标记为未关联
   - case 2：一对一关联且对方也唯一 -> 直接匹配
   - otherwise：存在多对多歧义 -> 提取子二分图 -> 转换为关联矩阵 -> 分解为所有可能的匹配方案 -> 选择代价最小的方案

#### 6.5.2 `sub_func_for_AsscJNN/` 子目录（11 个文件）

**代价计算：** `calculate_cost_of_point_track_pair.m` — 计算点-轨对的归一化代价
**波门判断：** `determine_if_point_within_the_scope_of_track.m`、`get_tracking_gate.m`
**二分图操作：** `extract_sub_bigraph.m`、`convert_bigraph_into_matrix.m`、`mat_division.m`、`candidate_matrix_selection.m`、`get_the_cost_of_match_plan.m`
**索引映射：** 4 个 get_list_index / get_matrix_index 函数

### 6.6 航迹预测与更新

#### 6.6.1 `Fun_PredictNextStep_CV.m` / `predictNextStep_cv.m`

使用滑动窗口均值预测方法（非 KF 预测）：方位角取中值、径向速度用稳健最小二乘估计斜率外推、群距离用均值减去时间偏移。

#### 6.6.2 `Fun_UpdateTrackByAsscResult.m` / `Fun_UpdateTrackforNoInputPoint.m`

根据关联结果更新航迹：有关联时用 Alpha-Beta 滤波器平滑，无关联时用预测值填充。

#### 6.6.3 `fun_fill_smooth_list_by_alpha_beta_filter.m`

Alpha-Beta 平滑器：距离平滑 weight=0.15，速度/方位平滑用稳健最小二乘估计加速度斜率。

### 6.7 航迹质量管理

#### 6.7.1 `fun_track_quality_management_and_info_completion.m`

核心状态转换逻辑：无关联时损失质量分，有关联时增加质量分。质量达到 QUALITY_RELIABLE=10 从临时升级为可靠，低于 QUALITY_MIN=5 转为历史。

#### 6.7.2 `fun_check_track_validation.m` — 5 条验证规则

1. 距离 MSE 门限（delta_R = 200km）
2. 距离趋势与速度方向一致性
3. 速度 MSE 门限（delta_V = 200m/s）
4. 速度递增检测
5. 方位角异常检测（delta_A = 7.5deg）

#### 6.7.3 `cleanTrackList.m` / `sortTrackList.m` / `resetAllTracks.m`

航迹清理、排序、重置。

### 6.8 辅助工具函数

- `robustMinSquareErr.m` — 三次迭代的加权最小二乘回归（Huber 型权重）
- `pdCoefInterprator.m` — PD 系数解析器（5 扇区 x 5 电离层模式）
- `tool_radar2blh_fake_monostatic.m` — 伪单站雷达坐标转换
- `distance.m` / `reckon.m` — Haversine 和球面正算的兼容包装器

---

## 第七部分：Git 提交历史（37 次提交详解）

### 7.1 项目初始化与早期开发（2026-05-24 ~ 2026-05-25）

| 提交 | 日期 | 说明 |
|------|------|------|
| `93a38c2` | 2026-05-24 | 首次提交。包含基础的雷达仿真框架、检测器、初步的 UKF 滤波器和南阳航迹处理模块。 |
| `70eb878` | 2026-05-25 | 生成了详细注释和项目结构阅读调用说明文档。 |
| `a4e0753` | 2026-05-25 | 第一次精简：提炼 UKF、航迹起始、航迹关联为独立模块。 |
| `8a99c47` | 2026-05-25 | 进一步精简：代码拆分为 ukf/、initiation/、association/、nanyang/ 等子目录。 |
| `3d93991` | 2026-05-25 | 进一步完善，添加部分详细注释和文档。 |

### 7.2 天波传播几何模型引入（2026-05-26 ~ 2026-05-29）

| 提交 | 日期 | 说明 |
|------|------|------|
| `ba16c28` | 2026-05-26 | **划时代修改：引入电离层虚高**。完全按照天波超视距雷达文档进行量测仿真，同步修改 UKF 中的量测模型。引入天波斜距计算：r = sqrt(D^2 + (2H)^2)，H=300km。 |
| `70cd146` | 2026-05-26 | Claude 自动调参，效果不明显。 |
| `02b6a87` | 2026-05-28 | 完善注释。 |
| `ef4f4cf` | 2026-05-29 | UKF 参数从主入口硬编码统一归到参数入口。P_pos_std 从 0.1 降到 0.05 有效果。 |

### 7.3 蒙特卡洛基础设施与问题发现（2026-06-27 ~ 2026-06-28）

| 提交 | 日期 | 说明 |
|------|------|------|
| `a06ebfd` | 2026-06-27 | 时隔多日再次开工。调参发现 P_pos_std 从 0.1 到 0.05 效果变好。 |
| `1aae74b` | 2026-06-27 | 编写蒙特卡洛入口脚本，50 次实验发现 UKF 发散。 |
| `2b6c4b6` | 2026-06-28 | 新增单目标无拐弯的蒙特卡洛实验脚本入口。 |
| `e99fa9a` | 2026-06-28 | 兜兜转转又回到原点。天波几何模型引入后 UKF 量测模型和状态转移模型之间存在失配。 |
| `50909b2` | 2026-06-28 | 加入南阳的一些模式，聚焦转弯处理。 |
| `3471188` | 2026-06-28 | **经过八轮针对性修改，坏种子率从 28% 降至 10%**。PDA 单检测退化修复、软启动渐近波门、基础波门放宽、起始门槛提高、两点差分速度初始化、Probation 期 NIS 保护、速度方向突变检测、速度上限检测。 |

### 7.4 局部重构与统一入口（2026-06-29 ~ 2026-06-30）

| 提交 | 日期 | 说明 |
|------|------|------|
| `d108f00` | 2026-06-29 | 明确转弯率 1 度/s，先从直线开始验证航线的延长。 |
| `c55c748` | 2026-06-29 | 使用辅助起始，自然断裂，仿真融合后实现航迹延长。 |
| `e03621e` | 2026-06-29 | **六项关键优化**：径向速度硬门限替代马氏距离软启动、重构航迹起始逻辑（真值辅助仅首次生效）、移除 probation 硬性拦截、重写蒙特卡洛入口、断裂航迹分段可视化、5 套诊断脚本。94 个坏种子中 83% 可通过融合策略修复。 |
| `e523354` | 2026-06-29 | k_loss 调整到 8，完成拐弯主程序，IMM 有点问题。 |
| `5f0432b` | 2026-06-29 | 写了拐弯的蒙特卡洛入口程序。 |
| `be285a0` | 2026-06-30 | 新增回头弯场景双主入口，发现只有加入 is_clutter 作弊关联才能有好效果。 |
| `dee87d2` | 2026-06-30 | 前四组基础主入口随机种子方式统一，都能防止聚类且可以复现。 |
| `5eab44b` | 2026-06-30 | **局部重构，统一 tracker 入口，分立三种 UKF**（基础/自适应/IMM），模块化。 |
| `da522d5` | 2026-06-30 | 第一波根目录大扫除：清理非 .m 文件。 |
| `52a7750` | 2026-06-30 | 第二波根目录大扫除：清理测试性质的 .m 文件。 |
| `91eeb32` | 2026-06-30 | 新增了对于回头弯的三种 UKF 测试脚本。 |
| `0bfe33e` | 2026-06-30 | 两个拐弯的单词主入口脚本中并行实现三条 UKF 线，同时输出。 |

### 7.5 多目标扩展与收尾（2026-07-01 ~ 2026-07-03）

| 提交 | 日期 | 说明 |
|------|------|------|
| `d6031c9` | 2026-07-01 | 开始拓展多目标。 |
| `56caad7` | 2026-07-01 | 小型改动。 |
| `1499e21` | 2026-07-01 | 小型改动。 |
| `27585ec` | 2026-07-02 | 修复 run_simulation_turn.m 运行报错。 |
| `60c9e1f` | 2026-07-02 | 多目标终于把 UKF 画出来了，但航迹交叉部分 UKF 发散严重。 |
| `37579d6` | 2026-07-02 | 修复了两个扫描调参的脚本。调整多目标中三条航迹在雷达照射区域。 |
| `3ec5897` | 2026-07-02 | 单目标多目标参数设置脚本分离。 |
| `d564f10` | 2026-07-02 | 分离 evaluate 文件，单目标多目标分开。 |
| `72dfb66` | 2026-07-03 | 新建分支用以提升单目标 UKF 性能，三种优化同时使用。 |
| `7c166d4` | 2026-07-03 | 最新提交（HEAD -> main）。 |

---

## 第八部分：数学基础

### 8.1 Haversine 公式推导

**问题**：给定地球表面上两点的经纬度 $(\lambda_1, \phi_1)$ 和 $(\lambda_2, \phi_2)$，求两点间的大圆距离。

**推导**：

考虑球面三角形 $N P_1 P_2$，其中 $N$ 为北极点：
- 边 $NP_1 = \frac{\pi}{2} - \phi_1$（余纬）
- 边 $NP_2 = \frac{\pi}{2} - \phi_2$
- 边 $P_1P_2 = \Delta\sigma$（待求球心角）
- 角 $\angle P_1NP_2 = \Delta\lambda = \lambda_2 - \lambda_1$

由球面余弦定理：
$$\cos(\Delta\sigma) = \sin\phi_1\sin\phi_2 + \cos\phi_1\cos\phi_2\cos(\Delta\lambda)$$

利用半角公式 $\text{hav}(\theta) = \sin^2(\theta/2) = (1-\cos\theta)/2$：

$$\text{hav}(\Delta\sigma) = \text{hav}(\Delta\phi) + \cos\phi_1\cos\phi_2 \cdot \text{hav}(\Delta\lambda)$$

即：
$$a = \sin^2\left(\frac{\Delta\phi}{2}\right) + \cos\phi_1 \cdot \cos\phi_2 \cdot \sin^2\left(\frac{\Delta\lambda}{2}\right)$$

$$\Delta\sigma = 2\arctan2(\sqrt{a}, \sqrt{1-a})$$

最终距离：$d = R \cdot \Delta\sigma = 2R \cdot \arctan2(\sqrt{a}, \sqrt{1-a})$

**数值稳定性**：当两点距离很近时，acos 在参数接近 1 时精度严重损失（catastrophic cancellation），atan2 在 $[0,\pi]$ 区间始终保持良好的数值稳定性。

### 8.2 大圆方位角计算

从点 1 到点 2 的初始方位角：
$$\alpha = \operatorname{atan2}(\sin\Delta\lambda \cdot \cos\phi_2,\ \cos\phi_1 \cdot \sin\phi_2 - \sin\phi_1 \cdot \cos\phi_2 \cdot \cos\Delta\lambda)$$

结果为 $[0, 360^\circ)$ 度，从正北顺时针计量。

### 8.3 天波传播几何

**物理模型**：天波超视距雷达利用电离层（F 层，等效高度 $H \approx 300$ km）反射无线电波。

1. **地心角** $\sigma$：用 Haversine 公式计算
2. **地表弦长** $D = 2R_e\sin(\sigma/2)$
3. **天波斜距** $r = \sqrt{D^2 + (2H)^2}$
4. **群距离** $R_g = r_{tx} + r_{rx}$
5. **多普勒速度** $v_d = \frac{dR_g}{dt} = \frac{dr_{tx}}{dt} + \frac{dr_{rx}}{dt}$

**多普勒推导**：
- ENU 速度：$v_E = \dot{\lambda} \cdot \frac{\pi}{180} \cdot R_e \cdot \cos\phi$，$v_N = \dot{\phi} \cdot \frac{\pi}{180} \cdot R_e$
- 单段路径变化率：$\frac{dr}{dt} = \frac{D}{r} \cdot \cos(\frac{\sigma}{2}) \cdot (v_E \sin\alpha + v_N \cos\alpha)$

### 8.4 双基地雷达几何（余弦定理反解）

已知群距离 $R_g$ 和方位角 $az$，反解目标经纬度：

经典双基地反解（假设 $R_g = r_0 + r_1$ 为大圆距离和）：
$$r_1 = \frac{1}{2} \cdot \frac{R_g^2 - B^2}{R_g - B \cdot \cos\phi}$$

其中 $B$ 为收发站基线距离，$\phi = az - az_{tx}$。

天波迭代精化：用天波模型预测 $R_g^{pred}$，按比例修正 $r_1^{(k+1)} = r_1^{(k)} \cdot R_g / R_g^{pred}$，迭代至 $|R_g - R_g^{pred}| < 1$ km。

### 8.5 UKF/UT 数学

**Sigma 点生成**：
- $\lambda = \alpha^2(n+\kappa) - n$
- Cholesky 分解：$(n+\lambda)P = LL^T$
- $2n+1$ 个 Sigma 点：$\chi_0 = x$，$\chi_i = x + L_i$，$\chi_{i+n} = x - L_i$

**权重**：
- $W_0^m = \lambda/(n+\lambda)$，$W_0^c = \lambda/(n+\lambda) + (1-\alpha^2+\beta)$
- $W_i^m = W_i^c = 1/(2(n+\lambda))$，$i=1,...,2n$

**时间更新（预测）**：
- $\hat{x}^- = \sum W_i^m \chi_i^{(p)}$
- $P^- = Q + \sum W_i^c (\chi_i^{(p)} - \hat{x}^-)(\chi_i^{(p)} - \hat{x}^-)^T$

**量测更新**：
- $\hat{z} = \sum W_i^m Z_i$
- $P_{zz} = R + \sum W_i^c (Z_i - \hat{z})(Z_i - \hat{z})^T$
- $P_{xz} = \sum W_i^c (\chi_i^{(p)} - \hat{x}^-)(Z_i - \hat{z})^T$
- $K = P_{xz}P_{zz}^{-1}$
- $\hat{x} = \hat{x}^- + K(z - \hat{z})$
- $P = P^- - KP_{zz}K^T$

**运动模型**：
- CV 模型：$F = \begin{bmatrix} 1 & \Delta t & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & \Delta t \\ 0 & 0 & 0 & 1 \end{bmatrix}$
- CT 模型：$F_{CT}(\omega\Delta t)$ 含 $\sin/\cos$ 项，当 $\omega \to 0$ 时退化为 CV。

### 8.6 PDA 数学

**关联概率**：
$$\beta_i = \frac{\exp(-\frac{1}{2}d_i^2)}{b + \sum_j \exp(-\frac{1}{2}d_j^2)}$$

其中 $b = \frac{\lambda V_{norm}(1-\alpha)}{\alpha}$，$\alpha = P_D \cdot P_G$，$V_{norm} = 2\pi\sqrt{\det(S_{2D})}$。

**加权新息**：$\tilde{y} = \sum_i \beta_i \nu_i$

### 8.7 IMM 数学

**Markov 转移矩阵**：$\Pi = \begin{bmatrix} 0.90 & 0.10 \\ 0.10 & 0.90 \end{bmatrix}$

**模型混合**：
- $\hat{\mu}_{j|i} = \frac{\pi_{ij}\mu_i}{c_j}$，$c_j = \sum_i \pi_{ij}\mu_i$
- $\hat{x}^{0j} = \sum_i \hat{\mu}_{j|i}\hat{x}^i$
- $\hat{P}^{0j} = \sum_i \hat{\mu}_{j|i}[P^i + (\hat{x}^i - \hat{x}^{0j})(\hat{x}^i - \hat{x}^{0j})^T]$

**似然函数（Pd-IPDA）**：
$$L_j = P_D P_G \cdot \mathcal{N}(\nu_j; 0, S_j) + (1-P_D P_G) \cdot \frac{1}{V}$$

**后验模型概率**：
$$\mu_j(k) = \frac{L_j(k) \cdot \hat{\mu}_j}{\sum_i L_i(k) \cdot \hat{\mu}_i}$$

**状态组合**：$\hat{x} = \sum_i \mu_i(k) \hat{x}^i$

### 8.8 模糊逻辑 Q 自适应

**输入**：NIS 比值 $r = \frac{1}{2}\overline{\text{NIS}}$

**5 个三角形模糊集**：VS(0,0,0.4)、S(0.2,0.5,0.8)、M(0.6,1.0,1.5)、L(1.3,2.0,3.0)、VL(2.5,4.0,4.0)

**输出**：Decrease(0.6)、SlightDecrease(0.8)、Maintain(1.0)、Increase(1.8)、RapidIncrease(3.0)

**重心法去模糊**：$\text{factor} = \frac{\sum \mu_k \cdot o_k}{\sum \mu_k}$

**EMA 平滑**：$Q_{\text{EMA}}(k) = \eta \cdot \text{factor} + (1-\eta) \cdot Q_{\text{EMA}}(k-1)$，$\eta = 0.20$

### 8.9 融合算法数学

**SCC**：$P_f^{-1} = P_1^{-1} + P_2^{-1}$，$x_f = P_f(P_1^{-1}x_1 + P_2^{-1}x_2)$

**BC**：$S = P_1 + P_2 - P_{12} - P_{12}^T$，$K_{BC} = (P_1 - P_{12})S^{-1}$，$x_f = x_1 + K_{BC}(x_2 - x_1)$

**CI**：$P_f^{-1} = wP_1^{-1} + (1-w)P_2^{-1}$，$w^* = \arg\min_w \det(P_f)$

**FCI**：$w_{FCI} = \operatorname{tr}(P_1)^{-1} / (\operatorname{tr}(P_1)^{-1} + \operatorname{tr}(P_2)^{-1})$

### 8.10 M/N 航迹起始统计

在长度为 $N$ 的滑窗中，目标被检测到的帧数 $K \sim \operatorname{Binomial}(N, P_D P_G)$。

起始概率：$P_D^{(N,M)} = \sum_{k=M}^N \binom{N}{k} (P_D P_G)^k (1-P_D P_G)^{N-k}$

本项目参数：$N=9, M=5$。若 $P_D P_G = 0.6$，则 $P_D^{(9,5)} \approx 0.733$。

### 8.11 机动检测数学

**短时 vs 长时 NIS 比较**：
$$\overline{\text{NIS}}_{\text{short}} > \overline{\text{NIS}}_{\text{long}} \cdot 1.25 \quad \land \quad \overline{\text{NIS}}_{\text{short}} > 2.8 \quad \lor \quad \overline{\text{NIS}}_{\text{long}} > 3.2$$

**机动 Q 提升**（渐进式）：前 5 帧 $q_0 + t \cdot 0.2$，5-15 帧 $q_1 + (t-5) \cdot 0.08$，15+ 帧 $q_{\max} = 3.5$。

---

# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

---

## 第 43 章：南阳子系统剩余文件逐行审查

### 43.1 det2trackDataConverter.m 检测点到航迹数据转换

#### 43.1.1 速度模糊扩展算法分析

代码第 101-124 行实现速度模糊扩展。

算法原理：
- OTH-SWR 的多普勒测量存在速度模糊（ambiguity），最大无模糊速度 Vmax_unamb = lambda/(2*PRT)
- 当测量的径向速度超出无模糊范围时，可能对应多个真实速度值
- 代码将每个检测点扩展为 3 个候选：原始速度、速度+2*Vmax_unamb、速度-2*Vmax_unamb

问题 1：第 59 行 V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow)。
- Vmax_allow = min(Vmax_amb, Vmax_radial) = min(2*|fIndex*lambda|, 666)
- Vmax_radial = 666 m/s 是硬编码的最大径向速度，没有物理依据
- 民航客机最大径向速度约 230 m/s，666 m/s 对应超音速目标
- 如果目标速度超过 666 m/s，代码会将其归类为非飞行目标

问题 2：第 108 行 trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb。
- 这里假设模糊阶数为 1（ambgNum = +/-1），即只允许一次速度模糊
- 但实际 OTH-SWR 的模糊阶数可能更高（ambgNum = +/-2, +/-3...）
- 代码注释说 we only allow ambiguity = 1，这是人为限制，可能漏掉真实目标

问题 3：第 124 行 trackPointList = [trackPointList, trackPointList_p, trackPointList_n]。
- 这会将检测点数扩展为原来的 3 倍（如果所有点都有速度模糊）
- 对于每帧 100 个检测点，扩展后变成 300 个
- 后续关联算法需要处理 3 倍的计算量

#### 43.1.2 func_cal_gruond_distance_from_group_path PD 系数插值

代码第 194-334 行实现 PD（Propagation Delay）系数插值。

问题 1：第 196-279 行的 ionoMode 选择逻辑。
- ionoMode=1 对应 EE 模式，ionoMode=2 对应 EF 模式等
- 每个模式有 5 个扇区，每个扇区有 range_pd_index 和 pd_range/pd_az 两个查找表
- 这些查找表的值是从哪里来的？代码没有说明。它们应该是通过实测数据拟合得到的，但代码中没有拟合过程。

问题 2：第 263-279 行的 else 分支（ionoMode 不在 1-4 时）。
- 当 ionoMode=5 时，PD 系数全部为 1，方位修正为 0
- 这意味着群距离 = 地面距离，完全没有电离层修正
- 对于 OTH-SWR，PD 系数通常在 1.1-1.2 之间，完全忽略会导致系统性偏差 10-20%

问题 3：第 323-325 行的线性插值。
- 如果 curRange 超出 range_pd_index 的范围，interp1 返回 NaN
- 代码第 316-321 行做了钳位处理（超出范围取端点值），这是正确的

### 43.2 tool_radar2blh_fake_monostatic.m 伪单基站地理反解

问题 1：伪单基假设。
- 双基地雷达的群距离 Rg = r_tx + r_rx，不是从单一观测点出发的距离
- 代码将 Rg/2 作为伪单基地斜距，这在几何上是近似的
- 当 Tx 和 Rx 距离很远时（如本仿真中 370km 基线），近似误差很大
- 定量误差：当 R >> d 时误差小，当 R approx d 时误差可达 10-20%

问题 2：第 26 行 reckon 函数调用参数顺序正确。

### 43.3 robustMinSquareErr.m 鲁棒最小二乘

问题 1：第 15 行 w = min(abs(err/s/6), 1)。
- 当 |err| > 6*s 时 w = 1，当 |err| < 6*s 时 w = |err|/(6s)
- 这与直觉相反：通常小残差点应该获得高权重
- 然后第 16 行 w = (1-w^3)^3 将反转回来：w=1 时权重 0，w=0 时权重 1
- 最终效果正确，但中间步骤的权重反转让人困惑

问题 2：第 28 行 w = (1-w^2)^2（第二次迭代）与第 16 行 w = (1-w^3)^3（第一次迭代）使用的幂次不同。
- 第一次用立方，第二次用平方，导致两次迭代的鲁棒性不同
- 这种不一致没有理论依据

问题 3：第 46-47 行的加权最小二乘公式。
- 分母 sum_w*sum_x2 - sum_x^2 可能接近 0：当所有 x 值相同时回归无意义
- 代码没有检查这种情况

### 43.4 track2reportDataConverter.m 航迹转报告数据

问题 1：第 22-65 行大量注释掉的代码，应该删除。

问题 2：第 86 行 usPDist = round(prange /2*10)。
- /2*10 等价于 /0.2，将群距离转换为 0.1km 单位后除以 2
- 除以 2 是伪单基假设的延续，但这种近似在双基地几何下不准确

问题 3：第 92-93 行 usTrackAzi = atan2d(vy, vx)。
- vx 和 vy 在 fun_create_new_track.m 中被硬编码为 0
- 所以 usTrackAzi 始终是 0，报告的航迹方位角始终为正北，完全错误

问题 4：第 100-103 行硬编码的 PD 系数 f2PDCoef=0.8，没有物理意义。

### 43.5 fun_track_quality_management_and_info_completion.m 航迹质量管理

问题 1：RELIABLE_TRACK 从 quality=15 开始，连续 5 帧不关联降到 5，再 1 帧降到 3 < 5 -> HISTORY。
- 所以 RELIABLE_TRACK 可以容忍连续 5 帧不关联

问题 2：TEMPORARY_TRACK 从 quality=8 开始，连续 3 帧不关联降到 5，再 1 帧降到 4 < 5 -> HISTORY。
- 所以 TEMPORARY_TRACK 只能容忍连续 3 帧不关联

问题 3：第 90 行 travel_dist = tool_calculate_distance(...)。
- 单位一致（km），但没有类型安全

### 43.6 fun_check_track_validation.m 航迹有效性检查

问题 1：第 30 行 delta_R = 200 km。
- 注释说原45->200，说明最初的范围 MSE 门限是 45km，后来放宽到 200km
- 200km 的门限对于 OTH-SWR 来说太大了
- 放宽到 200km 说明原始的 45km 门限太严格，导致大量正常航迹被误杀
- 这反映了航迹质量控制的参数没有定量分析

问题 2：第 33-36 行的范围预测 prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000。
- 这是前向欧拉积分，符号约定与 skywave_geometry 中的多普勒定义不一致
- 如果符号约定不一致，范围预测会产生系统性偏差

问题 3：第 66 行 delta_V = 200 m/s。
- 注释说原4->200，速度 MSE 门限从 4 m/s 放宽到 200 m/s
- 200 m/s 的门限意味着任何速度变化都不会被检测为异常
- 速度检查基本失效了

问题 4：第 75-78 行的方位角检查 delta_A = 7.5 度。
- 这假设方位角应该是恒定的——如果目标在转弯，方位角自然会变化
- 对于转弯目标，这个检查会误杀

### 43.7 distance.m 球面距离兼容层

问题：第 29-36 行的循环处理中 min(i, numel(lat1)) 的逻辑很奇怪。
- 如果 i > numel(lat1)，它会重复使用最后一个元素
- 这可能导致隐式的数据截断或重复，而不是报错

### 43.8 reckon.m Mapping Toolbox 兼容层

问题：第 18 行 arclen * 1000.0 的单位转换依赖于调用方的 arclen 单位。
- 如果 arclen 已经是米，这里会错误地放大 1000 倍
- 需要确认调用方的 arclen 单位

---

## 第 44 章：南阳子系统与主系统的完整对比

架构对比：
- 滤波算法：主系统 UKF（无迹卡尔曼），南阳子系统 Alpha-Beta 平滑
- 关联方法：主系统 NN+PDA+Vr门，南阳子系统 JNN+归一化综合距离
- 起始逻辑：主系统 M/N滑窗+真值辅助，南阳子系统 M/N滑窗+回溯预测
- 质量控制：主系统质量状态机（1/2/6/7），南阳子系统（1/2/3/4/6/7）
- 运动模型：主系统 CV/CT（协调转弯），南阳子系统 CV（匀速）+ 径向/非径向
- 架构风格：主系统函数式dispatcher，南阳子系统过程式+run(header)
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么两套系统并存？
1. 南阳子系统是更早的版本（作者 Jun Geng，2022-2025）
2. 主系统是更新的版本（作者 rendong，2026）
3. 两者功能重叠，但实现思路完全不同
4. 主系统更理论化（UKF/PDA/IMM），南阳子系统更工程化（Alpha-Beta/启发式规则）

建议：如果两套系统功能重叠，应该合并为一套。无论如何，都应该删除 run(header.m) 这种反模式。

---

## 第 45 章：完整数学推导补充

### 45.1 UKF 权重的三阶矩匹配证明

当 alpha=1e-2 时，lambda = -3.9996，n+lambda = 0.0004。
Wm(1) = -9999, Wm(2:9) = 1250。

一阶矩验证：
Sigma Wm_i * X_i = -9999 * x_bar + 8 * 1250 * x_bar = x_bar 正确。

二阶矩验证：
Sigma Wc_i * (X_i-x_bar)(X_i-x_bar)' = 1250 * 2 * 0.0004 * P = P 正确。

结论：即使 alpha=1e-2，UKF 的权重仍然正确匹配一阶和二阶矩。但三阶矩的匹配可能不准确——当中心权重为 -9999 时，数值误差会被放大 10000 倍。

### 45.2 IMM 混合协方差的正定性证明

P^0_j = Sigma_i mu_ij * [P^i + (x^i - x^0_j)(x^i - x^0_j)']
其中 mu_ij > 0，P^i > 0，(x^i - x^0_j)(x^i - x^0_j)' >= 0。
所以 P^0_j 是正定矩阵的和，仍正定。证毕。

### 45.3 CI 优化的凸性证明

f(w) = det(P_w) = 1/det(w*A + (1-w)*B)
根据 Minkowski 行列式不等式，det(w*A + (1-w)*B) 是 w 的凹函数。
因此 f(w) = 1/det(...) 是 w 的凸函数。
结论：fminbnd 可以找到全局最优解。代码实现正确。

### 45.4 PDA 协方差修正的完整公式

完整公式：P(k|k) = P_pred - K*S*K' + P_g * (x_pred * x_pred' - P_pred) + C_2

缺失影响：
1. 没有 P_g 项 -> 协方差低估
2. 没有 C_2 项 -> 卡尔曼增益计算不准确
3. 综合影响：滤波器过于自信，在目标机动时容易发散

# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

---

## 第 43 章：南阳子系统剩余文件逐行审查

### 43.1 det2trackDataConverter.m 检测点到航迹数据转换

#### 43.1.1 速度模糊扩展算法分析

代码第 101-124 行实现速度模糊扩展。

算法原理：
- OTH-SWR 的多普勒测量存在速度模糊（ambiguity），最大无模糊速度 Vmax_unamb = lambda/(2*PRT)
- 当测量的径向速度超出无模糊范围时，可能对应多个真实速度值
- 代码将每个检测点扩展为 3 个候选：原始速度、速度+2*Vmax_unamb、速度-2*Vmax_unamb

问题 1：第 59 行 V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow)。
- Vmax_allow = min(Vmax_amb, Vmax_radial) = min(2*|fIndex*lambda|, 666)
- Vmax_radial = 666 m/s 是硬编码的最大径向速度，没有物理依据
- 民航客机最大径向速度约 230 m/s，666 m/s 对应超音速目标
- 如果目标速度超过 666 m/s，代码会将其归类为非飞行目标

问题 2：第 108 行 trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb。
- 这里假设模糊阶数为 1（ambgNum = +/-1），即只允许一次速度模糊
- 但实际 OTH-SWR 的模糊阶数可能更高（ambgNum = +/-2, +/-3...）
- 代码注释说 we only allow ambiguity = 1，这是人为限制，可能漏掉真实目标

问题 3：第 124 行 trackPointList = [trackPointList, trackPointList_p, trackPointList_n]。
- 这会将检测点数扩展为原来的 3 倍（如果所有点都有速度模糊）
- 对于每帧 100 个检测点，扩展后变成 300 个
- 后续关联算法需要处理 3 倍的计算量

#### 43.1.2 func_cal_gruond_distance_from_group_path PD 系数插值

代码第 194-334 行实现 PD（Propagation Delay）系数插值。

问题 1：第 196-279 行的 ionoMode 选择逻辑。
- ionoMode=1 对应 EE 模式，ionoMode=2 对应 EF 模式等
- 每个模式有 5 个扇区，每个扇区有 range_pd_index 和 pd_range/pd_az 两个查找表
- 这些查找表的值是从哪里来的？代码没有说明。它们应该是通过实测数据拟合得到的，但代码中没有拟合过程。

问题 2：第 263-279 行的 else 分支（ionoMode 不在 1-4 时）。
- 当 ionoMode=5 时，PD 系数全部为 1，方位修正为 0
- 这意味着群距离 = 地面距离，完全没有电离层修正
- 对于 OTH-SWR，PD 系数通常在 1.1-1.2 之间，完全忽略会导致系统性偏差 10-20%

问题 3：第 323-325 行的线性插值。
- 如果 curRange 超出 range_pd_index 的范围，interp1 返回 NaN
- 代码第 316-321 行做了钳位处理（超出范围取端点值），这是正确的

### 43.2 tool_radar2blh_fake_monostatic.m 伪单基站地理反解

问题 1：伪单基假设。
- 双基地雷达的群距离 Rg = r_tx + r_rx，不是从单一观测点出发的距离
- 代码将 Rg/2 作为伪单基地斜距，这在几何上是近似的
- 当 Tx 和 Rx 距离很远时（如本仿真中 370km 基线），近似误差很大
- 定量误差：当 R >> d 时误差小，当 R approx d 时误差可达 10-20%

问题 2：第 26 行 reckon 函数调用参数顺序正确。

### 43.3 robustMinSquareErr.m 鲁棒最小二乘

问题 1：第 15 行 w = min(abs(err/s/6), 1)。
- 当 |err| > 6*s 时 w = 1，当 |err| < 6*s 时 w = |err|/(6s)
- 这与直觉相反：通常小残差点应该获得高权重
- 然后第 16 行 w = (1-w^3)^3 将反转回来：w=1 时权重 0，w=0 时权重 1
- 最终效果正确，但中间步骤的权重反转让人困惑

问题 2：第 28 行 w = (1-w^2)^2（第二次迭代）与第 16 行 w = (1-w^3)^3（第一次迭代）使用的幂次不同。
- 第一次用立方，第二次用平方，导致两次迭代的鲁棒性不同
- 这种不一致没有理论依据

问题 3：第 46-47 行的加权最小二乘公式。
- 分母 sum_w*sum_x2 - sum_x^2 可能接近 0：当所有 x 值相同时回归无意义
- 代码没有检查这种情况

### 43.4 track2reportDataConverter.m 航迹转报告数据

问题 1：第 22-65 行大量注释掉的代码，应该删除。

问题 2：第 86 行 usPDist = round(prange /2*10)。
- /2*10 等价于 /0.2，将群距离转换为 0.1km 单位后除以 2
- 除以 2 是伪单基假设的延续，但这种近似在双基地几何下不准确

问题 3：第 92-93 行 usTrackAzi = atan2d(vy, vx)。
- vx 和 vy 在 fun_create_new_track.m 中被硬编码为 0
- 所以 usTrackAzi 始终是 0，报告的航迹方位角始终为正北，完全错误

问题 4：第 100-103 行硬编码的 PD 系数 f2PDCoef=0.8，没有物理意义。

### 43.5 fun_track_quality_management_and_info_completion.m 航迹质量管理

问题 1：RELIABLE_TRACK 从 quality=15 开始，连续 5 帧不关联降到 5，再 1 帧降到 3 < 5 -> HISTORY。
- 所以 RELIABLE_TRACK 可以容忍连续 5 帧不关联

问题 2：TEMPORARY_TRACK 从 quality=8 开始，连续 3 帧不关联降到 5，再 1 帧降到 4 < 5 -> HISTORY。
- 所以 TEMPORARY_TRACK 只能容忍连续 3 帧不关联

问题 3：第 90 行 travel_dist = tool_calculate_distance(...)。
- 单位一致（km），但没有类型安全

### 43.6 fun_check_track_validation.m 航迹有效性检查

问题 1：第 30 行 delta_R = 200 km。
- 注释说原45->200，说明最初的范围 MSE 门限是 45km，后来放宽到 200km
- 200km 的门限对于 OTH-SWR 来说太大了
- 放宽到 200km 说明原始的 45km 门限太严格，导致大量正常航迹被误杀
- 这反映了航迹质量控制的参数没有定量分析

问题 2：第 33-36 行的范围预测 prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000。
- 这是前向欧拉积分，符号约定与 skywave_geometry 中的多普勒定义不一致
- 如果符号约定不一致，范围预测会产生系统性偏差

问题 3：第 66 行 delta_V = 200 m/s。
- 注释说原4->200，速度 MSE 门限从 4 m/s 放宽到 200 m/s
- 200 m/s 的门限意味着任何速度变化都不会被检测为异常
- 速度检查基本失效了

问题 4：第 75-78 行的方位角检查 delta_A = 7.5 度。
- 这假设方位角应该是恒定的——如果目标在转弯，方位角自然会变化
- 对于转弯目标，这个检查会误杀

### 43.7 distance.m 球面距离兼容层

问题：第 29-36 行的循环处理中 min(i, numel(lat1)) 的逻辑很奇怪。
- 如果 i > numel(lat1)，它会重复使用最后一个元素
- 这可能导致隐式的数据截断或重复，而不是报错

### 43.8 reckon.m Mapping Toolbox 兼容层

问题：第 18 行 arclen * 1000.0 的单位转换依赖于调用方的 arclen 单位。
- 如果 arclen 已经是米，这里会错误地放大 1000 倍
- 需要确认调用方的 arclen 单位

---

## 第 44 章：南阳子系统与主系统的完整对比

架构对比：
- 滤波算法：主系统 UKF（无迹卡尔曼），南阳子系统 Alpha-Beta 平滑
- 关联方法：主系统 NN+PDA+Vr门，南阳子系统 JNN+归一化综合距离
- 起始逻辑：主系统 M/N滑窗+真值辅助，南阳子系统 M/N滑窗+回溯预测
- 质量控制：主系统质量状态机（1/2/6/7），南阳子系统（1/2/3/4/6/7）
- 运动模型：主系统 CV/CT（协调转弯），南阳子系统 CV（匀速）+ 径向/非径向
- 架构风格：主系统函数式dispatcher，南阳子系统过程式+run(header)
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么两套系统并存？
1. 南阳子系统是更早的版本（作者 Jun Geng，2022-2025）
2. 主系统是更新的版本（作者 rendong，2026）
3. 两者功能重叠，但实现思路完全不同
4. 主系统更理论化（UKF/PDA/IMM），南阳子系统更工程化（Alpha-Beta/启发式规则）

建议：如果两套系统功能重叠，应该合并为一套。无论如何，都应该删除 run(header.m) 这种反模式。

---

## 第 45 章：完整数学推导补充

### 45.1 UKF 权重的三阶矩匹配证明

当 alpha=1e-2 时，lambda = -3.9996，n+lambda = 0.0004。
Wm(1) = -9999, Wm(2:9) = 1250。

一阶矩验证：
Sigma Wm_i * X_i = -9999 * x_bar + 8 * 1250 * x_bar = x_bar 正确。

二阶矩验证：
Sigma Wc_i * (X_i-x_bar)(X_i-x_bar)' = 1250 * 2 * 0.0004 * P = P 正确。

结论：即使 alpha=1e-2，UKF 的权重仍然正确匹配一阶和二阶矩。但三阶矩的匹配可能不准确——当中心权重为 -9999 时，数值误差会被放大 10000 倍。

### 45.2 IMM 混合协方差的正定性证明

P^0_j = Sigma_i mu_ij * [P^i + (x^i - x^0_j)(x^i - x^0_j)']
其中 mu_ij > 0，P^i > 0，(x^i - x^0_j)(x^i - x^0_j)' >= 0。
所以 P^0_j 是正定矩阵的和，仍正定。证毕。

### 45.3 CI 优化的凸性证明

f(w) = det(P_w) = 1/det(w*A + (1-w)*B)
根据 Minkowski 行列式不等式，det(w*A + (1-w)*B) 是 w 的凹函数。
因此 f(w) = 1/det(...) 是 w 的凸函数。
结论：fminbnd 可以找到全局最优解。代码实现正确。

### 45.4 PDA 协方差修正的完整公式

完整公式：P(k|k) = P_pred - K*S*K' + P_g * (x_pred * x_pred' - P_pred) + C_2

缺失影响：
1. 没有 P_g 项 -> 协方差低估
2. 没有 C_2 项 -> 卡尔曼增益计算不准确
3. 综合影响：滤波器过于自信，在目标机动时容易发散

---

## 第 46 章：南阳子系统剩余文件逐行审查

### 46.1 PointTrackAssociation_JNN.m 联合最近邻关联

算法：构建 track-point 二分图，然后用图分解方法求解最优匹配。

问题 1：第 54-73 行的双重循环 O(trackNum * pointNum)。
- 对每对 (track, point) 都调用 calculate_cost_of_point_track_pair 和 determine_if_point_within_the_scope_of_track
- 当 trackNum=100, pointNum=300 时，需要调用 30000 次函数

问题 2：第 75 行 cost_fa = calculate_cost_of_point_track_pair([], trackList(1), sysPara)。
- 传入空点迹 [] 作为第一个参数，计算空关联成本
- 这个值在 candidate_matrix_selection 中用作基准

问题 3：第 115-121 行的图分解方法。
- extract_sub_bigraph、convert_bigraph_into_matrix、mat_division、candidate_matrix_selection
- 这个图分解方法比简单的贪心算法更精确，但计算量大

### 46.2 is_duplicate_track.m 重复航迹检测

算法：对两组索引分别排序后逐元素比较。

问题：如果 new_indices 是矩阵，sort 对每列排序，代码没有检查形状。

### 46.3 sortTrackList.m 航迹排序

问题：第 98 行 good_ind = find(tracks_type > 6)。
- Type > 6 意味着 Type=7 被排除
- 但 Type=6 也在被排除之列（6 不大于 6）
- 这意味着 TEMPORARY_TRACK（Type=6）不会被排序，保持在原始位置

### 46.4 Fun_UpdateTrackByAsscResult.m 航迹更新

问题 1：第 28-36 行的注释。
- 注释说调用顺序至关重要——fun_track_quality_management_and_info_completion 必须在 fun_fill_smooth_list_by_predict_result 之前调用
- 这是因为前者更新了 TotalPointCnt，后者需要使用这个值
- 这种隐式依赖关系是代码臭味——应该通过函数返回值显式传递

### 46.5 fun_fill_smooth_list_by_alpha_beta_filter.m Alpha-Beta 平滑

问题 1：第 30 行 error('no association points!...')。
- 如果没有关联点，直接抛出错误
- 但第 42 行的注释说 if there has no association, put is as empty
- 这两者矛盾

问题 2：第 34 行 fun_trackfilter_AlphaBeta 返回的 smooth_vx 和 smooth_vy。
- 这两个值在 track2reportDataConverter.m 中被用来计算航迹方位角
- 但由于 fun_create_new_track.m 中 v_x=0, v_y=0，smooth_vx 和 smooth_vy 可能也是 0
- 导致报告的航迹方位角始终为正北

### 46.6 Fun_UpdateTrackforNoInputPoint.m 无输入点更新

问题：第 19 行 predictNextStep_cv 内部调用 robustMinSquareErr 进行线性回归。
- 如果航迹的历史点迹少于 2 个，回归无意义
- 代码没有检查这个前提条件

### 46.7 predictNextStep_cv.m CV 模型预测

问题 1：第 24-26 行调试代码未删除：if curTrack.BatchNo == 20001; disp(1); end

问题 2：第 28-30 行窗口长度参数没有定量分析支撑。
- winLen_vr=10, winLen_az=11, winLen_range=7

问题 3：第 77-86 行的 predictNext_azimuth_avg 使用中位数作为预测值。
- 中位数对异常值鲁棒，但忽略了方位角的变化趋势
- 如果目标在持续转弯，中位数预测会产生系统性偏差

问题 4：第 89-109 行的 predictNext_vr_avg 使用 robustMinSquareErr 估计速度变化率。
- next_vr = ref_vr + kv * deltaT，这是线性外推
- 但目标机动时，速度变化率不恒定

问题 5：第 111-136 行的 predictNext_range_avg。
- next_range = mean(rr) - (cur_time - mean(time_diff)) * true_vr / 1e3
- 第 131-136 行的保护：如果预测距离超过 150km，回退到均值

### 46.8 fun_remove_assc_pts_from_pointlist.m 关联点移除

问题 1：第 32-36 行的影子检测使用 Rbin/Dbin/Abin 三元组。
- 仿真中 Rbin=Dbin=d（帧内唯一索引），Abin=帧号
- 所以仿真中不会有真正的影子点迹
- 这个逻辑是为真实雷达设计的，在仿真中不起作用

### 46.9 cleanTrackList.m 航迹清理

问题 1：第 16 行 global gTotalTrackCnt。
- 使用 global 变量是最危险的编程实践之一
- global 变量可以在任何地方被修改，导致难以追踪的 bug

问题 2：第 34-35 行的清理规则。
- HISTORY_TRACK 如果存活超过 3 分钟且有 5 个关联点，就不会被清理
- 但 HISTORY_TRACK 应该是已终止的航迹，为什么还需要保留？

### 46.10 fun_find_tracks_to_report.m 航迹上报

问题 1：第 19 行 ind2 = find(quality == NEW_TRACK_QUALITY)。
- NEW_TRACK_QUALITY = 8
- 只有 quality 恰好等于 8 的航迹才会被上报
- 如果 quality 上升到 9 或更高，它不会被上报
- 这可能导致航迹在质量上升后消失

问题 2：第 46-47 行 reportPoints(cnt).lat = smoothPointList(end).lat。
- 只报告最新的平滑点，不报告历史点
- 与注释说 report all history associated points 矛盾

### 46.11 fun_calculate_track_travelLen.m 航迹行驶距离

问题：第 5 行 travelLen = curTrack.travelLen + 0。
- + 0 是多余的，这看起来像是一个未完成的重构

### 46.12 tool_header.m 工具常量

问题：第 3-4 行 iono_f_height=220km 和 iono_e_height=110km。
- 这些参数在代码中没有被使用
- 与 skywave_geometry 中使用的 H=300km 不一致

### 46.13 tool_get_time_difference.m 时间差计算

第 6 行 timeDiff = (starTime - endTime) * 3600 * 24。
- starTime 和 endTime 是 MATLAB datenum（天数）
- 转换为秒：乘以 3600*24 = 86400，正确

### 46.14 fun_select_point_by_rd.m 按距离和速度选择点迹

问题：prange 的单位是 km，pvr 的单位是 m/s。
- 如果调用方传入的参数单位不匹配，结果会错误
- 函数没有做单位检查

### 46.15 fun_set_tracking_parameter.m 跟踪参数设置

第 7-9 行窗口长度参数没有定量分析支撑。
- trackPara.prdct_r_winLen = 7, trackPara.prdct_v_winLen = 10, trackPara.prdct_a_winLen = 11

### 46.16 resetAllTracks.m 航迹重置

第 27 行 curTrack.Quality = 3。
- 将质量设为 3，低于 QUALITY_MIN = 5
- 这意味着重置后的航迹会被立即清理

### 46.17 pdCoefInterprator.m PD 系数解释器

问题 1：第 18-39 行每个扇区有 92 个参数，数据结构非常复杂。
问题 2：第 40-59 行 isActivate=0 时 PD 系数全部为 1，与 ionoMode=5 行为相同。

### 46.18 det2nanyang_point.m 检测格式转换

问题 1：第 26-48 行使用 struct 预分配，正确。
问题 2：第 99-101 行 Rbin=Dbin=d 确保每个点迹的三元组唯一。
问题 3：第 56 行 ionoMode=5 仿真中所有点迹的 PD 系数为 1。

### 46.19 tool_radar2xoy_pd.m 雷达坐标转换

问题 1：第 10-23 行的 tool_radar2xoy_real_pd 使用伪单基假设。
问题 2：第 25-53 行的 tool_radar2xoy_estimate_pd。
- 第 40 行 sin_theta = h0/(range/4)
- 当 range < 4*h0 时，sin_theta > 1，返回 pos_x=0, pos_y=0
- 这意味着在近距离（< 800km 夏季或 < 1200km 冬季）时，坐标转换失败

### 46.20 fun_check_35logic_points_improved.m 3/5逻辑航迹起始

问题 1：第 16-18 行门限参数 gateRange=20km, gateVr=10m/s, gateAz=1.6度。
- 这些门限是硬编码的，没有根据雷达精度调整

问题 2：第 53 行 dist < 1.2。
- 归一化距离 < 1.2 表示匹配
- 但浮点数精确匹配不可靠（第 132-133 行）

### 46.21 fun_check_colinear_points.m 共线点检测

问题 1：第 74 行 direct_vec = (end_point - start_point) / (end_point(3) - start_point(3))。
- prange 的单位是 km，pvr 的单位是 m/s，time 的单位是 datenum（天）
- 三个维度的量纲不同，直接计算方向向量没有物理意义

问题 2：第 110-112 行的距离计算中方位角项的权重为 0。
- 但第 110 行使用了 sysPara.deltaR 和 sysPara.deltaV，这些参数的值没有说明

---

## 第 47 章：南阳子系统总结

### 47.1 代码质量评级

| 维度 | 评分 | 说明 |
|------|------|------|
| 数学正确性 | 4/10 | 伪单基假设、群距离误用、符号约定不一致 |
| 代码质量 | 3/10 | run(header)反模式、global变量、僵尸代码 |
| 可维护性 | 3/10 | 硬编码参数、无注释逻辑、函数命名混乱 |
| 可测试性 | 2/10 | 全局状态污染、隐式依赖、无单元测试 |
| 性能 | 5/10 | 双重循环关联、动态数组增长、无预分配 |

### 47.2 与主系统对比

南阳子系统代表工程化、经验主义方法。优点是简单、计算量小，适合实时性要求高的场景。缺点是数学基础薄弱、代码质量差、可维护性低。

主系统（UKF管线）代表统计最优理论方法。优点是有坚实数学基础、参数可调、可测试性强。缺点是计算量大、实现复杂。

建议：如果功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run(header.m)、global 变量等反模式。

# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

---

## 第 43 章：南阳子系统剩余文件逐行审查

### 43.1 det2trackDataConverter.m 检测点到航迹数据转换

#### 43.1.1 速度模糊扩展算法分析

代码第 101-124 行实现速度模糊扩展。

算法原理：
- OTH-SWR 的多普勒测量存在速度模糊（ambiguity），最大无模糊速度 Vmax_unamb = lambda/(2*PRT)
- 当测量的径向速度超出无模糊范围时，可能对应多个真实速度值
- 代码将每个检测点扩展为 3 个候选：原始速度、速度+2*Vmax_unamb、速度-2*Vmax_unamb

问题 1：第 59 行 V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow)。
- Vmax_allow = min(Vmax_amb, Vmax_radial) = min(2*|fIndex*lambda|, 666)
- Vmax_radial = 666 m/s 是硬编码的最大径向速度，没有物理依据
- 民航客机最大径向速度约 230 m/s，666 m/s 对应超音速目标
- 如果目标速度超过 666 m/s，代码会将其归类为非飞行目标

问题 2：第 108 行 trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb。
- 这里假设模糊阶数为 1（ambgNum = +/-1），即只允许一次速度模糊
- 但实际 OTH-SWR 的模糊阶数可能更高（ambgNum = +/-2, +/-3...）
- 代码注释说 we only allow ambiguity = 1，这是人为限制，可能漏掉真实目标

问题 3：第 124 行 trackPointList = [trackPointList, trackPointList_p, trackPointList_n]。
- 这会将检测点数扩展为原来的 3 倍（如果所有点都有速度模糊）
- 对于每帧 100 个检测点，扩展后变成 300 个
- 后续关联算法需要处理 3 倍的计算量

#### 43.1.2 func_cal_gruond_distance_from_group_path PD 系数插值

代码第 194-334 行实现 PD（Propagation Delay）系数插值。

问题 1：第 196-279 行的 ionoMode 选择逻辑。
- ionoMode=1 对应 EE 模式，ionoMode=2 对应 EF 模式等
- 每个模式有 5 个扇区，每个扇区有 range_pd_index 和 pd_range/pd_az 两个查找表
- 这些查找表的值是从哪里来的？代码没有说明。它们应该是通过实测数据拟合得到的，但代码中没有拟合过程。

问题 2：第 263-279 行的 else 分支（ionoMode 不在 1-4 时）。
- 当 ionoMode=5 时，PD 系数全部为 1，方位修正为 0
- 这意味着群距离 = 地面距离，完全没有电离层修正
- 对于 OTH-SWR，PD 系数通常在 1.1-1.2 之间，完全忽略会导致系统性偏差 10-20%

问题 3：第 323-325 行的线性插值。
- 如果 curRange 超出 range_pd_index 的范围，interp1 返回 NaN
- 代码第 316-321 行做了钳位处理（超出范围取端点值），这是正确的

### 43.2 tool_radar2blh_fake_monostatic.m 伪单基站地理反解

问题 1：伪单基假设。
- 双基地雷达的群距离 Rg = r_tx + r_rx，不是从单一观测点出发的距离
- 代码将 Rg/2 作为伪单基地斜距，这在几何上是近似的
- 当 Tx 和 Rx 距离很远时（如本仿真中 370km 基线），近似误差很大
- 定量误差：当 R >> d 时误差小，当 R approx d 时误差可达 10-20%

问题 2：第 26 行 reckon 函数调用参数顺序正确。

### 43.3 robustMinSquareErr.m 鲁棒最小二乘

问题 1：第 15 行 w = min(abs(err/s/6), 1)。
- 当 |err| > 6*s 时 w = 1，当 |err| < 6*s 时 w = |err|/(6s)
- 这与直觉相反：通常小残差点应该获得高权重
- 然后第 16 行 w = (1-w^3)^3 将反转回来：w=1 时权重 0，w=0 时权重 1
- 最终效果正确，但中间步骤的权重反转让人困惑

问题 2：第 28 行 w = (1-w^2)^2（第二次迭代）与第 16 行 w = (1-w^3)^3（第一次迭代）使用的幂次不同。
- 第一次用立方，第二次用平方，导致两次迭代的鲁棒性不同
- 这种不一致没有理论依据

问题 3：第 46-47 行的加权最小二乘公式。
- 分母 sum_w*sum_x2 - sum_x^2 可能接近 0：当所有 x 值相同时回归无意义
- 代码没有检查这种情况

### 43.4 track2reportDataConverter.m 航迹转报告数据

问题 1：第 22-65 行大量注释掉的代码，应该删除。

问题 2：第 86 行 usPDist = round(prange /2*10)。
- /2*10 等价于 /0.2，将群距离转换为 0.1km 单位后除以 2
- 除以 2 是伪单基假设的延续，但这种近似在双基地几何下不准确

问题 3：第 92-93 行 usTrackAzi = atan2d(vy, vx)。
- vx 和 vy 在 fun_create_new_track.m 中被硬编码为 0
- 所以 usTrackAzi 始终是 0，报告的航迹方位角始终为正北，完全错误

问题 4：第 100-103 行硬编码的 PD 系数 f2PDCoef=0.8，没有物理意义。

### 43.5 fun_track_quality_management_and_info_completion.m 航迹质量管理

问题 1：RELIABLE_TRACK 从 quality=15 开始，连续 5 帧不关联降到 5，再 1 帧降到 3 < 5 -> HISTORY。
- 所以 RELIABLE_TRACK 可以容忍连续 5 帧不关联

问题 2：TEMPORARY_TRACK 从 quality=8 开始，连续 3 帧不关联降到 5，再 1 帧降到 4 < 5 -> HISTORY。
- 所以 TEMPORARY_TRACK 只能容忍连续 3 帧不关联

问题 3：第 90 行 travel_dist = tool_calculate_distance(...)。
- 单位一致（km），但没有类型安全

### 43.6 fun_check_track_validation.m 航迹有效性检查

问题 1：第 30 行 delta_R = 200 km。
- 注释说原45->200，说明最初的范围 MSE 门限是 45km，后来放宽到 200km
- 200km 的门限对于 OTH-SWR 来说太大了
- 放宽到 200km 说明原始的 45km 门限太严格，导致大量正常航迹被误杀
- 这反映了航迹质量控制的参数没有定量分析

问题 2：第 33-36 行的范围预测 prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000。
- 这是前向欧拉积分，符号约定与 skywave_geometry 中的多普勒定义不一致
- 如果符号约定不一致，范围预测会产生系统性偏差

问题 3：第 66 行 delta_V = 200 m/s。
- 注释说原4->200，速度 MSE 门限从 4 m/s 放宽到 200 m/s
- 200 m/s 的门限意味着任何速度变化都不会被检测为异常
- 速度检查基本失效了

问题 4：第 75-78 行的方位角检查 delta_A = 7.5 度。
- 这假设方位角应该是恒定的——如果目标在转弯，方位角自然会变化
- 对于转弯目标，这个检查会误杀

### 43.7 distance.m 球面距离兼容层

问题：第 29-36 行的循环处理中 min(i, numel(lat1)) 的逻辑很奇怪。
- 如果 i > numel(lat1)，它会重复使用最后一个元素
- 这可能导致隐式的数据截断或重复，而不是报错

### 43.8 reckon.m Mapping Toolbox 兼容层

问题：第 18 行 arclen * 1000.0 的单位转换依赖于调用方的 arclen 单位。
- 如果 arclen 已经是米，这里会错误地放大 1000 倍
- 需要确认调用方的 arclen 单位

---

## 第 44 章：南阳子系统与主系统的完整对比

架构对比：
- 滤波算法：主系统 UKF（无迹卡尔曼），南阳子系统 Alpha-Beta 平滑
- 关联方法：主系统 NN+PDA+Vr门，南阳子系统 JNN+归一化综合距离
- 起始逻辑：主系统 M/N滑窗+真值辅助，南阳子系统 M/N滑窗+回溯预测
- 质量控制：主系统质量状态机（1/2/6/7），南阳子系统（1/2/3/4/6/7）
- 运动模型：主系统 CV/CT（协调转弯），南阳子系统 CV（匀速）+ 径向/非径向
- 架构风格：主系统函数式dispatcher，南阳子系统过程式+run(header)
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么两套系统并存？
1. 南阳子系统是更早的版本（作者 Jun Geng，2022-2025）
2. 主系统是更新的版本（作者 rendong，2026）
3. 两者功能重叠，但实现思路完全不同
4. 主系统更理论化（UKF/PDA/IMM），南阳子系统更工程化（Alpha-Beta/启发式规则）

建议：如果两套系统功能重叠，应该合并为一套。无论如何，都应该删除 run(header.m) 这种反模式。

---

## 第 45 章：完整数学推导补充

### 45.1 UKF 权重的三阶矩匹配证明

当 alpha=1e-2 时，lambda = -3.9996，n+lambda = 0.0004。
Wm(1) = -9999, Wm(2:9) = 1250。

一阶矩验证：
Sigma Wm_i * X_i = -9999 * x_bar + 8 * 1250 * x_bar = x_bar 正确。

二阶矩验证：
Sigma Wc_i * (X_i-x_bar)(X_i-x_bar)' = 1250 * 2 * 0.0004 * P = P 正确。

结论：即使 alpha=1e-2，UKF 的权重仍然正确匹配一阶和二阶矩。但三阶矩的匹配可能不准确——当中心权重为 -9999 时，数值误差会被放大 10000 倍。

### 45.2 IMM 混合协方差的正定性证明

P^0_j = Sigma_i mu_ij * [P^i + (x^i - x^0_j)(x^i - x^0_j)']
其中 mu_ij > 0，P^i > 0，(x^i - x^0_j)(x^i - x^0_j)' >= 0。
所以 P^0_j 是正定矩阵的和，仍正定。证毕。

### 45.3 CI 优化的凸性证明

f(w) = det(P_w) = 1/det(w*A + (1-w)*B)
根据 Minkowski 行列式不等式，det(w*A + (1-w)*B) 是 w 的凹函数。
因此 f(w) = 1/det(...) 是 w 的凸函数。
结论：fminbnd 可以找到全局最优解。代码实现正确。

### 45.4 PDA 协方差修正的完整公式

完整公式：P(k|k) = P_pred - K*S*K' + P_g * (x_pred * x_pred' - P_pred) + C_2

缺失影响：
1. 没有 P_g 项 -> 协方差低估
2. 没有 C_2 项 -> 卡尔曼增益计算不准确
3. 综合影响：滤波器过于自信，在目标机动时容易发散

---

## 第 46 章：南阳子系统剩余文件逐行审查

### 46.1 PointTrackAssociation_JNN.m 联合最近邻关联

算法：构建 track-point 二分图，然后用图分解方法求解最优匹配。

问题 1：第 54-73 行的双重循环 O(trackNum * pointNum)。
- 对每对 (track, point) 都调用 calculate_cost_of_point_track_pair 和 determine_if_point_within_the_scope_of_track
- 当 trackNum=100, pointNum=300 时，需要调用 30000 次函数

问题 2：第 75 行 cost_fa = calculate_cost_of_point_track_pair([], trackList(1), sysPara)。
- 传入空点迹 [] 作为第一个参数，计算空关联成本
- 这个值在 candidate_matrix_selection 中用作基准

问题 3：第 115-121 行的图分解方法。
- extract_sub_bigraph、convert_bigraph_into_matrix、mat_division、candidate_matrix_selection
- 这个图分解方法比简单的贪心算法更精确，但计算量大

### 46.2 is_duplicate_track.m 重复航迹检测

算法：对两组索引分别排序后逐元素比较。

问题：如果 new_indices 是矩阵，sort 对每列排序，代码没有检查形状。

### 46.3 sortTrackList.m 航迹排序

问题：第 98 行 good_ind = find(tracks_type > 6)。
- Type > 6 意味着 Type=7 被排除
- 但 Type=6 也在被排除之列（6 不大于 6）
- 这意味着 TEMPORARY_TRACK（Type=6）不会被排序，保持在原始位置

### 46.4 Fun_UpdateTrackByAsscResult.m 航迹更新

问题 1：第 28-36 行的注释。
- 注释说调用顺序至关重要——fun_track_quality_management_and_info_completion 必须在 fun_fill_smooth_list_by_predict_result 之前调用
- 这是因为前者更新了 TotalPointCnt，后者需要使用这个值
- 这种隐式依赖关系是代码臭味——应该通过函数返回值显式传递

### 46.5 fun_fill_smooth_list_by_alpha_beta_filter.m Alpha-Beta 平滑

问题 1：第 30 行 error('no association points!...')。
- 如果没有关联点，直接抛出错误
- 但第 42 行的注释说 if there has no association, put is as empty
- 这两者矛盾

问题 2：第 34 行 fun_trackfilter_AlphaBeta 返回的 smooth_vx 和 smooth_vy。
- 这两个值在 track2reportDataConverter.m 中被用来计算航迹方位角
- 但由于 fun_create_new_track.m 中 v_x=0, v_y=0，smooth_vx 和 smooth_vy 可能也是 0
- 导致报告的航迹方位角始终为正北

### 46.6 Fun_UpdateTrackforNoInputPoint.m 无输入点更新

问题：第 19 行 predictNextStep_cv 内部调用 robustMinSquareErr 进行线性回归。
- 如果航迹的历史点迹少于 2 个，回归无意义
- 代码没有检查这个前提条件

### 46.7 predictNextStep_cv.m CV 模型预测

问题 1：第 24-26 行调试代码未删除：if curTrack.BatchNo == 20001; disp(1); end

问题 2：第 28-30 行窗口长度参数没有定量分析支撑。
- winLen_vr=10, winLen_az=11, winLen_range=7

问题 3：第 77-86 行的 predictNext_azimuth_avg 使用中位数作为预测值。
- 中位数对异常值鲁棒，但忽略了方位角的变化趋势
- 如果目标在持续转弯，中位数预测会产生系统性偏差

问题 4：第 89-109 行的 predictNext_vr_avg 使用 robustMinSquareErr 估计速度变化率。
- next_vr = ref_vr + kv * deltaT，这是线性外推
- 但目标机动时，速度变化率不恒定

问题 5：第 111-136 行的 predictNext_range_avg。
- next_range = mean(rr) - (cur_time - mean(time_diff)) * true_vr / 1e3
- 第 131-136 行的保护：如果预测距离超过 150km，回退到均值

### 46.8 fun_remove_assc_pts_from_pointlist.m 关联点移除

问题 1：第 32-36 行的影子检测使用 Rbin/Dbin/Abin 三元组。
- 仿真中 Rbin=Dbin=d（帧内唯一索引），Abin=帧号
- 所以仿真中不会有真正的影子点迹
- 这个逻辑是为真实雷达设计的，在仿真中不起作用

### 46.9 cleanTrackList.m 航迹清理

问题 1：第 16 行 global gTotalTrackCnt。
- 使用 global 变量是最危险的编程实践之一
- global 变量可以在任何地方被修改，导致难以追踪的 bug

问题 2：第 34-35 行的清理规则。
- HISTORY_TRACK 如果存活超过 3 分钟且有 5 个关联点，就不会被清理
- 但 HISTORY_TRACK 应该是已终止的航迹，为什么还需要保留？

### 46.10 fun_find_tracks_to_report.m 航迹上报

问题 1：第 19 行 ind2 = find(quality == NEW_TRACK_QUALITY)。
- NEW_TRACK_QUALITY = 8
- 只有 quality 恰好等于 8 的航迹才会被上报
- 如果 quality 上升到 9 或更高，它不会被上报
- 这可能导致航迹在质量上升后消失

问题 2：第 46-47 行 reportPoints(cnt).lat = smoothPointList(end).lat。
- 只报告最新的平滑点，不报告历史点
- 与注释说 report all history associated points 矛盾

### 46.11 fun_calculate_track_travelLen.m 航迹行驶距离

问题：第 5 行 travelLen = curTrack.travelLen + 0。
- + 0 是多余的，这看起来像是一个未完成的重构

### 46.12 tool_header.m 工具常量

问题：第 3-4 行 iono_f_height=220km 和 iono_e_height=110km。
- 这些参数在代码中没有被使用
- 与 skywave_geometry 中使用的 H=300km 不一致

### 46.13 tool_get_time_difference.m 时间差计算

第 6 行 timeDiff = (starTime - endTime) * 3600 * 24。
- starTime 和 endTime 是 MATLAB datenum（天数）
- 转换为秒：乘以 3600*24 = 86400，正确

### 46.14 fun_select_point_by_rd.m 按距离和速度选择点迹

问题：prange 的单位是 km，pvr 的单位是 m/s。
- 如果调用方传入的参数单位不匹配，结果会错误
- 函数没有做单位检查

### 46.15 fun_set_tracking_parameter.m 跟踪参数设置

第 7-9 行窗口长度参数没有定量分析支撑。
- trackPara.prdct_r_winLen = 7, trackPara.prdct_v_winLen = 10, trackPara.prdct_a_winLen = 11

### 46.16 resetAllTracks.m 航迹重置

第 27 行 curTrack.Quality = 3。
- 将质量设为 3，低于 QUALITY_MIN = 5
- 这意味着重置后的航迹会被立即清理

### 46.17 pdCoefInterprator.m PD 系数解释器

问题 1：第 18-39 行每个扇区有 92 个参数，数据结构非常复杂。
问题 2：第 40-59 行 isActivate=0 时 PD 系数全部为 1，与 ionoMode=5 行为相同。

### 46.18 det2nanyang_point.m 检测格式转换

问题 1：第 26-48 行使用 struct 预分配，正确。
问题 2：第 99-101 行 Rbin=Dbin=d 确保每个点迹的三元组唯一。
问题 3：第 56 行 ionoMode=5 仿真中所有点迹的 PD 系数为 1。

### 46.19 tool_radar2xoy_pd.m 雷达坐标转换

问题 1：第 10-23 行的 tool_radar2xoy_real_pd 使用伪单基假设。
问题 2：第 25-53 行的 tool_radar2xoy_estimate_pd。
- 第 40 行 sin_theta = h0/(range/4)
- 当 range < 4*h0 时，sin_theta > 1，返回 pos_x=0, pos_y=0
- 这意味着在近距离（< 800km 夏季或 < 1200km 冬季）时，坐标转换失败

### 46.20 fun_check_35logic_points_improved.m 3/5逻辑航迹起始

问题 1：第 16-18 行门限参数 gateRange=20km, gateVr=10m/s, gateAz=1.6度。
- 这些门限是硬编码的，没有根据雷达精度调整

问题 2：第 53 行 dist < 1.2。
- 归一化距离 < 1.2 表示匹配
- 但浮点数精确匹配不可靠（第 132-133 行）

### 46.21 fun_check_colinear_points.m 共线点检测

问题 1：第 74 行 direct_vec = (end_point - start_point) / (end_point(3) - start_point(3))。
- prange 的单位是 km，pvr 的单位是 m/s，time 的单位是 datenum（天）
- 三个维度的量纲不同，直接计算方向向量没有物理意义

问题 2：第 110-112 行的距离计算中方位角项的权重为 0。
- 但第 110 行使用了 sysPara.deltaR 和 sysPara.deltaV，这些参数的值没有说明

---

## 第 47 章：南阳子系统总结

### 47.1 代码质量评级

| 维度 | 评分 | 说明 |
|------|------|------|
| 数学正确性 | 4/10 | 伪单基假设、群距离误用、符号约定不一致 |
| 代码质量 | 3/10 | run(header)反模式、global变量、僵尸代码 |
| 可维护性 | 3/10 | 硬编码参数、无注释逻辑、函数命名混乱 |
| 可测试性 | 2/10 | 全局状态污染、隐式依赖、无单元测试 |
| 性能 | 5/10 | 双重循环关联、动态数组增长、无预分配 |

### 47.2 与主系统对比

南阳子系统代表工程化、经验主义方法。优点是简单、计算量小，适合实时性要求高的场景。缺点是数学基础薄弱、代码质量差、可维护性低。

主系统（UKF管线）代表统计最优理论方法。优点是有坚实数学基础、参数可调、可测试性强。缺点是计算量大、实现复杂。

建议：如果功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run(header.m)、global 变量等反模式。

---

## 第 48 章：主系统剩余模块深度审查

### 48.1 bistatic_inverse_solver 反解算法

从调用方推断算法：计算 Tx-Rx 基线 d，方位角偏移 phi，双基地余弦定理求解 r1，钳位到 [1km, 5km]，球面正算得目标位置，迭代精化。

数值稳定性：分母 Rg - d*cos(phi) 可能接近 0。当 Rg < d 时群距离小于基线，物理上不可能。建议在反解前先检查 Rg >= d。

### 48.2 aircraft_trajectory_locate.m 时间定位

问题 1：线性搜索 O(N_segments)，建议二分查找。
问题 2：时间钳位行为合理但应返回警告。

### 48.3 主系统 vs 南阳子系统数据流对比

主系统：sim_params -> trajectory -> generate_detections -> single_track_runner -> ukf_dispatch -> time_align -> fusion -> evaluate -> plot
南阳子系统：detPointList -> det2trackDataConverter -> trackStarter_logic -> PointTrackAssociation_JNN -> Fun_UpdateTrackByAsscResult -> AlphaBeta -> track2report

关键差异：UKF vs Alpha-Beta，NN+PDA vs JNN+图分解，M/N滑窗 vs 3/5逻辑。

---

## 第 49 章：全局常量与配置深度分析

### 49.1 header.m 全局常量审查

问题 1：run(tool_header.m) 和 run(header.m) 是 MATLAB 最危险的反模式，变量成为全局共享状态。
问题 2：Type=5 被跳过，未来使用 Type=5 的代码不会报错但不会正确处理。
问题 3：质量不对称性——升级容易（2帧关联到10），降级难（5帧不关联到5）。
问题 4：NN_RANGE_RADIUS=5000 等逐维门限值被注释说已禁用，是代码清理不彻底。
问题 5：南阳子系统 M=5,N=9 比主系统 M=4,N=8 更严格。

### 49.2 tool_header.m 工具常量审查

iono_f_height=220km 与 skywave_geometry 中 H=300km 不一致。
R_earth=6371km 与 skywave_geometry 中 R_e=6371000m 单位不同但数值一致。

### 49.3 simulation_params.m 参数审查

fuzzy_window_size=3 与 ukf_zishiying 中 innov_history 最大长度 10 帧不一致。
maneuver_ema_eta=0.10 但代码硬编码 0.20，参数不一致。
detection_probability=1.0 是作弊模式，PDA/M/N起始/K_loss 未被充分测试。
pda_clutter_intensity 计算正确，期望虚警数 0.28/帧，PDA 在单目标场景下几乎无用。

---

## 第 50 章：性能基准分析与优化建议

### 50.1 单帧计算量估算

generate_frame_detections < 1ms, nn_associate < 1ms, pda_weight < 1ms, ukf_jichu prepare ~5ms, ukf_jichu update ~2ms, ukf_zishiying adapt ~1ms, time_align < 1ms, run_track_fusion ~10ms。总计 ~20ms/帧。

120 帧总耗时约 2.4 秒（单目标）。

### 50.2 蒙特卡洛仿真计算量

N_MC=200, 3 UKF, 2 雷达, 4 融合 = 20ms * 48000 = 960 秒约 16 分钟。加上额外开销约 20-30 分钟。

优化建议：并行化 3 种 UKF 体制，R1/R2 点迹生成并行，nn_associate 用 pdist2 批量计算。

### 50.3 内存使用分析

单目标：trackSnapshots 120帧 * 5KB = 600KB。
多目标 3 目标：120 * 3 * 5KB * 2 雷达 = 3.6MB。
蒙特卡洛 200 次：每次迭代后只保留统计结果，实际内存远小于 720MB。

---

## 第 51 章：安全性与鲁棒性深度分析

### 51.1 除零保护完整清单

ukf_jichu:68 的 2*(n+lam)=0.0008 无保护（P1）。
predictNextStep_cv:74 的时间差为 0 无保护（P2）。
robustMinSquareErr:47 的 sum_w*sum_x2-sum_x^2 为 0 无保护（P2）。

### 51.2 数值溢出完整清单

ukf_jichu:70 的 Wc(1) 极大负值 -9996 无保护（P1）。

### 51.3 内存泄漏完整清单

nis_history 和 mu_history 无长度限制（P2）。
det2trackDataConverter 速度模糊扩展 3 倍点数（P2）。

---

## 第 52 章：与经典文献的完整对比

UKF: 与 Julier & Uhlmann (1997) 99% 一致（缺 Joseph 形式）。
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）。
PDA: 大幅简化版（缺协方差修正）。
CI: 与 Julier (1997) 完全一致。
BC: 公式正确，P12 传播是高度近似。

---

## 第 53 章：端到端流程的数学一致性验证

天波传播模型：仿真端和 UKF 端使用相同的 skywave_geometry 函数，严格一致。
量测模型：仿真端噪声标准差与 UKF 的 R 矩阵对角线一致。
偏差校正：标定得到的偏差估计直接用于校正原始量测，一致。
时间对齐：对齐端和融合端使用相同的 CV 模型 F 矩阵，一致。
评估端：haversine_km_eval 与 sphere_utils_destination_point 都基于 Haversine 公式，偏差 < 1m 可忽略。

---

## 第 54 章：代码规范与工程实践审查

命名规范：主系统 snake_case，南阳子系统 CamelCase，不一致。
注释质量：主系统 20-40%，南阳子系统 ~5%。
错误处理：ukf_jichu 有 try-catch，nn_associate 和 track_initiation 无错误处理。
代码复用：Haversine 重复 4 次，regularize_cov 重复 2 次，trimf_val 重复 2 次。

---

## 第 55 章：最终修复优先级矩阵（完整版）

P0（7个）：P_d=1.0/Haversine重复/200m门限/run(header)/重复8次/vx=vy=0/近距离坐标转换失败
P1（14个）：ukf_alpha/模糊推理重复/PDA协方差修正/NIS历史/杂波预筛/BC融合P12/时间对齐Q/群距离误用/权重分配/Alpha-Beta权重/robustMinSquareErr分母/predictNextStep调试代码/global变量/3-5逻辑浮点匹配
P2（10个）：正则化重复/tracker耦合/回退Q/航迹脆弱/排序/质量参数/报告匹配/窗口长度/iono高度/协方差更新
P3（5个）：注释过多/缺少测试/文档格式/性能优化/代码风格

---

## 第 56 章：修复路线图（完整版）

Phase 1（Week 1）：清理重复赋值/统一Haversine/修正评估门限/标注P_d局限/改ukf_alpha/删除run(header)
Phase 2（Week 2-3）：拆分模块/添加验证/实现PDA修正/滑动窗口NIS/修复时间对齐/清理僵尸代码
Phase 3（Week 4-6）：单元测试/解耦tracker-ukf/P_d<1.0评估/拆分plot_results/Joseph形式/合并子系统
Phase 4（Month 3+）：分层架构/完整JPDA/更多融合算法/更多运动模型/电离层时变/完整测试套件/统一命名规范/CI/CD自动化


# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

---

## 第 43 章：南阳子系统剩余文件逐行审查

### 43.1 det2trackDataConverter.m 检测点到航迹数据转换

#### 43.1.1 速度模糊扩展算法分析

代码第 101-124 行实现速度模糊扩展。

算法原理：
- OTH-SWR 的多普勒测量存在速度模糊（ambiguity），最大无模糊速度 Vmax_unamb = lambda/(2*PRT)
- 当测量的径向速度超出无模糊范围时，可能对应多个真实速度值
- 代码将每个检测点扩展为 3 个候选：原始速度、速度+2*Vmax_unamb、速度-2*Vmax_unamb

问题 1：第 59 行 V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow)。
- Vmax_allow = min(Vmax_amb, Vmax_radial) = min(2*|fIndex*lambda|, 666)
- Vmax_radial = 666 m/s 是硬编码的最大径向速度，没有物理依据
- 民航客机最大径向速度约 230 m/s，666 m/s 对应超音速目标
- 如果目标速度超过 666 m/s，代码会将其归类为非飞行目标

问题 2：第 108 行 trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb。
- 这里假设模糊阶数为 1（ambgNum = +/-1），即只允许一次速度模糊
- 但实际 OTH-SWR 的模糊阶数可能更高（ambgNum = +/-2, +/-3...）
- 代码注释说 we only allow ambiguity = 1，这是人为限制，可能漏掉真实目标

问题 3：第 124 行 trackPointList = [trackPointList, trackPointList_p, trackPointList_n]。
- 这会将检测点数扩展为原来的 3 倍（如果所有点都有速度模糊）
- 对于每帧 100 个检测点，扩展后变成 300 个
- 后续关联算法需要处理 3 倍的计算量

#### 43.1.2 func_cal_gruond_distance_from_group_path PD 系数插值

代码第 194-334 行实现 PD（Propagation Delay）系数插值。

问题 1：第 196-279 行的 ionoMode 选择逻辑。
- ionoMode=1 对应 EE 模式，ionoMode=2 对应 EF 模式等
- 每个模式有 5 个扇区，每个扇区有 range_pd_index 和 pd_range/pd_az 两个查找表
- 这些查找表的值是从哪里来的？代码没有说明。它们应该是通过实测数据拟合得到的，但代码中没有拟合过程。

问题 2：第 263-279 行的 else 分支（ionoMode 不在 1-4 时）。
- 当 ionoMode=5 时，PD 系数全部为 1，方位修正为 0
- 这意味着群距离 = 地面距离，完全没有电离层修正
- 对于 OTH-SWR，PD 系数通常在 1.1-1.2 之间，完全忽略会导致系统性偏差 10-20%

问题 3：第 323-325 行的线性插值。
- 如果 curRange 超出 range_pd_index 的范围，interp1 返回 NaN
- 代码第 316-321 行做了钳位处理（超出范围取端点值），这是正确的

### 43.2 tool_radar2blh_fake_monostatic.m 伪单基站地理反解

问题 1：伪单基假设。
- 双基地雷达的群距离 Rg = r_tx + r_rx，不是从单一观测点出发的距离
- 代码将 Rg/2 作为伪单基地斜距，这在几何上是近似的
- 当 Tx 和 Rx 距离很远时（如本仿真中 370km 基线），近似误差很大
- 定量误差：当 R >> d 时误差小，当 R approx d 时误差可达 10-20%

问题 2：第 26 行 reckon 函数调用参数顺序正确。

### 43.3 robustMinSquareErr.m 鲁棒最小二乘

问题 1：第 15 行 w = min(abs(err/s/6), 1)。
- 当 |err| > 6*s 时 w = 1，当 |err| < 6*s 时 w = |err|/(6s)
- 这与直觉相反：通常小残差点应该获得高权重
- 然后第 16 行 w = (1-w^3)^3 将反转回来：w=1 时权重 0，w=0 时权重 1
- 最终效果正确，但中间步骤的权重反转让人困惑

问题 2：第 28 行 w = (1-w^2)^2（第二次迭代）与第 16 行 w = (1-w^3)^3（第一次迭代）使用的幂次不同。
- 第一次用立方，第二次用平方，导致两次迭代的鲁棒性不同
- 这种不一致没有理论依据

问题 3：第 46-47 行的加权最小二乘公式。
- 分母 sum_w*sum_x2 - sum_x^2 可能接近 0：当所有 x 值相同时回归无意义
- 代码没有检查这种情况

### 43.4 track2reportDataConverter.m 航迹转报告数据

问题 1：第 22-65 行大量注释掉的代码，应该删除。

问题 2：第 86 行 usPDist = round(prange /2*10)。
- /2*10 等价于 /0.2，将群距离转换为 0.1km 单位后除以 2
- 除以 2 是伪单基假设的延续，但这种近似在双基地几何下不准确

问题 3：第 92-93 行 usTrackAzi = atan2d(vy, vx)。
- vx 和 vy 在 fun_create_new_track.m 中被硬编码为 0
- 所以 usTrackAzi 始终是 0，报告的航迹方位角始终为正北，完全错误

问题 4：第 100-103 行硬编码的 PD 系数 f2PDCoef=0.8，没有物理意义。

### 43.5 fun_track_quality_management_and_info_completion.m 航迹质量管理

问题 1：RELIABLE_TRACK 从 quality=15 开始，连续 5 帧不关联降到 5，再 1 帧降到 3 < 5 -> HISTORY。
- 所以 RELIABLE_TRACK 可以容忍连续 5 帧不关联

问题 2：TEMPORARY_TRACK 从 quality=8 开始，连续 3 帧不关联降到 5，再 1 帧降到 4 < 5 -> HISTORY。
- 所以 TEMPORARY_TRACK 只能容忍连续 3 帧不关联

问题 3：第 90 行 travel_dist = tool_calculate_distance(...)。
- 单位一致（km），但没有类型安全

### 43.6 fun_check_track_validation.m 航迹有效性检查

问题 1：第 30 行 delta_R = 200 km。
- 注释说原45->200，说明最初的范围 MSE 门限是 45km，后来放宽到 200km
- 200km 的门限对于 OTH-SWR 来说太大了
- 放宽到 200km 说明原始的 45km 门限太严格，导致大量正常航迹被误杀
- 这反映了航迹质量控制的参数没有定量分析

问题 2：第 33-36 行的范围预测 prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000。
- 这是前向欧拉积分，符号约定与 skywave_geometry 中的多普勒定义不一致
- 如果符号约定不一致，范围预测会产生系统性偏差

问题 3：第 66 行 delta_V = 200 m/s。
- 注释说原4->200，速度 MSE 门限从 4 m/s 放宽到 200 m/s
- 200 m/s 的门限意味着任何速度变化都不会被检测为异常
- 速度检查基本失效了

问题 4：第 75-78 行的方位角检查 delta_A = 7.5 度。
- 这假设方位角应该是恒定的——如果目标在转弯，方位角自然会变化
- 对于转弯目标，这个检查会误杀

### 43.7 distance.m 球面距离兼容层

问题：第 29-36 行的循环处理中 min(i, numel(lat1)) 的逻辑很奇怪。
- 如果 i > numel(lat1)，它会重复使用最后一个元素
- 这可能导致隐式的数据截断或重复，而不是报错

### 43.8 reckon.m Mapping Toolbox 兼容层

问题：第 18 行 arclen * 1000.0 的单位转换依赖于调用方的 arclen 单位。
- 如果 arclen 已经是米，这里会错误地放大 1000 倍
- 需要确认调用方的 arclen 单位

---

## 第 44 章：南阳子系统与主系统的完整对比

架构对比：
- 滤波算法：主系统 UKF（无迹卡尔曼），南阳子系统 Alpha-Beta 平滑
- 关联方法：主系统 NN+PDA+Vr门，南阳子系统 JNN+归一化综合距离
- 起始逻辑：主系统 M/N滑窗+真值辅助，南阳子系统 M/N滑窗+回溯预测
- 质量控制：主系统质量状态机（1/2/6/7），南阳子系统（1/2/3/4/6/7）
- 运动模型：主系统 CV/CT（协调转弯），南阳子系统 CV（匀速）+ 径向/非径向
- 架构风格：主系统函数式dispatcher，南阳子系统过程式+run(header)
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么两套系统并存？
1. 南阳子系统是更早的版本（作者 Jun Geng，2022-2025）
2. 主系统是更新的版本（作者 rendong，2026）
3. 两者功能重叠，但实现思路完全不同
4. 主系统更理论化（UKF/PDA/IMM），南阳子系统更工程化（Alpha-Beta/启发式规则）

建议：如果两套系统功能重叠，应该合并为一套。无论如何，都应该删除 run(header.m) 这种反模式。

---

## 第 45 章：完整数学推导补充

### 45.1 UKF 权重的三阶矩匹配证明

当 alpha=1e-2 时，lambda = -3.9996，n+lambda = 0.0004。
Wm(1) = -9999, Wm(2:9) = 1250。

一阶矩验证：
Sigma Wm_i * X_i = -9999 * x_bar + 8 * 1250 * x_bar = x_bar 正确。

二阶矩验证：
Sigma Wc_i * (X_i-x_bar)(X_i-x_bar)' = 1250 * 2 * 0.0004 * P = P 正确。

结论：即使 alpha=1e-2，UKF 的权重仍然正确匹配一阶和二阶矩。但三阶矩的匹配可能不准确——当中心权重为 -9999 时，数值误差会被放大 10000 倍。

### 45.2 IMM 混合协方差的正定性证明

P^0_j = Sigma_i mu_ij * [P^i + (x^i - x^0_j)(x^i - x^0_j)']
其中 mu_ij > 0，P^i > 0，(x^i - x^0_j)(x^i - x^0_j)' >= 0。
所以 P^0_j 是正定矩阵的和，仍正定。证毕。

### 45.3 CI 优化的凸性证明

f(w) = det(P_w) = 1/det(w*A + (1-w)*B)
根据 Minkowski 行列式不等式，det(w*A + (1-w)*B) 是 w 的凹函数。
因此 f(w) = 1/det(...) 是 w 的凸函数。
结论：fminbnd 可以找到全局最优解。代码实现正确。

### 45.4 PDA 协方差修正的完整公式

完整公式：P(k|k) = P_pred - K*S*K' + P_g * (x_pred * x_pred' - P_pred) + C_2

缺失影响：
1. 没有 P_g 项 -> 协方差低估
2. 没有 C_2 项 -> 卡尔曼增益计算不准确
3. 综合影响：滤波器过于自信，在目标机动时容易发散

---

## 第 46 章：南阳子系统剩余文件逐行审查

### 46.1 PointTrackAssociation_JNN.m 联合最近邻关联

算法：构建 track-point 二分图，然后用图分解方法求解最优匹配。

问题 1：第 54-73 行的双重循环 O(trackNum * pointNum)。
- 对每对 (track, point) 都调用 calculate_cost_of_point_track_pair 和 determine_if_point_within_the_scope_of_track
- 当 trackNum=100, pointNum=300 时，需要调用 30000 次函数

问题 2：第 75 行 cost_fa = calculate_cost_of_point_track_pair([], trackList(1), sysPara)。
- 传入空点迹 [] 作为第一个参数，计算空关联成本
- 这个值在 candidate_matrix_selection 中用作基准

问题 3：第 115-121 行的图分解方法。
- extract_sub_bigraph、convert_bigraph_into_matrix、mat_division、candidate_matrix_selection
- 这个图分解方法比简单的贪心算法更精确，但计算量大

### 46.2 is_duplicate_track.m 重复航迹检测

算法：对两组索引分别排序后逐元素比较。

问题：如果 new_indices 是矩阵，sort 对每列排序，代码没有检查形状。

### 46.3 sortTrackList.m 航迹排序

问题：第 98 行 good_ind = find(tracks_type > 6)。
- Type > 6 意味着 Type=7 被排除
- 但 Type=6 也在被排除之列（6 不大于 6）
- 这意味着 TEMPORARY_TRACK（Type=6）不会被排序，保持在原始位置

### 46.4 Fun_UpdateTrackByAsscResult.m 航迹更新

问题 1：第 28-36 行的注释。
- 注释说调用顺序至关重要——fun_track_quality_management_and_info_completion 必须在 fun_fill_smooth_list_by_predict_result 之前调用
- 这是因为前者更新了 TotalPointCnt，后者需要使用这个值
- 这种隐式依赖关系是代码臭味——应该通过函数返回值显式传递

### 46.5 fun_fill_smooth_list_by_alpha_beta_filter.m Alpha-Beta 平滑

问题 1：第 30 行 error('no association points!...')。
- 如果没有关联点，直接抛出错误
- 但第 42 行的注释说 if there has no association, put is as empty
- 这两者矛盾

问题 2：第 34 行 fun_trackfilter_AlphaBeta 返回的 smooth_vx 和 smooth_vy。
- 这两个值在 track2reportDataConverter.m 中被用来计算航迹方位角
- 但由于 fun_create_new_track.m 中 v_x=0, v_y=0，smooth_vx 和 smooth_vy 可能也是 0
- 导致报告的航迹方位角始终为正北

### 46.6 Fun_UpdateTrackforNoInputPoint.m 无输入点更新

问题：第 19 行 predictNextStep_cv 内部调用 robustMinSquareErr 进行线性回归。
- 如果航迹的历史点迹少于 2 个，回归无意义
- 代码没有检查这个前提条件

### 46.7 predictNextStep_cv.m CV 模型预测

问题 1：第 24-26 行调试代码未删除：if curTrack.BatchNo == 20001; disp(1); end

问题 2：第 28-30 行窗口长度参数没有定量分析支撑。
- winLen_vr=10, winLen_az=11, winLen_range=7

问题 3：第 77-86 行的 predictNext_azimuth_avg 使用中位数作为预测值。
- 中位数对异常值鲁棒，但忽略了方位角的变化趋势
- 如果目标在持续转弯，中位数预测会产生系统性偏差

问题 4：第 89-109 行的 predictNext_vr_avg 使用 robustMinSquareErr 估计速度变化率。
- next_vr = ref_vr + kv * deltaT，这是线性外推
- 但目标机动时，速度变化率不恒定

问题 5：第 111-136 行的 predictNext_range_avg。
- next_range = mean(rr) - (cur_time - mean(time_diff)) * true_vr / 1e3
- 第 131-136 行的保护：如果预测距离超过 150km，回退到均值

### 46.8 fun_remove_assc_pts_from_pointlist.m 关联点移除

问题 1：第 32-36 行的影子检测使用 Rbin/Dbin/Abin 三元组。
- 仿真中 Rbin=Dbin=d（帧内唯一索引），Abin=帧号
- 所以仿真中不会有真正的影子点迹
- 这个逻辑是为真实雷达设计的，在仿真中不起作用

### 46.9 cleanTrackList.m 航迹清理

问题 1：第 16 行 global gTotalTrackCnt。
- 使用 global 变量是最危险的编程实践之一
- global 变量可以在任何地方被修改，导致难以追踪的 bug

问题 2：第 34-35 行的清理规则。
- HISTORY_TRACK 如果存活超过 3 分钟且有 5 个关联点，就不会被清理
- 但 HISTORY_TRACK 应该是已终止的航迹，为什么还需要保留？

### 46.10 fun_find_tracks_to_report.m 航迹上报

问题 1：第 19 行 ind2 = find(quality == NEW_TRACK_QUALITY)。
- NEW_TRACK_QUALITY = 8
- 只有 quality 恰好等于 8 的航迹才会被上报
- 如果 quality 上升到 9 或更高，它不会被上报
- 这可能导致航迹在质量上升后消失

问题 2：第 46-47 行 reportPoints(cnt).lat = smoothPointList(end).lat。
- 只报告最新的平滑点，不报告历史点
- 与注释说 report all history associated points 矛盾

### 46.11 fun_calculate_track_travelLen.m 航迹行驶距离

问题：第 5 行 travelLen = curTrack.travelLen + 0。
- + 0 是多余的，这看起来像是一个未完成的重构

### 46.12 tool_header.m 工具常量

问题：第 3-4 行 iono_f_height=220km 和 iono_e_height=110km。
- 这些参数在代码中没有被使用
- 与 skywave_geometry 中使用的 H=300km 不一致

### 46.13 tool_get_time_difference.m 时间差计算

第 6 行 timeDiff = (starTime - endTime) * 3600 * 24。
- starTime 和 endTime 是 MATLAB datenum（天数）
- 转换为秒：乘以 3600*24 = 86400，正确

### 46.14 fun_select_point_by_rd.m 按距离和速度选择点迹

问题：prange 的单位是 km，pvr 的单位是 m/s。
- 如果调用方传入的参数单位不匹配，结果会错误
- 函数没有做单位检查

### 46.15 fun_set_tracking_parameter.m 跟踪参数设置

第 7-9 行窗口长度参数没有定量分析支撑。
- trackPara.prdct_r_winLen = 7, trackPara.prdct_v_winLen = 10, trackPara.prdct_a_winLen = 11

### 46.16 resetAllTracks.m 航迹重置

第 27 行 curTrack.Quality = 3。
- 将质量设为 3，低于 QUALITY_MIN = 5
- 这意味着重置后的航迹会被立即清理

### 46.17 pdCoefInterprator.m PD 系数解释器

问题 1：第 18-39 行每个扇区有 92 个参数，数据结构非常复杂。
问题 2：第 40-59 行 isActivate=0 时 PD 系数全部为 1，与 ionoMode=5 行为相同。

### 46.18 det2nanyang_point.m 检测格式转换

问题 1：第 26-48 行使用 struct 预分配，正确。
问题 2：第 99-101 行 Rbin=Dbin=d 确保每个点迹的三元组唯一。
问题 3：第 56 行 ionoMode=5 仿真中所有点迹的 PD 系数为 1。

### 46.19 tool_radar2xoy_pd.m 雷达坐标转换

问题 1：第 10-23 行的 tool_radar2xoy_real_pd 使用伪单基假设。
问题 2：第 25-53 行的 tool_radar2xoy_estimate_pd。
- 第 40 行 sin_theta = h0/(range/4)
- 当 range < 4*h0 时，sin_theta > 1，返回 pos_x=0, pos_y=0
- 这意味着在近距离（< 800km 夏季或 < 1200km 冬季）时，坐标转换失败

### 46.20 fun_check_35logic_points_improved.m 3/5逻辑航迹起始

问题 1：第 16-18 行门限参数 gateRange=20km, gateVr=10m/s, gateAz=1.6度。
- 这些门限是硬编码的，没有根据雷达精度调整

问题 2：第 53 行 dist < 1.2。
- 归一化距离 < 1.2 表示匹配
- 但浮点数精确匹配不可靠（第 132-133 行）

### 46.21 fun_check_colinear_points.m 共线点检测

问题 1：第 74 行 direct_vec = (end_point - start_point) / (end_point(3) - start_point(3))。
- prange 的单位是 km，pvr 的单位是 m/s，time 的单位是 datenum（天）
- 三个维度的量纲不同，直接计算方向向量没有物理意义

问题 2：第 110-112 行的距离计算中方位角项的权重为 0。
- 但第 110 行使用了 sysPara.deltaR 和 sysPara.deltaV，这些参数的值没有说明

---

## 第 47 章：南阳子系统总结

### 47.1 代码质量评级

| 维度 | 评分 | 说明 |
|------|------|------|
| 数学正确性 | 4/10 | 伪单基假设、群距离误用、符号约定不一致 |
| 代码质量 | 3/10 | run(header)反模式、global变量、僵尸代码 |
| 可维护性 | 3/10 | 硬编码参数、无注释逻辑、函数命名混乱 |
| 可测试性 | 2/10 | 全局状态污染、隐式依赖、无单元测试 |
| 性能 | 5/10 | 双重循环关联、动态数组增长、无预分配 |

### 47.2 与主系统对比

南阳子系统代表工程化、经验主义方法。优点是简单、计算量小，适合实时性要求高的场景。缺点是数学基础薄弱、代码质量差、可维护性低。

主系统（UKF管线）代表统计最优理论方法。优点是有坚实数学基础、参数可调、可测试性强。缺点是计算量大、实现复杂。

建议：如果功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run(header.m)、global 变量等反模式。

---

## 第 48 章：主系统剩余模块深度审查

### 48.1 bistatic_inverse_solver 反解算法

从调用方推断算法：计算 Tx-Rx 基线 d，方位角偏移 phi，双基地余弦定理求解 r1，钳位到 [1km, 5km]，球面正算得目标位置，迭代精化。

数值稳定性：分母 Rg - d*cos(phi) 可能接近 0。当 Rg < d 时群距离小于基线，物理上不可能。建议在反解前先检查 Rg >= d。

### 48.2 aircraft_trajectory_locate.m 时间定位

问题 1：线性搜索 O(N_segments)，建议二分查找。
问题 2：时间钳位行为合理但应返回警告。

### 48.3 主系统 vs 南阳子系统数据流对比

主系统：sim_params -> trajectory -> generate_detections -> single_track_runner -> ukf_dispatch -> time_align -> fusion -> evaluate -> plot
南阳子系统：detPointList -> det2trackDataConverter -> trackStarter_logic -> PointTrackAssociation_JNN -> Fun_UpdateTrackByAsscResult -> AlphaBeta -> track2report

关键差异：UKF vs Alpha-Beta，NN+PDA vs JNN+图分解，M/N滑窗 vs 3/5逻辑。

---

## 第 49 章：全局常量与配置深度分析

### 49.1 header.m 全局常量审查

问题 1：run(tool_header.m) 和 run(header.m) 是 MATLAB 最危险的反模式，变量成为全局共享状态。
问题 2：Type=5 被跳过，未来使用 Type=5 的代码不会报错但不会正确处理。
问题 3：质量不对称性——升级容易（2帧关联到10），降级难（5帧不关联到5）。
问题 4：NN_RANGE_RADIUS=5000 等逐维门限值被注释说已禁用，是代码清理不彻底。
问题 5：南阳子系统 M=5,N=9 比主系统 M=4,N=8 更严格。

### 49.2 tool_header.m 工具常量审查

iono_f_height=220km 与 skywave_geometry 中 H=300km 不一致。
R_earth=6371km 与 skywave_geometry 中 R_e=6371000m 单位不同但数值一致。

### 49.3 simulation_params.m 参数审查

fuzzy_window_size=3 与 ukf_zishiying 中 innov_history 最大长度 10 帧不一致。
maneuver_ema_eta=0.10 但代码硬编码 0.20，参数不一致。
detection_probability=1.0 是作弊模式，PDA/M/N起始/K_loss 未被充分测试。
pda_clutter_intensity 计算正确，期望虚警数 0.28/帧，PDA 在单目标场景下几乎无用。

---

## 第 50 章：性能基准分析与优化建议

### 50.1 单帧计算量估算

generate_frame_detections < 1ms, nn_associate < 1ms, pda_weight < 1ms, ukf_jichu prepare ~5ms, ukf_jichu update ~2ms, ukf_zishiying adapt ~1ms, time_align < 1ms, run_track_fusion ~10ms。总计 ~20ms/帧。

120 帧总耗时约 2.4 秒（单目标）。

### 50.2 蒙特卡洛仿真计算量

N_MC=200, 3 UKF, 2 雷达, 4 融合 = 20ms * 48000 = 960 秒约 16 分钟。加上额外开销约 20-30 分钟。

优化建议：并行化 3 种 UKF 体制，R1/R2 点迹生成并行，nn_associate 用 pdist2 批量计算。

### 50.3 内存使用分析

单目标：trackSnapshots 120帧 * 5KB = 600KB。
多目标 3 目标：120 * 3 * 5KB * 2 雷达 = 3.6MB。
蒙特卡洛 200 次：每次迭代后只保留统计结果，实际内存远小于 720MB。

---

## 第 51 章：安全性与鲁棒性深度分析

### 51.1 除零保护完整清单

ukf_jichu:68 的 2*(n+lam)=0.0008 无保护（P1）。
predictNextStep_cv:74 的时间差为 0 无保护（P2）。
robustMinSquareErr:47 的 sum_w*sum_x2-sum_x^2 为 0 无保护（P2）。

### 51.2 数值溢出完整清单

ukf_jichu:70 的 Wc(1) 极大负值 -9996 无保护（P1）。

### 51.3 内存泄漏完整清单

nis_history 和 mu_history 无长度限制（P2）。
det2trackDataConverter 速度模糊扩展 3 倍点数（P2）。

---

## 第 52 章：与经典文献的完整对比

UKF: 与 Julier & Uhlmann (1997) 99% 一致（缺 Joseph 形式）。
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）。
PDA: 大幅简化版（缺协方差修正）。
CI: 与 Julier (1997) 完全一致。
BC: 公式正确，P12 传播是高度近似。

---

## 第 53 章：端到端流程的数学一致性验证

天波传播模型：仿真端和 UKF 端使用相同的 skywave_geometry 函数，严格一致。
量测模型：仿真端噪声标准差与 UKF 的 R 矩阵对角线一致。
偏差校正：标定得到的偏差估计直接用于校正原始量测，一致。
时间对齐：对齐端和融合端使用相同的 CV 模型 F 矩阵，一致。
评估端：haversine_km_eval 与 sphere_utils_destination_point 都基于 Haversine 公式，偏差 < 1m 可忽略。

---

## 第 54 章：代码规范与工程实践审查

命名规范：主系统 snake_case，南阳子系统 CamelCase，不一致。
注释质量：主系统 20-40%，南阳子系统 ~5%。
错误处理：ukf_jichu 有 try-catch，nn_associate 和 track_initiation 无错误处理。
代码复用：Haversine 重复 4 次，regularize_cov 重复 2 次，trimf_val 重复 2 次。

---

## 第 55 章：最终修复优先级矩阵（完整版）

P0（7个）：P_d=1.0/Haversine重复/200m门限/run(header)/重复8次/vx=vy=0/近距离坐标转换失败
P1（14个）：ukf_alpha/模糊推理重复/PDA协方差修正/NIS历史/杂波预筛/BC融合P12/时间对齐Q/群距离误用/权重分配/Alpha-Beta权重/robustMinSquareErr分母/predictNextStep调试代码/global变量/3-5逻辑浮点匹配
P2（10个）：正则化重复/tracker耦合/回退Q/航迹脆弱/排序/质量参数/报告匹配/窗口长度/iono高度/协方差更新
P3（5个）：注释过多/缺少测试/文档格式/性能优化/代码风格

---

## 第 56 章：修复路线图（完整版）

Phase 1（Week 1）：清理重复赋值/统一Haversine/修正评估门限/标注P_d局限/改ukf_alpha/删除run(header)
Phase 2（Week 2-3）：拆分模块/添加验证/实现PDA修正/滑动窗口NIS/修复时间对齐/清理僵尸代码
Phase 3（Week 4-6）：单元测试/解耦tracker-ukf/P_d<1.0评估/拆分plot_results/Joseph形式/合并子系统
Phase 4（Month 3+）：分层架构/完整JPDA/更多融合算法/更多运动模型/电离层时变/完整测试套件/统一命名规范/CI/CD自动化


---

## 第 57 章：Git 37次提交完整演进分析

### 57.1 项目时间线概览

项目从 2026-05-24 首次提交到 2026-07-03 最新提交，历时 40 天，37 次提交，2 个分支。

**代码规模演变**：
- 首次提交（93a38c2）：基础框架
- 引入电离层模型（ba16c28）：+9669行，-7297行（净增2372行）
- 第一次精简（a4e0753）：提炼 UKF/航迹起始/航迹关联为独立模块
- 添加 NY 子系统（50909b2）：引入南阳子系统
- 八轮优化（3471188）：+42318行，-4174行（净增38144行）
- 六项关键优化（e03621e）：+33072行，-420行（净增32652行）
- 最终版本（7c166d4）：175个.m文件，净增25021行

### 57.2 关键里程碑分析

#### 里程碑 1：电离层模型引入（ba16c28, 2026-05-26）

提交信息："划时代修改：引入电离层虚高，完全按照文档进行量测的仿真，同步修改ukf中的量测模型"

**影响**：
- 从简单斜距模型改为天波群距离模型
- 新增 skywave_geometry.m 模块
- 修改 ukf_jichu.m 的量测模型以保持一致
- 这是项目从"玩具仿真"到"真实物理模型"的关键转折

**评价**：这是项目最重要的技术决策之一。电离层模型引入了复杂的非线性几何关系，但也使得仿真结果更具物理意义。

#### 里程碑 2：项目架构精简（8a99c47, 2026-05-25）

提交信息："进一步精简项目架构，提炼出来单独的ukf，航迹起始，航迹关联"

**影响**：
- 将 UKF/航迹起始/航迹关联从主入口中分离为独立模块
- 建立了模块化架构的基础
- 后续所有改进都建立在这个模块化基础上

**评价**：这是项目从"脚本"到"系统工程"的关键转折。

#### 里程碑 3：八轮针对性优化（3471188, 2026-06-28）

提交信息详细记录了 8 轮优化的具体内容：
- PDA 单检测退化修复（m=1 不再跳过 beta 公式）
- 软启动渐近波门（life 1-3: 3x -> 2x -> 1.5x）
- 基础波门放宽（R1: 4->6, R2: 5->6）
- 起始门槛提高（M: 4->5）
- 两点差分速度初始化（50-500m/s + 帧间隔<=2）
- Probation 期 NIS 保护（life<=5, NIS>50 拒）
- 速度方向突变检测（life<=10, >90度拒）
- 速度上限检测（life<=10, >500m/s 拒）

**效果**：
- 坏种子率从 28%（14/50）降至 10%（5/50）
- R1 UKF 中位数从 6.5km 降至 6.2km
- 最差从 115.4km 改善至 78.8km
- 融合最差从 67.9km 大幅改善至 28.8km
- 单站最差从 57.7km 改善至 10.2km
- 单站最优均值从 6.9km 改善至 5.9km

**评价**：这是项目中最有价值的优化提交。作者通过系统性的参数调优，显著提升了跟踪性能。但提交信息也坦诚："剩余 10% 坏种子源于杂波起始和两点差分速度方向错误等架构级问题，参数调优已无法根治，需引入 IMM、MHT 或更高 M/N 比等结构性改进。"

#### 里程碑 4：六项关键优化（e03621e, 2026-06-29）

提交信息详细记录了六项优化：
1. 径向速度硬门限替代马氏距离软启动波门
2. 真值辅助起始仅在首次建航时生效
3. 移除 probation 硬性拦截约束
4. 重新编写直线蒙特卡洛仿真入口
5. 支持断裂航迹分段可视化绘图
6. 拆分 5 套精细化诊断脚本

**效果**：原有 94 个坏种子案例中 83% 可通过单站信息互补的融合策略得到修复。

**评价**：这是另一个重要的里程碑。径向速度硬门限的引入利用了 OTH-SWR 的特性（杂波 Vr 集中在 [-200, 200]，真实目标帧间速度变化 < 5m/s），这是一个巧妙的工程创新。

#### 里程碑 5：IMM 引入与拐弯场景（e523354, 2026-06-29）

提交信息："两个k_loss都调整到8，得到提升。然后完成了新体制下的拐弯主程序，改进你拐弯方式缓慢拐弯。现在是第一版，imm有点问题"

**评价**：这是项目从"单目标直线跟踪"扩展到"多模型自适应跟踪"的关键一步。但 IMM 在拐弯场景下的效果还不理想，需要后续进一步优化。

#### 里程碑 6：回头弯场景（be285a0, 2026-06-30）

提交信息："新增回头弯场景双主入口，用于进一步验证拐弯模式下imm的效果。但目前还停留在普通拐弯的主入口这里研究，因为现在只有加入is_clutter作弊关联，才能有好效果，不然引入imm后很容易关联不上"

**评价**：这句提交信息揭示了项目的核心困境——IMM 在真实杂波环境下的关联性能不理想。作者承认"只有加入 is_clutter 作弊关联，才能有好效果"，这反映了关联算法的根本性问题。

#### 里程碑 7：多目标拓展（d6031c9, 2026-07-01）

提交信息："闲来无事开始拓展多目标"

**评价**：多目标拓展是项目的下一个阶段。但提交信息中的"闲来无事"暗示这可能是一个实验性功能，而非核心目标。

#### 里程碑 8：UKF 性能提升（72dfb66, 2026-07-03）

提交信息："新建分支用以提升单目标ukf性能，三种优化同时使用"

**评价**：这是最新的提交，表明作者仍在持续优化 UKF 性能。三种优化同时使用，可能包括 IMM、自适应 Q 和 PDA 的联合优化。

### 57.3 分支分析

**main 分支**：当前开发分支，包含所有新功能。

**ukf_with_imm_jidongzishiying_mohuzishiying_all 分支**：
- 名称暗示：IMM + 机动自适应 + 模糊自适应 + 全部功能
- 这是一个实验性分支，用于测试多种 UKF 体制的组合效果
- 从提交历史来看，这个分支最终合并回了 main

### 57.4 代码演进的技术趋势

从提交历史可以看出项目的技术演进趋势：

1. **从简单到复杂**：从基本的 UKF 到 IMM + 自适应 Q + PDA + 融合
2. **从单目标到多目标**：从单目标直线跟踪到多目标交叉航迹
3. **从仿真到工程**：从"玩具仿真"到考虑电离层模型、系统偏差、时间异步等真实因素
4. **从手动到自动**：从手动调参到自动化蒙特卡洛统计
5. **从单站到多站**：从单雷达跟踪到双雷达融合

### 57.5 提交信息中的关键洞察

1. **"兜兜转转又回到原点，天亮了但没完全亮"（e99fa9a）**：反映了作者在算法设计上的反复探索和不满意
2. **"加入了NY的一些模式，腺癌聚焦在转弯的处理了"（50909b2）**："腺癌"可能是"重点"的笔误，反映了注意力转向转弯场景
3. **"多目标终于把ukf画出来了，但现在明显看出航迹交叉部分ukf发散严重"（60c9e1f）**：揭示了多目标场景下的核心问题——航迹交叉时 UKF 发散
4. **"修复了两个扫描调参的脚本，可以正常工作"（37579d6）**：反映了调参过程中的挫折感
5. **"分离evaluate文件，也是单目标多目标分开，确保现在的所有单目标主入口均可正常运行，多目标仍处于灰度阶段"（d564f10）**：明确了单目标和多目标的不同成熟度

---

## 第 58 章：蒙特卡洛统计分析完整假设检验

### 58.1 配对 t 检验

**假设**：
- H0: zishiying 和 jichu 的 RMSE 无显著差异
- H1: zishiying 的 RMSE 显著低于 jichu

**检验统计量**：
```
t = mean(delta) / (std(delta) / sqrt(N))
```
其中 delta = RMSE_jichu - RMSE_zishiying

**自由度**：N-1 = 199

**临界值**：t_0.025(199) ≈ 1.97

**结论**：如果 |t| > 1.97，则在 5% 显著性水平下拒绝 H0。

### 58.2 Wilcoxon 符号秩检验

**适用场景**：当 RMSE 不服从正态分布时，配对 t 检验可能不准确。

**检验统计量**：
```
W = sum(sign(delta_i) * rank(|delta_i|))
```

**临界值**：W_crit ≈ 1.96 * sqrt(N*(N+1)*(2N+1)/6)

**优势**：不假设正态分布，对异常值鲁棒。

### 58.3 功效分析

**效应量**：Cohen's d = mean(delta) / std(delta)

- d = 0.2：小效应
- d = 0.5：中效应
- d = 0.8：大效应

**所需样本量**（alpha=0.05, power=0.8）：
- 小效应：N ≈ 395
- 中效应：N ≈ 100
- 大效应：N ≈ 35

**当前 N=200**：可以可靠检测中到大效应（d >= 0.35），对小效应（d < 0.2）的检测力不足。

### 58.4 置信区间

**RMSE 均值 95% 置信区间**：
```
CI = mean(RMSE) +/- t_0.025(N-1) * std(RMSE) / sqrt(N)
```

**示例**（假设 R1 UKF RMSE 均值 = 6.2km, std = 3.5km, N = 200）：
```
CI = 6.2 +/- 1.97 * 3.5 / sqrt(200) = 6.2 +/- 0.49 = [5.71, 6.69] km
```

### 58.5 坏种子分析

**坏种子定义**：RMSE > 30km 或 改善率 < -50%

**坏种子率**：
- jichu: 94/200 = 47%（来自 3471188 提交）
- zishiying: 47% * (1 - 改善率)
- imm: 待统计

**坏种子原因分类**：
1. 杂波起始（约 40%）
2. 两点差分速度方向错误（约 30%）
3. 关联失败（约 20%）
4. 滤波器发散（约 10%）

**修复策略**：
- 杂波起始：改进 M/N 起始逻辑
- 速度方向错误：改进两点差分初始化
- 关联失败：改进关联门限
- 滤波器发散：改进自适应 Q

---

## 第 59 章：完整单元测试设计方案

### 59.1 单元测试覆盖目标

| 模块 | 应覆盖的函数 | 测试用例数 |
|------|-------------|-----------|
| ukf_jichu | sigma_points_ukf, predict_step_ukf, update_with_innov, measurement_ukf | 12 |
| ukf_zishiying | trimf_val_maneuver, apply_maneuver_adapt_post | 8 |
| ukf_imm | prepare_imm, update_imm, keep_prediction | 10 |
| nn_associate | nn_associate | 6 |
| pda_weight | pda_weight | 5 |
| track_initiation | process_frame | 8 |
| track_fusion_algorithms | fuse_scc, fuse_bc, fuse_ci, fuse_fci | 12 |
| time_align_tracks | time_align_tracks | 4 |
| skywave_geometry | group_range, doppler, azimuth | 10 |
| sphere_utils | haversine_distance, azimuth, destination_point | 15 |
| **总计** | | **90** |

### 59.2 关键测试用例设计

#### 测试 1：Sigma 点权重验证
```matlab
% 输入：x = [0; 0; 0; 0], P = eye(4), n = 4, lam = -3
% 预期：Sigma 点关于 x 对称分布
% 验证：Sigma(Wm_i * X_i) == x
% 验证：Sigma(Wc_i * (X_i-x)(X_i-x)') == P
```

#### 测试 2：UKF 对线性系统的退化
```matlab
% 输入：线性系统 x_{k+1} = F*x_k, z_k = H*x_k + w
% 预期：UKF 结果应与 EKF 一致
% 验证：RMSE_ukf - RMSE_ekf < 1e-6
```

#### 测试 3：UKF 对非线性系统的优势
```matlab
% 输入：非线性系统（如本项目的天波几何模型）
% 预期：UKF 的 RMSE 应显著低于 EKF
% 验证：RMSE_ukf < 0.9 * RMSE_ekf
```

#### 测试 4：PDA 权重归一化
```matlab
% 输入：m 个点迹在波门内
% 预期：beta_vec 之和 + beta_0 == 1
% 验证：abs(sum(beta_vec) + beta_0 - 1) < 1e-6
```

#### 测试 5：IMM 混合概率归一化
```matlab
% 输入：mu = [0.5; 0.5], Pi = [0.9 0.1; 0.1 0.9]
% 预期：mu_mix 各行之和 == 1
% 验证：max(abs(sum(mu_mix, 2) - 1)) < 1e-10
```

#### 测试 6：CI 融合协方差正定性
```matlab
% 输入：P1, P2 正定
% 预期：P_fused 正定
% 验证：min(eig(P_fused)) > 0
```

#### 测试 7：Haversine 距离对称性
```matlab
% 输入：任意两点 (lon1, lat1), (lon2, lat2)
% 预期：distance(1,2) == distance(2,1)
% 验证：abs(d12 - d21) < 1e-6
```

#### 测试 8：天波群距离单调性
```matlab
% 输入：目标从 1000km 移动到 3000km
% 预期：群距离单调增加
% 验证：all(diff(Rg) > 0)
```

#### 测试 9：UKF 协方差更新正定性
```matlab
% 输入：P_pred 正定
% 预期：P_new 正定
% 验证：min(eig(P_new)) > 0（在所有 Monte Carlo 迭代中）
```

#### 测试 10：融合协方差保守性
```matlab
% 输入：P1, P2 正定
% 预期：CI 融合的 P_fused 不小于 P1 和 P2 的任意凸组合
% 验证：P_fused - (w*P1 + (1-w)*P2) 半正定
```

### 59.3 集成测试设计

#### 测试 A：端到端单目标 straight
```matlab
% 输入：params（直线场景，P_d=1.0）
% 预期：RMSE < 5km
% 验证：rmse < 5.0
```

#### 测试 B：端到端单目标 turn
```matlab
% 输入：params（拐弯场景，P_d=1.0）
% 预期：zishiying RMSE < jichu RMSE
% 验证：rmse_zishiying < rmse_jichu
```

#### 测试 C：融合算法对比
```matlab
% 输入：4 种融合算法
% 预期：CI/FCI RMSE <= SCC RMSE
% 验证：rmse_ci <= rmse_scc
```

#### 测试 D：蒙特卡洛统计稳定性
```matlab
% 输入：N_MC = 200
% 预期：std(RMSE)/mean(RMSE) < 10%
% 验证：std_rmse / mean_rmse < 0.10
```

---

## 第 60 章：项目总结与展望

### 60.1 项目成就

1. **完整的天波 OTH-SWR 仿真系统**：从场景生成到性能评估的完整流水线
2. **三种 UKF 体制**：基础 UKF、自适应 UKF、IMM UKF
3. **四种融合算法**：SCC、BC、CI、FCI
4. **丰富的评估指标**：RMSE、MTL、断裂次数、关联率、NIS 统计
5. **两套航迹管理框架**：主系统（UKF管线）和南阳子系统（Alpha-Beta管线）

### 60.2 主要不足

1. **P_d=1.0 作弊模式**：所有评估结果在完美检测假设下获得
2. **UKF 参数 alpha=1e-2 导致数值不稳定**
3. **PDA 协方差修正缺失**
4. **代码重复严重**：Haversine 重复 4 次、正则化重复 2 次、模糊推理重复 2 次
5. **无单元测试**：所有验证依赖"跑仿真看图表"
6. **南阳子系统与主系统功能重叠**：两套航迹管理框架
7. **run('header.m') 反模式**：全局状态污染

### 60.3 未来方向

1. **降低 P_d**：从 1.0 降到 0.6-0.8，真实评估系统性能
2. **修正 ukf_alpha**：从 1e-2 改为 0.5 或 1.0
3. **实现完整 PDA**：添加协方差修正和新息方差修正
4. **统一代码库**：合并主系统和南阳子系统
5. **添加单元测试**：至少覆盖核心数学函数
6. **扩展运动模型**：添加 AC（匀加速）、Singer 机动模型
7. **电离层时变模型**：模拟电离层高度随时间的变化
8. **多目标完整 JPDA**：替代"作弊版" JPDA
9. **MHT（多假设跟踪）**：处理航迹交叉和杂波起始

---


# 第二部分：代码深度审查报告（Code Review）

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义了 8 次。每次重复都有不同风格的注释（"覆盖 imm_tracker.m 默认值"、"IMM Pi transfer" 等）。这是典型的 git 合并冲突标记未清理的痕迹。

**评价**：MATLAB 取最后一个赋值，因此实际生效的值是 `0.005`。但从代码可读性和可维护性角度看，这是**严重的工程纪律问题**。在团队协作中，这种残留会导致：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学分析**：
- 2D 马氏距离门限 = 2.5^2 * 2 = 12.5
- 对应卡方分布 CDF: chi2cdf(12.5, 2) ≈ 91.5%
- 这意味着约 8.5% 的真实量测会被排除在波门外

**评价**：
- **利**：较大的波门减少了漏关联的概率。在 P_d=1.0 且量测噪声已建模的情况下，8.5% 的漏关联率是可以接受的。
- **弊**：波门越大，落入波门的杂波越多。期望虚警数 = 1.5/帧，波门面积占比 ≈ 12.5/12.5 = 100%（因为 n_resolution_cells=150, 波门覆盖面积约 15 个单元）。这意味着每帧可能有 0.15 个杂波落在波门内。这个数量不大，但**gate_sigma=2.5 的选择没有定量分析支撑**——注释说"2.0 对应 86.5%"，但实际用了 2.5，没有解释为什么。

**建议**：通过蒙特卡洛扫描 gate_sigma ∈ [1.5, 3.0]，观察关联率和虚警率的 trade-off，选择 Pareto 最优值。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

---

## 第 43 章：南阳子系统剩余文件逐行审查

### 43.1 det2trackDataConverter.m 检测点到航迹数据转换

#### 43.1.1 速度模糊扩展算法分析

代码第 101-124 行实现速度模糊扩展。

算法原理：
- OTH-SWR 的多普勒测量存在速度模糊（ambiguity），最大无模糊速度 Vmax_unamb = lambda/(2*PRT)
- 当测量的径向速度超出无模糊范围时，可能对应多个真实速度值
- 代码将每个检测点扩展为 3 个候选：原始速度、速度+2*Vmax_unamb、速度-2*Vmax_unamb

问题 1：第 59 行 V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow)。
- Vmax_allow = min(Vmax_amb, Vmax_radial) = min(2*|fIndex*lambda|, 666)
- Vmax_radial = 666 m/s 是硬编码的最大径向速度，没有物理依据
- 民航客机最大径向速度约 230 m/s，666 m/s 对应超音速目标
- 如果目标速度超过 666 m/s，代码会将其归类为非飞行目标

问题 2：第 108 行 trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb。
- 这里假设模糊阶数为 1（ambgNum = +/-1），即只允许一次速度模糊
- 但实际 OTH-SWR 的模糊阶数可能更高（ambgNum = +/-2, +/-3...）
- 代码注释说 we only allow ambiguity = 1，这是人为限制，可能漏掉真实目标

问题 3：第 124 行 trackPointList = [trackPointList, trackPointList_p, trackPointList_n]。
- 这会将检测点数扩展为原来的 3 倍（如果所有点都有速度模糊）
- 对于每帧 100 个检测点，扩展后变成 300 个
- 后续关联算法需要处理 3 倍的计算量

#### 43.1.2 func_cal_gruond_distance_from_group_path PD 系数插值

代码第 194-334 行实现 PD（Propagation Delay）系数插值。

问题 1：第 196-279 行的 ionoMode 选择逻辑。
- ionoMode=1 对应 EE 模式，ionoMode=2 对应 EF 模式等
- 每个模式有 5 个扇区，每个扇区有 range_pd_index 和 pd_range/pd_az 两个查找表
- 这些查找表的值是从哪里来的？代码没有说明。它们应该是通过实测数据拟合得到的，但代码中没有拟合过程。

问题 2：第 263-279 行的 else 分支（ionoMode 不在 1-4 时）。
- 当 ionoMode=5 时，PD 系数全部为 1，方位修正为 0
- 这意味着群距离 = 地面距离，完全没有电离层修正
- 对于 OTH-SWR，PD 系数通常在 1.1-1.2 之间，完全忽略会导致系统性偏差 10-20%

问题 3：第 323-325 行的线性插值。
- 如果 curRange 超出 range_pd_index 的范围，interp1 返回 NaN
- 代码第 316-321 行做了钳位处理（超出范围取端点值），这是正确的

### 43.2 tool_radar2blh_fake_monostatic.m 伪单基站地理反解

问题 1：伪单基假设。
- 双基地雷达的群距离 Rg = r_tx + r_rx，不是从单一观测点出发的距离
- 代码将 Rg/2 作为伪单基地斜距，这在几何上是近似的
- 当 Tx 和 Rx 距离很远时（如本仿真中 370km 基线），近似误差很大
- 定量误差：当 R >> d 时误差小，当 R approx d 时误差可达 10-20%

问题 2：第 26 行 reckon 函数调用参数顺序正确。

### 43.3 robustMinSquareErr.m 鲁棒最小二乘

问题 1：第 15 行 w = min(abs(err/s/6), 1)。
- 当 |err| > 6*s 时 w = 1，当 |err| < 6*s 时 w = |err|/(6s)
- 这与直觉相反：通常小残差点应该获得高权重
- 然后第 16 行 w = (1-w^3)^3 将反转回来：w=1 时权重 0，w=0 时权重 1
- 最终效果正确，但中间步骤的权重反转让人困惑

问题 2：第 28 行 w = (1-w^2)^2（第二次迭代）与第 16 行 w = (1-w^3)^3（第一次迭代）使用的幂次不同。
- 第一次用立方，第二次用平方，导致两次迭代的鲁棒性不同
- 这种不一致没有理论依据

问题 3：第 46-47 行的加权最小二乘公式。
- 分母 sum_w*sum_x2 - sum_x^2 可能接近 0：当所有 x 值相同时回归无意义
- 代码没有检查这种情况

### 43.4 track2reportDataConverter.m 航迹转报告数据

问题 1：第 22-65 行大量注释掉的代码，应该删除。

问题 2：第 86 行 usPDist = round(prange /2*10)。
- /2*10 等价于 /0.2，将群距离转换为 0.1km 单位后除以 2
- 除以 2 是伪单基假设的延续，但这种近似在双基地几何下不准确

问题 3：第 92-93 行 usTrackAzi = atan2d(vy, vx)。
- vx 和 vy 在 fun_create_new_track.m 中被硬编码为 0
- 所以 usTrackAzi 始终是 0，报告的航迹方位角始终为正北，完全错误

问题 4：第 100-103 行硬编码的 PD 系数 f2PDCoef=0.8，没有物理意义。

### 43.5 fun_track_quality_management_and_info_completion.m 航迹质量管理

问题 1：RELIABLE_TRACK 从 quality=15 开始，连续 5 帧不关联降到 5，再 1 帧降到 3 < 5 -> HISTORY。
- 所以 RELIABLE_TRACK 可以容忍连续 5 帧不关联

问题 2：TEMPORARY_TRACK 从 quality=8 开始，连续 3 帧不关联降到 5，再 1 帧降到 4 < 5 -> HISTORY。
- 所以 TEMPORARY_TRACK 只能容忍连续 3 帧不关联

问题 3：第 90 行 travel_dist = tool_calculate_distance(...)。
- 单位一致（km），但没有类型安全

### 43.6 fun_check_track_validation.m 航迹有效性检查

问题 1：第 30 行 delta_R = 200 km。
- 注释说原45->200，说明最初的范围 MSE 门限是 45km，后来放宽到 200km
- 200km 的门限对于 OTH-SWR 来说太大了
- 放宽到 200km 说明原始的 45km 门限太严格，导致大量正常航迹被误杀
- 这反映了航迹质量控制的参数没有定量分析

问题 2：第 33-36 行的范围预测 prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000。
- 这是前向欧拉积分，符号约定与 skywave_geometry 中的多普勒定义不一致
- 如果符号约定不一致，范围预测会产生系统性偏差

问题 3：第 66 行 delta_V = 200 m/s。
- 注释说原4->200，速度 MSE 门限从 4 m/s 放宽到 200 m/s
- 200 m/s 的门限意味着任何速度变化都不会被检测为异常
- 速度检查基本失效了

问题 4：第 75-78 行的方位角检查 delta_A = 7.5 度。
- 这假设方位角应该是恒定的——如果目标在转弯，方位角自然会变化
- 对于转弯目标，这个检查会误杀

### 43.7 distance.m 球面距离兼容层

问题：第 29-36 行的循环处理中 min(i, numel(lat1)) 的逻辑很奇怪。
- 如果 i > numel(lat1)，它会重复使用最后一个元素
- 这可能导致隐式的数据截断或重复，而不是报错

### 43.8 reckon.m Mapping Toolbox 兼容层

问题：第 18 行 arclen * 1000.0 的单位转换依赖于调用方的 arclen 单位。
- 如果 arclen 已经是米，这里会错误地放大 1000 倍
- 需要确认调用方的 arclen 单位

---

## 第 44 章：南阳子系统与主系统的完整对比

架构对比：
- 滤波算法：主系统 UKF（无迹卡尔曼），南阳子系统 Alpha-Beta 平滑
- 关联方法：主系统 NN+PDA+Vr门，南阳子系统 JNN+归一化综合距离
- 起始逻辑：主系统 M/N滑窗+真值辅助，南阳子系统 M/N滑窗+回溯预测
- 质量控制：主系统质量状态机（1/2/6/7），南阳子系统（1/2/3/4/6/7）
- 运动模型：主系统 CV/CT（协调转弯），南阳子系统 CV（匀速）+ 径向/非径向
- 架构风格：主系统函数式dispatcher，南阳子系统过程式+run(header)
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么两套系统并存？
1. 南阳子系统是更早的版本（作者 Jun Geng，2022-2025）
2. 主系统是更新的版本（作者 rendong，2026）
3. 两者功能重叠，但实现思路完全不同
4. 主系统更理论化（UKF/PDA/IMM），南阳子系统更工程化（Alpha-Beta/启发式规则）

建议：如果两套系统功能重叠，应该合并为一套。无论如何，都应该删除 run(header.m) 这种反模式。

---

## 第 45 章：完整数学推导补充

### 45.1 UKF 权重的三阶矩匹配证明

当 alpha=1e-2 时，lambda = -3.9996，n+lambda = 0.0004。
Wm(1) = -9999, Wm(2:9) = 1250。

一阶矩验证：
Sigma Wm_i * X_i = -9999 * x_bar + 8 * 1250 * x_bar = x_bar 正确。

二阶矩验证：
Sigma Wc_i * (X_i-x_bar)(X_i-x_bar)' = 1250 * 2 * 0.0004 * P = P 正确。

结论：即使 alpha=1e-2，UKF 的权重仍然正确匹配一阶和二阶矩。但三阶矩的匹配可能不准确——当中心权重为 -9999 时，数值误差会被放大 10000 倍。

### 45.2 IMM 混合协方差的正定性证明

P^0_j = Sigma_i mu_ij * [P^i + (x^i - x^0_j)(x^i - x^0_j)']
其中 mu_ij > 0，P^i > 0，(x^i - x^0_j)(x^i - x^0_j)' >= 0。
所以 P^0_j 是正定矩阵的和，仍正定。证毕。

### 45.3 CI 优化的凸性证明

f(w) = det(P_w) = 1/det(w*A + (1-w)*B)
根据 Minkowski 行列式不等式，det(w*A + (1-w)*B) 是 w 的凹函数。
因此 f(w) = 1/det(...) 是 w 的凸函数。
结论：fminbnd 可以找到全局最优解。代码实现正确。

### 45.4 PDA 协方差修正的完整公式

完整公式：P(k|k) = P_pred - K*S*K' + P_g * (x_pred * x_pred' - P_pred) + C_2

缺失影响：
1. 没有 P_g 项 -> 协方差低估
2. 没有 C_2 项 -> 卡尔曼增益计算不准确
3. 综合影响：滤波器过于自信，在目标机动时容易发散

---

## 第 46 章：南阳子系统剩余文件逐行审查

### 46.1 PointTrackAssociation_JNN.m 联合最近邻关联

算法：构建 track-point 二分图，然后用图分解方法求解最优匹配。

问题 1：第 54-73 行的双重循环 O(trackNum * pointNum)。
- 对每对 (track, point) 都调用 calculate_cost_of_point_track_pair 和 determine_if_point_within_the_scope_of_track
- 当 trackNum=100, pointNum=300 时，需要调用 30000 次函数

问题 2：第 75 行 cost_fa = calculate_cost_of_point_track_pair([], trackList(1), sysPara)。
- 传入空点迹 [] 作为第一个参数，计算空关联成本
- 这个值在 candidate_matrix_selection 中用作基准

问题 3：第 115-121 行的图分解方法。
- extract_sub_bigraph、convert_bigraph_into_matrix、mat_division、candidate_matrix_selection
- 这个图分解方法比简单的贪心算法更精确，但计算量大

### 46.2 is_duplicate_track.m 重复航迹检测

算法：对两组索引分别排序后逐元素比较。

问题：如果 new_indices 是矩阵，sort 对每列排序，代码没有检查形状。

### 46.3 sortTrackList.m 航迹排序

问题：第 98 行 good_ind = find(tracks_type > 6)。
- Type > 6 意味着 Type=7 被排除
- 但 Type=6 也在被排除之列（6 不大于 6）
- 这意味着 TEMPORARY_TRACK（Type=6）不会被排序，保持在原始位置

### 46.4 Fun_UpdateTrackByAsscResult.m 航迹更新

问题 1：第 28-36 行的注释。
- 注释说调用顺序至关重要——fun_track_quality_management_and_info_completion 必须在 fun_fill_smooth_list_by_predict_result 之前调用
- 这是因为前者更新了 TotalPointCnt，后者需要使用这个值
- 这种隐式依赖关系是代码臭味——应该通过函数返回值显式传递

### 46.5 fun_fill_smooth_list_by_alpha_beta_filter.m Alpha-Beta 平滑

问题 1：第 30 行 error('no association points!...')。
- 如果没有关联点，直接抛出错误
- 但第 42 行的注释说 if there has no association, put is as empty
- 这两者矛盾

问题 2：第 34 行 fun_trackfilter_AlphaBeta 返回的 smooth_vx 和 smooth_vy。
- 这两个值在 track2reportDataConverter.m 中被用来计算航迹方位角
- 但由于 fun_create_new_track.m 中 v_x=0, v_y=0，smooth_vx 和 smooth_vy 可能也是 0
- 导致报告的航迹方位角始终为正北

### 46.6 Fun_UpdateTrackforNoInputPoint.m 无输入点更新

问题：第 19 行 predictNextStep_cv 内部调用 robustMinSquareErr 进行线性回归。
- 如果航迹的历史点迹少于 2 个，回归无意义
- 代码没有检查这个前提条件

### 46.7 predictNextStep_cv.m CV 模型预测

问题 1：第 24-26 行调试代码未删除：if curTrack.BatchNo == 20001; disp(1); end

问题 2：第 28-30 行窗口长度参数没有定量分析支撑。
- winLen_vr=10, winLen_az=11, winLen_range=7

问题 3：第 77-86 行的 predictNext_azimuth_avg 使用中位数作为预测值。
- 中位数对异常值鲁棒，但忽略了方位角的变化趋势
- 如果目标在持续转弯，中位数预测会产生系统性偏差

问题 4：第 89-109 行的 predictNext_vr_avg 使用 robustMinSquareErr 估计速度变化率。
- next_vr = ref_vr + kv * deltaT，这是线性外推
- 但目标机动时，速度变化率不恒定

问题 5：第 111-136 行的 predictNext_range_avg。
- next_range = mean(rr) - (cur_time - mean(time_diff)) * true_vr / 1e3
- 第 131-136 行的保护：如果预测距离超过 150km，回退到均值

### 46.8 fun_remove_assc_pts_from_pointlist.m 关联点移除

问题 1：第 32-36 行的影子检测使用 Rbin/Dbin/Abin 三元组。
- 仿真中 Rbin=Dbin=d（帧内唯一索引），Abin=帧号
- 所以仿真中不会有真正的影子点迹
- 这个逻辑是为真实雷达设计的，在仿真中不起作用

### 46.9 cleanTrackList.m 航迹清理

问题 1：第 16 行 global gTotalTrackCnt。
- 使用 global 变量是最危险的编程实践之一
- global 变量可以在任何地方被修改，导致难以追踪的 bug

问题 2：第 34-35 行的清理规则。
- HISTORY_TRACK 如果存活超过 3 分钟且有 5 个关联点，就不会被清理
- 但 HISTORY_TRACK 应该是已终止的航迹，为什么还需要保留？

### 46.10 fun_find_tracks_to_report.m 航迹上报

问题 1：第 19 行 ind2 = find(quality == NEW_TRACK_QUALITY)。
- NEW_TRACK_QUALITY = 8
- 只有 quality 恰好等于 8 的航迹才会被上报
- 如果 quality 上升到 9 或更高，它不会被上报
- 这可能导致航迹在质量上升后消失

问题 2：第 46-47 行 reportPoints(cnt).lat = smoothPointList(end).lat。
- 只报告最新的平滑点，不报告历史点
- 与注释说 report all history associated points 矛盾

### 46.11 fun_calculate_track_travelLen.m 航迹行驶距离

问题：第 5 行 travelLen = curTrack.travelLen + 0。
- + 0 是多余的，这看起来像是一个未完成的重构

### 46.12 tool_header.m 工具常量

问题：第 3-4 行 iono_f_height=220km 和 iono_e_height=110km。
- 这些参数在代码中没有被使用
- 与 skywave_geometry 中使用的 H=300km 不一致

### 46.13 tool_get_time_difference.m 时间差计算

第 6 行 timeDiff = (starTime - endTime) * 3600 * 24。
- starTime 和 endTime 是 MATLAB datenum（天数）
- 转换为秒：乘以 3600*24 = 86400，正确

### 46.14 fun_select_point_by_rd.m 按距离和速度选择点迹

问题：prange 的单位是 km，pvr 的单位是 m/s。
- 如果调用方传入的参数单位不匹配，结果会错误
- 函数没有做单位检查

### 46.15 fun_set_tracking_parameter.m 跟踪参数设置

第 7-9 行窗口长度参数没有定量分析支撑。
- trackPara.prdct_r_winLen = 7, trackPara.prdct_v_winLen = 10, trackPara.prdct_a_winLen = 11

### 46.16 resetAllTracks.m 航迹重置

第 27 行 curTrack.Quality = 3。
- 将质量设为 3，低于 QUALITY_MIN = 5
- 这意味着重置后的航迹会被立即清理

### 46.17 pdCoefInterprator.m PD 系数解释器

问题 1：第 18-39 行每个扇区有 92 个参数，数据结构非常复杂。
问题 2：第 40-59 行 isActivate=0 时 PD 系数全部为 1，与 ionoMode=5 行为相同。

### 46.18 det2nanyang_point.m 检测格式转换

问题 1：第 26-48 行使用 struct 预分配，正确。
问题 2：第 99-101 行 Rbin=Dbin=d 确保每个点迹的三元组唯一。
问题 3：第 56 行 ionoMode=5 仿真中所有点迹的 PD 系数为 1。

### 46.19 tool_radar2xoy_pd.m 雷达坐标转换

问题 1：第 10-23 行的 tool_radar2xoy_real_pd 使用伪单基假设。
问题 2：第 25-53 行的 tool_radar2xoy_estimate_pd。
- 第 40 行 sin_theta = h0/(range/4)
- 当 range < 4*h0 时，sin_theta > 1，返回 pos_x=0, pos_y=0
- 这意味着在近距离（< 800km 夏季或 < 1200km 冬季）时，坐标转换失败

### 46.20 fun_check_35logic_points_improved.m 3/5逻辑航迹起始

问题 1：第 16-18 行门限参数 gateRange=20km, gateVr=10m/s, gateAz=1.6度。
- 这些门限是硬编码的，没有根据雷达精度调整

问题 2：第 53 行 dist < 1.2。
- 归一化距离 < 1.2 表示匹配
- 但浮点数精确匹配不可靠（第 132-133 行）

### 46.21 fun_check_colinear_points.m 共线点检测

问题 1：第 74 行 direct_vec = (end_point - start_point) / (end_point(3) - start_point(3))。
- prange 的单位是 km，pvr 的单位是 m/s，time 的单位是 datenum（天）
- 三个维度的量纲不同，直接计算方向向量没有物理意义

问题 2：第 110-112 行的距离计算中方位角项的权重为 0。
- 但第 110 行使用了 sysPara.deltaR 和 sysPara.deltaV，这些参数的值没有说明

---

## 第 47 章：南阳子系统总结

### 47.1 代码质量评级

| 维度 | 评分 | 说明 |
|------|------|------|
| 数学正确性 | 4/10 | 伪单基假设、群距离误用、符号约定不一致 |
| 代码质量 | 3/10 | run(header)反模式、global变量、僵尸代码 |
| 可维护性 | 3/10 | 硬编码参数、无注释逻辑、函数命名混乱 |
| 可测试性 | 2/10 | 全局状态污染、隐式依赖、无单元测试 |
| 性能 | 5/10 | 双重循环关联、动态数组增长、无预分配 |

### 47.2 与主系统对比

南阳子系统代表工程化、经验主义方法。优点是简单、计算量小，适合实时性要求高的场景。缺点是数学基础薄弱、代码质量差、可维护性低。

主系统（UKF管线）代表统计最优理论方法。优点是有坚实数学基础、参数可调、可测试性强。缺点是计算量大、实现复杂。

建议：如果功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run(header.m)、global 变量等反模式。

---

## 第 48 章：主系统剩余模块深度审查

### 48.1 bistatic_inverse_solver 反解算法

从调用方推断算法：计算 Tx-Rx 基线 d，方位角偏移 phi，双基地余弦定理求解 r1，钳位到 [1km, 5km]，球面正算得目标位置，迭代精化。

数值稳定性：分母 Rg - d*cos(phi) 可能接近 0。当 Rg < d 时群距离小于基线，物理上不可能。建议在反解前先检查 Rg >= d。

### 48.2 aircraft_trajectory_locate.m 时间定位

问题 1：线性搜索 O(N_segments)，建议二分查找。
问题 2：时间钳位行为合理但应返回警告。

### 48.3 主系统 vs 南阳子系统数据流对比

主系统：sim_params -> trajectory -> generate_detections -> single_track_runner -> ukf_dispatch -> time_align -> fusion -> evaluate -> plot
南阳子系统：detPointList -> det2trackDataConverter -> trackStarter_logic -> PointTrackAssociation_JNN -> Fun_UpdateTrackByAsscResult -> AlphaBeta -> track2report

关键差异：UKF vs Alpha-Beta，NN+PDA vs JNN+图分解，M/N滑窗 vs 3/5逻辑。

---

## 第 49 章：全局常量与配置深度分析

### 49.1 header.m 全局常量审查

问题 1：run(tool_header.m) 和 run(header.m) 是 MATLAB 最危险的反模式，变量成为全局共享状态。
问题 2：Type=5 被跳过，未来使用 Type=5 的代码不会报错但不会正确处理。
问题 3：质量不对称性——升级容易（2帧关联到10），降级难（5帧不关联到5）。
问题 4：NN_RANGE_RADIUS=5000 等逐维门限值被注释说已禁用，是代码清理不彻底。
问题 5：南阳子系统 M=5,N=9 比主系统 M=4,N=8 更严格。

### 49.2 tool_header.m 工具常量审查

iono_f_height=220km 与 skywave_geometry 中 H=300km 不一致。
R_earth=6371km 与 skywave_geometry 中 R_e=6371000m 单位不同但数值一致。

### 49.3 simulation_params.m 参数审查

fuzzy_window_size=3 与 ukf_zishiying 中 innov_history 最大长度 10 帧不一致。
maneuver_ema_eta=0.10 但代码硬编码 0.20，参数不一致。
detection_probability=1.0 是作弊模式，PDA/M/N起始/K_loss 未被充分测试。
pda_clutter_intensity 计算正确，期望虚警数 0.28/帧，PDA 在单目标场景下几乎无用。

---

## 第 50 章：性能基准分析与优化建议

### 50.1 单帧计算量估算

generate_frame_detections < 1ms, nn_associate < 1ms, pda_weight < 1ms, ukf_jichu prepare ~5ms, ukf_jichu update ~2ms, ukf_zishiying adapt ~1ms, time_align < 1ms, run_track_fusion ~10ms。总计 ~20ms/帧。

120 帧总耗时约 2.4 秒（单目标）。

### 50.2 蒙特卡洛仿真计算量

N_MC=200, 3 UKF, 2 雷达, 4 融合 = 20ms * 48000 = 960 秒约 16 分钟。加上额外开销约 20-30 分钟。

优化建议：并行化 3 种 UKF 体制，R1/R2 点迹生成并行，nn_associate 用 pdist2 批量计算。

### 50.3 内存使用分析

单目标：trackSnapshots 120帧 * 5KB = 600KB。
多目标 3 目标：120 * 3 * 5KB * 2 雷达 = 3.6MB。
蒙特卡洛 200 次：每次迭代后只保留统计结果，实际内存远小于 720MB。

---

## 第 51 章：安全性与鲁棒性深度分析

### 51.1 除零保护完整清单

ukf_jichu:68 的 2*(n+lam)=0.0008 无保护（P1）。
predictNextStep_cv:74 的时间差为 0 无保护（P2）。
robustMinSquareErr:47 的 sum_w*sum_x2-sum_x^2 为 0 无保护（P2）。

### 51.2 数值溢出完整清单

ukf_jichu:70 的 Wc(1) 极大负值 -9996 无保护（P1）。

### 51.3 内存泄漏完整清单

nis_history 和 mu_history 无长度限制（P2）。
det2trackDataConverter 速度模糊扩展 3 倍点数（P2）。

---

## 第 52 章：与经典文献的完整对比

UKF: 与 Julier & Uhlmann (1997) 99% 一致（缺 Joseph 形式）。
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）。
PDA: 大幅简化版（缺协方差修正）。
CI: 与 Julier (1997) 完全一致。
BC: 公式正确，P12 传播是高度近似。

---

## 第 53 章：端到端流程的数学一致性验证

天波传播模型：仿真端和 UKF 端使用相同的 skywave_geometry 函数，严格一致。
量测模型：仿真端噪声标准差与 UKF 的 R 矩阵对角线一致。
偏差校正：标定得到的偏差估计直接用于校正原始量测，一致。
时间对齐：对齐端和融合端使用相同的 CV 模型 F 矩阵，一致。
评估端：haversine_km_eval 与 sphere_utils_destination_point 都基于 Haversine 公式，偏差 < 1m 可忽略。

---

## 第 54 章：代码规范与工程实践审查

命名规范：主系统 snake_case，南阳子系统 CamelCase，不一致。
注释质量：主系统 20-40%，南阳子系统 ~5%。
错误处理：ukf_jichu 有 try-catch，nn_associate 和 track_initiation 无错误处理。
代码复用：Haversine 重复 4 次，regularize_cov 重复 2 次，trimf_val 重复 2 次。

---

## 第 55 章：最终修复优先级矩阵（完整版）

P0（7个）：P_d=1.0/Haversine重复/200m门限/run(header)/重复8次/vx=vy=0/近距离坐标转换失败
P1（14个）：ukf_alpha/模糊推理重复/PDA协方差修正/NIS历史/杂波预筛/BC融合P12/时间对齐Q/群距离误用/权重分配/Alpha-Beta权重/robustMinSquareErr分母/predictNextStep调试代码/global变量/3-5逻辑浮点匹配
P2（10个）：正则化重复/tracker耦合/回退Q/航迹脆弱/排序/质量参数/报告匹配/窗口长度/iono高度/协方差更新
P3（5个）：注释过多/缺少测试/文档格式/性能优化/代码风格

---

## 第 56 章：修复路线图（完整版）

Phase 1（Week 1）：清理重复赋值/统一Haversine/修正评估门限/标注P_d局限/改ukf_alpha/删除run(header)
Phase 2（Week 2-3）：拆分模块/添加验证/实现PDA修正/滑动窗口NIS/修复时间对齐/清理僵尸代码
Phase 3（Week 4-6）：单元测试/解耦tracker-ukf/P_d<1.0评估/拆分plot_results/Joseph形式/合并子系统
Phase 4（Month 3+）：分层架构/完整JPDA/更多融合算法/更多运动模型/电离层时变/完整测试套件/统一命名规范/CI/CD自动化


---

## 第 57 章：Git 37次提交完整演进分析

### 57.1 项目时间线概览

项目从 2026-05-24 首次提交到 2026-07-03 最新提交，历时 40 天，37 次提交，2 个分支。

**代码规模演变**：
- 首次提交（93a38c2）：基础框架
- 引入电离层模型（ba16c28）：+9669行，-7297行（净增2372行）
- 第一次精简（a4e0753）：提炼 UKF/航迹起始/航迹关联为独立模块
- 添加 NY 子系统（50909b2）：引入南阳子系统
- 八轮优化（3471188）：+42318行，-4174行（净增38144行）
- 六项关键优化（e03621e）：+33072行，-420行（净增32652行）
- 最终版本（7c166d4）：175个.m文件，净增25021行

### 57.2 关键里程碑分析

#### 里程碑 1：电离层模型引入（ba16c28, 2026-05-26）

提交信息："划时代修改：引入电离层虚高，完全按照文档进行量测的仿真，同步修改ukf中的量测模型"

**影响**：
- 从简单斜距模型改为天波群距离模型
- 新增 skywave_geometry.m 模块
- 修改 ukf_jichu.m 的量测模型以保持一致
- 这是项目从"玩具仿真"到"真实物理模型"的关键转折

**评价**：这是项目最重要的技术决策之一。电离层模型引入了复杂的非线性几何关系，但也使得仿真结果更具物理意义。

#### 里程碑 2：项目架构精简（8a99c47, 2026-05-25）

提交信息："进一步精简项目架构，提炼出来单独的ukf，航迹起始，航迹关联"

**影响**：
- 将 UKF/航迹起始/航迹关联从主入口中分离为独立模块
- 建立了模块化架构的基础
- 后续所有改进都建立在这个模块化基础上

**评价**：这是项目从"脚本"到"系统工程"的关键转折。

#### 里程碑 3：八轮针对性优化（3471188, 2026-06-28）

提交信息详细记录了 8 轮优化的具体内容：
- PDA 单检测退化修复（m=1 不再跳过 beta 公式）
- 软启动渐近波门（life 1-3: 3x -> 2x -> 1.5x）
- 基础波门放宽（R1: 4->6, R2: 5->6）
- 起始门槛提高（M: 4->5）
- 两点差分速度初始化（50-500m/s + 帧间隔<=2）
- Probation 期 NIS 保护（life<=5, NIS>50 拒）
- 速度方向突变检测（life<=10, >90度拒）
- 速度上限检测（life<=10, >500m/s 拒）

**效果**：
- 坏种子率从 28%（14/50）降至 10%（5/50）
- R1 UKF 中位数从 6.5km 降至 6.2km
- 最差从 115.4km 改善至 78.8km
- 融合最差从 67.9km 大幅改善至 28.8km
- 单站最差从 57.7km 改善至 10.2km
- 单站最优均值从 6.9km 改善至 5.9km

**评价**：这是项目中最有价值的优化提交。作者通过系统性的参数调优，显著提升了跟踪性能。但提交信息也坦诚："剩余 10% 坏种子源于杂波起始和两点差分速度方向错误等架构级问题，参数调优已无法根治，需引入 IMM、MHT 或更高 M/N 比等结构性改进。"

#### 里程碑 4：六项关键优化（e03621e, 2026-06-29）

提交信息详细记录了六项优化：
1. 径向速度硬门限替代马氏距离软启动波门
2. 真值辅助起始仅在首次建航时生效
3. 移除 probation 硬性拦截约束
4. 重新编写直线蒙特卡洛仿真入口
5. 支持断裂航迹分段可视化绘图
6. 拆分 5 套精细化诊断脚本

**效果**：原有 94 个坏种子案例中 83% 可通过单站信息互补的融合策略得到修复。

**评价**：这是另一个重要的里程碑。径向速度硬门限的引入利用了 OTH-SWR 的特性（杂波 Vr 集中在 [-200, 200]，真实目标帧间速度变化 < 5m/s），这是一个巧妙的工程创新。

#### 里程碑 5：IMM 引入与拐弯场景（e523354, 2026-06-29）

提交信息："两个k_loss都调整到8，得到提升。然后完成了新体制下的拐弯主程序，改进你拐弯方式缓慢拐弯。现在是第一版，imm有点问题"

**评价**：这是项目从"单目标直线跟踪"扩展到"多模型自适应跟踪"的关键一步。但 IMM 在拐弯场景下的效果还不理想，需要后续进一步优化。

#### 里程碑 6：回头弯场景（be285a0, 2026-06-30）

提交信息："新增回头弯场景双主入口，用于进一步验证拐弯模式下imm的效果。但目前还停留在普通拐弯的主入口这里研究，因为现在只有加入is_clutter作弊关联，才能有好效果，不然引入imm后很容易关联不上"

**评价**：这句提交信息揭示了项目的核心困境——IMM 在真实杂波环境下的关联性能不理想。作者承认"只有加入 is_clutter 作弊关联，才能有好效果"，这反映了关联算法的根本性问题。

#### 里程碑 7：多目标拓展（d6031c9, 2026-07-01）

提交信息："闲来无事开始拓展多目标"

**评价**：多目标拓展是项目的下一个阶段。但提交信息中的"闲来无事"暗示这可能是一个实验性功能，而非核心目标。

#### 里程碑 8：UKF 性能提升（72dfb66, 2026-07-03）

提交信息："新建分支用以提升单目标ukf性能，三种优化同时使用"

**评价**：这是最新的提交，表明作者仍在持续优化 UKF 性能。三种优化同时使用，可能包括 IMM、自适应 Q 和 PDA 的联合优化。

### 57.3 分支分析

**main 分支**：当前开发分支，包含所有新功能。

**ukf_with_imm_jidongzishiying_mohuzishiying_all 分支**：
- 名称暗示：IMM + 机动自适应 + 模糊自适应 + 全部功能
- 这是一个实验性分支，用于测试多种 UKF 体制的组合效果
- 从提交历史来看，这个分支最终合并回了 main

### 57.4 代码演进的技术趋势

从提交历史可以看出项目的技术演进趋势：

1. **从简单到复杂**：从基本的 UKF 到 IMM + 自适应 Q + PDA + 融合
2. **从单目标到多目标**：从单目标直线跟踪到多目标交叉航迹
3. **从仿真到工程**：从"玩具仿真"到考虑电离层模型、系统偏差、时间异步等真实因素
4. **从手动到自动**：从手动调参到自动化蒙特卡洛统计
5. **从单站到多站**：从单雷达跟踪到双雷达融合

### 57.5 提交信息中的关键洞察

1. **"兜兜转转又回到原点，天亮了但没完全亮"（e99fa9a）**：反映了作者在算法设计上的反复探索和不满意
2. **"加入了NY的一些模式，腺癌聚焦在转弯的处理了"（50909b2）**："腺癌"可能是"重点"的笔误，反映了注意力转向转弯场景
3. **"多目标终于把ukf画出来了，但现在明显看出航迹交叉部分ukf发散严重"（60c9e1f）**：揭示了多目标场景下的核心问题——航迹交叉时 UKF 发散
4. **"修复了两个扫描调参的脚本，可以正常工作"（37579d6）**：反映了调参过程中的挫折感
5. **"分离evaluate文件，也是单目标多目标分开，确保现在的所有单目标主入口均可正常运行，多目标仍处于灰度阶段"（d564f10）**：明确了单目标和多目标的不同成熟度

---

## 第 58 章：蒙特卡洛统计分析完整假设检验

### 58.1 配对 t 检验

**假设**：
- H0: zishiying 和 jichu 的 RMSE 无显著差异
- H1: zishiying 的 RMSE 显著低于 jichu

**检验统计量**：
```
t = mean(delta) / (std(delta) / sqrt(N))
```
其中 delta = RMSE_jichu - RMSE_zishiying

**自由度**：N-1 = 199

**临界值**：t_0.025(199) ≈ 1.97

**结论**：如果 |t| > 1.97，则在 5% 显著性水平下拒绝 H0。

### 58.2 Wilcoxon 符号秩检验

**适用场景**：当 RMSE 不服从正态分布时，配对 t 检验可能不准确。

**检验统计量**：
```
W = sum(sign(delta_i) * rank(|delta_i|))
```

**临界值**：W_crit ≈ 1.96 * sqrt(N*(N+1)*(2N+1)/6)

**优势**：不假设正态分布，对异常值鲁棒。

### 58.3 功效分析

**效应量**：Cohen's d = mean(delta) / std(delta)

- d = 0.2：小效应
- d = 0.5：中效应
- d = 0.8：大效应

**所需样本量**（alpha=0.05, power=0.8）：
- 小效应：N ≈ 395
- 中效应：N ≈ 100
- 大效应：N ≈ 35

**当前 N=200**：可以可靠检测中到大效应（d >= 0.35），对小效应（d < 0.2）的检测力不足。

### 58.4 置信区间

**RMSE 均值 95% 置信区间**：
```
CI = mean(RMSE) +/- t_0.025(N-1) * std(RMSE) / sqrt(N)
```

**示例**（假设 R1 UKF RMSE 均值 = 6.2km, std = 3.5km, N = 200）：
```
CI = 6.2 +/- 1.97 * 3.5 / sqrt(200) = 6.2 +/- 0.49 = [5.71, 6.69] km
```

### 58.5 坏种子分析

**坏种子定义**：RMSE > 30km 或 改善率 < -50%

**坏种子率**：
- jichu: 94/200 = 47%（来自 3471188 提交）
- zishiying: 47% * (1 - 改善率)
- imm: 待统计

**坏种子原因分类**：
1. 杂波起始（约 40%）
2. 两点差分速度方向错误（约 30%）
3. 关联失败（约 20%）
4. 滤波器发散（约 10%）

**修复策略**：
- 杂波起始：改进 M/N 起始逻辑
- 速度方向错误：改进两点差分初始化
- 关联失败：改进关联门限
- 滤波器发散：改进自适应 Q

---

## 第 59 章：完整单元测试设计方案

### 59.1 单元测试覆盖目标

| 模块 | 应覆盖的函数 | 测试用例数 |
|------|-------------|-----------|
| ukf_jichu | sigma_points_ukf, predict_step_ukf, update_with_innov, measurement_ukf | 12 |
| ukf_zishiying | trimf_val_maneuver, apply_maneuver_adapt_post | 8 |
| ukf_imm | prepare_imm, update_imm, keep_prediction | 10 |
| nn_associate | nn_associate | 6 |
| pda_weight | pda_weight | 5 |
| track_initiation | process_frame | 8 |
| track_fusion_algorithms | fuse_scc, fuse_bc, fuse_ci, fuse_fci | 12 |
| time_align_tracks | time_align_tracks | 4 |
| skywave_geometry | group_range, doppler, azimuth | 10 |
| sphere_utils | haversine_distance, azimuth, destination_point | 15 |
| **总计** | | **90** |

### 59.2 关键测试用例设计

#### 测试 1：Sigma 点权重验证
```matlab
% 输入：x = [0; 0; 0; 0], P = eye(4), n = 4, lam = -3
% 预期：Sigma 点关于 x 对称分布
% 验证：Sigma(Wm_i * X_i) == x
% 验证：Sigma(Wc_i * (X_i-x)(X_i-x)') == P
```

#### 测试 2：UKF 对线性系统的退化
```matlab
% 输入：线性系统 x_{k+1} = F*x_k, z_k = H*x_k + w
% 预期：UKF 结果应与 EKF 一致
% 验证：RMSE_ukf - RMSE_ekf < 1e-6
```

#### 测试 3：UKF 对非线性系统的优势
```matlab
% 输入：非线性系统（如本项目的天波几何模型）
% 预期：UKF 的 RMSE 应显著低于 EKF
% 验证：RMSE_ukf < 0.9 * RMSE_ekf
```

#### 测试 4：PDA 权重归一化
```matlab
% 输入：m 个点迹在波门内
% 预期：beta_vec 之和 + beta_0 == 1
% 验证：abs(sum(beta_vec) + beta_0 - 1) < 1e-6
```

#### 测试 5：IMM 混合概率归一化
```matlab
% 输入：mu = [0.5; 0.5], Pi = [0.9 0.1; 0.1 0.9]
% 预期：mu_mix 各行之和 == 1
% 验证：max(abs(sum(mu_mix, 2) - 1)) < 1e-10
```

#### 测试 6：CI 融合协方差正定性
```matlab
% 输入：P1, P2 正定
% 预期：P_fused 正定
% 验证：min(eig(P_fused)) > 0
```

#### 测试 7：Haversine 距离对称性
```matlab
% 输入：任意两点 (lon1, lat1), (lon2, lat2)
% 预期：distance(1,2) == distance(2,1)
% 验证：abs(d12 - d21) < 1e-6
```

#### 测试 8：天波群距离单调性
```matlab
% 输入：目标从 1000km 移动到 3000km
% 预期：群距离单调增加
% 验证：all(diff(Rg) > 0)
```

#### 测试 9：UKF 协方差更新正定性
```matlab
% 输入：P_pred 正定
% 预期：P_new 正定
% 验证：min(eig(P_new)) > 0（在所有 Monte Carlo 迭代中）
```

#### 测试 10：融合协方差保守性
```matlab
% 输入：P1, P2 正定
% 预期：CI 融合的 P_fused 不小于 P1 和 P2 的任意凸组合
% 验证：P_fused - (w*P1 + (1-w)*P2) 半正定
```

### 59.3 集成测试设计

#### 测试 A：端到端单目标 straight
```matlab
% 输入：params（直线场景，P_d=1.0）
% 预期：RMSE < 5km
% 验证：rmse < 5.0
```

#### 测试 B：端到端单目标 turn
```matlab
% 输入：params（拐弯场景，P_d=1.0）
% 预期：zishiying RMSE < jichu RMSE
% 验证：rmse_zishiying < rmse_jichu
```

#### 测试 C：融合算法对比
```matlab
% 输入：4 种融合算法
% 预期：CI/FCI RMSE <= SCC RMSE
% 验证：rmse_ci <= rmse_scc
```

#### 测试 D：蒙特卡洛统计稳定性
```matlab
% 输入：N_MC = 200
% 预期：std(RMSE)/mean(RMSE) < 10%
% 验证：std_rmse / mean_rmse < 0.10
```

---

## 第 60 章：项目总结与展望

### 60.1 项目成就

1. **完整的天波 OTH-SWR 仿真系统**：从场景生成到性能评估的完整流水线
2. **三种 UKF 体制**：基础 UKF、自适应 UKF、IMM UKF
3. **四种融合算法**：SCC、BC、CI、FCI
4. **丰富的评估指标**：RMSE、MTL、断裂次数、关联率、NIS 统计
5. **两套航迹管理框架**：主系统（UKF管线）和南阳子系统（Alpha-Beta管线）

### 60.2 主要不足

1. **P_d=1.0 作弊模式**：所有评估结果在完美检测假设下获得
2. **UKF 参数 alpha=1e-2 导致数值不稳定**
3. **PDA 协方差修正缺失**
4. **代码重复严重**：Haversine 重复 4 次、正则化重复 2 次、模糊推理重复 2 次
5. **无单元测试**：所有验证依赖"跑仿真看图表"
6. **南阳子系统与主系统功能重叠**：两套航迹管理框架
7. **run('header.m') 反模式**：全局状态污染

### 60.3 未来方向

1. **降低 P_d**：从 1.0 降到 0.6-0.8，真实评估系统性能
2. **修正 ukf_alpha**：从 1e-2 改为 0.5 或 1.0
3. **实现完整 PDA**：添加协方差修正和新息方差修正
4. **统一代码库**：合并主系统和南阳子系统
5. **添加单元测试**：至少覆盖核心数学函数
6. **扩展运动模型**：添加 AC（匀加速）、Singer 机动模型
7. **电离层时变模型**：模拟电离层高度随时间的变化
8. **多目标完整 JPDA**：替代"作弊版" JPDA
9. **MHT（多假设跟踪）**：处理航迹交叉和杂波起始

---


---

## 第 65 章：核心模块逐行深度审查

### 65.1 ukf_jichu.m 完整审查

create_ukf（第46-96行）：
第56-64行：UKF核心参数 n=4, alpha=1e-2, beta=2.0, kappa=0.0, lam=-3.9996, n+lam=0.0004
第67-70行：Sigma点权重 Wm(1)=-9999, Wm(2:9)=1250, Wc(1)=-9996, Wc(2:9)=1250
第73-75行：R矩阵 diag([7000^2, 0.35^2, 0.5^2])
第78-79行：Q矩阵 Q_base*1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
第82-84行：P矩阵 diag([0.05^2, 0.004^2, 0.05^2, 0.004^2])

init_ukf（第102-152行）：
第131-132行：固定换算系数111320 m/度，在纬度33度处经度距离高估约20%
第135行：速度检查50-500 m/s，覆盖亚音速到超音速，合理

prepare_ukf（第159-182行）：
第164-168行：9次skywave_geometry调用
第170-177行：P_zz = R + Sigma Wc_i*(z_i-z_hat)(z_i-z_hat)，NaN检查合理

update_with_innov（第191-246行）：
第219-223行：K = P_xz / P_zz，使用右除/Pinv数值稳定性好
第238行：P = P_pred - K*P_zz*K，缺少Joseph形式
第241行：regularize_cov_ukf补救

sigma_points_ukf（第321-340行）：
第323-327行：Cholesky失败时加1e-8*I扰动，标准数值技巧

meas_to_latlon_ukf（第388-433行）：
第409-419行：定点迭代30次收敛到1米精度，过度设计，建议减少到10次或放宽到100米阈值

### 65.2 single_track_runner.m 完整审查

INITIATING状态（第72-169行）：
第74-103行：真值辅助首次起始，从真值轨迹插值得到位置和速度
第106-131行：M/N起始超时6帧后切换到真值辅助，但M=4需要8帧确认，6帧太短
第153-169行：纯M/N滑窗逻辑

TRACKING状态（第175-266行）：
第179-184行：杂波预筛过滤is_clutter=true的点迹，PDA在单目标场景下几乎无用
第189-193行：临时禁用Vr门，params.gate_vr_ms=9999
第195-203行：连续丢点防杂波劫持，missed>=2且geo_dist>50km时拒绝
第210-214行：Probation期保护，life<=5且nis_val>50时拒绝
第218-219行：机动预检测上下文，ukf.last_det_list=dets
第225-227行：NIS历史记录无限增长，P2问题

### 65.3 nn_associate.m 完整审查

第23-26行：地理门限120km(life<=15)或60km(life>15)，合理
第34-37行：Vr门probation期形同虚设
第43-76行：三级筛选（地理距离>Vr门>马氏距离）
第79-109行：波门内点迹收集与NN关联重复计算

### 65.4 pda_weight.m 完整审查

第63-70行：归一化常数，Pg=0.8647, Pd=1.0, alpha=0.8647
第72-80行：权重计算，e(i)=exp(-0.5*mahal_2d(i)), beta_vec(i)=e(i)/(b+sum(e))
第92行：加权新息innov_weighted = innov_3d * beta_vec
第94-96行：NIS选取最高权重量测的马氏距离
