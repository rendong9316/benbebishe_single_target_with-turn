% =========================================================================
% 重构分析报告 — 单目标跟踪仿真项目
% =========================================================================
% 生成日期: 2026-07-20
% 分析范围: 全部 65 个 .m 文件
% =========================================================================

%% 1. 项目概览

% 项目文件总数: 65 个 .m 文件
% 代码总行数: ~20,550 行
% 目录分布:
%   tracker/     — 17 个文件, ~3,300 行 — 航迹生命周期、关联、起始、片段管理
%   ukf/         —  6 个文件, ~1,900 行 — UKF 滤波器（基础/IMM/自适应/调度）
%   simulation/  —  9 个文件, ~2,200 行 — 真值生成、检测生成、偏差标定
%   fusion/      —  7 个文件, ~2,100 行 — 时间对齐、片段提取、分组、融合算法、桥接
%   evaluation/  —  2 个文件,   ~950 行 — RMSE 评估
%   visualization/ — 6 个文件, ~1,600 行 — 绘图
%   utils/       —  9 个文件,   ~750 行 — 球面几何、坐标系
%   validation/  —  4 个文件, ~1,200 行 — 测试验证
%   入口脚本     —  4 个文件, ~2,200 行 — run.m / run_without_fusion.m 等
%   config/      —  1 个文件,   ~350 行 — 参数配置
%   io/          —  1 个文件,    ~90 行 — 数据提取工具

%% 2. 四条并行流水线（入口脚本）

% 2.1 run.m (660行)
%     完整融合 pipeline:
%     Phase 0: Oracle 场景初始化
%     Phase 1: ADS-B 系统偏差标定
%     Phase 2: 多目标点迹生成 + 偏差校正
%     Phase 3: UKF/IMM 滤波器模板初始化
%     Phase 4: 南阳式 Oracle 航迹维护
%     Phase 5: 跨雷达航迹时间对齐
%     Phase 6: 跨雷达航迹匹配（双门限法 / 传统 matcher）
%     Phase 7: 航迹级融合（SCC/BC/CI/FCI 四种算法）
%     Phase 8: RMSE 定量误差评估
%     Phase 9: 可视化绘图
%     Phase 10: 数据保存

% 2.2 run_without_fusion.m (200行)
%     基线对照: 单站跟踪 + RMSE，无融合

% 2.3 run_fragment_study.m (477行)
%     碎片实验: 人工制造互补 gap + 片段分组 + 外部评估

% 2.4 run_random_fade_fragment_fusion.m (191行)
%     衰落实验: 自动衰落窗口搜索 + 跟踪 + 片段提取 + 分组 + 融合

% 四条流水线服务不同的研究目的，应保持独立性。
% 它们不是"走不同路径的同一条路"，而是四条不同的研究管线。

%% 3. 发现的问题

% 3.1 重复工具函数（真正的冗余）

%   Haversine 距离公式 — 5 份重复实现:
%     权威: utils/sphere_utils_haversine_distance.m (102行)
%     副本1: ukf/ukf_jichu.m 内部 haversine_ukf (7行)
%     副本2: run_fragment_study.m 内部 haversine_km (8行)
%     副本3: tracker/track_matcher_dualgate.m 内部 haversine_km_local (13行)
%     副本4: evaluation/evaluate_all_multi.m 内部 haversine_km_eval (24行)
%     副本5: skywave_geometry.m 内部 geocentric_angle_impl (10行，Haversine 变体)
%
%   注意: skywave_geometry.m 的重复是受控的（内联优化，保持常量局部化），
%   其余 4 处副本没有正当理由。

%   regularize_cov 协方差正则化 — 3 份重复实现:
%     权威: fusion/regularize_cov.m (165行)
%     副本1: ukf/ukf_jichu.m 内部 regularize_cov_ukf (31行)
%     副本2: ukf/ukf_imm.m 内部 regularize_cov_imm (17行)
%     三者逻辑完全相同（对称化 + 特征值分解 + 双阈值裁剪 + 重构）

%   球面方位角 — 2 份重复:
%     权威: utils/sphere_utils_azimuth.m (107行)
%     副本: skywave_geometry.m 内部 azimuth_impl (10行)
%     受控重复（同 skywave_geometry 理由）

%   目的地点计算 — 2 份重复:
%     权威: utils/sphere_utils_destination_point.m (126行)
%     副本: ukf/ukf_jichu.m 内部 meas_to_latlon_ukf 中的球面正算 (10行)

% 3.2 空气墙文件

%   TRACK_MAIN_ORACLE.m (24行):
%     只做一层转发: [a,b,c,d,e] = Track_Process_for_HighRate_Oracle(...)
%     注释称"方便后续替换"但从未被替换过。

%   Fun_UpdateTrackforNoInputPoint_Oracle.m (43行):
%     注释自认"主循环中并不被直接调用"——所有航迹都走
%     Fun_UpdateTrackByAsscResult_Oracle 的纯预测路径。
%     作为"全空帧"快捷批量处理路径存在，但无人调用。

% 3.3 超大文件

%   evaluate_all_multi.m (754行):
%     包含两个完全独立的评估函数，各占 ~350 行。

%   ukf_jichu.m (624行):
%     包含 create/init/prepare/update/predict/measurement/sigma_points/
%     state_transition/CT模型/量测反解/协方差正则化/Haversine距离 — 12 个独立功能。

%   aircraft_trajectory_create.m (659行):
%     包含直线/拐弯/回头弯三种场景的创建逻辑。

%   plot_results_multi.m (639行):
%     单站绘图 + 融合绘图混在一起。

%   plot_tracks_without_fusion.m (586行):
%     与 plot_results_multi.m 有大量重叠。

%   run.m (660行):
%     主流程仅 ~80 行，其余 ~580 行全是辅助函数（打印/绘图/标定/生成）。

%   run_fragment_study.m (477行):
%     主流程仅 ~60 行，其余 ~417 行全是辅助函数。

% 3.4 命名风格不统一

%   Fun_ 前缀:  Fun_UpdateTrackByAsscResult_Oracle.m
%   track_ 前缀: track_matcher_dualgate.m
%   snake_case:  build_faded_track_segments.m
%   _Oracle 后缀: PointTrackAssociation_Oracle.m
%   整个 tracker/ 目录都是 Oracle 模式，后缀无区分意义。

% 3.5 数据流隐式耦合

%   每个模块输出巨型 struct（20+ 字段），下游只取需要的几个。
%   例如 result struct in run.m 有 20+ 字段，但 plot_oracle_figures
%   只用其中 5 个。修改一个模块的输出字段可能无声无息地破坏下游。

% 3.6 两套 IMM 实现

%   ukf/ukf_imm.m: 在线 IMM（CV+CT-left+CT-right），657行
%   bridge_smoother.m 内部: 离线 IMM（CV+CT-left+CT-right），内嵌在 smooth_gap 函数
%   两者共享 CT 模型公式但不共享代码。

%% 4. 重构方案

% 4.1 P0 — 立即可做，零风险（统一工具函数 + 删除空气墙）

%   操作 1: 删除 TRACK_MAIN_ORACLE.m
%     调用方改为直接调用 Track_Process_for_HighRate_Oracle.m
%     影响: run.m 中两处调用，run_without_fusion.m 中一处

%   操作 2: 删除 Fun_UpdateTrackforNoInputPoint_Oracle.m
%     确认无人调用后移除

%   操作 3: 统一 regularize_cov
%     删除 ukf_jichu.m 内部 regularize_cov_ukf
%     删除 ukf_imm.m 内部 regularize_cov_imm
%     将 fusion/regularize_cov.m 提升到根目录
%     两处内部调用改为调用根目录的 regularize_cov

%   操作 4: 统一 Haversine
%     删除 ukf_jichu.m 内部 haversine_ukf
%     删除 run_fragment_study.m 内部 haversine_km
%     删除 track_matcher_dualgate.m 内部 haversine_km_local
%     删除 evaluate_all_multi.m 内部 haversine_km_eval
%     统一调用 utils/sphere_utils_haversine_distance.m
%     注意单位转换：utils 返回米，部分调用方要公里（/1000）

% 4.2 P1 — 拆分大文件

%   操作 5: ukf_jichu.m 拆为 3 个文件
%     ukf_core.m (约 200行): create/init/prepare/update/predict/measurement
%     ukf_motion.m (约 80行): sigma_points, state_transition_ukf, state_transition_ct_ukf
%     ukf_meas.m (约 120行): measurement_ukf, meas_to_latlon_ukf, wrap_angle_ukf

%   操作 6: evaluate_all_multi.m 拆为 2 个文件
%     eval_tracking.m (~350行): compute_tracking_errors_multi + 辅助
%     eval_fusion.m (~350行): evaluate_fusion_multi + 辅助
%     保留 evaluate_all_multi.m (~50行) 作为调度器

%   操作 7: aircraft_trajectory_create.m 拆为 3 个文件
%     aircraft_trajectory_dispatcher.m (~50行): 根据场景名分发
%     aircraft_trajectory_straight.m (~100行): 直线场景
%     aircraft_trajectory_turn.m (~250行): 120° 拐弯
%     aircraft_trajectory_uturn.m (~260行): 180° 回头弯

%   操作 8: plot_results_multi.m + plot_tracks_without_fusion.m 重组
%     viz_plot_common.m (~200行): 公共绘图函数（地图轴、点迹绘制、航迹绘制）
%     viz_single_station.m (~250行): 单站绘图
%     viz_fusion.m (~250行): 融合绘图

%   操作 9: run.m 辅助函数提取
%     simulation/calibrate_bias_run.m (~66行)
%     simulation/generate_detections_run.m (~58行)
%     visualization/viz_plot_oracle.m (~33行)
%     visualization/viz_plot_helpers.m (~100行)
%     io/save_results.m (~10行)
%     utils/wrap_angle.m (~8行)
%     run.m 主流程保留 ~80 行

%   操作 10: run_fragment_study.m 辅助函数提取
%     simulation/manufacture_fragments.m (~100行)
%     evaluation/eval_fragment_fusion.m (~80行)
%     visualization/viz_fragment_study.m (~100行)
%     utils/merge_config.m (~10行)

% 4.3 P2 — 命名规范化

%   去掉 Fun_ 前缀:
%     Fun_UpdateTrackByAsscResult_Oracle.m → track_update_oracle.m
%     Fun_UpdateTrackforNoInputPoint_Oracle.m → (已删除)

%   去掉 _Oracle 后缀（整个 tracker/ 都是 Oracle 模式）:
%     PointTrackAssociation_Oracle.m → track_association_oracle.m
%     trackStarter_logic_oracle.m → track_starter_oracle.m
%     fun_create_new_track_oracle.m → track_create_oracle.m
%     fun_track_quality_management_and_info_completion_oracle.m → track_quality.m
%     fun_remove_assc_pts_from_pointlist_oracle.m → track_remove_assoc_points.m
%     sortTrackList_oracle.m → track_sort.m

%   统一 ukf/ 目录命名:
%     ukf_jichu.m → ukf_core.m
%     ukf_zishiying.m → ukf_adaptive.m
%     adapt_q.m → adaptive_q.m

%   统一其他命名:
%     post_init_multi.m → ukf_post_init.m
%     dual_threshold_decide.m → dual_threshold.m
%     build_faded_track_segments.m → track_segments.m

% 4.4 P3 — 文件移动

%   fusion/regularize_cov.m → 根目录 regularize_cov.m
%     理由: 被 ukf/、tracker/、fusion/ 三方共用

%   tracker/dual_threshold_decide.m → 根目录 dual_threshold.m
%     理由: 被 tracker/ 和 fusion/ 共用

%   io/extract_measurement_field.m → utils/extract_field.m
%     理由: 纯工具函数，utils 更合适

%% 5. 不动的文件（设计合理，无需修改）

%   ukf/ukf_dispatch.m (46行) — dispatcher 模式干净
%   ukf/ukf_imm.m (657行) — IMM 算法逻辑完整，内聚性好
%   ukf/radar_params.m (61行) — 职责单一
%   tracker/tracklet_grouping.m (476行) — 整数规划求解是核心创新
%   tracker/Track_Process_for_HighRate_Oracle.m (348行) — 6 阶段自然分割
%   tracker/run_oracle_tracker_sequence.m (65行) — 主循环入口
%   tracker/plan_controlled_fragmentation.m (494行) — 递归回溯搜索
%   tracker/track_matcher_dualgate.m (421行) — 双门限匹配
%   tracker/track_matcher.m (546行) — 旧版多特征匹配，服务于 run.m pipeline
%   fusion/time_align_tracks.m (206行) — 职责单一，逻辑清晰
%   fusion/bridge_smoother.m (367行) — 完整的 RTS+IMM 离线平滑系统
%   fusion/fuse_estimate_sequence.m (380行) — 四种融合算法调度器
%   fusion/run_track_fusion.m (43行) — 薄适配器
%   fusion/track_fusion_algorithms.m (601行) — 四种融合算法合集
%   simulation/prepare_oracle_tracking_inputs.m (215行) — 统一入口
%   simulation/build_truth_scenario.m (114行) — 薄而清晰
%   simulation/aircraft_trajectory_interpolate.m (170行) — 合理
%   simulation/aircraft_trajectory_locate.m (104行) — 合理
%   simulation/build_target_states_at_time.m (77行) — 合理
%   simulation/generate_frame_detections.m (230行) — 合理
%   simulation/generate_frame_detections_multi.m (233行) — 合理
%   simulation/bistatic_inverse_solver.m (119行) — 合理
%   simulation/radar_coverage_check.m (96行) — 合理
%   config/simulation_params_oracle.m (349行) — 参数集中定义
%   utils/skywave_geometry.m (180行) — action dispatcher 模式
%   utils/sphere_utils_haversine_distance.m (102行) — 权威实现
%   utils/sphere_utils_azimuth.m (107行) — 权威实现
%   utils/sphere_utils_destination_point.m (125行) — 权威实现
%   utils/sphere_utils_interpolate_great_circle.m (85行) — 合理
%   utils/sphere_utils_radial_velocity.m (130行) — 合理
%   utils/sphere_utils_seconds_to_datetime_str.m (72行) — 合理
%   utils/coord_systems_lla_to_ecef.m (49行) — 合理
%   validation/validate_oracle_invariants.m (432行) — 测试验证
%   validation/run_filter_math_tests.m (280行) — 测试
%   validation/run_fragment_fusion_tests.m (170行) — 测试
%   validation/run_oracle_lifecycle_tests.m (462行) — 测试
%   run_without_fusion.m (199行) — 结构清晰
%   run_random_fade_fragment_fusion.m (191行) — 结构清晰
%   visualization/plot_scene_overview.m (182行) — 合理
%   visualization/plot_scene_overview_multi.m (132行) — 合理
%   visualization/plot_point_cloud_3d.m (103行) — 合理
%   visualization/plot_fragment_study_dashboard.m (240行) — 合理

%% 6. 预期效果

%   最大文件行数: 从 754 行降至 ~350 行
%   重复工具函数: Haversine 5 份 → 1 份, regularize_cov 3 份 → 1 份
%   空气墙文件: 2 个 → 0
%   命名规范: Fun_/track_/snake_case 混用 → 全部 snake_case
%   入口脚本主流程: run.m 中混入 ~580 行辅助函数 → 主流程 ~80 行
%   文件总数: 65 → ~72（拆分后增加，但单文件更小更专注）

%% 7. 重构风险与注意事项

% 7.1 删除 TRACK_MAIN_ORACLE.m
%     风险: 低。调用方只有 run.m 和 run_without_fusion.m，
%     各有一处调用。改为直接调用 Track_Process_for_HighRate_Oracle.m 即可。
%     注意: 调用签名完全一致，无需修改参数。

% 7.2 删除 Fun_UpdateTrackforNoInputPoint_Oracle.m
%     风险: 极低。该文件注释自认"主循环中并不被直接调用"，
%     确认无人引用后移除。

% 7.3 统一 regularize_cov
%     风险: 低。三处实现完全相同（对称化 + eig + 双阈值裁剪 + 重构）。
%     注意: ukf_imm.m 中调用 regularize_cov_imm 的地方改为 regularize_cov。

% 7.4 统一 Haversine
%     风险: 低。utils 版本已是最权威实现。
%     注意: 单位差异。utils 返回米，以下调用方需要公里:
%       - run_fragment_study.m: haversine_km → 除以 1000
%       - track_matcher_dualgate.m: haversine_km_local → 除以 1000
%       - evaluate_all_multi.m: haversine_km_eval → 除以 1000

% 7.5 拆分 ukf_jichu.m
%     风险: 中。需要确保拆分后各文件间的内部函数调用关系正确。
%     注意: sigma_points_ukf 和 state_transition_ukf 被 prepare_ukf 调用，
%           应放在 ukf_motion.m 中。measurement_ukf 被 prepare_ukf 调用，
%           应放在 ukf_meas.m 中。

% 7.6 拆分 aircraft_trajectory_create.m
%     风险: 低。三种场景（直线/拐弯/回头弯）是自然分割。
%     注意: 保留 dispatcher 文件，内部根据场景名分发到三个子文件。

% 7.7 命名重命名
%     风险: 中。需要全局搜索所有引用并更新。
%     建议: 使用 MATLAB 的"重命名符号"功能（如果 IDE 支持），
%           或全局 grep 替换。

%% 8. 实施顺序建议

%   第一阶段（当天完成，零风险）:
%     1. 删除 TRACK_MAIN_ORACLE.m
%     2. 删除 Fun_UpdateTrackforNoInputPoint_Oracle.m
%     3. 统一 regularize_cov
%     4. 统一 Haversine
%     5. 运行所有测试验证

%   第二阶段（半天完成）:
%     6. 命名规范化（重命名文件 + 更新引用）
%     7. 文件移动（regularize_cov 提升到根目录）

%   第三阶段（一天完成）:
%     8. 拆分 ukf_jichu.m
%     9. 拆分 evaluate_all_multi.m
%    10. 拆分 aircraft_trajectory_create.m

%   第四阶段（按需）:
%    11. 拆分 plot_results_multi.m + plot_tracks_without_fusion.m
%    12. 提取 run.m 和 run_fragment_study.m 的辅助函数
