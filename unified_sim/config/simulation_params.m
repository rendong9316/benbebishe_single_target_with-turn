% =========================================================================
% simulation_params.m — 统一仿真参数配置（单/多目标通用）
% =========================================================================
%
% 【功能概述】
%   本文件是统一仿真系统的唯一参数配置入口。支持：
%     - 单目标（n_targets=1）直线/拐弯/180°回头弯场景
%     - 多目标（n_targets=3）三目标交叉场景
%     - 多种 UKF 后端（基础 UKF / 自适应 UKF / IMM CV+CT）
%
% 【参数设计原则】
%   - 异质噪声：R1 为精度站（低噪声），R2 为标准站（约 2 倍噪声）
%   - 系统偏差：R1 和 R2 的偏差符号相反，模拟雷达标定误差
%   - 时间异步：R2 采样时刻比 R1 晚 13 秒
%
% 【调用关系】
%   被调用方: run_sim.m / run_mc.m（统一仿真入口）
% =========================================================================

function params = simulation_params()

% =====================================================================
% 模块1: 时间设定
% =====================================================================
params.dt_sec = 30.0;
params.duration_sec = 3600.0;
params.ref_start_time = datetime(2026, 4, 27, 9, 30, 0);
params.time_offset_radar1_sec = 0.0;
params.time_offset_radar2_sec = 13.0;

% =====================================================================
% 模块2: 站点几何位置（经纬度，单位：度，WGS84）
% =====================================================================
params.radar1_lon = 113.0;   params.radar1_lat = 33.5;
params.radar2_lon = 115.0;   params.radar2_lat = 33.0;
params.radar1_tx_lon = 109.0;  params.radar1_tx_lat = 33.5;
params.radar2_tx_lon = 111.0;  params.radar2_tx_lat = 33.0;

% =====================================================================
% 模块3: 雷达威力覆盖参数
% =====================================================================
params.radar1_beam_center_deg = 92.0;
params.radar2_beam_center_deg = 91.0;
params.beam_width_deg = 15.0;
params.range_min_km = 1000.0;
params.range_max_km = 2000.0;
params.range_min_m = params.range_min_km * 1000;
params.range_max_m = params.range_max_km * 1000;

% =====================================================================
% 模块4: 目标航迹参数
% =====================================================================
% 航路点数组: N×2 矩阵 [lon, lat]
% 默认单目标直线场景
params.aircraft_waypoints = [127.5, 31.0; 130.5, 33.0];
params.aircraft_speed_ms = 230.0;

% 场景模式: 'straight' | 'gradual_turn' | 'uturn' | 'multi'
params.scenario = 'uturn';

% 航迹模式: 由 apply_scenario_params 按 scenario 派生
params.trajectory_mode = 'uturn';

% 目标数量: 由 apply_scenario_params 按 scenario 派生
params.n_targets = 1;

% =====================================================================
% 模块5: 量测噪声参数（异质传感器）
% =====================================================================
params.radar1_range_noise_std_m = 7000.0;
params.radar1_azimuth_noise_std_deg = 0.35;
params.radar2_range_noise_std_m = 14000.0;
params.radar2_azimuth_noise_std_deg = 0.6;
params.radial_vel_noise_std_ms = 0.5;

% UKF 滤波器内部使用的量测噪声参数（默认值）
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_rv_std_ms = params.radial_vel_noise_std_ms;

% 通用 Q_scale 默认值（雷达专属值见 radar1_ukf_Q_scale / radar2_ukf_Q_scale）
params.ukf_Q_scale = 1e5;  % 默认 R1 值，运行时由主入口覆盖

% 通用初始协方差默认值（雷达专属值见 radar1/2_ukf_P_*）
params.ukf_P_pos_std = 0.05;
params.ukf_P_vel_std = 0.004;

% =====================================================================
% 模块6: 系统偏差参数（模拟雷达标定误差）
% =====================================================================
params.radar1_range_bias_m = 20000.0;
params.radar1_azimuth_bias_deg = -3.0;
params.radar2_range_bias_m = -15000.0;
params.radar2_azimuth_bias_deg = 3.5;

% =====================================================================
% 模块7: UKF 参数
% =====================================================================
% 7.1 UT 三参数
params.ukf_alpha = 1e-2;
params.ukf_beta = 2.0;
params.ukf_kappa = 0.0;

% 7.2 过程噪声 Q（雷达专属）
params.radar1_ukf_Q_scale = 1e5;
params.radar2_ukf_Q_scale = 2e5;

% 7.3 初始状态协方差 P（雷达专属）
params.radar1_ukf_P_pos_std = 0.05;
params.radar1_ukf_P_vel_std = 0.004;
params.radar2_ukf_P_pos_std = 0.05;
params.radar2_ukf_P_vel_std = 0.005;

% 7.4 关联波门（雷达专属）
params.radar1_gate_sigma = 6;
params.radar2_gate_sigma = 6;
params.radar1_gate_vr_ms = 20;
params.radar2_gate_vr_ms = 40;

% 7.5 模糊自适应 Q
params.use_fuzzy_adaptive = true;
params.fuzzy_window_size = 3;
params.fuzzy_ema_eta = 0.10;
params.adaptive_Q_min = 0.5;
params.adaptive_Q_max = 4.0;

% 7.6 机动自适应 Q
params.maneuver_ema_eta = 0.50;

% 7.61 IMM 自适应模式
params.imm_adapt_mode = '3in1';
params.imm_transient_nis_start = 3.0;
params.imm_transient_nis_full = 12.0;
params.imm_transient_gain_max = 5.0;
params.imm_transient_ewma_alpha = 0.65;
params.imm_ct_fixed_Q_scale = 1.8;
params.imm_slow_Pi_CV_to_CT = 0.03;
params.imm_slow_Pi_CT_to_CV = 0.03;

% 7.7 航迹管理（雷达专属）
params.use_truth_init = true;
params.radar1_tracker_K_loss = 8;
params.radar2_tracker_K_loss = 8;

% =====================================================================
% 模块8: 航迹管理参数
% =====================================================================
params.tracker_M = 4;
params.tracker_N = 8;
params.gate_sigma = 2.5;

% =====================================================================
% 模块9: 检测概率与虚警参数
% =====================================================================
% 单目标: Pd=0.6（真实检测概率）
% 多目标: Pd=1.0（确保交叉区不丢失）
params.detection_probability = 0.6;
params.false_alarm_rate = 0.001;
params.range_resolution_km = 10.0;
params.azimuth_resolution_deg = 1.0;
params.n_resolution_cells = ...
    ((params.range_max_km - params.range_min_km) / params.range_resolution_km) * ...
    (params.beam_width_deg / params.azimuth_resolution_deg);

% =====================================================================
% 模块10: PDA 参数
% =====================================================================
params.pda_pd_gate = 0.8647;
params.pda_clutter_intensity = 1.5 / (2000e3 * 15);

% =====================================================================
% 模块11: 多目标 JPDA 参数
% =====================================================================
params.jpda_geo_gate_m_initial = 160000;
params.jpda_geo_gate_m_stable = 90000;
params.jpda_geo_gate_m_missed_step = 20000;
params.jpda_max_hypotheses = 5000;
params.jpda_min_update_prob = 0.05;
params.jpda_vr_gate_ms = 60;
params.jpda_star_enable = true;
params.motion_gate_margin_m = 25000;
params.motion_gate_max_m = 60000;

% =====================================================================
% 模块12: 多目标 M/N 起始参数
% =====================================================================
params.multi_truth_init_enable = true;
params.multi_truth_init_gate_m = 120000;
params.multi_truth_init_quality = 12;
params.multi_start_M = 3;
params.multi_start_N = 4;
params.multi_start_max_gap_frames = 2;
params.multi_start_max_misses = 2;
params.multi_start_min_speed_ms = 80;
params.multi_start_max_speed_ms = 380;
params.multi_start_heading_gate_deg = 60;
params.multi_start_initial_quality = 5;
params.multi_start_used_prob_threshold = 0.35;
params.multi_duplicate_gate_m = 50000;
params.multi_prune_duplicate_gate_m = 10000;
params.multi_prune_protect_life = 8;
params.multi_fallback_geo_gate_m = 90000;

% =====================================================================
% 模块13: 多目标航迹质量参数
% =====================================================================
params.multi_confirm_quality = 8;
params.multi_maintain_quality = 4;
params.tracker_K_loss = 15;
params.multi_truth_reinit_enable = false;
params.multi_truth_terminate_enable = true;

% =====================================================================
% 模块14: 跨雷达航迹匹配参数
% =====================================================================
params.track_matcher_method = 'dualgate';
params.dualgate_T1_km = 35;
params.dualgate_M = 8;
params.dualgate_var_km2 = 50;
params.dualgate_coexist_thresh = 5;
params.dualgate_mutual_exclusion = true;

% =====================================================================
% 模块15: IMM 模型转移概率
% =====================================================================
params.imm_Pi_CV_to_CT = 0.20;
params.imm_Pi_CT_to_CV = 0.20;

% =====================================================================
% 模块16: ADS-B 标定数据路径
% =====================================================================
params.adsb_csv_path = '2026-04-27 09-30-00.csv';

% =====================================================================
% 模块17: 随机种子
% =====================================================================
params.random_seed = 94;

end
