% =========================================================================
% simulation_params_oracle.m — Oracle 链路唯一参数配置入口
% =========================================================================
%
% 【功能】
%   双基地 OTH-SWR 单/多目标跟踪仿真的全部参数。所有数值在此一处定义，
%   不再有上游 config 文件，入口脚本（run.m / run_without_fusion.m）不
%   覆盖任何参数。
%
% 【UKF 参数挑选约定】
%   两台雷达异质（R1 精密 / R2 标准），噪声/Q/门限按 radar1_*/radar2_*
%   分别存放。创建 UKF 前必须调 ukf/radar_params.m 把指定雷达的数值
%   抄到通用字段名（ukf_*、gate_sigma、gate_vr_ms、tracker_K_loss）下：
%       params_r = radar_params(simulation_params_oracle(), 1);
%       ukf_imm('create', params_r, ...);
%
% 【硬约束】
%   detection_probability = 0.6、false_alarm_rate = 0.001 是雷达特性，
%   不可调整。Oracle 只能挑选已生成的检测，不能凭空造点或写真值速度。
% =========================================================================

function params = simulation_params_oracle()

% ---- 时间设定 ----
params.dt_sec                   = 30.0;    % 采样周期（s）
params.time_offset_radar1_sec   = 0.0;     % R1 起始偏移
params.time_offset_radar2_sec   = 13.0;    % R2 比 R1 晚 13s（异步采样）

% ---- 雷达接收/发射站几何（deg，WGS84）----
params.radar1_lon     = 113.0;  params.radar1_lat     = 33.5;
params.radar2_lon     = 115.0;  params.radar2_lat     = 33.0;
params.radar1_tx_lon  = 109.0;  params.radar1_tx_lat  = 33.5;   % R1 Tx-Rx 基线 ~370km
params.radar2_tx_lon  = 111.0;  params.radar2_tx_lat  = 33.0;

% ---- 威力覆盖 ----
params.radar1_beam_center_deg = 92.0;      % R1 波束指向（正东略偏南）
params.radar2_beam_center_deg = 91.0;
params.beam_width_deg         = 15.0;      % 3dB 波束宽度
params.range_min_km = 1000.0;               % OTH-SWR 单跳最小地面距离
params.range_max_km = 2000.0;
params.range_min_m  = params.range_min_km * 1000;
params.range_max_m  = params.range_max_km * 1000;

% ---- 目标航路（直线段；转弯/U 形场景由 build_truth_scenario 覆盖）----
params.aircraft_waypoints = [127.5, 31.0, 0.0; 130.5, 33.0, 0.0];
params.aircraft_speed_ms  = 230.0;          % 民航巡航速度 ~Ma0.78

% ---- 量测噪声（异质传感器：R2 噪声约 2× R1）----
params.radar1_range_noise_std_m     = 10000.0;  params.radar1_azimuth_noise_std_deg = 0.35;
params.radar2_range_noise_std_m     = 20000.0;  params.radar2_azimuth_noise_std_deg = 0.60;
params.radial_vel_noise_std_ms      = 0.5;      % 多普勒噪声两雷达共用

% ---- 系统偏差（R1 测远偏西、R2 测近偏东，便于融合交叉校验）----
params.radar1_range_bias_m     =  20000.0;  params.radar1_azimuth_bias_deg = -3.0;
params.radar2_range_bias_m     = -15000.0;  params.radar2_azimuth_bias_deg =  3.5;

% ---- UKF 过程噪声 / 初始协方差 / 关联门限 / K_loss（雷达专属）----
params.radar1_ukf_Q_scale   = 1e5;  params.radar2_ukf_Q_scale   = 2e5;
params.radar1_ukf_P_pos_std = 0.05; params.radar2_ukf_P_pos_std = 0.05;   % deg
params.radar1_ukf_P_vel_std = 0.004; params.radar2_ukf_P_vel_std = 0.005; % deg/s
params.radar1_gate_sigma    = 6;    params.radar2_gate_sigma    = 6;      % 马氏距离 sigma 倍数
params.radar1_gate_vr_ms    = 20;   params.radar2_gate_vr_ms    = 40;     % Vr 硬门
params.radar1_tracker_K_loss = 8;   params.radar2_tracker_K_loss = 8;     % 连续漏检终止帧数
params.tracker_K_loss        = 8;   % 顶层默认（未经 radar_params 时使用）

% ---- UKF 共用 UT 参数 ----
params.ukf_alpha = 1e-2;            % Sigma 点散布度
params.ukf_beta  = 2.0;             % 先验分布参数（高斯最优）
params.ukf_kappa = 0.0;
params.ukf_rv_std_ms = params.radial_vel_noise_std_ms;   % 多普勒量测噪声（共用，不被 radar_params 覆盖）

% ---- IMM 双模型（CV=常速 / CT=恒转弯）----
params.imm_Pi_CV_to_CT = 0.001;     % 模型转移概率（直线交叉场景，平均 1000 帧切一次）
params.imm_Pi_CT_to_CV = 0.001;
params.imm_adapt_mode  = '3in1';    % 3in1 = CV 瞬态增益 + CT 固定高机动 + IMM 慢概率融合
params.imm_slow_Pi_CV_to_CT = 0.03; % 3in1 慢变 IMM 转移概率
params.imm_slow_Pi_CT_to_CV = 0.03;
params.imm_ct_fixed_Q_scale = 1.8;  % CT 模型固定 Q 倍率
params.imm_transient_nis_start   = 3.0;   % CV 瞬态增益触发 NIS
params.imm_transient_nis_full    = 12.0;  % CV 瞬态增益满量程 NIS
params.imm_transient_gain_max    = 5.0;   % CV 最大增益倍率
params.imm_transient_ewma_alpha  = 0.65;  % 短时 NIS EWMA

% ---- 自适应 Q / 模糊（部分由 adapt_q.m isfield 守卫读取，保守保留）----
params.use_fuzzy_adaptive = true;
params.fuzzy_window_size  = 3;      % NIS 滑动窗口（Fun_UpdateTrackByAsscResult_Oracle 用）
params.fuzzy_ema_eta      = 0.10;   % adapt_q 模糊路径 EMA
params.adaptive_Q_min     = 0.5;    % adapt_q 自适应 Q 下限
params.adaptive_Q_max     = 4.0;    % adapt_q 自适应 Q 上限
params.maneuver_ema_eta   = 0.50;   % adapt_q 机动路径 EMA
params.pda_pd_gate        = 0.8647; % 门内检测概率（赋给 imm.Pg）

% ---- Oracle 起始 3/7 滑窗（严格物理帧）----
params.oracle_QUALIFY_NUM   = 3;    % 窗口内最少真实命中数
params.oracle_TOLERANT_NUM  = 7;    % 窗口跨度（最近 7 个物理帧）
params.oracle_confirm_quality    = 8;
params.oracle_maintain_quality  = 4;
params.oracle_max_quality       = 15;
params.oracle_loss_quality_penalty = 1;
params.oracle_truth_terminate_enable = true;   % 真值结束后航迹转 HISTORY；false 则只能 K_loss 终止

% ---- 航迹状态常量（南阳式）----
params.RELIABLE_TRACK  = 1;
params.MAINTAIN_TRACK  = 2;
params.TEMPORARY_TRACK = 6;
params.HISTORY_TRACK   = 7;

% ---- 雷达硬约束（不可调整）----
params.detection_probability = 0.6;
params.false_alarm_rate      = 0.001;
range_resolution_km     = 10.0;     % 内部用：距离分辨率
azimuth_resolution_deg  = 1.0;      % 内部用：方位分辨率
params.n_resolution_cells = ...
    ((params.range_max_km - params.range_min_km) / range_resolution_km) * ...
    (params.beam_width_deg / azimuth_resolution_deg);

% ---- 跨雷达航迹匹配（双门限法，run.m 用）----
params.track_matcher_method      = 'dualgate';
params.dualgate_T1_km            = 35;       % 第一门限：距离粗筛
params.dualgate_M                = 8;        % 第二门限：连续帧数
params.dualgate_var_km2          = 50;       % 方差校验阈值
params.dualgate_coexist_thresh   = 5;        % 最少共现帧数
params.dualgate_mutual_exclusion = true;     % 互斥后处理

% ---- 数据源 / 随机种子 ----
params.adsb_csv_path = '2026-04-27 09-30-00.csv';
params.random_seed   = 94;
end
