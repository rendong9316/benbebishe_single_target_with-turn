% =========================================================================
% simulation_params_oracle.m — Oracle 链路唯一参数配置入口
% =========================================================================
%
% 【功能概述】
%   本文件集中定义了双基地 OTH-SWR（超视距表面波雷达）单/多目标跟踪仿真
%   的全部参数。设计原则是"一处定义，处处生效"——所有数值在这里修改，
%   入口脚本（run.m / run_without_fusion.m）不再覆盖任何参数。
%
% 【架构变更说明】
%   此前参数分散在上游 config 文件和各入口脚本中，导致修改困难、行为不一致。
%   现在所有参数统一收敛到此函数，入口脚本直接调用 simulation_params_oracle()
%   获取完整参数结构体，不再做局部覆盖。
%
% 【UKF 参数挑选约定】
%   两台雷达硬件异质（R1 精密型 / R2 标准型），噪声、过程噪声、关联门限
%   等参数按雷达分别存储（radar1_xxx / radar2_xxx）。在创建 UKF 实例之前，
%   必须先调用 ukf/radar_params.m 将指定雷达的专属参数抄写到通用字段名下：
%       params_r = radar_params(simulation_params_oracle(), 1);  % 取 R1 参数
%       ukf_imm('create', params_r, ...);                         % 传入 UKF
%   这样做的原因是 UKF 内部使用统一的字段名（ukf_Q_scale、gate_sigma 等），
%   radar_params.m 负责将雷达专属字段映射到通用字段。
%
% 【硬约束声明】
%   detection_probability = 0.6 和 false_alarm_rate = 0.001 是雷达的物理特性
%   参数，在整个仿真中不可调整。Oracle 起始器只能从已生成的检测中进行挑选，
%   不能凭空造点（那等于假设 Pd=1.0，违背物理现实），也不能写真值速度。
%
% =========================================================================

function params = simulation_params_oracle()

    % ==================== 时间设定 ====================
    % 采样周期：每 30 秒一帧（典型 OTH-SWR 刷新率）
    params.dt_sec                   = 30.0;
    % R1 从 t=0 开始采样
    params.time_offset_radar1_sec   = 0.0;
    % R2 比 R1 晚 13 秒开始，形成异步采样（增加融合难度，更贴近实际）
    params.time_offset_radar2_sec   = 13.0;

    % ==================== 雷达站几何坐标（WGS84 经纬度）====================
    % R1 接收站位置
    params.radar1_lon     = 113.0;  params.radar1_lat     = 33.5;
    % R2 接收站位置
    params.radar2_lon     = 115.0;  params.radar2_lat     = 33.0;
    % R1 发射站位置（与接收站构成双基地几何）
    % R1 Tx-Rx 基线约 370 km（经度差 4° ≈ 370 km）
    params.radar1_tx_lon  = 109.0;  params.radar1_tx_lat  = 33.5;
    % R2 发射站位置
    params.radar2_tx_lon  = 111.0;  params.radar2_tx_lat  = 33.0;

    % ==================== 雷达波束覆盖参数 ====================
    % R1 波束指向：正东略偏南（92°）
    params.radar1_beam_center_deg = 92.0;
    % R2 波束指向：91°
    params.radar2_beam_center_deg = 91.0;
    % 3dB 波束宽度：15°
    params.beam_width_deg         = 15.0;
    % OTH-SWR 单跳的最小/最大地面距离（km）
    % 小于 1000km 的信号被地物杂波淹没，大于 2000km 信号衰减过大
    params.range_min_km = 1000.0;
    params.range_max_km = 2000.0;
    % 转换为米制单位供滤波器使用
    params.range_min_m  = params.range_min_km * 1000;
    params.range_max_m  = params.range_max_km * 1000;

    % ==================== 目标航路定义 ====================
    % 航路由起点和终点两个航路点定义（直线段飞行）
    % 格式：[经度, 纬度, 高度(km)]
    % 起点 (127.5, 31.0)，终点 (130.5, 33.0)，形成东北方向航迹
    params.aircraft_waypoints = [127.5, 31.0, 0.0; 130.5, 33.0, 0.0];
    % 民航巡航速度约 Ma0.78 ≈ 230 m/s
    params.aircraft_speed_ms  = 230.0;
    % 注意：转弯/U 形场景由 build_truth_scenario 函数覆盖此参数

    % ==================== 量测噪声模型（异质传感器）====================
    % R1 是精密雷达，R2 噪声约为 R1 的 2 倍
    % 距离噪声标准差
    params.radar1_range_noise_std_m     = 10000.0;  % R1: 10 km
    params.radar2_range_noise_std_m     = 20000.0;  % R2: 20 km
    % 方位角噪声标准差（度）
    params.radar1_azimuth_noise_std_deg = 0.35;       % R1: 0.35°
    params.radar2_azimuth_noise_std_deg = 0.60;       % R2: 0.60°
    % 径向速度（多普勒）噪声标准差（m/s），两雷达共用
    params.radial_vel_noise_std_ms      = 0.5;

    % ==================== 系统偏差（偏置）====================
    % R1 测距偏大 20km（测远偏西），方位偏小 -3°
    % R2 测距偏小 15km（测近偏东），方位偏大 +3.5°
    % 这种反向偏差设计使得融合时交叉校验更有效
    params.radar1_range_bias_m     =  20000.0;
    params.radar1_azimuth_bias_deg = -3.0;
    params.radar2_range_bias_m     = -15000.0;
    params.radar2_azimuth_bias_deg =  3.5;

    % ==================== UKF 过程噪声 / 初始协方差 / 关联门限 ====================
    % 每行格式：[雷达专属字段, 通用字段（未经 radar_params 时使用）]
    % 过程噪声 Q 的倍率：R2 更大（噪声大所以需要更多过程噪声来跟踪机动）
    params.radar1_ukf_Q_scale   = 1e5;
    params.radar2_ukf_Q_scale   = 2e5;
    % 初始位置协方差标准差（度）
    params.radar1_ukf_P_pos_std = 0.05;
    params.radar2_ukf_P_pos_std = 0.05;
    % 初始速度协方差标准差（度/秒）
    params.radar1_ukf_P_vel_std = 0.004;
    params.radar2_ukf_P_vel_std = 0.005;
    % 马氏距离关联门限的 sigma 倍数（6σ 覆盖 99.7% 的高斯分布）
    params.radar1_gate_sigma    = 6;
    params.radar2_gate_sigma    = 6;
    % 速度硬门限（Vr 超过此值直接拒绝关联，单位 m/s）
    params.radar1_gate_vr_ms    = 20;
    params.radar2_gate_vr_ms    = 40;
    % 连续漏检阈值：达到此帧数后航迹终止
    params.radar1_tracker_K_loss = 8;
    params.radar2_tracker_K_loss = 8;
    % 顶层默认值（如果不调用 radar_params 而直接用 params 时的后备值）
    params.tracker_K_loss        = 8;

    % ==================== UKF 共用 UT（Unscented Transform）参数 ====================
    % alpha：Sigma 点散布度参数，越小分布越集中
    params.ukf_alpha = 1e-2;
    % beta：先验分布参数，对于高斯分布 beta=2 是最优的
    params.ukf_beta  = 2.0;
    % kappa：二次精度参数，设为 0
    params.ukf_kappa = 0.0;
    % 多普勒量测噪声标准差（共用，不被 radar_params 覆盖）
    params.ukf_rv_std_ms = params.radial_vel_noise_std_ms;

    % ==================== IMM 双模型（CV/CT）参数 ====================
    % CV = Constant Velocity（常速模型），CT = Constant Turn（恒转弯模型）
    % 模型转移概率：直线飞行时极少切换到转弯模型
    params.imm_Pi_CV_to_CT = 0.001;     % CV→CT 转移概率（平均 1000 帧切换一次）
    params.imm_Pi_CT_to_CV = 0.001;     % CT→CV 转移概率
    % 自适应模式选择：'3in1' = CV 瞬态增益 + CT 固定高机动 + IMM 慢概率融合
    params.imm_adapt_mode  = '3in1';
    % 3in1 模式的慢变转移概率（比快速模式更平滑）
    params.imm_slow_Pi_CV_to_CT = 0.03;
    params.imm_slow_Pi_CT_to_CV = 0.03;
    % CT 模型的固定过程噪声倍率（高机动时增大 Q）
    params.imm_ct_fixed_Q_scale = 1.8;
    % CV 瞬态增益触发/满量程的 NIS（归一化创新平方）阈值
    params.imm_transient_nis_start   = 3.0;
    params.imm_transient_nis_full    = 12.0;
    % CV 最大增益倍率（NIS 大时临时增大观测权重）
    params.imm_transient_gain_max    = 5.0;
    % 短时 NIS 的 EWMA（指数加权移动平均）系数
    params.imm_transient_ewma_alpha  = 0.65;

    % ==================== 自适应 Q / 模糊控制参数 ====================
    % 启用模糊自适应 Q 估计
    params.use_fuzzy_adaptive = true;
    % NIS 滑动窗口大小（Fun_UpdateTrackByAsscResult_Oracle 中使用）
    % 用最近 3 帧的 NIS 值做模糊推理
    params.fuzzy_window_size  = 3;
    % 模糊路径的 EMA 系数（adapt_q.m 中使用）
    params.fuzzy_ema_eta      = 0.10;
    % 自适应 Q 的下限/上限（防止 Q 过小导致发散、过大导致噪声放大）
    params.adaptive_Q_min     = 0.5;
    params.adaptive_Q_max     = 4.0;
    % 机动路径的 EMA 系数（检测到机动时快速调整 Q）
    params.maneuver_ema_eta   = 0.50;
    % PDA（概率数据关联）门内检测概率
    % 对应门限下的理论 Pd，赋给 imm.Pg
    params.pda_pd_gate        = 0.8647;

    % ==================== Oracle 起始器滑窗参数 ====================
    % 3/7 滑窗起始规则：在最近 7 个物理帧中，至少有 3 帧有真实命中才确认航迹
    params.oracle_QUALIFY_NUM   = 3;    % 窗口内最少真实命中帧数
    params.oracle_TOLERANT_NUM  = 7;    % 窗口跨度（最近的 7 个物理帧）
    % 航迹质量参数（南阳式管理）
    params.oracle_confirm_quality    = 8;   % 确认时赋予的质量分
    params.oracle_maintain_quality  = 4;    % 维持时的质量分
    params.oracle_max_quality       = 15;   % 质量上限
    params.oracle_loss_quality_penalty = 1; % 漏检时的质量扣分

    % ==================== 真值终止开关 ====================
    % true: 真值轨迹结束后，航迹转为 HISTORY 状态（保留记录但不参与跟踪）
    % false: 只能靠 K_loss 连续漏检来终止航迹
    params.oracle_truth_terminate_enable = true;

    % ==================== 航迹状态常量（南阳式分类）====================
    params.RELIABLE_TRACK  = 1;   % 可靠航迹（质量达标，可用于决策）
    params.MAINTAIN_TRACK  = 2;   % 维持航迹（质量尚可但未达可靠级别）
    params.TEMPORARY_TRACK = 6;   % 临时航迹（新起始，还在验证中）
    params.HISTORY_TRACK   = 7;   % 历史航迹（已终止，保留归档）

    % ==================== 雷达硬约束（不可调整）====================
    % Pd = 0.6：雷达检测概率，物理特性决定
    params.detection_probability = 0.6;
    % Pfa = 0.001：虚警率，物理特性决定
    params.false_alarm_rate      = 0.001;

    % ==================== 内部计算用分辨率参数 ====================
    % 距离分辨率：10 km（内部用，不暴露给外部配置）
    range_resolution_km     = 10.0;
    % 方位分辨率：1°（内部用）
    azimuth_resolution_deg  = 1.0;
    % 计算分辨单元数量（range_bins × angle_bins）
    % 用于杂波密度估计等内部计算
    params.n_resolution_cells = ...
        ((params.range_max_km - params.range_min_km) / range_resolution_km) * ...
        (params.beam_width_deg / azimuth_resolution_deg);

    % ==================== 跨雷达航迹匹配参数（双门限法）====================
    % 匹配方法：dualgate（双门限匹配）
    params.track_matcher_method      = 'dualgate';
    % 第一门限：空间距离粗筛（35 km 内才进入下一步）
    params.dualgate_T1_km            = 35;
    % 第二门限：连续帧数要求（8 帧都满足距离条件）
    params.dualgate_M                = 8;
    % 方差校验阈值（50 km²，排除偶然接近的假匹配）
    params.dualgate_var_km2          = 50;
    % 最少共现帧数（两雷达同时覆盖的帧数下限）
    params.dualgate_coexist_thresh   = 5;
    % 互斥后处理：一个 R1 航迹最多匹配一个 R2 航迹
    params.dualgate_mutual_exclusion = true;

    % ==================== 数据源与随机种子 ====================
    % ADS-B 原始数据 CSV 文件路径（用于生成真值轨迹）
    params.adsb_csv_path = '2026-04-27 09-30-00.csv';
    % 随机种子：94，确保仿真结果可复现
    params.random_seed   = 94;
end
