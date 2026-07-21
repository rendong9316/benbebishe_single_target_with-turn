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
    % 函数入口：返回完整的仿真参数结构体 params
    % 调用方式：p = simulation_params_oracle();
    % 入口脚本（run.m / run_without_fusion.m）直接调用此函数获取参数，
    % 不再做任何局部覆盖，保证"一处定义，处处生效"。

    % ==================== 时间设定 ====================
    % 采样周期：每 30 秒一帧（典型 OTH-SWR 刷新率）
    params.dt_sec                   = 30.0;
    % R1 从 t=0 开始采样
    params.time_offset_radar1_sec   = 0.0;
    % R2 比 R1 晚 13 秒开始，形成异步采样（增加融合难度，更贴近实际）
    params.time_offset_radar2_sec   = 13.0;

    % ==================== 雷达站几何坐标（WGS84 经纬度）====================
    % R1 接收站位置（经度 113.0E, 纬度 33.5N，位于湖北境内）
    params.radar1_lon     = 113.0;  params.radar1_lat     = 33.5;
    % R2 接收站位置（经度 115.0E, 纬度 33.0N，位于河南境内，与 R1 相距约 200km）
    params.radar2_lon     = 115.0;  params.radar2_lat     = 33.0;
    % R1 发射站位置（与 R1 接收站构成双基地几何，经度差 4° 即约 370km 基线）
    % R1 Tx-Rx 基线约 370 km（经度差 4° ≈ 370 km）
    params.radar1_tx_lon  = 109.0;  params.radar1_tx_lat  = 33.5;
    % R2 发射站位置（与 R2 接收站同纬度，经度 111.0E，基线约 220km）
    params.radar2_tx_lon  = 111.0;  params.radar2_tx_lat  = 33.0;

    % ==================== 雷达波束覆盖参数 ====================
    % R1 波束指向：正东略偏南（92°，从正北顺时针度量）
    params.radar1_beam_center_deg = 92.0;
    % R2 波束指向：91°，与 R1 基本平行，覆盖区域有重叠
    params.radar2_beam_center_deg = 91.0;
    % 3dB 波束宽度：15°（波束半功率点宽度）
    params.beam_width_deg         = 15.0;
    % OTH-SWR 单跳的最小/最大地面距离（km）
    % 小于 1000km 的信号被地物杂波淹没，大于 2000km 信号衰减过大
    params.range_min_km = 1000.0;
    params.range_max_km = 2000.0;
    % 转换为米制单位供滤波器使用（1km = 1000m）
    params.range_min_m  = params.range_min_km * 1000;
    params.range_max_m  = params.range_max_km * 1000;

    % ==================== 目标航路定义 ====================
    % 航路由起点和终点两个航路点定义（直线段飞行，转弯场景由 build_truth_scenario 覆盖）
    % 格式：[经度, 纬度, 高度(km)]，高度 0 表示地面/海平面
    % 起点 (127.5, 31.0) 位于南海北部，终点 (130.5, 33.0) 位于东海海域
    params.aircraft_waypoints = [127.5, 31.0, 0.0; 130.5, 33.0, 0.0];
    % 民航巡航速度约 Ma0.78 ≈ 230 m/s（高空标准大气条件下）
    params.aircraft_speed_ms  = 230.0;
    params.truth_turn_rate_deg_per_sec = 1.0;
    % 注意：转弯/U 形场景由 build_truth_scenario 函数覆盖此参数

    % ==================== 量测噪声模型（异质传感器）====================
    % R1 是精密雷达，R2 噪声约为 R1 的 2 倍（硬件异质性建模）
    % 距离噪声标准差（R1: 10km, R2: 20km，典型 OTH-SWR 测距精度）
    params.radar1_range_noise_std_m     = 10000.0;  % R1: 10 km
    params.radar2_range_noise_std_m     = 20000.0;  % R2: 20 km
    % 方位角噪声标准差（度）（R1: 0.35°, R2: 0.60°）
    params.radar1_azimuth_noise_std_deg = 0.35;       % R1: 0.35°
    params.radar2_azimuth_noise_std_deg = 0.60;       % R2: 0.60°
    % 径向速度（多普勒）噪声标准差（m/s），两雷达共用
    % OTH-SWR 多普勒测量精度较高，典型值 0.5m/s
    params.radial_vel_noise_std_ms      = 0.5;

    % ==================== 系统偏差（偏置）====================
    % R1 测距偏大 20km（意味着测到的距离比实际远，等效于目标偏西）
    % R1 方位偏小 -3°（测到的方位角比实际小）
    % R2 测距偏小 15km（测到的距离比实际近，等效于目标偏东）
    % R2 方位偏大 +3.5°（测到的方位角比实际大）
    % 这种反向偏差设计使得融合时交叉校验更有效（两雷达误差方向相反）
    params.radar1_range_bias_m     =  20000.0;
    params.radar1_azimuth_bias_deg = -3.0;
    params.radar2_range_bias_m     = -15000.0;
    params.radar2_azimuth_bias_deg =  3.5;

    % ==================== UKF 过程噪声 / 初始协方差 / 关联门限 ====================
    % 每行格式：[雷达专属字段, 通用字段（未经 radar_params 时使用）]
    % 过程噪声 Q 的倍率：R2 更大（噪声大所以需要更多过程噪声来跟踪机动）
    % 1e5 的量级使 UKF 能更好地适应目标机动，但过大会引入噪声
    params.radar1_ukf_Q_scale   = 1e5;
    params.radar2_ukf_Q_scale   = 2e5;
    % 新物理过程噪声使用统一的连续白噪声加速度谱密度；同一目标不因
    % 雷达量测精度不同而具有不同运动噪声。旧 Q_scale 字段仅保留兼容。
    params.ukf_process_accel_psd_m2_s3 = 0.02;
    % 初始位置协方差标准差（度）（0.05° 约 5.5km 的地面分辨率）
    params.radar1_ukf_P_pos_std = 0.05;
    params.radar2_ukf_P_pos_std = 0.05;
    % 初始速度协方差标准差（度/秒）（0.004°/s 约 0.007m/s² 的加速度不确定度）
    params.radar1_ukf_P_vel_std = 0.004;
    params.radar2_ukf_P_vel_std = 0.005;
    % 初始化协方差以物理单位配置，并在 UKF 内按当前纬度转换。
    params.radar1_ukf_init_pos_std_m = 14000.0;
    params.radar2_ukf_init_pos_std_m = 24000.0;
    params.radar1_ukf_init_vel_std_ms = 130.0;
    params.radar2_ukf_init_vel_std_ms = 180.0;
    % 马氏距离关联门限的 sigma 倍数（6σ 覆盖 99.7% 的高斯分布）
    % 即关联门限 = 6 * sqrt(观测噪声方差)，放宽关联条件以减少漏关联
    params.radar1_gate_sigma    = 6;
    params.radar2_gate_sigma    = 6;
    % 速度硬门限（Vr 超过此值直接拒绝关联，单位 m/s）
    % R1 为 20m/s（精密雷达速度分辨好），R2 为 40m/s（放宽限制）
    params.radar1_gate_vr_ms    = 20;
    params.radar2_gate_vr_ms    = 40;
    % 连续漏检阈值：达到此帧数后航迹终止（8 帧 × 30s = 240s = 4分钟）
    params.radar1_tracker_K_loss = 8;
    params.radar2_tracker_K_loss = 8;
    % 顶层默认值（如果不调用 radar_params 而直接用 params 时的后备值）
    % 确保即使跳过 radar_params 映射，tracker 也能正常工作
    params.tracker_K_loss        = 8;

    % ==================== UKF 共用 UT（Unscented Transform）参数 ====================
    % alpha：Sigma 点散布度参数，越小分布越集中
    % alpha=1e-2 使 Sigma 点靠近均值，减少非线性区域的采样密度
    params.ukf_alpha = 0.3;
    % beta：先验分布参数，对于高斯分布 beta=2 是最优的
    % 配合 alpha 确定 Sigma 点的权重分配
    params.ukf_beta  = 2.0;
    % kappa：二次精度参数，设为 0（对于高斯分布不是必需的）
    % kappa=0 时中心矩的二阶项近似最优
    params.ukf_kappa = 0.0;
    % 多普勒量测噪声标准差（共用，不被 radar_params 覆盖）
    % 直接使用径向速度噪声标准差作为 UKF 的多普勒噪声输入
    params.ukf_rv_std_ms = params.radial_vel_noise_std_ms;

    % ==================== IMM 三模型（CV/CT-left/CT-right）参数 ====================
    % CV = Constant Velocity（常速模型），CT = Constant Turn（恒转弯模型）
    % 模型转移概率：直线飞行时极少切换到转弯模型
    % CV→CT 转移概率 0.001 意味着平均 1000 帧（约 5 分钟）才切换一次
    params.imm_Pi_CV_to_CT = 0.001;     % CV→CT 转移概率（平均 1000 帧切换一次）
    % CT→CV 转移概率同样为 0.001，转弯后大概率回到常速模式
    params.imm_Pi_CT_to_CV = 0.001;     % CT→CV 转移概率
    % Single-station IMM uses CV, fixed-rate left CT, and fixed-rate right CT.
    % Only the positive magnitude is configured; the right-turn model uses -omega.
    params.imm_turn_rate_rad_per_sec = 1.0 * pi / 180;
    % 自适应模式选择：'3in1' = CV 瞬态增益 + CT 固定高机动 + IMM 慢概率融合
    % 三种机制协同：瞬态增益应对突发机动，CT 模型跟踪持续转弯，IMM 平滑过渡
    params.imm_adapt_mode  = '3in1';
    % 3in1 模式的慢变转移概率（比快速模式 0.001 更激进，0.03 意味着平均 33 帧切换）
    % 慢变模式在检测到机动趋势时更快响应，但又不至于频繁震荡
    params.imm_slow_Pi_CV_to_CT = 0.03;
    params.imm_slow_Pi_CT_to_CV = 0.03;
    % 优先使用具有时间含义的驻留时间生成转移概率。
    params.imm_cv_dwell_time_sec = 2400.0;
    params.imm_ct_dwell_time_sec = 360.0;
    % CT 模型的固定过程噪声倍率（高机动时增大 Q）
    % 1.8 倍放大过程噪声使滤波器更信任预测而非观测，适应转弯运动
    params.imm_ct_fixed_Q_scale = 4.5;
    % CV 瞬态增益触发/满量程的 NIS（归一化创新平方）阈值
    % NIS < 3.0 时不触发增益，NIS > 12.0 时增益达到最大
    % 3.0~12.0 之间线性插值增益系数
    params.imm_transient_nis_start   = 3.0;
    params.imm_transient_nis_full    = 12.0;
    % CV 最大增益倍率（NIS 大时临时增大观测权重）
    % 5.0 倍意味着机动时观测更新步长放大 5 倍，加速收敛
    params.imm_transient_gain_max    = 7.0;
    % 短时 NIS 的 EWMA（指数加权移动平均）系数
    % 0.65 偏向近期 NIS 值，能快速反映机动状态变化
    params.imm_transient_ewma_alpha  = 0.65;

    % ==================== 自适应 Q / 模糊控制参数 ====================
    % 启用模糊自适应 Q 估计（根据 NIS 动态调整过程噪声）
    params.use_fuzzy_adaptive = true;
    % NIS 滑动窗口大小（Fun_UpdateTrackByAsscResult_Oracle 中使用）
    % 用最近 3 帧的 NIS 值做模糊推理，窗口越大越平滑但响应越慢
    params.fuzzy_window_size  = 3;
    % 模糊路径的 EMA 系数（adapt_q.m 中使用）
    % 0.10 的系数非常保守，Q 值缓慢漂移，避免剧烈波动
    params.fuzzy_ema_eta      = 0.10;
    % 自适应 Q 的下限/上限（防止 Q 过小导致发散、过大导致噪声放大）
    % Q_min=0.5 保证最小噪声水平，Q_max=4.0 防止过度扩散
    params.adaptive_Q_min     = 0.5;
    params.adaptive_Q_max     = 4.0;
    % 机动路径的 EMA 系数（检测到机动时快速调整 Q）
    % 0.50 比模糊路径快 5 倍，机动检测触发后立即增大 Q
    params.maneuver_ema_eta   = 0.50;
    % PDA（概率数据关联）门内检测概率
    % 对应门限下的理论 Pd，赋给 imm.Pg，用于 IMM 权重计算
    % 0.8647 是在给定关联门限和观测噪声下的积分结果
    params.pda_pd_gate        = 0.8647;

    % ==================== Oracle 起始器可配置滑窗参数 ====================
    % 在最近 TOLERANT_NUM 个物理帧中，至少有 QUALIFY_NUM 帧出现真实命中才确认航迹。
    % 两者可按需求调整，但必须是正整数且 QUALIFY_NUM <= TOLERANT_NUM。
    % oracle_QUALIFY_NUM=3：至少 3 帧真实命中（排除随机虚警）
    params.oracle_QUALIFY_NUM   = 3;    % 默认最少真实命中帧数
    % oracle_TOLERANT_NUM=7：在 7 帧窗口内统计（允许 4 帧漏检）
    params.oracle_TOLERANT_NUM  = 7;    % 默认滑动窗口长度（物理帧）
    % 航迹质量参数（南阳式管理）：航迹质量分范围 0~15
    % confirm=8：确认航迹时赋予较高质量，确保稳定跟踪
    params.oracle_confirm_quality    = 8;   % 确认时赋予的质量分
    % maintain=4：维持航迹时质量降低，逐步衰减
    params.oracle_maintain_quality  = 4;    % 维持时的质量分
    % max=15：质量上限，防止分数无限累积
    params.oracle_max_quality       = 15;   % 质量上限
    % loss_penalty=1：每漏检一帧扣 1 分，8 帧漏检后质量归零航迹终止
    params.oracle_loss_quality_penalty = 1; % 漏检时的质量扣分

    % ==================== 真值终止开关 ====================
    % true: 真值轨迹结束后，航迹转为 HISTORY 状态（保留记录但不参与跟踪）
    % false: 只能靠 K_loss 连续漏检来终止航迹
    % 开启后可以在真值消失后保留航迹记录用于事后分析
    params.oracle_truth_terminate_enable = true;

    % ==================== 航迹状态常量（南阳式分类）====================
    % RELIABLE_TRACK=1：可靠航迹（质量≥8，可用于作战决策）
    params.RELIABLE_TRACK  = 1;
    % MAINTAIN_TRACK=2：维持航迹（质量 4~7，仍在跟踪但未达可靠级别）
    params.MAINTAIN_TRACK  = 2;
    % TEMPORARY_TRACK=6：临时航迹（新起始，质量<4，还在验证中）
    params.TEMPORARY_TRACK = 6;
    % HISTORY_TRACK=7：历史航迹（已终止，保留归档用于事后评估）
    params.HISTORY_TRACK   = 7;

    % ==================== 雷达硬约束（不可调整）====================
    % Pd = 0.6：雷达检测概率，物理特性决定
    % 0.6 的检测率意味着每 10 次扫描中只有 6 次能检测到目标
    params.detection_probability = 0.6;
    % Pfa = 0.001：虚警率，物理特性决定
    % 0.001 的虚警率意味着每 1000 个检测中约有 1 个是虚假的
    params.false_alarm_rate      = 0.001;

    % ==================== 内部计算用分辨率参数 ====================
    % 距离分辨率：10 km（内部用，不暴露给外部配置）
    % 10km 分辨率意味着每个距离单元的跨度为 10km
    range_resolution_km     = 10.0;
    % 方位分辨率：1°（内部用）
    % 1° 方位分辨率意味着波束 15° 宽度内有 15 个方位单元
    azimuth_resolution_deg  = 1.0;
    % 计算分辨单元数量（range_bins × angle_bins）
    % range_bins = (2000-1000)/10 = 100 个距离单元
    % angle_bins = 15/1 = 15 个方位单元
    % 总计 100×15 = 1500 个分辨单元，用于杂波密度估计
    params.n_resolution_cells = ...
        ((params.range_max_km - params.range_min_km) / range_resolution_km) * ...
        (params.beam_width_deg / azimuth_resolution_deg);

    % ==================== 跨雷达航迹匹配参数（双门限法）====================
    % 匹配方法：dualgate（双门限匹配）
    % 先做空间距离粗筛，再做连续帧一致性校验，两步过滤降低误匹配率
    params.track_matcher_method      = 'dualgate';
    % 第一门限：空间距离粗筛（35 km 内才进入下一步）
    % 35km 考虑到 OTH-SWR 的测距精度（10-20km）和定位误差
    params.dualgate_T1_km            = 35;
    % 第二门限：连续帧数要求（8 帧都满足距离条件）
    % 8 帧 = 4 分钟，足够排除随机接近的假匹配
    params.dualgate_M                = 8;
    % 方差校验阈值（50 km²，排除偶然接近的假匹配）
    % 50 km² 相当于 7km 的标准差，要求两航迹协方差不显著重叠则拒绝
    params.dualgate_var_km2          = 50;
    % 最少共现帧数（两雷达同时覆盖的帧数下限）
    % 5 帧确保有足够的重叠数据做统计检验
    params.dualgate_coexist_thresh   = 5;
    % 互斥后处理仅供旧 pair matcher；动态片段凝聚不使用一对一互斥
    % mutual_exclusion=true 表示一旦某 R1 航迹匹配成功，就不再与其他 R2 匹配
    params.dualgate_mutual_exclusion = true;

    % ==================== 动态航迹片段凝聚参数 ====================
    % 跨站候选至少需要的 effective 重叠帧数（5 帧）
    % 内部漏检保留（允许少量帧缺失），末端 tail 排除（防止拼接不连续片段）
    params.tracklet_cross_min_overlap_frames = 5;
    % 接受边的最低评分（0~1），双门限通过后再做质量筛选
    % 0.05 是较低门槛，宁可多保留一些候选边再做后续过滤
    params.tracklet_cross_score_min = 0.05;
    % 同站前段最后 support 到后段第一 support 之间最多允许的空缺帧（18 帧 = 9 分钟）
    % 18 帧的容忍度允许航迹因遮挡或低 Pd 产生的长时间中断
    params.tracklet_successor_max_gap_frames = 18;
    % 同站 CV 外推端点地理距离严格小于该值（100 km）
    % 100km 确保前后片段在地理上是连续的，不会跳跃
    params.tracklet_successor_distance_km = 100;
    % 同站联合协方差 Mahalanobis 平方距离严格小于该值（25）
    % 25 对应 5σ 门限，统计上严格的连续性检验
    params.tracklet_successor_mahal_gate = 25;
    % 同站续接边最低评分（0~1）
    % 0.02 比跨站评分更低，同站续接更容易被接受
    params.tracklet_successor_score_min = 0.02;
    % group 至少包含的证据边数（1 条边即可成组）
    % 孤立片段单独保留而不发布为融合 group
    params.tracklet_group_min_support_edges = 1;
    % allow_isolated=true 允许孤立片段作为独立 group 输出
    params.tracklet_group_allow_isolated = true;
    % 同站端点外推附加过程噪声尺度（1e-8 极小值）
    % 外推时几乎不加额外噪声，保持外推精度
    params.tracklet_prediction_q = 1e-8;

    % ==================== 可控航迹碎片实验配置 ====================
    % 每个场景目标在每部雷达上必须自然形成的有效本地片段数。
    % 该值只供实验夹具规划衰落窗口，禁止进入匹配、分组和融合判据。
    params.fragmentation.enabled = true;
    params.fragmentation.segments_per_target_per_radar = 2;
    params.fragmentation.require_exact_count = true;
    % 每次衰落至少覆盖 K_loss 帧，确保旧航迹由原生命周期逻辑自然死亡。
    params.fragmentation.fade_length_frames = params.tracker_K_loss;
    % 计入目标片段数的最低有效区间和真实量测支持帧数。
    params.fragmentation.min_effective_frames = params.dualgate_M;
    params.fragmentation.min_support_frames = params.oracle_QUALIFY_NUM;
    % 两部雷达使用独立、可复现的窗口规划随机流。
    params.fragmentation.seed_r1 = 1101;
    params.fragmentation.seed_r2 = 2202;
    % 确定性回溯搜索的最大候选试验次数。
    params.fragmentation.max_search_nodes = 10000;

    % 跨站无充分重叠时的时序接力门限。
    params.tracklet_handoff_max_gap_frames = 18;
    params.tracklet_handoff_distance_km = 100;
    params.tracklet_handoff_mahal_gate = 25;
    params.tracklet_handoff_score_min = 0.02;

    % 全局候选 group 枚举与歧义判定。
    params.tracklet_group_max_hypotheses = 10000;
    params.tracklet_group_ambiguity_margin = 0.05;

    % Fusion-center offline bridge smoothing. Local tails are excluded.
    params.bridge.enabled = true;
    params.bridge.earth_radius_m = 6371000;
    params.bridge.turn_rate_rad_per_sec = 1.0 * pi / 180;
    params.bridge.accel_std_mps2 = 0.35;
    % Model order: CV, CT-left, CT-right.
    params.bridge.mode_transition = [0.96 0.02 0.02; ...
                                     0.08 0.90 0.02; ...
                                     0.08 0.02 0.90];
    params.bridge.mode_probability_init = [0.80 0.10 0.10];
    % Chi-square 99%% gate with four state dimensions; diagnostic only.
    params.bridge.confidence_mahal_gate = 13.2767;

    % ==================== 数据源与随机种子 ====================
    % ADS-B 原始数据 CSV 文件路径（用于生成真值轨迹）
    % 文件名含日期时间戳，便于追溯数据来源
    params.adsb_csv_path = '2026-04-27 09-30-00.csv';
    % 随机种子：42（确保仿真结果可复现）
    % MATLAB 中调用 rng(42) 设置随机数生成器状态
    params.random_seed   = 42;
end
