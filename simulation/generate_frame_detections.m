% =========================================================================
% generate_frame_detections.m — 单帧点迹生成函数
% =========================================================================
%
% 【功能概述】
%   对单部双基地雷达、单次扫描（一帧）生成完整的点迹列表，包含两个来源：
%     1. 目标点迹 — 真实目标经检测概率 P_d 筛选后，叠加系统偏差和随机噪声
%     2. 虚警杂波 — 在雷达威力覆盖区内按泊松分布随机生成的虚假点迹
%   函数不涉及任何跟踪或关联逻辑，仅模拟雷达信号处理后的检测输出。
%
% 【数学原理】
%
%   1. 目标检测（二元假设检验）：
%      首先判断目标是否在雷达威力范围内（距离 + 方位约束同时满足）。
%      若在覆盖区内，以概率 P_d 判定为"检测到"（模拟 SNR 波动和电离层
%      传播衰落导致的漏检）。检测到则计算含噪量测，未检测到则本帧无目标
%      点迹输出。
%
%   2. 双基地量测模型（3 维极坐标）：
%      ┌─────────────────────────────────────────────────────────┐
%      │ 群距离 Rg  = r0 + r1                                    │
%      │   r0 = Haversine(Tx → 目标)  发射站到目标的大圆距离     │
%      │   r1 = Haversine(Rx → 目标)  接收站到目标的大圆距离     │
%      │                                                         │
%      │ 方位角 az  = sphere_utils_azimuth(Rx, 目标)             │
%      │   从接收站看目标的真北偏东角度（0°=北，顺时针）          │
%      │                                                         │
%      │ 径向速度 vd = v_Tx_proj + v_Rx_proj                     │
%      │   目标速度分别在 Tx→目标 和 Rx→目标 方向上的投影之和     │
%      │   = radial_vel(v, az_Tx) + radial_vel(v, az_Rx)         │
%      └─────────────────────────────────────────────────────────┘
%
%   3. 量测误差模型：
%        Rg_meas = Rg_true + bias_range + ε_range     ε_range ~ N(0, σ_range²)
%        az_meas = az_true + bias_az + ε_az           ε_az    ~ N(0, σ_az²)
%        vd_meas = vd_true            + ε_vd          ε_vd    ~ N(0, σ_vd²)
%      其中 bias 为固定系统偏差（模拟雷达标定误差），ε 为随机噪声。
%      径向速度不含系统偏差（OTH-SWR 多普勒测量不受距离/方位标定误差影响）。
%
%   4. 虚警杂波模型（泊松点过程）：
%      每帧虚警数量: n_false ~ Poisson(λ)，λ = N_cells × P_fa
%      其中 N_cells = (Δrange/Δr) × (Δaz/Δaz_res) 为总分辨单元数，
%      P_fa = 0.001 为单分辨单元虚警率。
%
%      杂波在接收站极坐标 (r1, az) 中均匀生成，再通过球面正算转到
%      经纬度坐标。该方式确保杂波严格落在雷达威力覆盖区内（距离和方位
%      均满足约束），避免在直角坐标直接采样可能导致的覆盖区外杂波。
%
%      杂波双基地群距离: Rg_clutter = r0(Tx→clutter) + r1(杂波点)
%      杂波多普勒: 在 [-200, +200] m/s 内均匀分布
%        （OTH-SWR 电离层杂波的多普勒展宽典型范围）
%
%      关键设计：杂波的 prange 和 paz 字段也掺入了真实系统偏差（range_bias,
%      az_bias），使得主循环 Phase 4 偏差校正后 drange ≈ fake_Rg, daz ≈ fake_az，
%      保证杂波在量测空间和地理空间之间的一致性。
%
% 【输入参数】
%   rx_lon, rx_lat    — 接收站经纬度（度）
%   tx_lon, tx_lat    — 发射站经纬度（度）
%   tgt_lon, tgt_lat  — 目标真实经纬度（度）
%   tgt_lon_rate      — 目标经度变化率（度/秒）
%   tgt_lat_rate      — 目标纬度变化率（度/秒）
%   frameID           — 帧编号（整数，从 1 开始）
%   time_sec          — 当前帧的仿真时间（秒，相对于 ref_start_time）
%   range_bias        — 距离系统偏差（米）
%   az_bias           — 方位系统偏差（度）
%   beam_center       — 雷达波束中心方位角（度）
%   params            — 仿真参数结构体（需含 detection_probability,
%                       false_alarm_rate, n_resolution_cells, beam_width_deg,
%                       range_min_m, range_max_m, radial_vel_noise_std_ms）
%   range_noise       — （可选）距离噪声标准差（米），默认取 R1 的值
%   az_noise          — （可选）方位噪声标准差（度），默认取 R1 的值
%
% 【输出】
%   detList          — 结构体数组，本帧的所有点迹（目标 + 杂波）。
%                      每个点迹为 struct，字段包括：
%                        frameID, time_sec       — 帧标识和时间戳
%                        prange, paz, pvr        — 原始量测值（含偏差+噪声）
%                        range_meas, azimuth_meas, radial_vel_meas
%                                                — 量测值副本（目标）/ NaN（杂波）
%                        range_true, azimuth_true, radial_vel_true
%                                                — 真实值（目标）/ NaN（杂波）
%                        lat_true, lon_true      — 真实经纬度
%                        lat, lon                — 反算经纬度（目标为 NaN，
%                                                  杂波为球面正算值）
%                        is_clutter              — 逻辑值，true=杂波
%   has_target_det   — 逻辑值，本帧是否检测到目标（无论是否有杂波）
%
% 【调用关系】
%   被 run_simulation.m Phase 2 逐帧循环调用，分别为 R1 和 R2 生成点迹。
%   内部调用:
%     radar_coverage_check()            — 威力覆盖判定
%     sphere_utils_haversine_distance() — 球面大圆距离
%     sphere_utils_azimuth()            — 球面方位角
%     sphere_utils_radial_velocity()    — 径向速度投影
%     sphere_utils_destination_point()  — 球面正算（杂波定位用）
% =========================================================================

function [detList, has_target_det] = generate_frame_detections(rx_lon, rx_lat, ...
        tx_lon, tx_lat, tgt_lon, tgt_lat, tgt_lon_rate, tgt_lat_rate, ...
        frameID, time_sec, range_bias, az_bias, beam_center, params, ...
        range_noise, az_noise)

    % ---- 默认参数：未显式传入时使用 R1（精度站）的噪声水平 ----
    if nargin < 16, range_noise = params.radar1_range_noise_std_m; end
    if nargin < 17, az_noise = params.radar1_azimuth_noise_std_deg; end

    % ---- 初始化输出 ----
    detList = [];           % 空数组，后续逐步追加点迹
    has_target_det = false; % 默认未检测到目标

    % =====================================================================
    % 第 1 部分：目标检测
    % =====================================================================
    % 两步判定：
    %   Step A — 威力覆盖检查（距离 + 方位双约束）
    %   Step B — 检测概率判定（P_d 抽签，模拟 SNR 衰落导致的随机漏检）

    [in_cov, ~, ~] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, beam_center, params);

    if in_cov
        % ---- Step B: 检测概率抽签 ----
        % rand() 返回 [0,1) 均匀分布随机数，≤ P_d 表示"检测到"
        % P_d = 0.6 意味着平均每 3 帧有 1 帧漏检（40% 漏检率）
        if rand() <= params.detection_probability
            has_target_det = true;

            % ---- 计算目标真实极坐标量测（无噪声、无偏差） ----
            % 群距离 = Tx→目标 + Rx→目标（双基地总路径长度）
            r0 = sphere_utils_haversine_distance(tx_lon, tx_lat, tgt_lon, tgt_lat);
            r1_dist = sphere_utils_haversine_distance(rx_lon, rx_lat, tgt_lon, tgt_lat);
            Rg_true = r0 + r1_dist;

            % 方位角：仅与接收站有关（与单基地雷达相同）
            az_true = sphere_utils_azimuth(rx_lon, rx_lat, tgt_lon, tgt_lat);

            % 双基地径向速度：Tx 方向投影 + Rx 方向投影
            % 这是目标真实速度在双基地几何下的多普勒分量
            az_tx = sphere_utils_azimuth(tx_lon, tx_lat, tgt_lon, tgt_lat);
            rv_tx = sphere_utils_radial_velocity(tgt_lon_rate, tgt_lat_rate, tgt_lat, az_tx);
            rv_rx = sphere_utils_radial_velocity(tgt_lon_rate, tgt_lat_rate, tgt_lat, az_true);
            vd_true = rv_tx + rv_rx;

            % ---- 施加系统偏差和随机噪声 ----
            % 量测 = 真值 + 固定偏差 + 高斯随机噪声
            % 偏差代表雷达标定误差（系统性），噪声代表逐帧随机起伏
            Rg_meas = Rg_true + range_bias + randn() * range_noise;
            az_meas = az_true + az_bias + randn() * az_noise;
            vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms;

            % ---- 组装目标点迹结构体 ----
            % prange / paz / pvr: 原始量测（含偏差+噪声），即雷达直接输出的值
            % range_meas / azimuth_meas / radial_vel_meas: 量测副本
            % range_true / azimuth_true / radial_vel_true: 真实极坐标值（用于事后误差评估）
            % lat_true / lon_true: 真实经纬度
            % lat / lon: NaN（目标的经纬度由 Phase 4 偏差校正+反解得到，此处暂不计算）
            % is_clutter = false: 标识为真实目标点迹
            det = struct('frameID', frameID, 'time_sec', time_sec, ...
                'prange', Rg_meas, 'paz', az_meas, 'pvr', vd_meas, ...
                'range_meas', Rg_meas, 'azimuth_meas', az_meas, 'radial_vel_meas', vd_meas, ...
                'range_true', Rg_true, 'azimuth_true', az_true, 'radial_vel_true', vd_true, ...
                'lat_true', tgt_lat, 'lon_true', tgt_lon, ...
                'lat', NaN, 'lon', NaN, 'is_clutter', false);
            detList = [detList, det];
        end
        % 注：若 rand() > P_d，本帧漏检，不产生目标点迹
    end
    % 注：若目标不在覆盖区内，不进行任何检测尝试

    % =====================================================================
    % 第 2 部分：虚警杂波生成
    % =====================================================================
    % 虚警模型：泊松点过程（Poisson Point Process）
    %   - 每帧期望虚警数: λ = N_cells × P_fa = 1500 × 0.001 = 1.5 个
    %   - 实际虚警数: n_false ~ Poisson(λ)，由 poissrnd 随机生成
    %   - 杂波在接收站极坐标 (r1, az) 中均匀分布后再正算到地理坐标
    %
    % 为什么在极坐标而非直角坐标采样杂波？
    %   直角坐标（经纬度）中采样需要额外的覆盖区约束（方位和距离需同时
    %   满足），而极坐标可以直接通过控制 r1 ∈ [r_min, r_max] 和
    %   az ∈ [beam_center ± half_beam] 保证 100% 在威力范围内。

    % 泊松随机数：本帧实际产生的虚警个数//专业
    n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate);

    % 波束半宽度：用于确定虚警的方位采样范围
    half_beam = params.beam_width_deg / 2;

    for f = 1:n_false
        % ---- 在接收站极坐标 (r1, az) 中均匀随机采样 ----
        % r1 ∈ [range_min_m, range_max_m]: 接收站到杂波点的斜距
        fake_r1 = params.range_min_m + rand() * (params.range_max_m - params.range_min_m);
        % az ∈ [beam_center - half_beam, beam_center + half_beam]: 方位角
        fake_az = beam_center - half_beam + rand() * params.beam_width_deg;

        % ---- 球面正算：由 (r1, az) 求杂波点经纬度 ----
        % 利用大圆目的地点公式，从接收站出发沿方位角 az 走距离 r1
        [clut_lon, clut_lat] = sphere_utils_destination_point(rx_lon, rx_lat, fake_r1, fake_az);

        % ---- 计算杂波点的双基地群距离 ----
        % 杂波点也需要双基地群距离 Rg = r0 + r1，用于后续偏差校正的一致性
        r0 = sphere_utils_haversine_distance(tx_lon, tx_lat, clut_lon, clut_lat);
        fake_Rg = r0 + fake_r1;

        % ---- 杂波多普勒：在 [-200, +200] m/s 内均匀随机 ----
        % OTH-SWR 中电离层杂波的多普勒谱通常展宽在 ±200 m/s 范围内，
        % 对应电离层不规则体的运动速度（F 层典型多普勒展宽约 0.1~1 Hz，
        % 在 HF 频段约 10~20 MHz 时对应 ±50~±200 m/s）
        fake_vr = -200 + rand() * 400;

        % ---- 组装杂波点迹结构体 ----
        % 关键设计说明：
        %   杂波的 prange 和 paz 也掺入真实系统偏差（range_bias, az_bias）。
        %   这样做是为了保证 Phase 4 偏差校正后：
        %     drange = prange - dr_est ≈ fake_Rg
        %     daz    = paz    - da_est ≈ fake_az
        %   即校正后的杂波量测在数值上回到"无偏"状态，与地理坐标一致。
        %   如果不掺入偏差，校正后杂波会系统性偏移，与真实杂波地理分布不符。
        %
        % 字段说明：
        %   prange / paz / pvr: 含偏差的原始量测（pvr 不含偏差，直接取随机值）
        %   range_meas / azimuth_meas: NaN（杂波没有"真实量测"概念）
        %   range_true / azimuth_true / radial_vel_true: NaN（杂波无真值）
        %   lat_true / lon_true: 杂波点的经纬度（由球面正算得到）
        %   lat / lon: 与 lat_true / lon_true 相同（杂波无需反解，位置已确定）
        %   is_clutter = true: 标识为虚警杂波
        det = struct('frameID', frameID, 'time_sec', time_sec, ...
            'prange', fake_Rg + range_bias, ...
            'paz', fake_az + az_bias, ...
            'pvr', fake_vr, ...
            'range_meas', NaN, 'azimuth_meas', NaN, 'radial_vel_meas', fake_vr, ...
            'range_true', NaN, 'azimuth_true', NaN, 'radial_vel_true', NaN, ...
            'lat_true', clut_lat, 'lon_true', clut_lon, ...
            'lat', clut_lat, 'lon', clut_lon, ...
            'is_clutter', true);
        detList = [detList, det];
    end
end
