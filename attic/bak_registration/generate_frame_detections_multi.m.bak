% =========================================================================
% generate_frame_detections_multi.m — 多目标单帧点迹生成函数
% =========================================================================
%
% 【功能概述】
%   对单部双基地雷达、单次扫描（一帧）生成完整的点迹列表，包含：
%     1. 多个目标点迹 — 每个目标经检测概率 P_d 筛选后，叠加系统偏差和随机噪声
%     2. 虚警杂波 — 在雷达威力覆盖区内按泊松分布随机生成
%   每个检测点迹带有 aircraft_id 字段，标记所属的目标编号（1-based）。
%
% 【与 generate_frame_detections.m 的区别】
%   - 输入为 N 个目标的状态数组（每个目标独立检测），而非单个目标
%   - 每个检测输出带有 .aircraft_id 字段，用于后续多目标关联
%   - 虚警杂波的 aircraft_id = 0（表示不属于任何真实目标）
%   - 循环遍历所有目标，对每个目标独立执行检测和量测计算
%
% 【检测流程】
%   对每个目标：
%     1. 威力覆盖检查（距离 + 方位双约束）
%     2. 检测概率抽签（rand() <= P_d）
%     3. 计算真实天波量测（群距离、方位角、多普勒）
%     4. 叠加系统偏差和高斯随机噪声
%     5. 组装点迹结构体
%
%   对所有目标检测完成后：
%     6. 按泊松分布生成虚警数量
%     7. 在覆盖区内均匀随机生成杂波点迹
%     8. 组装杂波结构体
%
% 【输入参数】
%   rx_lon, rx_lat    — 接收站经纬度（度）
%   tx_lon, tx_lat    — 发射站经纬度（度）
%   tgt_states        — N_targets x 5 矩阵，每行 [lon, lat, lon_rate, lat_rate, aircraft_id]
%   frameID           — 帧编号（整数）
%   time_sec          — 当前帧的仿真时间（秒）
%   range_bias, az_bias — 距离和方位的系统偏差
%   beam_center       — 雷达波束中心方位角（度）
%   params            — 仿真参数结构体
%   range_noise, az_noise — 噪声标准差（可选，默认取 params 中的 R1 参数）
%
% 【输出】
%   detList           — 结构体数组，本帧所有点迹（目标点迹 + 虚警杂波）
%   has_target_dets   — N_targets x 1 逻辑数组，has_target_dets(i)=true 表示目标 i 被检测到
%
% =========================================================================

function [detList, has_target_dets] = generate_frame_detections_multi(rx_lon, rx_lat, ...
        tx_lon, tx_lat, tgt_states, frameID, time_sec, range_bias, az_bias, ...
        beam_center, params, range_noise, az_noise)

    % ---- 默认噪声参数 ----
    % 如果调用方没有显式传入噪声标准差，使用雷达1（精度站）的默认值
    % 这样可以保证不同调用场景下的噪声水平一致
    if nargin < 15, range_noise = params.radar1_range_noise_std_m; end
    if nargin < 16, az_noise = params.radar1_azimuth_noise_std_deg; end

    % 初始化点迹列表为空结构体数组，后续逐个点迹追加
    detList = [];

    % 获取当前帧的目标数量
    n_targets = size(tgt_states, 1);

    % 初始化检测标志数组，全部设为 false（未检测到）
    % 用于记录每个目标是否在本帧被成功检测
    has_target_dets = false(n_targets, 1);

    % 计算波束半宽度，用于后续虚警方位角的采样范围
    half_beam = params.beam_width_deg / 2;

    % 按泊松分布生成本帧的虚警数量
    % lambda = N_cells * P_fa = 分辨单元总数 × 单单元虚警率
    % 例如：1500 * 0.001 = 1.5，平均每帧 1.5 个虚警
    % poissrnd 从泊松分布中随机采样，实际值可能为 0, 1, 2, ...
    n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate);

    % =================================================================
    % 第 1 部分：逐个目标检测
    % 对每个目标独立执行：威力检查 -> 检测概率抽签 -> 量测计算 -> 点迹组装
    % =================================================================
    for t = 1:n_targets
        % 从 tgt_states 中提取第 t 个目标的状态分量
        % tgt_states(t, :) = [lon, lat, lon_rate, lat_rate, aircraft_id]
        tgt_lon = tgt_states(t, 1);       % 目标经度（度）
        tgt_lat = tgt_states(t, 2);       % 目标纬度（度）
        tgt_lr = tgt_states(t, 3);        % 经度变化率（度/秒）
        tgt_latr = tgt_states(t, 4);      % 纬度变化率（度/秒）
        ac_id = tgt_states(t, 5);         % 目标编号（1-based）

        % 威力覆盖检查：判断目标是否在雷达的探测范围内
        % 返回 in_cov（是否覆盖）、r1（距离）、az（方位角）
        % 这里只关心 in_cov，所以 ~ 忽略其他两个返回值
        [in_cov, ~, ~] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, ...
            beam_center, params);

        % 双重判定：
        %   条件1: in_cov == true — 目标在雷达威力覆盖区内（距离+方位）
        %   条件2: rand() <= params.detection_probability — 检测概率抽签
        %         rand() 返回 [0,1) 均匀分布随机数
        %         P_d = 0.6 表示 60% 概率检测到，40% 概率漏检
        %         这模拟了信噪比波动和电离层衰落导致的随机漏检
        if in_cov && rand() <= params.detection_probability
            % 标记目标 t 在本帧被检测到
            has_target_dets(t) = true;

            % ---- 计算目标的真实天波双基地量测值（无噪声、无偏差）----
            % 群距离 Rg = r_tx + r_rx，考虑电离层反射的真实传播路径
            Rg_true = skywave_geometry('group_range', tx_lon, tx_lat, ...
                rx_lon, rx_lat, tgt_lon, tgt_lat);

            % 方位角 az：从接收站指向目标的球面方位角
            az_true = skywave_geometry('azimuth', rx_lon, rx_lat, tgt_lon, tgt_lat);

            % 多普勒频率（径向速度）vd：总传播路径对时间的导数
            % vd = dRg/dt = dr_tx/dt + dr_rx/dt
            % 需要将目标的经纬度速度分解到传播路径方向上
            vd_true = skywave_geometry('doppler', tx_lon, tx_lat, rx_lon, rx_lat, ...
                tgt_lon, tgt_lat, tgt_lr, tgt_latr);

            % ---- 施加系统偏差和随机噪声，生成含噪量测 ----
            % 量测 = 真实值 + 固定偏差（标定误差） + 高斯随机噪声（逐帧起伏）
            % randn() 产生标准正态分布 N(0,1) 的随机数
            Rg_meas = Rg_true + range_bias + randn() * range_noise;
            az_meas = az_true + az_bias + randn() * az_noise;

            % 径向速度噪声标准差从 params 中读取
            % OTH-SWR 多普勒测量不受距离/方位标定误差影响，故无 bias
            vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms;

            % ---- 组装目标点迹结构体 ----
            % 每个点迹包含丰富的字段，用于后续处理和误差评估：
            %
            % 帧标识：
            %   frameID, time_sec — 帧编号和仿真时间戳
            %
            % 原始量测（含偏差+噪声，即雷达直接输出的值）：
            %   prange, paz, pvr — 原始群距离、方位角、多普勒
            %
            % 量测副本（与原始量测相同，用于算法内部引用）：
            %   range_meas, azimuth_meas, radial_vel_meas
            %
            % 真实量测（无噪声、无偏差，用于事后误差分析）：
            %   range_true, azimuth_true, radial_vel_true
            %
            % 真实地理坐标（用于与跟踪结果比对）：
            %   lat_true, lon_true
            %
            % 反解经纬度（NaN，因为目标的经纬度由后续 Phase 4 偏差校正+反解得到）：
            %   lat, lon
            %
            % 标识字段：
            %   is_clutter = false — 标识为真实目标点迹
            %   aircraft_id — 所属目标编号（1-based），用于多目标关联
            det = struct('frameID', frameID, 'time_sec', time_sec, ...
                'prange', Rg_meas, 'paz', az_meas, 'pvr', vd_meas, ...
                'range_meas', Rg_meas, 'azimuth_meas', az_meas, 'radial_vel_meas', vd_meas, ...
                'range_true', Rg_true, 'azimuth_true', az_true, 'radial_vel_true', vd_true, ...
                'lat_true', tgt_lat, 'lon_true', tgt_lon, ...
                'lat', NaN, 'lon', NaN, ...
                'is_clutter', false, ...
                'aircraft_id', int32(ac_id));

            % 将点迹追加到 detList 结构体数组末尾
            % MATLAB 结构体数组动态扩展，每次追加 O(N) 复杂度
            detList = [detList, det];
        end
        % 注：若 rand() > P_d（未通过检测概率抽签），本帧对该目标漏检，不产生点迹
    end
    % 注：若目标不在覆盖区内（in_cov=false），不进行任何检测尝试

    % =================================================================
    % 第 2 部分：虚警杂波生成
    % 在雷达威力覆盖区内按极坐标均匀分布随机生成虚警点迹
    % =================================================================
    for f = 1:n_false
        % ---- 在接收站极坐标 (r1, az) 中均匀随机采样杂波位置 ----
        % r1 ∈ [range_min_m, range_max_m]: 接收站到杂波点的斜距
        % rand() 生成 [0,1) 均匀随机数，线性映射到距离范围
        fake_r1 = params.range_min_m + rand() * (params.range_max_m - params.range_min_m);

        % az ∈ [beam_center - half_beam, beam_center + half_beam]: 方位角
        % 确保杂波严格落在雷达波束覆盖范围内
        fake_az = beam_center - half_beam + rand() * params.beam_width_deg;

        % ---- 球面正算：由极坐标 (r1, az) 求杂波点经纬度 ----
        % 从接收站出发，沿方位角 fake_az 走距离 fake_r1，
        % 得到杂波点的经纬度坐标
        [clut_lon, clut_lat] = sphere_utils_destination_point(rx_lon, rx_lat, fake_r1, fake_az);

        % ---- 计算杂波点的天波双基地群距离 ----
        % 杂波点复用与目标完全一致的天波几何模型
        fake_Rg = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, ...
            clut_lon, clut_lat);

        % ---- 杂波多普勒：在 [-200, +200] m/s 内均匀随机 ----
        % OTH-SWR 中电离层杂波的多普勒谱通常展宽在 ±200 m/s 范围内，
        % 对应电离层不规则体的运动速度（F 层典型多普勒展宽约 0.1~1 Hz，
        % 在 HF 频段约 10~20 MHz 时对应 ±50~±200 m/s）
        fake_vr = -200 + rand() * 400;

        % ---- 组装杂波点迹结构体 ----
        % 关键设计说明：
        %   杂波的 prange 和 paz 也掺入真实系统偏差（range_bias, az_bias）。
        %   这样做是为了保证后续 Phase 4 偏差校正后：
        %     drange = prange - dr_est ≈ fake_Rg（校正后的群距离回到无偏状态）
        %     daz    = paz    - da_est ≈ fake_az（校正后的方位角回到无偏状态）
        %   即校正后的杂波量测在数值上与地理坐标一致，不会系统性偏移。
        %   如果不掺入偏差，校正后杂波会偏离真实位置。
        %
        % 字段说明：
        %   prange / paz / pvr: 含偏差的原始量测
        %                      （pvr 不含偏差，直接取 [-200,+200] 随机值）
        %   range_meas / azimuth_meas: NaN（杂波没有"真实量测"概念）
        %   radial_vel_meas: fake_vr（杂波的多普勒既是量测也是"伪真值"）
        %   range_true / azimuth_true / radial_vel_true: NaN（杂波无真值）
        %   lat_true / lon_true: 杂波点的经纬度（由球面正算得到）
        %   lat / lon: 与 lat_true / lon_true 相同（杂波无需反解，位置已确定）
        %   is_clutter = true: 标识为虚警杂波
        %   aircraft_id = 0: 杂波不属于任何真实目标
        det = struct('frameID', frameID, 'time_sec', time_sec, ...
            'prange', fake_Rg + range_bias, ...
            'paz', fake_az + az_bias, ...
            'pvr', fake_vr, ...
            'range_meas', NaN, 'azimuth_meas', NaN, 'radial_vel_meas', fake_vr, ...
            'range_true', NaN, 'azimuth_true', NaN, 'radial_vel_true', NaN, ...
            'lat_true', clut_lat, 'lon_true', clut_lon, ...
            'lat', clut_lat, 'lon', clut_lon, ...
            'is_clutter', true, ...
            'aircraft_id', int32(0));

        % 将杂波点迹追加到 detList 末尾
        detList = [detList, det];
    end
end
