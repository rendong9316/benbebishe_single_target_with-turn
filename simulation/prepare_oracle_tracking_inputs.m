function inputs = prepare_oracle_tracking_inputs(scenario_name, param_overrides)
% PREPARE_ORACLE_TRACKING_INPUTS 构造 Oracle 跟踪所需的公共输入
%
% 这是 Oracle 跟踪仿真的统一入口函数，一次性完成:
%   1. 加载仿真参数
%   2. 生成真值场景（航迹 + 时间网格）
%   3. 基于 ADS-B 数据标定雷达偏差
%   4. 为 R1 和 R2 分别生成检测点迹序列
%   5. 创建两部雷达的 UKF 滤波器模板
%
% 所有输出打包到一个 struct 中，供上层仿真循环直接消费
%
% 输入:
%   scenario_name — 场景名称（可选，默认 'multi_cross'）
%
% 输出:
%   inputs — 包含 params, scenario, truth_all, detList_R1/R2,
%            ukf1_tpl/ukf2_tpl 等字段的组合结构体

    % 如果未传入场景名，使用默认场景
    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'multi_cross';
    end
    if nargin < 2 || isempty(param_overrides)
        param_overrides = struct();
    end

    % 加载 Oracle 模式的仿真参数
    params = simulation_params_oracle();
    override_names = fieldnames(param_overrides);
    for override_index = 1:numel(override_names)
        params.(override_names{override_index}) = ...
            param_overrides.(override_names{override_index});
    end

    % 设置随机数种子，确保仿真结果可复现
    rng(params.random_seed);

    % 生成真值场景（包含航迹、时间网格、帧数等）
    scenario = build_truth_scenario(scenario_name, params);

    % 提取场景中的关键数据
    truth_all = scenario.truth_all;          % 真值航迹（cell 数组）
    truthTrajs = scenario.truthTrajs;        % 真值轨迹结构体数组
    t1_grid = scenario.t1_grid;              % R1 采样时间网格
    t2_grid = scenario.t2_grid;              % R2 采样时间网格
    n_frames = scenario.n_frames;            % 共同覆盖帧数

    % 基于 ADS-B 数据标定两部雷达的系统偏差（距离偏差 + 方位偏差）
    [dr1_est, da1_est, dr2_est, da2_est] = calibrate_bias(params);

    % 为 R1 和 R2 分别生成完整的检测点迹序列
    % generate_detections 内部逐帧调用 generate_frame_detections_multi，
    % 然后进行偏差校正和 bistatic 反解
    detList_R1 = generate_detections(1, params, truth_all, t1_grid, n_frames, dr1_est, da1_est);
    detList_R2 = generate_detections(2, params, truth_all, t2_grid, n_frames, dr2_est, da2_est);

    % 获取两部雷达的专用参数（从通用 params 中提取）
    params_r1 = radar_params(params, 1);
    params_r2 = radar_params(params, 2);

    % 创建 R1 的 UKF IMM 滤波器模板
    % IMM = Interactive Multi-Model，包含 CV（恒速）和 CT（恒转弯）两个子滤波器
    ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

    % 创建 R2 的 UKF IMM 滤波器模板
    ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    % 将所有输入打包到一个结构体中，方便统一传递给仿真循环
    inputs = struct('params', params, 'scenario', scenario, 'truth_all', {truth_all}, ...
        'truthTrajs', {truthTrajs}, 't1_grid', t1_grid, 't2_grid', t2_grid, ...
        'detList_R1', {detList_R1}, 'detList_R2', {detList_R2}, ...
        'bias_estimates', [dr1_est, da1_est, dr2_est, da2_est], ...
        'params_r1', params_r1, 'params_r2', params_r2, ...
        'ukf1_tpl', ukf1_tpl, 'ukf2_tpl', ukf2_tpl);
end

function [dr1_est, da1_est, dr2_est, da2_est] = calibrate_bias(params)
% calibrate_bias — 基于 ADS-B 数据标定雷达系统偏差
%
% 算法:
%   1. 读取 ADS-B 真值轨迹 CSV 文件
%   2. 对 ADS-B 点进行降采样（每隔 cal_step 个点取一个），最多取 5000 个
%   3. 对每个 ADS-B 点:
%      - 检查是否在 R1/R2 的威力覆盖范围内
%      - 如果在覆盖区内，计算该点的理论偏差（标定偏差 + 随机噪声）
%   4. 对所有样本的偏差取均值作为最终估计
%
% 输入:
%   params — 仿真参数结构体
% 输出:
%   dr1_est, da1_est — R1 的距离和方位偏差估计
%   dr2_est, da2_est — R2 的距离和方位偏差估计

    % 设置随机数种子，确保标定结果可复现
    rng(params.random_seed);

    % 读取 ADS-B CSV 文件（无变量名头）
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);

    % 提取 ADS-B 的纬度和经度列
    adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;

    % 初始化偏差收集数组
    dr1 = []; da1 = []; dr2 = []; da2 = [];

    % 计算降采样参数：最多取 5000 个样本或全部 ADS-B 点数
    n_check = min(5000, height(T_adsb));
    % 降采样步长：每隔 cal_step 个点取一个
    cal_step = max(1, floor(height(T_adsb) / n_check));

    % 遍历降采样后的 ADS-B 点
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
        % 跳过经纬度为 NaN 的无效点
        if isnan(t_lon) || isnan(t_lat), continue; end

        % 检查该 ADS-B 点是否在 R1 威力覆盖范围内
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            % R1 距离偏差 = 标定偏差 + 高斯随机噪声（模拟多次测量的统计特性）
            dr1(end+1) = params.radar1_range_bias_m + randn()*params.radar1_range_noise_std_m; %#ok<AGROW>
            % R1 方位偏差 = 标定偏差 + 高斯随机噪声（wrap 到 [-180, 180]）
            da1(end+1) = wrap_angle(params.radar1_azimuth_bias_deg + randn()*params.radar1_azimuth_noise_std_deg); %#ok<AGROW>
        end

        % 同理检查 R2
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, t_lon, t_lat, params.radar2_beam_center_deg, params);
        if in2
            dr2(end+1) = params.radar2_range_bias_m + randn()*params.radar2_range_noise_std_m; %#ok<AGROW>
            da2(end+1) = wrap_angle(params.radar2_azimuth_bias_deg + randn()*params.radar2_azimuth_noise_std_deg); %#ok<AGROW>
        end
    end

    % 如果 R1 或 R2 没有足够的有效样本，报错
    if isempty(dr1) || isempty(da1) || isempty(dr2) || isempty(da2)
        error('prepare_oracle_tracking_inputs:calibrationNoSamples', 'ADS-B 标定没有两个雷达共同所需的有效样本');
    end

    % 对有效样本取均值作为最终偏差估计
    dr1_est = mean(dr1); da1_est = mean(da1); dr2_est = mean(dr2); da2_est = mean(da2);
end

function detList = generate_detections(radar_id, params, truth_all, t_grid, n_frames, dr_est, da_est)
% generate_detections — 为指定雷达生成完整的检测点迹序列
%
% 算法流程（逐帧循环）:
%   1. 根据当前帧时间，从真值航迹中提取所有目标的状态 [lon, lat, lon_rate, lat_rate, aircraft_id]
%   2. 调用 generate_frame_detections_multi 生成含噪检测（目标点迹 + 虚警）
%   3. 对每个检测点迹进行后处理:
%      a. 偏差校正：drange = prange - dr_est, daz = paz - da_est
%      b. 如果缺少 radial_vel_meas 字段，用 pvr 填充
%      c. 如果 lat/lon 为 NaN，用 bistatic 反解计算
%      d. 计算原始（未校正）的反解经纬度
%   4. 将处理后的点迹合并为单帧数组
%
% 输入:
%   radar_id — 雷达编号（1 或 2）
%   params   — 仿真参数
%   truth_all — 真值航迹
%   t_grid   — 时间网格
%   n_frames — 帧数
%   dr_est, da_est — 距离和方位偏差估计
%
% 输出:
%   detList  — cell 数组，每帧包含该帧的所有检测点迹

    % 初始化 cell 数组，每帧一个元素
    detList = cell(n_frames, 1);

    % 根据雷达编号设置对应的雷达参数
    if radar_id == 1
        % R1 参数：独立随机种子、接收站/发射站坐标、偏差、噪声、波束中心
        rng(params.random_seed + 1e7); rx_lon=params.radar1_lon; rx_lat=params.radar1_lat; tx_lon=params.radar1_tx_lon; tx_lat=params.radar1_tx_lat; range_bias=params.radar1_range_bias_m; az_bias=params.radar1_azimuth_bias_deg; beam=params.radar1_beam_center_deg; range_noise=params.radar1_range_noise_std_m; az_noise=params.radar1_azimuth_noise_std_deg;
    else
        % R2 参数
        rng(params.random_seed + 2e7); rx_lon=params.radar2_lon; rx_lat=params.radar2_lat; tx_lon=params.radar2_tx_lon; tx_lat=params.radar2_tx_lat; range_bias=params.radar2_range_bias_m; az_bias=params.radar2_azimuth_bias_deg; beam=params.radar2_beam_center_deg; range_noise=params.radar2_range_noise_std_m; az_noise=params.radar2_azimuth_noise_std_deg;
    end

    % 逐帧循环生成检测
    for k = 1:n_frames
        % 从真值航迹中提取当前时刻所有目标的状态
        states = build_target_states_at_time(truth_all, t_grid(k));

        % 调用多目标检测生成函数，生成含噪检测（目标 + 虚警）
        raw = generate_frame_detections_multi(rx_lon, rx_lat, tx_lon, tx_lat, states, k, t_grid(k), range_bias, az_bias, beam, params, range_noise, az_noise);

        % 将 raw 结构体数组转为 cell 数组，方便逐点后处理
        dets = cell(1, numel(raw));
        for d = 1:numel(raw)
            dp = raw(d);

            % 偏差校正：从原始量测中减去估计的系统偏差
            dp.drange = dp.prange - dr_est; dp.daz = dp.paz - da_est;
            % 校正后的量测覆盖原始量测字段
            dp.range_meas = dp.drange; dp.azimuth_meas = dp.daz;

            % 如果缺少 radial_vel_meas 字段或为 NaN，用 pvr 填充
            if ~isfield(dp, 'radial_vel_meas') || isnan(dp.radial_vel_meas), dp.radial_vel_meas = dp.pvr; end

            % 如果 lat/lon 为 NaN，用 bistatic 反解（校正后的量测）计算经纬度
            if isnan(dp.lat) || isnan(dp.lon), [~, dp.lat, dp.lon] = bistatic_inverse_solver(dp.drange, dp.daz, tx_lon, tx_lat, rx_lon, rx_lat); end

            % 计算原始（未校正）量测对应的经纬度，用于后续对比分析
            [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, tx_lon, tx_lat, rx_lon, rx_lat);

            % 保存处理后的点迹
            dets{d} = dp;
        end

        % 将 cell 数组合并为结构体数组（如果本帧有检测）
        if ~isempty(dets), detList{k} = [dets{:}]; end
    end
end

function a = wrap_angle(a)
% wrap_angle — 角度归一化到 [-180, 180] 范围
% 使用 mod 函数实现：先加 180 取模 360，再减 180
a = mod(a + 180, 360) - 180;
end
