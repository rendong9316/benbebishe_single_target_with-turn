% =========================================================================
% build_truth_scenario.m — 真值场景生成器
% =========================================================================
% 【功能】
%   根据场景名称（scenario_name）生成完整的真值航迹数据。
%   支持以下场景类型：
%     multi_cross    — 多目标交叉场景（3架飞机，航向交叉）
%     single_straight — 单目标直线场景
%     single_turn    — 单目标拐弯场景（约120°拐角）
%     single_uturn   — 单目标回头弯场景（180°转弯）
%
%   每个场景的输出包含：
%     truth_all  — N_targets × 1 cell，每个元素为 [lon, lat, lon_rate, lat_rate, time] 矩阵
%     truthTrajs — N_targets 结构体数组，包含 label、速度、时间序列、位置序列
%     t1_grid    — R1 采样时间网格（从 time_offset_radar1 开始，步长 dt_sec）
%     t2_grid    — R2 采样时间网格（从 time_offset_radar2 开始，步长 dt_sec）
%     n_frames   — 两雷达共同覆盖的帧数（取较短者）
%
% 【时间网格对齐】
%   R1 和 R2 的采样偏移不同（params.time_offset_radar1_sec vs
%   params.time_offset_radar2_sec），本函数计算两者的公共帧数 n_frames，
%   确保后续处理中两雷达的时间网格长度一致。
%
% 【输入】
%   scenario_name — 字符串，场景名称
%   params        — 仿真参数结构体，需含 aircraft_waypoints、
%                   aircraft_speed_ms、dt_sec、time_offset_* 等字段
%
% 【输出】
%   scenario — 结构体，包含 name、n_targets、truth_all、truthTrajs、
%              t1_grid、t2_grid、n_frames
% =========================================================================
function scenario = build_truth_scenario(scenario_name, params)
    % 默认场景名
    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'multi_cross';
    end

    % 根据场景名生成不同的航迹
    switch char(scenario_name)
        case 'multi_cross'
            % 多目标交叉场景：3架飞机，航向互相交叉
            % 目标A: 西南→东北 (128.8E,30.5N → 132.0E,32.5N)
            % 目标B: 西南→东北 (128.8E,32.5N → 132.0E,30.5N)
            % 目标C: 西→东北   (128.8E,31.5N → 130.5E,32.9N)
            waypoints = {
                [128.8, 30.5, 0; 132.0, 32.5, 0], ...
                [128.8, 32.5, 0; 132.0, 30.5, 0], ...
                [128.8, 31.5, 0; 130.5, 32.9, 0]};
            labels = {'A', 'B', 'C'};
            trajs = cell(3, 1);
            for i = 1:3
                trajs{i} = aircraft_trajectory_create(waypoints{i}, params.aircraft_speed_ms, params.dt_sec);
            end
        case 'single_straight'
            % 单目标直线场景：使用 params.aircraft_waypoints 定义的航路点
            labels = {'A'};
            trajs = {aircraft_trajectory_create(params.aircraft_waypoints, params.aircraft_speed_ms, params.dt_sec)};
        case 'single_turn'
            % 单目标拐弯场景：约120°拐角
            labels = {'A'};
            trajs = cell(1, 1);
            trajs{1} = aircraft_trajectory_create('gradual_turn', params);
        case {'single_uturn', 'single_u_turn'}
            % 单目标回头弯场景：180°左转半圆
            labels = {'A'};
            trajs = cell(1, 1);
            trajs{1} = aircraft_trajectory_create('uturn', params);
        otherwise
            error('build_truth_scenario: unknown scenario "%s"', scenario_name);
    end

    % ---- 对每条航迹进行插值，生成均匀时间采样的真值矩阵 ----
    n_targets = length(trajs);
    truth_all = cell(n_targets, 1);
    truthTrajs = cell(n_targets, 1);
    max_duration = 0;
    for ac = 1:n_targets
        % aircraft_trajectory_interpolate('generate', ...) 返回 N×5 矩阵：
        % [lon, lat, lon_rate, lat_rate, time_sec]
        tt = aircraft_trajectory_interpolate('generate', trajs{ac});
        truth_all{ac} = tt;
        max_duration = max(max_duration, tt(end, 5));
        % 组装 truthTrajs 结构体，供后续插值和评估使用
        truthTrajs{ac} = struct('label', labels{ac}, ...
            'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:, 5), ...
            'lat', tt(:, 2), ...
            'lon', tt(:, 1), ...
            'lon_rate', tt(:, 3), ...
            'lat_rate', tt(:, 4));
    end

    % ---- 计算两雷达的时间网格 ----
    % R1 和 R2 可能有不同的起始偏移，各自生成时间网格
    t1_grid = params.time_offset_radar1_sec : params.dt_sec : max_duration;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : max_duration;
    % 取公共帧数（较短的网格长度）
    n_frames = min(length(t1_grid), length(t2_grid));
    t1_grid = t1_grid(1:n_frames);
    t2_grid = t2_grid(1:n_frames);

    % ---- 组装输出场景结构体 ----
    scenario = struct();
    scenario.name = char(scenario_name);
    scenario.n_targets = n_targets;
    scenario.truth_all = truth_all;
    scenario.truthTrajs = truthTrajs;
    scenario.t1_grid = t1_grid;
    scenario.t2_grid = t2_grid;
    scenario.n_frames = n_frames;
end
