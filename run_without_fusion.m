function result = run_without_fusion(scenario_name)
    % 无融合模式主入口：执行单站跟踪全流程，不包含跨雷达匹配和融合
    %
    % 与 run.m 的区别：
    %   - 移除了 Phase 5-8（时间对齐、航迹匹配、融合、融合评估）
    %   - 移除了 Phase 9 的融合可视化
    %   - 增加了点迹消耗统计（print_consumption_summary）
    %   - 默认场景为 single_uturn（回头弯）

    % 参数检查：若调用方未传场景名，使用默认的多目标交叉场景
    if nargin < 1 || isempty(scenario_name)
        % 默认使用回头弯场景（单目标、高机动）
        scenario_name = 'multi_cross';
    end

    % 将所有子函数所在目录加入搜索路径
    addpath(genpath('.'));
    % 关闭所有已打开的图窗，避免新旧仿真结果混杂
    close all;

    % ====== Phase 0: Oracle 场景初始化 ======
    % 调用 prepare_oracle_tracking_inputs 一次性加载参数、场景、真值、时间网格、偏差标定等
    fprintf('========== Phase 0: Oracle 场景初始化 ==========%s', newline);
    inputs = prepare_oracle_tracking_inputs(scenario_name);
    params = inputs.params;                               % 全局仿真参数
    scenario = inputs.scenario;                           % 场景元信息
    truth_all = inputs.truth_all;                         % 真值航迹
    truthTrajs = inputs.truthTrajs;                       % 真值轨迹结构体数组
    t1_grid = inputs.t1_grid;                             % R1 时间网格
    t2_grid = inputs.t2_grid;                             % R2 时间网格
    n_frames = scenario.n_frames;                         % 总帧数
    fprintf('场景: %s | 目标数=%d | 帧数=%d | dt=%.0fs%s', ...
        scenario.name, scenario.n_targets, n_frames, params.dt_sec, newline);
    fprintf('雷达硬约束: Pd=%.2f, Pfa=%.4f%s', ...
        params.detection_probability, params.false_alarm_rate, newline);
    print_truth_summary_without_fusion(truthTrajs);

    % ====== Phase 1: ADS-B 系统偏差标定 ======
    % 偏差估计已在 prepare_oracle_tracking_inputs 中完成，此处直接提取
    dr1_est = inputs.bias_estimates(1);                   % R1 距离偏差
    da1_est = inputs.bias_estimates(2);                   % R1 方位偏差
    dr2_est = inputs.bias_estimates(3);                   % R2 距离偏差
    da2_est = inputs.bias_estimates(4);                   % R2 方位偏差
    fprintf('R1 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr1_est, da1_est, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, newline);
    fprintf('R2 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr2_est, da2_est, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, newline);

    % ====== Phase 2: 点迹生成 + 偏差校正 ======
    % 点迹生成同样在 prepare_oracle_tracking_inputs 中已完成
    detList_R1 = inputs.detList_R1;                       % R1 逐帧检测列表
    detList_R2 = inputs.detList_R2;                       % R2 逐帧检测列表
    print_detection_summary_without_fusion(detList_R1, 'R1');
    print_detection_summary_without_fusion(detList_R2, 'R2');

    % ====== Phase 3: 南阳式 Oracle 起始、关联与滤波 ======
    % 提取 R1/R2 的雷达专属参数和 UKF 滤波器模板
    params_r1 = inputs.params_r1;
    params_r2 = inputs.params_r2;
    ukf1_tpl = inputs.ukf1_tpl;
    ukf2_tpl = inputs.ukf2_tpl;

    % 执行 Oracle 航迹维护（每站独立）
    fprintf('--- R1 Oracle 航迹维护 ---%s', newline);
    % 最后一个参数 true 表示启用 verbose 模式，逐帧打印进度
    [trackList_R1, tempTrackList_R1, trackSnapshots_R1, diag_R1] = ...
        run_oracle_tracker_sequence(detList_R1, ukf1_tpl, params_r1, truth_all, t1_grid, true);
    fprintf('--- R2 Oracle 航迹维护 ---%s', newline);
    [trackList_R2, tempTrackList_R2, trackSnapshots_R2, diag_R2] = ...
        run_oracle_tracker_sequence(detList_R2, ukf2_tpl, params_r2, truth_all, t2_grid, true);

    print_track_summary_without_fusion(trackList_R1, 'R1', params);
    print_track_summary_without_fusion(trackList_R2, 'R2', params);
    % 输出点迹消耗统计：多少个校准点被航迹关联，多少个未使用
    print_consumption_summary(detList_R1, trackList_R1, 'R1');
    print_consumption_summary(detList_R2, trackList_R2, 'R2');

    % 验证 Oracle 不变量：点迹不重复消耗、快照字段完整、生命周期事件合法等
    validate_oracle_invariants(trackSnapshots_R1, detList_R1, diag_R1, params_r1, trackList_R1);
    validate_oracle_invariants(trackSnapshots_R2, detList_R2, diag_R2, params_r2, trackList_R2);
    fprintf('Oracle lifecycle invariants: R1/R2 通过%s', newline);

    % ====== Phase 4: 单站滤波 RMSE ======
    % 分别计算 R1/R2 的单站跟踪 RMSE
    errorStats_R1 = evaluate_all_multi('tracking_errors', trackSnapshots_R1, detList_R1, ...
        truthTrajs, t1_grid, t1_grid, 'R1');
    errorStats_R2 = evaluate_all_multi('tracking_errors', trackSnapshots_R2, detList_R2, ...
        truthTrajs, t2_grid, t2_grid, 'R2');
    print_tracking_rmse_without_fusion(errorStats_R1);
    print_tracking_rmse_without_fusion(errorStats_R2);

    % 组装返回结果（不含融合相关字段）
    result = struct('params', params, 'scenario', scenario, 'truth_all', {truth_all}, ...
        'truthTrajs', {truthTrajs}, 'detList_R1', {detList_R1}, 'detList_R2', {detList_R2}, ...
        'trackList_R1', {trackList_R1}, 'trackList_R2', {trackList_R2}, ...
        'tempTrackList_R1', tempTrackList_R1, 'tempTrackList_R2', tempTrackList_R2, ...
        'trackSnapshots_R1', {trackSnapshots_R1}, 'trackSnapshots_R2', {trackSnapshots_R2}, ...
        'diag_R1', {diag_R1}, 'diag_R2', {diag_R2}, ...
        'errorStats_R1', errorStats_R1, 'errorStats_R2', errorStats_R2);

    % ====== Phase 5: Figure 1-4 可视化 ======
    % 仅绘制单站跟踪结果图（无融合对比）
    plot_without_fusion_figures(result);
    fprintf('%sDone. 流水线已在单站滤波结束处停止。%s', newline, newline);
end

% ========== 以下是辅助函数 ==========

function plot_without_fusion_figures(result)
    % 绘制无融合模式的可视化图：场景总览、点迹云、航迹图
    % 先提取真值轨迹用于 legacy 绘图接口
    [track_A, track_B, track_C] = truth_tracks_for_legacy_without_fusion(result.truth_all);
    plot_scene_overview_multi(track_A, track_B, track_C, result.params, 'results');
    plot_point_cloud_3d(result.detList_R1, 'R1', '');
    plot_point_cloud_3d(result.detList_R2, 'R2', '');
    % 绘制 R1/R2 的航迹对比图
    plot_tracks_without_fusion(result.truth_all, result.detList_R1, result.detList_R2, ...
        result.trackSnapshots_R1, result.trackSnapshots_R2, ...
        result.trackList_R1, result.trackList_R2, result.params);
end

function [track_A, track_B, track_C] = truth_tracks_for_legacy_without_fusion(truth_all)
    % 将 truth_all 拆分为 A/B/C 三个变量（最多 3 个目标），用于 legacy 绘图接口
    empty_track = nan(1, 5);                              % 空航迹占位符
    track_A = empty_track;
    track_B = empty_track;
    track_C = empty_track;
    if length(truth_all) >= 1, track_A = truth_all{1}; end
    if length(truth_all) >= 2, track_B = truth_all{2}; end
    if length(truth_all) >= 3, track_C = truth_all{3}; end
end

function print_truth_summary_without_fusion(truthTrajs)
    % 打印每个目标的点数和飞行时长
    for a = 1:length(truthTrajs)
        tt = truthTrajs{a};
        fprintf('目标%s: 点数=%d, 时长=%.0fs%s', ...
            tt.label, length(tt.time_sec), tt.time_sec(end)-tt.time_sec(1), newline);
    end
end

function print_detection_summary_without_fusion(detList, label)
    % 统计检测点迹中真实目标和杂波的数量
    target = 0; clutter = 0;
    for k = 1:length(detList)                             % 逐帧遍历
        for i = 1:length(detList{k})
            if detList{k}(i).is_clutter, clutter=clutter+1; else, target=target+1; end
        end
    end
    fprintf('%s: 真实校准点=%d, 虚警=%d%s', label, target, clutter, newline);
end

function print_track_summary_without_fusion(trackList, label, params)
    % 统计航迹列表中活跃航迹和历史航迹的数量
    history = 0; active = 0;
    for i = 1:length(trackList)
        % type == HISTORY_TRACK 表示航迹已归档为历史，其余为当前活跃
        if trackList{i}.type == params.HISTORY_TRACK, history=history+1; else, active=active+1; end
    end
    fprintf('%s: 总航迹=%d, 当前活跃=%d, 历史=%d%s', label, length(trackList), active, history, newline);
end

function print_consumption_summary(detList, trackList, label)
    % 统计校准检测中被航迹关联消耗的数量 vs 未被使用的数量
    calibrated = 0;                                       % 校准后的真实检测总数
    for k = 1:length(detList)                             % 逐帧遍历检测列表
        dets = detList{k};
        for j = 1:length(dets)
            if ~dets(j).is_clutter                        % 只统计真实目标检测
                calibrated = calibrated + 1;
            end
        end
    end
    % 收集所有航迹关联的点帧-飞机 ID 键值
    consumed_keys = {};
    for i = 1:length(trackList)
        % 若航迹没有 asscPointList 字段则跳过
        if ~isfield(trackList{i}, 'asscPointList'), continue; end
        for j = 1:length(trackList{i}.asscPointList)
            dp = trackList{i}.asscPointList{j};           % 关联点迹
            % 每个点迹需包含 frameID 和 aircraft_id
            if isempty(dp) || ~isfield(dp, 'frameID') || ~isfield(dp, 'aircraft_id'), continue; end
            consumed_keys{end+1} = sprintf('%d_%d', dp.frameID, double(dp.aircraft_id));
        end
    end
    consumed = length(unique(consumed_keys));             % 去重后的消耗数量
    fprintf('%s 点迹消费: 校准真实点=%d, 纳入航迹=%d, 未纳入=%d%s', ...
        label, calibrated, consumed, calibrated-consumed, newline);
end

function print_tracking_rmse_without_fusion(stats)
    % 打印单站 UKF 跟踪 RMSE 统计（每个目标 + 总体）
    fprintf('%s UKF RMSE:%s', stats.radar, newline);
    for a = 1:length(stats.summary)
        s = stats.summary(a).ukf;
        fprintf('  目标%d: n=%d, RMSE=%.1fkm, median=%.1fkm%s', a, s.n, s.rms, s.median, newline);
    end
end
