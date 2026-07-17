function result = run_without_fusion(scenario_name)
    if nargin < 1 || isempty(scenario_name)
        % scenario_name = 'multi_cross';
        scenario_name = 'single_uturn';
    end

    addpath(genpath('.'));
    close all;

    fprintf('========== Phase 0: Oracle 场景初始化 ==========%s', newline);
    params = simulation_params_oracle();
    rng(params.random_seed);
    scenario = build_truth_scenario(scenario_name, params);
    truth_all = scenario.truth_all;
    truthTrajs = scenario.truthTrajs;
    t1_grid = scenario.t1_grid;
    t2_grid = scenario.t2_grid;
    n_frames = scenario.n_frames;
    fprintf('场景: %s | 目标数=%d | 帧数=%d | dt=%.0fs%s', ...
        scenario.name, scenario.n_targets, n_frames, params.dt_sec, newline);
    fprintf('雷达硬约束: Pd=%.2f, Pfa=%.4f%s', ...
        params.detection_probability, params.false_alarm_rate, newline);
    print_truth_summary_without_fusion(truthTrajs);

    fprintf('%s========== Phase 1: ADS-B 系统偏差标定 ==========%s', newline, newline);
    [dr1_est, da1_est, dr2_est, da2_est] = calibrate_bias_without_fusion(params);
    fprintf('R1 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr1_est, da1_est, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, newline);
    fprintf('R2 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr2_est, da2_est, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, newline);

    fprintf('%s========== Phase 2: 点迹生成 + 偏差校正 ==========%s', newline, newline);
    detList_R1 = generate_radar_detections_without_fusion( ...
        1, params, truth_all, t1_grid, n_frames, dr1_est, da1_est);
    detList_R2 = generate_radar_detections_without_fusion( ...
        2, params, truth_all, t2_grid, n_frames, dr2_est, da2_est);
    print_detection_summary_without_fusion(detList_R1, 'R1');
    print_detection_summary_without_fusion(detList_R2, 'R2');

    fprintf('%s========== Phase 3: 南阳式 Oracle 起始、关联与滤波 ==========%s', newline, newline);
    params_r1 = radar_params(params, 1);
    params_r2 = radar_params(params, 2);
    ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    fprintf('--- R1 Oracle 航迹维护 ---%s', newline);
    [trackList_R1, tempTrackList_R1, trackSnapshots_R1, diag_R1] = ...
        run_oracle_tracker_without_fusion(detList_R1, ukf1_tpl, params_r1, truth_all, t1_grid);
    fprintf('--- R2 Oracle 航迹维护 ---%s', newline);
    [trackList_R2, tempTrackList_R2, trackSnapshots_R2, diag_R2] = ...
        run_oracle_tracker_without_fusion(detList_R2, ukf2_tpl, params_r2, truth_all, t2_grid);
    print_track_summary_without_fusion(trackList_R1, 'R1', params);
    print_track_summary_without_fusion(trackList_R2, 'R2', params);
    print_consumption_summary(detList_R1, trackList_R1, 'R1');
    print_consumption_summary(detList_R2, trackList_R2, 'R2');
    validate_oracle_invariants(trackSnapshots_R1, detList_R1, diag_R1, params_r1, trackList_R1);
    validate_oracle_invariants(trackSnapshots_R2, detList_R2, diag_R2, params_r2, trackList_R2);
    fprintf('Oracle lifecycle invariants: R1/R2 通过%s', newline);

    fprintf('%s========== Phase 4: 单站滤波 RMSE ==========%s', newline, newline);
    errorStats_R1 = evaluate_all_multi('tracking_errors', trackSnapshots_R1, detList_R1, ...
        truthTrajs, n_frames, params.dt_sec, 'R1');
    errorStats_R2 = evaluate_all_multi('tracking_errors', trackSnapshots_R2, detList_R2, ...
        truthTrajs, n_frames, params.dt_sec, 'R2');
    print_tracking_rmse_without_fusion(errorStats_R1);
    print_tracking_rmse_without_fusion(errorStats_R2);

    result = struct('params', params, 'scenario', scenario, 'truth_all', {truth_all}, ...
        'truthTrajs', {truthTrajs}, 'detList_R1', {detList_R1}, 'detList_R2', {detList_R2}, ...
        'trackList_R1', {trackList_R1}, 'trackList_R2', {trackList_R2}, ...
        'tempTrackList_R1', tempTrackList_R1, 'tempTrackList_R2', tempTrackList_R2, ...
        'trackSnapshots_R1', {trackSnapshots_R1}, 'trackSnapshots_R2', {trackSnapshots_R2}, ...
        'diag_R1', {diag_R1}, 'diag_R2', {diag_R2}, ...
        'errorStats_R1', errorStats_R1, 'errorStats_R2', errorStats_R2);

    fprintf('%s========== Phase 5: Figure 1-4 可视化 ==========%s', newline, newline);
    plot_without_fusion_figures(result);
    fprintf('%sDone. 流水线已在单站滤波结束处停止。%s', newline, newline);
end

function plot_without_fusion_figures(result)
    [track_A, track_B, track_C] = truth_tracks_for_legacy_without_fusion(result.truth_all);
    plot_scene_overview_multi(track_A, track_B, track_C, result.params, 'results');
    plot_point_cloud_3d(result.detList_R1, 'R1', '');
    plot_point_cloud_3d(result.detList_R2, 'R2', '');
    plot_tracks_without_fusion(result.truth_all, result.detList_R1, result.detList_R2, ...
        result.trackSnapshots_R1, result.trackSnapshots_R2, ...
        result.trackList_R1, result.trackList_R2, result.params);
end

function [track_A, track_B, track_C] = truth_tracks_for_legacy_without_fusion(truth_all)
    empty_track = nan(1, 5);
    track_A = empty_track;
    track_B = empty_track;
    track_C = empty_track;
    if length(truth_all) >= 1, track_A = truth_all{1}; end
    if length(truth_all) >= 2, track_B = truth_all{2}; end
    if length(truth_all) >= 3, track_C = truth_all{3}; end
end

function print_truth_summary_without_fusion(truthTrajs)
    for a = 1:length(truthTrajs)
        tt = truthTrajs{a};
        fprintf('目标%s: 点数=%d, 时长=%.0fs%s', ...
            tt.label, length(tt.time_sec), tt.time_sec(end)-tt.time_sec(1), newline);
    end
end

function [dr1_est, da1_est, dr2_est, da2_est] = calibrate_bias_without_fusion(params)
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2;
    adsb_lon = T_adsb.Var3;
    dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat), continue; end
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
            t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            rg = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            az = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            dr1_list(end+1) = rg + params.radar1_range_bias_m + ...
                randn()*params.radar1_range_noise_std_m - rg;
            da1_list(end+1) = wrap_angle_without_fusion(az + params.radar1_azimuth_bias_deg + ...
                randn()*params.radar1_azimuth_noise_std_deg - az);
        end
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
            t_lon, t_lat, params.radar2_beam_center_deg, params);
        if in2
            rg = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat, t_lon, t_lat);
            az = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
            dr2_list(end+1) = rg + params.radar2_range_bias_m + ...
                randn()*params.radar2_range_noise_std_m - rg;
            da2_list(end+1) = wrap_angle_without_fusion(az + params.radar2_azimuth_bias_deg + ...
                randn()*params.radar2_azimuth_noise_std_deg - az);
        end
    end
    if isempty(dr1_list) || isempty(da1_list)
        error('run_without_fusion:calibrationNoSamples', ...
            'R1 ADS-B 标定没有雷达覆盖内的有效样本');
    end
    if isempty(dr2_list) || isempty(da2_list)
        error('run_without_fusion:calibrationNoSamples', ...
            'R2 ADS-B 标定没有雷达覆盖内的有效样本');
    end
    dr1_est = mean(dr1_list); da1_est = mean(da1_list);
    dr2_est = mean(dr2_list); da2_est = mean(da2_list);
    if any(~isfinite([dr1_est, da1_est, dr2_est, da2_est]))
        error('run_without_fusion:calibrationInvalid', 'ADS-B 标定结果包含非有限值');
    end
end

function detList = generate_radar_detections_without_fusion(radar_id, params, truth_all, t_grid, n_frames, dr_est, da_est)
    detList = cell(n_frames, 1);
    if radar_id == 1
        rng(params.random_seed + 1e7);
        rx_lon=params.radar1_lon; rx_lat=params.radar1_lat; tx_lon=params.radar1_tx_lon; tx_lat=params.radar1_tx_lat;
        range_bias=params.radar1_range_bias_m; az_bias=params.radar1_azimuth_bias_deg;
        beam=params.radar1_beam_center_deg; range_noise=params.radar1_range_noise_std_m; az_noise=params.radar1_azimuth_noise_std_deg;
    else
        rng(params.random_seed + 2e7);
        rx_lon=params.radar2_lon; rx_lat=params.radar2_lat; tx_lon=params.radar2_tx_lon; tx_lat=params.radar2_tx_lat;
        range_bias=params.radar2_range_bias_m; az_bias=params.radar2_azimuth_bias_deg;
        beam=params.radar2_beam_center_deg; range_noise=params.radar2_range_noise_std_m; az_noise=params.radar2_azimuth_noise_std_deg;
    end
    for k = 1:n_frames
        states = build_target_states_at_time(truth_all, t_grid(k));
        raw = generate_frame_detections_multi(rx_lon, rx_lat, tx_lon, tx_lat, states, ...
            k, t_grid(k), range_bias, az_bias, beam, params, range_noise, az_noise);
        dets = cell(1, length(raw));
        for d = 1:length(raw)
            dp = raw(d);
            dp.drange = dp.prange - dr_est;
            dp.daz = dp.paz - da_est;
            dp.range_meas = dp.drange;
            dp.azimuth_meas = dp.daz;
            if ~isfield(dp, 'radial_vel_meas') || isnan(dp.radial_vel_meas), dp.radial_vel_meas = dp.pvr; end
            if isnan(dp.lat) || isnan(dp.lon)
                [~, dp.lat, dp.lon] = bistatic_inverse_solver(dp.drange, dp.daz, tx_lon, tx_lat, rx_lon, rx_lat);
            end
            [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, tx_lon, tx_lat, rx_lon, rx_lat);
            dets{d} = dp;
        end
        if isempty(dets), detList{k} = []; else, detList{k} = [dets{:}]; end
    end
end


function [trackList, tempTrackList, snapshots, diagList] = run_oracle_tracker_without_fusion(detList, ukf_tpl, params, truth_all, t_grid)
    n_frames = length(detList);
    snapshots = cell(n_frames, 1); diagList = cell(n_frames, 1);
    trackList = {}; tempTrackList = struct([]); next_id = 1;
    for k = 1:n_frames
        [trackList, tempTrackList, snapshots{k}, next_id, diagList{k}] = TRACK_MAIN_ORACLE( ...
            trackList, tempTrackList, detList{k}, ukf_tpl, params, k, next_id, truth_all, t_grid);
        if k == 1 || mod(k, 10) == 0 || k == n_frames
            fprintf('  frame %3d/%3d: active=%d, total=%d%s', ...
                k, n_frames, count_active_without_fusion(trackList), length(trackList), newline);
        end
    end
    trackList = sortTrackList_oracle(trackList);
end

function n = count_active_without_fusion(trackList)
    n = 0;
    for i = 1:length(trackList), n = n + (trackList{i}.type ~= 7); end
end

function print_detection_summary_without_fusion(detList, label)
    target = 0; clutter = 0;
    for k = 1:length(detList)
        for i = 1:length(detList{k})
            if detList{k}(i).is_clutter, clutter=clutter+1; else, target=target+1; end
        end
    end
    fprintf('%s: 真实校准点=%d, 虚警=%d%s', label, target, clutter, newline);
end

function print_track_summary_without_fusion(trackList, label, params)
    history = 0; active = 0;
    for i = 1:length(trackList)
        if trackList{i}.type == params.HISTORY_TRACK, history=history+1; else, active=active+1; end
    end
    fprintf('%s: 总航迹=%d, 当前活跃=%d, 历史=%d%s', label, length(trackList), active, history, newline);
end

function print_consumption_summary(detList, trackList, label)
    calibrated = 0;
    for k = 1:length(detList)
        dets = detList{k};
        for j = 1:length(dets)
            if ~dets(j).is_clutter
                calibrated = calibrated + 1;
            end
        end
    end
    consumed_keys = {};
    for i = 1:length(trackList)
        if ~isfield(trackList{i}, 'asscPointList'), continue; end
        for j = 1:length(trackList{i}.asscPointList)
            dp = trackList{i}.asscPointList{j};
            if isempty(dp) || ~isfield(dp, 'frameID') || ~isfield(dp, 'aircraft_id'), continue; end
            consumed_keys{end+1} = sprintf('%d_%d', dp.frameID, double(dp.aircraft_id));
        end
    end
    consumed = length(unique(consumed_keys));
    fprintf('%s 点迹消费: 校准真实点=%d, 纳入航迹=%d, 未纳入=%d%s', ...
        label, calibrated, consumed, calibrated-consumed, newline);
end

function print_tracking_rmse_without_fusion(stats)
    fprintf('%s UKF RMSE:%s', stats.radar, newline);
    for a = 1:length(stats.summary)
        s = stats.summary(a).ukf;
        fprintf('  目标%d: n=%d, RMSE=%.1fkm, median=%.1fkm%s', a, s.n, s.rms, s.median, newline);
    end
end

function a = wrap_angle_without_fusion(a)
    while a > 180, a = a - 360; end
    while a < -180, a = a + 360; end
end
