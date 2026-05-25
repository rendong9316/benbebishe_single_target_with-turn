%% =========================================================================
% run_simulation.m
% 双基地外辐射源雷达单目标逐帧仿真主程序
% =========================================================================
% Phase 0: 场景初始化（单机航迹 + 覆盖检查）
% Phase 1: 系统偏差离线标定
% Phase 2: 原始点迹生成（含偏差，不做校正）
% Phase 3: 时间对齐策略（航迹级，延后到匹配前）
% Phase 4: 偏差校正（几何反解）
% Phase 5: 单目标航迹跟踪（UKF+PDA+模糊自适应Q）
% Phase 6: 航迹级时间对齐（R2→R1时间网格）
% Phase 7: 航迹融合（SCC/BC/CI/FCI，直接1对1）
% Phase 8: 定量误差评估（融合 + 单站）
% Phase 9: 可视化 + 数据保存
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ==================== Phase 0: 场景初始化 ====================
fprintf('========== Phase 0: 场景初始化 ==========\n');

params = simulation_params();
rng(params.random_seed);

% 单机航迹生成
traj = aircraft_trajectory_create(params.aircraft_waypoints, ...
    params.aircraft_speed_ms, params.dt_sec);
true_track = aircraft_trajectory_interpolate('generate', traj);
fprintf('真实航迹: %d 点, 总时长 %.0f s, 速度 %.0f m/s\n', ...
    size(true_track,1), traj.duration_sec, params.aircraft_speed_ms);

% 覆盖检查
n_in_r1 = 0; n_in_r2 = 0;
for i = 1:size(true_track, 1)
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track(i,1), true_track(i,2), params.radar1_beam_center_deg, params);
    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track(i,1), true_track(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1 = n_in_r1 + 1; end
    if in2, n_in_r2 = n_in_r2 + 1; end
end
fprintf('  在R1威力内: %d 点, 在R2威力内: %d 点 (共%d点)\n', ...
    n_in_r1, n_in_r2, size(true_track,1));

% 时间网格
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('仿真帧数: %d (dt=%.0fs)\n', n_frames, params.dt_sec);

% 真值结构体 (用于误差评估)
tt = true_track;
truthTraj = struct('label', 'A', 'speed_ms', params.aircraft_speed_ms, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));

%% ==================== Phase 1: ADS-B系统偏差标定 ====================
fprintf('\n========== Phase 1: ADS-B系统偏差标定 ==========\n');

rng(params.random_seed);

fprintf('加载ADS-B合作目标: %s\n', params.adsb_csv_path);
T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
adsb_lat = T_adsb.Var2;
adsb_lon = T_adsb.Var3;

dr1_list = []; da1_list = [];
dr2_list = []; da2_list = [];

n_check = min(5000, height(T_adsb));
cal_step = max(1, floor(height(T_adsb) / n_check));

for idx = 1:cal_step:height(T_adsb)
    t_lon = adsb_lon(idx);  t_lat = adsb_lat(idx);
    if isnan(t_lon) || isnan(t_lat), continue; end

    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        t_lon, t_lat, params.radar1_beam_center_deg, params);
    if in1
        r0 = sphere_utils_haversine_distance(params.radar1_tx_lon, params.radar1_tx_lat, t_lon, t_lat);
        r1 = sphere_utils_haversine_distance(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        Rg_true = r0 + r1;
        az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
        az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
        dr1_list(end+1) = Rg_meas - Rg_true;
        daz = az_meas - az_true;
        if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
        da1_list(end+1) = daz;
    end

    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        t_lon, t_lat, params.radar2_beam_center_deg, params);
    if in2
        r0 = sphere_utils_haversine_distance(params.radar2_tx_lon, params.radar2_tx_lat, t_lon, t_lat);
        r1 = sphere_utils_haversine_distance(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
        Rg_true = r0 + r1;
        az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
        Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
        az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
        dr2_list(end+1) = Rg_meas - Rg_true;
        daz = az_meas - az_true;
        if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
        da2_list(end+1) = daz;
    end
end

dr1_est = mean(dr1_list);  da1_est = mean(da1_list);
dr2_est = mean(dr2_list);  da2_est = mean(da2_list);
fprintf('ADS-B标校点数: R1=%d, R2=%d\n', length(dr1_list), length(dr2_list));
fprintf('R1: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr1_est, params.radar1_range_bias_m, da1_est, params.radar1_azimuth_bias_deg);
fprintf('R2: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr2_est, params.radar2_range_bias_m, da2_est, params.radar2_azimuth_bias_deg);

%% ==================== Phase 2: 原始点迹生成 ====================
fprintf('\n========== Phase 2: 原始点迹生成 ==========\n');

detRaw_R1 = cell(n_frames, 1);
detRaw_R2 = cell(n_frames, 1);

for k = 1:n_frames
    % R1
    rng(params.random_seed + k);
    [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
    detRaw_R1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, ...
        pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    for d = 1:length(detRaw_R1{k})
        detRaw_R1{k}(d).aircraft_id = 1;
    end

    % R2
    rng(params.random_seed + 10000 + k);
    [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
    detRaw_R2{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, ...
        pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
        params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
        params.radar2_beam_center_deg, params, ...
        params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
    for d = 1:length(detRaw_R2{k})
        detRaw_R2{k}(d).aircraft_id = 1;
    end
end

fprintf('原始点迹生成完成: R1共%d帧, R2共%d帧\n', n_frames, n_frames);

%% ==================== Phase 3: 时间对齐策略 ====================
fprintf('\n========== Phase 3: 时间对齐策略 ==========\n');
fprintf('R1采样: 0s/30s/60s/...  R2采样: 13s/43s/73s/...  偏移=%ds\n', ...
    params.time_offset_radar2_sec);
fprintf('策略: 点迹不做对齐, 两部雷达各自在原时间网格上滤波跟踪\n');
fprintf('      航迹级对齐延后到 Phase 6 融合前, 用 CV 模型全状态外推\n');

%% ==================== Phase 4: 偏差校正 + 几何反解 ====================
fprintf('\n========== Phase 4: 偏差校正 ==========\n');

detList_R1 = cell(n_frames, 1);
detList_R2 = cell(n_frames, 1);

for k = 1:n_frames
    % R1: 偏差校正
    dets_r1 = detRaw_R1{k};
    for d = 1:length(dets_r1)
        Rgc = dets_r1(d).prange - dr1_est;
        azc = dets_r1(d).paz - da1_est;
        dets_r1(d).drange = Rgc;
        dets_r1(d).daz = azc;
        dets_r1(d).range_meas = Rgc;
        dets_r1(d).azimuth_meas = azc;
        if ~(isfield(dets_r1(d), 'lat') && ~isnan(dets_r1(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets_r1(d).lat = lat_e;
            dets_r1(d).lon = lon_e;
        end
        [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r1(d).prange, dets_r1(d).paz, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat);
        dets_r1(d).raw_lat = raw_lat;
        dets_r1(d).raw_lon = raw_lon;
    end
    detList_R1{k} = dets_r1;

    % R2: 偏差校正
    dets_r2 = detRaw_R2{k};
    for d = 1:length(dets_r2)
        Rgc = dets_r2(d).prange - dr2_est;
        azc = dets_r2(d).paz - da2_est;
        dets_r2(d).drange = Rgc;
        dets_r2(d).daz = azc;
        dets_r2(d).range_meas = Rgc;
        dets_r2(d).azimuth_meas = azc;
        if ~(isfield(dets_r2(d), 'lat') && ~isnan(dets_r2(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets_r2(d).lat = lat_e;
            dets_r2(d).lon = lon_e;
        end
        [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r2(d).prange, dets_r2(d).paz, ...
            params.radar2_tx_lon, params.radar2_tx_lat, ...
            params.radar2_lon, params.radar2_lat);
        dets_r2(d).raw_lat = raw_lat;
        dets_r2(d).raw_lon = raw_lon;
    end
    detList_R2{k} = dets_r2;
end

fprintf('偏差校正完成: R1=%d帧, R2=%d帧\n', n_frames, n_frames);

%% ==================== Phase 5: 单目标航迹跟踪 ====================
fprintf('\n========== Phase 5: 单目标航迹跟踪 ==========\n');

% R1 UKF (precision station)
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale = 5e4;
params.ukf_P_pos_std = 0.2;
params.ukf_P_vel_std = 0.004;
params.gate_sigma = 2.0;
ukf1_tpl = ukf_filter(params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

% R2 params (standard station, ~2x noise of R1)
params_r2 = params;
params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
params_r2.gate_sigma = 2.5;
params_r2.ukf_Q_scale = 1e5;
params_r2.ukf_P_pos_std = 0.3;
params_r2.ukf_P_vel_std = 0.005;
params_r2.tracker_M = 4;
params_r2.tracker_N = 8;
params_r2.tracker_K_loss = 12;

ukf2_tpl = ukf_filter(params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

trackList_R1 = {};  trackList_R2 = {};
trackSnapshots_R1 = cell(n_frames, 1);
trackSnapshots_R2 = cell(n_frames, 1);

ac_det_count_r1 = 0;  ac_det_count_r2 = 0;

for k = 1:n_frames
    for d = 1:length(detList_R1{k})
        if ~detList_R1{k}(d).is_clutter
            ac_det_count_r1 = ac_det_count_r1 + 1;
        end
    end
    for d = 1:length(detList_R2{k})
        if ~detList_R2{k}(d).is_clutter
            ac_det_count_r2 = ac_det_count_r2 + 1;
        end
    end
end

% 单目标简化跟踪 (无M/N, 直接初始化)
[trackSnapshots_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames);
[trackSnapshots_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames);
trackList_R1 = {finalTrk1};
trackList_R2 = {finalTrk2};

fprintf('跟踪完成: %d 帧\n', n_frames);
fprintf('  R1目标检出=%d, R2目标检出=%d\n', ac_det_count_r1, ac_det_count_r2);

fprintf('\n--- 航迹统计 ---\n');
fprintf('R1: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk1.type), finalTrk1.quality, finalTrk1.life);
fprintf('R2: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk2.type), finalTrk2.quality, finalTrk2.life);

% ---- 关联诊断 ----
for radar_label = {'R1', 'R2'}
    snaps = trackSnapshots_R1;
    if strcmp(radar_label{1}, 'R2'), snaps = trackSnapshots_R2; end
    n_assoc = 0; n_predict = 0; n_init = 0; n_lost = 0;
    init_frame = 0; nis_vals = [];
    for k = 1:length(snaps)
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 6, n_init = n_init + 1;
        elseif trk.type == 1
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;
            else
                n_predict = n_predict + 1;
            end
            if isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
            end
        elseif trk.type == 7, n_lost = n_lost + 1; end
        if init_frame == 0 && trk.type == 1, init_frame = k; end
    end
    n_tracked = n_assoc + n_predict;
    fprintf('%s: 起始帧=%d | 关联=%d 纯预测=%d (关联率=%.0f%%) | 起始中=%d 丢失=%d\n', ...
        radar_label{1}, init_frame, n_assoc, n_predict, ...
        n_assoc/max(1,n_tracked)*100, n_init, n_lost);
    if ~isempty(nis_vals)
        nis_in_gate = sum(nis_vals < 4*2);
        fprintf('  NIS: 均值=%.2f 门内=%.0f%% (%d/%d)\n', ...
            mean(nis_vals), nis_in_gate/length(nis_vals)*100, nis_in_gate, length(nis_vals));
    end
end
fprintf('\n');

%% ==================== Phase 6: 航迹级时间对齐 ====================
fprintf('\n========== Phase 6: 航迹级时间对齐 ==========\n');
fprintf('将R2航迹 (t2_grid) 用CV模型全状态外推到R1时间网格 (t1_grid)\n');

aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
fprintf('R2航迹时间对齐完成\n');

%% ==================== Phase 7: 航迹融合 ====================
fprintf('\n========== Phase 7: 航迹融合 (四种算法) ==========\n');

% 单目标: 直接1对1融合, 各站严格1条航迹 (ID=1)
r1_id = 1; r2_id = 1;
fprintf('融合对: R1#1 <-> R2#1 (直接1对1)\n');

% 构建单对匹配
matched_pair = struct('R1_track_id', r1_id, 'R2_track_id', r2_id, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);

method_names = {'SCC', 'BC', 'CI', 'FCI'};
all_fused_snapshots = cell(length(method_names), 1);

for m = 1:length(method_names)
    method = method_names{m};
    fprintf('  运行 %s 融合...\n', method);
    all_fused_snapshots{m} = run_track_fusion(matched_pair, ...
        trackSnapshots_R1, aligned_R2, params, method);
end
fprintf('融合完成: %d 种算法\n', length(method_names));

%% ==================== Phase 8: 定量误差评估 ====================
fprintf('\n========== Phase 8: 定量误差评估 ==========\n');

% 构建matcher结构体 (用于evaluate_fusion, 简化版)
n_frames_val = n_frames;
matcher_simple = struct();
matcher_simple.matched_pairs = matched_pair;
matcher_simple.aligned_R2 = aligned_R2;
matcher_simple.r1_ids = r1_id;
matcher_simple.r2_ids = r2_id;

% 提取R1航迹位置历史 (用于evaluate_fusion)
r1_pos = nan(1, n_frames, 2);
for k = 1:n_frames
    snap = trackSnapshots_R1{k};
    if ~isempty(snap.trackList)
        for t = 1:length(snap.trackList)
            if snap.trackList{t}.id == r1_id
                r1_pos(1, k, 1) = snap.trackList{t}.lon;
                r1_pos(1, k, 2) = snap.trackList{t}.lat;
                break;
            end
        end
    end
end
matcher_simple.r1_pos = r1_pos;

r2_pos = nan(1, n_frames, 2);
for k = 1:n_frames
    snap = aligned_R2{k};
    if ~isempty(snap.trackList)
        for t = 1:length(snap.trackList)
            if snap.trackList{t}.id == r2_id
                r2_pos(1, k, 1) = snap.trackList{t}.lon;
                r2_pos(1, k, 2) = snap.trackList{t}.lat;
                break;
            end
        end
    end
end
matcher_simple.r2_pos = r2_pos;

% 融合误差评估 (用evaluate_fusion, 传入单目标真值)
truthTrajs = {truthTraj};
fusion_eval = evaluate_all.evaluate_fusion(all_fused_snapshots, method_names, ...
    matched_pair, trackSnapshots_R1, trackSnapshots_R2, ...
    truthTrajs, n_frames, params.dt_sec, matcher_simple);

% 打印融合 vs 单站对比表
fprintf('\n--- 融合误差对比 (RMSE km) ---\n');
fprintf('%-8s %8s %8s\n', '算法', 'RMSE', '中位');
fprintf('%-8s %8s %8s\n', '------', '------', '------');
all_method_labels = [method_names, {'R1_only', 'R2_only'}];
for m = 1:length(all_method_labels)
    s = fusion_eval.overall(m).s;
    fprintf('%-8s %8.1f %8.1f\n', all_method_labels{m}, s.rms, s.median);
end

rms_vals = arrayfun(@(x) x.s.rms, fusion_eval.overall(1:4));
[best_fusion_rmse, best_m] = min(rms_vals);
r1_rmse = fusion_eval.overall(5).s.rms;
r2_rmse = fusion_eval.overall(6).s.rms;
fprintf('\n最佳融合算法: %s (RMSE=%.1fkm)\n', method_names{best_m}, best_fusion_rmse);
fprintf('融合 vs R1(精密站): %+.1f%%\n', (1 - best_fusion_rmse/r1_rmse)*100);
fprintf('融合 vs R2(普通站): %+.1f%% 改善\n', (1 - best_fusion_rmse/r2_rmse)*100);

% 单站跟踪误差 (时间对齐后评估)
aligned_R2_eval = time_align_tracks(trackSnapshots_R2, params);
errorStats_R1 = evaluate_all.compute_tracking_errors(trackSnapshots_R1, detList_R1, ...
    truthTrajs, n_frames, params.dt_sec, 'R1');
errorStats_R2 = evaluate_all.compute_tracking_errors(aligned_R2_eval, detList_R2, ...
    truthTrajs, n_frames, params.dt_sec, 'R2');

for es = {errorStats_R1, errorStats_R2}
    e = es{1};
    fprintf('\n--- %s UKF滤波误差 ---\n', e.radar);
    fprintf('%-6s %6s %8s %8s %8s %8s %8s\n', ...
        '飞机', '点数', '中位(km)', '均值(km)', 'RMSE(km)', '95%(km)', 'vs检测');
    s_u = e.summary(1).ukf;
    fprintf('飞机A   %6d %8.1f %8.1f %8.1f %8.1f %7.0f%%\n', ...
        s_u.n, s_u.median, s_u.mean, s_u.rms, s_u.pct95, ...
        e.summary(1).ukf_vs_det_pct);
end

%% ==================== Phase 9: 可视化 + 数据保存 ====================
fprintf('\n========== Phase 9: 可视化 ==========\n');
if ~exist('results', 'dir'), mkdir('results'); end

% 暂禁MATLAB 2026a内部UI尺寸警告 (不影响实际出图)
warn_state = warning('off', 'all');

plot_scene_overview(true_track, params, 'results');
plot_point_cloud_3d(detList_R1, 'R1', 'results/fig2a_R1_point_cloud.png');
plot_point_cloud_3d(detList_R2, 'R2', 'results/fig2b_R2_point_cloud.png');

% 单目标跟踪综合图
plot_results('single_track', true_track, detList_R1, detList_R2, ...
    trackSnapshots_R1, trackSnapshots_R2, params, 'results');

% 融合可视化
plot_results('single_fusion', true_track, trackSnapshots_R1, trackSnapshots_R2, ...
    all_fused_snapshots, method_names, best_m, fusion_eval, truthTraj, params, 'results');

warning(warn_state);  % 恢复警告状态

fprintf('\n========== Phase 9: 数据保存 ==========\n');
sysPara = struct(...
    'dt_sec', params.dt_sec, 'n_frames', n_frames, ...
    'R1_lon', params.radar1_lon, 'R1_lat', params.radar1_lat, ...
    'R1_tx_lon', params.radar1_tx_lon, 'R1_tx_lat', params.radar1_tx_lat, ...
    'R1_beam_center_deg', params.radar1_beam_center_deg, ...
    'R1_range_bias_m', params.radar1_range_bias_m, ...
    'R1_azimuth_bias_deg', params.radar1_azimuth_bias_deg, ...
    'R2_lon', params.radar2_lon, 'R2_lat', params.radar2_lat, ...
    'R2_tx_lon', params.radar2_tx_lon, 'R2_tx_lat', params.radar2_tx_lat, ...
    'R2_beam_center_deg', params.radar2_beam_center_deg, ...
    'R2_range_bias_m', params.radar2_range_bias_m, ...
    'R2_azimuth_bias_deg', params.radar2_azimuth_bias_deg, ...
    'beam_width_deg', params.beam_width_deg, ...
    'range_km', [params.range_min_km, params.range_max_km], ...
    'detection_probability', params.detection_probability, ...
    'false_alarm_rate', params.false_alarm_rate, ...
    'radar1_range_noise_m', params.radar1_range_noise_std_m, ...
    'radar1_az_noise_deg', params.radar1_azimuth_noise_std_deg, ...
    'radar2_range_noise_m', params.radar2_range_noise_std_m, ...
    'radar2_az_noise_deg', params.radar2_azimuth_noise_std_deg, ...
    'radial_vel_noise_std_ms', params.radial_vel_noise_std_ms, ...
    'random_seed', params.random_seed);

calibResult = struct(...
    'dr1_est', dr1_est, 'da1_est', da1_est, ...
    'dr2_est', dr2_est, 'da2_est', da2_est, ...
    'dr1_true', params.radar1_range_bias_m, 'da1_true', params.radar1_azimuth_bias_deg, ...
    'dr2_true', params.radar2_range_bias_m, 'da2_true', params.radar2_azimuth_bias_deg, ...
    'n_cal_R1', length(dr1_list), 'n_cal_R2', length(dr2_list));

R1 = struct('detRaw', {detRaw_R1}, 'detList', {detList_R1}, ...
    'trackSnapshots', {trackSnapshots_R1}, 'finalTrack', finalTrk1, ...
    'targetDetCount', ac_det_count_r1);
R2 = struct('detRaw', {detRaw_R2}, 'detList', {detList_R2}, ...
    'trackSnapshots', {trackSnapshots_R2}, 'finalTrack', finalTrk2, ...
    'targetDetCount', ac_det_count_r2);

outf = fullfile('results', sprintf('simulation_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'sysPara', 'calibResult', 'truthTraj', 'R1', 'R2', 'params', ...
    'errorStats_R1', 'errorStats_R2', 'fusion_eval', ...
    'all_fused_snapshots', 'method_names');
fprintf('数据已保存: %s\n', outf);
fprintf('\nDone.\n');

% =========================================================================
% 内部函数
% =========================================================================

function s = get_type_str(t)
    switch t
        case 1, s = 'RELIABLE';
        case 2, s = 'MAINTAIN';
        case 6, s = 'TEMPORARY';
        case 7, s = 'HISTORY';
        otherwise, s = 'UNKNOWN';
    end
end

function idx = find_active_tracks(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7
            idx(end+1) = t;
        end
    end
end

function idx = find_reliable(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type == 1
            idx(end+1) = t;
        end
    end
end
