%% =========================================================================
% run_simulation_turn.m
% 拐弯目标仿真主程序 — 基础UKF vs 机动自适应UKF 对比
% =========================================================================
% Phase 0: 场景初始化 (拐弯航迹 + 覆盖检查)
% Phase 1: 系统偏差离线标定
% Phase 2: 原始点迹生成
% Phase 3: 时间对齐策略
% Phase 4: 偏差校正gye 
% Phase 5: 单目标航迹跟踪 (基础UKF + 机动自适应UKF)
% Phase 6: 航迹级时间对齐
% Phase 7: 航迹融合 (两组)
% Phase 8: 定量误差评估 (对比)
% Phase 9: 可视化 + 数据保存
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ==================== Phase 0: 场景初始化 ====================
fprintf('========== Phase 0: 场景初始化 (拐弯目标) ==========\n');

params = simulation_params();
rng(params.random_seed);

% 拐弯航迹生成
[traj, turn_waypoints] = aircraft_trajectory_create('turn', params);
true_track = aircraft_trajectory_interpolate('generate', traj);
fprintf('真实航迹 (拐弯): %d 点, 总时长 %.0f s, 速度 %.0f m/s\n', ...
    size(true_track,1), traj.duration_sec, traj.speed);
fprintf('  航路点 (%d个):\n', size(turn_waypoints,1));
for i = 1:size(turn_waypoints,1)
    fprintf('    W%d: (%.1f, %.1f)\n', i, turn_waypoints(i,1), turn_waypoints(i,2));
end

% 验证拐角
if traj.n_segments >= 2
    seg1 = traj.segments{1};
    seg2 = traj.segments{2};
    bearing_in = sphere_utils_azimuth(seg1.start(1), seg1.start(2), seg1.end(1), seg1.end(2));
    bearing_out = sphere_utils_azimuth(seg2.start(1), seg2.start(2), seg2.end(1), seg2.end(2));
    turn_angle = abs(bearing_out - bearing_in);
    if turn_angle > 180, turn_angle = 360 - turn_angle; end
    fprintf('  入向方位: %.1f°, 出向方位: %.1f°, 拐角: %.1f°\n', bearing_in, bearing_out, turn_angle);
end

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
fprintf('  R1覆盖: %d/%d点 (%.0f%%), R2覆盖: %d/%d点 (%.0f%%)\n', ...
    n_in_r1, size(true_track,1), n_in_r1/size(true_track,1)*100, ...
    n_in_r2, size(true_track,1), n_in_r2/size(true_track,1)*100);

% 时间网格
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('仿真帧数: %d (dt=%.0fs)\n', n_frames, params.dt_sec);

% 真值结构体
tt = true_track;
truthTraj = struct('label', 'A', 'speed_ms', traj.speed, ...
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
fprintf('R1采样: 0s/30s/30s/...  R2采样: 13s/43s/73s/...  偏移=%ds\n', ...
    params.time_offset_radar2_sec);

%% ==================== Phase 4: 偏差校正 + 几何反解 ====================
fprintf('\n========== Phase 4: 偏差校正 ==========\n');

detList_R1 = cell(n_frames, 1);
detList_R2 = cell(n_frames, 1);

for k = 1:n_frames
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

%% ==================== Phase 5: 航迹跟踪 (基础UKF + 机动自适应UKF) ====================
fprintf('\n========== Phase 5: 航迹跟踪 ==========\n');

% ---- R1 UKF (精密站, 拐弯场景放宽门限) ----
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale = 5e4;
params.ukf_P_pos_std = 0.2;
params.ukf_P_vel_std = 0.004;
params.gate_sigma = 2.5;
params.tracker_K_loss = 20;

ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

% ---- R2 params (普通站) ----
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

ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

% ---- 第5.1节: 基础UKF跟踪 ----
fprintf('--- 5.1 基础UKF (模糊自适应Q) ---\n');
[trackSnapshots_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames);
[trackSnapshots_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames);

fprintf('R1基础UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk1.type), finalTrk1.quality, finalTrk1.life);
fprintf('R2基础UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk2.type), finalTrk2.quality, finalTrk2.life);

% ---- 第5.2节: 机动自适应UKF跟踪 ----
fprintf('\n--- 5.2 机动自适应UKF (新息序列机动检测+Q提升) ---\n');

% 复位随机种子到相同状态, 确保点迹一致
rng(params.random_seed);
% 重建UKF模板 (避免参数污染)
ukf1_tpl_ad = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
ukf2_tpl_ad = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

[trackSnapshots_R1_ad, finalTrk1_ad] = single_track_runner_adaptive(detList_R1, ukf1_tpl_ad, params, n_frames);
[trackSnapshots_R2_ad, finalTrk2_ad] = single_track_runner_adaptive(detList_R2, ukf2_tpl_ad, params_r2, n_frames);

fprintf('R1自适应UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk1_ad.type), finalTrk1_ad.quality, finalTrk1_ad.life);
fprintf('R2自适应UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk2_ad.type), finalTrk2_ad.quality, finalTrk2_ad.life);

% ---- 机动检测统计 ----
fprintf('\n--- 机动自适应UKF诊断 ---\n');
for radar_label = {'R1', 'R2'}
    snaps = trackSnapshots_R1_ad;
    rname = 'R1';
    if strcmp(radar_label{1}, 'R2'), snaps = trackSnapshots_R2_ad; rname = 'R2'; end

    n_maneuver = 0; n_assoc = 0; n_predict = 0; nis_vals = [];
    init_frame = 0;
    maneuver_frames = [];
    for k = 1:length(snaps)
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 1
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
                    isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;
            else
                n_predict = n_predict + 1;
            end
            if isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
            end
            if isfield(trk.ukf, 'maneuver_active') && trk.ukf.maneuver_active
                n_maneuver = n_maneuver + 1;
                maneuver_frames(end+1) = k;
            end
        elseif trk.type == 6 && init_frame == 0
            % 尚未起始
        end
        if init_frame == 0 && trk.type == 1, init_frame = k; end
    end
    n_tracked = n_assoc + n_predict;
    fprintf('%s自适应: 起始帧=%d | 关联=%d (%.0f%%) | 机动帧=%d', ...
        rname, init_frame, n_assoc, n_assoc/max(1,n_tracked)*100, n_maneuver);
    if ~isempty(maneuver_frames)
        fprintf(' [%d-%d]', maneuver_frames(1), maneuver_frames(end));
    end
    fprintf('\n');
    if ~isempty(nis_vals)
        nis_in_gate = sum(nis_vals < 4*2);
        fprintf('  NIS: 均值=%.2f 门内=%.0f%% (%d/%d)\n', ...
            mean(nis_vals), nis_in_gate/length(nis_vals)*100, nis_in_gate, length(nis_vals));
    end
end

%% ==================== Phase 6: 航迹级时间对齐 ====================
fprintf('\n========== Phase 6: 航迹级时间对齐 ==========\n');

aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
aligned_R2_ad = time_align_tracks(trackSnapshots_R2_ad, params);
fprintf('R2航迹时间对齐完成 (基础+自适应)\n');

%% ==================== Phase 7: 航迹融合 ====================
fprintf('\n========== Phase 7: 航迹融合 ==========\n');

matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);

method_names = {'SCC', 'BC', 'CI', 'FCI'};

% 基础UKF融合
all_fused_snapshots = cell(length(method_names), 1);
for m = 1:length(method_names)
    all_fused_snapshots{m} = run_track_fusion(matched_pair, ...
        trackSnapshots_R1, aligned_R2, params, method_names{m});
end

% 机动自适应UKF融合
all_fused_snapshots_ad = cell(length(method_names), 1);
for m = 1:length(method_names)
    all_fused_snapshots_ad{m} = run_track_fusion(matched_pair, ...
        trackSnapshots_R1_ad, aligned_R2_ad, params, method_names{m});
end

fprintf('融合完成: 基础UKF 4种 + 自适应UKF 4种\n');

%% ==================== Phase 8: 定量误差评估 ====================
fprintf('\n========== Phase 8: 定量误差评估 ==========\n');

truthTrajs = {truthTraj};

% 构建简化matcher
n_frames_val = n_frames;

% --- 基础UKF评估 ---
r1_pos = build_pos_history(trackSnapshots_R1, 1, n_frames);
r2_pos = build_pos_history(aligned_R2, 1, n_frames);
matcher_base = make_matcher(r1_pos, r2_pos, aligned_R2);

fusion_eval_base = evaluate_all('fusion', all_fused_snapshots, method_names, ...
    matched_pair, trackSnapshots_R1, trackSnapshots_R2, ...
    truthTrajs, n_frames, params.dt_sec, matcher_base);

% --- 自适应UKF评估 ---
r1_pos_ad = build_pos_history(trackSnapshots_R1_ad, 1, n_frames);
r2_pos_ad = build_pos_history(aligned_R2_ad, 1, n_frames);
matcher_ad = make_matcher(r1_pos_ad, r2_pos_ad, aligned_R2_ad);

fusion_eval_ad = evaluate_all('fusion', all_fused_snapshots_ad, method_names, ...
    matched_pair, trackSnapshots_R1_ad, trackSnapshots_R2_ad, ...
    truthTrajs, n_frames, params.dt_sec, matcher_ad);

% 打印对比表
fprintf('\n--- 误差对比: 基础UKF vs 机动自适应UKF (RMSE km) ---\n');
fprintf('%-20s %8s %8s %8s\n', '算法', '基础UKF', '自适应', '改善');
fprintf('%-20s %8s %8s %8s\n', '------', '------', '------', '------');

all_labels = [method_names, {'R1_only', 'R2_only'}];
for m = 1:length(all_labels)
    rmse_base = fusion_eval_base.overall(m).s.rms;
    rmse_ad = fusion_eval_ad.overall(m).s.rms;
    if rmse_base > 0
        improvement = (1 - rmse_ad/rmse_base) * 100;
    else
        improvement = 0;
    end
    fprintf('%-20s %8.1f %8.1f %+7.1f%%\n', all_labels{m}, rmse_base, rmse_ad, improvement);
end

% 最佳算法
rms_vals_base = arrayfun(@(x) x.s.rms, fusion_eval_base.overall(1:4));
rms_vals_ad = arrayfun(@(x) x.s.rms, fusion_eval_ad.overall(1:4));
[~, best_m_base] = min(rms_vals_base);
[~, best_m_ad] = min(rms_vals_ad);

fprintf('\n基础UKF最佳融合: %s (%.1f km)\n', method_names{best_m_base}, rms_vals_base(best_m_base));
fprintf('自适应UKF最佳融合: %s (%.1f km)\n', method_names{best_m_ad}, rms_vals_ad(best_m_ad));

% 单站误差对比
fprintf('\n--- 单站UKF误差对比 ---\n');
aligned_R2_eval = time_align_tracks(trackSnapshots_R2, params);
aligned_R2_ad_eval = time_align_tracks(trackSnapshots_R2_ad, params);

errorStats_R1 = evaluate_all('tracking_errors', trackSnapshots_R1, detList_R1, truthTrajs, n_frames, params.dt_sec, 'R1');
errorStats_R2 = evaluate_all('tracking_errors', aligned_R2_eval, detList_R2, truthTrajs, n_frames, params.dt_sec, 'R2');
errorStats_R1_ad = evaluate_all('tracking_errors', trackSnapshots_R1_ad, detList_R1, truthTrajs, n_frames, params.dt_sec, 'R1-ad');
errorStats_R2_ad = evaluate_all('tracking_errors', aligned_R2_ad_eval, detList_R2, truthTrajs, n_frames, params.dt_sec, 'R2-ad');

for pair = {errorStats_R1, errorStats_R1_ad; errorStats_R2, errorStats_R2_ad}
    e_base = pair{1}; e_ad = pair{2};
    s_b = e_base.summary(1).ukf;
    s_a = e_ad.summary(1).ukf;
    if s_b.rms > 0
        imp = (1 - s_a.rms/s_b.rms)*100;
    else
        imp = 0;
    end
    fprintf('%s: 基础UKF RMSE=%.1fkm → 自适应 UKF RMSE=%.1fkm (%+.1f%%)\n', ...
        e_base.radar, s_b.rms, s_a.rms, imp);
end

%% ==================== Phase 9: 可视化 ====================
fprintf('\n========== Phase 9: 可视化 ==========\n');
if ~exist('results', 'dir'), mkdir('results'); end

warn_state = warning('off', 'all');

% 图1: 场景总览 (拐弯航迹 + 双雷达覆盖)
plot_scene_overview(true_track, params, 'results');

% 图2: 点云 + 基础UKF(虚线) + 自适应UKF(实线) 并排 (R1左 R2右)
plot_turn_spatial('point_clouds', true_track, detList_R1, detList_R2, ...
    trackSnapshots_R1, trackSnapshots_R2, ...
    trackSnapshots_R1_ad, trackSnapshots_R2_ad, params, 'results');

% 图3: R1 单站对比 (地图+拐弯放大+误差时间线+RMSE柱状图)
plot_turn_spatial('radar_compare', true_track, trackSnapshots_R1, trackSnapshots_R1_ad, ...
    'R1', params.radar1_lat, params.radar1_lon, params, 'results', 3);

% 图4: R2 单站对比 (地图+拐弯放大+误差时间线+RMSE柱状图)
plot_turn_spatial('radar_compare', true_track, trackSnapshots_R2, trackSnapshots_R2_ad, ...
    'R2', params.radar2_lat, params.radar2_lon, params, 'results', 4);

% 图5: 融合地图对比 (基础融合虚线 + 自适应融合实线 + 拐弯放大 + 信息面板)
plot_turn_spatial('fusion_map', true_track, ...
    all_fused_snapshots, method_names, best_m_base, ...
    all_fused_snapshots_ad, method_names, best_m_ad, params, 'results');

% 图6: RMSE柱状图总览 (全部方法 基础灰 vs 自适应绿 + 数值汇总)
plot_turn_stats('rmse_bars', fusion_eval_base, fusion_eval_ad, ...
    method_names, best_m_base, best_m_ad, params, 'results');

% 图7: 全图层综合对比 (地图 + 按钮控制显隐)
plot_turn_spatial('comprehensive', true_track, detList_R1, detList_R2, ...
    trackSnapshots_R1, trackSnapshots_R2, ...
    trackSnapshots_R1_ad, trackSnapshots_R2_ad, ...
    all_fused_snapshots{1}, all_fused_snapshots_ad{1}, params, 'results');

warning(warn_state);

fprintf('\n========== Phase 9: 数据保存 ==========\n');
outf = fullfile('results', sprintf('simulation_turn_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'params', 'truthTraj', 'true_track', 'turn_waypoints', ...
    'trackSnapshots_R1', 'trackSnapshots_R2', 'finalTrk1', 'finalTrk2', ...
    'trackSnapshots_R1_ad', 'trackSnapshots_R2_ad', 'finalTrk1_ad', 'finalTrk2_ad', ...
    'all_fused_snapshots', 'all_fused_snapshots_ad', 'method_names', ...
    'fusion_eval_base', 'fusion_eval_ad', ...
    'errorStats_R1', 'errorStats_R2', 'errorStats_R1_ad', 'errorStats_R2_ad', ...
    'dr1_est', 'dr2_est', 'da1_est', 'da2_est');
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

function pos = build_pos_history(snapshots, track_id, n_frames)
    pos = nan(1, n_frames, 2);
    for k = 1:n_frames
        snap = snapshots{k};
        if ~isempty(snap.trackList)
            for t = 1:length(snap.trackList)
                if snap.trackList{t}.id == track_id
                    pos(1, k, 1) = snap.trackList{t}.lon;
                    pos(1, k, 2) = snap.trackList{t}.lat;
                    break;
                end
            end
        end
    end
end

function m = make_matcher(r1_pos, r2_pos, aligned_r2)
    m = struct();
    m.r1_pos = r1_pos;
    m.r2_pos = r2_pos;
    m.matched_pairs = struct('R1_track_id', 1, 'R2_track_id', 1, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);
    m.aligned_R2 = aligned_r2;
    m.r1_ids = 1;
    m.r2_ids = 1;
end
