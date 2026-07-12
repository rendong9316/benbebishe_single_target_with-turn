% =========================================================================
% run_simulation_multi.m — 双基地OTH-SWR三目标交叉航迹IMM仿真主程序
% =========================================================================
% 【程序定位】
%   本程序是本仿真系统的三目标交叉场景主入口。与 run_simulation.m（单目标）
%   不同，本程序验证多目标场景下 JPDA/JNN 关联、跨雷达航迹匹配、
%   多对多融合的全链路性能。
%
% 【三目标交叉航迹设计】
%   目标A: 西南→东北，穿越覆盖区西部
%   目标B: 西北→东南，穿越覆盖区中部
%   目标C: 西→东，穿越覆盖区东部
%   三目标在覆盖区中心附近交叉，形成复杂的关联歧义场景
%
% 【跟踪算法】
%   R1/R2 各自使用 IMM CV+CT 双模型跟踪，每部雷达独立输出多条航迹
%
% 【9-Phase 流水线总览】
%   Phase 0: 场景初始化（三目标交叉航迹 + 覆盖检查 + 时间网格）
%   Phase 1: ADS-B系统偏差标定
%   Phase 2: 原始点迹生成（多目标，含aircraft_id）
%   Phase 3: 时间对齐策略
%   Phase 4: 偏差校正 + 几何反解
%   Phase 5: 航迹跟踪（IMM: CV+CT双模型，multi_track_manager）
%   Phase 6: 航迹级时间对齐
%   Phase 7: 航迹匹配 + 航迹融合（SCC/BC/CI/FCI）
%   Phase 8: 定量误差评估（按目标ID分别评估）
%   Phase 9: 可视化 + 数据保存
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ==================== Phase 0: 场景初始化（三目标交叉） ====================
fprintf('========== Phase 0: 场景初始化 (三目标交叉) ==========\n');

params = simulation_params_multi();
rng(params.random_seed);

% ---- 三目标航迹定义 ----
% 目标A: 西南→东北 (类似单目标但缩短)
way_A = [126.8, 31.5, 0; 130.0, 33.5, 0];
way_B = [126.8, 33.5, 0; 130.0, 31.5, 0];
way_C = [126.8, 32.5, 0; 130.8, 32.5, 0];

% 生成三条航迹
traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);

true_track_A = aircraft_trajectory_interpolate('generate', traj_A);
true_track_B = aircraft_trajectory_interpolate('generate', traj_B);
true_track_C = aircraft_trajectory_interpolate('generate', traj_C);

fprintf('目标A: %d 点, 总时长 %.0f s\n', size(true_track_A,1), traj_A.duration_sec);
fprintf('目标B: %d 点, 总时长 %.0f s\n', size(true_track_B,1), traj_B.duration_sec);
fprintf('目标C: %d 点, 总时长 %.0f s\n', size(true_track_C,1), traj_C.duration_sec);

% ---- 覆盖检查 ----
n_in_r1_A = 0; n_in_r2_A = 0;
n_in_r1_B = 0; n_in_r2_B = 0;
n_in_r1_C = 0; n_in_r2_C = 0;
for i = 1:size(true_track_A,1)
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track_A(i,1), true_track_A(i,2), params.radar1_beam_center_deg, params);
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track_A(i,1), true_track_A(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1_A = n_in_r1_A+1; end
    if in2, n_in_r2_A = n_in_r2_A+1; end
end
for i = 1:size(true_track_B,1)
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track_B(i,1), true_track_B(i,2), params.radar1_beam_center_deg, params);
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track_B(i,1), true_track_B(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1_B = n_in_r1_B+1; end
    if in2, n_in_r2_B = n_in_r2_B+1; end
end
for i = 1:size(true_track_C,1)
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track_C(i,1), true_track_C(i,2), params.radar1_beam_center_deg, params);
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track_C(i,1), true_track_C(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1_C = n_in_r1_C+1; end
    if in2, n_in_r2_C = n_in_r2_C+1; end
end
fprintf('覆盖统计:\n');
fprintf('  A: R1=%d/%d, R2=%d/%d\n', n_in_r1_A, size(true_track_A,1), n_in_r2_A, size(true_track_A,1));
fprintf('  B: R1=%d/%d, R2=%d/%d\n', n_in_r1_B, size(true_track_B,1), n_in_r2_B, size(true_track_B,1));
fprintf('  C: R1=%d/%d, R2=%d/%d\n', n_in_r1_C, size(true_track_C,1), n_in_r2_C, size(true_track_C,1));

% ---- 时间网格 ----
t1_grid = params.time_offset_radar1_sec : params.dt_sec : max(max(traj_A.duration_sec, traj_B.duration_sec), traj_C.duration_sec);
t2_grid = params.time_offset_radar2_sec : params.dt_sec : max(max(traj_A.duration_sec, traj_B.duration_sec), traj_C.duration_sec);
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('仿真帧数: %d (dt=%.0fs)\n', n_frames, params.dt_sec);

% ---- 真值结构体 (供Phase 8评估) ----
truthTrajs = cell(3, 1);
tt = true_track_A;
truthTrajs{1} = struct('label', 'A', 'speed_ms', params.aircraft_speed_ms, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
tt = true_track_B;
truthTrajs{2} = struct('label', 'B', 'speed_ms', params.aircraft_speed_ms, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
tt = true_track_C;
truthTrajs{3} = struct('label', 'C', 'speed_ms', params.aircraft_speed_ms, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));

%% ==================== Phase 1: ADS-B系统偏差标定 ====================
fprintf('\n========== Phase 1: ADS-B系统偏差标定 ==========\n');
rng(params.random_seed);
T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
adsb_lat = T_adsb.Var2;  adsb_lon = T_adsb.Var3;
dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
n_check = min(5000, height(T_adsb));
cal_step = max(1, floor(height(T_adsb) / n_check));
for idx = 1:cal_step:height(T_adsb)
    t_lon = adsb_lon(idx);  t_lat = adsb_lat(idx);
    if isnan(t_lon) || isnan(t_lat), continue; end
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        t_lon, t_lat, params.radar1_beam_center_deg, params);
    if in1
        Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
        az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
        dr1_list(end+1) = Rg_meas - Rg_true;
        daz = az_meas - az_true;
        if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
        da1_list(end+1) = daz;
    end
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        t_lon, t_lat, params.radar2_beam_center_deg, params);
    if in2
        Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
            params.radar2_lon, params.radar2_lat, t_lon, t_lat);
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
fprintf('R1 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)\n', ...
    dr1_est, da1_est, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg);
fprintf('R2 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)\n', ...
    dr2_est, da2_est, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg);

%% ==================== Phase 2+4: 多目标点迹生成 + 偏差校正 ====================
fprintf('\n========== Phase 2: 多目标点迹生成 ==========\n');

detList_R1 = cell(n_frames, 1);
detList_R2 = cell(n_frames, 1);

% R1 随机流
rng(params.random_seed + 1e7);
truth_all_R1 = {true_track_A, true_track_B, true_track_C};
for k = 1:n_frames
    t1 = t1_grid(k);
    % 收集三目标在当前帧的状态 [lon, lat, lon_rate, lat_rate, aircraft_id]
    tgt_states = zeros(3, 5);
    for ac = 1:3
        tt_ac = truth_all_R1{ac};
        if t1 >= tt_ac(1,5) && t1 <= tt_ac(end,5)
            % 手动线性插值
            t_vals = tt_ac(:,5);
            lon_vals = tt_ac(:,1);
            lat_vals = tt_ac(:,2);
            lr_vals = tt_ac(:,3);
            latr_vals = tt_ac(:,4);
            pos = interp1(t_vals, [lon_vals, lat_vals], t1, 'linear', 'extrap');
            lr = interp1(t_vals, lr_vals, t1, 'linear', 'extrap');
            latr = interp1(t_vals, latr_vals, t1, 'linear', 'extrap');
            tgt_states(ac,:) = [pos(1), pos(2), lr, latr, ac];
        else
            tgt_states(ac,:) = [NaN, NaN, NaN, NaN, ac];
        end
    end
    tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);

    if isempty(tgt_states)
        detList_R1{k} = [];
        continue;
    end

    detRaw = generate_frame_detections_multi(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, tgt_states, ...
        k, t1, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);

    % 偏差校正 + 反解
    dets_r1 = {};
    for d = 1:length(detRaw)
        dp = detRaw(d);
        Rgc = dp.prange - dr1_est;  azc = dp.paz - da1_est;
        dp.drange = Rgc;  dp.daz = azc;
        dp.range_meas = Rgc;  dp.azimuth_meas = azc;
        if ~isfield(dp, 'lat') || isnan(dp.lat)
            [~, dp.lat, dp.lon] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
        end
        [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat);
        dets_r1{end+1} = dp;
    end
    detList_R1{k} = [dets_r1{:}];
end
fprintf('R1 点迹生成完成: %d 帧\n', n_frames);

% R2 随机流
rng(params.random_seed + 2e7);
truth_all_R2 = {true_track_A, true_track_B, true_track_C};
for k = 1:n_frames
    t2 = t2_grid(k);
    tgt_states = zeros(3, 5);
    for ac = 1:3
        tt_ac = truth_all_R2{ac};
        if t2 >= tt_ac(1,5) && t2 <= tt_ac(end,5)
            t_vals = tt_ac(:,5);
            lon_vals = tt_ac(:,1);
            lat_vals = tt_ac(:,2);
            lr_vals = tt_ac(:,3);
            latr_vals = tt_ac(:,4);
            pos = interp1(t_vals, [lon_vals, lat_vals], t2, 'linear', 'extrap');
            lr = interp1(t_vals, lr_vals, t2, 'linear', 'extrap');
            latr = interp1(t_vals, latr_vals, t2, 'linear', 'extrap');
            tgt_states(ac,:) = [pos(1), pos(2), lr, latr, ac];
        else
            tgt_states(ac,:) = [NaN, NaN, NaN, NaN, ac];
        end
    end
    tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);

    if isempty(tgt_states)
        detList_R2{k} = [];
        continue;
    end

    detRaw = generate_frame_detections_multi(params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, tgt_states, ...
        k, t2, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
        params.radar2_beam_center_deg, params, ...
        params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);

    dets_r2 = {};
    for d = 1:length(detRaw)
        dp = detRaw(d);
        Rgc = dp.prange - dr2_est;  azc = dp.paz - da2_est;
        dp.drange = Rgc;  dp.daz = azc;
        dp.range_meas = Rgc;  dp.azimuth_meas = azc;
        if ~isfield(dp, 'lat') || isnan(dp.lat)
            [~, dp.lat, dp.lon] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
        end
        [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, ...
            params.radar2_tx_lon, params.radar2_tx_lat, ...
            params.radar2_lon, params.radar2_lat);
        dets_r2{end+1} = dp;
    end
    detList_R2{k} = [dets_r2{:}];
end
fprintf('R2 点迹生成完成: %d 帧\n', n_frames);

%% ==================== Phase 5: 多目标航迹跟踪 ====================
fprintf('\n========== Phase 5: 多目标航迹跟踪 (IMM CV+CT) ==========\n');

% R1 IMM 模板
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
params.gate_sigma = params.radar1_gate_sigma;
params.gate_vr_ms = params.radar1_gate_vr_ms;
params.tracker_K_loss = 15;
ukf1_tpl = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

% R2 IMM 模板
params_r2 = params;
params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
params_r2.gate_sigma = params.radar2_gate_sigma;
params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
params_r2.tracker_K_loss = 15;
ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

% ---- R1 多目标跟踪 ----
fprintf('--- R1 IMM 多目标跟踪 ---\n');
trackSnapshots_R1 = cell(n_frames, 1);
tempPool_R1 = {};
trackList_R1 = {};
next_id_R1 = 1;

fprintf('  [DIAG] Frame 1 R1 detections: %d\n', length(detList_R1{1}));

for k = 1:n_frames
    dets = detList_R1{k};
    [trackList_R1, tempPool_R1, trackSnapshots_R1{k}, next_id_R1] = ...
        multi_track_runner_kf(trackList_R1, tempPool_R1, dets, ukf1_tpl, ...
        params, k, next_id_R1, truth_all_R1, t1_grid);
end
fprintf('R1 最终航迹数: %d\n', length(trackList_R1));
fprintf('--- R2 IMM 多目标跟踪 ---\n');
trackSnapshots_R2 = cell(n_frames, 1);
tempPool_R2 = {};
trackList_R2 = {};
next_id_R2 = 1;

fprintf('  [DIAG] Frame 1 R2 detections: %d\n', length(detList_R2{1}));

for k = 1:n_frames
    dets = detList_R2{k};
    [trackList_R2, tempPool_R2, trackSnapshots_R2{k}, next_id_R2] = ...
        multi_track_runner_kf(trackList_R2, tempPool_R2, dets, ukf2_tpl, ...
        params_r2, k, next_id_R2, truth_all_R2, t2_grid);
end
fprintf('R2 最终航迹数: %d\n', length(trackList_R2));

%% ==================== Phase 6: 航迹级时间对齐 ====================
fprintf('\n========== Phase 6: 航迹级时间对齐 ==========\n');
aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
fprintf('R2航迹时间对齐完成\n');

%% ==================== Phase 7: 航迹匹配 + 融合 ====================
fprintf('\n========== Phase 7: 航迹匹配 + 融合 ==========\n');

% ---- 跨雷达航迹匹配 ----
fprintf('--- 跨雷达航迹匹配 ---\n');

% 诊断：打印每帧R1和R2的航迹位置
fprintf('  [DIAG] Sample track positions per frame:\n');
for k = [1, 10, 20, 30, 40, 50, 60, 68]
    if k > n_frames, continue; end
    trks1 = trackSnapshots_R1{k}.trackList;
    trks2 = aligned_R2{k}.trackList;
    fprintf('    Frame %d: ', k);
    for t = 1:length(trks1)
        if trks1{t}.type ~= 7 && ~isnan(trks1{t}.lat)
            fprintf('R1#%d(%.2f,%.2f) ', trks1{t}.id, trks1{t}.lon, trks1{t}.lat);
        end
    end
    fprintf('| ');
    for t = 1:length(trks2)
        if trks2{t}.type ~= 7 && ~isnan(trks2{t}.lat)
            fprintf('R2#%d(%.2f,%.2f) ', trks2{t}.id, trks2{t}.lon, trks2{t}.lat);
        end
    end
    fprintf('\n');
end

% ---- 跨雷达航迹匹配 ----
fprintf('--- 跨雷达航迹匹配 (real) ---\n');
matched_pairs_struct = track_matcher(trackSnapshots_R1, aligned_R2, params);
matched_pairs = cell(length(matched_pairs_struct), 1);
for p = 1:length(matched_pairs_struct)
    matched_pairs{p} = matched_pairs_struct(p);
end

fprintf('匹配到 %d 对航迹\n', length(matched_pairs));
for p = 1:length(matched_pairs)
    mp = matched_pairs{p};
    fprintf('  Pair %d: R1#%d <-> R2#%d, 共现=%d帧, 平均距离=%.1fkm\n', ...
        p, mp.R1_track_id, mp.R2_track_id, mp.coexist_count, mp.mean_dist_km);
end

% ---- 对每对匹配航迹执行四种融合 ----
method_names = {'SCC', 'BC', 'CI', 'FCI'};
all_fused_snapshots = cell(length(matched_pairs), length(method_names));

for p = 1:length(matched_pairs)
    fprintf('  匹配对 %d/%d:\n', p, length(matched_pairs));
    for m = 1:length(method_names)
        method = method_names{m};
        all_fused_snapshots{p,m} = run_track_fusion(matched_pairs{p}, ...
            trackSnapshots_R1, aligned_R2, params, method);
    end
end
fprintf('融合完成: %d 对 x %d 算法\n', length(matched_pairs), length(method_names));

%% ==================== Phase 8: 定量误差评估 ====================
fprintf('\n========== Phase 8: 定量误差评估 ==========\n');

matcher_multi = struct();
matcher_multi.matched_pairs = matched_pairs_struct;
matcher_multi.aligned_R2 = aligned_R2;

% 提取 R1/R2 航迹 ID 和位置（供 evaluate_all_multi 映射配对到真值）
r1_ids = []; r2_ids = [];
for k = 1:n_frames
    snap1 = trackSnapshots_R1{k};
    snap2 = aligned_R2{k};
    trks1 = snap1.trackList;
    for t = 1:length(trks1)
        trk = trks1{t};
        if trk.type ~= 7 && ~isnan(trk.lat)
            r1_ids(end+1, :) = [trk.id, k, trk.lon, trk.lat];
        end
    end
    trks2 = snap2.trackList;
    for t = 1:length(trks2)
        trk = trks2{t};
        if trk.type ~= 7 && ~isnan(trk.lat)
            r2_ids(end+1, :) = [trk.id, k, trk.lon, trk.lat];
        end
    end
end

% 构建 r1_pos[r1_id_idx, frame_idx, coord]
unique_r1_ids = unique(r1_ids(:,1));
unique_r2_ids = unique(r2_ids(:,1));
n_r1 = length(unique_r1_ids);
n_r2 = length(unique_r2_ids);

r1_pos = nan(n_r1, n_frames, 2);
r2_pos = nan(n_r2, n_frames, 2);
for i = 1:n_r1
    rid = unique_r1_ids(i);
    rows = r1_ids(r1_ids(:,1) == rid, :);
    for r = 1:size(rows,1)
        fk = round(rows(r,2));
        if fk >= 1 && fk <= n_frames
            r1_pos(i, fk, 1) = rows(r,3);
            r1_pos(i, fk, 2) = rows(r,4);
        end
    end
end
for i = 1:n_r2
    rid = unique_r2_ids(i);
    rows = r2_ids(r2_ids(:,1) == rid, :);
    for r = 1:size(rows,1)
        fk = round(rows(r,2));
        if fk >= 1 && fk <= n_frames
            r2_pos(i, fk, 1) = rows(r,3);
            r2_pos(i, fk, 2) = rows(r,4);
        end
    end
end
matcher_multi.r1_ids = r1_ids;
matcher_multi.r2_ids = r2_ids;
matcher_multi.unique_r1_ids = unique_r1_ids;
matcher_multi.unique_r2_ids = unique_r2_ids;
matcher_multi.r1_pos = r1_pos;
matcher_multi.r2_pos = r2_pos;

% 融合评估
fusion_eval = evaluate_all_multi('fusion', all_fused_snapshots, method_names, ...
    matched_pairs_struct, trackSnapshots_R1, trackSnapshots_R2, ...
    truthTrajs, n_frames, params.dt_sec, matcher_multi);

fprintf('\n--- 融合误差对比 (RMSE km) ---\n');
fprintf('%-8s %8s %8s\n', '算法', 'RMSE', '中位');
fprintf('%-8s %8s %8s\n', '------', '------', '------');
for m = 1:length(fusion_eval.overall)
    s = fusion_eval.overall(m).s;
    fprintf('%-8s %8.1f %8.1f\n', fusion_eval.overall(m).method, s.rms, s.median);
end

% 单站跟踪误差
errorStats_R1 = evaluate_all_multi('tracking_errors', trackSnapshots_R1, detList_R1, ...
    truthTrajs, n_frames, params.dt_sec, 'R1');
errorStats_R2 = evaluate_all_multi('tracking_errors', aligned_R2, detList_R2, ...
    truthTrajs, n_frames, params.dt_sec, 'R2');

for es = {errorStats_R1, errorStats_R2}
    e = es{1};
    fprintf('\n--- %s UKF滤波误差 ---\n', e.radar);
    for a = 1:length(e.summary)
        s_u = e.summary(a).ukf;
        fprintf('  目标%d: 点数=%d, 中位=%.1fkm, 均值=%.1fkm, RMSE=%.1fkm, 95%%=%.1fkm\n', ...
            a, s_u.n, s_u.median, s_u.mean, s_u.rms, s_u.pct95);
    end
end

%% ==================== Phase 9: 可视化 + 数据保存 ====================
fprintf('\n========== Phase 9: 可视化 ==========\n');
if ~exist('results', 'dir'), mkdir('results'); end

warn_state = warning('off', 'all');

% 图1: 场景总览
plot_scene_overview_multi(true_track_A, true_track_B, true_track_C, params, 'results');

% 图2: R1/R2 点迹3D
plot_point_cloud_3d(detList_R1, 'R1', 'results/fig2a_R1_point_cloud.png');
plot_point_cloud_3d(detList_R2, 'R2', 'results/fig2b_R2_point_cloud.png');

% 图3: 多目标跟踪综合图
plot_results_multi('single_track', true_track_A, true_track_B, true_track_C, ...
    detList_R1, detList_R2, trackSnapshots_R1, trackSnapshots_R2, ...
    params, 'results');

% 图4: 多目标融合可视化
plot_results_multi('single_fusion', true_track_A, true_track_B, true_track_C, ...
    trackSnapshots_R1, trackSnapshots_R2, all_fused_snapshots, ...
    method_names, matched_pairs, fusion_eval, truthTrajs, params, 'results');

warning(warn_state);

fprintf('\n========== Phase 9: 数据保存 ==========\n');

sysPara = struct(...
    'dt_sec', params.dt_sec, 'n_frames', n_frames, ...
    'R1_lon', params.radar1_lon, 'R1_lat', params.radar1_lat, ...
    'R1_tx_lon', params.radar1_tx_lon, 'R1_tx_lat', params.radar1_tx_lat, ...
    'R2_lon', params.radar2_lon, 'R2_lat', params.radar2_lat, ...
    'R2_tx_lon', params.radar2_tx_lon, 'R2_tx_lat', params.radar2_tx_lat, ...
    'detection_probability', params.detection_probability, ...
    'false_alarm_rate', params.false_alarm_rate, ...
    'random_seed', params.random_seed);

calibResult = struct(...
    'dr1_est', dr1_est, 'da1_est', da1_est, ...
    'dr2_est', dr2_est, 'da2_est', da2_est, ...
    'dr1_true', params.radar1_range_bias_m, 'da1_true', params.radar1_azimuth_bias_deg, ...
    'dr2_true', params.radar2_range_bias_m, 'da2_true', params.radar2_azimuth_bias_deg);

outf = fullfile('results', sprintf('simulation_multi_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'sysPara', 'calibResult', 'truthTrajs', 'detList_R1', 'detList_R2', ...
    'trackSnapshots_R1', 'trackSnapshots_R2', 'aligned_R2', ...
    'matched_pairs', 'all_fused_snapshots', 'method_names', ...
    'errorStats_R1', 'errorStats_R2', 'fusion_eval', 'params');
fprintf('数据已保存: %s\n', outf);
fprintf('\nDone.\n');

