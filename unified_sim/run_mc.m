% =========================================================================
% run_mc.m — 统一蒙特卡洛仿真入口（单/多目标通用）
% =========================================================================
% 【定位】
%   完全遵循 run_sim.m 的 Phase 0-9 流水线，逐种子循环输出统计。
%   架构: JPDA 跟踪 + track_matcher 跨雷达匹配 + SCC/BC/CI/FCI 融合
%
% 【配置方式】
%   修改底部 N_MC, SEED_BASE, trajectory_mode, n_targets 即可切换场景
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ---- 配置 ----
N_MC = 100;
SEED_BASE = 1;
trajectory_mode = 'straight';  % 'straight' | 'gradual_turn' | 'uturn'
n_targets = 1;                 % 1 或 3

% 预分配统计数组
n_frames_list = zeros(N_MC, 1);
rmse = struct();
rmse.ukf_R1 = nan(N_MC, 1);   rmse.ukf_R2 = nan(N_MC, 1);
rmse.ukf_R2_aligned = nan(N_MC, 1);
rmse.fus = nan(N_MC, 4);      rmse.fus_best = nan(N_MC, 1);
fus_best_method = cell(N_MC, 1);
mtl_R1 = nan(N_MC, 1);  mtl_R2 = nan(N_MC, 1);  mtl_fus = nan(N_MC, 1);
brk_R1 = nan(N_MC, 1);  brk_R2 = nan(N_MC, 1);  brk_fus = nan(N_MC, 1);
seg_count_R1 = nan(N_MC, 1);  seg_count_R2 = nan(N_MC, 1);  seg_count_fus = nan(N_MC, 1);
nis_mean_R1 = nan(N_MC, 1);   nis_mean_R2 = nan(N_MC, 1);
assoc_R1 = nan(N_MC, 1);      assoc_R2 = nan(N_MC, 1);
init_frame_R1 = nan(N_MC, 1); init_frame_R2 = nan(N_MC, 1);
imp_ukf_R1 = nan(N_MC, 1);    imp_ukf_R2 = nan(N_MC, 1);
imp_fus_vs_R1 = nan(N_MC, 1); imp_fus_vs_R2 = nan(N_MC, 1);
bad_seed = zeros(N_MC, 1);
bad_reason = cell(N_MC, 1);
seg_info = cell(N_MC, 1);

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     统一蒙特卡洛仿真  N=%d  mode=%s  n_targets=%d          ║\n', N_MC, trajectory_mode, n_targets);
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

tic;

%% ===== 主循环 =====
for mc = 1:N_MC
    seed = SEED_BASE + (mc - 1);
    fprintf('--- MC %d/%d (seed=%d) ---\n', mc, N_MC, seed);

    params = simulation_params();
    params.random_seed = seed;
    params.trajectory_mode = trajectory_mode;
    params.n_targets = n_targets;
    rng(params.random_seed);

    % ---- Phase 0: 场景初始化 ----
    if params.n_targets == 1
        switch lower(params.trajectory_mode)
            case 'straight'
                traj = aircraft_trajectory_create(params.aircraft_waypoints, ...
                    params.aircraft_speed_ms, params.dt_sec);
            case {'gradual_turn', 'turn'}
                [traj, ~] = aircraft_trajectory_create('gradual_turn', params);
            case 'uturn'
                [traj, ~] = aircraft_trajectory_create('uturn', params);
        end
        true_track = aircraft_trajectory_interpolate('generate', traj);
        tt = true_track;
        truthTraj = struct('label', 'A', 'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
            'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
        truthTrajs = {truthTraj};
        truth_all_cell = {true_track};  % for truth_init_tracks
        t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
        n_frames = min(length(t1_grid), length(t2_grid));
    else
        way_A = [127.0, 31.0, 0; 130.0, 34.0, 0];
        way_B = [126.0, 34.0, 0; 130.0, 31.0, 0];
        way_C = [126.0, 32.5, 0; 131.0, 32.5, 0];
        traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
        traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
        traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);
        true_track_A = aircraft_trajectory_interpolate('generate', traj_A);
        true_track_B = aircraft_trajectory_interpolate('generate', traj_B);
        true_track_C = aircraft_trajectory_interpolate('generate', traj_C);
        max_dur = max(traj_A.duration_sec, traj_B.duration_sec, traj_C.duration_sec);
        t1_grid = params.time_offset_radar1_sec : params.dt_sec : max_dur;
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : max_dur;
        n_frames = min(length(t1_grid), length(t2_grid));
        truthTrajs = cell(3, 1);
        truth_all_cell = {true_track_A, true_track_B, true_track_C};
        for ac = 1:3
            tt_ac = truth_all_cell{ac};
            truthTrajs{ac} = struct('label', char('A'+ac-1), 'speed_ms', params.aircraft_speed_ms, ...
                'time_sec', tt_ac(:,5), 'lat', tt_ac(:,2), 'lon', tt_ac(:,1), ...
                'lon_rate', tt_ac(:,3), 'lat_rate', tt_ac(:,4));
        end
        truth_all_cell = {true_track_A, true_track_B, true_track_C};
    end
    n_frames_list(mc) = n_frames;

    % ---- Phase 1: ADS-B 标定 ----
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2;  adsb_lon = T_adsb.Var3;
    dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx);  t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat), continue; end
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
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
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
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

    % ---- Phase 2+4: 点迹生成 + 偏差校正 ----
    detList_R1 = cell(n_frames, 1);  detList_R2 = cell(n_frames, 1);

    if params.n_targets == 1
        % 单目标统一用 multi 版本（tgt_states 单行退化）
        truth_all_R1 = truth_all_cell;
        rng(params.random_seed + 1e7);
        for k = 1:n_frames
            t = t1_grid(k);
            tgt_states = zeros(1, 5);
            tt_ac = truth_all_R1{1};
            if t >= tt_ac(1,5) && t <= tt_ac(end,5)
                t_vals = tt_ac(:,5);
                pos = interp1(t_vals, [tt_ac(:,1), tt_ac(:,2)], t, 'linear', 'extrap');
                lr = interp1(t_vals, tt_ac(:,3), t, 'linear', 'extrap');
                latr = interp1(t_vals, tt_ac(:,4), t, 'linear', 'extrap');
                tgt_states(1,:) = [pos(1), pos(2), lr, latr, 1];
            else
                tgt_states(1,:) = [NaN, NaN, NaN, NaN, 1];
            end
            if isnan(tgt_states(1,1)), detList_R1{k} = []; continue; end
            detRaw = generate_frame_detections_multi(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, tgt_states, ...
                k, t, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            detRaw = augment_dets(detRaw, dr1_est, da1_est, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            detList_R1{k} = detRaw;
        end

        rng(params.random_seed + 2e7);
        truth_all_R2 = truth_all_cell;
        for k = 1:n_frames
            t = t2_grid(k);
            tgt_states = zeros(1, 5);
            tt_ac = truth_all_R2{1};
            if t >= tt_ac(1,5) && t <= tt_ac(end,5)
                t_vals = tt_ac(:,5);
                pos = interp1(t_vals, [tt_ac(:,1), tt_ac(:,2)], t, 'linear', 'extrap');
                lr = interp1(t_vals, tt_ac(:,3), t, 'linear', 'extrap');
                latr = interp1(t_vals, tt_ac(:,4), t, 'linear', 'extrap');
                tgt_states(1,:) = [pos(1), pos(2), lr, latr, 1];
            else
                tgt_states(1,:) = [NaN, NaN, NaN, NaN, 1];
            end
            if isnan(tgt_states(1,1)), detList_R2{k} = []; continue; end
            detRaw = generate_frame_detections_multi(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, tgt_states, ...
                k, t, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            detRaw = augment_dets(detRaw, dr2_est, da2_est, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            detList_R2{k} = detRaw;
        end
    else
        % 三目标点迹生成（与 run_mc_multi.m 一致）
        truth_all_R1 = {true_track_A, true_track_B, true_track_C};
        rng(params.random_seed + 1e7);
        for k = 1:n_frames
            t = t1_grid(k);
            tgt_states = zeros(3, 5);
            for ac = 1:3
                tt_ac = truth_all_R1{ac};
                if t >= tt_ac(1,5) && t <= tt_ac(end,5)
                    t_vals = tt_ac(:,5);
                    pos = interp1(t_vals, [tt_ac(:,1), tt_ac(:,2)], t, 'linear', 'extrap');
                    lr = interp1(t_vals, tt_ac(:,3), t, 'linear', 'extrap');
                    latr = interp1(t_vals, tt_ac(:,4), t, 'linear', 'extrap');
                    tgt_states(ac,:) = [pos(1), pos(2), lr, latr, ac];
                else
                    tgt_states(ac,:) = [NaN, NaN, NaN, NaN, ac];
                end
            end
            tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);
            if isempty(tgt_states), detList_R1{k} = []; continue; end
            detRaw = generate_frame_detections_multi(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, tgt_states, ...
                k, t, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            detRaw = augment_dets(detRaw, dr1_est, da1_est, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            detList_R1{k} = detRaw;
        end

        rng(params.random_seed + 2e7);
        truth_all_R2 = {true_track_A, true_track_B, true_track_C};
        for k = 1:n_frames
            t = t2_grid(k);
            tgt_states = zeros(3, 5);
            for ac = 1:3
                tt_ac = truth_all_R2{ac};
                if t >= tt_ac(1,5) && t <= tt_ac(end,5)
                    [pos, vel] = aircraft_trajectory_interpolate(tt_ac, t);
                    tgt_states(ac,:) = [pos(1), pos(2), vel(1), vel(2), ac];
                else
                    tgt_states(ac,:) = [NaN, NaN, NaN, NaN, ac];
                end
            end
            tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);
            if isempty(tgt_states), detList_R2{k} = []; continue; end
            detRaw = generate_frame_detections_multi(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, tgt_states, ...
                k, t, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            detRaw = augment_dets(detRaw, dr2_est, da2_est, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            detList_R2{k} = detRaw;
        end
    end

    % ---- Phase 5: UKF 跟踪 ----
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    params.gate_sigma = params.radar1_gate_sigma;
    params.gate_vr_ms = params.radar1_gate_vr_ms;
    params.tracker_K_loss = params.radar1_tracker_K_loss;

    if strcmp(lower(trajectory_mode), 'uturn') || strcmp(lower(trajectory_mode), 'gradual_turn')
        ukf1_tpl = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    else
        ukf1_tpl = ukf_zishiying('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    end

    params_r2 = params;
    params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma = params.radar2_gate_sigma;
    params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
    params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;

    if strcmp(lower(trajectory_mode), 'uturn') || strcmp(lower(trajectory_mode), 'gradual_turn')
        ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    else
        ukf2_tpl = ukf_zishiying('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    end

    trackList_R1 = {};  tempPool_R1 = {};  next_id_R1 = 1;
    trackSnapshots_R1 = cell(n_frames, 1);
    for k = 1:n_frames
        [trackList_R1, tempPool_R1, trackSnapshots_R1{k}, next_id_R1] = ...
            multi_track_runner_kf(trackList_R1, tempPool_R1, detList_R1{k}, ukf1_tpl, ...
            params, k, next_id_R1, truth_all_cell, t1_grid);
    end

    trackList_R2 = {};  tempPool_R2 = {};  next_id_R2 = 1;
    trackSnapshots_R2 = cell(n_frames, 1);
    for k = 1:n_frames
        [trackList_R2, tempPool_R2, trackSnapshots_R2{k}, next_id_R2] = ...
            multi_track_runner_kf(trackList_R2, tempPool_R2, detList_R2{k}, ukf2_tpl, ...
            params_r2, k, next_id_R2, truth_all_cell, t2_grid);
    end

    % RMSE
    rmse.ukf_R1(mc) = rmse_tracks_mc(trackSnapshots_R1, truthTrajs, t1_grid, n_frames, params);
    rmse.ukf_R2(mc) = rmse_tracks_mc(trackSnapshots_R2, truthTrajs, t2_grid, n_frames, params);

    % NIS + 关联诊断
    [assoc_R1(mc), nis_mean_R1(mc), ~, ~, ~, init_frame_R1(mc)] = ...
        diagnose_tracking_mc(trackSnapshots_R1, n_frames);
    [assoc_R2(mc), nis_mean_R2(mc), ~, ~, ~, init_frame_R2(mc)] = ...
        diagnose_tracking_mc(trackSnapshots_R2, n_frames);

    % ---- Phase 6: 时间对齐 ----
    aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
    rmse.ukf_R2_aligned(mc) = rmse_tracks_mc(aligned_R2, truthTrajs, t1_grid, n_frames, params);

    % ---- Phase 7: 匹配 + 融合 ----
    matched_pairs = track_matcher(trackSnapshots_R1, aligned_R2, params);
    method_names = {'SCC', 'BC', 'CI', 'FCI'};
    all_fused = cell(length(method_names), 1);
    for m = 1:length(method_names)
        all_fused{m} = run_track_fusion(matched_pairs, trackSnapshots_R1, aligned_R2, params, method_names{m});
        rmse.fus(mc, m) = rmse_fusion_mc(all_fused{m}, truthTrajs, t1_grid, n_frames, matched_pairs);
    end
    [best_val, best_m] = min(rmse.fus(mc, :));
    rmse.fus_best(mc) = best_val;
    fus_best_method{mc} = method_names{best_m};
    imp_ukf_R1(mc) = (1 - rmse.ukf_R1(mc)/max(rmse.ukf_R1(mc), 0.001)) * 100;
    imp_ukf_R2(mc) = (1 - rmse.ukf_R2(mc)/max(rmse.ukf_R2(mc), 0.001)) * 100;
    imp_fus_vs_R1(mc) = (1 - best_val/rmse.ukf_R1(mc)) * 100;
    imp_fus_vs_R2(mc) = (1 - best_val/rmse.ukf_R2_aligned(mc)) * 100;

    % 航迹分段
    segs1 = extract_segments_mc(trackSnapshots_R1, n_frames);
    segs2 = extract_segments_mc(trackSnapshots_R2, n_frames);
    segs_f = extract_segments_mc(all_fused{best_m}, n_frames);
    mtl_R1(mc) = compute_mtl(segs1);  mtl_R2(mc) = compute_mtl(segs2);
    mtl_fus(mc) = compute_mtl(segs_f);
    brk_R1(mc) = max(0, size(segs1,1)-1);  brk_R2(mc) = max(0, size(segs2,1)-1);
    brk_fus(mc) = max(0, size(segs_f,1)-1);
    seg_count_R1(mc) = size(segs1,1);  seg_count_R2(mc) = size(segs2,1);
    seg_count_fus(mc) = size(segs_f,1);
    seg_info{mc} = struct('R1', segs1, 'R2', segs2, 'FUS', segs_f);

    % 坏种子判断
    if rmse.ukf_R1(mc) > 30 || rmse.ukf_R2(mc) > 30
        bad_seed(mc) = 1;
        bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', rmse.ukf_R1(mc), rmse.ukf_R2(mc));
    end

    fprintf('  RMSE=%.1fkm, 匹配对=%d\n', rmse.fus_best(mc), length(matched_pairs));
end

elapsed = toc;
close all;

%% ========================================================================
%% 汇总统计
%% ========================================================================
fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║          %d 次蒙特卡洛统计汇总 (%.0f s)                     ║\n', N_MC, elapsed);
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

n_bad = sum(bad_seed);
fprintf('坏种子: %d/%d (%.0f%%)\n', n_bad, N_MC, n_bad/N_MC*100);
fprintf('UKF R1: 均值=%.1fkm, std=%.1fkm\n', nanmean(rmse.ukf_R1), nanstd(rmse.ukf_R1));
fprintf('UKF R2: 均值=%.1fkm, std=%.1fkm\n', nanmean(rmse.ukf_R2), nanstd(rmse.ukf_R2));
fprintf('融合最优: 均值=%.1fkm, std=%.1fkm\n', nanmean(rmse.fus_best), nanstd(rmse.fus_best));

for m = 1:length(method_names)
    cnt = sum(strcmp(fus_best_method, method_names{m}));
    fprintf('  %s: %d/%d (%.0f%%)\n', method_names{m}, cnt, N_MC, cnt/N_MC*100);
end

if ~exist('results', 'dir'), mkdir('results'); end
outf = fullfile('results', sprintf('unified_mc_%s_n%d_%s.mat', ...
    trajectory_mode, n_targets, datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'rmse', 'mtl_R1', 'mtl_R2', 'mtl_fus', 'brk_R1', 'brk_R2', 'brk_fus', ...
    'seg_count_R1', 'seg_count_R2', 'seg_count_fus', ...
    'nis_mean_R1', 'nis_mean_R2', 'assoc_R1', 'assoc_R2', ...
    'fus_best_method', 'bad_seed', 'bad_reason', 'seg_info', ...
    'N_MC', 'SEED_BASE', 'trajectory_mode', 'n_targets');
fprintf('\n完整数据已保存: %s\n', outf);
fprintf('Done.\n');

%% ========================================================================
%% 工具函数
%% ========================================================================

function v = rmse_tracks_mc(snaps, truthTrajs, t_grid, n_frames, params)
    errs = [];
    n_ac = length(truthTrajs);
    for k = 1:n_frames
        for a = 1:n_ac
            tt = truthTrajs{a};
            tl = interp1(tt.time_sec, tt.lon, t_grid(k), 'linear', 'extrap');
            tb = interp1(tt.time_sec, tt.lat, t_grid(k), 'linear', 'extrap');
            snap = snaps{k};
            if ~isempty(snap.trackList)
                for t = 1:length(snap.trackList)
                    trk = snap.trackList{t};
                    if trk.type ~= 7 && ~isnan(trk.lat)
                        d = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
                        if d < 200  % 最近邻匹配
                            errs(end+1) = d;
                            break;
                        end
                    end
                end
            end
        end
    end
    v = rms_val(errs);
end

function v = rmse_fusion_mc(snaps, truthTrajs, t_grid, n_frames, matched_pairs)
    errs = [];
    n_ac = length(truthTrajs);
    for k = 1:n_frames
        for a = 1:n_ac
            tt = truthTrajs{a};
            tl = interp1(tt.time_sec, tt.lon, t_grid(k), 'linear', 'extrap');
            tb = interp1(tt.time_sec, tt.lat, t_grid(k), 'linear', 'extrap');
            snap = snaps{k};
            if ~isempty(snap.trackList)
                for t = 1:length(snap.trackList)
                    ft = snap.trackList{t};
                    if ~isnan(ft.lat)
                        d = sphere_utils_haversine_distance(ft.lon, ft.lat, tl, tb) / 1000;
                        if d < 200
                            errs(end+1) = d;
                            break;
                        end
                    end
                end
            end
        end
    end
    v = rms_val(errs);
end

function [assoc_rate, nis_mean, nis_gate, n_assoc, n_pred, init_frame] = diagnose_tracking_mc(snaps, n_frames)
    n_assoc = 0; n_pred = 0; n_init = 0;
    init_frame = 0; nis_vals = [];
    for k = 1:n_frames
        if isempty(snaps{k}.trackList), continue; end
        for t = 1:length(snaps{k}.trackList)
            trk = snaps{k}.trackList{t};
            if trk.type == 6, n_init = n_init + 1;
            elseif trk.type == 1
                if init_frame == 0, init_frame = k; end
                if ~isempty(trk.assoc_det)
                    n_assoc = n_assoc + 1;
                else
                    n_pred = n_pred + 1;
                end
                if isfield(trk, 'nis_history') && ~isempty(trk.nis_history)
                    nis_vals = [nis_vals, trk.nis_history];
                end
            end
        end
    end
    n_tracked = n_assoc + n_pred;
    assoc_rate = n_assoc / max(1, n_tracked) * 100;
    if ~isempty(nis_vals)
        nis_mean = mean(nis_vals);
        nis_gate = sum(nis_vals < 8) / length(nis_vals) * 100;
    else
        nis_mean = NaN; nis_gate = NaN;
    end
end

function segs = extract_segments_mc(snaps, n_frames)
    segs = [];
    in_seg = false; seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}.trackList)
            for t = 1:length(snaps{k}.trackList)
                trk = snaps{k}.trackList{t};
                if trk.type == 1 && ~isnan(trk.lat)
                    is_tracking = true;
                    break;
                end
            end
        end
        if is_tracking && ~in_seg
            in_seg = true; seg_start = k;
        elseif ~is_tracking && in_seg
            in_seg = false;
            segs(end+1, :) = [seg_start, k-1, k - seg_start];
        end
    end
    if in_seg
        segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1];
    end
end

function mtl = compute_mtl(segs)
    if isempty(segs), mtl = 0; else, mtl = mean(segs(:,3)); end
end

function v = rms_val(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end
