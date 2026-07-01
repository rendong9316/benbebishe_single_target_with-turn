% =========================================================================
% run_mc_multi.m — 三目标交叉场景蒙特卡洛仿真
% =========================================================================
% 【定位】
%   三目标交叉场景下，两部雷达各自用 IMM CV+CT 多目标跟踪，
%   跨雷达航迹匹配后执行四种融合算法，逐种子输出对比，
%   最终汇总统计。
%
% 【三目标交叉场景】
%   目标A: 西南→东北
%   目标B: 西北→东南
%   目标C: 西→东
%   三目标在覆盖区中心附近交叉，形成复杂关联歧义
%
% 【输出】
%   汇总：RMSE统计表 + 改善率 + 关联诊断 + 坏种子统计
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ---- 配置 ----
N_MC = 200;  % 三目标场景计算量大，先用200
SEED_BASE = 1;

% 预分配统计数组
n_frames_list = zeros(N_MC, 1);

% RMSE (per aircraft, per radar)
rmse = struct();
for ac = 1:3
    rmse.('raw_A' {ac}) = nan(N_MC, 1);
    rmse.('cal_A' {ac}) = nan(N_MC, 1);
    rmse.('ukf_R1_A' {ac}) = nan(N_MC, 1);
    rmse.('ukf_R2_A' {ac}) = nan(N_MC, 1);
    rmse.('fus_best_A' {ac}) = nan(N_MC, 1);
end
rmse.fus_best_overall = nan(N_MC, 1);

% 融合算法分布
fus_best_method = cell(N_MC, 1);

% MTL & 断裂
mtl_R1 = nan(N_MC, 1);  mtl_R2 = nan(N_MC, 1);  mtl_fus = nan(N_MC, 1);
brk_R1 = nan(N_MC, 1);  brk_R2 = nan(N_MC, 1);  brk_fus = nan(N_MC, 1);

% 关联 + NIS
assoc_R1 = nan(N_MC, 1);  assoc_R2 = nan(N_MC, 1);
nis_mean_R1 = nan(N_MC, 1);  nis_mean_R2 = nan(N_MC, 1);

% 坏种子标记
bad_seed = zeros(N_MC, 1);
bad_reason = cell(N_MC, 1);

%% ---- 打印表头 ----
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     三目标交叉场景蒙特卡洛仿真  N=%d                         ║\n', N_MC);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║ Pd=0.6  Pfa=0.001  dt=30s  N_TARGETS=3  IMM CV+CT           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

tic;

%% ===== 主循环 =====
for mc = 1:N_MC
    seed = SEED_BASE + (mc - 1);
    fprintf('--- MC %d/%d (seed=%d) ---\n', mc, N_MC, seed);

    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    % ---- Phase 0: 三目标航迹生成 ----
    way_A = [127.0, 31.0, 0; 130.0, 34.0, 0];
    way_B = [126.0, 34.0, 0; 130.0, 31.0, 0];
    way_C = [126.0, 32.5, 0; 131.0, 32.5, 0];

    traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
    traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
    traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);

    true_track_A = aircraft_trajectory_interpolate('generate', traj_A);
    true_track_B = aircraft_trajectory_interpolate('generate', traj_B);
    true_track_C = aircraft_trajectory_interpolate('generate', traj_C);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : max(traj_A.duration_sec, traj_B.duration_sec, traj_C.duration_sec);
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : max(traj_A.duration_sec, traj_B.duration_sec, traj_C.duration_sec);
    n_frames = min(length(t1_grid), length(t2_grid));
    n_frames_list(mc) = n_frames;

    % 真值结构体
    truthTrajs = cell(3, 1);
    truth_all_cell = {true_track_A, true_track_B, true_track_C};
    for ac = 1:3
        tt_ac = truth_all_cell{ac};
        truthTrajs{ac} = struct('label', char('A'+ac-1), 'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt_ac(:,5), 'lat', tt_ac(:,2), 'lon', tt_ac(:,1), ...
            'lon_rate', tt_ac(:,3), 'lat_rate', tt_ac(:,4));
    end

    % ---- Phase 1: 标定 ----
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

    % ---- Phase 2+4: 多目标点迹生成 ----
    detList_R1 = cell(n_frames, 1);
    detList_R2 = cell(n_frames, 1);

    rng(params.random_seed + 1e7);
    truth_all_R1 = {true_track_A, true_track_B, true_track_C};
    for k = 1:n_frames
        t1 = t1_grid(k);
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
        if isempty(tgt_states), detList_R1{k} = []; continue; end

        detRaw = generate_frame_detections_multi(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, tgt_states, ...
            k, t1, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
            params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);

        % 偏差校正 + 反解
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
            detRaw(d) = dp;
        end
        detList_R1{k} = detRaw;
    end

    rng(params.random_seed + 2e7);
    truth_all_R2 = {true_track_A, true_track_B, true_track_C};
    for k = 1:n_frames
        t2 = t2_grid(k);
        tgt_states = zeros(3, 5);
        for ac = 1:3
            tt_ac = truth_all_R2{ac};
            if t2 >= tt_ac(1,5) && t2 <= tt_ac(end,5)
                [pos, vel] = aircraft_trajectory_interpolate(tt_ac, t2);
                lr = vel(1); latr = vel(2);
                tgt_states(ac,:) = [pos(1), pos(2), lr, latr, ac];
            else
                tgt_states(ac,:) = [NaN, NaN, NaN, NaN, ac];
            end
        end
        tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);
        if isempty(tgt_states), detList_R2{k} = []; continue; end

        detRaw = generate_frame_detections_multi(params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, tgt_states, ...
            k, t2, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
            params.radar2_beam_center_deg, params, ...
            params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);

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
            detRaw(d) = dp;
        end
        detList_R2{k} = detRaw;
    end

    % ---- Phase 5: 多目标跟踪 ----
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    params.gate_sigma = params.radar1_gate_sigma;
    params.gate_vr_ms = params.radar1_gate_vr_ms;
    params.tracker_K_loss = params.radar1_tracker_K_loss;
    ukf1_tpl = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

    params_r2 = params;
    params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma = params.radar2_gate_sigma;
    params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
    params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
    ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    % R1 多目标跟踪
    trackList_R1 = {};  tempPool_R1 = {};  next_id_R1 = 1;
    trackSnapshots_R1 = cell(n_frames, 1);
    for k = 1:n_frames
        [trackList_R1, tempPool_R1, trackSnapshots_R1{k}, next_id_R1] = ...
            multi_track_runner_kf_mc(trackList_R1, tempPool_R1, detList_R1{k}, ukf1_tpl, ...
            params, k, next_id_R1);
    end

    % R2 多目标跟踪
    trackList_R2 = {};  tempPool_R2 = {};  next_id_R2 = 1;
    trackSnapshots_R2 = cell(n_frames, 1);
    for k = 1:n_frames
        [trackList_R2, tempPool_R2, trackSnapshots_R2{k}, next_id_R2] = ...
            multi_track_runner_kf_mc(trackList_R2, tempPool_R2, detList_R2{k}, ukf2_tpl, ...
            params_r2, k, next_id_R2);
    end

    % ---- Phase 6: 时间对齐 ----
    aligned_R2 = time_align_tracks(trackSnapshots_R2, params);

    % ---- Phase 7: 航迹匹配 + 融合 ----
    matched_pairs = track_matcher(trackSnapshots_R1, aligned_R2, params);

    method_names = {'SCC', 'BC', 'CI', 'FCI'};
    all_fused = cell(length(matched_pairs), length(method_names));

    for p = 1:length(matched_pairs)
        for m = 1:length(method_names)
            all_fused{p,m} = run_track_fusion(matched_pairs(p), ...
                trackSnapshots_R1, aligned_R2, params, method_names{m});
        end
    end

    % ---- 统计 RMSE (取最优融合对) ----
    if ~isempty(matched_pairs)
        truth_all_eval = {true_track_A, true_track_B, true_track_C};
        best_rmse_overall = inf;
        for p = 1:length(matched_pairs)
            for m = 1:length(method_names)
                errs = [];
                for k = 1:n_frames
                    if ~isempty(all_fused{p,m}{k}) && ~isempty(all_fused{p,m}{k}.trackList)
                        ft = all_fused{p,m}{k}.trackList{1};
                        if ~isnan(ft.lat)
                            % 找到对应的目标ID（通过matched_pairs）
                            ac_id = matched_pairs(p).R1_track_id;
                            if ac_id >= 1 && ac_id <= 3
                                tt_ac = truth_all_eval{ac_id};
                                tl = interp1(tt_ac(:,5), tt_ac(:,1), t1_grid(k), 'linear', 'extrap');
                                tb = interp1(tt_ac(:,5), tt_ac(:,2), t1_grid(k), 'linear', 'extrap');
                                errs(end+1) = sphere_utils_haversine_distance(ft.lon, ft.lat, tl, tb) / 1000;
                            end
                        end
                    end
                end
                rmse_val = sqrt(mean(errs.^2));
                if rmse_val < best_rmse_overall
                    best_rmse_overall = rmse_val;
                end
            end
        end
        rmse.fus_best_overall(mc) = best_rmse_overall;
    else
        rmse.fus_best_overall(mc) = inf;
    end

    % 坏种子判断
    if rmse.fus_best_overall(mc) > 50
        bad_seed(mc) = 1;
        bad_reason{mc} = 'FUSION_RMSE_TOO_HIGH';
    end

    fprintf('  RMSE=%.1fkm, 匹配对=%d, 坏种子=%d\n', ...
        rmse.fus_best_overall(mc), length(matched_pairs), bad_seed(mc));
end

elapsed = toc;

%% 汇总统计
fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║              %d 次蒙特卡洛统计汇总 (%.0f s)                  ║\n', N_MC, elapsed);
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

n_bad = sum(bad_seed);
fprintf('坏种子: %d/%d (%.0f%%)\n', n_bad, N_MC, n_bad/N_MC*100);
fprintf('融合最优RMSE: 均值=%.1fkm, std=%.1fkm, 中位=%.1fkm\n', ...
    nanmean(rmse.fus_best_overall), nanstd(rmse.fus_best_overall), nanmedian(rmse.fus_best_overall));

if n_bad > 0
    for mc = 1:N_MC
        if bad_seed(mc)
            fprintf('  seed=%d: %s\n', SEED_BASE+mc-1, bad_reason{mc});
        end
    end
end

% 保存数据
if ~exist('results', 'dir'), mkdir('results'); end
outf = fullfile('results', sprintf('mc_multi_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'rmse', 'mtl_R1', 'mtl_R2', 'mtl_fus', 'brk_R1', 'brk_R2', 'brk_fus', ...
    'assoc_R1', 'assoc_R2', 'nis_mean_R1', 'nis_mean_R2', ...
    'fus_best_method', 'bad_seed', 'bad_reason', 'N_MC', 'SEED_BASE');
fprintf('\n完整数据已保存: %s\n', outf);
fprintf('Done.\n');

% =========================================================================
% 内部函数
% =========================================================================

function [trackList, tempPool, snap, next_id] = multi_track_runner_kf_mc(trackList, tempPool, detList_k, ukf_tpl, ...
        params, frame_id, next_id)
    % 多目标单帧跟踪包装器（蒙特卡洛轻量版）

    if frame_id == 1 && ~isempty(detList_k)
        for d = 1:min(3, length(detList_k))
            dp = detList_k(d);
            new_ukf = ukf_imm('init', ukf_tpl, dp, dp);
            new_ukf = post_init_multi(new_ukf, params);
            trk = struct('id', next_id, 'type', 1, 'lat', dp.lat, 'lon', dp.lon, ...
                'ukf', new_ukf, 'life', 1, 'quality', 10, 'missed', 0, ...
                'assoc_det', dp, 'nis_history', []);
            trackList{end+1} = trk;
            next_id = next_id + 1;
        end
        used = false(1, length(detList_k));
        for d = 1:min(3, length(detList_k)), used(d) = true; end
        detList_k = detList_k(~used);
    end

    active_idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7
            active_idx(end+1) = t;
        end
    end
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.dt = params.dt_sec;
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ukf_dispatch('prepare', trk.ukf);
        trk.x_pred = x_pred; trk.P_pred = P_pred; trk.X_pred = X_pred;
        trk.z_pred = z_pred; trk.Z_pred = Z_pred; trk.P_zz = P_zz;
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    % JPDA 关联
    [assoc_pairs, dets_in_gate, innov_w] = jpda_multi(trackList, active_idx, detList_k, params);
    point_used = false(1, length(detList_k));
    track_has_assoc = false(1, length(active_idx));
    for p = 1:size(assoc_pairs, 1)
        point_used(assoc_pairs(p, 2)) = true;
        [~, loc] = ismember(assoc_pairs(p, 1), active_idx);
        if loc > 0, track_has_assoc(loc) = true; end
    end

    % 用 JPDA 加权新息更新
    for i = 1:length(active_idx)
        t = active_idx(i); trk = trackList{t};
        if ~isempty(dets_in_gate{i}) && ~isempty(innov_w{i})
            innov = innov_w{i};
            nis_val = innov' * (trk.P_zz(1:2,1:2) \ innov);
            if ~isfield(trk, 'nis_history'), trk.nis_history = []; end
            trk.nis_history(end+1) = nis_val;
            if length(trk.nis_history) > params.fuzzy_window_size, trk.nis_history(1) = []; end
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, innov);
            if ~isempty(dets_in_gate{i}), trk.assoc_det = dets_in_gate{i}{1}; end
        else
            trk.ukf.x = trk.x_pred; trk.ukf.P = trk.P_pred; nis_val = NaN;
            trk.assoc_det = [];
        end
        trk.lat = trk.ukf.x(3); trk.lon = trk.ukf.x(1);
        trk.missed = 0; trk.life = trk.life + 1;
        trackList{t} = trk;
    end

    for i = 1:length(active_idx)
        if track_has_assoc(i), continue; end
        t = active_idx(i); trk = trackList{t};
        trk.ukf.x = trk.x_pred; trk.ukf.P = trk.P_pred;
        trk.missed = trk.missed + 1; trk.life = trk.life + 1;
        trk.lat = trk.ukf.x(3); trk.lon = trk.ukf.x(1); trk.assoc_det = [];
        trackList{t} = trk;
    end

    trackList = track_management('quality', trackList, active_idx, params, frame_id);

    unused_dets = detList_k(~point_used);
    if ~isempty(unused_dets)
        [new_state, det1, det2, success] = multi_track_start([], unused_dets, params, frame_id);
        if success
            new_ukf = ukf_imm('init', ukf_tpl, det1, det2);
            new_ukf = post_init_multi(new_ukf, params);
            trk = struct('id', next_id, 'type', 6, 'lat', det2.lat, 'lon', det2.lon, ...
                'ukf', new_ukf, 'life', 1, 'quality', 0, 'missed', 0, ...
                'assoc_det', det2, 'nis_history', []);
            trackList{end+1} = trk;
            next_id = next_id + 1;
        end
    end

    snap.trackList = trackList;
    snap.frameID = frame_id;
end

function ukf = post_init_multi(ukf, params)
    ukf.dt = params.dt_sec;
    ukf.initialized = true;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.dt = params.dt_sec;
        ukf.ukf_cv.initialized = true;
        ukf.ukf_ct.dt = params.dt_sec;
        ukf.ukf_ct.initialized = true;
    end
    ukf.nis_history = [];
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        if isfield(ukf, 'Q')
            ukf.Q_base = ukf.Q;
        end
    end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
end
