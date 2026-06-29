% run_kloss_compare.m — 对比 R1 K_loss=4/5/6 的 MC 表现
% 在破 Toeplitz 结构的新 rng 流下，三种配置各跑 N=200 次

clear; close all; clc;
addpath(genpath('.'));

N_MC = 200;
SEED_BASE = 1;
kloss_values = [4, 5, 6];
n_configs = length(kloss_values);

% 存储汇总结果
summary = struct();
summary.kloss = kloss_values;
summary.n_bad = zeros(1, n_configs);
summary.bad_rate = zeros(1, n_configs);
summary.ukf_R1_mean = zeros(1, n_configs);
summary.ukf_R1_median = zeros(1, n_configs);
summary.ukf_R1_std = zeros(1, n_configs);
summary.ukf_R2_mean = zeros(1, n_configs);
summary.ukf_R2_median = zeros(1, n_configs);
summary.fus_best_mean = zeros(1, n_configs);
summary.fus_best_median = zeros(1, n_configs);
summary.mtl_R1_mean = zeros(1, n_configs);
summary.mtl_R2_mean = zeros(1, n_configs);
summary.mtl_fus_mean = zeros(1, n_configs);
summary.brk_R1_mean = zeros(1, n_configs);
summary.brk_R2_mean = zeros(1, n_configs);
summary.assoc_R1_mean = zeros(1, n_configs);
summary.assoc_R2_mean = zeros(1, n_configs);
summary.nis_R1_mean = zeros(1, n_configs);
summary.nis_R2_mean = zeros(1, n_configs);
summary.cal_R1_mean = zeros(1, n_configs);
summary.cal_R2_mean = zeros(1, n_configs);
summary.imp_ukf_R1_mean = zeros(1, n_configs);
summary.imp_ukf_R2_mean = zeros(1, n_configs);
summary.imp_fus_mean = zeros(1, n_configs);
summary.elapsed = zeros(1, n_configs);
summary.detection_rate_R1 = zeros(1, n_configs);
summary.detection_rate_R2 = zeros(1, n_configs);

for ci = 1:n_configs
    kl = kloss_values(ci);
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  配置 %d/%d: R1 K_loss=%d  R2 K_loss=6  (连续流rng)        ║\n', ...
        ci, n_configs, kl);
    fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

    tic;

    % 预分配
    n_frames_list = zeros(N_MC, 1);
    cal_R1 = nan(N_MC, 1);  cal_R2 = nan(N_MC, 1);
    ukf_R1 = nan(N_MC, 1);  ukf_R2 = nan(N_MC, 1);
    fus_best = nan(N_MC, 1);
    mtl_R1 = nan(N_MC, 1);  mtl_R2 = nan(N_MC, 1);  mtl_f = nan(N_MC, 1);
    brk_R1 = nan(N_MC, 1);  brk_R2 = nan(N_MC, 1);
    assoc_R1 = nan(N_MC, 1);  assoc_R2 = nan(N_MC, 1);
    nis_R1 = nan(N_MC, 1);    nis_R2 = nan(N_MC, 1);
    imp_ukf_r1 = nan(N_MC, 1);  imp_ukf_r2 = nan(N_MC, 1);
    imp_fus = nan(N_MC, 1);
    bad_seed = zeros(N_MC, 1);
    bad_reason = cell(N_MC, 1);
    det_rate_R1 = nan(N_MC, 1);  det_rate_R2 = nan(N_MC, 1);

    for mc = 1:N_MC
        seed = SEED_BASE + (mc - 1);

        params = simulation_params();
        params.random_seed = seed;
        params.radar1_tracker_K_loss = kl;      % <-- 变量 R1 K_loss
        params.tracker_K_loss = kl;
        rng(params.random_seed);

        traj = aircraft_trajectory_create(params.aircraft_waypoints, ...
            params.aircraft_speed_ms, params.dt_sec);
        true_track = aircraft_trajectory_interpolate('generate', traj);
        t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
        n_frames = min(length(t1_grid), length(t2_grid));
        n_frames_list(mc) = n_frames;

        % Phase 1: ADS-B 标定
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

        % Phase 2+4: 点迹生成 + 偏差校正 (连续流 rng, 破 Toeplitz)
        detList_R1 = cell(n_frames, 1);  detList_R2 = cell(n_frames, 1);

        rng(params.random_seed + 1e7);  % R1: 独立流
        n_det_R1 = 0;
        for k = 1:n_frames
            [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
            detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t1_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            for d = 1:length(detRaw)
                if ~detRaw(d).is_clutter, n_det_R1 = n_det_R1 + 1; end
                detRaw(d).aircraft_id = 1;
                Rgc = detRaw(d).prange - dr1_est;  azc = detRaw(d).paz - da1_est;
                detRaw(d).drange = Rgc;  detRaw(d).daz = azc;
                detRaw(d).range_meas = Rgc;  detRaw(d).azimuth_meas = azc;
                if ~(isfield(detRaw(d), 'lat') && ~isnan(detRaw(d).lat))
                    [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                    detRaw(d).lat = lat_e;  detRaw(d).lon = lon_e;
                end
                [~, raw_lat, raw_lon] = bistatic_inverse_solver(detRaw(d).prange, detRaw(d).paz, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                detRaw(d).raw_lat = raw_lat;  detRaw(d).raw_lon = raw_lon;
            end
            detList_R1{k} = detRaw;
        end
        det_rate_R1(mc) = n_det_R1 / n_frames * 100;

        rng(params.random_seed + 2e7);  % R2: 独立流
        n_det_R2 = 0;
        for k = 1:n_frames
            [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
            detRaw2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, pos2(1), pos2(2), vel2(1), vel2(2), ...
                k, t2_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            for d = 1:length(detRaw2)
                if ~detRaw2(d).is_clutter, n_det_R2 = n_det_R2 + 1; end
                detRaw2(d).aircraft_id = 1;
                Rgc = detRaw2(d).prange - dr2_est;  azc = detRaw2(d).paz - da2_est;
                detRaw2(d).drange = Rgc;  detRaw2(d).daz = azc;
                detRaw2(d).range_meas = Rgc;  detRaw2(d).azimuth_meas = azc;
                if ~(isfield(detRaw2(d), 'lat') && ~isnan(detRaw2(d).lat))
                    [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                    detRaw2(d).lat = lat_e;  detRaw2(d).lon = lon_e;
                end
                [~, raw_lat, raw_lon] = bistatic_inverse_solver(detRaw2(d).prange, detRaw2(d).paz, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                detRaw2(d).raw_lat = raw_lat;  detRaw2(d).raw_lon = raw_lon;
            end
            detList_R2{k} = detRaw2;
        end
        det_rate_R2(mc) = n_det_R2 / n_frames * 100;

        % RMSE 点迹
        cal_R1(mc) = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'cal');
        cal_R2(mc) = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'cal');

        % Phase 5: UKF
        params.ukf_range_std_m = params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale = params.radar1_ukf_Q_scale;
        params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
        params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
        params.gate_sigma = params.radar1_gate_sigma;
        params.gate_vr_ms = params.radar1_gate_vr_ms;
        params.tracker_K_loss = kl;  % R1 K_loss
        ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        [snaps_R1, ~] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames, true_track, t1_grid);

        params_r2 = params;
        params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        params_r2.gate_sigma = params.radar2_gate_sigma;
        params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
        params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
        params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
        params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
        params_r2.tracker_M = 4;  params_r2.tracker_N = 8;
        params_r2.tracker_K_loss = 6;  % R2 K_loss 固定为 6
        ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
        [snaps_R2, ~] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames, true_track, t2_grid);

        ukf_R1(mc) = rmse_tracks(snaps_R1, true_track, t1_grid, n_frames);
        ukf_R2(mc) = rmse_tracks(snaps_R2, true_track, t2_grid, n_frames);
        imp_ukf_r1(mc) = (1 - ukf_R1(mc)/cal_R1(mc)) * 100;
        imp_ukf_r2(mc) = (1 - ukf_R2(mc)/cal_R2(mc)) * 100;

        % NIS + 关联
        [assoc_R1(mc), nis_R1(mc), ~] = diagnose_tracking(snaps_R1, n_frames);
        [assoc_R2(mc), nis_R2(mc), ~] = diagnose_tracking(snaps_R2, n_frames);

        % Phase 6: 时间对齐
        aligned_R2 = time_align_tracks(snaps_R2, params);

        % Phase 7: 融合
        matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
            'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
            'mean_dist_km', 0, 'quality', 100);
        method_names = {'SCC', 'BC', 'CI', 'FCI'};
        all_fused = cell(length(method_names), 1);
        fus_rmses = nan(1, 4);
        for m = 1:length(method_names)
            all_fused{m} = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, method_names{m});
            fus_rmses(m) = rmse_fusion_snaps(all_fused{m}, true_track, t1_grid, n_frames);
        end
        [best_val, ~] = min(fus_rmses);
        fus_best(mc) = best_val;
        imp_fus(mc) = (1 - best_val/ukf_R1(mc)) * 100;

        % 航迹分段
        segs1 = extract_segments(snaps_R1, n_frames);
        segs2 = extract_segments(snaps_R2, n_frames);
        mtl_R1(mc) = compute_mtl(segs1);  mtl_R2(mc) = compute_mtl(segs2);
        brk_R1(mc) = max(0, size(segs1,1)-1);  brk_R2(mc) = max(0, size(segs2,1)-1);

        % 坏种子
        if ukf_R1(mc) > 30 || ukf_R2(mc) > 30
            bad_seed(mc) = 1;
            bad_reason{mc} = sprintf('DIVERGED');
        elseif imp_ukf_r1(mc) < -50 || imp_ukf_r2(mc) < -50
            bad_seed(mc) = 1;
            bad_reason{mc} = sprintf('DEGRADED');
        end

        if mod(mc, 20) == 0
            elapsed = toc;
            fprintf('  [%d/%d] %.0fs  bad=%d  UKF R1=%.1f R2=%.1f  Fus=%.1f\n', ...
                mc, N_MC, elapsed, sum(bad_seed), nanmedian(ukf_R1(1:mc)), ...
                nanmedian(ukf_R2(1:mc)), nanmedian(fus_best(1:mc)));
        end
    end

    elapsed_total = toc;

    % 汇总
    v = ukf_R1(~isnan(ukf_R1)); summary.ukf_R1_mean(ci) = mean(v); summary.ukf_R1_median(ci) = median(v); summary.ukf_R1_std(ci) = std(v);
    v = ukf_R2(~isnan(ukf_R2)); summary.ukf_R2_mean(ci) = mean(v); summary.ukf_R2_median(ci) = median(v);
    v = fus_best(~isnan(fus_best)); summary.fus_best_mean(ci) = mean(v); summary.fus_best_median(ci) = median(v);
    v = cal_R1(~isnan(cal_R1)); summary.cal_R1_mean(ci) = mean(v);
    v = cal_R2(~isnan(cal_R2)); summary.cal_R2_mean(ci) = mean(v);
    v = mtl_R1(~isnan(mtl_R1)); summary.mtl_R1_mean(ci) = mean(v);
    v = mtl_R2(~isnan(mtl_R2)); summary.mtl_R2_mean(ci) = mean(v);
    v = assoc_R1(~isnan(assoc_R1)); summary.assoc_R1_mean(ci) = mean(v);
    v = assoc_R2(~isnan(assoc_R2)); summary.assoc_R2_mean(ci) = mean(v);
    v = nis_R1(~isnan(nis_R1)); summary.nis_R1_mean(ci) = mean(v);
    v = nis_R2(~isnan(nis_R2)); summary.nis_R2_mean(ci) = mean(v);
    v = brk_R1(~isnan(brk_R1)); summary.brk_R1_mean(ci) = mean(v);
    v = brk_R2(~isnan(brk_R2)); summary.brk_R2_mean(ci) = mean(v);
    summary.imp_ukf_R1_mean(ci) = nanmean(imp_ukf_r1);
    summary.imp_ukf_R2_mean(ci) = nanmean(imp_ukf_r2);
    summary.imp_fus_mean(ci) = nanmean(imp_fus);
    summary.n_bad(ci) = sum(bad_seed);
    summary.bad_rate(ci) = sum(bad_seed) / N_MC * 100;
    summary.elapsed(ci) = elapsed_total;
    summary.detection_rate_R1(ci) = nanmean(det_rate_R1);
    summary.detection_rate_R2(ci) = nanmean(det_rate_R2);

    % 坏种子详细
    summary.bad_seeds{ci} = find(bad_seed);

    fprintf('\n  K_loss=%d 完成: %.0fs  坏种子=%d/200 (%.1f%%)\n', ...
        kl, elapsed_total, sum(bad_seed), sum(bad_seed)/N_MC*100);
    fprintf('  UKF R1: mean=%.1f median=%.1f  UKF R2: mean=%.1f median=%.1f\n', ...
        summary.ukf_R1_mean(ci), summary.ukf_R1_median(ci), ...
        summary.ukf_R2_mean(ci), summary.ukf_R2_median(ci));
    fprintf('  融合最优: mean=%.1f median=%.1f  MTL R1=%.1f  MTL R2=%.1f\n', ...
        summary.fus_best_mean(ci), summary.fus_best_median(ci), ...
        summary.mtl_R1_mean(ci), summary.mtl_R2_mean(ci));
end

%% ========================================================================
%% 汇总对比
%% ========================================================================
fprintf('\n\n');
fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║            R1 K_loss 对比汇总 (R2固定=6, 连续流rng)                 ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  %-16s %10s %10s %10s  ║\n', '指标', 'K=4', 'K=5', 'K=6');
fprintf('║  %-16s %10s %10s %10s  ║\n', '──', '──', '──', '──');
fprintf('║  %-16s %9.1f%% %9.1f%% %9.1f%%  ║\n', '坏种子率', summary.bad_rate);
fprintf('║  %-16s %9d %9d %9d  ║\n', '坏种子数', summary.n_bad);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', 'UKF R1 均值(km)', summary.ukf_R1_mean);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', 'UKF R1 中位(km)', summary.ukf_R1_median);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', 'UKF R1 std(km)', summary.ukf_R1_std);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', 'UKF R2 中位(km)', summary.ukf_R2_median);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', '融合最优 中位(km)', summary.fus_best_median);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', 'MTL R1(帧)', summary.mtl_R1_mean);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', 'MTL R2(帧)', summary.mtl_R2_mean);
fprintf('║  %-16s %10.2f %10.2f %10.2f  ║\n', '断裂R1(次)', summary.brk_R1_mean);
fprintf('║  %-16s %10.2f %10.2f %10.2f  ║\n', '断裂R2(次)', summary.brk_R2_mean);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', '关联率R1(%%)', summary.assoc_R1_mean);
fprintf('║  %-16s %10.1f %10.1f %10.1f  ║\n', '关联率R2(%%)', summary.assoc_R2_mean);
fprintf('║  %-16s %10.2f %10.2f %10.2f  ║\n', 'NIS R1', summary.nis_R1_mean);
fprintf('║  %-16s %10.2f %10.2f %10.2f  ║\n', 'NIS R2', summary.nis_R2_mean);
fprintf('║  %-16s %9.1f%% %9.1f%% %9.1f%%  ║\n', 'UKF改善R1', summary.imp_ukf_R1_mean);
fprintf('║  %-16s %9.1f%% %9.1f%% %9.1f%%  ║\n', 'UKF改善R2', summary.imp_ukf_R2_mean);
fprintf('║  %-16s %9.1f%% %9.1f%% %9.1f%%  ║\n', '融合vsR1改善', summary.imp_fus_mean);
fprintf('║  %-16s %10.0f %10.0f %10.0f  ║\n', '耗时(s)', summary.elapsed);
fprintf('║  %-16s %9.1f%% %9.1f%% %9.1f%%  ║\n', '检测率R1', summary.detection_rate_R1);
fprintf('║  %-16s %9.1f%% %9.1f%% %9.1f%%  ║\n', '检测率R2', summary.detection_rate_R2);
fprintf('╠══════════════════════════════════════════════════════════════════════╣\n');

% 判断最佳
[~, best_idx] = min(summary.bad_rate);
fprintf('║  结论: R1 K_loss=%d 坏种子率最低(%.1f%%)                          ║\n', ...
    kloss_values(best_idx), summary.bad_rate(best_idx));
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n');

if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'kloss_compare.mat'), 'summary', 'kloss_values', 'N_MC');
fprintf('\n结果已保存: results/kloss_compare.mat\n');
fprintf('Done.\n');


%% ========================================================================
%% 工具函数
%% ========================================================================

function v = rmse_detlist(detList, true_track, t_grid, n_frames, mode)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        for d = 1:length(detList{k})
            dp = detList{k}(d);
            if dp.is_clutter, continue; end
            if isfield(dp, 'lat') && ~isnan(dp.lat)
                errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
            end
        end
    end
    v = rms_val(errs);
end

function v = rmse_tracks(snaps, true_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        snap = snaps{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if isfield(trk, 'type') && trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            elseif ~isfield(trk, 'type') && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = rms_val(errs);
end

function v = rmse_fusion_snaps(snaps, true_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = rms_val(errs);
end

function [assoc_rate, nis_mean, nis_gate] = diagnose_tracking(snaps, n_frames)
    n_assoc = 0; n_pred = 0; nis_vals = [];
    for k = 1:n_frames
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 1
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
               isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;
            else
                n_pred = n_pred + 1;
            end
            if isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
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

function segs = extract_segments(snaps, n_frames)
    segs = [];
    in_seg = false; seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if isfield(trk, 'type') && trk.type == 1 && ~isnan(trk.lat)
                is_tracking = true;
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
