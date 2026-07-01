% =====================================================================
% run_mc_straight.m — 直线场景蒙特卡洛仿真
% =========================================================================
% 完全遵循 run_simulation.m 的 Phase 0-7 流水线。
% 架构: 首次真值辅助起始 + 重新纯M/N + 基础UKF + 模糊自适应Q。
% 无图窗，完整控制台输出：各阶段RMSE、航迹分段、MTL、NIS、改善率。
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ---- 配置 ----
N_MC = 500;
SEED_BASE = 1;  % 种子 = SEED_BASE + (mc-1)

% 预分配统计数组
n_frames_list = zeros(N_MC, 1);

% RMSE
rmse.raw_R1 = nan(N_MC, 1);  rmse.raw_R2 = nan(N_MC, 1);
rmse.cal_R1 = nan(N_MC, 1);  rmse.cal_R2 = nan(N_MC, 1);
rmse.ukf_R1 = nan(N_MC, 1);  rmse.ukf_R2 = nan(N_MC, 1);
rmse.ukf_R2_aligned = nan(N_MC, 1);
rmse.fus = nan(N_MC, 4);  % SCC/BC/CI/FCI
rmse.fus_best = nan(N_MC, 1);
fus_best_method = cell(N_MC, 1);

% MTL & 断裂
mtl_R1 = nan(N_MC, 1);  mtl_R2 = nan(N_MC, 1);  mtl_fus = nan(N_MC, 1);
brk_R1 = nan(N_MC, 1);  brk_R2 = nan(N_MC, 1);  brk_fus = nan(N_MC, 1);
seg_count_R1 = nan(N_MC, 1);  seg_count_R2 = nan(N_MC, 1);  seg_count_fus = nan(N_MC, 1);

% NIS + 关联
nis_mean_R1 = nan(N_MC, 1);  nis_mean_R2 = nan(N_MC, 1);
nis_gate_R1 = nan(N_MC, 1);  nis_gate_R2 = nan(N_MC, 1);
assoc_R1 = nan(N_MC, 1);     assoc_R2 = nan(N_MC, 1);
init_frame_R1 = nan(N_MC, 1); init_frame_R2 = nan(N_MC, 1);

% 改善率
imp_ukf_R1 = nan(N_MC, 1);  imp_ukf_R2 = nan(N_MC, 1);
imp_fus_vs_R1 = nan(N_MC, 1);  imp_fus_vs_R2 = nan(N_MC, 1);

% 坏种子标记
bad_seed = zeros(N_MC, 1);
bad_reason = cell(N_MC, 1);

% 分段详情（每个种子保存）
seg_info = cell(N_MC, 1);

%% ---- 打印表头 ----
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     直线场景蒙特卡洛仿真  N=%d  架构:真值辅助+纯M/N        ║\n', N_MC);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║ Pd=0.6  Pfa=0.001  R1_Kloss=4 R2_Kloss=6  dt=30s  n_frames≈52  ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

tic;

%% ===== 主循环 =====
for mc = 1:N_MC
    seed = SEED_BASE + (mc - 1);
    rng('default');

    %% ---------- Phase 0-7: 完整流水线 ----------
    % (内部逻辑完全不变，省略重复注释)
    params = simulation_params();
    params.random_seed = seed;
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

    % Phase 2+4: 点迹生成 + 偏差校正
    % 关键设计: 每部雷达仅调用 rng() 一次，帧间随机流连续推进。
    % 不同种子间使用不同偏移(1e7/2e7)完全隔离随机流，
    % 打破旧 rng(seed+k) 的 Toeplitz 对角线相关性。
    detList_R1 = cell(n_frames, 1);  detList_R2 = cell(n_frames, 1);

    rng(params.random_seed + 1e7);  % R1: 独立随机流，连续推进
    for k = 1:n_frames
        [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
        detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
            k, t1_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
            params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
        for d = 1:length(detRaw)
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

    rng(params.random_seed + 2e7);  % R2: 独立随机流，与R1完全隔离
    for k = 1:n_frames
        [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
        detRaw2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, pos2(1), pos2(2), vel2(1), vel2(2), ...
            k, t2_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
            params.radar2_beam_center_deg, params, ...
            params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
        for d = 1:length(detRaw2)
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

    % Phase 4b: 点迹RMSE
    rmse.raw_R1(mc) = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'raw');
    rmse.raw_R2(mc) = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'raw');
    rmse.cal_R1(mc) = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'cal');
    rmse.cal_R2(mc) = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'cal');

    % Phase 5: UKF
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    params.gate_sigma = params.radar1_gate_sigma;
    params.gate_vr_ms = params.radar1_gate_vr_ms;
    params.tracker_K_loss = params.radar1_tracker_K_loss;
    ukf1_tpl = ukf_zishiying('create', params, params.radar1_lon, params.radar1_lat, ...
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
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
    ukf2_tpl = ukf_zishiying('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    [snaps_R2, ~] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames, true_track, t2_grid);

    rmse.ukf_R1(mc) = rmse_tracks(snaps_R1, true_track, t1_grid, n_frames);
    rmse.ukf_R2(mc) = rmse_tracks(snaps_R2, true_track, t2_grid, n_frames);
    imp_ukf_R1(mc) = (1 - rmse.ukf_R1(mc)/rmse.cal_R1(mc)) * 100;
    imp_ukf_R2(mc) = (1 - rmse.ukf_R2(mc)/rmse.cal_R2(mc)) * 100;

    % NIS + 关联
    [assoc_R1(mc), nis_mean_R1(mc), nis_gate_R1(mc), n_assoc1, n_pred1, init_frame_R1(mc)] = ...
        diagnose_tracking(snaps_R1, n_frames);
    [assoc_R2(mc), nis_mean_R2(mc), nis_gate_R2(mc), n_assoc2, n_pred2, init_frame_R2(mc)] = ...
        diagnose_tracking(snaps_R2, n_frames);

    % Phase 6: 时间对齐
    aligned_R2 = time_align_tracks(snaps_R2, params);
    rmse.ukf_R2_aligned(mc) = rmse_tracks(aligned_R2, true_track, t1_grid, n_frames);

    % Phase 7: 融合
    matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);
    method_names = {'SCC', 'BC', 'CI', 'FCI'};
    all_fused = cell(length(method_names), 1);
    for m = 1:length(method_names)
        all_fused{m} = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, method_names{m});
        rmse.fus(mc, m) = rmse_fusion_snaps(all_fused{m}, true_track, t1_grid, n_frames);
    end
    [best_val, best_m] = min(rmse.fus(mc, :));
    rmse.fus_best(mc) = best_val;
    fus_best_method{mc} = method_names{best_m};
    imp_fus_vs_R1(mc) = (1 - best_val/rmse.ukf_R1(mc)) * 100;
    imp_fus_vs_R2(mc) = (1 - best_val/rmse.ukf_R2_aligned(mc)) * 100;

    % 航迹分段
    segs1 = extract_segments(snaps_R1, n_frames);
    segs2 = extract_segments(snaps_R2, n_frames);
    segs_f = extract_fusion_segments(all_fused{best_m}, n_frames);
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
    elseif imp_ukf_R1(mc) < -50 || imp_ukf_R2(mc) < -50
        bad_seed(mc) = 1;
        bad_reason{mc} = sprintf('DEGRADED R1=%+.0f%% R2=%+.0f%%', imp_ukf_R1(mc), imp_ukf_R2(mc));
    end

    %% ---- 逐种子详细输出 ----
    fprintf('───── MC #%d (seed=%d) ─────\n', mc, seed);
    fprintf('  Phase4: R1 原始%.0fkm 校准%.1fkm | R2 原始%.0fkm 校准%.1fkm\n', ...
        rmse.raw_R1(mc), rmse.cal_R1(mc), rmse.raw_R2(mc), rmse.cal_R2(mc));
    fprintf('  Phase5: R1 UKF=%.1fkm(%+.0f%%) 起始=%d 关联=%d+%d(%.0f%%) NIS=%.2f(%.0f%%门内)\n', ...
        rmse.ukf_R1(mc), imp_ukf_R1(mc), init_frame_R1(mc), ...
        n_assoc1, n_pred1, assoc_R1(mc), nis_mean_R1(mc), nis_gate_R1(mc));
    fprintf('          R2 UKF=%.1fkm(%+.0f%%) 起始=%d 关联=%d+%d(%.0f%%) NIS=%.2f(%.0f%%门内)\n', ...
        rmse.ukf_R2(mc), imp_ukf_R2(mc), init_frame_R2(mc), ...
        n_assoc2, n_pred2, assoc_R2(mc), nis_mean_R2(mc), nis_gate_R2(mc));
    fprintf('  Phase7: 融合 %s=%.1fkm vs R1(%+.0f%%) vs R2(%+.0f%%)\n', ...
        fus_best_method{mc}, rmse.fus_best(mc), imp_fus_vs_R1(mc), imp_fus_vs_R2(mc));

    % 分段打印
    fprintf('  Segments: R1(%d段) ', seg_count_R1(mc));
    for s = 1:size(segs1,1)
        fprintf('[%d-%d:%d] ', segs1(s,1), segs1(s,2), segs1(s,3));
    end
    fprintf('MTL=%.1f\n', mtl_R1(mc));
    fprintf('            R2(%d段) ', seg_count_R2(mc));
    for s = 1:size(segs2,1)
        fprintf('[%d-%d:%d] ', segs2(s,1), segs2(s,2), segs2(s,3));
    end
    fprintf('MTL=%.1f\n', mtl_R2(mc));
    fprintf('            FUS(%d段) ', seg_count_fus(mc));
    for s = 1:size(segs_f,1)
        fprintf('[%d-%d:%d] ', segs_f(s,1), segs_f(s,2), segs_f(s,3));
    end
    fprintf('MTL=%.1f\n', mtl_fus(mc));

    if bad_seed(mc)
        fprintf('  *** BAD SEED: %s ***\n', bad_reason{mc});
    end
    fprintf('\n');
end

elapsed = toc;
close all;

%% ========================================================================
%% 汇总统计
%% ========================================================================
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║              %d 次蒙特卡洛统计汇总 (%.0f s)                  ║\n', N_MC, elapsed);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

%% ---- RMSE 汇总 ----
fprintf('║                                                              ║\n');
fprintf('║  ─── RMSE 绝对值 (km) ───                                   ║\n');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
print_mc_row('原始点迹 R1', rmse.raw_R1);
print_mc_row('原始点迹 R2', rmse.raw_R2);
print_mc_row('校准后 R1', rmse.cal_R1);
print_mc_row('校准后 R2', rmse.cal_R2);
print_mc_row('UKF R1', rmse.ukf_R1);
print_mc_row('UKF R2(对齐)', rmse.ukf_R2_aligned);
print_mc_row('融合 SCC', rmse.fus(:,1));
print_mc_row('融合 BC', rmse.fus(:,2));
print_mc_row('融合 CI', rmse.fus(:,3));
print_mc_row('融合 FCI', rmse.fus(:,4));
print_mc_row('融合最优', rmse.fus_best);

%% ---- 改善率汇总 ----
fprintf('║                                                              ║\n');
fprintf('║  ─── 阶段改善率 (%%) ───                                    ║\n');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');

cal_imp_R1 = (1 - rmse.cal_R1 ./ rmse.raw_R1) * 100;
cal_imp_R2 = (1 - rmse.cal_R2 ./ rmse.raw_R2) * 100;
print_mc_row('校准改善 R1', cal_imp_R1);
print_mc_row('校准改善 R2', cal_imp_R2);
print_mc_row('UKF改善 R1', imp_ukf_R1);
print_mc_row('UKF改善 R2', imp_ukf_R2);
print_mc_row('融合 vs R1', imp_fus_vs_R1);
print_mc_row('融合 vs R2', imp_fus_vs_R2);

%% ---- MTL + 断裂汇总 ----
fprintf('║                                                              ║\n');
fprintf('║  ─── MTL 航迹平均长度 (帧) ───                              ║\n');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
print_mc_row('MTL R1', mtl_R1);
print_mc_row('MTL R2', mtl_R2);
print_mc_row('MTL 融合', mtl_fus);

mtl_imp = (mtl_fus ./ max(mtl_R1, mtl_R2) - 1) * 100;
fprintf('║  MTL 融合延长              %+6.1f%% (%.1f→%.1f)               ║\n', ...
    nanmean(mtl_imp), nanmean(max(mtl_R1, mtl_R2)), nanmean(mtl_fus));

fprintf('║                                                              ║\n');
fprintf('║  ─── 断裂次数 ───                                           ║\n');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
print_mc_row('断裂 R1', brk_R1);
print_mc_row('断裂 R2', brk_R2);
print_mc_row('断裂 融合', brk_fus);

%% ---- 关联 + NIS 汇总 ----
fprintf('║                                                              ║\n');
fprintf('║  ─── 关联诊断 ───                                           ║\n');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
print_mc_row('关联率 R1(%)', assoc_R1);
print_mc_row('关联率 R2(%)', assoc_R2);
print_mc_row('NIS均值 R1', nis_mean_R1);
print_mc_row('NIS均值 R2', nis_mean_R2);
print_mc_row('NIS门内 R1(%)', nis_gate_R1);
print_mc_row('NIS门内 R2(%)', nis_gate_R2);
print_mc_row('起始帧号 R1', init_frame_R1);
print_mc_row('起始帧号 R2', init_frame_R2);

%% ---- 坏种子统计 ----
n_bad = sum(bad_seed);
fprintf('║                                                              ║\n');
fprintf('║  ─── 坏种子: %d/%d (%.0f%%) ───                            ║\n', ...
    n_bad, N_MC, n_bad/N_MC*100);
if n_bad > 0
    for mc = 1:N_MC
        if bad_seed(mc)
            fprintf('║    seed=%d: %s  ║\n', SEED_BASE+mc-1, bad_reason{mc});
        end
    end
end

%% ---- 融合算法分布 ----
fprintf('║                                                              ║\n');
fprintf('║  ─── 最优融合算法分布 ───                                   ║\n');
for m = 1:length(method_names)
    cnt = sum(strcmp(fus_best_method, method_names{m}));
    fprintf('║    %s: %d/%d (%.0f%%)                                          ║\n', ...
        method_names{m}, cnt, N_MC, cnt/N_MC*100);
end

fprintf('╚══════════════════════════════════════════════════════════════╝\n');

%% ---- 保存数据 ----
if ~exist('results', 'dir'), mkdir('results'); end
outf = fullfile('results', sprintf('mc_straight_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'rmse', 'mtl_R1', 'mtl_R2', 'mtl_fus', 'brk_R1', 'brk_R2', 'brk_fus', ...
    'imp_ukf_R1', 'imp_ukf_R2', 'imp_fus_vs_R1', 'imp_fus_vs_R2', ...
    'nis_mean_R1', 'nis_mean_R2', 'assoc_R1', 'assoc_R2', ...
    'fus_best_method', 'bad_seed', 'bad_reason', 'seg_info', 'N_MC', 'SEED_BASE');
fprintf('\n完整数据已保存: %s\n', outf);
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
            if strcmp(mode, 'raw')
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    errs(end+1) = sphere_utils_haversine_distance(dp.raw_lon, dp.raw_lat, tl, tb) / 1000;
                end
            else
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
                end
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

function [assoc_rate, nis_mean, nis_gate, n_assoc, n_pred, init_frame] = diagnose_tracking(snaps, n_frames)
    n_assoc = 0; n_pred = 0; n_init = 0; n_lost = 0;
    init_frame = 0; nis_vals = [];
    for k = 1:n_frames
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 6
            n_init = n_init + 1;
        elseif trk.type == 1
            if init_frame == 0, init_frame = k; end
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
               isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;
            else
                n_pred = n_pred + 1;
            end
            if isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
            end
        elseif trk.type == 7
            n_lost = n_lost + 1;
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
            segs(end+1, :) = [seg_start, k-1, k - seg_start];  %#ok<AGROW>
        end
    end
    if in_seg
        segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1];  %#ok<AGROW>
    end
end

function segs = extract_fusion_segments(snaps, n_frames)
    segs = [];
    in_seg = false; seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat)
                is_tracking = true;
            end
        end
        if is_tracking && ~in_seg
            in_seg = true; seg_start = k;
        elseif ~is_tracking && in_seg
            in_seg = false;
            segs(end+1, :) = [seg_start, k-1, k - seg_start];  %#ok<AGROW>
        end
    end
    if in_seg
        segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1];  %#ok<AGROW>
    end
end

function mtl = compute_mtl(segs)
    if isempty(segs), mtl = 0; else, mtl = mean(segs(:,3)); end
end

function v = rms_val(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function print_mc_row(label, vals)
    v = vals(~isnan(vals) & ~isinf(vals));
    if isempty(v)
        fprintf('║  %-22s %7s %7s %7s %7s %7s  ║\n', label, 'NaN', 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('║  %-22s %7.1f %7.1f %7.1f %7.1f %7.1f  ║\n', ...
            label, mean(v), std(v), median(v), min(v), max(v));
    end
end
