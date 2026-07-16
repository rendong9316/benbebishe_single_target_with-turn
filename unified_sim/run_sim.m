% =========================================================================
% run_sim.m — 统一仿真主入口（单/多目标通用）
% =========================================================================
% 一套流程覆盖 straight / gradual_turn / uturn / multi。
% 单目标是多目标 n_targets=1 的自然退化：统一真值 cell、统一点迹生成、
% 统一 JPDA 跟踪、统一跨雷达匹配、统一融合评估。
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ==================== 场景配置 ====================
params = simulation_params();
params = apply_scenario_params(params);

fprintf('============================================================\n');
fprintf(' 统一仿真: scenario=%s, n_targets=%d, trajectory_mode=%s\n', ...
    params.scenario, params.n_targets, params.trajectory_mode);
fprintf('============================================================\n\n');

%% ==================== Phase 0: 场景初始化 ====================
fprintf('========== Phase 0: 场景初始化 ==========\n');

scenario = build_truth_scenario(params);
truth_all_cell = scenario.truth_all_cell;
truthTrajs = scenario.truthTrajs;
t1_grid = scenario.t1_grid;
t2_grid = scenario.t2_grid;
n_frames = scenario.n_frames;
true_track = truth_all_cell{1};
truthTraj = truthTrajs{1};

fprintf('目标数: %d, 仿真帧数: %d (dt=%.0fs)\n', scenario.n_targets, n_frames, params.dt_sec);
for ac = 1:scenario.n_targets
    tt_ac = truth_all_cell{ac};
    fprintf('  目标%s: %d 点, 总时长 %.0f s\n', ...
        truthTrajs{ac}.label, size(tt_ac, 1), tt_ac(end, 5));
end

n_in_r1 = 0; n_in_r2 = 0; n_total_truth = 0;
for ac = 1:scenario.n_targets
    tt_ac = truth_all_cell{ac};
    n_total_truth = n_total_truth + size(tt_ac, 1);
    for i = 1:size(tt_ac, 1)
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
            tt_ac(i,1), tt_ac(i,2), params.radar1_beam_center_deg, params);
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
            tt_ac(i,1), tt_ac(i,2), params.radar2_beam_center_deg, params);
        if in1, n_in_r1 = n_in_r1 + 1; end
        if in2, n_in_r2 = n_in_r2 + 1; end
    end
end
fprintf('  在R1威力内: %d 点, 在R2威力内: %d 点 (共%d点)\n', ...
    n_in_r1, n_in_r2, n_total_truth);

%% ==================== Phase 1: ADS-B 系统偏差标定 ====================
fprintf('\n========== Phase 1: ADS-B 系统偏差标定 ==========\n');
[dr1_est, da1_est, dr2_est, da2_est] = estimate_adsb_bias(params);
fprintf('R1: dr_est=%.1f m, da_est=%.4f deg\n', dr1_est, da1_est);
fprintf('R2: dr_est=%.1f m, da_est=%.4f deg\n', dr2_est, da2_est);

%% ==================== Phase 2+4: 点迹生成 + 偏差校正 ====================
fprintf('\n========== Phase 2+4: 点迹生成 + 偏差校正 ==========\n');

radar1_cfg = struct('radar_lon', params.radar1_lon, 'radar_lat', params.radar1_lat, ...
    'tx_lon', params.radar1_tx_lon, 'tx_lat', params.radar1_tx_lat, ...
    'range_bias_m', params.radar1_range_bias_m, ...
    'azimuth_bias_deg', params.radar1_azimuth_bias_deg, ...
    'beam_center_deg', params.radar1_beam_center_deg, ...
    'range_noise_std_m', params.radar1_range_noise_std_m, ...
    'azimuth_noise_std_deg', params.radar1_azimuth_noise_std_deg);
radar2_cfg = struct('radar_lon', params.radar2_lon, 'radar_lat', params.radar2_lat, ...
    'tx_lon', params.radar2_tx_lon, 'tx_lat', params.radar2_tx_lat, ...
    'range_bias_m', params.radar2_range_bias_m, ...
    'azimuth_bias_deg', params.radar2_azimuth_bias_deg, ...
    'beam_center_deg', params.radar2_beam_center_deg, ...
    'range_noise_std_m', params.radar2_range_noise_std_m, ...
    'azimuth_noise_std_deg', params.radar2_azimuth_noise_std_deg);

detList_R1 = generate_all_radar_detections(params, truth_all_cell, t1_grid(1:n_frames), ...
    radar1_cfg, dr1_est, da1_est, 1e7);
detList_R2 = generate_all_radar_detections(params, truth_all_cell, t2_grid(1:n_frames), ...
    radar2_cfg, dr2_est, da2_est, 2e7);
fprintf('点迹生成完成: R1=%d 帧, R2=%d 帧\n', n_frames, n_frames);

%% ==================== Phase 5: 多目标航迹跟踪 ====================
fprintf('\n========== Phase 5: 多目标航迹跟踪 ==========\n');

params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
params.gate_sigma = params.radar1_gate_sigma;
params.gate_vr_ms = params.radar1_gate_vr_ms;
params.tracker_K_loss = params.radar1_tracker_K_loss;
ukf1_tpl = create_ukf_template(params, params.radar1_lon, params.radar1_lat, ...
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
ukf2_tpl = create_ukf_template(params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

trackList_R1 = {}; tempPool_R1 = {}; next_id_R1 = 1;
trackSnapshots_R1 = cell(n_frames, 1);
for k = 1:n_frames
    [trackList_R1, tempPool_R1, trackSnapshots_R1{k}, next_id_R1] = ...
        multi_track_runner_kf(trackList_R1, tempPool_R1, detList_R1{k}, ukf1_tpl, ...
        params, k, next_id_R1, truth_all_cell, t1_grid);
end
fprintf('  R1 跟踪完成: %d 帧, 最终航迹数 = %d\n', n_frames, length(trackList_R1));

trackList_R2 = {}; tempPool_R2 = {}; next_id_R2 = 1;
trackSnapshots_R2 = cell(n_frames, 1);
for k = 1:n_frames
    [trackList_R2, tempPool_R2, trackSnapshots_R2{k}, next_id_R2] = ...
        multi_track_runner_kf(trackList_R2, tempPool_R2, detList_R2{k}, ukf2_tpl, ...
        params_r2, k, next_id_R2, truth_all_cell, t2_grid);
end
fprintf('  R2 跟踪完成: %d 帧, 最终航迹数 = %d\n', n_frames, length(trackList_R2));

%% ==================== Phase 6: 航迹级时间对齐 ====================
fprintf('\n========== Phase 6: 航迹级时间对齐 ==========\n');
aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
fprintf('R2 航迹时间对齐完成\n');

%% ==================== Phase 7: 航迹匹配 + 融合 ====================
fprintf('\n========== Phase 7: 航迹匹配 + 融合 ==========\n');
matched_pairs = track_matcher(trackSnapshots_R1, aligned_R2, params);
fprintf('匹配对数: %d\n', length(matched_pairs));

method_names = {'SCC', 'BC', 'CI', 'FCI'};
all_fused_snapshots = cell(length(method_names), 1);
for m = 1:length(method_names)
    all_fused_snapshots{m} = run_track_fusion(matched_pairs, ...
        trackSnapshots_R1, aligned_R2, params, method_names{m});
    fprintf('  %s 融合完成\n', method_names{m});
end

%% ==================== Phase 8: 误差评估 ====================
fprintf('\n========== Phase 8: 误差评估 ==========\n');

matcher_simple = struct();
matcher_simple.matched_pairs = matched_pairs;
matcher_simple.aligned_R2 = aligned_R2;
if isempty(matched_pairs)
    matcher_simple.r1_ids = [];
    matcher_simple.r2_ids = [];
else
    matcher_simple.r1_ids = [matched_pairs.R1_track_id];
    matcher_simple.r2_ids = [matched_pairs.R2_track_id];
end
matcher_simple.r1_pos = extract_track_positions_by_ids(trackSnapshots_R1, matcher_simple.r1_ids, n_frames);

fusion_eval = evaluate_all('fusion', all_fused_snapshots, method_names, ...
    matched_pairs, trackSnapshots_R1, trackSnapshots_R2, ...
    truthTrajs, n_frames, params.dt_sec, matcher_simple);

fprintf('\n--- 融合误差对比 (RMSE km) ---\n');
fprintf('%-8s %8s %8s\n', '算法', 'RMSE', '中位');
fprintf('%-8s %8s %8s\n', '------', '------', '------');
all_method_labels = [method_names, {'R1_only', 'R2_only'}];
for m = 1:length(all_method_labels)
    s = fusion_eval.overall(m).s;
    fprintf('%-8s %8.1f %8.1f\n', all_method_labels{m}, s.rms, s.median);
end

best_fusion_rmse = inf; best_m = 1;
for m = 1:length(method_names)
    rms_m = fusion_eval.overall(m).s.rms;
    if rms_m < best_fusion_rmse
        best_fusion_rmse = rms_m;
        best_m = m;
    end
end
fprintf('\n最佳融合算法: %s (RMSE=%.1fkm)\n', method_names{best_m}, best_fusion_rmse);

errorStats_R1 = evaluate_all('tracking_errors', trackSnapshots_R1, detList_R1, ...
    truthTrajs, n_frames, params.dt_sec, 'R1', t1_grid(1:n_frames));
errorStats_R2 = evaluate_all('tracking_errors', trackSnapshots_R2, detList_R2, ...
    truthTrajs, n_frames, params.dt_sec, 'R2', t2_grid(1:n_frames));

%% ==================== Phase 9: 可视化 + 数据保存 ====================
fprintf('\n========== Phase 9: 可视化 + 数据保存 ==========\n');
if ~exist('results', 'dir'), mkdir('results'); end

plot_scene_overview(true_track, params, 'results');
plot_point_cloud_3d(detList_R1, 'R1', 'results/fig2a_R1_point_cloud.png');
plot_point_cloud_3d(detList_R2, 'R2', 'results/fig2b_R2_point_cloud.png');

if params.n_targets == 1
    plot_results('single_track', true_track, detList_R1, detList_R2, ...
        trackSnapshots_R1, trackSnapshots_R2, params, 'results');
    plot_results('single_fusion', true_track, trackSnapshots_R1, trackSnapshots_R2, ...
        all_fused_snapshots, method_names, best_m, fusion_eval, truthTraj, params, 'results');
else
    plot_results_multi('single_track', truth_all_cell{1}, truth_all_cell{2}, truth_all_cell{3}, ...
        detList_R1, detList_R2, trackSnapshots_R1, trackSnapshots_R2, params, 'results');
    plot_results_multi('single_fusion', truth_all_cell{1}, truth_all_cell{2}, truth_all_cell{3}, ...
        trackSnapshots_R1, trackSnapshots_R2, all_fused_snapshots, ...
        method_names, matched_pairs, fusion_eval, truthTrajs, params, 'results');
end

outf = fullfile('results', sprintf('unified_sim_%s_%s.mat', ...
    datestr(now, 'yyyymmdd_HHMMSS'), params.scenario));
save(outf, 'params', 'truthTrajs', 'truth_all_cell', 'detList_R1', 'detList_R2', ...
    'trackSnapshots_R1', 'trackSnapshots_R2', 'aligned_R2', ...
    'matched_pairs', 'all_fused_snapshots', 'method_names', ...
    'fusion_eval', 'errorStats_R1', 'errorStats_R2', ...
    'best_m', 'best_fusion_rmse');
fprintf('数据已保存: %s\n', outf);
fprintf('\nDone.\n');


function pos = extract_track_positions_by_ids(trackSnapshots, track_ids, n_frames)
    pos = nan(length(track_ids), n_frames, 2);
    for p = 1:length(track_ids)
        track_id = track_ids(p);
        for k = 1:n_frames
            snap = trackSnapshots{k};
            if isempty(snap) || ~isfield(snap, 'trackList') || isempty(snap.trackList)
                continue;
            end
            for t = 1:length(snap.trackList)
                trk = snap.trackList{t};
                if isfield(trk, 'id') && trk.id == track_id && ~isnan(trk.lat)
                    pos(p, k, 1) = trk.lon;
                    pos(p, k, 2) = trk.lat;
                    break;
                end
            end
        end
    end
end


function ukf_tpl = create_ukf_template(params, radar_lon, radar_lat, tx_lon, tx_lat, dt_sec)
    backend = 'zishiying';
    if isfield(params, 'ukf_backend')
        backend = lower(params.ukf_backend);
    end

    if contains(backend, 'imm')
        ukf_tpl = ukf_imm('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt_sec);
        fprintf('  使用 IMM CV+CT 后端\n');
    else
        ukf_tpl = ukf_zishiying('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt_sec);
        fprintf('  使用自适应 UKF 后端\n');
    end
end
