% =========================================================================
% tune_ukf_params.m — UKF参数自动调优脚本 (v2: 扩大范围、加密步进)
% =========================================================================
% 改进：
%   Q_scale: 1e3 ~ 1e6 (1000倍范围, 15个对数步进)
%   gate_sigma: 1.5 ~ 6.0 (4倍范围, 步长0.5)
%   alpha: 1e-4 ~ 0.1 (1000倍范围)
%   P_pos_std: 0.02 ~ 0.50
%   自适应参数也扩大范围
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

N_SEEDS = 10;
BASE_SEED = 42;

fprintf('========================================\n');
fprintf('  UKF 参数自动调优 v2 (扩大范围全局搜索)\n');
fprintf('  随机种子数: %d\n', N_SEEDS);
fprintf('========================================\n');

% =========================================================================
% Phase 0-4: 预生成各种子的检测数据
% =========================================================================
fprintf('\n>>> 预生成检测数据 (%d 个种子)...\n', N_SEEDS);

all_caches = cell(N_SEEDS, 1);

for seed_idx = 1:N_SEEDS
    seed = BASE_SEED + seed_idx - 1;
    fprintf('  种子 %d/%d (seed=%d)...', seed_idx, N_SEEDS, seed);
    tic;

    params = simulation_params();
    rng(seed);

    [traj, ~] = aircraft_trajectory_create('turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    turn_info = struct();
    if traj.n_segments >= 2
        turn_info.turn_start_time = traj.segments{1}.dur;
        turn_info.turn_end_time = traj.segments{1}.dur + 300;
    else
        turn_info.turn_start_time = inf;
        turn_info.turn_end_time = inf;
    end

    % Phase 1: ADS-B 偏差标定
    rng(seed);
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

    % Phase 2: 原始点迹生成
    detRaw_R1 = cell(n_frames, 1);
    detRaw_R2 = cell(n_frames, 1);

    for k = 1:n_frames
        rng(seed + k);
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

        rng(seed + 10000 + k);
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

    % Phase 4: 偏差校正 + 几何反解
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
        end
        detList_R2{k} = dets_r2;
    end

    cache = struct();
    cache.traj = traj;
    cache.true_track = true_track;
    cache.t1_grid = t1_grid;
    cache.t2_grid = t2_grid;
    cache.n_frames = n_frames;
    cache.detList_R1 = detList_R1;
    cache.detList_R2 = detList_R2;
    cache.turn_info = turn_info;
    all_caches{seed_idx} = cache;

    elapsed = toc;
    fprintf(' 完成 (%.1fs, %d帧)\n', elapsed, n_frames);
end

fprintf('检测数据预生成完成。\n');

% =========================================================================
% Round 1: Q_scale × gate_sigma 全局粗搜索（扩大范围、加密步进）
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Round 1: Q_scale × gate_sigma 全局搜索\n');
fprintf('  Q: 1e3 ~ 1e6 (1000倍范围, 15步)\n');
fprintf('  gate: 1.5 ~ 6.0 (步长0.5, 10步)\n');
fprintf('========================================\n');

% Q_scale: 对数步进，覆盖 1e3 到 1e6
Q_vals_R1 = [1e3, 2e3, 3e3, 5e3, 7e3, 1e4, 2e4, 3e4, 5e4, 7e4, 1e5, 2e5, 3e5, 5e5, 1e6];
gate_vals = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0];

results_r1 = cell(length(Q_vals_R1), length(gate_vals));
best_score_r1 = inf;
best_params_r1 = struct();
all_r1_results = {};

combo_count = length(Q_vals_R1) * length(gate_vals);
combo_idx = 0;

for qi = 1:length(Q_vals_R1)
    for gi = 1:length(gate_vals)
        combo_idx = combo_idx + 1;

        override = struct();
        override.ukf_Q_scale = Q_vals_R1(qi);
        override.ukf_Q_scale_R2 = Q_vals_R1(qi) * 2;
        override.gate_sigma = gate_vals(gi);
        override.tracker_K_loss = 20;

        fprintf('[%3d/%3d] Q=%7.0e gate=%.1f ...', combo_idx, combo_count, Q_vals_R1(qi), gate_vals(gi));
        tic;
        r = eval_params(override, all_caches, N_SEEDS);
        elapsed = toc;

        results_r1{qi, gi} = r;
        all_r1_results{end+1} = struct(...
            'Q', Q_vals_R1(qi), 'gate', gate_vals(gi), ...
            'score', r.score, 'rmse', r.rmse_ad_avg, ...
            'rmse_turn', r.rmse_ad_turn_avg, 'smooth', r.smooth_ad_avg);

        fprintf(' score=%.2f RMSE=%.1f turn=%.1f (%.1fs)\n', ...
            r.score, r.rmse_ad_avg, r.rmse_ad_turn_avg, elapsed);

        if r.score < best_score_r1
            best_score_r1 = r.score;
            best_params_r1.Q_scale_R1 = Q_vals_R1(qi);
            best_params_r1.Q_scale_R2 = Q_vals_R1(qi) * 2;
            best_params_r1.gate_sigma = gate_vals(gi);
            best_params_r1.score = r.score;
            best_params_r1.rmse = r.rmse_ad_avg;
            best_params_r1.rmse_turn = r.rmse_ad_turn_avg;
        end
    end
end

fprintf('\n--- Round 1 最佳 ---\n');
fprintf('Q_scale: R1=%.0e R2=%.0e  gate=%.1f  score=%.2f  RMSE=%.1f km\n', ...
    best_params_r1.Q_scale_R1, best_params_r1.Q_scale_R2, ...
    best_params_r1.gate_sigma, best_params_r1.score, best_params_r1.rmse);

% 打印 Round 1 前 5
scores = cellfun(@(x) x.score, all_r1_results);
[~, idx] = sort(scores);
fprintf('\nRound 1 前5名:\n');
for i = 1:min(5, length(idx))
    r = all_r1_results{idx(i)};
    fprintf('  #%d: Q=%7.0e gate=%.1f score=%.2f RMSE=%.1f\n', ...
        i, r.Q, r.gate, r.score, r.rmse);
end

% =========================================================================
% Round 2: Q_scale 细网格 + gate_sigma 细网格（在最优附近）
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Round 2: 最优附近加密搜索\n');
fprintf('========================================\n');

best_Q = best_params_r1.Q_scale_R1;
best_gate = best_params_r1.gate_sigma;

% Q: 在最优值 ±50% 范围内取 7 个点
Q_fine = unique([best_Q*0.5, best_Q*0.65, best_Q*0.8, best_Q, best_Q*1.2, best_Q*1.4, best_Q*1.6]);
% 同时检查 R2/R1 比例是否需要微调
Q_ratio_vals = [1.5, 2.0, 2.5];

% gate: 在最优值 ±1.0 范围内，步长 0.2
gate_fine = best_gate-1.0:0.2:best_gate+1.0;
gate_fine = gate_fine(gate_fine >= 1.0);  % 截断

results_r2 = cell(length(Q_fine), length(gate_fine));
best_score_r2 = inf;
best_params_r2 = struct();

combo_count = length(Q_fine) * length(gate_fine) + length(Q_ratio_vals);
combo_idx = 0;

% 先测试 Q 和 gate 的精细组合
for qi = 1:length(Q_fine)
    for gi = 1:length(gate_fine)
        combo_idx = combo_idx + 1;

        override = struct();
        override.ukf_Q_scale = Q_fine(qi);
        override.ukf_Q_scale_R2 = Q_fine(qi) * 2;
        override.gate_sigma = gate_fine(gi);
        override.tracker_K_loss = 20;

        fprintf('[%2d/%2d] Q=%.0e gate=%.1f ...', combo_idx, combo_count, Q_fine(qi), gate_fine(gi));
        tic;
        r = eval_params(override, all_caches, N_SEEDS);
        elapsed = toc;

        results_r2{qi, gi} = r;
        fprintf(' score=%.2f RMSE=%.1f (%.1fs)\n', r.score, r.rmse_ad_avg, elapsed);

        if r.score < best_score_r2
            best_score_r2 = r.score;
            best_params_r2.Q_scale_R1 = Q_fine(qi);
            best_params_r2.Q_scale_R2 = Q_fine(qi) * 2;
            best_params_r2.gate_sigma = gate_fine(gi);
            best_params_r2.score = r.score;
            best_params_r2.rmse = r.rmse_ad_avg;
            best_params_r2.rmse_turn = r.rmse_ad_turn_avg;
        end
    end
end

% 再测试 R2/R1 Q比例
best_Q_r2 = best_params_r2.Q_scale_R1;
best_gate_r2 = best_params_r2.gate_sigma;
for ri = 1:length(Q_ratio_vals)
    combo_idx = combo_idx + 1;
    ratio = Q_ratio_vals(ri);

    override = struct();
    override.ukf_Q_scale = best_Q_r2;
    override.ukf_Q_scale_R2 = best_Q_r2 * ratio;
    override.gate_sigma = best_gate_r2;
    override.tracker_K_loss = 20;

    fprintf('[%2d/%2d] Q_ratio=%.1f (R1=%.0e R2=%.0e) ...', ...
        combo_idx, combo_count, ratio, best_Q_r2, best_Q_r2*ratio);
    tic;
    r = eval_params(override, all_caches, N_SEEDS);
    elapsed = toc;
    fprintf(' score=%.2f RMSE=%.1f (%.1fs)\n', r.score, r.rmse_ad_avg, elapsed);

    if r.score < best_score_r2
        best_score_r2 = r.score;
        best_params_r2.Q_scale_R1 = best_Q_r2;
        best_params_r2.Q_scale_R2 = best_Q_r2 * ratio;
        best_params_r2.gate_sigma = best_gate_r2;
        best_params_r2.score = r.score;
        best_params_r2.rmse = r.rmse_ad_avg;
        best_params_r2.rmse_turn = r.rmse_ad_turn_avg;
    end
end

fprintf('\n--- Round 2 最佳 ---\n');
fprintf('Q_scale: R1=%.0e R2=%.0e (ratio=%.1f)  gate=%.2f  score=%.2f  RMSE=%.1f km\n', ...
    best_params_r2.Q_scale_R1, best_params_r2.Q_scale_R2, ...
    best_params_r2.Q_scale_R2/best_params_r2.Q_scale_R1, ...
    best_params_r2.gate_sigma, best_params_r2.score, best_params_r2.rmse);

% =========================================================================
% Round 3: alpha × P_pos_std（扩大范围）
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Round 3: alpha × P_pos_std (扩大范围)\n');
fprintf('  alpha: 1e-4 ~ 1e-1 (1000倍)\n');
fprintf('  P_pos: 0.02 ~ 0.50\n');
fprintf('========================================\n');

alpha_vals = [1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2, 1e-1];
P_pos_vals = [0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50];

results_r3 = cell(length(alpha_vals), length(P_pos_vals));
best_score_r3 = inf;
best_params_r3 = struct();

combo_count = length(alpha_vals) * length(P_pos_vals);
combo_idx = 0;

for ai = 1:length(alpha_vals)
    for pi = 1:length(P_pos_vals)
        combo_idx = combo_idx + 1;

        override = struct();
        override.ukf_Q_scale = best_params_r2.Q_scale_R1;
        override.ukf_Q_scale_R2 = best_params_r2.Q_scale_R2;
        override.gate_sigma = best_params_r2.gate_sigma;
        override.ukf_alpha = alpha_vals(ai);
        override.ukf_P_pos_std = P_pos_vals(pi);
        override.tracker_K_loss = 20;

        fprintf('[%2d/%2d] alpha=%.0e P_pos=%.2f ...', combo_idx, combo_count, alpha_vals(ai), P_pos_vals(pi));
        tic;
        r = eval_params(override, all_caches, N_SEEDS);
        elapsed = toc;

        results_r3{ai, pi} = r;

        fprintf(' score=%.2f RMSE=%.1f (%.1fs)\n', r.score, r.rmse_ad_avg, elapsed);

        if r.score < best_score_r3
            best_score_r3 = r.score;
            best_params_r3.ukf_alpha = alpha_vals(ai);
            best_params_r3.ukf_P_pos_std = P_pos_vals(pi);
            best_params_r3.score = r.score;
            best_params_r3.rmse = r.rmse_ad_avg;
        end
    end
end

fprintf('\n--- Round 3 最佳 ---\n');
fprintf('alpha=%.0e P_pos=%.2f  score=%.2f  RMSE=%.1f km\n', ...
    best_params_r3.ukf_alpha, best_params_r3.ukf_P_pos_std, ...
    best_params_r3.score, best_params_r3.rmse);

% =========================================================================
% Round 4: 自适应参数（扩大范围）
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Round 4: 自适应参数 (扩大范围)\n');
fprintf('  fuzzy_window: 3 ~ 16\n');
fprintf('  ema_eta: 0.05 ~ 0.50\n');
fprintf('========================================\n');

window_vals = [3, 5, 8, 12, 16];
eta_vals = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50];

results_r4 = cell(length(window_vals), length(eta_vals));
best_score_r4 = inf;
best_params_r4 = struct();

combo_count = length(window_vals) * length(eta_vals);
combo_idx = 0;

for wi = 1:length(window_vals)
    for ei = 1:length(eta_vals)
        combo_idx = combo_idx + 1;

        override = struct();
        override.ukf_Q_scale = best_params_r2.Q_scale_R1;
        override.ukf_Q_scale_R2 = best_params_r2.Q_scale_R2;
        override.gate_sigma = best_params_r2.gate_sigma;
        override.ukf_alpha = best_params_r3.ukf_alpha;
        override.ukf_P_pos_std = best_params_r3.ukf_P_pos_std;
        override.fuzzy_window_size = window_vals(wi);
        override.fuzzy_ema_eta = eta_vals(ei);
        override.maneuver_ema_eta = eta_vals(ei);
        override.tracker_K_loss = 20;

        fprintf('[%2d/%2d] win=%d eta=%.2f ...', combo_idx, combo_count, window_vals(wi), eta_vals(ei));
        tic;
        r = eval_params(override, all_caches, N_SEEDS);
        elapsed = toc;

        results_r4{wi, ei} = r;

        fprintf(' score=%.2f RMSE=%.1f (%.1fs)\n', r.score, r.rmse_ad_avg, elapsed);

        if r.score < best_score_r4
            best_score_r4 = r.score;
            best_params_r4.fuzzy_window_size = window_vals(wi);
            best_params_r4.eta = eta_vals(ei);
            best_params_r4.score = r.score;
            best_params_r4.rmse = r.rmse_ad_avg;
        end
    end
end

fprintf('\n--- Round 4 最佳 ---\n');
fprintf('fuzzy_window=%d eta=%.2f  score=%.2f  RMSE=%.1f km\n', ...
    best_params_r4.fuzzy_window_size, best_params_r4.eta, ...
    best_params_r4.score, best_params_r4.rmse);

% =========================================================================
% 汇总最佳参数
% =========================================================================
fprintf('\n========================================\n');
fprintf('  最终最佳参数汇总\n');
fprintf('========================================\n');

best = struct();
best.ukf_Q_scale_R1 = best_params_r2.Q_scale_R1;
best.ukf_Q_scale_R2 = best_params_r2.Q_scale_R2;
best.gate_sigma = best_params_r2.gate_sigma;
best.ukf_alpha = best_params_r3.ukf_alpha;
best.ukf_P_pos_std = best_params_r3.ukf_P_pos_std;
best.fuzzy_window_size = best_params_r4.fuzzy_window_size;
best.maneuver_ema_eta = best_params_r4.eta;
best.fuzzy_ema_eta = best_params_r4.eta;

fprintf('%-30s %-15s %s\n', '参数', '最优值', '说明');
fprintf('%-30s %-15s %s\n', '------------------------------', '---------------', '----------------------');
fprintf('%-30s %-15.0e %s\n', 'ukf_Q_scale (R1)', best.ukf_Q_scale_R1, '过程噪声缩放因子(R1)');
fprintf('%-30s %-15.0e %s\n', 'ukf_Q_scale (R2)', best.ukf_Q_scale_R2, '过程噪声缩放因子(R2)');
fprintf('%-30s %-15.2f %s\n', 'gate_sigma', best.gate_sigma, '关联门限标准差倍数');
fprintf('%-30s %-15.0e %s\n', 'ukf_alpha', best.ukf_alpha, 'UT散布度参数');
fprintf('%-30s %-15.2f %s\n', 'ukf_P_pos_std', best.ukf_P_pos_std, '初始位置标准差(°)');
fprintf('%-30s %-15d %s\n', 'fuzzy_window_size', best.fuzzy_window_size, 'NIS滑动窗口大小');
fprintf('%-30s %-15.2f %s\n', 'maneuver_ema_eta', best.maneuver_ema_eta, 'Q因子EMA平滑系数');

% 最佳参数详细评估
fprintf('\n>>> 最佳参数详细评估 (10种子平均)...\n');
best_override = struct();
best_override.ukf_Q_scale = best.ukf_Q_scale_R1;
best_override.ukf_Q_scale_R2 = best.ukf_Q_scale_R2;
best_override.gate_sigma = best.gate_sigma;
best_override.ukf_alpha = best.ukf_alpha;
best_override.ukf_P_pos_std = best.ukf_P_pos_std;
best_override.fuzzy_window_size = best.fuzzy_window_size;
best_override.fuzzy_ema_eta = best.fuzzy_ema_eta;
best_override.maneuver_ema_eta = best.maneuver_ema_eta;
best_override.tracker_K_loss = 20;

best_result = eval_params(best_override, all_caches, N_SEEDS);

fprintf('\n最佳参数性能指标:\n');
fprintf('  自适应UKF R1 RMSE:       %.1f km\n', best_result.rmse_ad_r1);
fprintf('  自适应UKF R2 RMSE:       %.1f km\n', best_result.rmse_ad_r2);
fprintf('  自适应UKF 平均 RMSE:     %.1f km\n', best_result.rmse_ad_avg);
fprintf('  拐弯区域R1 RMSE:          %.1f km\n', best_result.rmse_ad_turn_r1);
fprintf('  拐弯区域R2 RMSE:          %.1f km\n', best_result.rmse_ad_turn_r2);
fprintf('  拐弯区域平均 RMSE:        %.1f km\n', best_result.rmse_ad_turn_avg);
fprintf('  R1 航迹生命期:            %.0f 帧\n', best_result.life_ad_r1);
fprintf('  R2 航迹生命期:            %.0f 帧\n', best_result.life_ad_r2);
fprintf('  R1 平滑度:                %.2f km std\n', best_result.smooth_ad_r1);
fprintf('  R2 平滑度:                %.2f km std\n', best_result.smooth_ad_r2);
fprintf('  综合得分:                 %.2f\n', best_result.score);

% =========================================================================
% 全局排名：前 20 名
% =========================================================================
fprintf('\n========================================\n');
fprintf('  全局排名：前20名参数组合\n');
fprintf('========================================\n');

all_results = {};

% Round 1
for i = 1:length(all_r1_results)
    r = all_r1_results{i};
    all_results{end+1} = struct('desc', ...
        sprintf('Q=%7.0e gate=%.1f (R1)', r.Q, r.gate), ...
        'score', r.score, 'rmse', r.rmse, 'rmse_turn', r.rmse_turn, 'smooth', r.smooth);
end

% Round 2
for qi = 1:length(Q_fine)
    for gi = 1:length(gate_fine)
        if ~isempty(results_r2{qi, gi})
            r = results_r2{qi, gi};
            all_results{end+1} = struct('desc', ...
                sprintf('Q=%.0e gate=%.1f (R2)', Q_fine(qi), gate_fine(gi)), ...
                'score', r.score, 'rmse', r.rmse_ad_avg, 'rmse_turn', r.rmse_ad_turn_avg, 'smooth', r.smooth_ad_avg);
        end
    end
end

% Round 3
for ai = 1:length(alpha_vals)
    for pi = 1:length(P_pos_vals)
        if ~isempty(results_r3{ai, pi})
            r = results_r3{ai, pi};
            all_results{end+1} = struct('desc', ...
                sprintf('alpha=%.0e P=%.2f (R3)', alpha_vals(ai), P_pos_vals(pi)), ...
                'score', r.score, 'rmse', r.rmse_ad_avg, 'rmse_turn', r.rmse_ad_turn_avg, 'smooth', r.smooth_ad_avg);
        end
    end
end

% Round 4
for wi = 1:length(window_vals)
    for ei = 1:length(eta_vals)
        if ~isempty(results_r4{wi, ei})
            r = results_r4{wi, ei};
            all_results{end+1} = struct('desc', ...
                sprintf('win=%d eta=%.2f (R4)', window_vals(wi), eta_vals(ei)), ...
                'score', r.score, 'rmse', r.rmse_ad_avg, 'rmse_turn', r.rmse_ad_turn_avg, 'smooth', r.smooth_ad_avg);
        end
    end
end

scores_arr = cellfun(@(x) x.score, all_results);
[~, idx] = sort(scores_arr);
all_results = all_results(idx);

fprintf('%-5s %-8s %-8s %-8s %-8s %s\n', '排名', '得分', 'RMSE', '拐弯', '平滑', '参数说明');
fprintf('%-5s %-8s %-8s %-8s %-8s %s\n', '----', '------', '------', '------', '------', '--------');
for i = 1:min(20, length(all_results))
    r = all_results{i};
    fprintf('%-5d %-8.2f %-8.1f %-8.1f %-8.2f %s\n', ...
        i, r.score, r.rmse, r.rmse_turn, r.smooth, r.desc);
end

% =========================================================================
% 参数对比：最优 vs 原始
% =========================================================================
fprintf('\n========================================\n');
fprintf('  原始参数 vs 最优参数 性能对比\n');
fprintf('========================================\n');

% 用原始参数评估
orig_override = struct();
orig_override.ukf_Q_scale = 5e4;
orig_override.ukf_Q_scale_R2 = 1e5;
orig_override.gate_sigma = 2.5;
orig_override.ukf_alpha = 1e-3;
orig_override.ukf_P_pos_std = 0.2;
orig_override.fuzzy_window_size = 8;
orig_override.tracker_K_loss = 20;

orig_result = eval_params(orig_override, all_caches, N_SEEDS);

fprintf('%-25s %10s %10s %10s\n', '指标', '原始', '最优', '改善');
fprintf('%-25s %10s %10s %10s\n', '-------------------------', '----------', '----------', '----------');
fprintf('%-25s %10.1f %10.1f %+9.1f%%\n', 'R1 RMSE (km)', ...
    orig_result.rmse_ad_r1, best_result.rmse_ad_r1, ...
    (1-best_result.rmse_ad_r1/orig_result.rmse_ad_r1)*100);
fprintf('%-25s %10.1f %10.1f %+9.1f%%\n', 'R2 RMSE (km)', ...
    orig_result.rmse_ad_r2, best_result.rmse_ad_r2, ...
    (1-best_result.rmse_ad_r2/orig_result.rmse_ad_r2)*100);
fprintf('%-25s %10.1f %10.1f %+9.1f%%\n', '平均 RMSE (km)', ...
    orig_result.rmse_ad_avg, best_result.rmse_ad_avg, ...
    (1-best_result.rmse_ad_avg/orig_result.rmse_ad_avg)*100);
fprintf('%-25s %10.1f %10.1f %+9.1f%%\n', '拐弯 RMSE (km)', ...
    orig_result.rmse_ad_turn_avg, best_result.rmse_ad_turn_avg, ...
    (1-best_result.rmse_ad_turn_avg/orig_result.rmse_ad_turn_avg)*100);
fprintf('%-25s %10.1f %10.1f %+9.1f%%\n', 'R1 生命期 (帧)', ...
    orig_result.life_ad_r1, best_result.life_ad_r1, ...
    (best_result.life_ad_r1/orig_result.life_ad_r1-1)*100);
fprintf('%-25s %10.1f %10.1f %+9.1f%%\n', 'R2 生命期 (帧)', ...
    orig_result.life_ad_r2, best_result.life_ad_r2, ...
    (best_result.life_ad_r2/orig_result.life_ad_r2-1)*100);
fprintf('%-25s %10.2f %10.2f %+9.1f%%\n', '平滑度 (km std)', ...
    orig_result.smooth_ad_avg, best_result.smooth_ad_avg, ...
    (1-best_result.smooth_ad_avg/orig_result.smooth_ad_avg)*100);

% =========================================================================
% 可直接使用的参数配置
% =========================================================================
fprintf('\n========================================\n');
fprintf('  可直接使用的参数配置\n');
fprintf('========================================\n');

fprintf('\n%% === R1 UKF参数（v2调优后） ===\n');
fprintf('params.ukf_Q_scale = %.0e;\n', best.ukf_Q_scale_R1);
fprintf('params.gate_sigma = %.2f;\n', best.gate_sigma);
fprintf('params.ukf_alpha = %.0e;\n', best.ukf_alpha);
fprintf('params.ukf_P_pos_std = %.2f;\n', best.ukf_P_pos_std);
fprintf('params.fuzzy_window_size = %d;\n', best.fuzzy_window_size);
fprintf('params.fuzzy_ema_eta = %.2f;\n', best.fuzzy_ema_eta);
fprintf('params.maneuver_ema_eta = %.2f;\n', best.maneuver_ema_eta);
fprintf('params.tracker_K_loss = 20;\n');

fprintf('\n%% === R2 UKF参数（v2调优后） ===\n');
fprintf('params_r2.ukf_Q_scale = %.0e;\n', best.ukf_Q_scale_R2);
fprintf('params_r2.gate_sigma = %.2f;\n', best.gate_sigma);
fprintf('params_r2.ukf_alpha = %.0e;\n', best.ukf_alpha);
fprintf('params_r2.ukf_P_pos_std = %.2f;\n', best.ukf_P_pos_std);
fprintf('params_r2.fuzzy_window_size = %d;\n', best.fuzzy_window_size);
fprintf('params_r2.fuzzy_ema_eta = %.2f;\n', best.fuzzy_ema_eta);
fprintf('params_r2.maneuver_ema_eta = %.2f;\n', best.maneuver_ema_eta);
fprintf('params_r2.tracker_K_loss = 12;\n');

fprintf('\nDone.\n');

% =========================================================================
% 局部函数
% =========================================================================

function result = eval_params(params_override, caches, N_SEEDS)
    rmse_basic_r1 = zeros(N_SEEDS, 1);
    rmse_basic_r2 = zeros(N_SEEDS, 1);
    rmse_ad_r1 = zeros(N_SEEDS, 1);
    rmse_ad_r2 = zeros(N_SEEDS, 1);
    rmse_ad_turn_r1 = zeros(N_SEEDS, 1);
    rmse_ad_turn_r2 = zeros(N_SEEDS, 1);
    life_ad_r1 = zeros(N_SEEDS, 1);
    life_ad_r2 = zeros(N_SEEDS, 1);
    smooth_ad_r1 = zeros(N_SEEDS, 1);
    smooth_ad_r2 = zeros(N_SEEDS, 1);

    for s = 1:N_SEEDS
        cache = caches{s};

        params = simulation_params();
        fn = fieldnames(params_override);
        for i = 1:length(fn)
            params.(fn{i}) = params_override.(fn{i});
        end

        params.ukf_range_std_m = params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

        params_r2 = params;
        params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        if isfield(params_override, 'ukf_Q_scale_R2')
            params_r2.ukf_Q_scale = params_override.ukf_Q_scale_R2;
        else
            params_r2.ukf_Q_scale = params.ukf_Q_scale * 2;
        end
        if isfield(params_override, 'ukf_P_pos_std_R2')
            params_r2.ukf_P_pos_std = params_override.ukf_P_pos_std_R2;
        end
        if isfield(params_override, 'ukf_P_vel_std_R2')
            params_r2.ukf_P_vel_std = params_override.ukf_P_vel_std_R2;
        end
        ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

        [snap_R1, ~] = single_track_runner(cache.detList_R1, ukf1_tpl, params, cache.n_frames);
        [snap_R2, ~] = single_track_runner(cache.detList_R2, ukf2_tpl, params_r2, cache.n_frames);

        ukf1_tpl_ad = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        ukf2_tpl_ad = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

        [snap_R1_ad, ~] = single_track_runner_adaptive(cache.detList_R1, ukf1_tpl_ad, params, cache.n_frames);
        [snap_R2_ad, ~] = single_track_runner_adaptive(cache.detList_R2, ukf2_tpl_ad, params_r2, cache.n_frames);

        [rmse_basic_r1(s), ~, ~] = compute_rmse_local(snap_R1, cache.true_track, cache.t1_grid, cache.turn_info);
        [rmse_basic_r2(s), ~, ~] = compute_rmse_local(snap_R2, cache.true_track, cache.t2_grid, cache.turn_info);
        [rmse_ad_r1(s), life_ad_r1(s), smooth_ad_r1(s), rmse_ad_turn_r1(s)] = ...
            compute_rmse_local(snap_R1_ad, cache.true_track, cache.t1_grid, cache.turn_info);
        [rmse_ad_r2(s), life_ad_r2(s), smooth_ad_r2(s), rmse_ad_turn_r2(s)] = ...
            compute_rmse_local(snap_R2_ad, cache.true_track, cache.t2_grid, cache.turn_info);
    end

    result = struct();
    result.rmse_basic_r1 = mean(rmse_basic_r1);
    result.rmse_basic_r2 = mean(rmse_basic_r2);
    result.rmse_ad_r1 = mean(rmse_ad_r1);
    result.rmse_ad_r2 = mean(rmse_ad_r2);
    result.rmse_ad_avg = (mean(rmse_ad_r1) + mean(rmse_ad_r2)) / 2;
    result.rmse_ad_turn_r1 = mean(rmse_ad_turn_r1);
    result.rmse_ad_turn_r2 = mean(rmse_ad_turn_r2);
    result.rmse_ad_turn_avg = (mean(rmse_ad_turn_r1) + mean(rmse_ad_turn_r2)) / 2;
    result.life_ad_r1 = mean(life_ad_r1);
    result.life_ad_r2 = mean(life_ad_r2);
    result.smooth_ad_r1 = mean(smooth_ad_r1);
    result.smooth_ad_r2 = mean(smooth_ad_r2);
    result.smooth_ad_avg = (mean(smooth_ad_r1) + mean(smooth_ad_r2)) / 2;
    result.score = result.rmse_ad_avg * 0.5 + result.rmse_ad_turn_avg * 0.3 + result.smooth_ad_avg * 0.2;
end

function [rmse, life, smoothness, rmse_turn] = compute_rmse_local(snaps, true_track, t_grid, turn_info)
    n_frames = length(snaps);
    errors = [];
    turn_errors = [];
    step_dists = [];
    prev_lon = NaN;
    prev_lat = NaN;
    life = 0;

    for k = 1:n_frames
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type ~= 1, continue; end
        if isnan(trk.lon) || isnan(trk.lat), continue; end

        life = life + 1;

        t = t_grid(k);
        t_idx = find(true_track(:,5) <= t, 1, 'last');
        if isempty(t_idx), t_idx = 1; end
        if t_idx < size(true_track, 1)
            frac = (t - true_track(t_idx, 5)) / (true_track(t_idx+1, 5) - true_track(t_idx, 5));
            true_lon = true_track(t_idx, 1) + frac * (true_track(t_idx+1, 1) - true_track(t_idx, 1));
            true_lat = true_track(t_idx, 2) + frac * (true_track(t_idx+1, 2) - true_track(t_idx, 2));
        else
            true_lon = true_track(t_idx, 1);
            true_lat = true_track(t_idx, 2);
        end

        err_km = sphere_utils_haversine_distance(trk.lon, trk.lat, true_lon, true_lat) / 1000;
        errors(end+1) = err_km;

        if t >= turn_info.turn_start_time && t <= turn_info.turn_end_time
            turn_errors(end+1) = err_km;
        end

        if ~isnan(prev_lon)
            step_km = sphere_utils_haversine_distance(trk.lon, trk.lat, prev_lon, prev_lat) / 1000;
            step_dists(end+1) = step_km;
        end
        prev_lon = trk.lon;
        prev_lat = trk.lat;
    end

    if isempty(errors)
        rmse = 999;
        rmse_turn = 999;
        smoothness = 999;
    else
        rmse = sqrt(mean(errors .^ 2));
        if isempty(turn_errors)
            rmse_turn = rmse;
        else
            rmse_turn = sqrt(mean(turn_errors .^ 2));
        end
        if length(step_dists) < 2
            smoothness = 0;
        else
            smoothness = std(step_dists);
        end
    end
end
