% test_sweep_Pi_CV_to_CT.m — 控制变量：扫 IMM CV→CT 转移概率
% 不修改任何项目文件，参数在脚本内覆盖
clear; close all; clc; addpath(genpath('.'));

N_MC = 200;
SEED_BASE = 1;

% 测试的 Pi_CV_to_CT 值（保持 Pi_CT_to_CV = 0.10 不变）
Pi_values = [0.05, 0.10, 0.20, 0.30, 0.50];
n_pi = length(Pi_values);

% 存储结果
results = struct();
for i = 1:n_pi
    results(i).pi_cv_ct = Pi_values(i);
    results(i).ukf_R1 = nan(N_MC, 1);
    results(i).ukf_R2 = nan(N_MC, 1);
    results(i).fus_best = nan(N_MC, 1);
    results(i).ct_turn_R1 = nan(N_MC, 1);
    results(i).ct_turn_R2 = nan(N_MC, 1);
    results(i).ct_dom_R1 = nan(N_MC, 1);
    results(i).ct_dom_R2 = nan(N_MC, 1);
    results(i).assoc_R1 = nan(N_MC, 1);
    results(i).assoc_R2 = nan(N_MC, 1);
    results(i).mtl_R1 = nan(N_MC, 1);
    results(i).mtl_R2 = nan(N_MC, 1);
end

fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║  Pi_CV→CT 扫描: [0.05, 0.10, 0.20, 0.30, 0.50]    ║\n');
fprintf('║  Pi_CT→CV = 0.10 固定, N_MC = %d                   ║\n', N_MC);
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

for pi_idx = 1:n_pi
    p_cv_ct = Pi_values(pi_idx);
    p_ct_cv = 0.10;  % 固定

    fprintf('─── Pi_CV→CT = %.2f (Pi_CT→CV = %.2f) ───\n', p_cv_ct, p_ct_cv);
    t_start = tic;

    for mc = 1:N_MC
        seed = SEED_BASE + mc - 1;

        % 创建参数并覆盖
        params = simulation_params();
        params.random_seed = seed;
        params.imm_Pi_CV_to_CT = p_cv_ct;
        params.imm_Pi_CT_to_CV = p_ct_cv;
        rng(params.random_seed);

        % 生成航迹
        [traj, waypoints] = aircraft_trajectory_create('uturn', params);
        true_track = aircraft_trajectory_interpolate('generate', traj);

        t1 = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
        t2 = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
        n_frames = min(length(t1), length(t2));

        % 确定转弯帧
        turn_rate_rad = +1.0 * pi / 180.0;
        approach_dur = 120e3 / params.aircraft_speed_ms;
        turn_dur = 180;
        tf_start = find(t1 >= approach_dur, 1);
        tf_end = find(t1 >= approach_dur + turn_dur, 1);

        % Phase 4: 校准 + 点迹生成
        [de1, dae1, de2, dae2] = calibrate_adsb(params);

        dR1 = cell(n_frames, 1);
        dR2 = cell(n_frames, 1);

        rng(seed + 1e7);
        for k = 1:n_frames
            [p, v] = aircraft_trajectory_interpolate(traj, t1(k));
            dR1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, p(1), p(2), v(1), v(2), ...
                k, t1(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            for d = 1:length(dR1{k})
                dR1{k}(d).aircraft_id = 1;
                Rgc = dR1{k}(d).prange - de1;  azc = dR1{k}(d).paz - dae1;
                dR1{k}(d).drange = Rgc;  dR1{k}(d).daz = azc;
                dR1{k}(d).range_meas = Rgc;  dR1{k}(d).azimuth_meas = azc;
                if ~(isfield(dR1{k}(d),'lat') && ~isnan(dR1{k}(d).lat))
                    [~,lat_e,lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                    dR1{k}(d).lat = lat_e;  dR1{k}(d).lon = lon_e;
                end
                [~,raw_lat,raw_lon] = bistatic_inverse_solver(dR1{k}(d).prange, dR1{k}(d).paz, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                dR1{k}(d).raw_lat = raw_lat;  dR1{k}(d).raw_lon = raw_lon;
            end
        end

        rng(seed + 2e7);
        for k = 1:n_frames
            [p, v] = aircraft_trajectory_interpolate(traj, t2(k));
            dR2{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, p(1), p(2), v(1), v(2), ...
                k, t2(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            for d = 1:length(dR2{k})
                dR2{k}(d).aircraft_id = 1;
                Rgc = dR2{k}(d).prange - de2;  azc = dR2{k}(d).paz - dae2;
                dR2{k}(d).drange = Rgc;  dR2{k}(d).daz = azc;
                dR2{k}(d).range_meas = Rgc;  dR2{k}(d).azimuth_meas = azc;
                if ~(isfield(dR2{k}(d),'lat') && ~isnan(dR2{k}(d).lat))
                    [~,lat_e,lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                    dR2{k}(d).lat = lat_e;  dR2{k}(d).lon = lon_e;
                end
                [~,raw_lat,raw_lon] = bistatic_inverse_solver(dR2{k}(d).prange, dR2{k}(d).paz, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                dR2{k}(d).raw_lat = raw_lat;  dR2{k}(d).raw_lon = raw_lon;
            end
        end

        % Phase 5: IMM 跟踪
        % R1 UKF
        params.ukf_range_std_m = params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale = params.radar1_ukf_Q_scale;
        params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
        params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
        params.gate_sigma = params.radar1_gate_sigma;
        params.gate_vr_ms = params.radar1_gate_vr_ms;
        params.tracker_K_loss = params.radar1_tracker_K_loss;

        ukf_cv_r1 = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        ukf_ct_r1 = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        ukf_ct_r1.model_type = 'CT';
        ukf_ct_r1.turn_rate_rad_per_sec = turn_rate_rad;

        [snaps_R1, ft1] = imm_tracker(dR1, ukf_cv_r1, ukf_ct_r1, params, n_frames, true_track, t1);

        % R2 UKF
        params_r2 = params;
        params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        params_r2.gate_sigma = params.radar2_gate_sigma;
        params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
        params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
        params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
        params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
        params_r2.tracker_M = 4;
        params_r2.tracker_N = 8;
        params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
        params_r2.imm_Pi_CV_to_CT = p_cv_ct;
        params_r2.imm_Pi_CT_to_CV = p_ct_cv;

        ukf_cv_r2 = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
        ukf_ct_r2 = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
        ukf_ct_r2.model_type = 'CT';
        ukf_ct_r2.turn_rate_rad_per_sec = turn_rate_rad;

        [snaps_R2, ft2] = imm_tracker(dR2, ukf_cv_r2, ukf_ct_r2, params_r2, n_frames, true_track, t2);

        % Phase 6: R2 时间对齐
        snaps_R2_aligned = time_align_tracks(snaps_R2, params_r2);

        % Phase 7: 融合
        matcher = struct('R1_track_id', 1, 'R2_track_id', 1, ...
            'match_count', 0, 'coexist_count', 0, 'match_ratio', 1, ...
            'mean_dist_km', 0, 'quality', 100);
        fusion_labels = {'SCC', 'BC', 'CI', 'FCI'};
        fusion_rmse = nan(1, 4);
        for m = 1:4
            af = run_track_fusion(matcher, snaps_R1, snaps_R2_aligned, params_r2, fusion_labels{m});
            fusion_rmse(m) = rmse_tracks(af, true_track, t1);
        end

        % RMSE
        results(pi_idx).ukf_R1(mc) = rmse_tracks(snaps_R1, true_track, t1);
        results(pi_idx).ukf_R2(mc) = rmse_tracks(snaps_R2_aligned, true_track, t1);
        results(pi_idx).fus_best(mc) = min(fusion_rmse);

        % CT 概率（转弯段）
        if ~isempty(ft1) && isfield(ft1, 'mu_history')
            mh = ft1.mu_history;
            if ~isempty(tf_start)
                tf_range = tf_start:min(tf_end, size(mh, 1));
                results(pi_idx).ct_turn_R1(mc) = mean(mh(tf_range, 2)) * 100;
            end
        end
        if ~isempty(ft2) && isfield(ft2, 'mu_history')
            mh = ft2.mu_history;
            if ~isempty(tf_start)
                tf_range = tf_start:min(tf_end, size(mh, 1));
                results(pi_idx).ct_turn_R2(mc) = mean(mh(tf_range, 2)) * 100;
            end
        end

        % CT 占优帧数
        if ~isempty(ft1) && isfield(ft1, 'model_dominant')
            results(pi_idx).ct_dom_R1(mc) = sum(ft1.model_dominant == 2);
        end
        if ~isempty(ft2) && isfield(ft2, 'model_dominant')
            results(pi_idx).ct_dom_R2(mc) = sum(ft2.model_dominant == 2);
        end

        % 关联率
        if ~isempty(ft1) && isfield(ft1, 'assoc_count')
            results(pi_idx).assoc_R1(mc) = ft1.assoc_count / n_frames * 100;
        end
        if ~isempty(ft2) && isfield(ft2, 'assoc_count')
            results(pi_idx).assoc_R2(mc) = ft2.assoc_count / n_frames * 100;
        end

        % MTL
        results(pi_idx).mtl_R1(mc) = compute_mtl(snaps_R1);
        results(pi_idx).mtl_R2(mc) = compute_mtl(snaps_R2_aligned);

        if mod(mc, 40) == 0
            fprintf('  [%s] %d/%d (%.0fs)\n', datestr(now,'HH:MM:SS'), mc, N_MC, toc(t_start));
        end
    end

    elapsed = toc(t_start);
    fprintf('  Done in %.0f s\n\n', elapsed);
end

% ===== 汇总 =====
fprintf('╔════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  Pi_CV→CT 扫描结果汇总 (N=%d)                                         ║\n', N_MC);
fprintf('╠════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║ %-8s %8s %8s %8s %8s %8s %8s ║\n', ...
    'Pi_CVCT', 'UKF_R1', 'UKF_R2', 'FusBest', 'CTturn%', 'CTdom', 'Assoc%');
fprintf('║ %-8s %8s %8s %8s %8s %8s %8s ║\n', ...
    '---', '---', '---', '---', '---', '---', '---');
for i = 1:n_pi
    v = results(i);
    fprintf('║ %-8.2f %7.1fkm %7.1fkm %7.1fkm %7.1f%% %6.1ffr %6.1f%% ║\n', ...
        v.pi_cv_ct, ...
        nanmedian(v.ukf_R1), nanmedian(v.ukf_R2), nanmedian(v.fus_best), ...
        nanmedian(v.ct_turn_R1), nanmedian(v.ct_dom_R1), nanmedian(v.assoc_R1));
end
fprintf('╚════════════════════════════════════════════════════════════════════════╝\n');

% 保存
save('results\sweep_Pi_CV_to_CT.mat', 'results', 'Pi_values', 'N_MC');
fprintf('Results saved: results\\sweep_Pi_CV_to_CT.mat\n');

% ===== 辅助函数 =====
function rmse = rmse_tracks(snaps, true_track, t_grid)
    errs = [];
    for k = 1:length(snaps)
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            tr = snaps{k}.trackList{1};
            if isfield(tr, 'type') && tr.type ~= 7 && ~isnan(tr.lat)
                tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                errs(end+1) = sphere_utils_haversine_distance(tr.lon, tr.lat, tl, tb) / 1000;
            end
        end
    end
    if isempty(errs), rmse = NaN; else, rmse = sqrt(mean(errs.^2)); end
end

function mtl = compute_mtl(snaps)
    segments = [];
    in_track = false;
    seg_start = NaN;
    for k = 1:length(snaps)
        has_track = ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList) && ...
            snaps{k}.trackList{1}.type ~= 7;
        if has_track && ~in_track
            seg_start = k;
            in_track = true;
        elseif ~has_track && in_track
            segments(end+1) = k - seg_start;
            in_track = false;
        end
    end
    if in_track, segments(end+1) = length(snaps) - seg_start + 1; end
    if isempty(segments), mtl = NaN; else, mtl = mean(segments); end
end

function [de1, dae1, de2, dae2] = calibrate_adsb(params)
    T = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    al = T.Var2; ao = T.Var3;
    d1 = []; da1 = []; d2 = []; da2 = [];
    nc = min(5000, height(T));
    cs = max(1, floor(height(T) / nc));
    for idx = 1:cs:height(T)
        tl = ao(idx); tb = al(idx);
        if isnan(tl) || isnan(tb), continue; end
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, tl, tb, ...
            params.radar1_beam_center_deg, params);
        if in1
            Rg = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat, tl, tb);
            az = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, tl, tb);
            Rm = Rg + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
            am = az + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
            d1(end+1) = Rm - Rg;
            daz = am - az;
            if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
            da1(end+1) = daz;
        end
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, tl, tb, ...
            params.radar2_beam_center_deg, params);
        if in2
            Rg = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat, tl, tb);
            az = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, tl, tb);
            Rm = Rg + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
            am = az + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
            d2(end+1) = Rm - Rg;
            daz = am - az;
            if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
            da2(end+1) = daz;
        end
    end
    de1 = mean(d1); dae1 = mean(da1);
    de2 = mean(d2); dae2 = mean(da2);
end
