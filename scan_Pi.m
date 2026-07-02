% =========================================================================
% scan_Pi.m — Pi (IMM 转移概率) 参数扫描主入口（内联 MC 逻辑）
% =========================================================================
% 功能：系统扫描 imm_Pi_CV_to_CT = imm_Pi_CT_to_CV，评估对拐弯场景跟踪性能的影响
% 用法：在 MATLAB 中运行 scan_Pi
% =========================================================================
addpath(genpath('.'));

%% ---- 配置 ----
Pi_values = [0.001, 0.005, 0.01, 0.03, 0.05, 0.10, 0.20, 0.30, 0.50];
n_pi = length(Pi_values);
N_MC = 100;
SEED_BASE = 1;
UKF_NAMES = {'jichu', 'zishiying', 'imm'};
N_UKF = 3;
FUSION_METHODS = {'SCC', 'BC', 'CI', 'FCI'};
N_FUS = length(FUSION_METHODS);

fprintf('============================================================\n');
fprintf(' Pi_CV_to_CT / Pi_CT_to_CV 扫描: %d 个值\n', n_pi);
fprintf(' MC 次数: %d\n', N_MC);
fprintf(' 场景: gradual_turn + 180deg_uturn\n');
fprintf('============================================================\n\n');

%% ---- 扫描主循环 ----
for pi_idx = 1:n_pi
    pi_val = Pi_values(pi_idx);

    % ---- 检查是否已完成 ----
    skip_file = fullfile('results', sprintf('scan_Pi_done_Pi%g.mat', pi_val));
    if exist(skip_file, 'file')
        fprintf('\n============================================================\n');
        fprintf('  [%d/%d] Pi = %g — 已存在结果，跳过\n', pi_idx, n_pi, pi_val);
        fprintf('============================================================\n');
        continue;
    end

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('  [%d/%d] Pi = %g\n', pi_idx, n_pi, pi_val);
    fprintf('============================================================\n');

    % ---- 修改 simulation_params.m ----
    params_file = 'config/simulation_params.m';
    params_bak = 'config/simulation_params.m.bak';
    if exist(params_bak, 'file')
        copyfile(params_bak, params_file, 'f');
    else
        copyfile(params_file, params_bak);
    end
    copyfile(params_file, params_bak);

    fid = fopen(params_file, 'r');
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = lines{1};
    for li = 1:length(lines)
        if contains(lines{li}, 'params.imm_Pi_CV_to_CT')
            lines{li} = sprintf('params.imm_Pi_CV_to_CT = %g;', pi_val);
        end
        if contains(lines{li}, 'params.imm_Pi_CT_to_CV')
            lines{li} = sprintf('params.imm_Pi_CT_to_CV = %g;', pi_val);
        end
    end
    fid = fopen(params_file, 'w');
    for li = 1:length(lines)
        fprintf(fid, '%s\n', lines{li});
    end
    fclose(fid);

    rehash toolboxcache;

    % ==== 场景 1: gradual_turn ====
    fprintf('\n  >>> 场景 1: gradual_turn <<<\n');
    run_mc_gradual_turn(N_MC, SEED_BASE, UKF_NAMES, N_UKF, FUSION_METHODS, N_FUS, pi_val);

    % ==== 场景 2: 180deg uturn ====
    fprintf('\n  >>> 场景 2: 180deg_uturn <<<\n');
    run_mc_180deg_uturn(N_MC, SEED_BASE, UKF_NAMES, N_UKF, FUSION_METHODS, N_FUS, pi_val);

    % ---- 保存本轮结果标记 ----
    result_file = fullfile('results', sprintf('scan_Pi_done_Pi%g.mat', pi_val));
    save(result_file, 'pi_val');
    fprintf('  本轮结果已保存: %s\n', result_file);

    % ---- 恢复 simulation_params.m ----
    copyfile(params_bak, params_file);
end

fprintf('\n============================================================\n');
fprintf(' 扫描完成！结果保存在 results/ 目录\n');
fprintf('============================================================\n');

% =========================================================================
% 内部函数
% =========================================================================

function run_mc_gradual_turn(N_MC, SEED_BASE, UKF_NAMES, N_UKF, FUSION_METHODS, N_FUS, pi_val)
    % 预分配
    for u = 1:N_UKF
        s(u).name = UKF_NAMES{u};
        s(u).rmse_ukf_R1 = nan(N_MC, 1);
        s(u).rmse_ukf_R2 = nan(N_MC, 1);
        s(u).rmse_ukf_R2_alg = nan(N_MC, 1);
        s(u).rmse_fus = nan(N_MC, N_FUS);
        s(u).rmse_fus_best = nan(N_MC, 1);
        s(u).fus_best_method = cell(N_MC, 1);
        s(u).assoc_R1 = nan(N_MC, 1);
        s(u).assoc_R2 = nan(N_MC, 1);
        s(u).nis_mean_R1 = nan(N_MC, 1);
        s(u).nis_mean_R2 = nan(N_MC, 1);
        s(u).nis_gate_R1 = nan(N_MC, 1);
        s(u).nis_gate_R2 = nan(N_MC, 1);
        s(u).init_fr_R1 = nan(N_MC, 1);
        s(u).init_fr_R2 = nan(N_MC, 1);
        s(u).mtl_R1 = nan(N_MC, 1);
        s(u).mtl_R2 = nan(N_MC, 1);
        s(u).mtl_fus = nan(N_MC, 1);
        s(u).brk_R1 = nan(N_MC, 1);
        s(u).brk_R2 = nan(N_MC, 1);
        s(u).brk_fus = nan(N_MC, 1);
        s(u).imp_ukf_R1 = nan(N_MC, 1);
        s(u).imp_ukf_R2 = nan(N_MC, 1);
        s(u).imp_fus_vs_R1 = nan(N_MC, 1);
        s(u).imp_fus_vs_R2 = nan(N_MC, 1);
        s(u).bad_seed = zeros(N_MC, 1);
        s(u).bad_reason = cell(N_MC, 1);
        if u == 3
            s(u).mu_ct_avg_R1 = nan(N_MC, 1);
            s(u).mu_ct_avg_R2 = nan(N_MC, 1);
            s(u).mu_ct_turn_R1 = nan(N_MC, 1);
            s(u).mu_ct_turn_R2 = nan(N_MC, 1);
            s(u).mu_ct_dom_R1 = nan(N_MC, 1);
            s(u).mu_ct_dom_R2 = nan(N_MC, 1);
        end
    end
    rmse_cal_R1 = nan(N_MC, 1);
    rmse_cal_R2 = nan(N_MC, 1);
    rmse_raw_R1 = nan(N_MC, 1);
    rmse_raw_R2 = nan(N_MC, 1);

    % 预计算转弯信息（轨迹不依赖 random_seed，只需一次）
    params0 = simulation_params();
    [turn_waypoints, turn_angle_deg, turn_rate_rad_per_sec] = get_turn_info(params0);
    fprintf('  转弯: %.1f deg @ %.4f rad/s\n', turn_angle_deg, turn_rate_rad_per_sec);

    utraj = aircraft_trajectory_create('gradual_turn', params0);
    utrue_track = aircraft_trajectory_interpolate('generate', utraj);

    tic;
    for mc = 1:N_MC
        seed = SEED_BASE + (mc - 1);
        rng('default');
        params = simulation_params();
        params.random_seed = seed;
        rng(params.random_seed);

        t1_grid = params.time_offset_radar1_sec : params.dt_sec : utraj.duration_sec;
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : utraj.duration_sec;
        n_frames = min(length(t1_grid), length(t2_grid));

        turn_frames = find_turn_frames_mc(utrue_track, 0.5);

        % ADS-B 标定
        rng(params.random_seed);
        T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
        adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
        dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
        n_check = min(5000, height(T_adsb));
        cal_step = max(1, floor(height(T_adsb) / n_check));
        for idx = 1:cal_step:height(T_adsb)
            t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
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
        dr1_est = mean(dr1_list); da1_est = mean(da1_list);
        dr2_est = mean(dr2_list); da2_est = mean(da2_list);

        % 点迹生成
        detList_R1 = cell(n_frames, 1);
        detList_R2 = cell(n_frames, 1);
        rng(params.random_seed + 1e7);
        for k = 1:n_frames
            [pos, vel] = aircraft_trajectory_interpolate(utraj, t1_grid(k));
            detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t1_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            for d = 1:length(detRaw)
                detRaw(d).aircraft_id = 1;
                Rgc = detRaw(d).prange - dr1_est; azc = detRaw(d).paz - da1_est;
                detRaw(d).drange = Rgc; detRaw(d).daz = azc;
                detRaw(d).range_meas = Rgc; detRaw(d).azimuth_meas = azc;
                if ~(isfield(detRaw(d), 'lat') && ~isnan(detRaw(d).lat))
                    [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                    detRaw(d).lat = lat_e; detRaw(d).lon = lon_e;
                end
                [~, raw_lat, raw_lon] = bistatic_inverse_solver(detRaw(d).prange, detRaw(d).paz, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                detRaw(d).raw_lat = raw_lat; detRaw(d).raw_lon = raw_lon;
            end
            detList_R1{k} = detRaw;
        end
        rng(params.random_seed + 2e7);
        for k = 1:n_frames
            [pos2, vel2] = aircraft_trajectory_interpolate(utraj, t2_grid(k));
            detRaw2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, pos2(1), pos2(2), vel2(1), vel2(2), ...
                k, t2_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            for d = 1:length(detRaw2)
                detRaw2(d).aircraft_id = 1;
                Rgc = detRaw2(d).prange - dr2_est; azc = detRaw2(d).paz - da2_est;
                detRaw2(d).drange = Rgc; detRaw2(d).daz = azc;
                detRaw2(d).range_meas = Rgc; detRaw2(d).azimuth_meas = azc;
                if ~(isfield(detRaw2(d), 'lat') && ~isnan(detRaw2(d).lat))
                    [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                    detRaw2(d).lat = lat_e; detRaw2(d).lon = lon_e;
                end
                [~, raw_lat, raw_lon] = bistatic_inverse_solver(detRaw2(d).prange, detRaw2(d).paz, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                detRaw2(d).raw_lat = raw_lat; detRaw2(d).raw_lon = raw_lon;
            end
            detList_R2{k} = detRaw2;
        end

        rmse_raw_R1(mc) = rmse_detlist(detList_R1, utrue_track, t1_grid, n_frames, 'raw');
        rmse_raw_R2(mc) = rmse_detlist(detList_R2, utrue_track, t2_grid, n_frames, 'raw');
        rmse_cal_R1(mc) = rmse_detlist(detList_R1, utrue_track, t1_grid, n_frames, 'cal');
        rmse_cal_R2(mc) = rmse_detlist(detList_R2, utrue_track, t2_grid, n_frames, 'cal');

        % 三体制跟踪
        for u = 1:N_UKF
            ukf_type = UKF_NAMES{u};
            params_r1 = params;
            params_r1.ukf_range_std_m = params.radar1_range_noise_std_m;
            params_r1.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
            params_r1.ukf_Q_scale = params.radar1_ukf_Q_scale;
            params_r1.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
            params_r1.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
            params_r1.gate_sigma = params.radar1_gate_sigma;
            params_r1.gate_vr_ms = params.radar1_gate_vr_ms;
            params_r1.tracker_K_loss = params.radar1_tracker_K_loss;
            if u == 3, params_r1.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

            switch ukf_type
                case 'jichu', ukf1_tpl = ukf_jichu('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
                case 'zishiying', ukf1_tpl = ukf_zishiying('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
                case 'imm', ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
            end
            [snaps_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, ...
                params_r1, n_frames, utrue_track, t1_grid);

            params_r2 = params;
            params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
            params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
            params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
            params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
            params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
            params_r2.gate_sigma = params.radar2_gate_sigma;
            params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
            params_r2.tracker_M = 4;
            params_r2.tracker_N = 8;
            params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
            if u == 3, params_r2.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

            switch ukf_type
                case 'jichu', ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
                case 'zishiying', ukf2_tpl = ukf_zishiying('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
                case 'imm', ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
            end
            [snaps_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, ...
                params_r2, n_frames, utrue_track, t2_grid);

            s(u).rmse_ukf_R1(mc) = rmse_tracks(snaps_R1, utrue_track, t1_grid, n_frames);
            s(u).rmse_ukf_R2(mc) = rmse_tracks(snaps_R2, utrue_track, t2_grid, n_frames);
            s(u).imp_ukf_R1(mc) = (1 - s(u).rmse_ukf_R1(mc) / rmse_cal_R1(mc)) * 100;
            s(u).imp_ukf_R2(mc) = (1 - s(u).rmse_ukf_R2(mc) / rmse_cal_R2(mc)) * 100;

            [s(u).assoc_R1(mc), s(u).nis_mean_R1(mc), s(u).nis_gate_R1(mc), ...
                ~, ~, s(u).init_fr_R1(mc)] = diagnose_tracking(snaps_R1, n_frames);
            [s(u).assoc_R2(mc), s(u).nis_mean_R2(mc), s(u).nis_gate_R2(mc), ...
                ~, ~, s(u).init_fr_R2(mc)] = diagnose_tracking(snaps_R2, n_frames);

            if u == 3
                if isfield(finalTrk1, 'mu_history')
                    mu_hist1 = finalTrk1.mu_history;
                    s(u).mu_ct_avg_R1(mc) = mean(mu_hist1(:,2)) * 100;
                    if ~isempty(turn_frames)
                        tf = turn_frames(turn_frames <= size(mu_hist1, 1));
                        if ~isempty(tf), s(u).mu_ct_turn_R1(mc) = mean(mu_hist1(tf, 2)) * 100; end
                    end
                    s(u).mu_ct_dom_R1(mc) = sum(mu_hist1(:,2) > 0.5);
                end
                if isfield(finalTrk2, 'mu_history')
                    mu_hist2 = finalTrk2.mu_history;
                    s(u).mu_ct_avg_R2(mc) = mean(mu_hist2(:,2)) * 100;
                    if ~isempty(turn_frames)
                        tf = turn_frames(turn_frames <= size(mu_hist2, 1));
                        if ~isempty(tf), s(u).mu_ct_turn_R2(mc) = mean(mu_hist2(tf, 2)) * 100; end
                    end
                    s(u).mu_ct_dom_R2(mc) = sum(mu_hist2(:,2) > 0.5);
                end
            end

            aligned_R2 = time_align_tracks(snaps_R2, params);
            s(u).rmse_ukf_R2_alg(mc) = rmse_tracks(aligned_R2, utrue_track, t1_grid, n_frames);

            matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
                'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
                'mean_dist_km', 0, 'quality', 100);
            for m = 1:N_FUS
                all_fused = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, FUSION_METHODS{m});
                s(u).rmse_fus(mc, m) = rmse_fusion_snaps(all_fused, utrue_track, t1_grid, n_frames);
            end
            [best_val, best_m] = min(s(u).rmse_fus(mc, :));
            s(u).rmse_fus_best(mc) = best_val;
            s(u).fus_best_method{mc} = FUSION_METHODS{best_m};
            s(u).imp_fus_vs_R1(mc) = (1 - best_val / s(u).rmse_ukf_R1(mc)) * 100;
            s(u).imp_fus_vs_R2(mc) = (1 - best_val / s(u).rmse_ukf_R2_alg(mc)) * 100;

            segs1 = extract_segments(snaps_R1, n_frames);
            segs2 = extract_segments(snaps_R2, n_frames);
            all_fused_best = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, FUSION_METHODS{best_m});
            segs_f = extract_fusion_segments(all_fused_best, n_frames);
            s(u).mtl_R1(mc) = compute_mtl(segs1);
            s(u).mtl_R2(mc) = compute_mtl(segs2);
            s(u).mtl_fus(mc) = compute_mtl(segs_f);
            s(u).brk_R1(mc) = max(0, size(segs1, 1) - 1);
            s(u).brk_R2(mc) = max(0, size(segs2, 1) - 1);
            s(u).brk_fus(mc) = max(0, size(segs_f, 1) - 1);

            if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
                s(u).bad_seed(mc) = 1;
                s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
                    s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
            elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
                s(u).bad_seed(mc) = 1;
                s(u).bad_reason{mc} = sprintf('DEGRADED R1=%+.0f%% R2=%+.0f%%', ...
                    s(u).imp_ukf_R1(mc), s(u).imp_ukf_R2(mc));
            end
        end

        if mod(mc, 10) == 0 || mc == N_MC
            fprintf('    MC #%d/%d done (%.0f s)\n', mc, N_MC, toc);
        end
    end

    elapsed = toc;
    fprintf('  gradual_turn MC 完成 (%.0f s)\n', elapsed);

    % 打印汇总
    fprintf('\n  --- gradual_turn 汇总 (Pi=%.3g) ---\n', pi_val);
    for u = 1:N_UKF
        fprintf('  %-12s R1_UKF=%.1f R2_UKF=%.1f Fus=%.1f\n', ...
            s(u).name, nanmean(s(u).rmse_ukf_R1), nanmean(s(u).rmse_ukf_R2), ...
            nanmean(s(u).rmse_fus_best));
    end

    % 保存
    outf = fullfile('results', sprintf('gradual_N%d_Pi%g.mat', N_MC, pi_val));
    save(outf, 's', 'rmse_cal_R1', 'rmse_cal_R2', 'rmse_raw_R1', 'rmse_raw_R2', ...
        'N_MC', 'SEED_BASE', 'UKF_NAMES', 'FUSION_METHODS', ...
        'turn_angle_deg', 'turn_rate_rad_per_sec');
end

function run_mc_180deg_uturn(N_MC, SEED_BASE, UKF_NAMES, N_UKF, FUSION_METHODS, N_FUS, pi_val)
    for u = 1:N_UKF
        s(u).name = UKF_NAMES{u};
        s(u).rmse_ukf_R1 = nan(N_MC, 1);
        s(u).rmse_ukf_R2 = nan(N_MC, 1);
        s(u).rmse_ukf_R2_alg = nan(N_MC, 1);
        s(u).rmse_fus = nan(N_MC, N_FUS);
        s(u).rmse_fus_best = nan(N_MC, 1);
        s(u).fus_best_method = cell(N_MC, 1);
        s(u).assoc_R1 = nan(N_MC, 1);
        s(u).assoc_R2 = nan(N_MC, 1);
        s(u).nis_mean_R1 = nan(N_MC, 1);
        s(u).nis_mean_R2 = nan(N_MC, 1);
        s(u).nis_gate_R1 = nan(N_MC, 1);
        s(u).nis_gate_R2 = nan(N_MC, 1);
        s(u).init_fr_R1 = nan(N_MC, 1);
        s(u).init_fr_R2 = nan(N_MC, 1);
        s(u).mtl_R1 = nan(N_MC, 1);
        s(u).mtl_R2 = nan(N_MC, 1);
        s(u).mtl_fus = nan(N_MC, 1);
        s(u).brk_R1 = nan(N_MC, 1);
        s(u).brk_R2 = nan(N_MC, 1);
        s(u).brk_fus = nan(N_MC, 1);
        s(u).imp_ukf_R1 = nan(N_MC, 1);
        s(u).imp_ukf_R2 = nan(N_MC, 1);
        s(u).imp_fus_vs_R1 = nan(N_MC, 1);
        s(u).imp_fus_vs_R2 = nan(N_MC, 1);
        s(u).bad_seed = zeros(N_MC, 1);
        s(u).bad_reason = cell(N_MC, 1);
        if u == 3
            s(u).mu_ct_avg_R1 = nan(N_MC, 1);
            s(u).mu_ct_avg_R2 = nan(N_MC, 1);
            s(u).mu_ct_turn_R1 = nan(N_MC, 1);
            s(u).mu_ct_turn_R2 = nan(N_MC, 1);
            s(u).mu_ct_dom_R1 = nan(N_MC, 1);
            s(u).mu_ct_dom_R2 = nan(N_MC, 1);
        end
    end
    rmse_cal_R1 = nan(N_MC, 1);
    rmse_cal_R2 = nan(N_MC, 1);
    rmse_raw_R1 = nan(N_MC, 1);
    rmse_raw_R2 = nan(N_MC, 1);

    omega = pi / 180.0;
    fprintf('  回头弯180度: 直线(90) -> 左转180度圆弧(1deg/s) -> 直线(270)\n');

    % 预计算转弯信息（轨迹不依赖 random_seed，只需一次）
    params0 = simulation_params();
    utraj = aircraft_trajectory_create('uturn', params0);
    utrue_track = aircraft_trajectory_interpolate('generate', utraj);

    tic;
    for mc = 1:N_MC
        seed = SEED_BASE + (mc - 1);
        rng('default');
        params = simulation_params();
        params.random_seed = seed;
        rng(params.random_seed);

        t1_grid = params.time_offset_radar1_sec : params.dt_sec : utraj.duration_sec;
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : utraj.duration_sec;
        n_frames = min(length(t1_grid), length(t2_grid));

        turn_frames = find_turn_frames_mc(utrue_track, 0.5);

        rng(params.random_seed);
        T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
        adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
        dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
        n_check = min(5000, height(T_adsb));
        cal_step = max(1, floor(height(T_adsb) / n_check));
        for idx = 1:cal_step:height(T_adsb)
            t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
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
        dr1_est = mean(dr1_list); da1_est = mean(da1_list);
        dr2_est = mean(dr2_list); da2_est = mean(da2_list);

        detList_R1 = cell(n_frames, 1);
        detList_R2 = cell(n_frames, 1);
        rng(params.random_seed + 1e7);
        for k = 1:n_frames
            [pos, vel] = aircraft_trajectory_interpolate(utraj, t1_grid(k));
            detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t1_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            for d = 1:length(detRaw)
                detRaw(d).aircraft_id = 1;
                Rgc = detRaw(d).prange - dr1_est; azc = detRaw(d).paz - da1_est;
                detRaw(d).drange = Rgc; detRaw(d).daz = azc;
                detRaw(d).range_meas = Rgc; detRaw(d).azimuth_meas = azc;
                if ~(isfield(detRaw(d), 'lat') && ~isnan(detRaw(d).lat))
                    [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                    detRaw(d).lat = lat_e; detRaw(d).lon = lon_e;
                end
                [~, raw_lat, raw_lon] = bistatic_inverse_solver(detRaw(d).prange, detRaw(d).paz, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                detRaw(d).raw_lat = raw_lat; detRaw(d).raw_lon = raw_lon;
            end
            detList_R1{k} = detRaw;
        end
        rng(params.random_seed + 2e7);
        for k = 1:n_frames
            [pos2, vel2] = aircraft_trajectory_interpolate(utraj, t2_grid(k));
            detRaw2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, pos2(1), pos2(2), vel2(1), vel2(2), ...
                k, t2_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            for d = 1:length(detRaw2)
                detRaw2(d).aircraft_id = 1;
                Rgc = detRaw2(d).prange - dr2_est; azc = detRaw2(d).paz - da2_est;
                detRaw2(d).drange = Rgc; detRaw2(d).daz = azc;
                detRaw2(d).range_meas = Rgc; detRaw2(d).azimuth_meas = azc;
                if ~(isfield(detRaw2(d), 'lat') && ~isnan(detRaw2(d).lat))
                    [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                        params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                    detRaw2(d).lat = lat_e; detRaw2(d).lon = lon_e;
                end
                [~, raw_lat, raw_lon] = bistatic_inverse_solver(detRaw2(d).prange, detRaw2(d).paz, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                detRaw2(d).raw_lat = raw_lat; detRaw2(d).raw_lon = raw_lon;
            end
            detList_R2{k} = detRaw2;
        end

        rmse_raw_R1(mc) = rmse_detlist(detList_R1, utrue_track, t1_grid, n_frames, 'raw');
        rmse_raw_R2(mc) = rmse_detlist(detList_R2, utrue_track, t2_grid, n_frames, 'raw');
        rmse_cal_R1(mc) = rmse_detlist(detList_R1, utrue_track, t1_grid, n_frames, 'cal');
        rmse_cal_R2(mc) = rmse_detlist(detList_R2, utrue_track, t2_grid, n_frames, 'cal');

        for u = 1:N_UKF
            ukf_type = UKF_NAMES{u};
            params_r1 = params;
            params_r1.ukf_range_std_m = params.radar1_range_noise_std_m;
            params_r1.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
            params_r1.ukf_Q_scale = params.radar1_ukf_Q_scale;
            params_r1.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
            params_r1.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
            params_r1.gate_sigma = params.radar1_gate_sigma;
            params_r1.gate_vr_ms = params.radar1_gate_vr_ms;
            params_r1.tracker_K_loss = params.radar1_tracker_K_loss;
            if u == 3, params_r1.imm_turn_rate_rad_per_sec = omega; end

            switch ukf_type
                case 'jichu', ukf1_tpl = ukf_jichu('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
                case 'zishiying', ukf1_tpl = ukf_zishiying('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
                case 'imm', ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
            end
            [snaps_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, ...
                params_r1, n_frames, utrue_track, t1_grid);

            params_r2 = params;
            params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
            params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
            params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
            params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
            params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
            params_r2.gate_sigma = params.radar2_gate_sigma;
            params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
            params_r2.tracker_M = 4;
            params_r2.tracker_N = 8;
            params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
            if u == 3, params_r2.imm_turn_rate_rad_per_sec = omega; end

            switch ukf_type
                case 'jichu', ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
                case 'zishiying', ukf2_tpl = ukf_zishiying('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
                case 'imm', ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
            end
            [snaps_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, ...
                params_r2, n_frames, utrue_track, t2_grid);

            s(u).rmse_ukf_R1(mc) = rmse_tracks(snaps_R1, utrue_track, t1_grid, n_frames);
            s(u).rmse_ukf_R2(mc) = rmse_tracks(snaps_R2, utrue_track, t2_grid, n_frames);
            s(u).imp_ukf_R1(mc) = (1 - s(u).rmse_ukf_R1(mc) / rmse_cal_R1(mc)) * 100;
            s(u).imp_ukf_R2(mc) = (1 - s(u).rmse_ukf_R2(mc) / rmse_cal_R2(mc)) * 100;

            [s(u).assoc_R1(mc), s(u).nis_mean_R1(mc), s(u).nis_gate_R1(mc), ...
                ~, ~, s(u).init_fr_R1(mc)] = diagnose_tracking(snaps_R1, n_frames);
            [s(u).assoc_R2(mc), s(u).nis_mean_R2(mc), s(u).nis_gate_R2(mc), ...
                ~, ~, s(u).init_fr_R2(mc)] = diagnose_tracking(snaps_R2, n_frames);

            if u == 3
                if isfield(finalTrk1, 'mu_history')
                    mu_hist1 = finalTrk1.mu_history;
                    s(u).mu_ct_avg_R1(mc) = mean(mu_hist1(:,2)) * 100;
                    if ~isempty(turn_frames)
                        tf = turn_frames(turn_frames <= size(mu_hist1, 1));
                        if ~isempty(tf), s(u).mu_ct_turn_R1(mc) = mean(mu_hist1(tf, 2)) * 100; end
                    end
                    s(u).mu_ct_dom_R1(mc) = sum(mu_hist1(:,2) > 0.5);
                end
                if isfield(finalTrk2, 'mu_history')
                    mu_hist2 = finalTrk2.mu_history;
                    s(u).mu_ct_avg_R2(mc) = mean(mu_hist2(:,2)) * 100;
                    if ~isempty(turn_frames)
                        tf = turn_frames(turn_frames <= size(mu_hist2, 1));
                        if ~isempty(tf), s(u).mu_ct_turn_R2(mc) = mean(mu_hist2(tf, 2)) * 100; end
                    end
                    s(u).mu_ct_dom_R2(mc) = sum(mu_hist2(:,2) > 0.5);
                end
            end

            aligned_R2 = time_align_tracks(snaps_R2, params);
            s(u).rmse_ukf_R2_alg(mc) = rmse_tracks(aligned_R2, utrue_track, t1_grid, n_frames);

            matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
                'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
                'mean_dist_km', 0, 'quality', 100);
            for m = 1:N_FUS
                all_fused = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, FUSION_METHODS{m});
                s(u).rmse_fus(mc, m) = rmse_fusion_snaps(all_fused, utrue_track, t1_grid, n_frames);
            end
            [best_val, best_m] = min(s(u).rmse_fus(mc, :));
            s(u).rmse_fus_best(mc) = best_val;
            s(u).fus_best_method{mc} = FUSION_METHODS{best_m};
            s(u).imp_fus_vs_R1(mc) = (1 - best_val / s(u).rmse_ukf_R1(mc)) * 100;
            s(u).imp_fus_vs_R2(mc) = (1 - best_val / s(u).rmse_ukf_R2_alg(mc)) * 100;

            segs1 = extract_segments(snaps_R1, n_frames);
            segs2 = extract_segments(snaps_R2, n_frames);
            all_fused_best = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, FUSION_METHODS{best_m});
            segs_f = extract_fusion_segments(all_fused_best, n_frames);
            s(u).mtl_R1(mc) = compute_mtl(segs1);
            s(u).mtl_R2(mc) = compute_mtl(segs2);
            s(u).mtl_fus(mc) = compute_mtl(segs_f);
            s(u).brk_R1(mc) = max(0, size(segs1, 1) - 1);
            s(u).brk_R2(mc) = max(0, size(segs2, 1) - 1);
            s(u).brk_fus(mc) = max(0, size(segs_f, 1) - 1);

            if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
                s(u).bad_seed(mc) = 1;
                s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
                    s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
            elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
                s(u).bad_seed(mc) = 1;
                s(u).bad_reason{mc} = sprintf('DEGRADED R1=%+.0f%% R2=%+.0f%%', ...
                    s(u).imp_ukf_R1(mc), s(u).imp_ukf_R2(mc));
            end
        end

        if mod(mc, 10) == 0 || mc == N_MC
            fprintf('    MC #%d/%d done (%.0f s)\n', mc, N_MC, toc);
        end
    end

    elapsed = toc;
    fprintf('  180deg_uturn MC 完成 (%.0f s)\n', elapsed);

    fprintf('\n  --- 180deg_uturn 汇总 (Pi=%.3g) ---\n', pi_val);
    for u = 1:N_UKF
        fprintf('  %-12s R1_UKF=%.1f R2_UKF=%.1f Fus=%.1f\n', ...
            s(u).name, nanmean(s(u).rmse_ukf_R1), nanmean(s(u).rmse_ukf_R2), ...
            nanmean(s(u).rmse_fus_best));
    end

    outf = fullfile('results', sprintf('uturn_N%d_Pi%g.mat', N_MC, pi_val));
    save(outf, 's', 'rmse_cal_R1', 'rmse_cal_R2', 'rmse_raw_R1', 'rmse_raw_R2', ...
        'N_MC', 'SEED_BASE', 'UKF_NAMES', 'FUSION_METHODS', 'omega');
end

%% ========================================================================
%% 工具函数
%% ========================================================================

function [wp, turn_angle_deg, omega] = get_turn_info(params)
    W1 = [126.6685, 32.2184];
    W2 = [128.2501, 31.0887];
    W3 = [132.0502, 31.4379];
    wp = [W1(1), W1(2); W2(1), W2(2); W3(1), W3(2)];
    b_in = sphere_utils_azimuth(W1(1), W1(2), W2(1), W2(2));
    b_out = sphere_utils_azimuth(W2(1), W2(2), W3(1), W3(2));
    dh = b_out - b_in;
    if dh > 180, dh = dh - 360; elseif dh < -180, dh = dh + 360; end
    turn_angle_deg = abs(dh);
    sgn = sign(dh); if sgn == 0, sgn = 1; end
    omega = sgn * 1.0 * pi / 180.0;
end

function frames = find_turn_frames_mc(utrue_track, thresh_deg_per_s)
    n = size(utrue_track, 1);
    if n < 2, frames = []; return; end
    lon = utrue_track(:,1); lat = utrue_track(:,2);
    dlon = diff(lon(1:n)); dlat = diff(lat(1:n));
    hdg = atan2d(dlon, dlat);
    hdg_diff = diff(hdg);
    hdg_diff(hdg_diff > 180) = hdg_diff(hdg_diff > 180) - 360;
    hdg_diff(hdg_diff < -180) = hdg_diff(hdg_diff < -180) + 360;
    hdg_rate = abs(hdg_diff);
    pad = [0; hdg_rate];
    dt_est = mean(diff(utrue_track(:,5)));
    if dt_est > 0, hdg_rate_per_s = pad / dt_est; else, hdg_rate_per_s = pad; end
    frames = find(hdg_rate_per_s > thresh_deg_per_s);
    if isempty(frames)
        [vals, idx] = sort(hdg_rate_per_s, 'descend');
        frames = sort(idx(1:min(3, length(idx))));
    end
end

function v = rmse_detlist(detList, utrue_track, t_grid, n_frames, mode)
    errs = [];
    for k = 1:n_frames
        tl = interp1(utrue_track(:,5), utrue_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(utrue_track(:,5), utrue_track(:,2), t_grid(k), 'linear', 'extrap');
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

function v = rmse_tracks(snaps, utrue_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(utrue_track(:,5), utrue_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(utrue_track(:,5), utrue_track(:,2), t_grid(k), 'linear', 'extrap');
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

function v = rmse_fusion_snaps(snaps, utrue_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(utrue_track(:,5), utrue_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(utrue_track(:,5), utrue_track(:,2), t_grid(k), 'linear', 'extrap');
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
    n_assoc = 0; n_pred = 0; n_init = 0;
    init_frame = 0; nis_vals = [];
    for k = 1:n_frames
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 6, n_init = n_init + 1;
        elseif trk.type == 1
            if init_frame == 0, init_frame = k; end
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
               isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;
            else
                n_pred = n_pred + 1;
            end
            if isfield(trk, 'ukf') && ~isempty(trk.ukf) && isstruct(trk.ukf) && ...
                    isfield(trk.ukf, 'nis_history')
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
    segs = []; in_seg = false; seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if isfield(trk, 'type') && trk.type == 1 && ~isnan(trk.lat)
                is_tracking = true;
            end
        end
        if is_tracking && ~in_seg, in_seg = true; seg_start = k;
        elseif ~is_tracking && in_seg, in_seg = false; segs(end+1, :) = [seg_start, k-1, k - seg_start]; end
    end
    if in_seg, segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1]; end
end

function segs = extract_fusion_segments(snaps, n_frames)
    segs = []; in_seg = false; seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat), is_tracking = true; end
        end
        if is_tracking && ~in_seg, in_seg = true; seg_start = k;
        elseif ~is_tracking && in_seg, in_seg = false; segs(end+1, :) = [seg_start, k-1, k - seg_start]; end
    end
    if in_seg, segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1]; end
end

function mtl = compute_mtl(segs)
    if isempty(segs), mtl = 0; else, mtl = mean(segs(:,3)); end
end

function v = rms_val(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end
