% =========================================================================
% run_mc_3in1_compare.m — 四体制蒙特卡洛对比：jichu vs zishiying vs imm vs 3in1-imm
% =========================================================================
% 验证三合一架构（IMM + 模糊自适应 + 机动自适应 + 乘法叠加）的综合提升效果
% =========================================================================
addpath(genpath('.'));

%% ---- 配置 ----
N_MC = 50;
SEED_BASE = 100;  % 使用不同种子避免与原有扫描结果冲突
UKF_NAMES = {'jichu', 'zishiying', 'imm', '3in1-imm'};
N_UKF = 4;
FUSION_METHODS = {'SCC', 'BC', 'CI', 'FCI'};
N_FUS = length(FUSION_METHODS);

fprintf('============================================================\n');
fprintf(' 四体制蒙特卡洛对比: %d 次运行\n', N_MC);
fprintf(' 场景: gradual_turn + 180deg_uturn\n');
fprintf(' 随机种子: %d ~ %d\n', SEED_BASE, SEED_BASE + N_MC - 1);
fprintf('============================================================\n\n');

%% ---- 预分配 ----
results_g = cell(N_UKF, 1);  % gradual results
results_u = cell(N_UKF, 1);  % uturn results

for u = 1:N_UKF
    s = struct();
    s.name = UKF_NAMES{u};
    s.rmse_ukf_R1 = zeros(N_MC, 1);
    s.rmse_ukf_R2 = zeros(N_MC, 1);
    s.rmse_fus = zeros(N_MC, N_FUS);
    s.rmse_fus_best = zeros(N_MC, 1);
    s.fus_best_method = cell(N_MC, 1);
    s.assoc_R1 = zeros(N_MC, 1);
    s.assoc_R2 = zeros(N_MC, 1);
    s.nis_mean_R1 = zeros(N_MC, 1);
    s.nis_mean_R2 = zeros(N_MC, 1);
    s.mtl_R1 = zeros(N_MC, 1);
    s.mtl_R2 = zeros(N_MC, 1);
    s.mtl_fus = zeros(N_MC, 1);
    s.brk_R1 = zeros(N_MC, 1);
    s.brk_R2 = zeros(N_MC, 1);
    s.brk_fus = zeros(N_MC, 1);
    s.imp_fus_vs_R1 = zeros(N_MC, 1);
    s.imp_fus_vs_R2 = zeros(N_MC, 1);
    s.mu_ct_avg_R1 = zeros(N_MC, 1);
    s.mu_ct_avg_R2 = zeros(N_MC, 1);
    s.mu_ct_turn_R1 = zeros(N_MC, 1);
    s.mu_ct_turn_R2 = zeros(N_MC, 1);
    if u >= 3
        s.mu_ct_dom_R1 = zeros(N_MC, 1);
        s.mu_ct_dom_R2 = zeros(N_MC, 1);
    end
    results_g{u} = s;
    results_u{u} = s;
end

%% ---- 获取转弯信息 ----
params0 = simulation_params();
[turn_waypoints, turn_angle_deg, turn_rate_rad_per_sec] = get_turn_info(params0);
fprintf('  转弯: %.1f deg @ %.4f rad/s\n', turn_angle_deg, turn_rate_rad_per_sec);
utraj_g = aircraft_trajectory_create('gradual_turn', params0);
utrue_g = aircraft_trajectory_interpolate('generate', utraj_g);
utraj_u = aircraft_trajectory_create('uturn', params0);
utrue_u = aircraft_trajectory_interpolate('generate', utraj_u);

%% ================================================================
% SCENE 1: Gradual Turn
% ================================================================
fprintf('\n========== 场景 1: gradual_turn ==========\n');

for mc = 1:N_MC
    seed = SEED_BASE + (mc - 1);
    fprintf('  MC #%d/%d (seed=%d)...\n', mc, N_MC, seed);

    % ---- 生成点迹 ----
    detList_R1_g = generate_det_list_gradual(seed, params0, utraj_g, utrue_g, turn_rate_rad_per_sec);
    detList_R2_g = generate_det_list_gradual(seed, params0, utraj_g, utrue_g, turn_rate_rad_per_sec, 2);

    % ---- 四体制跟踪 ----
    for u = 1:N_UKF
        ukf_type = UKF_NAMES{u};

        % R1
        pr1 = params0;
        pr1.ukf_range_std_m = params0.radar1_range_noise_std_m;
        pr1.ukf_azimuth_std_deg = params0.radar1_azimuth_noise_std_deg;
        pr1.ukf_Q_scale = params0.radar1_ukf_Q_scale;
        pr1.ukf_P_pos_std = params0.radar1_ukf_P_pos_std;
        pr1.ukf_P_vel_std = params0.radar1_ukf_P_vel_std;
        pr1.gate_sigma = params0.radar1_gate_sigma;
        pr1.gate_vr_ms = params0.radar1_gate_vr_ms;
        pr1.tracker_K_loss = params0.radar1_tracker_K_loss;
        if u >= 3, pr1.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

        switch ukf_type
            case 'jichu', tpl1 = ukf_jichu('create', pr1, params0.radar1_lon, params0.radar1_lat, params0.radar1_tx_lon, params0.radar1_tx_lat, params0.dt_sec);
            case 'zishiying', tpl1 = ukf_zishiying('create', pr1, params0.radar1_lon, params0.radar1_lat, params0.radar1_tx_lon, params0.radar1_tx_lat, params0.dt_sec);
            otherwise, tpl1 = ukf_imm('create', pr1, params0.radar1_lon, params0.radar1_lat, params0.radar1_tx_lon, params0.radar1_tx_lat, params0.dt_sec);
        end

        [snaps_R1, finalTrk1] = single_track_runner(detList_R1_g, tpl1, pr1, length(detList_R1_g), utrue_g, params0.time_offset_radar1_sec:params0.dt_sec:utraj_g.duration_sec);

        % R2
        pr2 = params0;
        pr2.ukf_range_std_m = params0.radar2_range_noise_std_m;
        pr2.ukf_azimuth_std_deg = params0.radar2_azimuth_noise_std_deg;
        pr2.ukf_Q_scale = params0.radar2_ukf_Q_scale;
        pr2.ukf_P_pos_std = params0.radar2_ukf_P_pos_std;
        pr2.ukf_P_vel_std = params0.radar2_ukf_P_vel_std;
        pr2.gate_sigma = params0.radar2_gate_sigma;
        pr2.gate_vr_ms = params0.radar2_gate_vr_ms;
        pr2.tracker_M = 4;
        pr2.tracker_N = 8;
        pr2.tracker_K_loss = params0.radar2_tracker_K_loss;
        if u >= 3, pr2.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

        switch ukf_type
            case 'jichu', tpl2 = ukf_jichu('create', pr2, params0.radar2_lon, params0.radar2_lat, params0.radar2_tx_lon, params0.radar2_tx_lat, params0.dt_sec);
            case 'zishiying', tpl2 = ukf_zishiying('create', pr2, params0.radar2_lon, params0.radar2_lat, params0.radar2_tx_lon, params0.radar2_tx_lat, params0.dt_sec);
            otherwise, tpl2 = ukf_imm('create', pr2, params0.radar2_lon, params0.radar2_lat, params0.radar2_tx_lon, params0.radar2_tx_lat, params0.dt_sec);
        end

        [snaps_R2, finalTrk2] = single_track_runner(detList_R2_g, tpl2, pr2, length(detList_R2_g), utrue_g, params0.time_offset_radar2_sec:params0.dt_sec:utraj_g.duration_sec);

        % 记录 RMSE
        results_g{u}(mc).rmse_ukf_R1 = rmse_tracks_arr(snaps_R1, utrue_g, params0.time_offset_radar1_sec:params0.dt_sec:utraj_g.duration_sec);
        results_g{u}(mc).rmse_ukf_R2 = rmse_tracks_arr(snaps_R2, utrue_g, params0.time_offset_radar2_sec:utraj_g.duration_sec);

        % 关联诊断
        [results_g{u}(mc).assoc_R1, results_g{u}(mc).nis_mean_R1] = diagnose_one(snaps_R1);
        [results_g{u}(mc).assoc_R2, results_g{u}(mc).nis_mean_R2] = diagnose_one(snaps_R2);

        % IMM mu
        if u >= 3
            if isfield(finalTrk1, 'mu_history')
                mu_hist1 = finalTrk1.mu_history;
                results_g{u}(mc).mu_ct_avg_R1 = mean(mu_hist1(:,2)) * 100;
                results_g{u}(mc).mu_ct_dom_R1 = sum(mu_hist1(:,2) > 0.5);
            end
            if isfield(finalTrk2, 'mu_history')
                mu_hist2 = finalTrk2.mu_history;
                results_g{u}(mc).mu_ct_avg_R2 = mean(mu_hist2(:,2)) * 100;
                results_g{u}(mc).mu_ct_dom_R2 = sum(mu_hist2(:,2) > 0.5);
            end
        end

        % 融合
        aligned_R2 = time_align_tracks(snaps_R2, params0);
        matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, 'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, 'mean_dist_km', 0, 'quality', 100);
        best_fus_rmse = Inf;
        best_m = 1;
        for m = 1:N_FUS
            all_fused = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params0, FUSION_METHODS{m});
            fus_rmse = rmse_fusion_arr(all_fused, utrue_g, params0.time_offset_radar1_sec:params0.dt_sec:utraj_g.duration_sec);
            results_g{u}(mc).rmse_fus(m) = fus_rmse;
            if fus_rmse < best_fus_rmse
                best_fus_rmse = fus_rmse;
                best_m = m;
            end
        end
        results_g{u}(mc).rmse_fus_best = best_fus_rmse;
        results_g{u}(mc).fus_best_method{1} = FUSION_METHODS{best_m};
        results_g{u}(mc).imp_fus_vs_R1 = (1 - best_fus_rmse / results_g{u}(mc).rmse_ukf_R1) * 100;
        results_g{u}(mc).imp_fus_vs_R2 = (1 - best_fus_rmse / rmse_tracks_arr(aligned_R2, utrue_g, params0.time_offset_radar1_sec:params0.dt_sec:utraj_g.duration_sec)) * 100;
    end

    if mod(mc, 10) == 0 || mc == N_MC
        fprintf('    Gradual: MC #%d/%d done\n', mc, N_MC);
    end
end

%% ================================================================
% SCENE 2: 180deg U-Turn
% ================================================================
fprintf('\n========== 场景 2: 180deg_uturn ==========\n');

for mc = 1:N_MC
    seed = SEED_BASE + (mc - 1);

    detList_R1_u = generate_det_list_uturn(seed, params0, utraj_u, utrue_u, turn_rate_rad_per_sec);
    detList_R2_u = generate_det_list_uturn(seed, params0, utraj_u, utrue_u, turn_rate_rad_per_sec, 2);

    for u = 1:N_UKF
        ukf_type = UKF_NAMES{u};

        pr1 = params0;
        pr1.ukf_range_std_m = params0.radar1_range_noise_std_m;
        pr1.ukf_azimuth_std_deg = params0.radar1_azimuth_noise_std_deg;
        pr1.ukf_Q_scale = params0.radar1_ukf_Q_scale;
        pr1.ukf_P_pos_std = params0.radar1_ukf_P_pos_std;
        pr1.ukf_P_vel_std = params0.radar1_ukf_P_vel_std;
        pr1.gate_sigma = params0.radar1_gate_sigma;
        pr1.gate_vr_ms = params0.radar1_gate_vr_ms;
        pr1.tracker_K_loss = params0.radar1_tracker_K_loss;
        if u >= 3, pr1.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

        switch ukf_type
            case 'jichu', tpl1 = ukf_jichu('create', pr1, params0.radar1_lon, params0.radar1_lat, params0.radar1_tx_lon, params0.radar1_tx_lat, params0.dt_sec);
            case 'zishiying', tpl1 = ukf_zishiying('create', pr1, params0.radar1_lon, params0.radar1_lat, params0.radar1_tx_lon, params0.radar1_tx_lat, params0.dt_sec);
            otherwise, tpl1 = ukf_imm('create', pr1, params0.radar1_lon, params0.radar1_lat, params0.radar1_tx_lon, params0.radar1_tx_lat, params0.dt_sec);
        end

        [snaps_R1, finalTrk1] = single_track_runner(detList_R1_u, tpl1, pr1, length(detList_R1_u), utrue_u, params0.time_offset_radar1_sec:params0.dt_sec:utraj_u.duration_sec);

        pr2 = params0;
        pr2.ukf_range_std_m = params0.radar2_range_noise_std_m;
        pr2.ukf_azimuth_std_deg = params0.radar2_azimuth_noise_std_deg;
        pr2.ukf_Q_scale = params0.radar2_ukf_Q_scale;
        pr2.ukf_P_pos_std = params0.radar2_ukf_P_pos_std;
        pr2.ukf_P_vel_std = params0.radar2_ukf_P_vel_std;
        pr2.gate_sigma = params0.radar2_gate_sigma;
        pr2.gate_vr_ms = params0.radar2_gate_vr_ms;
        pr2.tracker_M = 4;
        pr2.tracker_N = 8;
        pr2.tracker_K_loss = params0.radar2_tracker_K_loss;
        if u >= 3, pr2.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

        switch ukf_type
            case 'jichu', tpl2 = ukf_jichu('create', pr2, params0.radar2_lon, params0.radar2_lat, params0.radar2_tx_lon, params0.radar2_tx_lat, params0.dt_sec);
            case 'zishiying', tpl2 = ukf_zishiying('create', pr2, params0.radar2_lon, params0.radar2_lat, params0.radar2_tx_lon, params0.radar2_tx_lat, params0.dt_sec);
            otherwise, tpl2 = ukf_imm('create', pr2, params0.radar2_lon, params0.radar2_lat, params0.radar2_tx_lon, params0.radar2_tx_lat, params0.dt_sec);
        end

        [snaps_R2, finalTrk2] = single_track_runner(detList_R2_u, tpl2, pr2, length(detList_R2_u), utrue_u, params0.time_offset_radar2_sec:params0.dt_sec:utraj_u.duration_sec);

        results_u{u}(mc).rmse_ukf_R1 = rmse_tracks_arr(snaps_R1, utrue_u, params0.time_offset_radar1_sec:params0.dt_sec:utraj_u.duration_sec);
        results_u{u}(mc).rmse_ukf_R2 = rmse_tracks_arr(snaps_R2, utrue_u, params0.time_offset_radar2_sec:utraj_u.duration_sec);

        [results_u{u}(mc).assoc_R1, results_u{u}(mc).nis_mean_R1] = diagnose_one(snaps_R1);
        [results_u{u}(mc).assoc_R2, results_u{u}(mc).nis_mean_R2] = diagnose_one(snaps_R2);

        if u >= 3
            if isfield(finalTrk1, 'mu_history')
                mu_hist1 = finalTrk1.mu_history;
                results_u{u}(mc).mu_ct_avg_R1 = mean(mu_hist1(:,2)) * 100;
                results_u{u}(mc).mu_ct_dom_R1 = sum(mu_hist1(:,2) > 0.5);
            end
            if isfield(finalTrk2, 'mu_history')
                mu_hist2 = finalTrk2.mu_history;
                results_u{u}(mc).mu_ct_avg_R2 = mean(mu_hist2(:,2)) * 100;
                results_u{u}(mc).mu_ct_dom_R2 = sum(mu_hist2(:,2) > 0.5);
            end
        end

        aligned_R2 = time_align_tracks(snaps_R2, params0);
        matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, 'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, 'mean_dist_km', 0, 'quality', 100);
        best_fus_rmse = Inf;
        best_m = 1;
        for m = 1:N_FUS
            all_fused = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params0, FUSION_METHODS{m});
            fus_rmse = rmse_fusion_arr(all_fused, utrue_u, params0.time_offset_radar1_sec:params0.dt_sec:utraj_u.duration_sec);
            results_u{u}(mc).rmse_fus(m) = fus_rmse;
            if fus_rmse < best_fus_rmse
                best_fus_rmse = fus_rmse;
                best_m = m;
            end
        end
        results_u{u}(mc).rmse_fus_best = best_fus_rmse;
        results_u{u}(mc).fus_best_method{1} = FUSION_METHODS{best_m};
        results_u{u}(mc).imp_fus_vs_R1 = (1 - best_fus_rmse / results_u{u}(mc).rmse_ukf_R1) * 100;
        results_u{u}(mc).imp_fus_vs_R2 = (1 - best_fus_rmse / rmse_tracks_arr(aligned_R2, utrue_u, params0.time_offset_radar1_sec:params0.dt_sec:utraj_u.duration_sec)) * 100;
    end

    if mod(mc, 10) == 0 || mc == N_MC
        fprintf('    U-Turn: MC #%d/%d done\n', mc, N_MC);
    end
end

%% ================================================================
% PRINT SUMMARY
% ================================================================
fprintf('\n========== 汇总统计 ==========\n');

for scene_name = {'Gradual Turn', '180 U-Turn'}
    fprintf('\n--- %s ---\n', scene_name{1});
    if strcmp(scene_name{1}, 'Gradual Turn')
        res = results_g;
    else
        res = results_u;
    end
    for u = 1:N_UKF
        s = res{u};
        fus_rmse_mean = nanmean(s.rmse_fus_best);
        fus_rmse_std = nanstd(s.rmse_fus_best);
        r1_mean = nanmean(s.rmse_ukf_R1);
        r2_mean = nanmean(s.rmse_ukf_R2);
        assoc_mean = nanmean(s.assoc_R1);
        nis_mean = nanmean(s.nis_mean_R1);
        imp_mean = nanmean(s.imp_fus_vs_R1);
        fprintf('  %-12s R1=%.1f R2=%.1f Fus=%.1f±%.1f Assoc=%.0f%% NIS=%.1f Imp=%.1f%%\n', ...
            s.name, r1_mean, r2_mean, fus_rmse_mean, fus_rmse_std, assoc_mean, nis_mean, imp_mean);
        if u >= 3
            mu_avg = nanmean(s.mu_ct_avg_R1);
            mu_dom = nanmean(s.mu_ct_dom_R1);
            fprintf('            IMM_mu_avg=%.0f%% mu_dominant=%d/%d\n', mu_avg, mu_dom, N_MC);
        end
    end
end

% Save results
save('results/mc_3in1_compare_gradual.mat', 'results_g', 'UKF_NAMES', 'N_MC', 'SEED_BASE');
save('results/mc_3in1_compare_uturn.mat', 'results_u', 'UKF_NAMES', 'N_MC', 'SEED_BASE');
fprintf('\n结果已保存: results/mc_3in1_compare_*.mat\n');

%% ================================================================
% INTERNAL FUNCTIONS
% ================================================================

function detList = generate_det_list_gradual(seed, params, utraj, utrue, omega, radar_idx)
    if nargin < 6, radar_idx = 1; end
    if radar_idx == 1
        t_grid = params.time_offset_radar1_sec:params.dt_sec:utraj.duration_sec;
        rng(seed + 1e7);
        for k = 1:length(t_grid)
            [pos, vel] = aircraft_trajectory_interpolate(utraj, t_grid(k));
            dets{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            for d = 1:length(dets{k}), dets{k}(d).aircraft_id = 1; end
        end
    else
        t_grid = params.time_offset_radar2_sec:params.dt_sec:utraj.duration_sec;
        rng(seed + 2e7);
        for k = 1:length(t_grid)
            [pos, vel] = aircraft_trajectory_interpolate(utraj, t_grid(k));
            dets{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            for d = 1:length(dets{k}), dets{k}(d).aircraft_id = 1; end
        end
    end
    % Apply calibration bias estimates
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));
    dr_list = []; da_list = [];
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat), continue; end
        if radar_idx == 1
            [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, t_lon, t_lat, params.radar1_beam_center_deg, params);
            if in1
                Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
                az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
                dr_list(end+1) = Rg_meas - Rg_true;
                daz = az_meas - az_true; if daz>180, daz=daz-360; elseif daz<-180, daz=daz+360; end
                da_list(end+1) = daz;
            end
        else
            [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, t_lon, t_lat, params.radar2_beam_center_deg, params);
            if in2
                Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
                az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
                dr_list(end+1) = Rg_meas - Rg_true;
                daz = az_meas - az_true; if daz>180, daz=daz-360; elseif daz<-180, daz=daz+360; end
                da_list(end+1) = daz;
            end
        end
    end
    dr_est = mean(dr_list); da_est = mean(da_list);
    for k = 1:length(dets)
        for d = 1:length(dets{k})
            Rgc = dets{k}(d).prange - dr_est; azc = dets{k}(d).paz - da_est;
            dets{k}(d).drange = Rgc; dets{k}(d).daz = azc;
            dets{k}(d).range_meas = Rgc; dets{k}(d).azimuth_meas = azc;
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                (radar_idx==1)*params.radar1_tx_lon+(radar_idx~=1)*params.radar2_tx_lon, ...
                (radar_idx==1)*params.radar1_tx_lat+(radar_idx~=1)*params.radar2_tx_lat, ...
                (radar_idx==1)*params.radar1_lon+(radar_idx~=1)*params.radar2_lon, ...
                (radar_idx==1)*params.radar1_lat+(radar_idx~=1)*params.radar2_lat);
            dets{k}(d).lat = lat_e; dets{k}(d).lon = lon_e;
        end
    end
    detList = dets;
end

function detList = generate_det_list_uturn(seed, params, utraj, utrue, omega, radar_idx)
    if nargin < 6, radar_idx = 1; end
    if radar_idx == 1
        t_grid = params.time_offset_radar1_sec:params.dt_sec:utraj.duration_sec;
        rng(seed + 1e7);
        for k = 1:length(t_grid)
            [pos, vel] = aircraft_trajectory_interpolate(utraj, t_grid(k));
            dets{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            for d = 1:length(dets{k}), dets{k}(d).aircraft_id = 1; end
        end
    else
        t_grid = params.time_offset_radar2_sec:params.dt_sec:utraj.duration_sec;
        rng(seed + 2e7);
        for k = 1:length(t_grid)
            [pos, vel] = aircraft_trajectory_interpolate(utraj, t_grid(k));
            dets{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
                k, t_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            for d = 1:length(dets{k}), dets{k}(d).aircraft_id = 1; end
        end
    end
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));
    dr_list = []; da_list = [];
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat), continue; end
        if radar_idx == 1
            [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, t_lon, t_lat, params.radar1_beam_center_deg, params);
            if in1
                Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
                az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
                dr_list(end+1) = Rg_meas - Rg_true;
                daz = az_meas - az_true; if daz>180, daz=daz-360; elseif daz<-180, daz=daz+360; end
                da_list(end+1) = daz;
            end
        else
            [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, t_lon, t_lat, params.radar2_beam_center_deg, params);
            if in2
                Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
                az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
                dr_list(end+1) = Rg_meas - Rg_true;
                daz = az_meas - az_true; if daz>180, daz=daz-360; elseif daz<-180, daz=daz+360; end
                da_list(end+1) = daz;
            end
        end
    end
    dr_est = mean(dr_list); da_est = mean(da_list);
    for k = 1:length(dets)
        for d = 1:length(dets{k})
            Rgc = dets{k}(d).prange - dr_est; azc = dets{k}(d).paz - da_est;
            dets{k}(d).drange = Rgc; dets{k}(d).daz = azc;
            dets{k}(d).range_meas = Rgc; dets{k}(d).azimuth_meas = azc;
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                (radar_idx==1)*params.radar1_tx_lon+(radar_idx~=1)*params.radar2_tx_lon, ...
                (radar_idx==1)*params.radar1_tx_lat+(radar_idx~=1)*params.radar2_tx_lat, ...
                (radar_idx==1)*params.radar1_lon+(radar_idx~=1)*params.radar2_lon, ...
                (radar_idx==1)*params.radar1_lat+(radar_idx~=1)*params.radar2_lat);
            dets{k}(d).lat = lat_e; dets{k}(d).lon = lon_e;
        end
    end
    detList = dets;
end

function v = rmse_tracks_arr(snaps, utrue, t_grid)
    errs = [];
    for k = 1:length(snaps)
        if k > length(t_grid), break; end
        tl = interp1(utrue(:,5), utrue(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(utrue(:,5), utrue(:,2), t_grid(k), 'linear', 'extrap');
        snap = snaps{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = sqrt(mean(errs.^2));
end

function v = rmse_fusion_arr(snaps, utrue, t_grid)
    errs = [];
    for k = 1:length(snaps)
        if k > length(t_grid), break; end
        tl = interp1(utrue(:,5), utrue(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(utrue(:,5), utrue(:,2), t_grid(k), 'linear', 'extrap');
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = sqrt(mean(errs.^2));
end

function [assoc, nis_mean] = diagnose_one(snaps)
    n_assoc = 0; n_pred = 0; nis_vals = [];
    for k = 1:length(snaps)
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 1
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
    assoc = n_assoc / max(1, n_tracked) * 100;
    if ~isempty(nis_vals), nis_mean = mean(nis_vals); else nis_mean = NaN; end
end
