% =========================================================================
% run_mc_turn_compare.m — 拐弯场景三体制 UKF 对比蒙特卡洛仿真
% =========================================================================
% 【定位】
%   拐弯场景下，同一点迹数据，并行运行三种 UKF 后端（jichu / zishiying / imm），
%   每种后端均经 R1/R2 单站跟踪 + 四种融合（SCC/BC/CI/FCI），
%   逐种子输出对比，最终汇总统计。
%
% 【三种 UKF 后端】
%   ukf_jichu     — 基础 CV-UKF，固定 Q
%   ukf_zishiying — CV-UKF + 模糊自适应 Q + 机动检测
%   ukf_imm       — CV+CT 双模型 IMM-UKF + Pd-IPDA 似然
%
% 【输出】
%   每种子：三行对比（UKF RMSE / 关联率 / 融合最优）
%   汇总：  分体制统计表 + 交叉对比改善率
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ---- 配置 ----
SEED_LIST = [7, 19, 42, 68, 94, 137, 211, 305, 512, 731];
N_MC = numel(SEED_LIST);
SEED_BASE = SEED_LIST(1);

% UKF 类型
UKF_NAMES = {'jichu', 'zishiying', 'imm', 'imm_3in1'};
N_UKF = 4;

% 融合方法
FUSION_METHODS = {'SCC', 'BC', 'CI', 'FCI'};
N_FUS = length(FUSION_METHODS);

%% ---- 预分配统计结构 ----
% 使用 struct 数组: s(u).field = nan(N_MC, 1) 或 nan(N_MC, 4)
for u = 1:N_UKF
    s(u).name = UKF_NAMES{u};  %#ok<*SAGROW>

    s(u).rmse_ukf_R1     = nan(N_MC, 1);
    s(u).rmse_ukf_R2     = nan(N_MC, 1);
    s(u).rmse_ukf_R2_alg = nan(N_MC, 1);
    s(u).rmse_fus        = nan(N_MC, N_FUS);
    s(u).rmse_fus_best   = nan(N_MC, 1);
    s(u).fus_best_method = cell(N_MC, 1);

    s(u).assoc_R1    = nan(N_MC, 1);
    s(u).assoc_R2    = nan(N_MC, 1);
    s(u).nis_mean_R1 = nan(N_MC, 1);
    s(u).nis_mean_R2 = nan(N_MC, 1);
    s(u).nis_gate_R1 = nan(N_MC, 1);
    s(u).nis_gate_R2 = nan(N_MC, 1);
    s(u).init_fr_R1  = nan(N_MC, 1);
    s(u).init_fr_R2  = nan(N_MC, 1);

    s(u).mtl_R1  = nan(N_MC, 1);
    s(u).mtl_R2  = nan(N_MC, 1);
    s(u).mtl_fus = nan(N_MC, 1);
    s(u).brk_R1  = nan(N_MC, 1);
    s(u).brk_R2  = nan(N_MC, 1);
    s(u).brk_fus = nan(N_MC, 1);

    s(u).imp_ukf_R1    = nan(N_MC, 1);
    s(u).imp_ukf_R2    = nan(N_MC, 1);
    s(u).imp_fus_vs_R1 = nan(N_MC, 1);
    s(u).imp_fus_vs_R2 = nan(N_MC, 1);

    s(u).bad_seed  = zeros(N_MC, 1);
    s(u).bad_reason = cell(N_MC, 1);

    % IMM 专属 (u==3 或 u==4)
    if u >= 3
        s(u).mu_ct_avg_R1  = nan(N_MC, 1);
        s(u).mu_ct_avg_R2  = nan(N_MC, 1);
        s(u).mu_ct_turn_R1 = nan(N_MC, 1);
        s(u).mu_ct_turn_R2 = nan(N_MC, 1);
        s(u).mu_ct_dom_R1  = nan(N_MC, 1);
        s(u).mu_ct_dom_R2  = nan(N_MC, 1);
    end
end

% 公用统计
rmse_cal_R1 = nan(N_MC, 1);
rmse_cal_R2 = nan(N_MC, 1);
rmse_raw_R1 = nan(N_MC, 1);
rmse_raw_R2 = nan(N_MC, 1);
n_frames_list = nan(N_MC, 1);

%% ---- 预计算转弯信息（与种子无关） ----
params0 = simulation_params();
[turn_waypoints, turn_angle_deg, turn_rate_rad_per_sec] = get_turn_info(params0);

fprintf('╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║     拐弯场景四体制 UKF 对比 MC  N=%d  (jichu × zishiying × imm × imm_3in1)  ║\n', N_MC);
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  转弯: %.1f°@%.4f rad/s  融合: SCC/BC/CI/FCI  dt=30s  ~81帧          ║\n', ...
    turn_angle_deg, turn_rate_rad_per_sec);
fprintf('╚══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n');

tic;

%% ========================================================================
%% 主循环
%% ========================================================================
for mc = 1:N_MC
    seed = SEED_LIST(mc);
    rng('default');

    %% ---------- Phase 0: 场景初始化 ----------
    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    [traj, ~] = aircraft_trajectory_create('gradual_turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));
    n_frames_list(mc) = n_frames;

    turn_frames = find_turn_frames(true_track, 0.5);

    %% ---------- Phase 1: ADS-B 标定 ----------
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

    %% ---------- Phase 2+4: 点迹生成 + 偏差校正 ----------
    detList_R1 = cell(n_frames, 1);
    detList_R2 = cell(n_frames, 1);

    rng(params.random_seed + 1e7);
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

    rng(params.random_seed + 2e7);
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

    % 点迹 RMSE
    rmse_raw_R1(mc) = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'raw');
    rmse_raw_R2(mc) = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'raw');
    rmse_cal_R1(mc) = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'cal');
    rmse_cal_R2(mc) = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'cal');

    %% ---------- Phase 5+6+7: 三体制并行 ----------
    for u = 1:N_UKF
        ukf_type = UKF_NAMES{u};

        % ===== R1 参数配置 =====
        params_r1 = params;
        params_r1.ukf_range_std_m   = params.radar1_range_noise_std_m;
        params_r1.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        params_r1.ukf_Q_scale       = params.radar1_ukf_Q_scale;
        params_r1.ukf_P_pos_std     = params.radar1_ukf_P_pos_std;
        params_r1.ukf_P_vel_std     = params.radar1_ukf_P_vel_std;
        params_r1.gate_sigma        = params.radar1_gate_sigma;
        params_r1.gate_vr_ms        = params.radar1_gate_vr_ms;
        params_r1.tracker_K_loss    = params.radar1_tracker_K_loss;
        if u == 3  % 普通 IMM 只启用模糊自适应，用作对照
            params_r1.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec;
            params_r1.imm_adapt_mode = 'fuzzy_only';
        end
        if u == 4  % imm_3in1 需要转弯率 + 三合一自适应模式
            params_r1.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec;
            params_r1.imm_adapt_mode = '3in1';
        end

        % ===== 创建 UKF 模板 + R1 跟踪 =====
        switch ukf_type
            case 'jichu'
                ukf1_tpl = ukf_jichu('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
            case 'zishiying'
                ukf1_tpl = ukf_zishiying('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
            case 'imm'
                ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
            case 'imm_3in1'
                ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...
                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
                ukf1_tpl.filter_type = 'imm_3in1';
        end

        [snaps_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, ...
            params_r1, n_frames, true_track, t1_grid);

        % ===== R2 参数配置 =====
        params_r2 = params;
        params_r2.ukf_range_std_m   = params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        params_r2.ukf_Q_scale       = params.radar2_ukf_Q_scale;
        params_r2.ukf_P_pos_std     = params.radar2_ukf_P_pos_std;
        params_r2.ukf_P_vel_std     = params.radar2_ukf_P_vel_std;
        params_r2.gate_sigma        = params.radar2_gate_sigma;
        params_r2.gate_vr_ms        = params.radar2_gate_vr_ms;
        params_r2.tracker_M         = 4;
        params_r2.tracker_N         = 8;
        params_r2.tracker_K_loss    = params.radar2_tracker_K_loss;
        if u == 3
            params_r2.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec;
            params_r2.imm_adapt_mode = 'fuzzy_only';
        end
        if u == 4
            params_r2.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec;
            params_r2.imm_adapt_mode = '3in1';
        end

        % ===== 创建 UKF 模板 + R2 跟踪 =====
        switch ukf_type
            case 'jichu'
                ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
            case 'zishiying'
                ukf2_tpl = ukf_zishiying('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
            case 'imm'
                ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
            case 'imm_3in1'
                ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...
                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
                ukf2_tpl.filter_type = 'imm_3in1';
        end

        [snaps_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, ...
            params_r2, n_frames, true_track, t2_grid);

        % ===== UKF RMSE =====
        s(u).rmse_ukf_R1(mc) = rmse_tracks(snaps_R1, true_track, t1_grid, n_frames);
        s(u).rmse_ukf_R2(mc) = rmse_tracks(snaps_R2, true_track, t2_grid, n_frames);
        s(u).imp_ukf_R1(mc)  = (1 - s(u).rmse_ukf_R1(mc) / rmse_cal_R1(mc)) * 100;
        s(u).imp_ukf_R2(mc)  = (1 - s(u).rmse_ukf_R2(mc) / rmse_cal_R2(mc)) * 100;

        % ===== 关联诊断 =====
        [s(u).assoc_R1(mc), s(u).nis_mean_R1(mc), s(u).nis_gate_R1(mc), ...
            n_assoc1, n_pred1, s(u).init_fr_R1(mc)] = diagnose_tracking(snaps_R1, n_frames);
        [s(u).assoc_R2(mc), s(u).nis_mean_R2(mc), s(u).nis_gate_R2(mc), ...
            n_assoc2, n_pred2, s(u).init_fr_R2(mc)] = diagnose_tracking(snaps_R2, n_frames);

        % ===== IMM 专属: 模型概率 =====
        if u == 3
            if isfield(finalTrk1, 'mu_history')
                mu_hist1 = finalTrk1.mu_history;
                s(u).mu_ct_avg_R1(mc) = mean(mu_hist1(:,2)) * 100;
                if ~isempty(turn_frames)
                    tf = turn_frames(turn_frames <= size(mu_hist1,1));
                    if ~isempty(tf)
                        s(u).mu_ct_turn_R1(mc) = mean(mu_hist1(tf, 2)) * 100;
                    end
                end
                s(u).mu_ct_dom_R1(mc) = sum(mu_hist1(:,2) > 0.5);
            end
            if isfield(finalTrk2, 'mu_history')
                mu_hist2 = finalTrk2.mu_history;
                s(u).mu_ct_avg_R2(mc) = mean(mu_hist2(:,2)) * 100;
                if ~isempty(turn_frames)
                    tf = turn_frames(turn_frames <= size(mu_hist2,1));
                    if ~isempty(tf)
                        s(u).mu_ct_turn_R2(mc) = mean(mu_hist2(tf, 2)) * 100;
                    end
                end
                s(u).mu_ct_dom_R2(mc) = sum(mu_hist2(:,2) > 0.5);
            end
        end

        % ===== 时间对齐 =====
        aligned_R2 = time_align_tracks(snaps_R2, params);
        s(u).rmse_ukf_R2_alg(mc) = rmse_tracks(aligned_R2, true_track, t1_grid, n_frames);

        % ===== 融合 =====
        matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
            'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
            'mean_dist_km', 0, 'quality', 100);

        for m = 1:N_FUS
            all_fused = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, FUSION_METHODS{m});
            s(u).rmse_fus(mc, m) = rmse_fusion_snaps(all_fused, true_track, t1_grid, n_frames);
        end
        [best_val, best_m] = min(s(u).rmse_fus(mc, :));
        s(u).rmse_fus_best(mc) = best_val;
        s(u).fus_best_method{mc} = FUSION_METHODS{best_m};
        s(u).imp_fus_vs_R1(mc) = (1 - best_val / s(u).rmse_ukf_R1(mc)) * 100;
        s(u).imp_fus_vs_R2(mc) = (1 - best_val / s(u).rmse_ukf_R2_alg(mc)) * 100;

        % ===== 航迹分段 =====
        segs1 = extract_segments(snaps_R1, n_frames);
        segs2 = extract_segments(snaps_R2, n_frames);
        % 用最优融合方法的分段
        all_fused_best = run_track_fusion(matched_pair, snaps_R1, aligned_R2, params, FUSION_METHODS{best_m});
        segs_f = extract_fusion_segments(all_fused_best, n_frames);
        s(u).mtl_R1(mc)  = compute_mtl(segs1);
        s(u).mtl_R2(mc)  = compute_mtl(segs2);
        s(u).mtl_fus(mc) = compute_mtl(segs_f);
        s(u).brk_R1(mc)  = max(0, size(segs1,1) - 1);
        s(u).brk_R2(mc)  = max(0, size(segs2,1) - 1);
        s(u).brk_fus(mc) = max(0, size(segs_f,1) - 1);

        % ===== 坏种子判断 =====
        if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
            s(u).bad_seed(mc) = 1;
            s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
                s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
        elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
            s(u).bad_seed(mc) = 1;
            s(u).bad_reason{mc} = sprintf('DEGRADED R1=%+.0f%% R2=%+.0f%%', ...
                s(u).imp_ukf_R1(mc), s(u).imp_ukf_R2(mc));
        end
    end  % end UKF types loop

    %% ---------- 逐种子对比输出 ----------
    fprintf('── MC #%d (seed=%d) ──\n', mc, seed);
    fprintf('  点迹: R1原始%.0f 校准%.1f | R2原始%.0f 校准%.1f km\n', ...
        rmse_raw_R1(mc), rmse_cal_R1(mc), rmse_raw_R2(mc), rmse_cal_R2(mc));
    fprintf('  %-12s │ %8s %8s │ %7s %7s │ %8s %8s │ %6s\n', ...
        'UKF', 'R1_UKF', 'R2_UKF', 'AssocR1', 'AssocR2', 'FusBest', 'FusRMSE', 'Method');
    fprintf('  %-12s─┼%s─┼%s─┼%s─┼%s\n', ...
        '────────────', '──────────────────', '────────────────', '───────────────────', '───────');

    for u = 1:N_UKF
        markers = '';
        if s(u).bad_seed(mc), markers = ' ***BAD***'; end
        fprintf('  %-12s │ %6.1fkm %6.1fkm │ %5.0f%% %5.0f%% │ %6.1fkm %6s │ %s%s\n', ...
            s(u).name, ...
            s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc), ...
            s(u).assoc_R1(mc), s(u).assoc_R2(mc), ...
            s(u).rmse_fus_best(mc), s(u).fus_best_method{mc}, ...
            s(u).name, markers);
    end

    % 额外: 三体制融合最优交叉对比
    best_rmses = [s(1).rmse_fus_best(mc), s(2).rmse_fus_best(mc), s(3).rmse_fus_best(mc)];
    [~, best_u] = min(best_rmses);
    fprintf('  → 最优体制: %s (融合RMSE=%.1fkm)\n', s(best_u).name, best_rmses(best_u));

    % IMM 模型概率（若可用）
    if ~isnan(s(3).mu_ct_avg_R1(mc))
        fprintf('  IMM CT概率: R1 avg=%.0f%% turn=%.0f%% dom=%d | R2 avg=%.0f%% turn=%.0f%% dom=%d\n', ...
            s(3).mu_ct_avg_R1(mc), s(3).mu_ct_turn_R1(mc), s(3).mu_ct_dom_R1(mc), ...
            s(3).mu_ct_avg_R2(mc), s(3).mu_ct_turn_R2(mc), s(3).mu_ct_dom_R2(mc));
    end
    fprintf('\n');
end

elapsed = toc;
close all;

%% ========================================================================
%% 汇总统计
%% ========================================================================
fprintf('╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║           %d 次蒙特卡洛统计汇总 (%.0f s)                                 ║\n', N_MC, elapsed);
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');

%% ---- UKF RMSE 对比 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── UKF RMSE 四体制对比 (km) ───                                      ║\n');
fprintf('║  %-18s │ %10s │ %10s │ %10s │ %10s ║\n', '指标', 'jichu', 'zishiying', 'imm', 'imm_3in1');
fprintf('║  %-18s─┼%s─┼%s─┼%s  ║\n', ...
    '──────────────────', '────────────', '────────────', '────────────');
print_4way('R1 UKF RMSE', s(1).rmse_ukf_R1, s(2).rmse_ukf_R1, s(3).rmse_ukf_R1, s(4).rmse_ukf_R1);
print_4way('R2 UKF RMSE', s(1).rmse_ukf_R2, s(2).rmse_ukf_R2, s(3).rmse_ukf_R2, s(4).rmse_ukf_R2);
print_4way('R2对齐 RMSE', s(1).rmse_ukf_R2_alg, s(2).rmse_ukf_R2_alg, s(3).rmse_ukf_R2_alg, s(4).rmse_ukf_R2_alg);

%% ---- 融合 RMSE 对比 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── 融合 RMSE 四体制对比 (km) ───                                     ║\n');
fprintf('║  %-18s │ %10s │ %10s │ %10s │ %10s ║\n', '指标', 'jichu', 'zishiying', 'imm', 'imm_3in1');
fprintf('║  %-18s─┼%s─┼%s─┼%s  ║\n', ...
    '──────────────────', '────────────', '────────────', '────────────');
for m = 1:N_FUS
    print_4way(sprintf('融合 %s', FUSION_METHODS{m}), ...
        s(1).rmse_fus(:,m), s(2).rmse_fus(:,m), s(3).rmse_fus(:,m));
end
print_4way('融合最优', s(1).rmse_fus_best, s(2).rmse_fus_best, s(3).rmse_fus_best, s(4).rmse_fus_best);

%% ---- 改善率对比 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── 改善率四体制对比 (%%) ───                                          ║\n');
fprintf('║  %-18s │ %10s │ %10s │ %10s │ %10s ║\n', '指标', 'jichu', 'zishiying', 'imm', 'imm_3in1');
fprintf('║  %-18s─┼%s─┼%s─┼%s  ║\n', ...
    '──────────────────', '────────────', '────────────', '────────────');
print_4way('UKF改善 R1', s(1).imp_ukf_R1, s(2).imp_ukf_R1, s(3).imp_ukf_R1, s(4).imp_ukf_R1);
print_4way('UKF改善 R2', s(1).imp_ukf_R2, s(2).imp_ukf_R2, s(3).imp_ukf_R2, s(4).imp_ukf_R2);
print_4way('融合 vs R1', s(1).imp_fus_vs_R1, s(2).imp_fus_vs_R1, s(3).imp_fus_vs_R1, s(4).imp_fus_vs_R1);
print_4way('融合 vs R2', s(1).imp_fus_vs_R2, s(2).imp_fus_vs_R2, s(3).imp_fus_vs_R2, s(4).imp_fus_vs_R2);

%% ---- 关联 + NIS 对比 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── 关联诊断四体制对比 ───                                             ║\n');
fprintf('║  %-18s │ %10s │ %10s │ %10s │ %10s ║\n', '指标', 'jichu', 'zishiying', 'imm', 'imm_3in1');
fprintf('║  %-18s─┼%s─┼%s─┼%s  ║\n', ...
    '──────────────────', '────────────', '────────────', '────────────');
print_4way_pct('关联率 R1(%)', s(1).assoc_R1, s(2).assoc_R1, s(3).assoc_R1);
print_4way_pct('关联率 R2(%)', s(1).assoc_R2, s(2).assoc_R2, s(3).assoc_R2);
print_4way('NIS均值 R1', s(1).nis_mean_R1, s(2).nis_mean_R1, s(3).nis_mean_R1, s(4).nis_mean_R1);
print_4way('NIS均值 R2', s(1).nis_mean_R2, s(2).nis_mean_R2, s(3).nis_mean_R2, s(4).nis_mean_R2);
print_4way_pct('NIS门内 R1(%)', s(1).nis_gate_R1, s(2).nis_gate_R1, s(3).nis_gate_R1);
print_4way_pct('NIS门内 R2(%)', s(1).nis_gate_R2, s(2).nis_gate_R2, s(3).nis_gate_R2);
print_4way('起始帧 R1', s(1).init_fr_R1, s(2).init_fr_R1, s(3).init_fr_R1, s(4).init_fr_R1);
print_4way('起始帧 R2', s(1).init_fr_R2, s(2).init_fr_R2, s(3).init_fr_R2, s(4).init_fr_R2);

%% ---- MTL + 断裂对比 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── MTL 航迹平均长度 (帧) ───                                          ║\n');
fprintf('║  %-18s │ %10s │ %10s │ %10s │ %10s ║\n', '指标', 'jichu', 'zishiying', 'imm', 'imm_3in1');
fprintf('║  %-18s─┼%s─┼%s─┼%s  ║\n', ...
    '──────────────────', '────────────', '────────────', '────────────');
print_4way('MTL R1', s(1).mtl_R1, s(2).mtl_R1, s(3).mtl_R1, s(4).mtl_R1);
print_4way('MTL R2', s(1).mtl_R2, s(2).mtl_R2, s(3).mtl_R2, s(4).mtl_R2);
print_4way('MTL 融合', s(1).mtl_fus, s(2).mtl_fus, s(3).mtl_fus, s(4).mtl_fus);
print_4way('断裂 R1', s(1).brk_R1, s(2).brk_R1, s(3).brk_R1, s(4).brk_R1);
print_4way('断裂 R2', s(1).brk_R2, s(2).brk_R2, s(3).brk_R2, s(4).brk_R2);
print_4way('断裂 融合', s(1).brk_fus, s(2).brk_fus, s(3).brk_fus, s(4).brk_fus);

%% ---- 坏种子统计 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── 坏种子统计 ───                                                     ║\n');
for u = 1:N_UKF
    n_bad = sum(s(u).bad_seed);
    fprintf('║  %-10s: %d/%d (%.0f%%)                                              ║\n', ...
        s(u).name, n_bad, N_MC, n_bad/N_MC*100);
end
% 列出所有体制下都坏的种子
all_bad = s(1).bad_seed & s(2).bad_seed & s(3).bad_seed & s(4).bad_seed;
n_all_bad = sum(all_bad);
fprintf('║  四体制均坏: %d seeds                                                  ║\n', n_all_bad);
if n_all_bad > 0
    bad_seeds_list = find(all_bad);
    for i = 1:length(bad_seeds_list)
        mc_idx = bad_seeds_list(i);
        fprintf('║    seed=%d: R1=[%.0f,%.0f,%.0f,%.0f] R2=[%.0f,%.0f,%.0f,%.0f]               ║\n', ...
            SEED_LIST(mc_idx), ...
            s(1).rmse_ukf_R1(mc_idx), s(2).rmse_ukf_R1(mc_idx), s(3).rmse_ukf_R1(mc_idx), s(4).rmse_ukf_R1(mc_idx), ...
            s(1).rmse_ukf_R2(mc_idx), s(2).rmse_ukf_R2(mc_idx), s(3).rmse_ukf_R2(mc_idx), s(4).rmse_ukf_R2(mc_idx));
    end
end

%% ---- 融合算法分布 ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── 最优融合算法分布 ───                                               ║\n');
for u = 1:N_UKF
    fprintf('║  %s:', s(u).name);
    for m = 1:N_FUS
        cnt = sum(strcmp(s(u).fus_best_method, FUSION_METHODS{m}));
        fprintf('  %s=%d(%.0f%%)', FUSION_METHODS{m}, cnt, cnt/N_MC*100);
    end
    fprintf('  ║\n');
end

%% ---- IMM 模型概率（仅 imm） ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── IMM 模型概率 (仅 imm 体制 (imm & imm_3in1)) ───                                     ║\n');
print_imm_mu('CT均值 R1(%)', s(3).mu_ct_avg_R1);
print_imm_mu('CT均值 R2(%)', s(3).mu_ct_avg_R2);
print_imm_mu('CT转弯 R1(%)', s(3).mu_ct_turn_R1);
print_imm_mu('CT转弯 R2(%)', s(3).mu_ct_turn_R2);
print_imm_mu('CT占优帧 R1', s(3).mu_ct_dom_R1);
print_imm_mu('CT占优帧 R2', s(3).mu_ct_dom_R2);

%% ---- 交叉对比: zishiying vs jichu, imm vs jichu ----
fprintf('║                                                                          ║\n');
fprintf('║  ─── 交叉对比: 体制间差异 (%%) ───                                      ║\n');
fprintf('║  %-24s │ %10s │ %10s  ║\n', '指标', 'zishiying vs jichu', 'imm vs jichu', '3in1 vs jichu');
fprintf('║  %-24s─┼%s─┼%s  ║\n', ...
    '────────────────────────', '────────────', '────────────');

% 逐种子配对差异
delta_z_vs_j_R1 = (s(2).rmse_ukf_R1 - s(1).rmse_ukf_R1) ./ s(1).rmse_ukf_R1 * 100;
delta_i_vs_j_R1 = (s(3).rmse_ukf_R1 - s(1).rmse_ukf_R1) ./ s(1).rmse_ukf_R1 * 100;
delta_z_vs_j_fus = (s(2).rmse_fus_best - s(1).rmse_fus_best) ./ s(1).rmse_fus_best * 100;
delta_i_vs_j_fus = (s(3).rmse_fus_best - s(1).rmse_fus_best) ./ s(1).rmse_fus_best * 100;

print_cross_row('Δ R1 UKF (%)', delta_z_vs_j_R1, delta_i_vs_j_R1);
print_cross_row('Δ 融合最优 (%)', delta_z_vs_j_fus, delta_i_vs_j_fus);

% 胜率统计
n_z_better_R1 = sum(s(2).rmse_ukf_R1 < s(1).rmse_ukf_R1);
n_i_better_R1 = sum(s(3).rmse_ukf_R1 < s(1).rmse_ukf_R1);
n_z_better_fus = sum(s(2).rmse_fus_best < s(1).rmse_fus_best);
n_i_better_fus = sum(s(3).rmse_fus_best < s(1).rmse_fus_best);
fprintf('║  胜率(R1): zishiying %d/%d(%.0f%%)  imm %d/%d(%.0f%%)               ║\n', ...
    n_z_better_R1, N_MC, n_z_better_R1/N_MC*100, n_i_better_R1, N_MC, n_i_better_R1/N_MC*100);
fprintf('║  胜率(融合): zishiying %d/%d(%.0f%%)  imm %d/%d(%.0f%%)               ║\n', ...
    n_z_better_fus, N_MC, n_z_better_fus/N_MC*100, n_i_better_fus, N_MC, n_i_better_fus/N_MC*100);

% 三体制终极PK
best_ukf_for_seed = zeros(N_MC, 1);
best_ukf_name = cell(N_MC, 1);
for mc_idx = 1:N_MC
    rmses = [s(1).rmse_fus_best(mc_idx), s(2).rmse_fus_best(mc_idx), s(3).rmse_fus_best(mc_idx)];
    [~, best_ukf_for_seed(mc_idx)] = min(rmses);
    best_ukf_name{mc_idx} = UKF_NAMES{best_ukf_for_seed(mc_idx)};
end
fprintf('║                                                                          ║\n');
fprintf('║  终极最优体制分布 (融合RMSE最小):                                       ║\n');
for u = 1:N_UKF
    cnt = sum(best_ukf_for_seed == u);
    fprintf('║    %s: %d/%d (%.0f%%)                                                ║\n', ...
        UKF_NAMES{u}, cnt, N_MC, cnt/N_MC*100);
end

fprintf('╚══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n');

%% ---- 保存数据 ----
if ~exist('results', 'dir'), mkdir('results'); end
outf = fullfile('results', sprintf('mc_turn_compare_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 's', 'rmse_cal_R1', 'rmse_cal_R2', 'rmse_raw_R1', 'rmse_raw_R2', ...
    'n_frames_list', 'N_MC', 'SEED_BASE', 'SEED_LIST', 'UKF_NAMES', 'FUSION_METHODS', ...
    'turn_angle_deg', 'turn_rate_rad_per_sec', 'best_ukf_for_seed', 'best_ukf_name');
fprintf('\n完整数据已保存: %s\n', outf);
fprintf('Done.\n');


%% ========================================================================
%% 工具函数
%% ========================================================================

% ---- 转弯信息 ----
function [wp, turn_angle_deg, omega] = get_turn_info(params)
    W1 = [126.6685, 32.2184];
    W2 = [128.2501, 31.0887];
    W3 = [132.0502, 31.4379];
    wp = [W1(1), W1(2); W2(1), W2(2); W3(1), W3(2)];
    b_in  = sphere_utils_azimuth(W1(1), W1(2), W2(1), W2(2));
    b_out = sphere_utils_azimuth(W2(1), W2(2), W3(1), W3(2));
    dh = b_out - b_in;
    if dh > 180, dh = dh - 360; elseif dh < -180, dh = dh + 360; end
    turn_angle_deg = abs(dh);
    sgn = sign(dh); if sgn == 0, sgn = 1; end
    omega = sgn * 1.0 * pi / 180.0;
end

% ---- 定位转弯帧 ----
function frames = find_turn_frames(true_track, thresh_deg_per_s)
    n = size(true_track, 1);
    if n < 2, frames = []; return; end
    lon = true_track(:,1);
    lat = true_track(:,2);
    dlon = diff(lon(1:n));
    dlat = diff(lat(1:n));
    hdg = atan2d(dlon, dlat);
    hdg_diff = diff(hdg);
    hdg_diff(hdg_diff > 180) = hdg_diff(hdg_diff > 180) - 360;
    hdg_diff(hdg_diff < -180) = hdg_diff(hdg_diff < -180) + 360;
    hdg_rate = abs(hdg_diff);
    pad = [0; hdg_rate];
    dt_est = mean(diff(true_track(:,5)));
    if dt_est > 0
        hdg_rate_per_s = pad / dt_est;
    else
        hdg_rate_per_s = pad;
    end
    frames = find(hdg_rate_per_s > thresh_deg_per_s);
    if isempty(frames)
        [vals, idx] = sort(hdg_rate_per_s, 'descend');
        frames = sort(idx(1:min(3, length(idx))));
    end
end

% ---- 点迹RMSE ----
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

% ---- 航迹RMSE ----
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

% ---- 融合RMSE ----
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

% ---- 诊断 ----
function [assoc_rate, nis_mean, nis_gate, n_assoc, n_pred, init_frame] = diagnose_tracking(snaps, n_frames)
    n_assoc = 0; n_pred = 0; n_init = 0;
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
            if isfield(trk, 'ukf') && ~isempty(trk.ukf) && isstruct(trk.ukf) && ...
                    isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
            end
        elseif trk.type == 7
            % lost frame
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

% ---- 航迹分段 ----
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

% ---- 打印函数 ----
function print_4way(label, varargin)
    if numel(varargin) == 3
        fprintf('║  %-18s │ %7.1f±%4.1f │ %7.1f±%4.1f │ %7.1f±%4.1f  ║\n', ...
            label, nanmean(varargin{1}), nanstd(varargin{1}), nanmean(varargin{2}), nanstd(varargin{2}), nanmean(varargin{3}), nanstd(varargin{3}));
    else
        fprintf('║  %-18s │ %7.1f±%4.1f │ %7.1f±%4.1f │ %7.1f±%4.1f │ %7.1f±%4.1f ║\n', ...
            label, nanmean(varargin{1}), nanstd(varargin{1}), nanmean(varargin{2}), nanstd(varargin{2}), nanmean(varargin{3}), nanstd(varargin{3}), nanmean(varargin{4}), nanstd(varargin{4}));
    end
end

function print_4way_pct(label, varargin)
    print_4way(label, varargin{:});
end

function print_imm_mu(label, v)
    if all(isnan(v))
        fprintf('║  %-24s │ %10s                        ║\n', label, 'N/A');
    else
        fprintf('║  %-24s │ %7.1f±%4.1f                       ║\n', ...
            label, nanmean(v), nanstd(v));
    end
end

function print_cross_row(label, v1, v2)
    fprintf('║  %-24s │ %+7.1f±%4.1f │ %+7.1f±%4.1f │ %+7.1f±%4.1f  ║\n', ...
        label, nanmean(v1), nanstd(v1), nanmean(v2), nanstd(v2));
end
