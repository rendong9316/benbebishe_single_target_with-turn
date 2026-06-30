% =========================================================================
% run_mc_turn_180deg.m — 回头弯(180°)蒙特卡洛仿真（IMM: CV+CT双模型）
% =========================================================================
% 航迹: 回头弯180° (125.5°E,33°N起, 正东飞→左转180°半圆→正西飞回)
%      转弯段180s≈6帧(@30s), 总帧数~41
% 跟踪器: IMM (CV+CT双模型, Pd-IPDA似然, Pi=[.90 .10])
% 默认500次MC, 无图窗, 完整控制台统计输出
%
% ==== 可调参数（每次只改一个，逐次迭代）====
% 参数均来自 simulation_params.m, 在此覆盖测试值
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

%% ==================== 可调参数区 ====================
% 说明: 基线值来自simulation_params.m。每次只改一个参数，改后跑MC验证。
%       记录每次改动: 参数名, 旧值→新值, 效果(融合RMSE/坏种子率)

N_MC = 2;
SEED_BASE = 1;

% ---- IMM参数 ----
% Markov转移矩阵: Pi(1,2)=CV→CT概率, Pi(2,1)=CT→CV概率
IMM_Pi_CV_to_CT = 0.10;   % 基线0.10 (平均10帧后切换)
IMM_Pi_CT_to_CV = 0.10;   % 基线0.10 (平均10帧后切换)
IMM_mu_init_CV = 0.50;    % CV初始概率 (基线0.50)

% ---- 波门参数 ----
% gate_sigma: 关联波门大小(标准差倍数)，调大→更多检测落入波门→关联率↑
% 但太大→杂波关联概率↑
GATE_SIGMA_R1 = 3.0;      % 基线来自simulation_params.m (radar1_gate_sigma)
GATE_SIGMA_R2 = 3.0;      % 基线来自simulation_params.m (radar2_gate_sigma)

% ---- UKF参数 ----
% Q_scale: 过程噪声缩放，调大→滤波器更信任量测→跟踪机动能力↑但噪声敏感
UKF_Q_SCALE_R1 = 0;       % 0=用simulation_params默认值
UKF_Q_SCALE_R2 = 0;

% ---- 航迹起始参数 ----
TRACKER_M_R2 = 4;         % M/N起始的M (R2)
TRACKER_N_R2 = 8;         % M/N起始的N (R2)

% ---- 其他 ----
USE_FUZZY_ADAPTIVE = true;  % 模糊自适应Q开关
% =====================================================

%% ---- 预计算转弯信息（与种子无关） ----
params0 = simulation_params();
turn_rate_rad_per_sec = +1.0 * pi / 180.0;  % 左转 +1°/s

% 航迹长度信息(与种子无关, 用于预知帧数)
[traj0, ~] = aircraft_trajectory_create('uturn', params0);
n_frames_ref = traj0.n_steps;
approach_dur = 120e3 / params0.aircraft_speed_ms;
turn_dur = 180.0;
turn_start_sec = approach_dur;
turn_end_sec = approach_dur + turn_dur;

fprintf('回头弯180°几何: 正东→左转180°半圆(1°/s, R=13.2km, 180s)→正西\n');
fprintf('  转弯段: t=%.0f~%.0fs, 约帧%d~%d (共~6帧 @30s)\n', ...
    turn_start_sec, turn_end_sec, ...
    floor(turn_start_sec/params0.dt_sec), ceil(turn_end_sec/params0.dt_sec));
fprintf('  参考帧数: %d\n\n', n_frames_ref);

%% ---- 预分配 ----
n_frames_list = zeros(N_MC, 1);
rmse = struct();
rmse.raw_R1 = nan(N_MC,1); rmse.raw_R2 = nan(N_MC,1);
rmse.cal_R1 = nan(N_MC,1); rmse.cal_R2 = nan(N_MC,1);
rmse.ukf_R1 = nan(N_MC,1); rmse.ukf_R2 = nan(N_MC,1);
rmse.ukf_R2_aligned = nan(N_MC,1);
rmse.fus = nan(N_MC,4); rmse.fus_best = nan(N_MC,1);
fus_best_method = cell(N_MC,1);

mtl_R1 = nan(N_MC,1); mtl_R2 = nan(N_MC,1); mtl_fus = nan(N_MC,1);
brk_R1 = nan(N_MC,1); brk_R2 = nan(N_MC,1); brk_fus = nan(N_MC,1);
seg_count_R1 = nan(N_MC,1); seg_count_R2 = nan(N_MC,1); seg_count_fus = nan(N_MC,1);

nis_mean_R1 = nan(N_MC,1); nis_mean_R2 = nan(N_MC,1);
nis_gate_R1 = nan(N_MC,1); nis_gate_R2 = nan(N_MC,1);
assoc_R1 = nan(N_MC,1); assoc_R2 = nan(N_MC,1);
init_frame_R1 = nan(N_MC,1); init_frame_R2 = nan(N_MC,1);

mu_ct_avg_R1 = nan(N_MC,1); mu_ct_avg_R2 = nan(N_MC,1);
mu_ct_turn_R1 = nan(N_MC,1); mu_ct_turn_R2 = nan(N_MC,1);
mu_ct_dom_R1 = nan(N_MC,1); mu_ct_dom_R2 = nan(N_MC,1);

imp_ukf_R1 = nan(N_MC,1); imp_ukf_R2 = nan(N_MC,1);
imp_fus_vs_R1 = nan(N_MC,1); imp_fus_vs_R2 = nan(N_MC,1);

bad_seed = zeros(N_MC,1);
bad_reason = cell(N_MC,1);
seg_info = cell(N_MC,1);

%% ---- 表头 ----
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  回头弯180°MC N=%d  IMM:CV+CT  Pi=[%.2f %.2f; %.2f %.2f]  ║\n', ...
    N_MC, 1-IMM_Pi_CV_to_CT, IMM_Pi_CV_to_CT, IMM_Pi_CT_to_CV, 1-IMM_Pi_CT_to_CV);
fprintf('║  gate_sigma R1=%.1f R2=%.1f  dt=30s  n_frames~%d              ║\n', ...
    GATE_SIGMA_R1, GATE_SIGMA_R2, n_frames_ref);
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

tic;

%% ===== 主循环 =====
for mc = 1:N_MC
    seed = SEED_BASE + (mc - 1);
    rng('default');

    %% ---------- Phase 0: 场景初始化 ----------
    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    [traj, ~] = aircraft_trajectory_create('uturn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));
    n_frames_list(mc) = n_frames;

    % 转弯帧定位
    tf_start = find(t1_grid >= turn_start_sec, 1);
    tf_end = find(t1_grid >= turn_end_sec, 1);
    if isempty(tf_start), tf_start = 0; end
    if isempty(tf_end), tf_end = n_frames; end
    turn_frames = (tf_start:tf_end)';

    %% ---------- Phase 1: ADS-B 标定 ----------
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

    %% ---------- Phase 2+4: 点迹生成 + 偏差校正 ----------
    detList_R1 = cell(n_frames, 1); detList_R2 = cell(n_frames, 1);

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
        [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
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

    rmse.raw_R1(mc) = mc_rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'raw');
    rmse.raw_R2(mc) = mc_rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'raw');
    rmse.cal_R1(mc) = mc_rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'cal');
    rmse.cal_R2(mc) = mc_rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'cal');

    %% ---------- Phase 5: IMM 跟踪 ----------
    % R1参数配置
    params.gate_sigma = GATE_SIGMA_R1;
    params.gate_vr_ms = params.radar1_gate_vr_ms;
    params.tracker_K_loss = params.radar1_tracker_K_loss;
    params.use_fuzzy_adaptive = USE_FUZZY_ADAPTIVE;
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    if UKF_Q_SCALE_R1 > 0
        params.ukf_Q_scale = UKF_Q_SCALE_R1;
    else
        params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    end
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;

    ukf1_cv = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf1_ct = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf1_ct.model_type = 'CT';
    ukf1_ct.turn_rate_rad_per_sec = turn_rate_rad_per_sec;

    [snaps_R1, ft1] = imm_tracker(detList_R1, ukf1_cv, ukf1_ct, ...
        params, n_frames, true_track, t1_grid);

    % R2参数配置
    params_r2 = params;
    params_r2.gate_sigma = GATE_SIGMA_R2;
    params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
    params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    if UKF_Q_SCALE_R2 > 0
        params_r2.ukf_Q_scale = UKF_Q_SCALE_R2;
    else
        params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    end
    params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    params_r2.tracker_M = TRACKER_M_R2; params_r2.tracker_N = TRACKER_N_R2;
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;

    ukf2_cv = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    ukf2_ct = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    ukf2_ct.model_type = 'CT';
    ukf2_ct.turn_rate_rad_per_sec = turn_rate_rad_per_sec;

    [snaps_R2, ft2] = imm_tracker(detList_R2, ukf2_cv, ukf2_ct, ...
        params_r2, n_frames, true_track, t2_grid);

    % ---- 覆盖IMM参数到imm_tracker？不，imm_tracker内部硬编码了Pi和mu初始值
    % 暂时接受默认值，后续如需调参需修改imm_tracker.m

    rmse.ukf_R1(mc) = mc_rmse_tracks(snaps_R1, true_track, t1_grid, n_frames);
    rmse.ukf_R2(mc) = mc_rmse_tracks(snaps_R2, true_track, t2_grid, n_frames);
    imp_ukf_R1(mc) = (1 - rmse.ukf_R1(mc)/rmse.cal_R1(mc)) * 100;
    imp_ukf_R2(mc) = (1 - rmse.ukf_R2(mc)/rmse.cal_R2(mc)) * 100;

    [assoc_R1(mc), nis_mean_R1(mc), nis_gate_R1(mc), n_assoc1, n_pred1, init_frame_R1(mc)] = ...
        mc_diag(snaps_R1, n_frames);
    [assoc_R2(mc), nis_mean_R2(mc), nis_gate_R2(mc), n_assoc2, n_pred2, init_frame_R2(mc)] = ...
        mc_diag(snaps_R2, n_frames);

    if isfield(ft1, 'mu_history')
        mh1 = ft1.mu_history;
        mu_ct_avg_R1(mc) = mean(mh1(:,2)) * 100;
        if ~isempty(turn_frames)
            tf = turn_frames(turn_frames <= size(mh1,1));
            mu_ct_turn_R1(mc) = mean(mh1(tf,2)) * 100;
        end
        mu_ct_dom_R1(mc) = sum(mh1(:,2) > 0.5);
    end
    if isfield(ft2, 'mu_history')
        mh2 = ft2.mu_history;
        mu_ct_avg_R2(mc) = mean(mh2(:,2)) * 100;
        if ~isempty(turn_frames)
            tf = turn_frames(turn_frames <= size(mh2,1));
            mu_ct_turn_R2(mc) = mean(mh2(tf,2)) * 100;
        end
        mu_ct_dom_R2(mc) = sum(mh2(:,2) > 0.5);
    end

    %% ---------- Phase 6: 时间对齐 ----------
    aligned_R2 = time_align_tracks(snaps_R2, params);
    rmse.ukf_R2_aligned(mc) = mc_rmse_tracks(aligned_R2, true_track, t1_grid, n_frames);

    %% ---------- Phase 7: 融合 ----------
    mp = struct('R1_track_id', 1, 'R2_track_id', 1, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);
    methods = {'SCC', 'BC', 'CI', 'FCI'};
    for m = 1:4
        af = run_track_fusion(mp, snaps_R1, aligned_R2, params, methods{m});
        rmse.fus(mc, m) = mc_rmse_fusion(af, true_track, t1_grid, n_frames);
    end
    [best_val, best_m] = min(rmse.fus(mc, :));
    rmse.fus_best(mc) = best_val;
    fus_best_method{mc} = methods{best_m};
    imp_fus_vs_R1(mc) = (1 - best_val/rmse.ukf_R1(mc)) * 100;
    imp_fus_vs_R2(mc) = (1 - best_val/rmse.ukf_R2_aligned(mc)) * 100;

    segs1 = mc_extract_segs(snaps_R1, n_frames);
    segs2 = mc_extract_segs(snaps_R2, n_frames);
    af_best = run_track_fusion(mp, snaps_R1, aligned_R2, params, methods{best_m});
    segs_f = mc_extract_fusion_segs(af_best, n_frames);
    mtl_R1(mc) = mc_compute_mtl(segs1); mtl_R2(mc) = mc_compute_mtl(segs2);
    mtl_fus(mc) = mc_compute_mtl(segs_f);
    brk_R1(mc) = max(0, size(segs1,1)-1); brk_R2(mc) = max(0, size(segs2,1)-1);
    brk_fus(mc) = max(0, size(segs_f,1)-1);
    seg_count_R1(mc) = size(segs1,1); seg_count_R2(mc) = size(segs2,1);
    seg_count_fus(mc) = size(segs_f,1);
    seg_info{mc} = struct('R1', segs1, 'R2', segs2, 'FUS', segs_f);

    % 坏种子
    if rmse.ukf_R1(mc) > 30 || rmse.ukf_R2(mc) > 30
        bad_seed(mc) = 1;
        bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', rmse.ukf_R1(mc), rmse.ukf_R2(mc));
    elseif imp_ukf_R1(mc) < -50 || imp_ukf_R2(mc) < -50
        bad_seed(mc) = 1;
        bad_reason{mc} = sprintf('DEGRADED R1=%+.0f%% R2=%+.0f%%', imp_ukf_R1(mc), imp_ukf_R2(mc));
    end

    %% ---- 逐种子输出 ----
    fprintf('MC#%d/%d s=%d: ', mc, N_MC, seed);
    fprintf('R1=%.1fkm(%+.0f%%) a=%.0f%% NIS=%.1f(%.0f%%) | ', ...
        rmse.ukf_R1(mc), imp_ukf_R1(mc), assoc_R1(mc), nis_mean_R1(mc), nis_gate_R1(mc));
    fprintf('R2=%.1fkm(%+.0f%%) a=%.0f%% NIS=%.1f(%.0f%%) | ', ...
        rmse.ukf_R2(mc), imp_ukf_R2(mc), assoc_R2(mc), nis_mean_R2(mc), nis_gate_R2(mc));
    fprintf('CT R1=%.0f%% R2=%.0f%% | Fus=%.1fkm(%s)', ...
        mu_ct_turn_R1(mc), mu_ct_turn_R2(mc), rmse.fus_best(mc), fus_best_method{mc});
    if bad_seed(mc), fprintf(' ***BAD***'); end
    fprintf('\n');
end

elapsed = toc;
close all;

%% ========================================================================
%% 汇总统计
%% ========================================================================
fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║       回头弯180° %d次MC统计 (%.0fs)                        ║\n', N_MC, elapsed);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

fprintf('║  ─── RMSE (km) ───                                          ║\n');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
mc_print('原始点迹 R1', rmse.raw_R1);
mc_print('原始点迹 R2', rmse.raw_R2);
mc_print('校准后 R1', rmse.cal_R1);
mc_print('校准后 R2', rmse.cal_R2);
mc_print('UKF R1', rmse.ukf_R1);
mc_print('UKF R2(对齐)', rmse.ukf_R2_aligned);
mc_print('融合 SCC', rmse.fus(:,1));
mc_print('融合 BC', rmse.fus(:,2));
mc_print('融合 CI', rmse.fus(:,3));
mc_print('融合 FCI', rmse.fus(:,4));
mc_print('融合最优', rmse.fus_best);

fprintf('║                                                              ║\n');
fprintf('║  ─── 改善率 (%%) ───                                        ║\n');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
cal_imp_R1 = (1 - rmse.cal_R1 ./ rmse.raw_R1) * 100;
cal_imp_R2 = (1 - rmse.cal_R2 ./ rmse.raw_R2) * 100;
mc_print('校准改善 R1', cal_imp_R1);
mc_print('校准改善 R2', cal_imp_R2);
mc_print('UKF改善 R1', imp_ukf_R1);
mc_print('UKF改善 R2', imp_ukf_R2);
mc_print('融合 vs R1', imp_fus_vs_R1);
mc_print('融合 vs R2', imp_fus_vs_R2);

fprintf('║                                                              ║\n');
fprintf('║  ─── MTL + 断裂 ───                                         ║\n');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
mc_print('MTL R1', mtl_R1); mc_print('MTL R2', mtl_R2); mc_print('MTL 融合', mtl_fus);
mc_print('断裂 R1', brk_R1); mc_print('断裂 R2', brk_R2); mc_print('断裂 融合', brk_fus);

fprintf('║                                                              ║\n');
fprintf('║  ─── IMM 模型概率 (%%) ───                                   ║\n');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
mc_print('CT均值 R1', mu_ct_avg_R1); mc_print('CT均值 R2', mu_ct_avg_R2);
mc_print('CT转弯 R1', mu_ct_turn_R1); mc_print('CT转弯 R2', mu_ct_turn_R2);
mc_print('CT占优帧 R1', mu_ct_dom_R1); mc_print('CT占优帧 R2', mu_ct_dom_R2);

fprintf('║                                                              ║\n');
fprintf('║  ─── 关联诊断 ───                                            ║\n');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '指标','均值','std','中位','最小','最大');
fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', '─','─','─','─','─','─');
mc_print('关联率 R1(%)', assoc_R1); mc_print('关联率 R2(%)', assoc_R2);
mc_print('NIS均值 R1', nis_mean_R1); mc_print('NIS均值 R2', nis_mean_R2);
mc_print('NIS门内 R1(%)', nis_gate_R1); mc_print('NIS门内 R2(%)', nis_gate_R2);
mc_print('起始帧 R1', init_frame_R1); mc_print('起始帧 R2', init_frame_R2);

n_bad = sum(bad_seed);
fprintf('║                                                              ║\n');
fprintf('║  坏种子: %d/%d (%.1f%%)                                      ║\n', n_bad, N_MC, n_bad/N_MC*100);
if n_bad > 0
    for mc = 1:N_MC
        if bad_seed(mc)
            fprintf('║    seed=%d: %s  ║\n', SEED_BASE+mc-1, bad_reason{mc});
        end
    end
end

fprintf('║                                                              ║\n');
fprintf('║  最优融合算法分布:                                           ║\n');
for m = 1:4
    cnt = sum(strcmp(fus_best_method, methods{m}));
    fprintf('║    %s: %d/%d (%.0f%%)                                          ║\n', ...
        methods{m}, cnt, N_MC, cnt/N_MC*100);
end
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

%% ---- 保存 ----
if ~exist('results', 'dir'), mkdir('results'); end
outf = fullfile('results', sprintf('mc_turn180_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'rmse', 'mtl_R1', 'mtl_R2', 'mtl_fus', 'brk_R1', 'brk_R2', 'brk_fus', ...
    'imp_ukf_R1', 'imp_ukf_R2', 'imp_fus_vs_R1', 'imp_fus_vs_R2', ...
    'nis_mean_R1', 'nis_mean_R2', 'assoc_R1', 'assoc_R2', ...
    'mu_ct_avg_R1', 'mu_ct_avg_R2', 'mu_ct_turn_R1', 'mu_ct_turn_R2', ...
    'mu_ct_dom_R1', 'mu_ct_dom_R2', ...
    'fus_best_method', 'bad_seed', 'bad_reason', 'seg_info', 'N_MC', 'SEED_BASE');
fprintf('\n数据已保存: %s\n', outf);
fprintf('Done.\n');


%% ========================================================================
%% 工具函数（带mc_前缀避免与run_mc_turn.m冲突）
%% ========================================================================

function v = mc_rmse_detlist(detList, true_track, t_grid, n_frames, mode)
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
    v = mc_rms(errs);
end

function v = mc_rmse_tracks(snaps, true_track, t_grid, n_frames)
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
    v = mc_rms(errs);
end

function v = mc_rmse_fusion(snaps, true_track, t_grid, n_frames)
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
    v = mc_rms(errs);
end

function [assoc_rate, nis_mean, nis_gate, n_assoc, n_pred, init_frame] = mc_diag(snaps, n_frames)
    n_assoc = 0; n_pred = 0; init_frame = 0; nis_vals = [];
    for k = 1:n_frames
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};
        if trk.type == 1
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

function segs = mc_extract_segs(snaps, n_frames)
    segs = []; in_seg = false; seg_start = 0;
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
    if in_seg, segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1]; end
end

function segs = mc_extract_fusion_segs(snaps, n_frames)
    segs = []; in_seg = false; seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat), is_tracking = true; end
        end
        if is_tracking && ~in_seg
            in_seg = true; seg_start = k;
        elseif ~is_tracking && in_seg
            in_seg = false;
            segs(end+1, :) = [seg_start, k-1, k - seg_start];
        end
    end
    if in_seg, segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1]; end
end

function mtl = mc_compute_mtl(segs)
    if isempty(segs), mtl = 0; else, mtl = mean(segs(:,3)); end
end

function v = mc_rms(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function mc_print(label, vals)
    v = vals(~isnan(vals) & ~isinf(vals));
    if isempty(v)
        fprintf('║  %-24s %7s %7s %7s %7s %7s  ║\n', label, 'NaN', 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('║  %-24s %7.1f %7.1f %7.1f %7.1f %7.1f  ║\n', ...
            label, mean(v), std(v), median(v), min(v), max(v));
    end
end
