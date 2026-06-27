% =========================================================================
% run_mc_turn.m — 拐弯场景蒙特卡洛仿真 (N=50次)
% =========================================================================
% 不弹图窗、不保存.mat、控制台仅输出进度条和最终统计表。
% 随机种子从 1 遍历到 50，每次独立运行完整仿真流水线。
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

N_MC = 50;

% ---- 预分配结果数组 ----
rmse.raw_R1   = nan(N_MC, 1);
rmse.raw_R2   = nan(N_MC, 1);
rmse.cal_R1   = nan(N_MC, 1);
rmse.cal_R2   = nan(N_MC, 1);
rmse.ukf_R1   = nan(N_MC, 1);
rmse.ukf_R2   = nan(N_MC, 1);
rmse.ad_R1    = nan(N_MC, 1);
rmse.ad_R2    = nan(N_MC, 1);
rmse.fus_base = nan(N_MC, 1);   % 基础UKF四种融合中最优
rmse.fus_ad   = nan(N_MC, 1);   % 自适应UKF四种融合中最优
rmse.sgl_base = nan(N_MC, 1);   % 基础UKF单站中最优(R1/R2对齐后取min)
rmse.sgl_ad   = nan(N_MC, 1);   % 自适应UKF单站中最优

fprintf('========== 拐弯场景蒙特卡洛仿真 N=%d ==========\n', N_MC);
tic;

for mc = 1:N_MC
    fprintf('MC %2d/%-2d ', mc, N_MC);

    % 关闭上次可能残留的图窗（静默模式）
    close all;

    r = run_one(mc);

    rmse.raw_R1(mc)   = r.raw_R1;
    rmse.raw_R2(mc)   = r.raw_R2;
    rmse.cal_R1(mc)   = r.cal_R1;
    rmse.cal_R2(mc)   = r.cal_R2;
    rmse.ukf_R1(mc)   = r.ukf_R1;
    rmse.ukf_R2(mc)   = r.ukf_R2;
    rmse.ad_R1(mc)    = r.ad_R1;
    rmse.ad_R2(mc)    = r.ad_R2;
    rmse.fus_base(mc) = r.fus_base;
    rmse.fus_ad(mc)   = r.fus_ad;
    rmse.sgl_base(mc) = r.sgl_base;
    rmse.sgl_ad(mc)   = r.sgl_ad;

    elapsed = toc;
    fprintf('| %.0fs\n', elapsed);
end
close all;

% ---- 计算改善率（逐次算, 再求均值）----
imp_cal_R1  = (1 - rmse.cal_R1 ./ rmse.raw_R1)  * 100;
imp_cal_R2  = (1 - rmse.cal_R2 ./ rmse.raw_R2)  * 100;
imp_ukf_R1  = (1 - rmse.ukf_R1 ./ rmse.cal_R1)  * 100;
imp_ukf_R2  = (1 - rmse.ukf_R2 ./ rmse.cal_R2)  * 100;
imp_ad_R1   = (1 - rmse.ad_R1  ./ rmse.ukf_R1)  * 100;
imp_ad_R2   = (1 - rmse.ad_R2  ./ rmse.ukf_R2)  * 100;
imp_fus_base = (1 - rmse.fus_base ./ rmse.sgl_base) * 100;
imp_fus_ad   = (1 - rmse.fus_ad   ./ rmse.sgl_ad)   * 100;

% ---- 输出统计表 ----
fprintf('\n========== 蒙特卡洛 %d 次统计结果 ==========\n', N_MC);

fprintf('\n--- RMSE 绝对值 (km) ---\n');
fprintf('%-28s %8s %8s %8s %8s\n', '指标', '均值', '标准差', '最小', '最大');
fprintf('%-28s %8s %8s %8s %8s\n', '----------------------------', '------', '------', '------', '------');

print_row('原始点迹 R1', rmse.raw_R1);
print_row('原始点迹 R2', rmse.raw_R2);
print_row('校准后 R1',   rmse.cal_R1);
print_row('校准后 R2',   rmse.cal_R2);
print_row('基础UKF R1',  rmse.ukf_R1);
print_row('基础UKF R2',  rmse.ukf_R2);
print_row('自适应UKF R1', rmse.ad_R1);
print_row('自适应UKF R2', rmse.ad_R2);
print_row('基础融合最优', rmse.fus_base);
print_row('自适应融合最优', rmse.fus_ad);
print_row('基础单站最优(对齐)', rmse.sgl_base);
print_row('自适应单站最优(对齐)', rmse.sgl_ad);

fprintf('\n--- 阶段改善率 (%%) ---\n');
fprintf('%-28s %8s %8s %8s %8s\n', '指标', '均值', '标准差', '最小', '最大');
fprintf('%-28s %8s %8s %8s %8s\n', '----------------------------', '------', '------', '------', '------');

print_pct('校准改善 R1', imp_cal_R1);
print_pct('校准改善 R2', imp_cal_R2);
print_pct('UKF改善 R1',  imp_ukf_R1);
print_pct('UKF改善 R2',  imp_ukf_R2);
print_pct('自适应改善 R1', imp_ad_R1);
print_pct('自适应改善 R2', imp_ad_R2);
print_pct('融合改善(基础)', imp_fus_base);
print_pct('融合改善(自适应)', imp_fus_ad);

fprintf('\n总耗时: %.0f 秒\n', toc);


% =========================================================================
% run_one — 单次仿真运行, 返回各阶段RMSE
% =========================================================================
function r = run_one(seed)
    % ---- Phase 0: 场景初始化 ----
    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    [traj, turn_waypoints] = aircraft_trajectory_create('turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    % ---- Phase 1: ADS-B偏差标定 ----
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2;
    adsb_lon = T_adsb.Var3;

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

    % ---- Phase 2: 原始点迹生成 ----
    detRaw_R1 = cell(n_frames, 1);
    detRaw_R2 = cell(n_frames, 1);

    for k = 1:n_frames
        rng(params.random_seed + k);
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

        rng(params.random_seed + 10000 + k);
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

    % ---- Phase 4: 偏差校正 + 几何反解 ----
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
            [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r1(d).prange, dets_r1(d).paz, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets_r1(d).raw_lat = raw_lat;
            dets_r1(d).raw_lon = raw_lon;
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
            [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r2(d).prange, dets_r2(d).paz, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets_r2(d).raw_lat = raw_lat;
            dets_r2(d).raw_lon = raw_lon;
        end
        detList_R2{k} = dets_r2;
    end

    % ---- 计算点迹RMSE（原始 + 校准后）----
    r.raw_R1 = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'raw');
    r.raw_R2 = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'raw');
    r.cal_R1 = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'cal');
    r.cal_R2 = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'cal');

    % ---- Phase 5.1: 基础UKF ----
    params.ukf_range_std_m    = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
    params.gate_sigma      = params.radar1_gate_sigma;
    params.tracker_K_loss  = params.radar1_tracker_K_loss;

    ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

    params_r2 = params;
    params_r2.ukf_range_std_m    = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma      = params.radar2_gate_sigma;
    params_r2.ukf_Q_scale     = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std   = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std   = params.radar2_ukf_P_vel_std;
    params_r2.tracker_M       = 4;
    params_r2.tracker_N       = 8;
    params_r2.tracker_K_loss  = params.radar2_tracker_K_loss;

    ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    [trackSnapshots_R1, ~] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames);
    [trackSnapshots_R2, ~] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames);

    r.ukf_R1 = rmse_tracks(trackSnapshots_R1, true_track, t1_grid, n_frames);
    r.ukf_R2 = rmse_tracks(trackSnapshots_R2, true_track, t2_grid, n_frames);

    % ---- Phase 5.2: 机动自适应UKF ----
    rng(params.random_seed);
    ukf1_tpl_ad = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl_ad = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    [trackSnapshots_R1_ad, ~] = single_track_runner_adaptive(detList_R1, ukf1_tpl_ad, params, n_frames);
    [trackSnapshots_R2_ad, ~] = single_track_runner_adaptive(detList_R2, ukf2_tpl_ad, params_r2, n_frames);

    r.ad_R1 = rmse_tracks(trackSnapshots_R1_ad, true_track, t1_grid, n_frames);
    r.ad_R2 = rmse_tracks(trackSnapshots_R2_ad, true_track, t2_grid, n_frames);

    % ---- Phase 6: 时间对齐 ----
    aligned_R2    = time_align_tracks(trackSnapshots_R2,    params);
    aligned_R2_ad = time_align_tracks(trackSnapshots_R2_ad, params);

    % ---- Phase 7: 融合（基础UKF + 自适应UKF）----
    matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);
    method_names = {'SCC', 'BC', 'CI', 'FCI'};

    fus_base_rmses = nan(1, 4);
    fus_ad_rmses   = nan(1, 4);

    for m = 1:4
        snaps_base = run_track_fusion(matched_pair, trackSnapshots_R1, aligned_R2, params, method_names{m});
        snaps_ad   = run_track_fusion(matched_pair, trackSnapshots_R1_ad, aligned_R2_ad, params, method_names{m});
        fus_base_rmses(m) = rmse_fusion_snaps(snaps_base, true_track, t1_grid, n_frames);
        fus_ad_rmses(m)   = rmse_fusion_snaps(snaps_ad,   true_track, t1_grid, n_frames);
    end

    r.fus_base = min(fus_base_rmses);
    r.fus_ad   = min(fus_ad_rmses);

    % ---- 单站RMSE（对齐后，R2用aligned_R2）----
    sgl_R1_base = rmse_tracks(trackSnapshots_R1, true_track, t1_grid, n_frames);
    sgl_R2_base = rmse_tracks(aligned_R2,       true_track, t1_grid, n_frames);
    sgl_R1_ad   = rmse_tracks(trackSnapshots_R1_ad, true_track, t1_grid, n_frames);
    sgl_R2_ad   = rmse_tracks(aligned_R2_ad,       true_track, t1_grid, n_frames);

    r.sgl_base = min(sgl_R1_base, sgl_R2_base);
    r.sgl_ad   = min(sgl_R1_ad,   sgl_R2_ad);
end

% =========================================================================
% rmse_detlist — 从detList计算原始/校准后点迹RMSE
%   mode: 'raw'使用raw_lat/raw_lon, 'cal'使用lat/lon
% =========================================================================
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
    v = rms_km_val(errs);
end

% =========================================================================
% rmse_tracks — 从trackSnapshots计算航迹RMSE
% =========================================================================
function v = rmse_tracks(snaps, true_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        snap = snaps{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = rms_km_val(errs);
end

% =========================================================================
% rmse_fusion_snaps — 从融合快照计算RMSE
% =========================================================================
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
    v = rms_km_val(errs);
end

% =========================================================================
% 工具函数
% =========================================================================
function v = rms_km_val(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function print_row(label, vals)
    v = vals(~isnan(vals));
    if isempty(v)
        fprintf('%-28s %8s %8s %8s %8s\n', label, 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('%-28s %8.1f %8.1f %8.1f %8.1f\n', label, mean(v), std(v), min(v), max(v));
    end
end

function print_pct(label, vals)
    v = vals(~isnan(vals) & ~isinf(vals));
    if isempty(v)
        fprintf('%-28s %8s %8s %8s %8s\n', label, 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('%-28s %7.1f %8.1f %7.1f %7.1f\n', label, mean(v), std(v), min(v), max(v));
    end
end
