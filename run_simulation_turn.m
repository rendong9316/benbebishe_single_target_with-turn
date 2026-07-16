% =========================================================================
% run_simulation_turn.m — 双基地OTH-SWR单目标渐进拐弯三体制对比仿真主程序
% =========================================================================
% 【程序定位】
%   本程序是本仿真系统的拐弯场景主入口。与 run_simulation.m（直线航迹）
%   不同，本程序专门验证渐进拐弯机动场景下三种 UKF 后端的性能对比：
%   jichu (CV-UKF) / zishiying (自适应UKF) / imm (CV+CT双模型IMM)
%
% 【渐进拐弯航迹设计】
%   三个航路点，以民航标准 1°/s 转弯率渐进转弯（非突变）：
%     W1: (126.6685°E, 32.2184°N) — 起点
%     W2: (128.2501°E, 31.0887°N) — 拐弯顶点
%     W3: (132.0502°E, 31.4379°N) — 终点
%   转弯模型：协调转弯 R = v/ω，转弯提前量 d = R·tan(θ/2)
%   航迹结构：直线入弯 → 圆弧转弯（1°/s）→ 直线出弯
%
% 【三体制 UKF 后端】
%   ukf_jichu     — 基础 CV-UKF，固定 Q
%   ukf_zishiying — CV-UKF + 模糊自适应 Q + 机动检测
%   ukf_imm       — CV+CT 双模型 IMM-UKF + Pd-IPDA 似然
%
% 【9-Phase 流水线总览（三体制并行）】
%   Phase 0: 场景初始化（渐进拐弯航迹 + 覆盖检查 + 时间网格）
%   Phase 1: ADS-B系统偏差标定
%   Phase 2: 原始点迹生成
%   Phase 3: 时间对齐策略
%   Phase 4: 偏差校正 + 几何反解
%   Phase 5: 三体制航迹跟踪（jichu / zishiying / imm 并行）
%   Phase 6: 航迹级时间对齐（每体制独立）
%   Phase 7: 航迹融合（每体制独立 SCC/BC/CI/FCI）
%   Phase 8: 定量误差评估（每体制独立）
%   Phase 9: 可视化 + 数据保存（每体制各两图）
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

UKF_NAMES = {'jichu', 'zishiying', 'imm', '3in1-imm'};

%% ==================== Phase 0: 场景初始化（渐进拐弯目标） ====================
fprintf('========== Phase 0: 场景初始化 (渐进拐弯目标) ==========\n');

params = simulation_params();
rng(params.random_seed);

% ---- 渐进拐弯航迹生成 ----
[traj, turn_waypoints] = aircraft_trajectory_create('gradual_turn', params);
true_track = aircraft_trajectory_interpolate('generate', traj);
fprintf('真实航迹 (渐进拐弯): %d 点, 总时长 %.0f s, 速度 %.0f m/s\n', ...
    size(true_track,1), traj.duration_sec, params.aircraft_speed_ms);

% ---- 计算转弯方向和角速率 ----
bearing_in  = sphere_utils_azimuth(turn_waypoints(1,1), turn_waypoints(1,2), ...
    turn_waypoints(2,1), turn_waypoints(2,2));
bearing_out = sphere_utils_azimuth(turn_waypoints(2,1), turn_waypoints(2,2), ...
    turn_waypoints(3,1), turn_waypoints(3,2));
delta_hdg = bearing_out - bearing_in;
if delta_hdg > 180, delta_hdg = delta_hdg - 360;
elseif delta_hdg < -180, delta_hdg = delta_hdg + 360; end
turn_angle_deg = abs(delta_hdg);
turn_sign = sign(delta_hdg);
if turn_sign == 0, turn_sign = 1; end
turn_rate_rad_per_sec = turn_sign * 1.0 * pi / 180.0;
fprintf('  入向: %.1f°, 出向: %.1f°, 拐角: %.1f°, CT模型ω=%.4f rad/s\n', ...
    bearing_in, bearing_out, turn_angle_deg, turn_rate_rad_per_sec);

% ---- 覆盖检查 ----
n_in_r1 = 0; n_in_r2 = 0;
for i = 1:size(true_track, 1)
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track(i,1), true_track(i,2), params.radar1_beam_center_deg, params);
    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track(i,1), true_track(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1 = n_in_r1 + 1; end
    if in2, n_in_r2 = n_in_r2 + 1; end
end
fprintf('  在R1威力内: %d 点, 在R2威力内: %d 点 (共%d点)\n', ...
    n_in_r1, n_in_r2, size(true_track,1));

% ---- 时间网格 ----
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('仿真帧数: %d (dt=%.0fs)\n', n_frames, params.dt_sec);

% ---- 真值结构体 ----
tt = true_track;
truthTraj = struct('label', 'A', 'speed_ms', params.aircraft_speed_ms, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));

%% ==================== Phase 1: ADS-B系统偏差标定 ====================
fprintf('\n========== Phase 1: ADS-B系统偏差标定 ==========\n');

rng(params.random_seed);

fprintf('加载ADS-B合作目标: %s\n', params.adsb_csv_path);
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
fprintf('ADS-B标校点数: R1=%d, R2=%d\n', length(dr1_list), length(dr2_list));
fprintf('R1: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr1_est, params.radar1_range_bias_m, da1_est, params.radar1_azimuth_bias_deg);
fprintf('R2: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr2_est, params.radar2_range_bias_m, da2_est, params.radar2_azimuth_bias_deg);

%% ==================== Phase 2: 原始点迹生成 ====================
fprintf('\n========== Phase 2: 原始点迹生成 ==========\n');

detRaw_R1 = cell(n_frames, 1);
detRaw_R2 = cell(n_frames, 1);

rng(params.random_seed + 1e7);  % R1: 独立随机流
for k = 1:n_frames
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
end

rng(params.random_seed + 2e7);  % R2: 独立随机流
for k = 1:n_frames
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

fprintf('原始点迹生成完成: R1共%d帧, R2共%d帧\n', n_frames, n_frames);

%% ==================== Phase 3: 时间对齐策略 ====================
fprintf('\n========== Phase 3: 时间对齐策略 ==========\n');
fprintf('R1采样: 0s/30s/60s/...  R2采样: 13s/43s/73s/...  偏移=%ds\n', ...
    params.time_offset_radar2_sec);
fprintf('策略: 点迹不做对齐, 三部雷达各自在原时间网格上滤波跟踪\n');
fprintf('      航迹级对齐延后到 Phase 6 融合前, 用 CV 模型全状态外推\n');

%% ==================== Phase 4: 偏差校正 + 几何反解 ====================
fprintf('\n========== Phase 4: 偏差校正 ==========\n');

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

fprintf('偏差校正完成: R1=%d帧, R2=%d帧\n', n_frames, n_frames);

%% ---- 点迹定位RMSE统计 ----
fprintf('\n--- 点迹定位RMSE ---\n');

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R1{k})
        dp = detList_R1{k}(d);
        if ~dp.is_clutter && isfield(dp,'raw_lat') && ~isnan(dp.raw_lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.raw_lon, dp.raw_lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 原始点迹(含偏差)    RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R2{k})
        dp = detList_R2{k}(d);
        if ~dp.is_clutter && isfield(dp,'raw_lat') && ~isnan(dp.raw_lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.raw_lon, dp.raw_lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 原始点迹(含偏差)    RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R1{k})
        dp = detList_R1{k}(d);
        if ~dp.is_clutter && isfield(dp,'lat') && ~isnan(dp.lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 校准后点迹          RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R2{k})
        dp = detList_R2{k}(d);
        if ~dp.is_clutter && isfield(dp,'lat') && ~isnan(dp.lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 校准后点迹          RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

%% ==================== Phase 5: 三体制航迹跟踪 ====================
fprintf('\n========== Phase 5: 四体制航迹跟踪（jichu × zishiying × imm × 3in1-imm） ==========\n');

UKF_TYPES = UKF_NAMES;
N_UKF = 4;

% 预分配: ukf_snaps{u}{radar} = cell(n_frames,1), finalTrks{u} = struct
ukf_snaps_R1 = cell(N_UKF, 1);
ukf_snaps_R2 = cell(N_UKF, 1);
finalTrks = cell(N_UKF, 1);

for u = 1:N_UKF
    ukf_type = UKF_NAMES{u};

    % ---- R1 参数配置 ----
    pr1 = params;
    pr1.ukf_range_std_m = params.radar1_range_noise_std_m;
    pr1.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    pr1.ukf_Q_scale = params.radar1_ukf_Q_scale;
    pr1.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    pr1.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    pr1.gate_sigma = params.radar1_gate_sigma;
    pr1.gate_vr_ms = params.radar1_gate_vr_ms;
    pr1.tracker_K_loss = params.radar1_tracker_K_loss;
    pr1.multi_single_assoc_mode = 'oracle';  % 真值辅助关联
    if u >= 3, pr1.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

    % ---- 创建 UKF 模板 ----
    switch ukf_type
        case 'jichu'
            tpl1 = ukf_jichu('create', pr1, params.radar1_lon, ...
                params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        case 'zishiying'
            tpl1 = ukf_zishiying('create', pr1, params.radar1_lon, ...
                params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        case 'imm'
            pr1.imm_adapt_mode = 'fuzzy_only';
            tpl1 = ukf_imm('create', pr1, params.radar1_lon, ...
                params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
        case '3in1-imm'
            pr1.imm_adapt_mode = '3in1';
            tpl1 = ukf_imm('create', pr1, params.radar1_lon, ...
                params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    end

    % ---- R1 跟踪 ----
    fprintf('  [%s] R1 跟踪中...', ukf_type);
    [ukf_snaps_R1{u}, finalTrks{u}] = single_track_runner(detList_R1, tpl1, ...
        pr1, n_frames, true_track, t1_grid);
    fprintf('type=%s quality=%d life=%d\n', ...
        get_type_str(finalTrks{u}.type), finalTrks{u}.quality, finalTrks{u}.life);

    % ---- R2 参数配置 ----
    pr2 = params;
    pr2.ukf_range_std_m = params.radar2_range_noise_std_m;
    pr2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    pr2.gate_sigma = params.radar2_gate_sigma;
    pr2.gate_vr_ms = params.radar2_gate_vr_ms;
    pr2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    pr2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
    pr2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    pr2.tracker_M = 4;
    pr2.tracker_N = 8;
    pr2.tracker_K_loss = params.radar2_tracker_K_loss;
    pr2.multi_single_assoc_mode = 'oracle';  % 真值辅助关联
    if u >= 3, pr2.imm_turn_rate_rad_per_sec = turn_rate_rad_per_sec; end

    % ---- 创建 UKF 模板 ----
    switch ukf_type
        case 'jichu'
            tpl2 = ukf_jichu('create', pr2, params.radar2_lon, ...
                params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
        case 'zishiying'
            tpl2 = ukf_zishiying('create', pr2, params.radar2_lon, ...
                params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
        case 'imm'
            pr2.imm_adapt_mode = 'fuzzy_only';
            tpl2 = ukf_imm('create', pr2, params.radar2_lon, ...
                params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
        case '3in1-imm'
            pr2.imm_adapt_mode = '3in1';
            tpl2 = ukf_imm('create', pr2, params.radar2_lon, ...
                params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    end

    % ---- R2 跟踪 ----
    fprintf('  [%s] R2 跟踪中...', ukf_type);
    [ukf_snaps_R2{u}, finalTrks{u}] = single_track_runner(detList_R2, tpl2, ...
        pr2, n_frames, true_track, t2_grid);
    fprintf('type=%s quality=%d life=%d\n', ...
        get_type_str(finalTrks{u}.type), finalTrks{u}.quality, finalTrks{u}.life);

    % ---- 关联诊断 ----
    for radar_label = {'R1', 'R2'}
        snaps = ukf_snaps_R1{u};
        if strcmp(radar_label{1}, 'R2'), snaps = ukf_snaps_R2{u}; end
        n_assoc = 0; n_predict = 0; n_init = 0; n_lost = 0;
        init_frame = 0; nis_vals = [];
        for k = 1:length(snaps)
            if isempty(snaps{k}.trackList), continue; end
            trk = snaps{k}.trackList{1};
            if trk.type == 6
                n_init = n_init + 1;
            elseif trk.type == 1
                if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
                        isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                    n_assoc = n_assoc + 1;
                else
                    n_predict = n_predict + 1;
                end
                if isfield(trk, 'ukf') && ~isempty(trk.ukf) && isstruct(trk.ukf) && ...
                        isfield(trk.ukf, 'nis_history')
                    nis_vals = [nis_vals, trk.ukf.nis_history];
                end
            elseif trk.type == 7
                n_lost = n_lost + 1;
            end
            if init_frame == 0 && trk.type == 1, init_frame = k; end
        end
        n_tracked = n_assoc + n_predict;
        fprintf('    %s[%s]: 起始帧=%d | 关联=%d 纯预测=%d (关联率=%.0f%%) | 起始中=%d 丢失=%d\n', ...
            ukf_type, radar_label{1}, init_frame, n_assoc, n_predict, ...
            n_assoc/max(1,n_tracked)*100, n_init, n_lost);
        if ~isempty(nis_vals)
            nis_in_gate = sum(nis_vals < 4*2);
            fprintf('      NIS: 均值=%.2f 门内=%.0f%% (%d/%d)\n', ...
                mean(nis_vals), nis_in_gate/length(nis_vals)*100, nis_in_gate, length(nis_vals));
        end
    end

    % ---- IMM 模型概率诊断 ----
    if u >= 3 && isfield(finalTrks{u}, 'mu_history')
        fprintf('  [%s] IMM 模型概率诊断:\n', ukf_type);
        for r = 1:2
            mu_hist = finalTrks{u}.mu_history;
            n_ct_dominant = sum(mu_hist(:,2) > 0.5);
            avg_mu_ct = mean(mu_hist(:,2)) * 100;
            n_ct_high = sum(mu_hist(:,2) > 0.8);
            n_ct_low = sum(mu_hist(:,2) < 0.2);
            fprintf('    R%d: CT平均概率=%.0f%%, CT>80%%=%d, CT<20%%=%d, CT>50%%=%d/%d\n', ...
                r, avg_mu_ct, n_ct_high, n_ct_low, n_ct_dominant, n_frames);
            % 分布直方图
            bin_edges = [0:0.1:1.0];
            fprintf('      分布: ');
            for b = 1:(length(bin_edges)-1)
                cnt = sum(mu_hist(:,2) >= bin_edges(b) & mu_hist(:,2) < bin_edges(b+1));
                fprintf('[%.1f-%.1f]=%3d ', bin_edges(b), bin_edges(b+1), cnt);
            end
            fprintf('\n');
        end
    end
end

fprintf('\n');

%% ---- 三体制 UKF 滤波 RMSE ----
fprintf('\n--- 四体制 UKF 滤波 RMSE ---\n');
for u = 1:N_UKF
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        snap = ukf_snaps_R1{u}{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    fprintf('  %-12s R1滤波 RMSE: %6.1f km (n=%d)\n', UKF_NAMES{u}, rms_km(errs), length(errs));

    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
        snap = ukf_snaps_R2{u}{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    fprintf('  %-12s R2滤波 RMSE: %6.1f km (n=%d)\n', UKF_NAMES{u}, rms_km(errs), length(errs));
end

%% ==================== Phase 6+7+8: 每体制独立做对齐/融合/评估 ====================
fprintf('\n========== Phase 6-8: 每体制独立对齐+融合+评估 ==========\n');

method_names = {'SCC', 'BC', 'CI', 'FCI'};
N_FUS = length(method_names);

% 每体制的融合结果: fus_data{u}.snapshots{m}, .eval, .errors{...}
fus_data = cell(N_UKF, 1);

for u = 1:N_UKF
    ukf_type = UKF_NAMES{u};
    fprintf('\n--- [%s] Phase 6: 航迹级时间对齐 ---\n', ukf_type);

    aligned_R2_u = time_align_tracks(ukf_snaps_R2{u}, params);
    fprintf('  R2航迹时间对齐完成\n');

    fprintf('--- [%s] Phase 7: 航迹融合 (四种算法) ---\n', ukf_type);

    r1_id = 1; r2_id = 1;
    matched_pair = struct('R1_track_id', r1_id, 'R2_track_id', r2_id, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);

    all_fused_u = cell(N_FUS, 1);
    for m = 1:N_FUS
        fprintf('  [%s] 运行 %s 融合...\n', ukf_type, method_names{m});
        all_fused_u{m} = run_track_fusion(matched_pair, ...
            ukf_snaps_R1{u}, aligned_R2_u, params, method_names{m});
    end
    fprintf('  [%s] 融合完成: %d 种算法\n', ukf_type, N_FUS);

    % ---- 融合RMSE统计 ----
    fprintf('  [%s] --- 融合RMSE ---\n', ukf_type);
    fus_rmse_u = zeros(N_FUS, 1);
    for m = 1:N_FUS
        errs = [];
        snaps = all_fused_u{m};
        for k = 1:n_frames
            tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
            tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
            if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
                trk = snaps{k}.trackList{1};
                if ~isnan(trk.lat)
                    errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
                end
            end
        end
        fus_rmse_u(m) = rms_km(errs);
        fprintf('  [%s] %s 融合 RMSE: %6.1f km (n=%d)\n', ukf_type, method_names{m}, fus_rmse_u(m), length(errs));
    end

    errs_r1 = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        snap = ukf_snaps_R1{u}{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs_r1(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    fprintf('  [%s] R1 单站(对齐后) RMSE: %6.1f km\n', ukf_type, rms_km(errs_r1));

    errs_r2 = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        snap = aligned_R2_u{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs_r2(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    fprintf('  [%s] R2 单站(对齐后) RMSE: %6.1f km\n', ukf_type, rms_km(errs_r2));

    % ---- Phase 8: 定量误差评估 ----
    fprintf('  [%s] Phase 8: 定量误差评估\n', ukf_type);

    matcher_u = struct();
    matcher_u.matched_pairs = matched_pair;
    matcher_u.aligned_R2 = aligned_R2_u;
    matcher_u.r1_ids = r1_id;
    matcher_u.r2_ids = r2_id;

    r1_pos = nan(1, n_frames, 2);
    for k = 1:n_frames
        snap = ukf_snaps_R1{u}{k};
        if ~isempty(snap.trackList)
            for t = 1:length(snap.trackList)
                if snap.trackList{t}.id == r1_id
                    r1_pos(1, k, 1) = snap.trackList{t}.lon;
                    r1_pos(1, k, 2) = snap.trackList{t}.lat;
                    break;
                end
            end
        end
    end
    matcher_u.r1_pos = r1_pos;

    r2_pos = nan(1, n_frames, 2);
    for k = 1:n_frames
        snap = aligned_R2_u{k};
        if ~isempty(snap.trackList)
            for t = 1:length(snap.trackList)
                if snap.trackList{t}.id == r2_id
                    r2_pos(1, k, 1) = snap.trackList{t}.lon;
                    r2_pos(1, k, 2) = snap.trackList{t}.lat;
                    break;
                end
            end
        end
    end
    matcher_u.r2_pos = r2_pos;

    truthTrajs = {truthTraj};
    eval_u = evaluate_all('fusion', all_fused_u, method_names, ...
        matched_pair, ukf_snaps_R1{u}, ukf_snaps_R2{u}, ...
        truthTrajs, n_frames, params.dt_sec, matcher_u);

    rms_vals = arrayfun(@(x) x.s.rms, eval_u.overall(1:N_FUS));
    [best_fus_rmse, best_m] = min(rms_vals);
    r1_rmse = eval_u.overall(end-1).s.rms;
    r2_rmse = eval_u.overall(end).s.rms;
    fprintf('  [%s] 最佳融合算法: %s (RMSE=%.1fkm)\n', ukf_type, method_names{best_m}, best_fus_rmse);
    fprintf('  [%s] 融合 vs R1: %+.1f%%  vs R2: %+.1f%%\n', ...
        ukf_type, (1-best_fus_rmse/r1_rmse)*100, (1-best_fus_rmse/r2_rmse)*100);

    % 保存体制数据
    fus_data{u}.snapshots = all_fused_u;
    fus_data{u}.eval = eval_u;
    fus_data{u}.best_m = best_m;
    fus_data{u}.best_rmse = best_fus_rmse;
    fus_data{u}.aligned_R2 = aligned_R2_u;
    fus_data{u}.matcher = matcher_u;
end

%% ==================== Phase 9: 可视化 + 数据保存 ====================
fprintf('\n========== Phase 9: 可视化 ==========\n');
if ~exist('results', 'dir'), mkdir('results'); end

warn_state = warning('off', 'all');

for u = 1:N_UKF
    ukf_type = UKF_NAMES{u};
    fprintf('  [%s] 绘图中...', ukf_type);

    % 图1: 地图叠加（跟踪航迹）
    plot_results('single_track', true_track, detList_R1, detList_R2, ...
        ukf_snaps_R1{u}, ukf_snaps_R2{u}, params, 'results');

    % 图2: 融合可视化
    plot_results('single_fusion', true_track, ukf_snaps_R1{u}, ukf_snaps_R2{u}, ...
        fus_data{u}.snapshots, method_names, fus_data{u}.best_m, ...
        fus_data{u}.eval, truthTraj, params, 'results');

    fprintf('done\n');
end

warning(warn_state);

fprintf('\n========== Phase 9: 数据保存 ==========\n');

sysPara = struct(...
    'dt_sec', params.dt_sec, 'n_frames', n_frames, ...
    'R1_lon', params.radar1_lon, 'R1_lat', params.radar1_lat, ...
    'R1_tx_lon', params.radar1_tx_lon, 'R1_tx_lat', params.radar1_tx_lat, ...
    'R1_beam_center_deg', params.radar1_beam_center_deg, ...
    'R1_range_bias_m', params.radar1_range_bias_m, ...
    'R1_azimuth_bias_deg', params.radar1_azimuth_bias_deg, ...
    'R2_lon', params.radar2_lon, 'R2_lat', params.radar2_lat, ...
    'R2_tx_lon', params.radar2_tx_lon, 'R2_tx_lat', params.radar2_tx_lat, ...
    'R2_beam_center_deg', params.radar2_beam_center_deg, ...
    'R2_range_bias_m', params.radar2_range_bias_m, ...
    'R2_azimuth_bias_deg', params.radar2_azimuth_bias_deg, ...
    'beam_width_deg', params.beam_width_deg, ...
    'range_km', [params.range_min_km, params.range_max_km], ...
    'detection_probability', params.detection_probability, ...
    'false_alarm_rate', params.false_alarm_rate, ...
    'radar1_range_noise_m', params.radar1_range_noise_std_m, ...
    'radar1_az_noise_deg', params.radar1_azimuth_noise_std_deg, ...
    'radar2_range_noise_m', params.radar2_range_noise_std_m, ...
    'radar2_az_noise_deg', params.radar2_azimuth_noise_std_deg, ...
    'radial_vel_noise_std_ms', params.radial_vel_noise_std_ms, ...
    'random_seed', params.random_seed);

calibResult = struct(...
    'dr1_est', dr1_est, 'da1_est', da1_est, ...
    'dr2_est', dr2_est, 'da2_est', da2_est, ...
    'dr1_true', params.radar1_range_bias_m, 'da1_true', params.radar1_azimuth_bias_deg, ...
    'dr2_true', params.radar2_range_bias_m, 'da2_true', params.radar2_azimuth_bias_deg, ...
    'n_cal_R1', length(dr1_list), 'n_cal_R2', length(dr2_list));

ac_det_count_r1 = 0; ac_det_count_r2 = 0;
for k = 1:n_frames
    for d = 1:length(detList_R1{k})
        if ~detList_R1{k}(d).is_clutter, ac_det_count_r1 = ac_det_count_r1 + 1; end
    end
    for d = 1:length(detList_R2{k})
        if ~detList_R2{k}(d).is_clutter, ac_det_count_r2 = ac_det_count_r2 + 1; end
    end
end

% 保存三体制数据
R1_all = cell(N_UKF, 1);
R2_all = cell(N_UKF, 1);
for u = 1:N_UKF
    R1_all{u} = struct('detRaw', {detRaw_R1}, 'detList', {detList_R1}, ...
        'trackSnapshots', {ukf_snaps_R1{u}}, 'finalTrack', finalTrks{u}, ...
        'targetDetCount', ac_det_count_r1);
    R2_all{u} = struct('detRaw', {detRaw_R2}, 'detList', {detList_R2}, ...
        'trackSnapshots', {ukf_snaps_R2{u}}, 'finalTrack', finalTrks{u}, ...
        'targetDetCount', ac_det_count_r2);
end

outf = fullfile('results', sprintf('simulation_turn_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'sysPara', 'calibResult', 'truthTraj', 'R1_all', 'R2_all', 'params', ...
    'fus_data', 'UKF_NAMES', 'method_names', ...
    'turn_waypoints', 'turn_angle_deg', 'turn_rate_rad_per_sec');
fprintf('数据已保存: %s\n', outf);
fprintf('\nDone.\n');

% =========================================================================
% 内部函数
% =========================================================================

function s = get_type_str(t)
    switch t
        case 1, s = 'RELIABLE';
        case 2, s = 'MAINTAIN';
        case 6, s = 'TEMPORARY';
        case 7, s = 'HISTORY';
        otherwise, s = 'UNKNOWN';
    end
end

function v = rms_km(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end
