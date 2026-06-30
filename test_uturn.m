% test_uturn.m — 回头弯（180°）航迹IMM跟踪测试
% 拐弯率1°/s，转弯段180秒≈6帧，充分检验IMM的CT模型切换能力
clear; close all; clc; addpath(genpath('.'));

fprintf('===== 回头弯(180°) IMM跟踪测试 =====\n\n');

params = simulation_params();
rng(params.random_seed);

%% ===== 第1步：计算雷达覆盖内回头弯航路点 =====
fprintf('--- 回头弯航迹设计 ---\n');

speed_ms = params.aircraft_speed_ms;
dt = params.dt_sec;
omega_deg = 1.0;  % 1°/s 转弯率
omega_rad = omega_deg * pi / 180.0;
R_turn_m = speed_ms / omega_rad;  % 转弯半径 (m)
turn_dur_sec = 180.0 / omega_deg;  % 转弯时长 = 180s
arc_length_m = pi * R_turn_m;      % 半圆弧长

fprintf('  转弯半径: %.1f km, 转弯时长: %.0f s, 弧长: %.1f km\n', ...
    R_turn_m/1000, turn_dur_sec, arc_length_m/1000);

% 设计航路点（雷达覆盖区域内）
% R1(113.0°E,33.5°N), beam=92°/15°, range 1000-2000km
% R2(115.0°E,33.0°N), beam=91°/15°, range 1000-2000km
% 两波束在 ~127°E,33°N 附近交汇，选取该区域
W1 = [125.5, 33.0];  % 起点
bearing_in = 90.0;    % 入向：正东
turn_dir = +1;        % +1=左转(CCW), -1=右转(CW)

% 飞行路径: W1 东飞 → 回头弯(左转180°) → 西飞回
% 入弯前直线段长度（缩短以保证全程在覆盖内）
straight_approach_m = 120e3;  % 120km
straight_exit_m = 120e3;      % 120km

% 计算入弯点（entry tangent point）
[entry_lon, entry_lat] = haversine_forward_uturn(W1(1), W1(2), bearing_in, straight_approach_m);

% 转弯圆心（左转：圆心在入向左侧 = bearing_in + 90°）
[center_lon, center_lat] = haversine_forward_uturn(entry_lon, entry_lat, ...
    bearing_in + 90.0 * turn_dir, R_turn_m);

% 出弯点（圆心对面，180°）
exit_bearing_from_center = bearing_in - 90.0 * turn_dir;  % 圆心→出弯点的方位
[exit_lon, exit_lat] = haversine_forward_uturn(center_lon, center_lat, ...
    exit_bearing_from_center, R_turn_m);

% 出弯后终点
bearing_out = mod(bearing_in + 180.0 * turn_dir, 360);
[W3_lon, W3_lat] = haversine_forward_uturn(exit_lon, exit_lat, bearing_out, straight_exit_m);

fprintf('  W1 (起点):     (%.4f°E, %.4f°N)\n', W1(1), W1(2));
fprintf('  入弯点:        (%.4f°E, %.4f°N)\n', entry_lon, entry_lat);
fprintf('  转弯圆心:      (%.4f°E, %.4f°N)\n', center_lon, center_lat);
fprintf('  出弯点:        (%.4f°E, %.4f°N)\n', exit_lon, exit_lat);
fprintf('  W3 (终点):     (%.4f°E, %.4f°N)\n', W3_lon, W3_lat);
fprintf('  入向: %.1f° → 出向: %.1f° (拐角: 180°)\n', bearing_in, bearing_out);

%% ===== 第2步：构建航迹结构体 =====
fprintf('\n--- 构建航迹结构体 ---\n');

segments = {};
t_cum = 0;

% 航段1：入弯直线
dur1 = straight_approach_m / speed_ms;
segments{1} = struct('start', [W1(1), W1(2)], ...
    'end', [entry_lon, entry_lat], ...
    'lon_rate', (entry_lon - W1(1)) / dur1, ...
    'lat_rate', (entry_lat - W1(2)) / dur1, ...
    'dur', dur1, 't_start', 0);
t_cum = t_cum + dur1;

% 航段2：180°半圆（按dt_sec分小段打包）
% 每 arc_step=1s 采样一个弧点
arc_step = 1.0;
n_arc_pts = floor(turn_dur_sec / arc_step);
arc_pts = zeros(n_arc_pts, 2);  % [lon, lat]
prev_lon = entry_lon;
prev_lat = entry_lat;
for i = 1:n_arc_pts
    t_arc = i * arc_step;
    hdg = bearing_in + turn_dir * omega_deg * t_arc;  % 当前航向
    if hdg >= 360, hdg = hdg - 360; end
    if hdg < 0, hdg = hdg + 360; end
    [arc_pts(i,1), arc_pts(i,2)] = haversine_forward_uturn(prev_lon, prev_lat, ...
        hdg, speed_ms * arc_step);
    prev_lon = arc_pts(i,1);
    prev_lat = arc_pts(i,2);
end

% 按dt秒一组打包
pts_per_seg = round(dt / arc_step);
seg_start_lon = entry_lon;
seg_start_lat = entry_lat;
for i_start = 1:pts_per_seg:n_arc_pts
    i_end = min(i_start + pts_per_seg - 1, n_arc_pts);
    seg_end_lon = arc_pts(i_end, 1);
    seg_end_lat = arc_pts(i_end, 2);
    seg_dur = (i_end - i_start + 1) * arc_step;
    segments{end+1} = struct('start', [seg_start_lon, seg_start_lat], ...
        'end', [seg_end_lon, seg_end_lat], ...
        'lon_rate', (seg_end_lon - seg_start_lon) / seg_dur, ...
        'lat_rate', (seg_end_lat - seg_start_lat) / seg_dur, ...
        'dur', seg_dur, 't_start', t_cum);
    t_cum = t_cum + seg_dur;
    seg_start_lon = seg_end_lon;
    seg_start_lat = seg_end_lat;
end

% 航段3：出弯直线
dur3 = straight_exit_m / speed_ms;
segments{end+1} = struct('start', [exit_lon, exit_lat], ...
    'end', [W3_lon, W3_lat], ...
    'lon_rate', (W3_lon - exit_lon) / dur3, ...
    'lat_rate', (W3_lat - exit_lat) / dur3, ...
    'dur', dur3, 't_start', t_cum);
t_cum = t_cum + dur3;

traj.speed = speed_ms;
traj.dt_sec = dt;
traj.segments = segments';
traj.waypoints = [W1(1), W1(2); NaN, NaN; W3_lon, W3_lat];  % 中间点是虚拟拐点
traj.duration_sec = t_cum;
traj.n_segments = length(segments);
traj.time_array = 0:dt:t_cum;
traj.n_steps = length(traj.time_array);

fprintf('  总航程: %.0f km, 总时长: %.0f s, 帧数: %d\n', ...
    (straight_approach_m + arc_length_m + straight_exit_m)/1000, t_cum, traj.n_steps);

%% ===== 第3步：覆盖检查 =====
true_track = aircraft_trajectory_interpolate('generate', traj);
n_in_r1 = 0; n_in_r2 = 0;
for i = 1:size(true_track, 1)
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track(i,1), true_track(i,2), params.radar1_beam_center_deg, params);
    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track(i,1), true_track(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1 = n_in_r1 + 1; end
    if in2, n_in_r2 = n_in_r2 + 1; end
end
fprintf('  覆盖: R1=%d/%d, R2=%d/%d\n', n_in_r1, size(true_track,1), n_in_r2, size(true_track,1));

if n_in_r1 == 0 || n_in_r2 == 0
    error('航迹不在雷达覆盖范围内！');
end

%% ===== 第4步：时间网格 =====
t1_grid = params.time_offset_radar1_sec : dt : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : dt : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('  仿真帧数: %d\n', n_frames);

%% ===== 第5步：ADS-B标校 + 点迹生成 + 偏差校正（精简版，复用run_simulation_turn逻辑）=====
fprintf('\n--- 点迹生成与标校 ---\n');
rng(params.random_seed);

% ADS-B 标校
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

% 生成点迹
detList_R1 = cell(n_frames, 1); detList_R2 = cell(n_frames, 1);
for k = 1:n_frames
    rng(params.random_seed + k);
    [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
    dets = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, ...
        pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    for d = 1:length(dets)
        dets(d).aircraft_id = 1;
        Rgc = dets(d).prange - dr1_est; azc = dets(d).paz - da1_est;
        dets(d).drange = Rgc; dets(d).daz = azc;
        dets(d).range_meas = Rgc; dets(d).azimuth_meas = azc;
        if ~(isfield(dets(d), 'lat') && ~isnan(dets(d).lat))
            [~, le, lo] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets(d).lat = le; dets(d).lon = lo;
        end
    end
    detList_R1{k} = dets;

    rng(params.random_seed + 10000 + k);
    [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
    dets2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, ...
        pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
        params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
        params.radar2_beam_center_deg, params, ...
        params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
    for d = 1:length(dets2)
        dets2(d).aircraft_id = 1;
        Rgc = dets2(d).prange - dr2_est; azc = dets2(d).paz - da2_est;
        dets2(d).drange = Rgc; dets2(d).daz = azc;
        dets2(d).range_meas = Rgc; dets2(d).azimuth_meas = azc;
        if ~(isfield(dets2(d), 'lat') && ~isnan(dets2(d).lat))
            [~, le, lo] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets2(d).lat = le; dets2(d).lon = lo;
        end
    end
    detList_R2{k} = dets2;
end
fprintf('  点迹生成完成: R1=%d帧, R2=%d帧\n', n_frames, n_frames);

%% ===== 第6步：IMM跟踪 =====
fprintf('\n========== IMM跟踪 (CV+CT, 回头弯180°) ==========\n');

% 转弯率（已知真值：左转 +1°/s）
turn_rate_rad_per_sec = turn_dir * omega_deg * pi / 180.0;

% R1 UKF配置
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
params.gate_sigma = params.radar1_gate_sigma;
params.gate_vr_ms = params.radar1_gate_vr_ms;
params.tracker_K_loss = params.radar1_tracker_K_loss;

ukf1_cv = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, dt);
ukf1_ct = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, dt);
ukf1_ct.model_type = 'CT';
ukf1_ct.turn_rate_rad_per_sec = turn_rate_rad_per_sec;

% R2 UKF配置
pr2 = params;
pr2.ukf_range_std_m = params.radar2_range_noise_std_m;
pr2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
pr2.gate_sigma = params.radar2_gate_sigma;
pr2.gate_vr_ms = params.radar2_gate_vr_ms;
pr2.ukf_Q_scale = params.radar2_ukf_Q_scale;
pr2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
pr2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
pr2.tracker_M = 4; pr2.tracker_N = 8;
pr2.tracker_K_loss = params.radar2_tracker_K_loss;

ukf2_cv = ukf_jichu('create', pr2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, dt);
ukf2_ct = ukf_jichu('create', pr2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, dt);
ukf2_ct.model_type = 'CT';
ukf2_ct.turn_rate_rad_per_sec = turn_rate_rad_per_sec;

% IMM跟踪
fprintf('  ω=%.4f rad/s (%.1f°/s 左转)\n', turn_rate_rad_per_sec, omega_deg);
tic;
[snap_R1, ft1] = imm_tracker(detList_R1, ukf1_cv, ukf1_ct, params, n_frames, true_track, t1_grid);
[snap_R2, ft2] = imm_tracker(detList_R2, ukf2_cv, ukf2_ct, pr2, n_frames, true_track, t2_grid);
et = toc;

fprintf('R1: type=%s quality=%d life=%d\n', getTypeStr(ft1.type), ft1.quality, ft1.life);
fprintf('R2: type=%s quality=%d life=%d\n', getTypeStr(ft2.type), ft2.quality, ft2.life);

%% ===== 第7步：RMSE统计 + 模型概率诊断 =====
fprintf('\n--- UKF RMSE ---\n');

errs1 = []; errs2 = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    if ~isempty(snap_R1{k}) && ~isempty(snap_R1{k}.trackList)
        tr = snap_R1{k}.trackList{1};
        if tr.type ~= 7 && ~isnan(tr.lat)
            errs1(end+1) = sphere_utils_haversine_distance(tr.lon, tr.lat, tl, tb) / 1000;
        end
    end
end
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    if ~isempty(snap_R2{k}) && ~isempty(snap_R2{k}.trackList)
        tr = snap_R2{k}.trackList{1};
        if tr.type ~= 7 && ~isnan(tr.lat)
            errs2(end+1) = sphere_utils_haversine_distance(tr.lon, tr.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 IMM RMSE: median=%.1f mean=%.1f km (n=%d)\n', median(errs1), mean(errs1), length(errs1));
fprintf('R2 IMM RMSE: median=%.1f mean=%.1f km (n=%d)\n', median(errs2), mean(errs2), length(errs2));

%% ===== 第8步：模型概率时间序列 =====
fprintf('\n--- CT模型概率诊断 ---\n');

mu_hist1 = ft1.mu_history;
mu_hist2 = ft2.mu_history;

% 找出转弯帧范围
turn_start_sec = straight_approach_m / speed_ms;
turn_end_sec = turn_start_sec + turn_dur_sec;
turn_start_frame = find(t1_grid >= turn_start_sec, 1);
turn_end_frame = find(t1_grid >= turn_end_sec, 1);
if isempty(turn_start_frame), turn_start_frame = 1; end
if isempty(turn_end_frame), turn_end_frame = n_frames; end

fprintf('  转弯段: t=%.0f~%.0fs, 帧#%d~#%d (共%d帧)\n', ...
    turn_start_sec, turn_end_sec, turn_start_frame, turn_end_frame, ...
    turn_end_frame - turn_start_frame + 1);

% 全局统计
fprintf('  R1 CT概率: 平均=%.0f%%, 最大值=%.0f%%\n', mean(mu_hist1(:,2))*100, max(mu_hist1(:,2))*100);
fprintf('  R2 CT概率: 平均=%.0f%%, 最大值=%.0f%%\n', mean(mu_hist2(:,2))*100, max(mu_hist2(:,2))*100);

% 转弯段统计
if turn_end_frame >= turn_start_frame
    mu1_turn = mu_hist1(turn_start_frame:min(turn_end_frame,end), 2);
    mu2_turn = mu_hist2(turn_start_frame:min(turn_end_frame,end), 2);
    fprintf('  R1 转弯段CT概率: 平均=%.0f%%, 最大=%.0f%%\n', mean(mu1_turn)*100, max(mu1_turn)*100);
    fprintf('  R2 转弯段CT概率: 平均=%.0f%%, 最大=%.0f%%\n', mean(mu2_turn)*100, max(mu2_turn)*100);
end

%% ===== 第9步：打印模型概率时间序列（转弯段附近） =====
fprintf('\n--- 模型概率逐帧（转弯段前后） ---\n');
fprintf('%-5s %-12s %-12s %-12s %-12s\n', '帧', 'R1 μ_CV', 'R1 μ_CT', 'R2 μ_CV', 'R2 μ_CT');
fprintf('%-5s %-12s %-12s %-12s %-12s\n', '---', '------', '------', '------', '------');
% 打印转弯前后各3帧
f1 = max(1, turn_start_frame - 3);
fe = min(n_frames, turn_end_frame + 3);
for k = f1:fe
    marker = '';
    if k >= turn_start_frame && k <= turn_end_frame
        marker = ' ←转弯';
    end
    fprintf('%-5d %11.1f%% %11.1f%% %11.1f%% %11.1f%%%s\n', ...
        k, mu_hist1(k,1)*100, mu_hist1(k,2)*100, ...
        mu_hist2(k,1)*100, mu_hist2(k,2)*100, marker);
end

%% ===== 第10步：融合 =====
fprintf('\n--- 融合 ---\n');
aR2 = time_align_tracks(snap_R2, pr2);
mp = struct('R1_track_id', 1, 'R2_track_id', 1, 'match_count', 0, ...
    'coexist_count', 0, 'match_ratio', 1, 'mean_dist_km', 0, 'quality', 100);
methods = {'SCC', 'BC', 'CI', 'FCI'};
f_errs = nan(1, 4);
for m = 1:4
    af = run_track_fusion(mp, snap_R1, aR2, params, methods{m});
    e = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        if ~isempty(af{k}) && ~isempty(af{k}.trackList)
            tr = af{k}.trackList{1};
            if ~isnan(tr.lat)
                e(end+1) = sphere_utils_haversine_distance(tr.lon, tr.lat, tl, tb) / 1000;
            end
        end
    end
    f_errs(m) = rms_km(e);
    fprintf('  %s: RMSE=%.1f km\n', methods{m}, f_errs(m));
end

%% ===== 第11步：可视化 =====
fprintf('\n--- 可视化 ---\n');
if ~exist('results', 'dir'), mkdir('results'); end

% 图1：航迹总览（2×2子图）
figure('Position', [100 100 1400 900]);

% 左上：地图上的航迹和雷达覆盖
subplot(2,2,1); hold on;
plot(true_track(:,1), true_track(:,2), 'k-', 'LineWidth', 1.5);
plot(W1(1), W1(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(W3_lon, W3_lat, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(entry_lon, entry_lat, 'b^', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
plot(exit_lon, exit_lat, 'bv', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
plot(center_lon, center_lat, 'm+', 'MarkerSize', 10);
xlabel('Longitude (degE)'); ylabel('Latitude (degN)');
title('U-turn Trajectory (180deg, 1deg/s)');
legend('Truth', 'Start W1', 'End W3', 'Entry', 'Exit', 'Turn Center', 'Location', 'best');
axis equal; grid on;

% 右上：R1 IMM跟踪 vs 真值
subplot(2,2,2); hold on;
plot(true_track(:,1), true_track(:,2), 'k-', 'LineWidth', 1.5);
r1x = []; r1y = [];
for k = 1:n_frames
    if ~isempty(snap_R1{k}) && ~isempty(snap_R1{k}.trackList)
        tr = snap_R1{k}.trackList{1};
        if tr.type ~= 7 && ~isnan(tr.lat)
            r1x(end+1) = tr.lon; r1y(end+1) = tr.lat;
        end
    end
end
plot(r1x, r1y, 'b.--', 'MarkerSize', 8);
xlabel('Longitude (degE)'); ylabel('Latitude (degN)');
title(sprintf('R1 IMM Track (RMSE=%.1f km)', rms_km(errs1)));
legend('Truth', 'IMM', 'Location', 'best');
axis equal; grid on;

% 下半：模型概率 + 瞬时误差（对齐有效帧）
subplot(2,2,[3 4]); hold on;
% 找出各帧对应的误差（用NaN填充无效帧）
err1_aligned = nan(n_frames, 1);
err2_aligned = nan(n_frames, 1);
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    if ~isempty(snap_R1{k}) && ~isempty(snap_R1{k}.trackList)
        tr = snap_R1{k}.trackList{1};
        if tr.type ~= 7 && ~isnan(tr.lat)
            err1_aligned(k) = sphere_utils_haversine_distance(tr.lon, tr.lat, tl, tb) / 1000;
        end
    end
    if ~isempty(snap_R2{k}) && ~isempty(snap_R2{k}.trackList)
        tr = snap_R2{k}.trackList{1};
        if tr.type ~= 7 && ~isnan(tr.lat)
            err2_aligned(k) = sphere_utils_haversine_distance(tr.lon, tr.lat, tl, tb) / 1000;
        end
    end
end

tt_frame = 1:n_frames;
yyaxis left;
plot(tt_frame, mu_hist1(:,2)*100, 'b-', 'LineWidth', 1.5);
plot(tt_frame, mu_hist2(:,2)*100, 'r--', 'LineWidth', 1.5);
ylabel('CT model probability (%)');
ylim([0 105]);
xline([turn_start_frame turn_end_frame], '--k', {'Turn Start', 'Turn End'});
yyaxis right;
plot(tt_frame, err1_aligned, 'b.-', 'MarkerSize', 4);
plot(tt_frame, err2_aligned, 'r.--', 'MarkerSize', 4);
ylabel('Instant error (km)');
xlabel('Frame');
title('CT Model Probability vs Instant Error');
legend('R1 CT prob', 'R2 CT prob', 'R1 error', 'R2 error', 'Location', 'best');
grid on;

saveas(gcf, 'results/fig_uturn_overview.png');
fprintf('  图表已保存: results/fig_uturn_overview.png\n');

fprintf('\n========== 完成 ==========\n');
fprintf('  总耗时: %.0fs\n', et);

%% ===== 辅助函数 =====
function s = getTypeStr(t)
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

function [lon1, lat1] = haversine_forward_uturn(lon0, lat0, bearing, dist_m)
    R = 6371000.0;
    lat0_rad = lat0 * pi / 180.0;
    lon0_rad = lon0 * pi / 180.0;
    bearing_rad = bearing * pi / 180.0;
    dR = dist_m / R;
    lat1_rad = asin(sin(lat0_rad) * cos(dR) + cos(lat0_rad) * sin(dR) * cos(bearing_rad));
    lon1_rad = lon0_rad + atan2(sin(bearing_rad) * sin(dR) * cos(lat0_rad), ...
        cos(dR) - sin(lat0_rad) * sin(lat1_rad));
    lat1 = lat1_rad * 180.0 / pi;
    lon1 = lon1_rad * 180.0 / pi;
end
