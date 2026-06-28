%% debug_nanyang_init.m — 诊断南阳起始系统
% 从种子24的仿真数据中提取前6帧，独立测试 trackStarter_logic
clear; close all;
addpath(genpath('.'));
addpath(genpath('nanyang'));

% 加载参数
params = simulation_params();
rng(params.random_seed);

% 设置 UKF 参数（与 run_simulation_turn.m Phase 5 一致）
params.ukf_range_std_m    = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
params.gate_sigma      = params.radar1_gate_sigma;
params.tracker_K_loss  = params.radar1_tracker_K_loss;

% 生成航迹和检测数据（前6帧）
traj = aircraft_trajectory_create('turn', params);
true_track = aircraft_trajectory_interpolate('generate', traj);
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
n_frames_test = min(15, length(t1_grid));

% 偏差（使用简化标定）
dr1_est = params.radar1_range_bias_m;
da1_est = params.radar1_azimuth_bias_deg;

% 生成前6帧检测
detList_R1 = cell(n_frames_test, 1);
for k = 1:n_frames_test
    rng(params.random_seed + k);
    [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
    dets = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, ...
        pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    % 偏差校正
    for d = 1:length(dets)
        dets(d).range_meas = dets(d).prange - dr1_est;
        dets(d).azimuth_meas = dets(d).paz - da1_est;
        [~, lat_e, lon_e] = bistatic_inverse_solver(dets(d).range_meas, dets(d).azimuth_meas, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat);
        dets(d).lat = lat_e;
        dets(d).lon = lon_e;
    end
    detList_R1{k} = dets;
    fprintf('Frame %d: %d detections\n', k, length(dets));
end

% 构建南阳 sysPara
ukf_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
sysPara = struct();
sysPara.T_inter = params.dt_sec;
sysPara.datenum = now;
sysPara.frameID = 1;
sysPara.deltaR = 10;   % km, 量测不确定性（用于归一化NN距离，非分辨率）
sysPara.deltaAz = 2;    % deg
sysPara.deltaV = 20;    % m/s
sysPara.tx_BLH = [ukf_tpl.tx_lat, ukf_tpl.tx_lon];
sysPara.rx_BLH = [ukf_tpl.radar_lat, ukf_tpl.radar_lon];
sysPara.f0 = 10.0;
sysPara.lambda = 30.0;
sysPara.prt = 0.05;
sysPara.fIndex = [0, 0];
sysPara.aIndex = [0, 360];
sysPara.rIndex = [0, 5000];
sysPara.ucMode = 9;
sysPara.tx_XOY = [0, 0];

% 使用固定基准时间（避免 now 在不同帧返回相同值的问题）
baseTime = datenum(2026, 6, 28, 0, 0, 0);

% 逐帧模拟 trackStarter_logic
tempTrackList = [];
M = 3;
N = 8;
fprintf('\nM=%d, N=%d\n', M, N);

for k = 1:n_frames_test
    curTime = baseTime + k * params.dt_sec / 86400.0;
    pointList = det2nanyang_point(detList_R1{k}, k, curTime);

    fprintf('\n=== Frame %d: %d new points, tempTrackList has %d total ===\n', ...
        k, length(pointList), length(tempTrackList));

    % 打印检测信息
    for p = 1:length(pointList)
        fprintf('  Point %d: frame=%d, prange=%.1f km, paz=%.2f deg, pvr=%.1f m/s, lat=%.4f, lon=%.4f\n', ...
            p, pointList(p).frameID, pointList(p).prange, pointList(p).paz, ...
            pointList(p).pvr, pointList(p).lat, pointList(p).lon);
    end

    % 调用 trackStarter_logic（关闭 polyfit 警告）
    warn_state = warning('off', 'MATLAB:polyfit:RepeatedPointsOrRescale');
    [tempTrackList, valid_tracks] = trackStarter_logic(...
        tempTrackList, pointList, sysPara, M, N);
    warning(warn_state);

    fprintf('  tempTrackList now has %d points\n', length(tempTrackList));
    if ~isempty(valid_tracks)
        fprintf('  *** FOUND %d VALID TRACK(S) ***\n', length(valid_tracks));
        for vt = 1:length(valid_tracks)
            assc = valid_tracks(vt).asscPointList;
            fprintf('  Track %d: %d associated points, frames [', vt, length(assc));
            for a = 1:length(assc)
                fprintf('%d ', assc(a).frameID);
            end
            fprintf(']\n');
        end
    else
        fprintf('  No valid tracks yet\n');
    end
end

% 直接测试 frame 3 的 NN 匹配
fprintf('\n=== Direct test: fun_find_best_asscpoints_NN for frame 3 real target ===\n');

% 逐帧运行 trackStarter_logic（不要预填充 tempTrackList！）
tempDirect = [];
for kk = 1:2
    ct = baseTime + kk * params.dt_sec / 86400.0;
    pl = det2nanyang_point(detList_R1{kk}, kk, ct);
    [tempDirect, vts] = trackStarter_logic(tempDirect, pl, sysPara, 3, 8);
    fprintf('Frame %d: tempDirect=%d pts, valid=%d\n', kk, length(tempDirect), length(vts));
end

fprintf('After 2 frames, tempDirect has %d points\n', length(tempDirect));

% 帧3的真实目标
k3 = 3;
ct3 = baseTime + k3 * params.dt_sec / 86400.0;
pl3 = det2nanyang_point(detList_R1{k3}, k3, ct3);
curPt = pl3(1);  % 真实目标
fprintf('Frame 3 real target: prange=%.1f, paz=%.2f, pvr=%.1f\n', curPt.prange, curPt.paz, curPt.pvr);

% 直接调用内置的 fun_find_best_asscpoints_NN（需要访问 trackStarter_logic 的内部函数）
% 由于这是子函数，无法直接调用。改为检查所有帧间距离。

fprintf('\nFrame 3 target vs all previous points:\n');
for i = 1:length(tempDirect)
    pt = tempDirect(i);
    r_diff = abs(pt.prange - curPt.prange);
    a_diff = abs(pt.paz - curPt.paz);
    v_diff = abs(pt.pvr - curPt.pvr);
    n_dist = sqrt((r_diff/10)^2 + (v_diff/20)^2 + 0.2*(a_diff/2)^2);
    fprintf('  F%d Pt: r=%.1f(Δ%.1f) az=%.2f(Δ%.2f) vr=%.1f(Δ%.1f) ndist=%.2f\n', ...
        pt.frameID, pt.prange, r_diff, pt.paz, a_diff, pt.pvr, v_diff, n_dist);
end

fprintf('\n=== Summary ===\n');
fprintf('Final tempTrackList size: %d\n', length(tempTrackList));
fprintf('M=%d, N=%d, n_frames=%d\n', M, N, n_frames_test);
