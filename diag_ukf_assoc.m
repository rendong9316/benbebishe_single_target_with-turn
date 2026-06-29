% diag_ukf_assoc.m — 诊断UKF逐帧关联的是真检测还是杂波
clear; close all; clc; addpath(genpath('.'));

seed = 184;
params = simulation_params(); params.random_seed = seed; rng(seed);
traj = aircraft_trajectory_create(params.aircraft_waypoints, params.aircraft_speed_ms, params.dt_sec);
true_track = aircraft_trajectory_interpolate('generate', traj);
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t1_grid));

dr1_est = params.radar1_range_bias_m; da1_est = params.radar1_azimuth_bias_deg;
detList_R1 = cell(n_frames, 1);
for k = 1:n_frames
    rng(seed + k);
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
        if ~(isfield(detRaw(d),'lat')&&~isnan(detRaw(d).lat))
            [~,lat_e,lon_e] = bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
            detRaw(d).lat=lat_e; detRaw(d).lon=lon_e;
        end
    end
    detList_R1{k} = detRaw;
end

params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std = params.radar1_ukf_P_pos_std; params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
params.gate_sigma = params.radar1_gate_sigma; params.gate_vr_ms = params.radar1_gate_vr_ms;
params.tracker_K_loss = params.radar1_tracker_K_loss;
ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
[snaps_R1, ~] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames, true_track, t1_grid);

fprintf('===== seed=%d R1 UKF帧帧关联分析 =====\n', seed);
fprintf('类型: TRACK=跟踪 INIT=起始中 LOST=丢失\n');
fprintf('关联: [R]=真实检测 [C]=杂波 [P]=纯预测(无关联)\n\n');

n_r = 0; n_c = 0; n_p = 0;
for k = 1:n_frames
    snap = snaps_R1{k};
    if isempty(snap.trackList), continue; end
    trk = snap.trackList{1};
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');

    if ~isnan(trk.lat)
        err = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb)/1000;
    else
        err = NaN;
    end

    det_info = ' - ';
    has_det = isfield(trk, 'assoc_det') && isstruct(trk.assoc_det) && ...
              isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange);

    if trk.type == 1  % TRACKING
        if has_det
            if trk.assoc_det.is_clutter
                det_info = '[C]'; n_c = n_c + 1;
            else
                det_info = '[R]'; n_r = n_r + 1;
            end
        else
            det_info = '[P]'; n_p = n_p + 1;
        end
        fprintf('%-3d | TRACK | err=%5.1fkm | assoc=%s | missed=%d life=%d\n', ...
            k, err, det_info, trk.missed, trk.life);
    elseif trk.type == 6
        fprintf('%-3d | INIT  |     ---     |   -   | -\n', k);
    elseif trk.type == 7
        fprintf('%-3d | LOST  |     ---     |   -   | -\n', k);
    end
end
fprintf('\n关联统计: [R]=%d  [C]=%d  [P]=%d  | 杂波率=%.1f%%\n', n_r, n_c, n_p, n_c/(n_r+n_c+n_p)*100);
fprintf('Done.\n');
