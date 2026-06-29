% Full test: Phase 0-5 of run_simulation_turn.m pipeline
clear; close all; clc;
addpath(genpath('.'));

fprintf('=== Phase 0: Trajectory ===\n');
params = simulation_params();
rng(params.random_seed);
[traj, turn_waypoints] = aircraft_trajectory_create('gradual_turn', params);
true_track = aircraft_trajectory_interpolate('generate', traj);
fprintf('OK: %d pts, %.0fs\n', size(true_track,1), traj.duration_sec);

bearing_in  = sphere_utils_azimuth(turn_waypoints(1,1), turn_waypoints(1,2), ...
    turn_waypoints(2,1), turn_waypoints(2,2));
bearing_out = sphere_utils_azimuth(turn_waypoints(2,1), turn_waypoints(2,2), ...
    turn_waypoints(3,1), turn_waypoints(3,2));
delta_hdg = bearing_out - bearing_in;
if delta_hdg > 180, delta_hdg = delta_hdg - 360;
elseif delta_hdg < -180, delta_hdg = delta_hdg + 360; end
turn_sign = sign(delta_hdg);
if turn_sign == 0, turn_sign = 1; end
turn_rate_rad_per_sec = turn_sign * 1.0 * pi / 180.0;

t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('   n_frames=%d, omega=%.4f\n', n_frames, turn_rate_rad_per_sec);

fprintf('\n=== Phase 1: ADS-B Calib ===\n');
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
fprintf('OK: R1=%dpts(dr=%.0f, da=%.3f), R2=%dpts(dr=%.0f, da=%.3f)\n', ...
    length(dr1_list), dr1_est, da1_est, length(dr2_list), dr2_est, da2_est);

fprintf('\n=== Phase 2+4: Detections + Bias Corr ===\n');
detList_R1 = cell(n_frames, 1);
detList_R2 = cell(n_frames, 1);
for k = 1:n_frames
    rng(params.random_seed + k);
    [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
    detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, ...
        pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    for d = 1:length(detRaw), detRaw(d).aircraft_id = 1; end
    dets = detRaw;
    for d = 1:length(dets)
        Rgc = dets(d).prange - dr1_est;
        azc = dets(d).paz - da1_est;
        dets(d).drange = Rgc; dets(d).daz = azc;
        dets(d).range_meas = Rgc; dets(d).azimuth_meas = azc;
        if ~(isfield(dets(d), 'lat') && ~isnan(dets(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets(d).lat = lat_e; dets(d).lon = lon_e;
        end
    end
    detList_R1{k} = dets;

    rng(params.random_seed + 10000 + k);
    [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
    detRaw2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, ...
        pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
        params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
        params.radar2_beam_center_deg, params, ...
        params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
    for d = 1:length(detRaw2), detRaw2(d).aircraft_id = 1; end
    dets2 = detRaw2;
    for d = 1:length(dets2)
        Rgc2 = dets2(d).prange - dr2_est;
        azc2 = dets2(d).paz - da2_est;
        dets2(d).drange = Rgc2; dets2(d).daz = azc2;
        dets2(d).range_meas = Rgc2; dets2(d).azimuth_meas = azc2;
        if ~(isfield(dets2(d), 'lat') && ~isnan(dets2(d).lat))
            [~, lat_e2, lon_e2] = bistatic_inverse_solver(Rgc2, azc2, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets2(d).lat = lat_e2; dets2(d).lon = lon_e2;
        end
    end
    detList_R2{k} = dets2;
end
n_r1 = 0; n_r2 = 0;
for k = 1:n_frames
    for d = 1:length(detList_R1{k})
        if ~detList_R1{k}(d).is_clutter, n_r1 = n_r1 + 1; end
    end
    for d = 1:length(detList_R2{k})
        if ~detList_R2{k}(d).is_clutter, n_r2 = n_r2 + 1; end
    end
end
fprintf('OK: R1 ac-det=%d, R2 ac-det=%d over %d frames\n', n_r1, n_r2, n_frames);

fprintf('\n=== Phase 5: IMM Tracker (R1, full) ===\n');
% Set R1 UKF params (same as run_simulation_turn.m lines 314-321)
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
params.gate_sigma      = params.radar1_gate_sigma;
params.gate_vr_ms      = params.radar1_gate_vr_ms;
params.tracker_K_loss  = params.radar1_tracker_K_loss;

ukf1_cv_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
ukf1_ct_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
ukf1_ct_tpl.model_type = 'CT';
ukf1_ct_tpl.turn_rate_rad_per_sec = turn_rate_rad_per_sec;
ukf1_ct_tpl.Q = ukf1_ct_tpl.Q * 1.5;
fprintf('CV model_type=%s, CT model_type=%s omega=%.4f\n', ...
    ukf1_cv_tpl.model_type, ukf1_ct_tpl.model_type, ukf1_ct_tpl.turn_rate_rad_per_sec);

fprintf('Running IMM tracker (R1, %d frames)...\n', n_frames);
[trackSnapshots_R1, finalTrk1] = imm_tracker(detList_R1, ukf1_cv_tpl, ukf1_ct_tpl, ...
    params, n_frames, true_track, t1_grid);

% Check model probabilities
if isfield(finalTrk1, 'mu_history')
    mu_cv_avg = mean(finalTrk1.mu_history(:,1));
    mu_ct_avg = mean(finalTrk1.mu_history(:,2));
    n_ct_dom = sum(finalTrk1.mu_history(:,2) > 0.5);
    fprintf('Model prob: CV avg=%.3f, CT avg=%.3f, CT dominant=%d/%d frames\n', ...
        mu_cv_avg, mu_ct_avg, n_ct_dom, n_frames);
end

% Use same type_str logic as run_simulation_turn.m
type_str = 'UNKNOWN';
switch finalTrk1.type
    case 1, type_str = 'RELIABLE';
    case 2, type_str = 'MAINTAIN';
    case 6, type_str = 'TEMPORARY';
    case 7, type_str = 'HISTORY';
end
fprintf('Final track: type=%s life=%d quality=%d\n', type_str, finalTrk1.life, finalTrk1.quality);

% Association diagnostics
n_assoc = 0; n_predict = 0; n_init = 0; n_lost = 0; init_frame = 0;
for k = 1:length(trackSnapshots_R1)
    if isempty(trackSnapshots_R1{k}.trackList), continue; end
    trk = trackSnapshots_R1{k}.trackList{1};
    if trk.type == 6
        n_init = n_init + 1;
    elseif trk.type == 1
        if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
                isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
            n_assoc = n_assoc + 1;
        else
            n_predict = n_predict + 1;
        end
    elseif trk.type == 7
        n_lost = n_lost + 1;
    end
    if init_frame == 0 && trk.type == 1, init_frame = k; end
end
n_tracked = n_assoc + n_predict;
fprintf('Diag: init_frame=%d | assoc=%d predict=%d (%%%.0f) | init=%d lost=%d\n', ...
    init_frame, n_assoc, n_predict, n_assoc/max(1,n_tracked)*100, n_init, n_lost);

% RMSE
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R1{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('RMSE: %.1f km (n=%d)\n', sqrt(mean(errs.^2)), length(errs));

fprintf('\n=== ALL PHASES COMPLETE ===\n');
