% Quick test: only Phase 0-5, no visualization
clear; close all; clc;
addpath(genpath('.'));

fprintf('=== Phase 0: Trajectory Test ===\n');
params = simulation_params();
rng(params.random_seed);

try
    [traj, turn_waypoints] = aircraft_trajectory_create('gradual_turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);
    fprintf('OK: Trajectory generated: %d points, %.0f s duration\n', size(true_track,1), traj.duration_sec);
    fprintf('   Waypoints: (%.4f,%.4f) -> (%.4f,%.4f) -> (%.4f,%.4f)\n', ...
        turn_waypoints(1,1), turn_waypoints(1,2), ...
        turn_waypoints(2,1), turn_waypoints(2,2), ...
        turn_waypoints(3,1), turn_waypoints(3,2));
catch e
    fprintf('FAIL: Trajectory generation error: %s\n', e.message);
end

% Turn rate
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
fprintf('   Bearing in=%.1f, out=%.1f, delta=%.1f, omega=%.4f\n', ...
    bearing_in, bearing_out, delta_hdg, turn_rate_rad_per_sec);

% Time grids
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('   n_frames = %d\n', n_frames);

fprintf('\n=== Phase 1: ADS-B Calibration Test ===\n');
try
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2;
    adsb_lon = T_adsb.Var3;
    dr1_list = []; da1_list = [];
    dr2_list = []; da2_list = [];
    n_check = min(100, height(T_adsb));  % Only 100 for quick test
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
    fprintf('OK: ADS-B calibration: R1=%d pts, R2=%d pts\n', length(dr1_list), length(dr2_list));
catch e
    fprintf('FAIL: ADS-B calibration: %s\n', e.message);
    dr1_est = params.radar1_range_bias_m; da1_est = params.radar1_azimuth_bias_deg;
    dr2_est = params.radar2_range_bias_m; da2_est = params.radar2_azimuth_bias_deg;
end

fprintf('\n=== Phase 2+4: Detections + Bias Correction Test ===\n');
try
    detRaw_R1 = cell(n_frames, 1);
    detRaw_R2 = cell(n_frames, 1);
    detList_R1 = cell(n_frames, 1);
    detList_R2 = cell(n_frames, 1);
    for k = 1:n_frames
        rng(params.random_seed + k);
        [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
        detRaw_R1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
            params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
            params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
        for d = 1:length(detRaw_R1{k}), detRaw_R1{k}(d).aircraft_id = 1; end

        rng(params.random_seed + 10000 + k);
        [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
        detRaw_R2{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, ...
            pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
            params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
            params.radar2_beam_center_deg, params, ...
            params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
        for d = 1:length(detRaw_R2{k}), detRaw_R2{k}(d).aircraft_id = 1; end

        % Bias correction
        dets_r1 = detRaw_R1{k};
        for d = 1:length(dets_r1)
            dets_r1(d).drange = dets_r1(d).prange - dr1_est;
            dets_r1(d).daz = dets_r1(d).paz - da1_est;
            dets_r1(d).range_meas = dets_r1(d).drange;
            dets_r1(d).azimuth_meas = dets_r1(d).daz;
        end
        detList_R1{k} = dets_r1;

        dets_r2 = detRaw_R2{k};
        for d = 1:length(dets_r2)
            dets_r2(d).drange = dets_r2(d).prange - dr2_est;
            dets_r2(d).daz = dets_r2(d).paz - da2_est;
            dets_r2(d).range_meas = dets_r2(d).drange;
            dets_r2(d).azimuth_meas = dets_r2(d).daz;
        end
        detList_R2{k} = dets_r2;
    end
    fprintf('OK: Generated %d frames of detections for both radars\n', n_frames);
catch e
    fprintf('FAIL: Detection generation: %s\n', e.message);
    fprintf('  at %s:%d\n', e.stack(1).file, e.stack(1).line);
end

fprintf('\n=== Phase 5: IMM Tracker Test ===\n');
try
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
    params.gate_sigma      = params.radar1_gate_sigma;
    params.gate_vr_ms      = params.radar1_gate_vr_ms;
    params.tracker_K_loss  = params.radar1_tracker_K_loss;

    % CV UKF template
    ukf1_cv_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

    % CT UKF template
    ukf1_ct_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf1_ct_tpl.model_type = 'CT';
    ukf1_ct_tpl.turn_rate_rad_per_sec = turn_rate_rad_per_sec;
    ukf1_ct_tpl.Q = ukf1_ct_tpl.Q * 1.5;

    fprintf('   CV UKF model_type=%s\n', ukf1_cv_tpl.model_type);
    fprintf('   CT UKF model_type=%s, omega=%.4f\n', ukf1_ct_tpl.model_type, ukf1_ct_tpl.turn_rate_rad_per_sec);

    % Test UKF CT prediction step
    test_ukf = ukf_jichu('init', ukf1_ct_tpl, detList_R1{1}(1), detList_R1{2}(1));
    test_ukf.dt = params.dt_sec;
    [x_pred, P_pred] = ukf_jichu('predict', test_ukf);
    fprintf('   CT predict test: x=[%.4f, %.6f, %.4f, %.6f]\n', x_pred(1), x_pred(2), x_pred(3), x_pred(4));

    % Run IMM tracker (just R1, first 20 frames for speed)
    fprintf('   Running IMM tracker (R1, 20 frames)...\n');
    [snaps, ft] = imm_tracker(detList_R1(1:min(20,n_frames)), ukf1_cv_tpl, ukf1_ct_tpl, ...
        params, min(20,n_frames), true_track, t1_grid);
    fprintf('OK: IMM tracker: final type=%d, life=%d\n', ft.type, ft.life);
    % Check model probabilities
    if isfield(ft, 'mu_history')
        fprintf('   Final mu: CV=%.3f, CT=%.3f\n', ft.mu_history(end,1), ft.mu_history(end,2));
    end
catch e
    fprintf('FAIL: IMM tracker: %s\n', e.message);
    for i=1:length(e.stack)
        fprintf('  at %s:%d\n', e.stack(i).file, e.stack(i).line);
    end
end

fprintf('\n=== All tests completed ===\n');
