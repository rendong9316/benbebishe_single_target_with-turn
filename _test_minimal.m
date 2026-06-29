% Minimal ASCII-only test for run_simulation_turn.m components
clear; close all; clc;
addpath(genpath('.'));

fprintf('=== Minimal Test ===\n');

% Phase 0: Trajectory
fprintf('\n--- Phase 0: Trajectory ---\n');
params = simulation_params();
rng(params.random_seed);

try
    [traj, turn_waypoints] = aircraft_trajectory_create('gradual_turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);
    fprintf('OK: Trajectory: %d pts, %.0f s\n', size(true_track,1), traj.duration_sec);
    fprintf('   W1=(%.4f,%.4f), W2=(%.4f,%.4f), W3=(%.4f,%.4f)\n', ...
        turn_waypoints(1,1), turn_waypoints(1,2), ...
        turn_waypoints(2,1), turn_waypoints(2,2), ...
        turn_waypoints(3,1), turn_waypoints(3,2));
catch e
    fprintf('FAIL: Trajectory: %s\n', e.message);
    for i=1:length(e.stack)
        fprintf('  at %s:%d\n', e.stack(i).file, e.stack(i).line);
    end
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
fprintf('   n_frames = %d (R1=%d, R2=%d)\n', n_frames, length(t1_grid), length(t2_grid));

% Phase 1: ADS-B calibration
fprintf('\n--- Phase 1: ADS-B Calibration ---\n');
try
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
    fprintf('OK: ADS-B calib: R1=%d pts (dr=%.1f, da=%.4f), R2=%d pts (dr=%.1f, da=%.4f)\n', ...
        length(dr1_list), dr1_est, da1_est, length(dr2_list), dr2_est, da2_est);
catch e
    fprintf('FAIL: ADS-B: %s\n', e.message);
    dr1_est = params.radar1_range_bias_m; da1_est = params.radar1_azimuth_bias_deg;
    dr2_est = params.radar2_range_bias_m; da2_est = params.radar2_azimuth_bias_deg;
end

% Phase 2: Generate detections (just first 5 frames for speed)
fprintf('\n--- Phase 2+4: Detections (first 5 frames) ---\n');
try
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
        % Bias correction
        dets = detRaw;
        for d = 1:length(dets)
            drange = dets(d).prange - dr1_est;
            daz = dets(d).paz - da1_est;
            dets(d).drange = drange;
            dets(d).daz = daz;
            dets(d).range_meas = drange;
            dets(d).azimuth_meas = daz;
            if ~(isfield(dets(d), 'lat') && ~isnan(dets(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(drange, daz, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, ...
                    params.radar1_lon, params.radar1_lat);
                dets(d).lat = lat_e;
                dets(d).lon = lon_e;
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
            drange3 = dets2(d).prange - dr2_est;
            daz3 = dets2(d).paz - da2_est;
            dets2(d).drange = drange3;
            dets2(d).daz = daz3;
            dets2(d).range_meas = drange3;
            dets2(d).azimuth_meas = daz3;
            if ~(isfield(dets2(d), 'lat') && ~isnan(dets2(d).lat))
                [~, lat_e2, lon_e2] = bistatic_inverse_solver(drange3, daz3, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, ...
                    params.radar2_lon, params.radar2_lat);
                dets2(d).lat = lat_e2;
                dets2(d).lon = lon_e2;
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
catch e
    fprintf('FAIL: Detections: %s\n', e.message);
    for i=1:length(e.stack)
        fprintf('  at %s:%d\n', e.stack(i).file, e.stack(i).line);
    end
end

% Phase 5: IMM Tracker (R1 only, first 20 frames for speed)
fprintf('\n--- Phase 5: IMM Tracker (R1, 20 frames) ---\n');
try
    % R1 UKF params
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
    params.gate_sigma      = params.radar1_gate_sigma;
    params.gate_vr_ms      = params.radar1_gate_vr_ms;
    params.tracker_K_loss  = params.radar1_tracker_K_loss;

    % Create CV UKF template
    ukf1_cv_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

    % Create CT UKF template
    ukf1_ct_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf1_ct_tpl.model_type = 'CT';
    ukf1_ct_tpl.turn_rate_rad_per_sec = turn_rate_rad_per_sec;
    ukf1_ct_tpl.Q = ukf1_ct_tpl.Q * 1.5;

    fprintf('   CV model_type=%s, CT model_type=%s omega=%.4f\n', ...
        ukf1_cv_tpl.model_type, ukf1_ct_tpl.model_type, ukf1_ct_tpl.turn_rate_rad_per_sec);

    % Test UKF CT prediction
    test_ukf = ukf_jichu('init', ukf1_ct_tpl, detList_R1{1}(1), detList_R1{2}(1));
    test_ukf.dt = params.dt_sec;
    [x_pred, P_pred] = ukf_jichu('predict', test_ukf);
    fprintf('   CT predict test: x=[%.4f, %.6f, %.4f, %.6f]\n', x_pred(1), x_pred(2), x_pred(3), x_pred(4));

    % Run IMM tracker
    n_test = min(20, n_frames);
    fprintf('   Running IMM tracker (%d frames)...\n', n_test);
    [snaps, ft] = imm_tracker(detList_R1(1:n_test), ukf1_cv_tpl, ukf1_ct_tpl, ...
        params, n_test, true_track, t1_grid);
    fprintf('   OK: IMM final: type=%d, life=%d, quality=%d\n', ft.type, ft.life, ft.quality);
    if isfield(ft, 'mu_history')
        fprintf('   Final mu: CV=%.3f, CT=%.3f\n', ft.mu_history(end,1), ft.mu_history(end,2));
        fprintf('   Avg mu:   CV=%.3f, CT=%.3f\n', mean(ft.mu_history(:,1)), mean(ft.mu_history(:,2)));
    end
catch e
    fprintf('FAIL: IMM: %s\n', e.message);
    for i=1:length(e.stack)
        fprintf('  at %s:%d\n', e.stack(i).file, e.stack(i).line);
    end
end

fprintf('\n=== All minimal tests done ===\n');
