% Quick 10-seed MC to verify direction protection fix
clear; close all; clc;
addpath(genpath('.'));
addpath(genpath('nanyang'));

N_MC = 10;
fprintf('===== Quick Fix Verification: N=%d =====\n', N_MC);

bad_count = 0;
for mc = 1:N_MC
    close all;
    r = run_one_test(mc);

    is_bad = false;
    if r.ukf_R1 > 30 || r.ukf_R2 > 30 || r.ad_R1 > 30 || r.ad_R2 > 30
        is_bad = true;
    end
    if ~is_bad
        if (r.ukf_R1 > r.cal_R1*1.5 && r.ukf_R1 > 15) || ...
           (r.ukf_R2 > r.cal_R2*1.5 && r.ukf_R2 > 15) || ...
           (r.ad_R1 > r.cal_R1*1.5 && r.ad_R1 > 15) || ...
           (r.ad_R2 > r.cal_R2*1.5 && r.ad_R2 > 15)
            is_bad = true;
        end
    end

    if is_bad, bad_count = bad_count + 1; end

    fprintf('Seed %2d: UKF R1=%6.1f R2=%6.1f  ad R1=%6.1f R2=%6.1f  cal R1=%5.1f R2=%5.1f  %s\n', ...
        mc, r.ukf_R1, r.ukf_R2, r.ad_R1, r.ad_R2, r.cal_R1, r.cal_R2, ...
        iif(is_bad, '*** BAD ***', 'OK'));
end
fprintf('Bad seeds: %d/%d (%.0f%%)\n', bad_count, N_MC, bad_count/N_MC*100);

function r = run_one_test(seed)
    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    [traj, ~] = aircraft_trajectory_create('turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    % Phase 1: ADS-B calibration (simplified)
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
    dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat), continue; end
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
            az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
            dr1_list(end+1) = Rg_meas - Rg_true;
            daz = az_meas - az_true;
            if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
            da1_list(end+1) = daz;
        end
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, t_lon, t_lat, params.radar2_beam_center_deg, params);
        if in2
            Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat, t_lon, t_lat);
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

    % Phase 2: Generate detections
    detRaw_R1 = cell(n_frames, 1); detRaw_R2 = cell(n_frames, 1);
    for k = 1:n_frames
        rng(params.random_seed + k);
        [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
        detRaw_R1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
            params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
        for d = 1:length(detRaw_R1{k}), detRaw_R1{k}(d).aircraft_id = 1; end
        rng(params.random_seed + 10000 + k);
        [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
        detRaw_R2{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
            params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, params.radar2_beam_center_deg, params, ...
            params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
        for d = 1:length(detRaw_R2{k}), detRaw_R2{k}(d).aircraft_id = 1; end
    end

    % Phase 4: Calibration + geolocation
    detList_R1 = cell(n_frames, 1); detList_R2 = cell(n_frames, 1);
    for k = 1:n_frames
        dets_r1 = detRaw_R1{k};
        for d = 1:length(dets_r1)
            dets_r1(d).drange = dets_r1(d).prange - dr1_est;
            dets_r1(d).daz = dets_r1(d).paz - da1_est;
            dets_r1(d).range_meas = dets_r1(d).drange;
            dets_r1(d).azimuth_meas = dets_r1(d).daz;
            if ~(isfield(dets_r1(d), 'lat') && ~isnan(dets_r1(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(dets_r1(d).drange, dets_r1(d).daz, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                dets_r1(d).lat = lat_e; dets_r1(d).lon = lon_e;
            end
            [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r1(d).prange, dets_r1(d).paz, ...
                params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
            dets_r1(d).raw_lat = raw_lat; dets_r1(d).raw_lon = raw_lon;
        end
        detList_R1{k} = dets_r1;
        dets_r2 = detRaw_R2{k};
        for d = 1:length(dets_r2)
            dets_r2(d).drange = dets_r2(d).prange - dr2_est;
            dets_r2(d).daz = dets_r2(d).paz - da2_est;
            dets_r2(d).range_meas = dets_r2(d).drange;
            dets_r2(d).azimuth_meas = dets_r2(d).daz;
            if ~(isfield(dets_r2(d), 'lat') && ~isnan(dets_r2(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(dets_r2(d).drange, dets_r2(d).daz, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                dets_r2(d).lat = lat_e; dets_r2(d).lon = lon_e;
            end
            [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r2(d).prange, dets_r2(d).paz, ...
                params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
            dets_r2(d).raw_lat = raw_lat; dets_r2(d).raw_lon = raw_lon;
        end
        detList_R2{k} = dets_r2;
    end

    r.cal_R1 = rmse_detlist_qt(detList_R1, true_track, t1_grid, n_frames, 'cal');
    r.cal_R2 = rmse_detlist_qt(detList_R2, true_track, t2_grid, n_frames, 'cal');

    % Phase 5.1: Base UKF
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    params.gate_sigma = params.radar1_gate_sigma;
    params.tracker_K_loss = params.radar1_tracker_K_loss;
    ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    params_r2 = params;
    params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma = params.radar2_gate_sigma;
    params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    params_r2.tracker_M = 4; params_r2.tracker_N = 8;
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
    ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    [snaps1, ~] = single_track_runner_nanyang(detList_R1, ukf1_tpl, params, n_frames);
    [snaps2, ~] = single_track_runner_nanyang(detList_R2, ukf2_tpl, params_r2, n_frames);
    r.ukf_R1 = rmse_tracks_qt(snaps1, true_track, t1_grid, n_frames);
    r.ukf_R2 = rmse_tracks_qt(snaps2, true_track, t2_grid, n_frames);

    % Phase 5.2: Adaptive UKF
    rng(params.random_seed);
    ukf1_tpl_ad = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl_ad = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    [snaps1_ad, ~] = single_track_runner_nanyang_adaptive(detList_R1, ukf1_tpl_ad, params, n_frames);
    [snaps2_ad, ~] = single_track_runner_nanyang_adaptive(detList_R2, ukf2_tpl_ad, params_r2, n_frames);
    r.ad_R1 = rmse_tracks_qt(snaps1_ad, true_track, t1_grid, n_frames);
    r.ad_R2 = rmse_tracks_qt(snaps2_ad, true_track, t2_grid, n_frames);
end

function v = rmse_detlist_qt(detList, true_track, t_grid, n_frames, mode)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        for d = 1:length(detList{k})
            dp = detList{k}(d);
            if dp.is_clutter, continue; end
            if strcmp(mode, 'cal')
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
                end
            end
        end
    end
    v = iif(isempty(errs), NaN, sqrt(mean(errs.^2)));
end

function v = rmse_tracks_qt(snaps, true_track, t_grid, n_frames)
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
    v = iif(isempty(errs), NaN, sqrt(mean(errs.^2)));
end

function v = iif(cond, t, f)
    if cond, v = t; else, v = f; end
end
