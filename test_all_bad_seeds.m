% test_all_bad_seeds.m — 验证所有94个坏种子在当前配置下的表现
clear; close all; clc; addpath(genpath('.'));

load('results/mc_straight_20260629_181812.mat');
bad_idx = find(bad_seed);
all_bad_seeds = SEED_BASE + bad_idx - 1;
N = length(all_bad_seeds);

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║         验证 %d 个坏种子 (当前全量修复配置)                  ║\n', N);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

results = struct();
n_fixed = 0; n_r1bad = 0; n_r2bad = 0; n_double = 0;
cal_R1 = 9.3; cal_R2 = 10.6;

for idx = 1:N
    seed = all_bad_seeds(idx);
    params = simulation_params(); params.random_seed = seed; rng(seed);
    traj = aircraft_trajectory_create(params.aircraft_waypoints, params.aircraft_speed_ms, params.dt_sec);
    true_track = aircraft_trajectory_interpolate('generate', traj);
    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    dr1_est = params.radar1_range_bias_m; da1_est = params.radar1_azimuth_bias_deg;
    dr2_est = params.radar2_range_bias_m; da2_est = params.radar2_azimuth_bias_deg;
    detList_R1 = cell(n_frames, 1); detList_R2 = cell(n_frames, 1);
    for k = 1:n_frames
        rng(seed + k);
        [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
        detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
            k, t1_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
            params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
        for d = 1:length(detRaw)
            detRaw(d).aircraft_id = 1; Rgc = detRaw(d).prange - dr1_est; azc = detRaw(d).paz - da1_est;
            detRaw(d).drange = Rgc; detRaw(d).daz = azc;
            detRaw(d).range_meas = Rgc; detRaw(d).azimuth_meas = azc;
            if ~(isfield(detRaw(d),'lat')&&~isnan(detRaw(d).lat))
                [~,lat_e,lon_e] = bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                detRaw(d).lat=lat_e; detRaw(d).lon=lon_e;
            end
        end
        detList_R1{k} = detRaw;
        rng(seed + 10000 + k);
        [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
        detRaw2 = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, pos2(1), pos2(2), vel2(1), vel2(2), ...
            k, t2_grid(k), params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
            params.radar2_beam_center_deg, params, ...
            params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
        for d = 1:length(detRaw2)
            detRaw2(d).aircraft_id = 1; Rgc = detRaw2(d).prange - dr2_est; azc = detRaw2(d).paz - da2_est;
            detRaw2(d).drange = Rgc; detRaw2(d).daz = azc;
            detRaw2(d).range_meas = Rgc; detRaw2(d).azimuth_meas = azc;
            if ~(isfield(detRaw2(d),'lat')&&~isnan(detRaw2(d).lat))
                [~,lat_e,lon_e] = bistatic_inverse_solver(Rgc,azc,params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
                detRaw2(d).lat=lat_e; detRaw2(d).lon=lon_e;
            end
        end
        detList_R2{k} = detRaw2;
    end

    % R1 UKF
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std; params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    params.gate_sigma = params.radar1_gate_sigma; params.gate_vr_ms = params.radar1_gate_vr_ms;
    params.tracker_K_loss = params.radar1_tracker_K_loss;
    ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    [snaps_R1, ~] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames, true_track, t1_grid);

    % R2 UKF
    params_r2 = params;
    params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma = params.radar2_gate_sigma; params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
    params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std; params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    params_r2.tracker_M = 4; params_r2.tracker_N = 8;
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
    ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    [snaps_R2, ~] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames, true_track, t2_grid);

    % 评估
    errs_R1 = []; errs_R2 = []; segs_R1 = []; segs_R2 = [];
    in_seg1 = false; in_seg2 = false; s1 = 0; s2 = 0;
    n_r1 = 0; n_c1 = 0; n_total1 = 0; n_r2 = 0; n_c2 = 0; n_total2 = 0;

    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        if ~isempty(snaps_R1{k}) && ~isempty(snaps_R1{k}.trackList)
            trk = snaps_R1{k}.trackList{1};
            if trk.type == 1 && ~isnan(trk.lat)
                errs_R1(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb)/1000;
                if ~in_seg1, in_seg1 = true; s1 = k; end
                n_total1 = n_total1 + 1;
                if isfield(trk,'assoc_det')&&isstruct(trk.assoc_det)&&isfield(trk.assoc_det,'prange')&&~isempty(trk.assoc_det.prange)
                    if trk.assoc_det.is_clutter, n_c1 = n_c1 + 1; else, n_r1 = n_r1 + 1; end
                end
            elseif in_seg1, in_seg1 = false; segs_R1(end+1,:) = [s1, k-1, k-s1]; end
        elseif in_seg1, in_seg1 = false; segs_R1(end+1,:) = [s1, k-1, k-s1]; end

        tl2 = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
        tb2 = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
        if ~isempty(snaps_R2{k}) && ~isempty(snaps_R2{k}.trackList)
            trk = snaps_R2{k}.trackList{1};
            if trk.type == 1 && ~isnan(trk.lat)
                errs_R2(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl2, tb2)/1000;
                if ~in_seg2, in_seg2 = true; s2 = k; end
                n_total2 = n_total2 + 1;
                if isfield(trk,'assoc_det')&&isstruct(trk.assoc_det)&&isfield(trk.assoc_det,'prange')&&~isempty(trk.assoc_det.prange)
                    if trk.assoc_det.is_clutter, n_c2 = n_c2 + 1; else, n_r2 = n_r2 + 1; end
                end
            elseif in_seg2, in_seg2 = false; segs_R2(end+1,:) = [s2, k-1, k-s2]; end
        elseif in_seg2, in_seg2 = false; segs_R2(end+1,:) = [s2, k-1, k-s2]; end
    end
    if in_seg1, segs_R1(end+1,:) = [s1, n_frames, n_frames-s1+1]; end
    if in_seg2, segs_R2(end+1,:) = [s2, n_frames, n_frames-s2+1]; end

    rmse_R1 = sqrt(mean(errs_R1.^2)); rmse_R2 = sqrt(mean(errs_R2.^2));
    imp_R1 = (1-rmse_R1/cal_R1)*100; imp_R2 = (1-rmse_R2/cal_R2)*100;
    mtl_R1 = iif(isempty(segs_R1),0,mean(segs_R1(:,3)));
    mtl_R2 = iif(isempty(segs_R2),0,mean(segs_R2(:,3)));
    nseg_R1 = size(segs_R1,1); nseg_R2 = size(segs_R2,1);
    clutter_R1 = n_c1/max(1,n_total1)*100; clutter_R2 = n_c2/max(1,n_total2)*100;

    % 分类
    if imp_R1 > 0 && imp_R2 > 0 && nseg_R1 <= 2 && nseg_R2 <= 2
        tag = '✅已修复';
        n_fixed = n_fixed + 1;
    elseif imp_R1 <= 0 && imp_R2 > 0
        tag = 'R1坏R2好';
        n_r1bad = n_r1bad + 1;
    elseif imp_R1 > 0 && imp_R2 <= 0
        tag = 'R1好R2坏';
        n_r2bad = n_r2bad + 1;
    else
        tag = '❌双坏';
        n_double = n_double + 1;
    end

    fprintf('seed=%-3d %s | R1:%.0fkm(%.0f%%)%d段MTL=%.0f C=%.0f%% | R2:%.0fkm(%.0f%%)%d段MTL=%.0f C=%.0f%%\n', ...
        seed, tag, rmse_R1, imp_R1, nseg_R1, mtl_R1, clutter_R1, ...
        rmse_R2, imp_R2, nseg_R2, mtl_R2, clutter_R2);
end

fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  汇总: 已修复=%d | R1坏R2好=%d | R1好R2坏=%d | 双坏=%d     ║\n', ...
    n_fixed, n_r1bad, n_r2bad, n_double);
fprintf('║  融合可救=%d/%d (%.0f%%)                                    ║\n', ...
    n_fixed+n_r1bad+n_r2bad, N, (n_fixed+n_r1bad+n_r2bad)/N*100);
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('Done.\n');

function v = iif(c,t,f), if c, v=t; else, v=f; end; end
