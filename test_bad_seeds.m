% test_bad_seeds.m — 快速针对性测试已知坏种子
clear; close all; clc; addpath(genpath('.'));

% 代表性坏种子（覆盖三类失败模式）
bad_seeds = [127, 132, 140, 146, 163, 176, 184, 192, 275, 289, 396, 402, 418, 437];
% 再加几个好种子做对照
good_seeds = [1, 2, 50, 100, 200];
test_seeds = [good_seeds, bad_seeds];

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║         针对性测试 %d 个种子 (M/N优先+超时兜底)              ║\n', length(test_seeds));
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

for idx = 1:length(test_seeds)
    seed = test_seeds(idx);
    params = simulation_params();
    params.random_seed = seed;
    rng(seed);

    traj = aircraft_trajectory_create(params.aircraft_waypoints, ...
        params.aircraft_speed_ms, params.dt_sec);
    true_track = aircraft_trajectory_interpolate('generate', traj);
    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    % 简化标定 + 点迹生成（跳过 ADS-B 校准，用真值偏差直接测）
    rng(seed);
    dr1_est = params.radar1_range_bias_m;  da1_est = params.radar1_azimuth_bias_deg;
    dr2_est = params.radar2_range_bias_m;  da2_est = params.radar2_azimuth_bias_deg;

    detList_R1 = cell(n_frames, 1);  detList_R2 = cell(n_frames, 1);
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
            Rgc = detRaw(d).prange - dr1_est;  azc = detRaw(d).paz - da1_est;
            detRaw(d).drange = Rgc;  detRaw(d).daz = azc;
            detRaw(d).range_meas = Rgc;  detRaw(d).azimuth_meas = azc;
            if ~(isfield(detRaw(d), 'lat') && ~isnan(detRaw(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat);
                detRaw(d).lat = lat_e;  detRaw(d).lon = lon_e;
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
            detRaw2(d).aircraft_id = 1;
            Rgc = detRaw2(d).prange - dr2_est;  azc = detRaw2(d).paz - da2_est;
            detRaw2(d).drange = Rgc;  detRaw2(d).daz = azc;
            detRaw2(d).range_meas = Rgc;  detRaw2(d).azimuth_meas = azc;
            if ~(isfield(detRaw2(d), 'lat') && ~isnan(detRaw2(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat);
                detRaw2(d).lat = lat_e;  detRaw2(d).lon = lon_e;
            end
        end
        detList_R2{k} = detRaw2;
    end

    % R1 UKF
    params.ukf_range_std_m = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
    params.gate_sigma = params.radar1_gate_sigma;
    params.gate_vr_ms = params.radar1_gate_vr_ms;
    params.tracker_K_loss = params.radar1_tracker_K_loss;
    ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    [snaps_R1, ~] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames, true_track, t1_grid);

    % R2 UKF
    params_r2 = params;
    params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma = params.radar2_gate_sigma;
    params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
    params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
    params_r2.tracker_M = 4;  params_r2.tracker_N = 8;
    params_r2.tracker_K_loss = params.radar2_tracker_K_loss;
    ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    [snaps_R2, ~] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames, true_track, t2_grid);

    % 评估
    errs_R1 = []; errs_R2 = [];
    segs_R1 = []; segs_R2 = [];
    in_seg1 = false; in_seg2 = false; s1 = 0; s2 = 0;
    n_assoc1 = 0; n_pred1 = 0; n_assoc2 = 0; n_pred2 = 0;
    n_total1 = 0; n_total2 = 0;

    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');

        % R1
        if ~isempty(snaps_R1{k}) && ~isempty(snaps_R1{k}.trackList)
            trk = snaps_R1{k}.trackList{1};
            if isfield(trk,'type') && trk.type == 1 && ~isnan(trk.lat)
                errs_R1(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb)/1000;
                if ~in_seg1, in_seg1 = true; s1 = k; end
                n_total1 = n_total1 + 1;
                if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
                   isfield(trk.assoc_det,'prange') && ~isempty(trk.assoc_det.prange)
                    n_assoc1 = n_assoc1 + 1;
                else, n_pred1 = n_pred1 + 1; end
            elseif in_seg1
                in_seg1 = false; segs_R1(end+1,:) = [s1, k-1, k-s1];
            end
        elseif in_seg1
            in_seg1 = false; segs_R1(end+1,:) = [s1, k-1, k-s1];
        end

        % R2
        tl2 = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
        tb2 = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
        if ~isempty(snaps_R2{k}) && ~isempty(snaps_R2{k}.trackList)
            trk = snaps_R2{k}.trackList{1};
            if isfield(trk,'type') && trk.type == 1 && ~isnan(trk.lat)
                errs_R2(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl2, tb2)/1000;
                if ~in_seg2, in_seg2 = true; s2 = k; end
                n_total2 = n_total2 + 1;
                if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
                   isfield(trk.assoc_det,'prange') && ~isempty(trk.assoc_det.prange)
                    n_assoc2 = n_assoc2 + 1;
                else, n_pred2 = n_pred2 + 1; end
            elseif in_seg2
                in_seg2 = false; segs_R2(end+1,:) = [s2, k-1, k-s2];
            end
        elseif in_seg2
            in_seg2 = false; segs_R2(end+1,:) = [s2, k-1, k-s2];
        end
    end
    if in_seg1, segs_R1(end+1,:) = [s1, n_frames, n_frames-s1+1]; end
    if in_seg2, segs_R2(end+1,:) = [s2, n_frames, n_frames-s2+1]; end

    rmse_R1 = sqrt(mean(errs_R1.^2));
    rmse_R2 = sqrt(mean(errs_R2.^2));
    cal_R1 = 9.3; cal_R2 = 10.6;  % 基准校准RMSE
    imp_R1 = (1 - rmse_R1/cal_R1)*100;
    imp_R2 = (1 - rmse_R2/cal_R2)*100;
    mtl_R1 = mean(segs_R1(:,3)); mtl_R2 = mean(segs_R2(:,3));
    assoc_R1 = n_assoc1/max(1,n_total1)*100;
    assoc_R2 = n_assoc2/max(1,n_total2)*100;

    is_bad = any(test_seeds(idx) == bad_seeds);
    tag = '';
    if is_bad
        if imp_R1 < -20 && imp_R2 > 0, tag = '[原R1坏→]';
        elseif imp_R1 > 0 && imp_R2 < -20, tag = '[原R2坏→]';
        elseif imp_R1 < -20 && imp_R2 < -20, tag = '[原双坏→]';
        else, tag = '[已修复!]';
        end
    else
        tag = '[好种子参照]';
    end

    fprintf('seed=%-3d %s\n', seed, tag);
    fprintf('  R1: RMSE=%.1fkm(%+.0f%%) %d段MTL=%.1f 关联=%.0f%%\n', ...
        rmse_R1, imp_R1, size(segs_R1,1), mtl_R1, assoc_R1);
    for s = 1:size(segs_R1,1)
        fprintf('      段%d: [%d-%d:%d帧] ', s, segs_R1(s,1), segs_R1(s,2), segs_R1(s,3));
    end
    fprintf('\n');
    fprintf('  R2: RMSE=%.1fkm(%+.0f%%) %d段MTL=%.1f 关联=%.0f%%\n', ...
        rmse_R2, imp_R2, size(segs_R2,1), mtl_R2, assoc_R2);
    for s = 1:size(segs_R2,1)
        fprintf('      段%d: [%d-%d:%d帧] ', s, segs_R2(s,1), segs_R2(s,2), segs_R2(s,3));
    end
    fprintf('\n\n');
end
fprintf('Done.\n');
