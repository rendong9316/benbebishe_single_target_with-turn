% diag_mn_init.m — 诊断坏种子M/N起始选了真检测还是杂波
clear; close all; clc; addpath(genpath('.'));

seeds_to_test = [184, 192, 396, 402, 132, 176];  % 钉子户
params_base = simulation_params();

for si = 1:length(seeds_to_test)
    seed = seeds_to_test(si);
    params = params_base; params.random_seed = seed; rng(seed);
    traj = aircraft_trajectory_create(params.aircraft_waypoints, params.aircraft_speed_ms, params.dt_sec);
    true_track = aircraft_trajectory_interpolate('generate', traj);
    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t1_grid));

    dr1_est = params.radar1_range_bias_m; da1_est = params.radar1_azimuth_bias_deg;
    detList = cell(n_frames, 1);
    for k = 1:n_frames
        rng(seed + k);
        [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
        detRaw = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, pos(1), pos(2), vel(1), vel(2), ...
            k, t1_grid(k), params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
            params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
        for d = 1:length(detRaw)
            Rgc = detRaw(d).prange - dr1_est; azc = detRaw(d).paz - da1_est;
            detRaw(d).drange = Rgc; detRaw(d).daz = azc;
            detRaw(d).range_meas = Rgc; detRaw(d).azimuth_meas = azc;
            if ~(isfield(detRaw(d),'lat')&&~isnan(detRaw(d).lat))
                [~,lat_e,lon_e] = bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                detRaw(d).lat=lat_e; detRaw(d).lon=lon_e;
            end
        end
        detList{k} = detRaw;
    end

    fprintf('===== seed=%d =====\n', seed);
    fprintf('M/N条件: M=%d N=%d 速度门30-600m/s 共识分≥1\n', params.tracker_M, params.tracker_N);
    fprintf('检测标记: [R]=真实 [C]=杂波\n\n');

    init_state = track_initiation('init', params);

    for k = 1:n_frames
        dets = detList{k};
        [init_state, det1, det2, success] = track_initiation('process', init_state, dets, params, k);

        if success
            r1 = ~det1.is_clutter; r2 = ~det2.is_clutter;
            tl1 = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
            tb1 = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
            dt_err = sphere_utils_haversine_distance(det2.lon, det2.lat, tl1, tb1)/1000;

            n_frames_apart = 1;  % 默认1帧差
            est_speed = sphere_utils_haversine_distance(det1.lon, det1.lat, det2.lon, det2.lat) / (n_frames_apart * params.dt_sec);

            fprintf('帧%2d: det1=%s det2=%s | 估计速度=%.0f m/s | det2离真值=%.1f km\n', ...
                k, iif(r1,'[R]','[C]'), iif(r2,'[R]','[C]'), est_speed, dt_err);

            % 窗内统计
            n_r = 0; n_c = 0;
            for w = 1:length(init_state.window)
                wd = init_state.window{w};
                if ~isempty(wd)
                    for dd = 1:length(wd)
                        if wd(dd).is_clutter, n_c = n_c + 1; else, n_r = n_r + 1; end
                    end
                end
            end
            fprintf('  窗内: %d真 %d杂 | 有检测帧=%d/%d\n', n_r, n_c, sum(init_state.has_det), length(init_state.has_det));

            if ~r1 || ~r2
                fprintf('  *** 杂波混入起始! ***\n');
            end

            init_state = track_initiation('reset', params);
        end
    end
    fprintf('\n');
end
fprintf('Done.\n');

function v = iif(c,t,f)
    if c, v = t; else, v = f; end
end
