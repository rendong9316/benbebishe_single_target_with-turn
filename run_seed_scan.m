% =========================================================================
% run_seed_scan.m — 多种子鲁棒性扫描
% =========================================================================
% 临时覆盖 random_seed，调用 run_simulation_multi 主流程的核心代码，
% 收集三档指标的鲁棒性数据。
% =========================================================================

function run_seed_scan(seeds)
    if nargin < 1
        seeds = [94, 77, 123, 256, 8];
    end
    addpath(genpath('.'));

    results = struct();
    results.seeds = seeds;
    results.r1_rmse = zeros(length(seeds), 3);
    results.r2_rmse = zeros(length(seeds), 3);
    results.r1_ratio = zeros(length(seeds), 3);
    results.r2_ratio = zeros(length(seeds), 3);
    results.r1_n_tracks = zeros(length(seeds), 1);
    results.r2_n_tracks = zeros(length(seeds), 1);

    for s = 1:length(seeds)
        seed = seeds(s);
        fprintf('\n========== SEED %d ==========\n', seed);
        [r1_rmse, r2_rmse, r1_ratio, r2_ratio, n1, n2] = run_one_seed(seed);
        results.r1_rmse(s, :) = r1_rmse;
        results.r2_rmse(s, :) = r2_rmse;
        results.r1_ratio(s, :) = r1_ratio;
        results.r2_ratio(s, :) = r2_ratio;
        results.r1_n_tracks(s) = n1;
        results.r2_n_tracks(s) = n2;
    end

    print_summary(results);
    save('results/seed_scan.mat', 'results', '-v7.3');
end


function [r1_rmse, r2_rmse, r1_ratio, r2_ratio, n1, n2] = run_one_seed(seed)
    % 用 try/catch 包住整个仿真，单个种子失败不影响其他种子
    try
        params = simulation_params_multi();
        params.random_seed = seed;

        % ===== Phase 0: 三条强交叉航迹 =====
        way_A = [128.8, 30.5, 0; 132.0, 32.5, 0];
        way_B = [128.8, 32.5, 0; 132.0, 30.5, 0];
        way_C = [128.8, 31.5, 0; 130.5, 32.9, 0];

        traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
        traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
        traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);
        true_track_A = aircraft_trajectory_interpolate('generate', traj_A);
        true_track_B = aircraft_trajectory_interpolate('generate', traj_B);
        true_track_C = aircraft_trajectory_interpolate('generate', traj_C);

        t1_grid = params.time_offset_radar1_sec : params.dt_sec : max(max(traj_A.duration_sec, traj_B.duration_sec), traj_C.duration_sec);
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : max(max(traj_A.duration_sec, traj_B.duration_sec), traj_C.duration_sec);
        n_frames = min(length(t1_grid), length(t2_grid));

        truthTrajs = cell(3, 1);
        tt = true_track_A;
        truthTrajs{1} = struct('label', 'A', 'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
            'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
        tt = true_track_B;
        truthTrajs{2} = struct('label', 'B', 'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
            'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
        tt = true_track_C;
        truthTrajs{3} = struct('label', 'C', 'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
            'lon_rate', tt(:,3), 'lat_rate', tt(:,4));

        truth_all = {true_track_A, true_track_B, true_track_C};

        % ===== Phase 1: 偏差标定（用 seed 控制）=====
        rng(params.random_seed);
        adsb_T = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
        adsb_lat = adsb_T.Var2; adsb_lon = adsb_T.Var3;
        dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
        n_check = min(5000, height(adsb_T));
        cal_step = max(1, floor(height(adsb_T) / n_check));
        for idx = 1:cal_step:height(adsb_T)
            t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
            if isnan(t_lon) || isnan(t_lat), continue, end
            [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, t_lon, t_lat, params.radar1_beam_center_deg, params);
            if in1
                Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
                dr1_list(end+1) = Rg_meas - Rg_true;
                az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
                daz = az_meas - az_true;
                if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
                da1_list(end+1) = daz;
            end
            [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, t_lon, t_lat, params.radar2_beam_center_deg, params);
            if in2
                Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
                dr2_list(end+1) = Rg_meas - Rg_true;
                az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
                daz = az_meas - az_true;
                if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
                da2_list(end+1) = daz;
            end
        end
        dr1_est = mean(dr1_list); da1_est = mean(da1_list);
        dr2_est = mean(dr2_list); da2_est = mean(da2_list);

        % ===== Phase 2: 点迹生成 =====
        detList_R1 = cell(n_frames, 1);
        detList_R2 = cell(n_frames, 1);
        rng(params.random_seed + 1e7);
        for k = 1:n_frames
            t1 = t1_grid(k);
            tgt_states = zeros(3, 5);
            for ac = 1:3
                tt_ac = truth_all{ac};
                if t1 >= tt_ac(1,5) && t1 <= tt_ac(end,5)
                    tgt_states(ac,:) = [interp1(tt_ac(:,5), tt_ac(:,1), t1, 'linear','extrap'), ...
                                        interp1(tt_ac(:,5), tt_ac(:,2), t1, 'linear','extrap'), ...
                                        interp1(tt_ac(:,5), tt_ac(:,3), t1, 'linear','extrap'), ...
                                        interp1(tt_ac(:,5), tt_ac(:,4), t1, 'linear','extrap'), ac];
                else
                    tgt_states(ac,:) = [NaN NaN NaN NaN ac];
                end
            end
            tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);
            if isempty(tgt_states), detList_R1{k} = []; continue; end
            detRaw = generate_frame_detections_multi(params.radar1_lon, params.radar1_lat, ...
                params.radar1_tx_lon, params.radar1_tx_lat, tgt_states, k, t1, ...
                params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
                params.radar1_beam_center_deg, params, ...
                params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
            dets = {};
            for d = 1:length(detRaw)
                dp = detRaw(d);
                Rgc = dp.prange - dr1_est; azc = dp.paz - da1_est;
                dp.drange = Rgc; dp.daz = azc;
                dp.range_meas = Rgc; dp.azimuth_meas = azc;
                if ~isfield(dp,'lat') || isnan(dp.lat)
                    [~,dp.lat,dp.lon] = bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                end
                dets{end+1} = dp;
            end
            detList_R1{k} = [dets{:}];
        end
        rng(params.random_seed + 2e7);
        for k = 1:n_frames
            t2 = t2_grid(k);
            tgt_states = zeros(3, 5);
            for ac = 1:3
                tt_ac = truth_all{ac};
                if t2 >= tt_ac(1,5) && t2 <= tt_ac(end,5)
                    tgt_states(ac,:) = [interp1(tt_ac(:,5), tt_ac(:,1), t2, 'linear','extrap'), ...
                                        interp1(tt_ac(:,5), tt_ac(:,2), t2, 'linear','extrap'), ...
                                        interp1(tt_ac(:,5), tt_ac(:,3), t2, 'linear','extrap'), ...
                                        interp1(tt_ac(:,5), tt_ac(:,4), t2, 'linear','extrap'), ac];
                else
                    tgt_states(ac,:) = [NaN NaN NaN NaN ac];
                end
            end
            tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);
            if isempty(tgt_states), detList_R2{k} = []; continue; end
            detRaw = generate_frame_detections_multi(params.radar2_lon, params.radar2_lat, ...
                params.radar2_tx_lon, params.radar2_tx_lat, tgt_states, k, t2, ...
                params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
                params.radar2_beam_center_deg, params, ...
                params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
            dets = {};
            for d = 1:length(detRaw)
                dp = detRaw(d);
                Rgc = dp.prange - dr2_est; azc = dp.paz - da2_est;
                dp.drange = Rgc; dp.daz = azc;
                dp.range_meas = Rgc; dp.azimuth_meas = azc;
                if ~isfield(dp,'lat') || isnan(dp.lat)
                    [~,dp.lat,dp.lon] = bistatic_inverse_solver(Rgc,azc,params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
                end
                dets{end+1} = dp;
            end
            detList_R2{k} = [dets{:}];
        end

        % ===== Phase 5: 跟踪 =====
        params.ukf_range_std_m = params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale = params.radar1_ukf_Q_scale;
        params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
        params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
        params.gate_sigma = params.radar1_gate_sigma;
        params.gate_vr_ms = params.radar1_gate_vr_ms;
        ukf1_tpl = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

        params_r2 = params;
        params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        params_r2.gate_sigma = params.radar2_gate_sigma;
        params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
        params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
        params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
        params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
        ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

        [trackSnapshots_R1, trackList_R1] = run_tracker(detList_R1, ukf1_tpl, params, n_frames, t1_grid, truth_all);
        [trackSnapshots_R2, trackList_R2] = run_tracker(detList_R2, ukf2_tpl, params_r2, n_frames, t2_grid, truth_all);

        % ===== 收集指标 =====
        [r1_rmse, r1_ratio] = compute_metrics(trackSnapshots_R1, truthTrajs, n_frames, params.dt_sec);
        [r2_rmse, r2_ratio] = compute_metrics(trackSnapshots_R2, truthTrajs, n_frames, params.dt_sec);
        n1 = count_active(trackList_R1);
        n2 = count_active(trackList_R2);

        fprintf('  R1 RMSE: T1=%.1f T2=%.1f T3=%.1f km | ratios=[%.2f %.2f %.2f] | tracks=%d\n', ...
            r1_rmse(1), r1_rmse(2), r1_rmse(3), r1_ratio(1), r1_ratio(2), r1_ratio(3), n1);
        fprintf('  R2 RMSE: T1=%.1f T2=%.1f T3=%.1f km | ratios=[%.2f %.2f %.2f] | tracks=%d\n', ...
            r2_rmse(1), r2_rmse(2), r2_rmse(3), r2_ratio(1), r2_ratio(2), r2_ratio(3), n2);

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        r1_rmse = [NaN NaN NaN];
        r2_rmse = [NaN NaN NaN];
        r1_ratio = [0 0 0];
        r2_ratio = [0 0 0];
        n1 = 0;
        n2 = 0;
    end
end


function [trackSnapshots, trackList] = run_tracker(detList, ukf_tpl, params, n_frames, t_grid, truth_all)
    trackSnapshots = cell(n_frames, 1);
    trackList = {};
    tempPool = {};
    next_id = 1;
    for k = 1:n_frames
        dets = detList{k};
        [trackList, tempPool, trackSnapshots{k}, next_id] = ...
            multi_track_runner_kf(trackList, tempPool, dets, ukf_tpl, params, k, next_id, truth_all, t_grid);
    end
end


function [rmse, ratio] = compute_metrics(snaps, truthTrajs, n_frames, dt_sec)
    n_ac = length(truthTrajs);
    rmse = zeros(1, n_ac);
    ratio = zeros(1, n_ac);
    for ac = 1:n_ac
        tt = truthTrajs{ac};
        % 只评估真值存在的时间段
        t_start = tt.time_sec(1);
        t_end = tt.time_sec(end);
        n_true_frames = floor((t_end - t_start) / dt_sec) + 1;
        if n_true_frames <= 0, n_true_frames = n_frames; end

        ids = [];
        for k = 1:n_frames
            trks = snaps{k}.trackList;
            for t = 1:length(trks)
                if trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
                    ids(end+1) = trks{t}.id;
                end
            end
        end
        ids = unique(ids);

        % 找最佳航迹（在真值时间段内 active 最多的）
        best_id = 0;
        best_active = 0;
        for id = ids
            active = 0;
            for k = 1:min(n_true_frames, n_frames)
                trks = snaps{k}.trackList;
                for t = 1:length(trks)
                    trk = trks{t};
                    if trk.id == id && trk.type ~= 7 && ~isnan(trk.lat)
                        active = active + 1;
                    end
                end
            end
            if active > best_active
                best_active = active;
                best_id = id;
            end
        end

        % 计算 RMSE 和 ratio（只在真值时间段内评估）
        n_err = 0;
        n_assoc = 0;
        n_active = 0;
        sq_sum = 0;
        for k = 1:min(n_true_frames, n_frames)
            tnow = (k-1) * dt_sec;
            tl = interp1(tt.time_sec, tt.lon, tnow, 'linear', 'extrap');
            tb = interp1(tt.time_sec, tt.lat, tnow, 'linear', 'extrap');
            trks = snaps{k}.trackList;
            % 找当前帧离真值最近的航迹
            best_d = inf;
            for t = 1:length(trks)
                trk = trks{t};
                if trk.type ~= 7 && ~isnan(trk.lat)
                    dist = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
                    if dist < best_d
                        best_d = dist;
                        best_trk = trk;
                    end
                end
            end
            if best_d < 80 && best_d < inf
                n_active = n_active + 1;
                sq_sum = sq_sum + best_d^2;
                n_err = n_err + 1;
                if isfield(best_trk, 'assoc_det') && ~isempty(best_trk.assoc_det)
                    n_assoc = n_assoc + 1;
                end
            end
        end
        if n_err > 0
            rmse(ac) = sqrt(sq_sum / n_err);
        else
            rmse(ac) = NaN;
        end
        if n_active > 0
            ratio(ac) = n_assoc / n_active;
        end
    end
end


function n = count_active(trackList)
    n = 0;
    for i = 1:length(trackList)
        if trackList{i}.type ~= 7
            n = n + 1;
        end
    end
end


function print_summary(results)
    fprintf('\n\n========== SUMMARY ==========\n');
    fprintf('%-6s | %-30s | %-30s\n', 'Seed', 'R1 (RMSE km / min ratio)', 'R2 (RMSE km / min ratio)');
    fprintf('%s\n', repmat('-', 1, 80));
    for s = 1:length(results.seeds)
        r1_min_ratio = min(results.r1_ratio(s, :));
        r2_min_ratio = min(results.r2_ratio(s, :));
        fprintf('%-6d | T=[%.1f %.1f %.1f] min=%.2f n=%d | T=[%.1f %.1f %.1f] min=%.2f n=%d\n', ...
            results.seeds(s), ...
            results.r1_rmse(s,1), results.r1_rmse(s,2), results.r1_rmse(s,3), r1_min_ratio, results.r1_n_tracks(s), ...
            results.r2_rmse(s,1), results.r2_rmse(s,2), results.r2_rmse(s,3), r2_min_ratio, results.r2_n_tracks(s));
    end

    fprintf('\nAverage across seeds:\n');
    fprintf('  R1 mean RMSE: T1=%.1f T2=%.1f T3=%.1f km\n', mean(results.r1_rmse(:,1)), mean(results.r1_rmse(:,2)), mean(results.r1_rmse(:,3)));
    fprintf('  R2 mean RMSE: T1=%.1f T2=%.1f T3=%.1f km\n', mean(results.r2_rmse(:,1)), mean(results.r2_rmse(:,2)), mean(results.r2_rmse(:,3)));
    fprintf('  R1 min ratio across all seeds/targets: %.2f\n', min(results.r1_ratio(:)));
    fprintf('  R2 min ratio across all seeds/targets: %.2f\n', min(results.r2_ratio(:)));
end
