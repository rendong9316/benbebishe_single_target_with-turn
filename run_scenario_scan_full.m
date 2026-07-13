% =========================================================================
% run_scenario_scan_full.m — 多场景 × 多种子完整扫描
% =========================================================================
% 目的：验证多目标起始、关联、滤波、匹配、融合的全链路稳定性
%
% 场景矩阵：5种 × 10个种子 = 50次仿真
% 指标：R1/R2 RMSE(3目标) / 关联率 / 航迹数 / 匹配对数 / 融合RMSE
% =========================================================================

function run_scenario_scan_full()
    addpath(genpath('.'));

    % 场景定义：{name, way_A, way_B, way_C}
    scenario_defs = {
        'strong_cross'  [128.8, 30.5, 0; 132.0, 32.5, 0]  [128.8, 32.5, 0; 132.0, 30.5, 0]  [128.8, 31.5, 0; 130.5, 32.9, 0];
        'parallel_sep'  [128.8, 30.5, 0; 131.5, 31.0, 0]  [129.0, 30.5, 0; 131.7, 31.0, 0]  [129.2, 30.5, 0; 131.9, 31.0, 0];
        'converge'      [128.8, 30.5, 0; 130.5, 31.5, 0]  [131.5, 30.5, 0; 130.5, 31.5, 0]  [130.0, 32.8, 0; 130.5, 31.5, 0];
        'speed_diff'    [128.8, 30.5, 0; 131.8, 32.2, 0]  [128.8, 32.0, 0; 130.8, 30.8, 0]  [128.8, 31.2, 0; 130.0, 31.8, 0];
        'cross_180'     [128.8, 30.5, 0; 131.8, 32.0, 0]  [131.8, 30.5, 0; 128.8, 32.0, 0]  [129.8, 31.5, 0; 130.8, 31.5, 0];
    };

    n_scenarios = 5;

    % 种子列表
    seeds = [7, 13, 23, 42, 55, 77, 88, 94, 101, 256];
    n_seeds = length(seeds);

    % 结果结构：results(scenario, seed)
    results = cell(n_scenarios, n_seeds);

    for sc = 1:n_scenarios
        sname = scenario_defs{sc, 1};
        way_A = scenario_defs{sc, 2};
        way_B = scenario_defs{sc, 3};
        way_C = scenario_defs{sc, 4};
        fprintf('\n\n############################################################################\n');
        fprintf('#  SCENARIO %d/%d: %s\n', sc, n_scenarios, sname);
        fprintf('############################################################################\n');

        for sd = 1:n_seeds
            seed = seeds(sd);
            fprintf('\n  --- Seed %d/%d (%d) ---\n', sd, n_seeds, seed);
            metrics = run_one_run(sname, way_A, way_B, way_C, seed);
            results{sc, sd} = metrics;

            % 每完成一个就保存，防止意外中断丢失
            if mod(sd, 5) == 0 || sd == n_seeds
                save_partial(results, seeds, n_scenarios, n_seeds);
            end
        end
    end

    save('results/scan_full.mat', 'results', 'seeds', 'n_scenarios', 'n_seeds', '-v7.3');
    print_full_summary(results, seeds, n_scenarios, n_seeds);
end


function metrics = run_one_run(sname, way_A, way_B, way_C, seed)
    metrics = struct();
    metrics.scenario = sname;
    metrics.seed = seed;

    try
        params = simulation_params_multi();
        params.random_seed = seed;
        params.track_matcher_method = 'dualgate';

        % === Phase 0: 场景初始化 ===
        traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
        traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
        traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);
        true_track_A = aircraft_trajectory_interpolate('generate', traj_A);
        true_track_B = aircraft_trajectory_interpolate('generate', traj_B);
        true_track_C = aircraft_trajectory_interpolate('generate', traj_C);

        t1_grid = params.time_offset_radar1_sec : params.dt_sec : ...
            max(max(traj_A.duration_sec, traj_B.duration_sec), traj_C.duration_sec);
        t2_grid = params.time_offset_radar2_sec : params.dt_sec : ...
            max(max(traj_A.duration_sec, traj_B.duration_sec), traj_C.duration_sec);
        n_frames = min(length(t1_grid), length(t2_grid));

        truthTrajs = cell(3, 1);
        for ac = 1:3
            switch ac
                case 1, tt = true_track_A; lbl = 'A';
                case 2, tt = true_track_B; lbl = 'B';
                case 3, tt = true_track_C; lbl = 'C';
            end
            truthTrajs{ac} = struct('label', lbl, 'speed_ms', params.aircraft_speed_ms, ...
                'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
                'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
        end
        truth_all = {true_track_A, true_track_B, true_track_C};

        % === Phase 1: 偏差标定 ===
        rng(seed);
        adsb_T = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
        adsb_lat = adsb_T.Var2; adsb_lon = adsb_T.Var3;
        dr1=[]; da1=[]; dr2=[]; da2=[];
        cal_step = max(1, floor(height(adsb_T) / 5000));
        for idx = 1:cal_step:height(adsb_T)
            t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
            if isnan(t_lon) || isnan(t_lat), continue; end
            [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
                t_lon, t_lat, params.radar1_beam_center_deg, params);
            if in1
                Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                    params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
                dr1(end+1) = Rg_meas - Rg_true;
                az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
                az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
                daz = az_meas - az_true;
                if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
                da1(end+1) = daz;
            end
            [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
                t_lon, t_lat, params.radar2_beam_center_deg, params);
            if in2
                Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                    params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
                dr2(end+1) = Rg_meas - Rg_true;
                az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
                az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
                daz = az_meas - az_true;
                if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
                da2(end+1) = daz;
            end
        end
        dr1_est = mean(dr1); da1_est = mean(da1);
        dr2_est = mean(dr2); da2_est = mean(da2);

        % === Phase 2: 点迹生成 ===
        detList_R1 = cell(n_frames, 1);
        detList_R2 = cell(n_frames, 1);

        rng(seed + 1e7);
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
                    [~,dp.lat,dp.lon] = bistatic_inverse_solver(Rgc,azc,...
                        params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                end
                dets{end+1} = dp;
            end
            detList_R1{k} = [dets{:}];
        end

        rng(seed + 2e7);
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
                    [~,dp.lat,dp.lon] = bistatic_inverse_solver(Rgc,azc,...
                        params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
                end
                dets{end+1} = dp;
            end
            detList_R2{k} = [dets{:}];
        end

        % === Phase 5: 跟踪 ===
        params.ukf_range_std_m = params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale = params.radar1_ukf_Q_scale;
        params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
        params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
        params.gate_sigma = params.radar1_gate_sigma;
        params.gate_vr_ms = params.radar1_gate_vr_ms;
        ukf1_tpl = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

        params_r2 = params;
        params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        params_r2.gate_sigma = params.radar2_gate_sigma;
        params_r2.gate_vr_ms = params.radar2_gate_vr_ms;
        params_r2.ukf_Q_scale = params.radar2_ukf_Q_scale;
        params_r2.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
        params_r2.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
        ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

        [trackSnapshots_R1, trackList_R1] = run_tracker(detList_R1, ukf1_tpl, params, n_frames, t1_grid, truth_all);
        [trackSnapshots_R2, trackList_R2] = run_tracker(detList_R2, ukf2_tpl, params_r2, n_frames, t2_grid, truth_all);

        % === Phase 6: 时间对齐 ===
        aligned_R2 = time_align_tracks(trackSnapshots_R2, params);

        % === Phase 7: 匹配 + 融合 ===
        matched_pairs_struct = track_matcher_dualgate(trackSnapshots_R1, aligned_R2, params);
        n_pairs = length(matched_pairs_struct);
        matched_pairs = cell(n_pairs, 1);
        for p = 1:n_pairs
            matched_pairs{p} = matched_pairs_struct(p);
        end

        method_names = {'SCC','BC','CI','FCI'};
        all_fused_snapshots = cell(n_pairs, length(method_names));
        for p = 1:n_pairs
            for m = 1:length(method_names)
                all_fused_snapshots{p,m} = run_track_fusion(matched_pairs{p}, ...
                    trackSnapshots_R1, aligned_R2, params, method_names{m});
            end
        end

        % === Phase 8: 评估 ===
        [r1_rmse, r1_ratio] = compute_metrics_local(trackSnapshots_R1, truthTrajs, n_frames, params.dt_sec);
        [r2_rmse, r2_ratio] = compute_metrics_local(trackSnapshots_R2, truthTrajs, n_frames, params.dt_sec);

        % 融合 RMSE (SCC) — 从 matched_pairs_struct 中提取 R1/R2 trackList
        fusion_rmse = compute_fusion_rmse_local(all_fused_snapshots, matched_pairs_struct, ...
            trackSnapshots_R1, trackSnapshots_R2, truthTrajs, n_frames, params.dt_sec, aligned_R2);

        % 航迹数
        n1 = count_active_local(trackList_R1);
        n2 = count_active_local(trackList_R2);

        % 每帧检测数统计
        avg_dets_r1 = mean(cellfun(@length, detList_R1));
        avg_dets_r2 = mean(cellfun(@length, detList_R2));

        metrics.r1_rmse = r1_rmse(1);
        metrics.r2_rmse = r2_rmse(1);
        metrics.r1_ratio = r1_ratio(1);
        metrics.r2_ratio = r2_ratio(1);
        metrics.fusion_rmse = fusion_rmse;
        metrics.n_pairs = n_pairs;
        metrics.n1_tracks = n1;
        metrics.n2_tracks = n2;
        metrics.avg_dets_r1 = avg_dets_r1;
        metrics.avg_dets_r2 = avg_dets_r2;
        metrics.success = true;
        metrics.error = '';
        metrics.all_r1_rmse = r1_rmse;
        metrics.all_r2_rmse = r2_rmse;
        metrics.all_r1_ratio = r1_ratio;
        metrics.all_r2_ratio = r2_ratio;

        fprintf('  R1 RMSE: T1=%.1f T2=%.1f T3=%.1f | ratios=[%.2f %.2f %.2f] | n=%d | avg_dets=%.1f\n', ...
            metrics.all_r1_rmse(1), metrics.all_r1_rmse(2), metrics.all_r1_rmse(3), ...
            metrics.all_r1_ratio(1), metrics.all_r1_ratio(2), metrics.all_r1_ratio(3), n1, avg_dets_r1);
        fprintf('  R2 RMSE: T1=%.1f T2=%.1f T3=%.1f | ratios=[%.2f %.2f %.2f] | n=%d | avg_dets=%.1f\n', ...
            metrics.all_r2_rmse(1), metrics.all_r2_rmse(2), metrics.all_r2_rmse(3), ...
            metrics.all_r2_ratio(1), metrics.all_r2_ratio(2), metrics.all_r2_ratio(3), n2, avg_dets_r2);
        fprintf('  Pairs=%d, Fusion(SCC) RMSE=%.1f km\n\n', n_pairs, fusion_rmse);

    catch ME
        fprintf('  FAILED: %s\n', ME.message);
        metrics.all_r1_rmse = nan(1,3);
        metrics.all_r2_rmse = nan(1,3);
        metrics.all_r1_ratio = [0 0 0];
        metrics.all_r2_ratio = [0 0 0];
        metrics.fusion_rmse = NaN;
        metrics.n_pairs = 0;
        metrics.n1_tracks = 0;
        metrics.n2_tracks = 0;
        metrics.avg_dets_r1 = 0;
        metrics.avg_dets_r2 = 0;
        metrics.success = false;
        metrics.error = ME.message;
        fprintf('  FAILED: %s\n', metrics.error);
    end
end


function [rmse, ratio] = compute_metrics_local(snaps, truthTrajs, n_frames, dt_sec)
    n_ac = length(truthTrajs);
    rmse = zeros(1, n_ac);
    ratio = zeros(1, n_ac);
    for ac = 1:n_ac
        tt = truthTrajs{ac};
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

        best_id = 0; best_active = 0;
        for id = ids
            active = 0;
            for k = 1:min(n_true_frames, n_frames)
                trks = snaps{k}.trackList;
                for t = 1:length(trks)
                    if trks{t}.id == id && trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
                        active = active + 1;
                    end
                end
            end
            if active > best_active
                best_active = active;
                best_id = id;
            end
        end

        n_err = 0; n_assoc = 0; n_active = 0; sq_sum = 0;
        for k = 1:min(n_true_frames, n_frames)
            tnow = (k-1) * dt_sec;
            tl = interp1(tt.time_sec, tt.lon, tnow, 'linear', 'extrap');
            tb = interp1(tt.time_sec, tt.lat, tnow, 'linear', 'extrap');
            trks = snaps{k}.trackList;
            best_d = inf; best_trk = [];
            for t = 1:length(trks)
                trk = trks{t};
                if trk.type ~= 7 && ~isnan(trk.lat)
                    d = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
                    if d < best_d
                        best_d = d;
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
        if n_err > 0, rmse(ac) = sqrt(sq_sum / n_err); else, rmse(ac) = NaN; end
        if n_active > 0, ratio(ac) = n_assoc / n_active; end
    end
end


function fusion_rmse = compute_fusion_rmse_local(all_fused, matched_pairs, ...
        snap_R1, snap_R2, truthTrajs, n_frames, dt_sec, aligned_R2)
    if isempty(matched_pairs) || length(matched_pairs) == 0
        fusion_rmse = NaN;
        return;
    end
    n_pairs = length(matched_pairs);
    n_ac = length(truthTrajs);
    frame_times = (0:n_frames-1) * dt_sec;

    % 用 evaluate_all_multi 的逻辑映射 pair -> aircraft
    matcher = struct();
    matcher.pair_to_aircraft = zeros(n_pairs, 1);
    for p = 1:n_pairs
        mp = matched_pairs(p);
        r1_id = mp.R1_track_id;
        best_ac = 0; best_d = inf;
        for ac = 1:n_ac
            tt = truthTrajs{ac};
            d_sum = 0; n = 0;
            for k = 1:n_frames
                tnow = frame_times(k);
                if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
                tl = interp1(tt.time_sec, tt.lon, tnow, 'linear', 'extrap');
                tb = interp1(tt.time_sec, tt.lat, tnow, 'linear', 'extrap');
                trks = snap_R1{k}.trackList;
                for t = 1:length(trks)
                    if ~isnan(trks{t}.lon)
                        d = sphere_utils_haversine_distance(trks{t}.lon, trks{t}.lat, tl, tb) / 1000;
                        d_sum = d_sum + d; n = n + 1;
                        break;
                    end
                end
            end
            if n > 0 && d_sum/n < best_d
                best_d = d_sum/n;
                best_ac = ac;
            end
        end
        matcher.pair_to_aircraft(p) = best_ac;
    end

    % 计算融合 RMSE（SCC = method 1）
    sq_sum = 0; n_err = 0;
    for p = 1:n_pairs
        ac = matcher.pair_to_aircraft(p);
        if ac == 0, continue; end
        tt = truthTrajs{ac};
        fused_snaps = all_fused{p, 1}; % SCC
        for k = 1:n_frames
            tnow = frame_times(k);
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
            tl = interp1(tt.time_sec, tt.lon, tnow, 'linear', 'extrap');
            tb = interp1(tt.time_sec, tt.lat, tnow, 'linear', 'extrap');
            fused_k = fused_snaps{k};
            if isempty(fused_k.trackList), continue; end
            for t = 1:length(fused_k.trackList)
                ftrk = fused_k.trackList{t};
                if isnan(ftrk.lat), continue; end
                d = sphere_utils_haversine_distance(ftrk.lon, ftrk.lat, tl, tb) / 1000;
                if d < 100
                    sq_sum = sq_sum + d^2;
                    n_err = n_err + 1;
                end
                break;
            end
        end
    end
    if n_err > 0
        fusion_rmse = sqrt(sq_sum / n_err);
    else
        fusion_rmse = NaN;
    end
end


function n = count_active_local(trackList)
    n = 0;
    for i = 1:length(trackList)
        if trackList{i}.type ~= 7
            n = n + 1;
        end
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


function save_partial(results, seeds, n_scenarios, n_seeds)
    save('results/scan_full_partial.mat', 'results', 'seeds', '-v7.3');
end


function print_full_summary(results, seeds, n_scenarios, n_seeds)
    fprintf('\n\n');
    fprintf('############################################################################\n');
    fprintf('#             MULTI-SCENARIO MULTI-SEED SCAN — FULL SUMMARY                #\n');
    fprintf('############################################################################\n');

    scenario_names = {'strong_cross', 'parallel_sep', 'converge', 'speed_diff', 'cross_180'};

    % 按场景统计
    fprintf('\n=== Per-Scenario Statistics (over %d seeds) ===\n\n', n_seeds);

    for sc = 1:n_scenarios
        fprintf('--- Scenario %d: %s ---\n', sc, char(scenario_names{sc}));

        % 收集成功运行的种子
        r1_all = zeros(n_seeds, 3);
        r2_all = zeros(n_seeds, 3);
        ratios_r1 = zeros(n_seeds, 3);
        ratios_r2 = zeros(n_seeds, 3);
        fusion_all = zeros(n_seeds, 1);
        pairs_all = zeros(n_seeds, 1);
        ntracks_all = zeros(n_seeds, 2);
        success_count = 0;

        for sd = 1:n_seeds
            m = results{sc, sd};
            if ~isstruct(m) || ~isfield(m, 'success') || ~m.success
                continue;
            end
            success_count = success_count + 1;
            r1_all(success_count, 1) = m.all_r1_rmse(1);
            r1_all(success_count, 2) = m.all_r1_rmse(2);
            r1_all(success_count, 3) = m.all_r1_rmse(3);
            r2_all(success_count, 1) = m.all_r2_rmse(1);
            r2_all(success_count, 2) = m.all_r2_rmse(2);
            r2_all(success_count, 3) = m.all_r2_rmse(3);
            ratios_r1(success_count, 1) = m.all_r1_ratio(1);
            ratios_r1(success_count, 2) = m.all_r1_ratio(2);
            ratios_r1(success_count, 3) = m.all_r1_ratio(3);
            ratios_r2(success_count, 1) = m.all_r2_ratio(1);
            ratios_r2(success_count, 2) = m.all_r2_ratio(2);
            ratios_r2(success_count, 3) = m.all_r2_ratio(3);
            fusion_all(success_count) = m.fusion_rmse;
            pairs_all(success_count) = m.n_pairs;
            ntracks_all(success_count, 1) = m.n1_tracks;
            ntracks_all(success_count, 2) = m.n2_tracks;
        end

        if success_count == 0
            fprintf('  ALL FAILED!\n\n');
            continue;
        end

        fprintf('  Success: %d/%d\n', success_count, n_seeds);

        for ac = 1:3
            r1m = mean(r1_all(1:success_count, ac));
            r1s = std(r1_all(1:success_count, ac));
            r2m = mean(r2_all(1:success_count, ac));
            r2s = std(r2_all(1:success_count, ac));
            fprintf('  Target %d: R1 RMSE=%.1f±%.1f km | R2 RMSE=%.1f±%.1f km\n', ...
                ac, r1m, r1s, r2m, r2s);
        end

        r1_min_ratio = min(min(ratios_r1(1:success_count, :)));
        r2_min_ratio = min(min(ratios_r2(1:success_count, :)));
        fprintf('  Min association ratio: R1=%.2f | R2=%.2f\n', r1_min_ratio, r2_min_ratio);

        fm = mean(fusion_all(1:success_count));
        fs = std(fusion_all(1:success_count));
        fprintf('  Fusion(SCC) RMSE: %.1f±%.1f km\n', fm, fs);

        pm = mean(pairs_all(1:success_count));
        ps = std(pairs_all(1:success_count));
        fprintf('  Match pairs: %.1f±%.1f\n', pm, ps);

        nm1 = mean(ntracks_all(1:success_count, 1));
        ns1 = std(ntracks_all(1:success_count, 1));
        nm2 = mean(ntracks_all(1:success_count, 2));
        ns2 = std(ntracks_all(1:success_count, 2));
        fprintf('  Active tracks: R1=%.1f±%.1f | R2=%.1f±%.1f\n\n', nm1, ns1, nm2, ns2);
    end

    % 全局排名
    fprintf('\n=== Scenario Stability Ranking (by R1 RMSE mean) ===\n');
    rank_data = struct();
    for sc = 1:n_scenarios
        rmse_sum = 0; count = 0;
        for sd = 1:n_seeds
            m = results{sc, sd};
            if ~isstruct(m) || ~isfield(m, 'success') || ~m.success, continue; end
            valid = ~isnan(m.all_r1_rmse);
            rmse_sum = rmse_sum + sum(m.all_r1_rmse(valid));
            count = count + sum(valid);
        end
        if count > 0
            rank_data(sc).mean_rmse = rmse_sum / count;
            rank_data(sc).name = char(scenario_names{sc});
        end
    end
    [sorted_rmse, idx] = sort([rank_data.mean_rmse]);
    for r = 1:length(idx)
        fprintf('  #%d: %s (mean RMSE=%.1f km)\n', r, rank_data(idx(r)).name, sorted_rmse(r));
    end

    fprintf('\nDone. Results saved to results/scan_full.mat\n');
end
