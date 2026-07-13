% =========================================================================
% run_scenario_scan.m — 多场景扫描
% =========================================================================
% 在平行分离 / 弱交叉 / 中等交叉 / 强交叉四种场景下分别运行完整仿真，
% 收集单站 RMSE / 关联率 / 融合 RMSE / 匹配对数等指标。
% =========================================================================

function run_scenario_scan()
    addpath(genpath('.'));
    scenarios = {
        'parallel', [128.8, 30.8, 0; 131.6, 31.1, 0], [128.8, 31.6, 0; 131.6, 31.9, 0], [128.8, 32.4, 0; 131.6, 32.7, 0];
        'weak',     [128.8, 31.0, 0; 131.6, 31.7, 0], [128.8, 31.9, 0; 131.6, 31.2, 0], [128.8, 32.5, 0; 131.6, 32.8, 0];
        'medium',   [128.8, 30.7, 0; 131.6, 32.3, 0], [128.8, 31.5, 0; 131.6, 31.5, 0], [128.8, 32.3, 0; 131.6, 30.7, 0];
        'strong',   [128.8, 30.5, 0; 132.0, 32.5, 0], [128.8, 32.5, 0; 132.0, 30.5, 0], [128.8, 31.5, 0; 130.5, 32.9, 0];
    };

    results = struct();
    for s = 1:size(scenarios, 1)
        name = scenarios{s, 1};
        way_A = scenarios{s, 2};
        way_B = scenarios{s, 3};
        way_C = scenarios{s, 4};
        fprintf('\n========== SCENARIO: %s ==========\n', name);
        [r1_rmse, r2_rmse, r1_ratio, r2_ratio, fusion_rmse, n_pairs] = run_one_scenario(way_A, way_B, way_C);
        results(s).name = name;
        results(s).r1_rmse = r1_rmse;
        results(s).r2_rmse = r2_rmse;
        results(s).r1_ratio = r1_ratio;
        results(s).r2_ratio = r2_ratio;
        results(s).fusion_rmse = fusion_rmse;
        results(s).n_pairs = n_pairs;
    end

    print_summary(results);
    save('results/scenario_scan.mat', 'results', '-v7.3');
end


function [r1_rmse, r2_rmse, r1_ratio, r2_ratio, fusion_rmse, n_pairs] = run_one_scenario(way_A, way_B, way_C)
    try
        params = simulation_params_multi();
        params.random_seed = 94;
        params.track_matcher_method = 'dualgate';

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
        truthTrajs{1} = struct('label','A','speed_ms',params.aircraft_speed_ms,'time_sec',tt(:,5),'lat',tt(:,2),'lon',tt(:,1),'lon_rate',tt(:,3),'lat_rate',tt(:,4));
        tt = true_track_B;
        truthTrajs{2} = struct('label','B','speed_ms',params.aircraft_speed_ms,'time_sec',tt(:,5),'lat',tt(:,2),'lon',tt(:,1),'lon_rate',tt(:,3),'lat_rate',tt(:,4));
        tt = true_track_C;
        truthTrajs{3} = struct('label','C','speed_ms',params.aircraft_speed_ms,'time_sec',tt(:,5),'lat',tt(:,2),'lon',tt(:,1),'lon_rate',tt(:,3),'lat_rate',tt(:,4));
        truth_all = {true_track_A, true_track_B, true_track_C};

        % Phase 1: 偏差标定
        rng(params.random_seed);
        adsb_T = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
        adsb_lat = adsb_T.Var2; adsb_lon = adsb_T.Var3;
        dr1=[];da1=[];dr2=[];da2=[];
        for idx = 1:100:height(adsb_T)
            t_lon = adsb_lon(idx); t_lat = adsb_lat(idx);
            if isnan(t_lon)||isnan(t_lat), continue, end
            [in1,~,~] = radar_coverage_check(params.radar1_lon,params.radar1_lat,t_lon,t_lat,params.radar1_beam_center_deg,params);
            if in1
                Rg_true = skywave_geometry('group_range',params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat,t_lon,t_lat);
                Rg_meas = Rg_true + params.radar1_range_bias_m + randn()*params.radar1_range_noise_std_m;
                dr1(end+1) = Rg_meas - Rg_true;
                az_true = sphere_utils_azimuth(params.radar1_lon,params.radar1_lat,t_lon,t_lat);
                az_meas = az_true + params.radar1_azimuth_bias_deg + randn()*params.radar1_azimuth_noise_std_deg;
                daz = az_meas - az_true;
                if daz>180, daz=daz-360; elseif daz<-180, daz=daz+360; end
                da1(end+1) = daz;
            end
            [in2,~,~] = radar_coverage_check(params.radar2_lon,params.radar2_lat,t_lon,t_lat,params.radar2_beam_center_deg,params);
            if in2
                Rg_true = skywave_geometry('group_range',params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat,t_lon,t_lat);
                Rg_meas = Rg_true + params.radar2_range_bias_m + randn()*params.radar2_range_noise_std_m;
                dr2(end+1) = Rg_meas - Rg_true;
                az_true = sphere_utils_azimuth(params.radar2_lon,params.radar2_lat,t_lon,t_lat);
                az_meas = az_true + params.radar2_azimuth_bias_deg + randn()*params.radar2_azimuth_noise_std_deg;
                daz = az_meas - az_true;
                if daz>180, daz=daz-360; elseif daz<-180, daz=daz+360; end
                da2(end+1) = daz;
            end
        end
        dr1_est=mean(dr1); da1_est=mean(da1); dr2_est=mean(dr2); da2_est=mean(da2);

        % Phase 2: 点迹生成
        detList_R1 = cell(n_frames,1); detList_R2 = cell(n_frames,1);
        rng(params.random_seed + 1e7);
        for k=1:n_frames
            t1=t1_grid(k);
            tgt_states=zeros(3,5);
            for ac=1:3
                tt_ac=truth_all{ac};
                if t1>=tt_ac(1,5)&&t1<=tt_ac(end,5)
                    tgt_states(ac,:)=[interp1(tt_ac(:,5),tt_ac(:,1),t1,'linear','extrap'),interp1(tt_ac(:,5),tt_ac(:,2),t1,'linear','extrap'),interp1(tt_ac(:,5),tt_ac(:,3),t1,'linear','extrap'),interp1(tt_ac(:,5),tt_ac(:,4),t1,'linear','extrap'),ac];
                else
                    tgt_states(ac,:)=[NaN NaN NaN NaN ac];
                end
            end
            tgt_states=tgt_states(~isnan(tgt_states(:,1)),:);
            if isempty(tgt_states), detList_R1{k}=[]; continue; end
            detRaw=generate_frame_detections_multi(params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,tgt_states,k,t1,params.radar1_range_bias_m,params.radar1_azimuth_bias_deg,params.radar1_beam_center_deg,params,params.radar1_range_noise_std_m,params.radar1_azimuth_noise_std_deg);
            dets={};
            for d=1:length(detRaw)
                dp=detRaw(d); Rgc=dp.prange-dr1_est; azc=dp.paz-da1_est;
                dp.drange=Rgc; dp.daz=azc; dp.range_meas=Rgc; dp.azimuth_meas=azc;
                if ~isfield(dp,'lat')||isnan(dp.lat)
                    [~,dp.lat,dp.lon]=bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                end
                dets{end+1}=dp;
            end
            detList_R1{k}=[dets{:}];
        end
        rng(params.random_seed + 2e7);
        for k=1:n_frames
            t2=t2_grid(k);
            tgt_states=zeros(3,5);
            for ac=1:3
                tt_ac=truth_all{ac};
                if t2>=tt_ac(1,5)&&t2<=tt_ac(end,5)
                    tgt_states(ac,:)=[interp1(tt_ac(:,5),tt_ac(:,1),t2,'linear','extrap'),interp1(tt_ac(:,5),tt_ac(:,2),t2,'linear','extrap'),interp1(tt_ac(:,5),tt_ac(:,3),t2,'linear','extrap'),interp1(tt_ac(:,5),tt_ac(:,4),t2,'linear','extrap'),ac];
                else
                    tgt_states(ac,:)=[NaN NaN NaN NaN ac];
                end
            end
            tgt_states=tgt_states(~isnan(tgt_states(:,1)),:);
            if isempty(tgt_states), detList_R2{k}=[]; continue; end
            detRaw=generate_frame_detections_multi(params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,tgt_states,k,t2,params.radar2_range_bias_m,params.radar2_azimuth_bias_deg,params.radar2_beam_center_deg,params,params.radar2_range_noise_std_m,params.radar2_azimuth_noise_std_deg);
            dets={};
            for d=1:length(detRaw)
                dp=detRaw(d); Rgc=dp.prange-dr2_est; azc=dp.paz-da2_est;
                dp.drange=Rgc; dp.daz=azc; dp.range_meas=Rgc; dp.azimuth_meas=azc;
                if ~isfield(dp,'lat')||isnan(dp.lat)
                    [~,dp.lat,dp.lon]=bistatic_inverse_solver(Rgc,azc,params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
                end
                dets{end+1}=dp;
            end
            detList_R2{k}=[dets{:}];
        end

        % Phase 5: 跟踪
        params.ukf_range_std_m=params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg=params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale=params.radar1_ukf_Q_scale;
        params.ukf_P_pos_std=params.radar1_ukf_P_pos_std;
        params.ukf_P_vel_std=params.radar1_ukf_P_vel_std;
        params.gate_sigma=params.radar1_gate_sigma;
        params.gate_vr_ms=params.radar1_gate_vr_ms;
        ukf1_tpl=ukf_imm('create',params,params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);

        params_r2=params;
        params_r2.ukf_range_std_m=params.radar2_range_noise_std_m;
        params_r2.ukf_azimuth_std_deg=params.radar2_azimuth_noise_std_deg;
        params_r2.gate_sigma=params.radar2_gate_sigma;
        params_r2.gate_vr_ms=params.radar2_gate_vr_ms;
        params_r2.ukf_Q_scale=params.radar2_ukf_Q_scale;
        params_r2.ukf_P_pos_std=params.radar2_ukf_P_pos_std;
        params_r2.ukf_P_vel_std=params.radar2_ukf_P_vel_std;
        ukf2_tpl=ukf_imm('create',params_r2,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);

        [trackSnapshots_R1, trackList_R1] = run_tracker(detList_R1, ukf1_tpl, params, n_frames, t1_grid, truth_all);
        [trackSnapshots_R2, trackList_R2] = run_tracker(detList_R2, ukf2_tpl, params_r2, n_frames, t2_grid, truth_all);

        % Phase 6: 时间对齐
        aligned_R2 = time_align_tracks(trackSnapshots_R2, params);

        % Phase 7: 匹配 + 融合
        matched_pairs_struct = track_matcher_dualgate(trackSnapshots_R1, aligned_R2, params);
        n_pairs = length(matched_pairs_struct);
        matched_pairs = cell(n_pairs, 1);
        for p=1:n_pairs, matched_pairs{p} = matched_pairs_struct(p); end

        method_names = {'SCC','BC','CI','FCI'};
        all_fused_snapshots = cell(n_pairs, length(method_names));
        for p=1:n_pairs
            for m=1:length(method_names)
                all_fused_snapshots{p,m} = run_track_fusion(matched_pairs{p}, trackSnapshots_R1, aligned_R2, params, method_names{m});
            end
        end

        % Phase 8: 评估
        [r1_rmse, r1_ratio] = compute_metrics(trackSnapshots_R1, truthTrajs, n_frames, params.dt_sec);
        [r2_rmse, r2_ratio] = compute_metrics(trackSnapshots_R2, truthTrajs, n_frames, params.dt_sec);

        % 融合 RMSE（SCC）
        fusion_rmse = compute_fusion_rmse(all_fused_snapshots, matched_pairs, truthTrajs, n_frames, params.dt_sec);

        fprintf('  R1 RMSE: T1=%.1f T2=%.1f T3=%.1f | ratios=[%.2f %.2f %.2f]\n', r1_rmse(1),r1_rmse(2),r1_rmse(3),r1_ratio(1),r1_ratio(2),r1_ratio(3));
        fprintf('  R2 RMSE: T1=%.1f T2=%.1f T3=%.1f | ratios=[%.2f %.2f %.2f]\n', r2_rmse(1),r2_rmse(2),r2_rmse(3),r2_ratio(1),r2_ratio(2),r2_ratio(3));
        fprintf('  匹配对数: %d, SCC 融合 RMSE: %.1f km\n', n_pairs, fusion_rmse);

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        r1_rmse=[NaN NaN NaN]; r2_rmse=[NaN NaN NaN];
        r1_ratio=[0 0 0]; r2_ratio=[0 0 0];
        fusion_rmse=NaN; n_pairs=0;
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
        t_end = tt.time_sec(end);
        n_true_frames = floor((t_end - tt.time_sec(1)) / dt_sec) + 1;
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
                    if d < best_d, best_d = d; best_trk = trk; end
                end
            end
            if best_d < 80
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


function fusion_rmse = compute_fusion_rmse(all_fused, matched_pairs, truthTrajs, n_frames, dt_sec)
    if isempty(matched_pairs)
        fusion_rmse = NaN;
        return;
    end
    % 用 SCC (第1种算法) 的融合结果
    snaps_m = all_fused(:, 1);
    % 把每对匹配映射到真值飞机
    pair_to_ac = zeros(length(matched_pairs), 1);
    for p = 1:length(matched_pairs)
        mp = matched_pairs{p};
        r1_id = mp.R1_track_id;
        % 找该 R1 航迹在各真值飞机下的最近距离
        best_ac = 0; best_d = inf;
        for ac = 1:length(truthTrajs)
            tt = truthTrajs{ac};
            d_sum = 0; n = 0;
            for k = 1:length(snaps_m)
                snap = snaps_m{p};
                fused_k = snap{k};
                if isempty(fused_k.trackList), continue; end
                tnow = (k-1) * dt_sec;
                tl = interp1(tt.time_sec, tt.lon, tnow, 'linear', 'extrap');
                tb = interp1(tt.time_sec, tt.lat, tnow, 'linear', 'extrap');
                for t = 1:length(fused_k.trackList)
                    ftrk = fused_k.trackList{t};
                    if isnan(ftrk.lat), continue; end
                    d = sphere_utils_haversine_distance(ftrk.lon, ftrk.lat, tl, tb) / 1000;
                    d_sum = d_sum + d; n = n + 1;
                    break;
                end
            end
            if n > 0 && d_sum/n < best_d
                best_d = d_sum/n;
                best_ac = ac;
            end
        end
        pair_to_ac(p) = best_ac;
    end
    % 计算融合 RMSE
    sq_sum = 0; n_err = 0;
    for p = 1:length(matched_pairs)
        ac = pair_to_ac(p);
        if ac == 0, continue; end
        tt = truthTrajs{ac};
        snap = snaps_m{p};
        for k = 1:length(snap)
            fused_k = snap{k};
            if isempty(fused_k.trackList), continue; end
            tnow = (k-1) * dt_sec;
            if tnow > tt.time_sec(end), continue; end
            tl = interp1(tt.time_sec, tt.lon, tnow, 'linear', 'extrap');
            tb = interp1(tt.time_sec, tt.lat, tnow, 'linear', 'extrap');
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


function print_summary(results)
    fprintf('\n\n========== SCENARIO SUMMARY ==========\n');
    fprintf('%-10s | %-25s | %-25s | %-12s | %-6s\n', ...
        'Scenario', 'R1 RMSE T1/T2/T3', 'R2 RMSE T1/T2/T3', 'SCC RMSE', 'Pairs');
    fprintf('%s\n', repmat('-', 1, 100));
    for s = 1:length(results)
        r = results(s);
        fprintf('%-10s | %.1f/%.1f/%.1f           | %.1f/%.1f/%.1f           | %-12.1f | %d\n', ...
            r.name, r.r1_rmse(1), r.r1_rmse(2), r.r1_rmse(3), ...
            r.r2_rmse(1), r.r2_rmse(2), r.r2_rmse(3), ...
            r.fusion_rmse, r.n_pairs);
    end
    fprintf('\nMin association ratio across all scenarios/targets:\n');
    fprintf('  R1: %.2f\n', min([results.r1_ratio]));
    fprintf('  R2: %.2f\n', min([results.r2_ratio]));
end
