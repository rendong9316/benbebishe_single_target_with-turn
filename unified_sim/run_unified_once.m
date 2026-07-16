function result = run_unified_once(params)
% run_unified_once — 单次统一流水线核心执行函数，不绘图、不保存

    user_pd = [];
    if isfield(params, 'detection_probability')
        user_pd = params.detection_probability;
    end
    user_seed = [];
    if isfield(params, 'random_seed')
        user_seed = params.random_seed;
    end
    params = apply_scenario_params(params);
    if ~isempty(user_pd)
        params.detection_probability = user_pd;
    end
    if ~isempty(user_seed)
        params.random_seed = user_seed;
    end

    scenario = build_truth_scenario(params);
    truth_all_cell = scenario.truth_all_cell;
    truthTrajs = scenario.truthTrajs;
    t1_grid = scenario.t1_grid;
    t2_grid = scenario.t2_grid;
    n_frames = scenario.n_frames;

    [dr1_est, da1_est, dr2_est, da2_est] = estimate_adsb_bias(params);

    radar1_cfg = struct('radar_lon', params.radar1_lon, 'radar_lat', params.radar1_lat, ...
        'tx_lon', params.radar1_tx_lon, 'tx_lat', params.radar1_tx_lat, ...
        'range_bias_m', params.radar1_range_bias_m, ...
        'azimuth_bias_deg', params.radar1_azimuth_bias_deg, ...
        'beam_center_deg', params.radar1_beam_center_deg, ...
        'range_noise_std_m', params.radar1_range_noise_std_m, ...
        'azimuth_noise_std_deg', params.radar1_azimuth_noise_std_deg);
    radar2_cfg = struct('radar_lon', params.radar2_lon, 'radar_lat', params.radar2_lat, ...
        'tx_lon', params.radar2_tx_lon, 'tx_lat', params.radar2_tx_lat, ...
        'range_bias_m', params.radar2_range_bias_m, ...
        'azimuth_bias_deg', params.radar2_azimuth_bias_deg, ...
        'beam_center_deg', params.radar2_beam_center_deg, ...
        'range_noise_std_m', params.radar2_range_noise_std_m, ...
        'azimuth_noise_std_deg', params.radar2_azimuth_noise_std_deg);

    detList_R1 = generate_all_radar_detections(params, truth_all_cell, t1_grid(1:n_frames), ...
        radar1_cfg, dr1_est, da1_est, 1e7);
    detList_R2 = generate_all_radar_detections(params, truth_all_cell, t2_grid(1:n_frames), ...
        radar2_cfg, dr2_est, da2_est, 2e7);

    params_r1 = configure_radar_filter_params(params, 1);
    params_r2 = configure_radar_filter_params(params, 2);
    ukf1_tpl = create_ukf_template_local(params_r1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl = create_ukf_template_local(params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    [trackSnapshots_R1, trackList_R1] = run_tracker_for_radar(detList_R1, ukf1_tpl, params_r1, ...
        truth_all_cell, t1_grid, n_frames);
    [trackSnapshots_R2, trackList_R2] = run_tracker_for_radar(detList_R2, ukf2_tpl, params_r2, ...
        truth_all_cell, t2_grid, n_frames);

    aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
    matched_pairs = track_matcher(trackSnapshots_R1, aligned_R2, params);

    method_names = {'SCC', 'BC', 'CI', 'FCI'};
    all_fused_snapshots = cell(length(method_names), 1);
    for m = 1:length(method_names)
        all_fused_snapshots{m} = run_track_fusion(matched_pairs, ...
            trackSnapshots_R1, aligned_R2, params, method_names{m});
    end

    matcher_simple = struct();
    matcher_simple.matched_pairs = matched_pairs;
    matcher_simple.aligned_R2 = aligned_R2;
    if isempty(matched_pairs)
        matcher_simple.r1_ids = [];
        matcher_simple.r2_ids = [];
    else
        matcher_simple.r1_ids = [matched_pairs.R1_track_id];
        matcher_simple.r2_ids = [matched_pairs.R2_track_id];
    end
    matcher_simple.r1_pos = extract_track_positions_by_ids_local(trackSnapshots_R1, matcher_simple.r1_ids, n_frames);

    fusion_eval = evaluate_all('fusion', all_fused_snapshots, method_names, ...
        matched_pairs, trackSnapshots_R1, trackSnapshots_R2, ...
        truthTrajs, n_frames, params.dt_sec, matcher_simple);
    errorStats_R1 = evaluate_all('tracking_errors', trackSnapshots_R1, detList_R1, ...
        truthTrajs, n_frames, params.dt_sec, 'R1', t1_grid(1:n_frames));
    errorStats_R2 = evaluate_all('tracking_errors', trackSnapshots_R2, detList_R2, ...
        truthTrajs, n_frames, params.dt_sec, 'R2', t2_grid(1:n_frames));

    rmse_fusion = nan(1, length(method_names));
    for m = 1:length(method_names)
        rmse_fusion(m) = fusion_eval.overall(m).s.rms;
    end
    [best_fusion_rmse, best_m] = min(rmse_fusion);

    result = struct();
    result.params = params;
    result.scenario = scenario;
    result.n_frames = n_frames;
    result.trackSnapshots_R1 = trackSnapshots_R1;
    result.trackSnapshots_R2 = trackSnapshots_R2;
    result.aligned_R2 = aligned_R2;
    result.matched_pairs = matched_pairs;
    result.all_fused_snapshots = all_fused_snapshots;
    result.method_names = method_names;
    result.fusion_eval = fusion_eval;
    result.errorStats_R1 = errorStats_R1;
    result.errorStats_R2 = errorStats_R2;
    result.best_m = best_m;
    result.best_fusion_rmse = best_fusion_rmse;
    result.rmse_fusion = rmse_fusion;
    result.rmse_R1 = errorStats_R1.overall.ukf.rms;
    result.rmse_R2 = errorStats_R2.overall.ukf.rms;
    result.n_tracks_R1 = length(trackList_R1);
    result.n_tracks_R2 = length(trackList_R2);
end


function params = configure_radar_filter_params(params, radar_id)
    if radar_id == 1
        params.ukf_range_std_m = params.radar1_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale = params.radar1_ukf_Q_scale;
        params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
        params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
        params.gate_sigma = params.radar1_gate_sigma;
        params.gate_vr_ms = params.radar1_gate_vr_ms;
        params.tracker_K_loss = params.radar1_tracker_K_loss;
    else
        params.ukf_range_std_m = params.radar2_range_noise_std_m;
        params.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
        params.ukf_Q_scale = params.radar2_ukf_Q_scale;
        params.ukf_P_pos_std = params.radar2_ukf_P_pos_std;
        params.ukf_P_vel_std = params.radar2_ukf_P_vel_std;
        params.gate_sigma = params.radar2_gate_sigma;
        params.gate_vr_ms = params.radar2_gate_vr_ms;
        params.tracker_K_loss = params.radar2_tracker_K_loss;
    end
end


function ukf_tpl = create_ukf_template_local(params, radar_lon, radar_lat, tx_lon, tx_lat, dt_sec)
    backend = 'zishiying';
    if isfield(params, 'ukf_backend')
        backend = lower(params.ukf_backend);
    end
    if contains(backend, 'imm')
        ukf_tpl = ukf_imm('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt_sec);
    else
        ukf_tpl = ukf_zishiying('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt_sec);
    end
end


function [trackSnapshots, trackList] = run_tracker_for_radar(detList, ukf_tpl, params, truth_all_cell, t_grid, n_frames)
    trackList = {};
    tempPool = {};
    next_id = 1;
    trackSnapshots = cell(n_frames, 1);
    for k = 1:n_frames
        [trackList, tempPool, trackSnapshots{k}, next_id] = ...
            multi_track_runner_kf(trackList, tempPool, detList{k}, ukf_tpl, ...
            params, k, next_id, truth_all_cell, t_grid);
    end
end


function pos = extract_track_positions_by_ids_local(trackSnapshots, track_ids, n_frames)
    pos = nan(length(track_ids), n_frames, 2);
    for p = 1:length(track_ids)
        track_id = track_ids(p);
        for k = 1:n_frames
            snap = trackSnapshots{k};
            if isempty(snap) || ~isfield(snap, 'trackList') || isempty(snap.trackList)
                continue;
            end
            for t = 1:length(snap.trackList)
                trk = snap.trackList{t};
                if isfield(trk, 'id') && trk.id == track_id && ~isnan(trk.lat)
                    pos(p, k, 1) = trk.lon;
                    pos(p, k, 2) = trk.lat;
                    break;
                end
            end
        end
    end
end
