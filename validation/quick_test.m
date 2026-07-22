function quick_test()
    addpath(genpath('D:/Desktop/single_target_with-turn'));
    params = simulation_params_oracle();
    prepared = prepare_oracle_tracking_inputs('single_turn', struct('random_seed', 42));
    inp = prepared;

    tpl = ukf_imm('create', radar_params(params, 1), ...
        params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    [tracks, ~, snaps] = run_oracle_tracker_sequence( ...
        inp.detList_R1, tpl, radar_params(params, 1), inp.truth_all, inp.t1_grid, false);

    pos_sq = [];
    for f = 1:numel(snaps)
        if isempty(snaps{f}) || ~isfield(snaps{f}, 'trackList'), continue; end
        for ti = 1:numel(snaps{f}.trackList)
            trk = snaps{f}.trackList{ti};
            if ~trk.updated || ~isfinite(trk.combined_nis), continue; end
            truth = inp.truthTrajs{trk.truth_idx};
            if isempty(truth), continue; end
            t_now = inp.t1_grid(f);
            if t_now < truth.time_sec(1) || t_now > truth.time_sec(end), continue; end
            tl = interp1(truth.time_sec, truth.lat, t_now, 'linear');
            tlon = interp1(truth.time_sec, truth.lon, t_now, 'linear');
            if ~all(isfinite([tl, tlon])), continue; end
            pos_err = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), tlon, tl);
            pos_sq(end+1) = pos_err^2;
        end
    end

    fprintf('pos_rmse=%.3f km samples=%d\n', sqrt(mean(pos_sq))/1000, numel(pos_sq));
end

function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
