function quick_check()
    addpath(genpath('D:/Desktop/single_target_with-turn'));
    inp = prepare_oracle_tracking_inputs('single_straight', struct('random_seed',10001));
    p = inp.params;

    tp_def = ukf_imm('create', radar_params(p,1), ...
        p.radar1_lon,p.radar1_lat,p.radar1_tx_lon,p.radar1_tx_lat, ...
        radar_params(p,1).dt_sec);
    disp(['Default imm_cv_dwell: ' num2str(tp_def.params.imm_cv_dwell_time_sec)]);

    rp2 = radar_params(p,1);
    rp2.imm_cv_dwell_time_sec = 2500;
    rp2.imm_ct_fixed_Q_scale = 5.3;
    rp2.imm_transient_gain_max = 11.0;
    rp2.imm_ct_dwell_time_sec = 660;
    rp2.imm_mu_init_CV = 0.5;
    rp2.imm_adapt_mode = '3in1';
    tp_cfg = ukf_imm('create', rp2, ...
        p.radar1_lon,p.radar1_lat,p.radar1_tx_lon,p.radar1_tx_lat,rp2.dt_sec);
    disp(['Config  imm_cv_dwell: ' num2str(tp_cfg.params.imm_cv_dwell_time_sec)]);

    [td1, ~, sn1] = run_oracle_tracker_sequence(inp.detList_R1, tp_def, p, inp.truth_all, inp.t1_grid, false);
    [td2, ~, sn2] = run_oracle_tracker_sequence(inp.detList_R1, tp_cfg, p, inp.truth_all, inp.t1_grid, false);

    % Check if snapshots differ
    match = true;
    for k = 1:min(5,length(sn1))
        t1 = get_first_track(sn1{k});
        t2 = get_first_track(sn2{k});
        if ~isempty(t1) && isfield(t1,'lon') && ~isempty(t2) && isfield(t2,'lon')
            diff_lon = abs(t1.lon - t2.lon);
            diff_lat = abs(t1.lat - t2.lat);
            fprintf('Frame %d: default(%.4f,%.4f) cfg(%.4f,%.4f) dlon=%.6f dlat=%.6f\n', ...
                k, t1.lon, t1.lat, t2.lon, t2.lat, diff_lon, diff_lat);
            if diff_lon > 1e-6 || diff_lat > 1e-6
                match = false;
            end
        end
    end
    fprintf('Tracks immediately diverge: %s\n', num2str(~match));
end

function trk = get_first_track(snap)
    trk = struct();
    if isempty(snap) || ~isfield(snap, 'trackList') || isempty(snap.trackList)
        return;
    end
    trk = snap.trackList{1};
end
