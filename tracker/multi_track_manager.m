% =========================================================================
% multi_track_manager.m
% Multi-target track manager - per-frame dispatch engine
% =========================================================================
% Architecture adapted from myOTHR/modules/mainTrackingEngine.m:
%   1. Separate active/history tracks
%   2. No active tracks -> direct initiation
%   3. Batch UKF predict for all active tracks
%   4. JNN global point-track association
%   5. Update associated tracks (UKF+PDA+fuzzy adaptive Q)
%   6. Update unassociated tracks (pure prediction)
%   7. Quality state machine
%   8. Initiate new tracks from remaining points
% =========================================================================

function [trackList, tempPool, trackSnapshot] = multi_track_manager(...
        trackList, tempPool, detList, ukf_tpl, params, frame_id)

    TYPE_HISTORY = 7;
    trackSnapshot = struct('frameID', frame_id, 'trackList', {{}});

    if isempty(detList)
        for t = 1:length(trackList)
            trk = trackList{t};
            if trk.type == TYPE_HISTORY, continue; end
            trk.ukf.dt = params.dt_sec;
            [x_pred, P_pred, ~, trk.ukf] = ukf_predict_step(trk.ukf);
            trk.ukf.x = x_pred;
            trk.ukf.P = P_pred;
            trk.missed = trk.missed + 1;
            trk.life = trk.life + 1;
            trk.lat = x_pred(3);
            trk.lon = x_pred(1);
            trk.assoc_det = [];
            trackList{t} = trk;
        end
        active_idx = find_active(trackList);
        trackList = manage_track_quality(trackList, active_idx, params, frame_id);
        trackSnapshot.trackList = trackList;
        return;
    end

    % ---- Step 1: Separate active tracks ----
    active_idx = find_active(trackList);

    % ---- Step 2: No active tracks -> direct initiation ----
    if isempty(active_idx)
        [trackList, tempPool] = track_starter_mofn(trackList, tempPool, ...
            detList, ukf_tpl, params, frame_id);
        trackSnapshot.trackList = trackList;
        return;
    end

    % ---- Step 3: Batch UKF predict ----
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.dt = params.dt_sec;
        [x_pred, P_pred, X_pred, trk.ukf] = ukf_predict_step(trk.ukf);

        z_pred = ukf_measurement_model(trk.ukf, x_pred);
        Z_pred = zeros(trk.ukf.m, 2*trk.ukf.n + 1);
        for s = 1:(2*trk.ukf.n + 1)
            Z_pred(:, s) = ukf_measurement_model(trk.ukf, X_pred(:, s));
        end
        P_zz = trk.ukf.R;
        for s = 1:(2*trk.ukf.n + 1)
            dz = Z_pred(:, s) - z_pred;
            P_zz = P_zz + trk.ukf.Wc(s) * (dz * dz');
        end
        if any(isnan(P_zz(:)))
            P_zz = trk.ukf.R;
        end

        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    % ---- Step 4: JNN global association ----
    assoc_pairs = jnn_association(trackList, active_idx, detList, params);
    point_used = false(1, length(detList));
    track_has_assoc = false(1, length(active_idx));
    for p = 1:size(assoc_pairs, 1)
        point_used(assoc_pairs(p, 2)) = true;
        [~, loc] = ismember(assoc_pairs(p, 1), active_idx);
        if loc > 0, track_has_assoc(loc) = true; end
    end

    % ---- Step 5: Update associated tracks ----
    for p = 1:size(assoc_pairs, 1)
        t = assoc_pairs(p, 1);
        d = assoc_pairs(p, 2);
        trk = trackList{t};
        det = detList(d);

        dets_in_gate = {det};
        P_zz_2d = trk.P_zz(1:2, 1:2);
        gate_threshold = params.gate_sigma^2 * 2;
        for j = 1:length(detList)
            if j == d || point_used(j), continue; end
            dp = detList(j);
            if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end
            z_m = [dp.drange; dp.daz];
            innov = z_m - trk.z_pred(1:2);
            if innov(2) > 180, innov(2) = innov(2) - 360;
            elseif innov(2) < -180, innov(2) = innov(2) + 360; end
            if innov' * (P_zz_2d \ innov) < gate_threshold
                dets_in_gate{end+1} = dp;
            end
        end

        [~, ~, trk.ukf, best, nis_val] = ukf_pda_update(trk.ukf, dets_in_gate, ...
            trk.z_pred, trk.Z_pred, trk.X_pred, trk.x_pred, trk.P_pred, ...
            trk.P_zz, params);

        trk.lat = trk.ukf.x(3);
        trk.lon = trk.ukf.x(1);
        trk.missed = 0;
        trk.life = trk.life + 1;
        trk.assoc_det = best;
        trk.nis_history(end+1) = nis_val;

        if length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history(1) = [];
        end

        if params.use_fuzzy_adaptive
            trk.ukf = ukf_fuzzy_adapt(trk.ukf, trk.nis_history, trk.life, params);
        end

        trackList{t} = trk;
    end

    % ---- Step 6: Update unassociated tracks (pure prediction) ----
    for i = 1:length(active_idx)
        if track_has_assoc(i), continue; end
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.x = trk.x_pred;
        trk.ukf.P = trk.P_pred;
        trk.missed = trk.missed + 1;
        trk.life = trk.life + 1;
        trk.lat = trk.ukf.x(3);
        trk.lon = trk.ukf.x(1);
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    % ---- Step 7: Quality state machine ----
    trackList = manage_track_quality(trackList, active_idx, params, frame_id);

    % ---- Step 8: Initiate new tracks from remaining points ----
    unused_dets = detList(~point_used);
    if ~isempty(unused_dets)
        [trackList, tempPool] = track_starter_mofn(trackList, tempPool, ...
            unused_dets, ukf_tpl, params, frame_id);
    else
        tempPool = cleanup_stale(tempPool, frame_id, params.tracker_N);
    end

    trackSnapshot.trackList = trackList;
end

function idx = find_active(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7
            idx(end+1) = t;
        end
    end
end

function tempPool = cleanup_stale(tempPool, current_frame, N)
    keep = true(1, length(tempPool));
    for c = 1:length(tempPool)
        if current_frame - tempPool{c}.lastFrame > N
            keep(c) = false;
        end
    end
    tempPool = tempPool(keep);
end
