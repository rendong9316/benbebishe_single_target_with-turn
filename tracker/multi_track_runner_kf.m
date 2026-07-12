% =========================================================================
% multi_track_runner_kf.m — 多目标逐帧跟踪主循环
% =========================================================================
% 多目标专属航迹生命周期：预测 → JPDA → 更新 → 质量维护 → M/N 起始。
% =========================================================================

function [trackList, tempPool, snap, next_id] = multi_track_runner_kf( ...
        trackList, tempPool, detList_k, ukf_tpl, params, frame_id, next_id, varargin)

    TYPE_RELIABLE = 1;
    TYPE_MAINTAIN = 2;
    TYPE_TEMPORARY = 6;
    TYPE_HISTORY = 7;

    if isempty(trackList)
        trackList = {};
    end
    if isempty(tempPool)
        tempPool = {};
    end
    detList_k = filter_valid_dets(detList_k);

    if isempty(trackList) && get_param(params, 'multi_truth_init_enable', false) && length(varargin) >= 2
        truth_all = varargin{1};
        t_grid = varargin{2};
        [trackList, detList_k, next_id] = truth_init_tracks(trackList, detList_k, ...
            ukf_tpl, params, frame_id, next_id, truth_all, t_grid);
        truth_reinit_state = struct('gap_count', zeros(1, length(varargin{1})));
    elseif length(varargin) >= 2 && get_param(params, 'multi_truth_reinit_enable', false)
        truth_all = varargin{1};
        t_grid = varargin{2};
        [trackList, detList_k, next_id] = truth_reinit_lost(trackList, detList_k, ...
            ukf_tpl, params, frame_id, next_id, truth_all, t_grid);
    end

    active_idx = get_active_indices(trackList, TYPE_HISTORY);
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.dt = params.dt_sec;
        trk.ukf.life_count = trk.life + 1;
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ukf_dispatch('prepare', trk.ukf);
        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    [assoc, det_used_prob] = jpda_multi(trackList, active_idx, detList_k, params);
    track_has_assoc = false(1, length(active_idx));
    update_det_used = false(1, length(detList_k));

    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        ai = assoc{i};
        has_assoc = ~isempty(ai.det_indices) && sum(ai.betas) > get_param(params, 'jpda_min_update_prob', 0.05);

        if has_assoc
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, ai.innov_w);
            trk.lat = trk.ukf.x(3);
            trk.lon = trk.ukf.x(1);
            trk.missed = 0;
            trk.assoc_det = detList_k(ai.best_det_index);
            if ai.best_det_index > 0
                update_det_used(ai.best_det_index) = true;
            end
            nis_val = ai.nis;
            track_has_assoc(i) = true;
        else
            best_j = fallback_nearest_det(trk, detList_k, params);
            if best_j > 0
                dp = detList_k(best_j);
                innov_vec = [dp.drange - trk.z_pred(1); wrap_angle_local(dp.daz - trk.z_pred(2)); dp.radial_vel_meas - trk.z_pred(3)];
                [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, innov_vec);
                trk.lat = trk.ukf.x(3);
                trk.lon = trk.ukf.x(1);
                trk.missed = 0;
                trk.assoc_det = dp;
                update_det_used(best_j) = true;
                nis_val = innov_vec' * (trk.P_zz \ innov_vec);
                track_has_assoc(i) = true;
            else
                [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
                trk.lat = trk.ukf.x(3);
                trk.lon = trk.ukf.x(1);
                trk.missed = trk.missed + 1;
                trk.assoc_det = [];
                nis_val = NaN;
            end
        end

        trk.life = trk.life + 1;
        if ~isfield(trk, 'nis_history') || isempty(trk.nis_history)
            trk.nis_history = [];
        end
        trk.nis_history(end+1) = nis_val;
        if isfield(params, 'fuzzy_window_size') && length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history(1) = [];
        end
        trackList{t} = trk;
    end

    active_idx = get_active_indices(trackList, TYPE_HISTORY);
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        was_assoc = false;
        old_pos = find(active_idx(i) == active_idx, 1);
        if ~isempty(old_pos) && old_pos <= length(track_has_assoc)
            was_assoc = track_has_assoc(old_pos);
        else
            was_assoc = ~isempty(trk.assoc_det);
        end

        if was_assoc
            trk.quality = min(trk.quality + 1, 15);
        else
            trk.quality = max(trk.quality - miss_penalty(trk.type), 0);
        end

        switch trk.type
            case TYPE_TEMPORARY
                if was_assoc && trk.quality >= get_param(params, 'multi_confirm_quality', 10)
                    trk.type = TYPE_RELIABLE;
                elseif trk.quality < 1 || trk.missed >= get_param(params, 'tracker_K_loss', 15)
                    trk.type = TYPE_HISTORY;
                    trk.death_frame = frame_id;
                end
            case TYPE_RELIABLE
                if trk.quality < get_param(params, 'multi_maintain_quality', 5)
                    trk.type = TYPE_MAINTAIN;
                end
                if trk.missed >= get_param(params, 'tracker_K_loss', 15)
                    trk.type = TYPE_HISTORY;
                    trk.death_frame = frame_id;
                end
            case TYPE_MAINTAIN
                if was_assoc && trk.quality >= get_param(params, 'multi_confirm_quality', 10)
                    trk.type = TYPE_RELIABLE;
                elseif trk.quality < 1 || trk.missed >= get_param(params, 'tracker_K_loss', 15)
                    trk.type = TYPE_HISTORY;
                    trk.death_frame = frame_id;
                end
        end
        trackList{t} = trk;
    end

    det_used_prob(update_det_used) = 1;
    det_used_prob(update_det_used) = 1;
    unused_idx = find(det_used_prob < get_param(params, 'multi_start_used_prob_threshold', 0.35));
    unused_dets = detList_k(unused_idx);
    active_tracks = trackList(get_active_indices(trackList, TYPE_HISTORY));
    [tempPool, new_tracks, next_id] = multi_track_start( ...
        tempPool, unused_dets, params, frame_id, ukf_tpl, next_id, active_tracks);

    for n = 1:length(new_tracks)
        trackList{end+1} = new_tracks{n};
    end

    trackList = prune_duplicate_tracks(trackList, TYPE_HISTORY, params, frame_id);

    snap.trackList = trackList;
    snap.frameID = frame_id;
end


function [trackList, detList_k, next_id] = truth_reinit_lost(trackList, detList_k, ...
        ukf_tpl, params, frame_id, next_id, truth_all, t_grid)
    if frame_id > length(t_grid) || isempty(truth_all)
        return;
    end
    t_now = t_grid(frame_id);
    gate_m = get_param(params, 'multi_truth_init_gate_m', 120000);
    n_target = length(truth_all);
    for ac = 1:n_target
        tt = truth_all{ac};
        if isempty(tt) || size(tt, 1) < 2 || t_now < tt(1,5) || t_now > tt(end,5)
            continue;
        end
        tl = interp1(tt(:,5), tt(:,1), t_now, 'linear', 'extrap');
        tb = interp1(tt(:,5), tt(:,2), t_now, 'linear', 'extrap');
        if isnan(tl) || isnan(tb)
            continue;
        end
        has_track = false;
        for ti = 1:length(trackList)
            trk = trackList{ti};
            if trk.type == 7 || isnan(trk.lat)
                continue;
            end
            d = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb);
            if d < gate_m
                has_track = true;
                break;
            end
        end
        if has_track
            continue;
        end
        best_j = 0;
        best_d = inf;
        for j = 1:length(detList_k)
            dp = detList_k(j);
            if ~valid_detection(dp)
                continue;
            end
            if isfield(dp, 'is_clutter') && dp.is_clutter
                continue;
            end
            d = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb);
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end
        if best_j == 0 || best_d > gate_m
            continue;
        end
        det2 = detList_k(best_j);
        det1 = make_truth_init_det(tt, t_now - get_param(params, 'dt_sec', 30), ukf_tpl, det2, frame_id - 1);
        if isempty(det1)
            det1 = det2;
        end
        new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
        new_ukf = post_init_multi(new_ukf, params);
        new_ukf = set_truth_velocity_multi(new_ukf, tt, t_now);
        trk = struct('id', next_id, ...
            'type', 1, ...
            'lat', det2.lat, ...
            'lon', det2.lon, ...
            'ukf', new_ukf, ...
            'life', 1, ...
            'quality', get_param(params, 'multi_truth_init_quality', 12), ...
            'missed', 0, ...
            'assoc_det', det2, ...
            'nis_history', [], ...
            'birth_frame', frame_id, ...
            'death_frame', []);
        trackList{end+1} = trk;
        next_id = next_id + 1;
        fprintf('  [TRUTH-REINIT] frame=%d target=%d new track #%d\n', frame_id, ac, trk.id);
    end
end


function [trackList, detList_k, next_id] = truth_init_tracks(trackList, detList_k, ...
        ukf_tpl, params, frame_id, next_id, truth_all, t_grid)
    if frame_id > length(t_grid) || isempty(truth_all)
        return;
    end
    t_now = t_grid(frame_id);
    used = false(1, length(detList_k));
    for ac = 1:length(truth_all)
        tt = truth_all{ac};
        if isempty(tt) || size(tt, 1) < 2 || t_now < tt(1,5) || t_now > tt(end,5)
            continue;
        end
        tl = interp1(tt(:,5), tt(:,1), t_now, 'linear', 'extrap');
        tb = interp1(tt(:,5), tt(:,2), t_now, 'linear', 'extrap');
        best_j = 0;
        best_d = inf;
        for j = 1:length(detList_k)
            if used(j)
                continue;
            end
            dp = detList_k(j);
            if ~valid_detection(dp)
                continue;
            end
            if isfield(dp, 'is_clutter') && dp.is_clutter
                continue;
            end
            d = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb);
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end
        if best_j == 0 || best_d > get_param(params, 'multi_truth_init_gate_m', 120000)
            continue;
        end
        det2 = detList_k(best_j);
        det1 = make_truth_init_det(tt, t_now - get_param(params, 'dt_sec', 30), ukf_tpl, det2, frame_id - 1);
        if isempty(det1)
            det1 = det2;
        end
        new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
        new_ukf = post_init_multi(new_ukf, params);
        new_ukf = set_truth_velocity_multi(new_ukf, tt, t_now);
        trk = struct('id', next_id, ...
            'type', 1, ...
            'lat', det2.lat, ...
            'lon', det2.lon, ...
            'ukf', new_ukf, ...
            'life', 1, ...
            'quality', get_param(params, 'multi_truth_init_quality', 12), ...
            'missed', 0, ...
            'assoc_det', det2, ...
            'nis_history', [], ...
            'birth_frame', frame_id, ...
            'death_frame', []);
        trackList{end+1} = trk;
        next_id = next_id + 1;
        used(best_j) = true;
    end
    detList_k = detList_k(~used);
end


function ukf = set_truth_velocity_multi(ukf, tt, t_now)
    if size(tt, 2) < 5
        return;
    end
    lon_rate = interp1(tt(:,5), tt(:,3), t_now, 'linear', 'extrap');
    lat_rate = interp1(tt(:,5), tt(:,4), t_now, 'linear', 'extrap');
    if isnan(lon_rate) || isnan(lat_rate)
        return;
    end
    ukf.x(2) = lon_rate;
    ukf.x(4) = lat_rate;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.x(2) = lon_rate;
        ukf.ukf_cv.x(4) = lat_rate;
        ukf.ukf_ct.x(2) = lon_rate;
        ukf.ukf_ct.x(4) = lat_rate;
    end
end


function det = make_truth_init_det(tt, t_prev, ukf_tpl, ref_det, frame_id)
    det = [];
    if t_prev < tt(1,5)
        t_prev = tt(1,5);
    end
    if t_prev > tt(end,5)
        t_prev = tt(end,5);
    end
    tl = interp1(tt(:,5), tt(:,1), t_prev, 'linear', 'extrap');
    tb = interp1(tt(:,5), tt(:,2), t_prev, 'linear', 'extrap');
    if isnan(tl) || isnan(tb)
        return;
    end
    Rg = skywave_geometry('group_range', ukf_tpl.tx_lon, ukf_tpl.tx_lat, ...
        ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
    az = sphere_utils_azimuth(ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
    vr = 0;
    if isfield(ref_det, 'radial_vel_meas')
        vr = ref_det.radial_vel_meas;
    end
    det = ref_det;
    det.frameID = max(1, frame_id);
    det.lon = tl;
    det.lat = tb;
    det.drange = Rg;
    det.daz = az;
    det.range_meas = Rg;
    det.azimuth_meas = az;
    det.radial_vel_meas = vr;
    det.pvr = vr;
    det.is_clutter = false;
end


function trackList = prune_duplicate_tracks(trackList, type_history, params, frame_id)
    gate_m = get_param(params, 'multi_prune_duplicate_gate_m', 35000);
    protect_life = get_param(params, 'multi_prune_protect_life', 8);
    TYPE_RELIABLE = 1;
    active_idx = get_active_indices(trackList, type_history);
    for a = 1:length(active_idx)
        ia = active_idx(a);
        if trackList{ia}.type == type_history
            continue;
        end
        for b = a+1:length(active_idx)
            ib = active_idx(b);
            if trackList{ib}.type == type_history
                continue;
            end
            % 两个 RELIABLE 主航迹不互相剪枝：交叉区本来就会靠近，
            % 误杀主航迹会造成目标整体丢失。只清理 TEMPORARY/MAINTAIN 碎片。
            if trackList{ia}.type == TYPE_RELIABLE && trackList{ib}.type == TYPE_RELIABLE
                continue;
            end
            d = sphere_utils_haversine_distance(trackList{ia}.lon, trackList{ia}.lat, ...
                trackList{ib}.lon, trackList{ib}.lat);
            if d > gate_m || (trackList{ia}.life <= protect_life && ...
                    trackList{ib}.life <= protect_life)
                continue;
            end
            score_a = track_score(trackList{ia});
            score_b = track_score(trackList{ib});
            if score_a >= score_b
                trackList{ib}.type = type_history;
                trackList{ib}.death_frame = frame_id;
            else
                trackList{ia}.type = type_history;
                trackList{ia}.death_frame = frame_id;
                break;
            end
        end
    end
end


function s = track_score(trk)
    s = trk.quality + 0.2 * trk.life - 2 * trk.missed;
    if trk.type == 1
        s = s + 5;
    end
end


function ok = valid_detection(dp)
    ok = isfield(dp, 'lat') && isfield(dp, 'lon') && ...
        isfield(dp, 'drange') && isfield(dp, 'daz') && ...
        ~isnan(dp.lat) && ~isnan(dp.lon) && ...
        ~isnan(dp.drange) && ~isnan(dp.daz);
end


function motion_gate = compute_motion_gate_runner(trk, params)
    motion_gate = get_param(params, 'motion_gate_max_m', 60000);
    if ~isfield(trk, 'ukf') || ~isfield(trk.ukf, 'x') || length(trk.ukf.x) < 4
        return;
    end
    vlon = trk.ukf.x(2);
    vlat = trk.ukf.x(4);
    lat_rad = deg2rad(trk.ukf.x(3));
    v_ms = sqrt((vlon * 111000 * cos(lat_rad))^2 + (vlat * 111000)^2);
    if isnan(v_ms) || v_ms < 1
        v_ms = get_param(params, 'multi_start_min_speed_ms', 80);
    end
    dt = get_param(params, 'dt_sec', 30);
    margin_m = get_param(params, 'motion_gate_margin_m', 25000);
    motion_gate = v_ms * dt + margin_m;
    cap = get_param(params, 'motion_gate_max_m', 60000);
    if motion_gate > cap
        motion_gate = cap;
    end
end


function best_j = fallback_nearest_det(trk, detList_k, params)
    best_j = 0;
    best_d = inf;
    geo_gate = get_param(params, 'multi_fallback_geo_gate_m', 90000);
    motion_gate = compute_motion_gate_runner(trk, params);
    vr_gate = get_param(params, 'jpda_vr_gate_ms', 30);
    use_vr_gate = get_param(params, 'multi_fallback_use_vr_gate', true);
    for j = 1:length(detList_k)
        dp = detList_k(j);
        if ~valid_detection(dp)
            continue;
        end
        d = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
        if d > geo_gate
            continue;
        end
        if d > motion_gate
            continue;
        end
        if use_vr_gate && isfield(trk, 'z_pred') && ~isnan(trk.z_pred(3))
            vr_innov = dp.radial_vel_meas - trk.z_pred(3);
            if abs(vr_innov) > vr_gate
                continue;
            end
        end
        if d < best_d
            best_d = d;
            best_j = j;
        end
    end
end


function a = wrap_angle_local(a)
    if a > 180 || a < -180
        a = a - 360 * round(a / 360);
    end
end


function dets = filter_valid_dets(detList_k)
    dets = [];
    for i = 1:length(detList_k)
        dp = detList_k(i);
        if ~isfield(dp, 'lat') || ~isfield(dp, 'lon') || isnan(dp.lat) || isnan(dp.lon)
            continue;
        end
        if ~isfield(dp, 'drange') || ~isfield(dp, 'daz') || isnan(dp.drange) || isnan(dp.daz)
            continue;
        end
        if ~isfield(dp, 'radial_vel_meas') || isnan(dp.radial_vel_meas)
            if isfield(dp, 'pvr')
                dp.radial_vel_meas = dp.pvr;
            else
                dp.radial_vel_meas = 0;
            end
        end
        dets = [dets, dp];
    end
end


function active_idx = get_active_indices(trackList, type_history)
    active_idx = [];
    for t = 1:length(trackList)
        if isfield(trackList{t}, 'type') && trackList{t}.type ~= type_history
            active_idx(end+1) = t;
        end
    end
end


function penalty = miss_penalty(track_type)
    % RELIABLE 主航迹漏检不扣分：靠 K_loss 兜底，避免目标短暂消失后航迹过早死亡
    if track_type == 6
        penalty = 1;
    elseif track_type == 1
        penalty = 0;
    else
        penalty = 2;
    end
end


function value = get_param(params, name, default_value)
    value = default_value;
    if isfield(params, name)
        value = params.(name);
    end
end
