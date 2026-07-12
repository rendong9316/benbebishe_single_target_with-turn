% =========================================================================
% jpda_multi.m — 多目标 JPDA 关联
% =========================================================================
% 仅依赖航迹预测、校正后点迹和参数，不读取真值标签。
% =========================================================================

function [assoc, det_used_prob] = jpda_multi(trackList, active_idx, detList, params)
    n_tracks = length(active_idx);
    n_dets = length(detList);
    assoc = cell(n_tracks, 1);
    det_used_prob = zeros(1, n_dets);

    for i = 1:n_tracks
        assoc{i} = empty_assoc(active_idx(i), i);
    end
    if n_tracks == 0 || n_dets == 0
        return;
    end

    gate = build_gate(trackList, active_idx, detList, params);
    events = enumerate_events(gate.gated_indices, n_dets, params);
    if isempty(events)
        events = zeros(1, n_tracks);
    end

    log_w = compute_event_log_weights(events, gate, params);

    % JPDA* 置换剪枝 (Blom & Bloem, NLR-TP-2006-690)：
    % 对每组使用相同检测集合的联合事件，只保留似然最高的那个置换，
    % 避免 JPDA 的"置换平均"导致 track coalescence。
    if get_param(params, 'jpda_star_enable', true)
        [events, log_w] = jpda_star_prune(events, log_w);
    end

    max_log_w = max(log_w);
    if ~isfinite(max_log_w)
        event_w = zeros(size(log_w));
        all_miss = all(events == 0, 2);
        if any(all_miss)
            event_w(find(all_miss, 1)) = 1;
        else
            event_w(1) = 1;
        end
    else
        event_w = exp(log_w - max_log_w);
        total_w = sum(event_w);
        if total_w <= 0 || ~isfinite(total_w)
            event_w = zeros(size(log_w));
            event_w(1) = 1;
        else
            event_w = event_w / total_w;
        end
    end

    beta = zeros(n_tracks, n_dets);
    beta0 = zeros(n_tracks, 1);
    for e = 1:size(events, 1)
        for i = 1:n_tracks
            j = events(e, i);
            if j == 0
                beta0(i) = beta0(i) + event_w(e);
            else
                beta(i, j) = beta(i, j) + event_w(e);
            end
        end
    end
    det_used_prob = min(1, sum(beta, 1));

    for i = 1:n_tracks
        det_indices = find(beta(i, :) > 1e-6);
        betas = beta(i, det_indices);
        innov_w = zeros(3, 1);
        for d = 1:length(det_indices)
            j = det_indices(d);
            innov_w = innov_w + beta(i, j) * gate.innov3{i, j};
        end

        nis = NaN;
        if ~isempty(det_indices)
            trk = trackList{active_idx(i)};
            if isfield(trk, 'P_zz') && isequal(size(trk.P_zz), [3, 3])
                P = regularize_local(trk.P_zz);
                nis = innov_w' * (P \ innov_w);
            end
        end

        best_det_index = 0;
        if ~isempty(det_indices)
            [~, pos] = max(betas);
            best_det_index = det_indices(pos);
        end

        assoc{i} = struct('track_index', active_idx(i), ...
            'active_position', i, ...
            'det_indices', det_indices, ...
            'betas', betas, ...
            'beta0', beta0(i), ...
            'innov_w', innov_w, ...
            'nis', nis, ...
            'best_det_index', best_det_index);
    end
end


function motion_gate_m = compute_motion_gate(trk, params)
    motion_gate_m = get_param(params, 'motion_gate_max_m', 60000);
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
    motion_gate_m = v_ms * dt + margin_m;
    cap = get_param(params, 'motion_gate_max_m', 60000);
    if motion_gate_m > cap
        motion_gate_m = cap;
    end
end


function gate = build_gate(trackList, active_idx, detList, params)
    n_tracks = length(active_idx);
    n_dets = length(detList);
    gate.gated = false(n_tracks, n_dets);
    gate.gated_indices = cell(n_tracks, 1);
    gate.log_likelihood = -inf(n_tracks, n_dets);
    gate.innov3 = cell(n_tracks, n_dets);

    gate_threshold = get_param(params, 'gate_sigma', 6)^2 * 2;
    Pd = get_param(params, 'detection_probability', 0.9);
    Pg = get_param(params, 'pda_pd_gate', 0.8647);
    log_pd = log(max(Pd * Pg, realmin));

    for i = 1:n_tracks
        trk = trackList{active_idx(i)};
        if ~isfield(trk, 'P_zz') || ~isfield(trk, 'z_pred') || ~isfield(trk, 'x_pred')
            continue;
        end
        if ~isequal(size(trk.P_zz), [3, 3])
            continue;
        end

        P2 = regularize_local(trk.P_zz(1:2, 1:2));
        detP2 = max(det(P2), realmin);
        log_norm = -0.5 * (2 * log(2*pi) + log(detP2));
        if isfield(trk, 'life') && trk.life <= 5
            geo_gate_m = get_param(params, 'jpda_geo_gate_m_initial', 120000);
        else
            geo_gate_m = get_param(params, 'jpda_geo_gate_m_stable', 80000);
        end
        if isfield(trk, 'missed')
            geo_gate_m = geo_gate_m + trk.missed * get_param(params, 'jpda_geo_gate_m_missed_step', 15000);
        end
        % 硬性运动门：基于航迹当前速度计算物理可达半径，
        % 即使 Mahalanobis 门因协方差膨胀变宽，也禁止离谱的远距离关联
        motion_gate_m = compute_motion_gate(trk, params);

        for j = 1:n_dets
            dp = detList(j);
            if ~valid_det(dp)
                continue;
            end

            geo_dist = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
            if geo_dist > geo_gate_m
                continue;
            end
            if geo_dist > motion_gate_m
                continue;
            end

            z2 = [dp.drange; dp.daz];
            innov2 = z2 - trk.z_pred(1:2);
            innov2(2) = wrap_angle(innov2(2));
            mahal = innov2' * (P2 \ innov2);
            if mahal >= gate_threshold
                continue;
            end

            z3 = [dp.drange; dp.daz; get_vr(dp)];
            innov3 = z3 - trk.z_pred;
            innov3(2) = wrap_angle(innov3(2));

            gate.gated(i, j) = true;
            gate.gated_indices{i}(end+1) = j;
            gate.innov3{i, j} = innov3;
            gate.log_likelihood(i, j) = log_pd + log_norm - 0.5 * mahal;
        end
    end
end


function [events_kept, log_w_kept] = jpda_star_prune(events, log_w)
    n_events = size(events, 1);
    if n_events <= 1
        events_kept = events;
        log_w_kept = log_w;
        return;
    end

    keys = cell(n_events, 1);
    for e = 1:n_events
        dets = events(e, :);
        dets = sort(dets(dets > 0));
        keys{e} = mat2str(dets);
    end

    keep = false(n_events, 1);
    visited = false(n_events, 1);
    n_groups = 0;
    n_pruned = 0;
    for e = 1:n_events
        if visited(e)
            continue;
        end
        group_members = e;
        visited(e) = true;
        for f = e+1:n_events
            if visited(f)
                continue;
            end
            if strcmp(keys{f}, keys{e})
                group_members(end+1) = f;
                visited(f) = true;
            end
        end
        [~, best_local] = max(log_w(group_members));
        keep(group_members(best_local)) = true;
        n_groups = n_groups + 1;
        n_pruned = n_pruned + length(group_members) - 1;
    end

    events_kept = events(keep, :);
    log_w_kept = log_w(keep);
end


function events = enumerate_events(gated_indices, n_dets, params)
    n_tracks = length(gated_indices);
    events = zeros(0, n_tracks);
    current = zeros(1, n_tracks);
    used = false(1, n_dets);
    max_hyp = get_param(params, 'jpda_max_hypotheses', 5000);
    events = recurse_events(1, gated_indices, used, current, events, max_hyp);
end


function events = recurse_events(i, gated_indices, used, current, events, max_hyp)
    if size(events, 1) >= max_hyp
        return;
    end
    if i > length(gated_indices)
        events(end+1, :) = current;
        return;
    end

    current(i) = 0;
    events = recurse_events(i + 1, gated_indices, used, current, events, max_hyp);
    if size(events, 1) >= max_hyp
        return;
    end

    cand = gated_indices{i};
    for c = 1:length(cand)
        j = cand(c);
        if used(j)
            continue;
        end
        used(j) = true;
        current(i) = j;
        events = recurse_events(i + 1, gated_indices, used, current, events, max_hyp);
        used(j) = false;
        if size(events, 1) >= max_hyp
            return;
        end
    end
end


function log_w = compute_event_log_weights(events, gate, params)
    n_events = size(events, 1);
    n_tracks = size(events, 2);
    log_w = zeros(n_events, 1);
    miss_prob = max(1 - get_param(params, 'detection_probability', 0.9) * ...
        get_param(params, 'pda_pd_gate', 0.8647), realmin);
    log_miss = log(miss_prob);

    for e = 1:n_events
        lw = 0;
        for i = 1:n_tracks
            j = events(e, i);
            if j == 0
                lw = lw + log_miss;
            else
                lw = lw + gate.log_likelihood(i, j);
            end
        end
        log_w(e) = lw;
    end
end


function assoc = empty_assoc(track_index, active_position)
    assoc = struct('track_index', track_index, ...
        'active_position', active_position, ...
        'det_indices', [], ...
        'betas', [], ...
        'beta0', 1, ...
        'innov_w', zeros(3, 1), ...
        'nis', NaN, ...
        'best_det_index', 0);
end


function ok = valid_det(dp)
    ok = isfield(dp, 'lat') && isfield(dp, 'lon') && ...
        isfield(dp, 'drange') && isfield(dp, 'daz') && ...
        ~isnan(dp.lat) && ~isnan(dp.lon) && ...
        ~isnan(dp.drange) && ~isnan(dp.daz);
end


function vr = get_vr(dp)
    if isfield(dp, 'radial_vel_meas') && ~isnan(dp.radial_vel_meas)
        vr = dp.radial_vel_meas;
    elseif isfield(dp, 'pvr') && ~isnan(dp.pvr)
        vr = dp.pvr;
    else
        vr = 0;
    end
end


function P = regularize_local(P)
    P = (P + P') / 2;
    if any(isnan(P(:))) || any(isinf(P(:)))
        P = eye(size(P));
        return;
    end
    jitter = 1e-9;
    tries = 0;
    while rcond(P) < 1e-12 && tries < 6
        P = P + eye(size(P)) * jitter;
        jitter = jitter * 10;
        tries = tries + 1;
    end
end


function a = wrap_angle(a)
    if a > 180 || a < -180
        a = a - 360 * round(a / 360);
    end
end


function value = get_param(params, name, default_value)
    value = default_value;
    if isfield(params, name)
        value = params.(name);
    end
end
