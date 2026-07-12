% =========================================================================
% multi_track_start.m — 多目标专属 M/N 航迹起始
% =========================================================================
% 维护多个临时起始候选，不依赖单目标 track_initiation。
% =========================================================================

function [tempPool, new_tracks, next_id] = multi_track_start( ...
        tempPool, unused_dets, params, frame_id, ukf_tpl, next_id, active_tracks)

    if nargin < 7
        active_tracks = {};
    end
    if isempty(tempPool)
        tempPool = {};
    end

    new_tracks = {};
    if isempty(unused_dets)
        tempPool = age_temp_pool(tempPool, params);
        return;
    end

    updated = false(1, length(tempPool));
    det_assigned = false(1, length(unused_dets));

    for d = 1:length(unused_dets)
        det = unused_dets(d);
        if ~valid_start_det(det)
            continue;
        end

        best_idx = 0;
        best_score = inf;
        for p = 1:length(tempPool)
            tmp = tempPool{p};
            if updated(p) || isempty(tmp.det_history)
                continue;
            end
            [ok, score] = can_attach(tmp, det, params, frame_id);
            if ok && score < best_score
                best_score = score;
                best_idx = p;
            end
        end

        if best_idx > 0
            tmp = tempPool{best_idx};
            tmp.det_history{end+1} = det;
            tmp.hits = tmp.hits + 1;
            tmp.misses = 0;
            tmp.last_frame = frame_id;
            tmp.score = tmp.score + best_score;
            tempPool{best_idx} = tmp;
            updated(best_idx) = true;
            det_assigned(d) = true;
        end
    end

    for d = 1:length(unused_dets)
        if det_assigned(d) || ~valid_start_det(unused_dets(d))
            continue;
        end
        tempPool{end+1} = struct('id', next_temp_id(tempPool), ...
            'hits', 1, ...
            'misses', 0, ...
            'first_frame', frame_id, ...
            'last_frame', frame_id, ...
            'det_history', {{unused_dets(d)}}, ...
            'score', 0);
        updated(end+1) = true;
    end

    for p = 1:length(tempPool)
        if p > length(updated) || ~updated(p)
            tmp = tempPool{p};
            tmp.misses = tmp.misses + 1;
            tempPool{p} = tmp;
        end
    end

    keep = true(1, length(tempPool));
    for p = 1:length(tempPool)
        tmp = tempPool{p};
        if tmp.misses > get_param(params, 'multi_start_max_misses', 2)
            keep(p) = false;
            continue;
        end

        if is_confirmed(tmp, params)
            det1 = first_valid_det(tmp.det_history);
            det2 = tmp.det_history{end};
            if isempty(det1) || duplicate_track(det2, active_tracks, params)
                keep(p) = false;
                continue;
            end

            new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
            new_ukf = post_init_multi(new_ukf, params);
            trk = struct('id', next_id, ...
                'type', 6, ...
                'lat', det2.lat, ...
                'lon', det2.lon, ...
                'ukf', new_ukf, ...
                'life', 1, ...
                'quality', get_param(params, 'multi_start_initial_quality', 5), ...
                'missed', 0, ...
                'assoc_det', det2, ...
                'nis_history', [], ...
                'birth_frame', frame_id, ...
                'death_frame', []);
            new_tracks{end+1} = trk;
            next_id = next_id + 1;
            keep(p) = false;
        end
    end
    tempPool = tempPool(keep);
end


function tempPool = age_temp_pool(tempPool, params)
    keep = true(1, length(tempPool));
    for p = 1:length(tempPool)
        tmp = tempPool{p};
        tmp.misses = tmp.misses + 1;
        tempPool{p} = tmp;
        if tmp.misses > get_param(params, 'multi_start_max_misses', 2)
            keep(p) = false;
        end
    end
    tempPool = tempPool(keep);
end


function [ok, score] = can_attach(tmp, det, params, frame_id)
    ok = false;
    score = inf;
    last_det = tmp.det_history{end};
    gap = frame_id - tmp.last_frame;
    if gap < 1 || gap > get_param(params, 'multi_start_max_gap_frames', 2)
        return;
    end

    dt = gap * get_param(params, 'dt_sec', 30);
    dist_m = sphere_utils_haversine_distance(last_det.lon, last_det.lat, det.lon, det.lat);
    speed_ms = dist_m / max(dt, eps);
    if speed_ms < get_param(params, 'multi_start_min_speed_ms', 80) || ...
            speed_ms > get_param(params, 'multi_start_max_speed_ms', 350)
        return;
    end

    if length(tmp.det_history) >= 2
        prev_det = tmp.det_history{end-1};
        h1 = sphere_utils_azimuth(prev_det.lon, prev_det.lat, last_det.lon, last_det.lat);
        h2 = sphere_utils_azimuth(last_det.lon, last_det.lat, det.lon, det.lat);
        dh = abs(wrap_angle(h2 - h1));
        if dh > get_param(params, 'multi_start_heading_gate_deg', 45)
            return;
        end
        score = dist_m * (1 + dh / 180);
    else
        score = dist_m;
    end
    ok = true;
end


function confirmed = is_confirmed(tmp, params)
    M = get_param(params, 'multi_start_M', 3);
    N = get_param(params, 'multi_start_N', 4);
    span = tmp.last_frame - tmp.first_frame + 1;
    confirmed = tmp.hits >= M && span <= max(N, M + tmp.misses);
end


function det = first_valid_det(det_history)
    det = [];
    for i = 1:length(det_history)
        if valid_start_det(det_history{i})
            det = det_history{i};
            return;
        end
    end
end


function yes = duplicate_track(det, active_tracks, params)
    yes = false;
    gate_m = get_param(params, 'multi_duplicate_gate_m', 50000);
    for i = 1:length(active_tracks)
        trk = active_tracks{i};
        if ~isfield(trk, 'type') || trk.type == 7 || isnan(trk.lat)
            continue;
        end
        d = sphere_utils_haversine_distance(trk.lon, trk.lat, det.lon, det.lat);
        if d < gate_m
            yes = true;
            return;
        end
    end
end


function id = next_temp_id(tempPool)
    id = 1;
    for i = 1:length(tempPool)
        if isfield(tempPool{i}, 'id')
            id = max(id, tempPool{i}.id + 1);
        end
    end
end


function ok = valid_start_det(det)
    ok = isfield(det, 'lat') && isfield(det, 'lon') && ...
        isfield(det, 'drange') && isfield(det, 'daz') && ...
        ~isnan(det.lat) && ~isnan(det.lon) && ...
        ~isnan(det.drange) && ~isnan(det.daz);
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
