function [trackList, tempTrackList, snap, next_id, diagInfo] = Track_Process_for_HighRate_Oracle( ...
        trackList, tempTrackList, pointList, ukf_tpl, params, frame_id, next_id, truth_all, t_grid)

    if isempty(trackList), trackList = {}; end
    if isempty(tempTrackList), tempTrackList = struct([]); end
    pointList = normalize_point_list(pointList);
    n_points = length(pointList);
    history_type = params.HISTORY_TRACK;

    before_truth_termination = trackList;
    if is_truth_termination_enabled(params)
        trackList = terminate_finished_truth( ...
            trackList, truth_all, t_grid, frame_id, history_type);
    end
    truth_events = collect_lifecycle_events( ...
        before_truth_termination, trackList, frame_id, history_type);
    [activeTrackList, historyTrackList] = partition_tracks(trackList, history_type);

    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        trk.ukf.dt = params.dt_sec;
        trk.ukf.life_count = trk.life + 1;
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ...
            ukf_dispatch('prepare', trk.ukf);
        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        activeTrackList{i} = trk;
    end

    before_update = activeTrackList;
    [TPmatch_result, singlePointsIndex, association_used_det] = ...
        PointTrackAssociation_Oracle(activeTrackList, pointList, frame_id);
    activeTrackList = Fun_UpdateTrackByAsscResult_Oracle( ...
        activeTrackList, pointList, TPmatch_result, params, frame_id);
    update_events = collect_lifecycle_events( ...
        before_update, activeTrackList, frame_id, history_type);

    [stillActiveTrackList, newlyHistoryTrackList] = ...
        partition_tracks(activeTrackList, history_type);
    historyTrackList = [historyTrackList, newlyHistoryTrackList];

    [remainingPointList, pointOriginalIndex] = ...
        fun_remove_assc_pts_from_pointlist_oracle(pointList, association_used_det);
    [tempTrackList, valid_tracks, next_id, starter_used_det] = ...
        trackStarter_logic_oracle(tempTrackList, remainingPointList, ...
        pointOriginalIndex, params, params.oracle_QUALIFY_NUM, ...
        params.oracle_TOLERANT_NUM, ukf_tpl, params, frame_id, next_id, ...
        truth_all, t_grid, stillActiveTrackList, n_points);
    confirm_events = collect_confirmation_events(valid_tracks, frame_id);

    used_det = association_used_det | starter_used_det;
    trackList = [stillActiveTrackList, valid_tracks, historyTrackList];
    snap = make_snap([stillActiveTrackList, valid_tracks], frame_id);

    diagInfo = struct('frameID', frame_id, 'n_points', n_points, ...
        'TPmatch_result', TPmatch_result, ...
        'association_used_det', association_used_det, ...
        'starter_used_det', starter_used_det, 'used_det', used_det, ...
        'singlePointsIndex', singlePointsIndex, 'unused_det', find(~used_det), ...
        'lifecycle_events', [truth_events, update_events, confirm_events]);
end

function enabled = is_truth_termination_enabled(params)
    enabled = isfield(params, 'oracle_truth_terminate_enable') && ...
        isscalar(params.oracle_truth_terminate_enable) && ...
        logical(params.oracle_truth_terminate_enable);
end

function [active, history] = partition_tracks(trackList, history_type)
    active = {};
    history = {};
    for i = 1:length(trackList)
        if get_track_type(trackList{i}) == history_type
            history{end+1} = trackList{i};
        else
            active{end+1} = trackList{i};
        end
    end
end

function pointList = normalize_point_list(pointList)
    if isempty(pointList), pointList = []; return; end
    for i = 1:length(pointList)
        if ~isfield(pointList(i), 'aircraft_id')
            pointList(i).aircraft_id = int32(0);
        end
        if ~isfield(pointList(i), 'is_clutter')
            pointList(i).is_clutter = double(pointList(i).aircraft_id) == 0;
        end
    end
end

function trackList = terminate_finished_truth( ...
        trackList, truth_all, t_grid, frame_id, history_type)
    if isempty(truth_all) || frame_id > length(t_grid), return; end
    t_now = t_grid(frame_id);
    for i = 1:length(trackList)
        trk = trackList{i};
        if get_track_type(trk) == history_type || ~isfield(trk, 'truth_idx')
            continue;
        end
        truth_id = double(trk.truth_idx);
        if ~isscalar(truth_id) || ~isfinite(truth_id) || truth_id < 1 || ...
                truth_id > length(truth_all) || isempty(truth_all{truth_id})
            continue;
        end
        if t_now > truth_all{truth_id}(end, 5)
            trk.Type = history_type;
            trk.type = history_type;
            trk.death_frame = frame_id;
            trk.death_reason = 'truth_ended';
            trackList{i} = trk;
        end
    end
end

function events = collect_lifecycle_events(before, after, frame_id, history_type)
    events = empty_events();
    if isempty(after), return; end
    before_type_by_id = nan(1, max_track_id(before));
    for i = 1:length(before)
        id = before{i}.id;
        if is_valid_id(id)
            before_type_by_id(id) = get_track_type(before{i});
        end
    end
    for i = 1:length(after)
        trk = after{i};
        id = trk.id;
        if ~is_valid_id(id) || id > length(before_type_by_id) || ...
                isnan(before_type_by_id(id)) || ...
                before_type_by_id(id) == history_type || ...
                get_track_type(trk) ~= history_type
            continue;
        end
        events(end+1) = make_event('died', trk, frame_id);
    end
end

function max_id = max_track_id(trackList)
    max_id = 0;
    for i = 1:length(trackList)
        id = trackList{i}.id;
        if is_valid_id(id), max_id = max(max_id, id); end
    end
end

function tf = is_valid_id(id)
    tf = isscalar(id) && isfinite(id) && id >= 1 && id == floor(id);
end

function events = collect_confirmation_events(valid_tracks, frame_id)
    events = empty_events();
    for i = 1:length(valid_tracks)
        events(end+1) = make_event('confirmed', valid_tracks{i}, frame_id);
    end
end

function events = empty_events()
    events = struct('event', {}, 'track_id', {}, 'truth_idx', {}, ...
        'frameID', {}, 'birth_frame', {}, 'confirm_frame', {}, ...
        'death_frame', {}, 'death_reason', {}, 'Quality', {}, ...
        'SuccLossPointCnt', {}, 'TotalPointCnt', {}, ...
        'AsscPointCnt', {}, 'TotalLostPointCnt', {});
end

function event = make_event(name, trk, frame_id)
    event = struct('event', name, 'track_id', trk.id, ...
        'truth_idx', trk.truth_idx, 'frameID', frame_id, ...
        'birth_frame', field_or(trk, 'birth_frame', NaN), ...
        'confirm_frame', field_or(trk, 'confirm_frame', NaN), ...
        'death_frame', field_or(trk, 'death_frame', NaN), ...
        'death_reason', field_or(trk, 'death_reason', ''), ...
        'Quality', trk.Quality, ...
        'SuccLossPointCnt', trk.SuccLossPointCnt, ...
        'TotalPointCnt', trk.TotalPointCnt, ...
        'AsscPointCnt', trk.AsscPointCnt, ...
        'TotalLostPointCnt', trk.TotalLostPointCnt);
end

function value = field_or(s, name, default_value)
    if isfield(s, name), value = s.(name); else, value = default_value; end
end

function type = get_track_type(trk)
    if isfield(trk, 'Type'), type = trk.Type; else, type = trk.type; end
end

function snap = make_snap(activeTrackList, frame_id)
    slim_tracks = cell(1, length(activeTrackList));
    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        slim_ukf = struct('x', trk.ukf.x, 'P', trk.ukf.P, 'Q', trk.ukf.Q);
        if isfield(trk, 'P_pred') && ~isempty(trk.P_pred)
            P_pred = trk.P_pred;
        else
            P_pred = trk.ukf.P;
        end
        slim_tracks{i} = struct('id', trk.id, 'type', get_track_type(trk), ...
            'life', trk.life, 'truth_idx', trk.truth_idx, ...
            'lat', trk.lat, 'lon', trk.lon, 'P_pred', P_pred, ...
            'ukf', slim_ukf);
    end
    snap = struct('trackList', {slim_tracks}, 'frameID', frame_id);
end
