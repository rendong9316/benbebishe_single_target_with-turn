function [trackList, tempTrackList, snap, next_id, diagInfo] = Track_Process_for_HighRate_Oracle( ...
        trackList, tempTrackList, pointList, ukf_tpl, params, frame_id, next_id, truth_all, t_grid)

    if isempty(trackList), trackList = {}; end
    if isempty(tempTrackList), tempTrackList = struct([]); end
    pointList = normalize_point_list(pointList);
    n_points = length(pointList);
    TYPE_HISTORY = params.HISTORY_TRACK;

    previous_trackList = trackList;
    trackList = terminate_finished_truth(trackList, truth_all, t_grid, frame_id, TYPE_HISTORY);
    truth_events = collect_lifecycle_events(previous_trackList, trackList, frame_id);
    [activeTrackList, historyTrackList] = partition_tracks(trackList, TYPE_HISTORY);

    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        trk.ukf.dt = params.dt_sec;
        trk.ukf.life_count = trk.life + 1;
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ukf_dispatch('prepare', trk.ukf);
        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        activeTrackList{i} = trk;
    end

    association_used_det = false(1, n_points);
    TPmatch_result = zeros(length(activeTrackList), 2);
    singlePointsIndex = 1:n_points;
    before_update = activeTrackList;
    if isempty(pointList)
        activeTrackList = Fun_UpdateTrackforNoInputPoint_Oracle(activeTrackList, params, frame_id);
    else
        [TPmatch_result, singlePointsIndex, association_used_det] = ...
            PointTrackAssociation_Oracle(activeTrackList, pointList, params);
        activeTrackList = Fun_UpdateTrackByAsscResult_Oracle( ...
            activeTrackList, pointList, TPmatch_result, params, frame_id);
    end
    update_events = collect_lifecycle_events(before_update, activeTrackList, frame_id);

    combined_after_update = [activeTrackList, historyTrackList];
    [stillActiveTrackList, newlyHistoryTrackList] = partition_tracks(combined_after_update, TYPE_HISTORY);
    historyTrackList = newlyHistoryTrackList;

    [remainingPointList, pointOriginalIndex] = ...
        fun_remove_assc_pts_from_pointlist_oracle(pointList, association_used_det);
    next_id_before = next_id;
    [tempTrackList, valid_tracks, next_id, starter_used_det] = trackStarter_logic_oracle( ...
        tempTrackList, remainingPointList, pointOriginalIndex, params, ...
        params.oracle_QUALIFY_NUM, params.oracle_TOLERANT_NUM, ukf_tpl, params, ...
        frame_id, next_id, truth_all, t_grid, stillActiveTrackList, n_points);
    confirm_events = collect_confirmation_events(valid_tracks, frame_id, next_id_before);

    used_det = association_used_det | starter_used_det;
    trackList = [stillActiveTrackList, valid_tracks, historyTrackList];
    trackList = sortTrackList_oracle(trackList);
    snap = make_snap(trackList, frame_id);

    diagInfo = struct();
    diagInfo.frameID = frame_id;
    diagInfo.n_points = n_points;
    diagInfo.TPmatch_result = TPmatch_result;
    diagInfo.association_used_det = association_used_det;
    diagInfo.starter_used_det = starter_used_det;
    diagInfo.used_det = used_det;
    diagInfo.singlePointsIndex = singlePointsIndex;
    diagInfo.unused_det = find(~used_det);
    diagInfo.lifecycle_events = [truth_events, update_events, confirm_events];
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
        if ~isfield(pointList(i), 'aircraft_id'), pointList(i).aircraft_id = int32(0); end
        if ~isfield(pointList(i), 'is_clutter')
            pointList(i).is_clutter = double(pointList(i).aircraft_id) == 0;
        end
    end
end

function trackList = terminate_finished_truth(trackList, truth_all, t_grid, frame_id, TYPE_HISTORY)
    if isempty(truth_all) || frame_id > length(t_grid), return; end
    t_now = t_grid(frame_id);
    for i = 1:length(trackList)
        trk = trackList{i};
        if get_track_type(trk) == TYPE_HISTORY || ~isfield(trk, 'truth_idx'), continue; end
        ac = double(trk.truth_idx);
        if ac < 1 || ac > length(truth_all) || isempty(truth_all{ac}), continue; end
        if t_now > truth_all{ac}(end, 5)
            trk.Type = TYPE_HISTORY;
            trk.type = TYPE_HISTORY;
            trk.death_frame = frame_id;
            trk.death_reason = 'truth_ended';
            trackList{i} = trk;
        end
    end
end

function events = collect_lifecycle_events(before, after, frame_id)
    events = empty_events();
    for i = 1:length(after)
        trk = after{i};
        old = find_track(before, trk.id);
        if isempty(old) || get_track_type(old) == 7 || get_track_type(trk) ~= 7, continue; end
        events(end+1) = make_event('died', trk, frame_id);
    end
end

function events = collect_confirmation_events(valid_tracks, frame_id, next_id_before)
    events = empty_events();
    for i = 1:length(valid_tracks)
        trk = valid_tracks{i};
        if trk.id < next_id_before, continue; end
        events(end+1) = make_event('confirmed', trk, frame_id);
    end
end

function events = empty_events()
    events = struct('event', {}, 'track_id', {}, 'truth_idx', {}, 'frameID', {}, ...
        'birth_frame', {}, 'confirm_frame', {}, 'death_frame', {}, 'death_reason', {}, ...
        'Quality', {}, 'SuccLossPointCnt', {}, 'TotalPointCnt', {}, ...
        'AsscPointCnt', {}, 'TotalLostPointCnt', {});
end

function event = make_event(name, trk, frame_id)
    event = struct('event', name, 'track_id', trk.id, 'truth_idx', trk.truth_idx, ...
        'frameID', frame_id, 'birth_frame', field_or(trk, 'birth_frame', NaN), ...
        'confirm_frame', field_or(trk, 'confirm_frame', NaN), ...
        'death_frame', field_or(trk, 'death_frame', NaN), ...
        'death_reason', field_or(trk, 'death_reason', ''), ...
        'Quality', trk.Quality, 'SuccLossPointCnt', trk.SuccLossPointCnt, ...
        'TotalPointCnt', trk.TotalPointCnt, 'AsscPointCnt', trk.AsscPointCnt, ...
        'TotalLostPointCnt', trk.TotalLostPointCnt);
end

function trk = find_track(trackList, id)
    trk = [];
    for i = 1:length(trackList)
        if trackList{i}.id == id, trk = trackList{i}; return; end
    end
end

function v = field_or(s, name, default_value)
    if isfield(s, name), v = s.(name); else, v = default_value; end
end

function t = get_track_type(trk)
    if isfield(trk, 'Type'), t = trk.Type; else, t = trk.type; end
end

function snap = make_snap(trackList, frame_id)
    snap = struct('trackList', {trackList}, 'frameID', frame_id);
end
