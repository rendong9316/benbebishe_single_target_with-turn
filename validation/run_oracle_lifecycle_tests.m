function run_oracle_lifecycle_tests()
    params = simulation_params_oracle();
    params.tracker_K_loss = 8;

    track = fixture_track(params);
    for k = 1:7
        track = fun_track_quality_management_and_info_completion_oracle(track, [], params, params, k);
        assert(track.Type ~= params.HISTORY_TRACK);
    end
    track = fun_track_quality_management_and_info_completion_oracle(track, [], params, params, 8);
    assert(track.Type == params.HISTORY_TRACK);
    assert(strcmp(track.death_reason, 'k_loss'));
    assert(track.death_frame == 8);
    assert(track.SuccLossPointCnt == 8);

    track = fixture_track(params);
    track.Quality = 0;
    track.quality = 0;
    track = fun_track_quality_management_and_info_completion_oracle(track, [], params, params, 1);
    assert(track.Type ~= params.HISTORY_TRACK);

    tempTrackList = struct([]);
    truth_all = {[0, 0, 0, 0, 0; 1, 1, 0, 0, 100]};
    ukf_tpl = ukf_jichu('create', radar_params(params, 1), 113, 33.5, 109, 33.5, params.dt_sec);
    next_id = 1;
    frames_with_detection = [1, 3, 5];
    created = {};
    for frame_id = 1:5
        if ismember(frame_id, frames_with_detection)
            dp = fixture_detection(frame_id, 1);
            remaining = dp;
            original_index = 1;
            n_original = 1;
        else
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        [tempTrackList, new_tracks, next_id, mask] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, params, 3, 7, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:30:120, {}, n_original);
        if frame_id < 5
            assert(isempty(new_tracks));
        else
            created = new_tracks;
            assert(length(mask) == 1 && mask(1));
        end
    end
    assert(length(created) == 1);
    trk = created{1};
    assert(trk.Type == params.RELIABLE_TRACK);
    assert(trk.birth_frame == 1 && trk.confirm_frame == 5);
    assert(trk.TotalPointCnt == 5 && trk.AsscPointCnt == 3 && trk.TotalLostPointCnt == 2);
    assert(trk.life == 5 && length(trk.asscPointList) == 3);

    tempTrackList = struct([]);
    next_id = 1;
    hits = [1, 2, 8];
    for frame_id = 1:8
        if ismember(frame_id, hits)
            remaining = fixture_detection(frame_id, 1);
            original_index = 1;
            n_original = 1;
        else
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        [tempTrackList, new_tracks, next_id] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, params, 3, 7, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:30:210, {}, n_original);
        assert(isempty(new_tracks));
    end

    test_current_hit_required(params, ukf_tpl, truth_all);
    test_association_frame_and_duplicate_candidates(params);
    test_truth_termination_switch(params, ukf_tpl);
    test_invalid_creation_history(params, ukf_tpl);

    disp('oracle lifecycle tests ok');
end

function test_current_hit_required(params, ukf_tpl, truth_all)
    tempTrackList = struct([]);
    next_id = 1;
    for frame_id = 1:4
        if frame_id <= 3
            remaining = fixture_detection(frame_id, 1);
            original_index = 1;
            n_original = 1;
        else
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        [tempTrackList, new_tracks, next_id] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, params, 3, 7, ukf_tpl, ...
            params, frame_id, next_id, truth_all, 0:30:120, {}, n_original);
        if frame_id < 3
            assert(isempty(new_tracks));
        elseif frame_id == 3
            assert(length(new_tracks) == 1);
        else
            assert(isempty(new_tracks));
        end
    end

    preloaded = struct('truth_idx', 1, 'pointHistory', ...
        struct('frameID', {1,2,3}, ...
        'point', {fixture_detection(1,1), fixture_detection(2,1), fixture_detection(3,1)}, ...
        'origIndex', {1,1,1}), 'missCount', 0);
    [~, new_tracks] = trackStarter_logic_oracle(preloaded, [], [], params, 3, 7, ...
        ukf_tpl, params, 4, 1, truth_all, 0:30:120, {}, 0);
    assert(isempty(new_tracks));
end

function test_association_frame_and_duplicate_candidates(params)
    trk = fixture_track(params);
    trk.x_pred = [129.02; 0; 31.02; 0];
    stale = fixture_detection(1, 1);
    current_far = fixture_detection(2, 1);
    current_far.lon = 130;
    current_far.lat = 32;
    current_near = fixture_detection(2, 1);
    current_near.lon = 129.021;
    current_near.lat = 31.021;
    clutter = fixture_detection(2, 0);
    clutter.is_clutter = true;
    [matches, remaining, used] = PointTrackAssociation_Oracle( ...
        {trk}, [stale, current_far, current_near, clutter], 2);
    assert(matches(1,2) == 3);
    assert(isequal(find(used), 3));
    assert(isequal(remaining, [1,2,4]));
end

function test_truth_termination_switch(params, ukf_tpl)
    truth_all = {[0,0,0,0,0; 1,1,0,0,30]};
    t_grid = [0,30,60];
    trk = fixture_track(params);
    trk.ukf = ukf_tpl;
    trk.ukf = ukf_dispatch('init', trk.ukf, ...
        fixture_detection(1,1), fixture_detection(2,1));
    trk.asscPointList = {fixture_detection(1,1), fixture_detection(2,1)};

    enabled = params;
    enabled.oracle_truth_terminate_enable = true;
    [tracks, ~, ~, ~, diag] = Track_Process_for_HighRate_Oracle( ...
        {trk}, struct([]), [], ukf_tpl, enabled, 3, 2, truth_all, t_grid);
    assert(tracks{1}.Type == params.HISTORY_TRACK);
    assert(strcmp(tracks{1}.death_reason, 'truth_ended'));
    assert(length(diag.lifecycle_events) == 1);

    disabled = params;
    disabled.oracle_truth_terminate_enable = false;
    [tracks, ~, ~, ~, diag] = Track_Process_for_HighRate_Oracle( ...
        {trk}, struct([]), [], ukf_tpl, disabled, 3, 2, truth_all, t_grid);
    assert(tracks{1}.Type ~= params.HISTORY_TRACK);
    assert(isempty(diag.lifecycle_events));
end

function test_invalid_creation_history(params, ukf_tpl)
    failed = false;
    try
        fun_create_new_track_oracle(fixture_detection(1,1), ...
            fixture_detection(2,1), ukf_tpl, params, 2, 1, 1, struct([]));
    catch exception
        failed = strcmp(exception.identifier, ...
            'fun_create_new_track_oracle:invalidHistory');
    end
    assert(failed);
end

function track = fixture_track(params)
    ukf = struct('x', zeros(4,1), 'P', eye(4), 'Q', eye(4));
    track = struct('id', 1, 'truth_idx', 1, 'Type', params.RELIABLE_TRACK, ...
        'type', params.RELIABLE_TRACK, 'Quality', 1, 'quality', 1, ...
        'TotalPointCnt', 0, 'AsscPointCnt', 0, 'TotalLostPointCnt', 0, ...
        'SuccLossPointCnt', 0, 'missed', 0, 'life', 0, 'death_frame', [], ...
        'death_reason', '', 'ukf', ukf, 'smoothPointList', {{}}, ...
        'outputPointList', {{}}, 'isNewTrack', 0);
end

function dp = fixture_detection(frame_id, aircraft_id)
    dp = struct('frameID', frame_id, 'time_sec', (frame_id-1)*30, ...
        'prange', 1.5e6 + frame_id*1000, 'paz', 90 + frame_id*0.01, 'pvr', 100, ...
        'range_meas', 1.5e6 + frame_id*1000, 'azimuth_meas', 90 + frame_id*0.01, ...
        'radial_vel_meas', 100, 'drange', 1.5e6 + frame_id*1000, ...
        'daz', 90 + frame_id*0.01, 'lat', 31 + frame_id*0.01, ...
        'lon', 129 + frame_id*0.01, 'is_clutter', false, ...
        'aircraft_id', int32(aircraft_id));
end
