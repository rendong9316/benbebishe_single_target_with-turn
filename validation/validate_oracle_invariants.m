function validate_oracle_invariants(trackSnapshots, detList, diagList, params, finalTrackList)
    if nargin < 5, finalTrackList = {}; end
    assert(abs(params.detection_probability - 0.6) < eps, 'Pd hard constraint violated');
    assert(abs(params.false_alarm_rate - 0.001) < eps, 'Pfa hard constraint violated');
    assert(params.oracle_QUALIFY_NUM == 3 && params.oracle_TOLERANT_NUM == 7, ...
        'Oracle starter must remain strict 3/7');

    confirmation_hits = cell(1, max(1, max_truth_id_from_events(diagList)));
    for k = 1:length(trackSnapshots)
        dets = detList{k};
        diag = diagList{k};
        n_dets = length(dets);
        assert(length(diag.association_used_det) == n_dets, ...
            'Frame %d association mask length mismatch', k);
        assert(length(diag.starter_used_det) == n_dets, ...
            'Frame %d starter mask length mismatch', k);
        assert(length(diag.used_det) == n_dets, ...
            'Frame %d combined mask length mismatch', k);
        assert(~any(diag.association_used_det & diag.starter_used_det), ...
            'Frame %d detection consumed twice', k);
        assert(isequal(diag.used_det, ...
            diag.association_used_det | diag.starter_used_det), ...
            'Frame %d combined mask mismatch', k);
        assert(isequal(diag.unused_det, find(~diag.used_det)), ...
            'Frame %d unused indices mismatch', k);
        validate_consumed_detections(dets, diag, k);

        snap = trackSnapshots{k};
        assert(isstruct(snap) && isfield(snap, 'trackList') && ...
            isfield(snap, 'frameID') && snap.frameID == k, ...
            'Frame %d invalid snapshot', k);
        seen_ids = [];
        seen_truth = [];
        for i = 1:length(snap.trackList)
            trk = snap.trackList{i};
            validate_slim_track(trk, params, k);
            assert(~ismember(trk.id, seen_ids), 'Frame %d duplicate active id', k);
            assert(~ismember(double(trk.truth_idx), seen_truth), ...
                'Frame %d duplicate active truth_idx', k);
            seen_ids(end+1) = trk.id;
            seen_truth(end+1) = double(trk.truth_idx);
        end
        [confirmation_hits, frame_events] = record_confirmation_hits( ...
            confirmation_hits, diag.lifecycle_events, detList, k, params);
        validate_lifecycle_events(frame_events, params, k);
    end

    validate_final_tracks(finalTrackList, detList, params);
end

function validate_slim_track(trk, params, frame_id)
    expected = {'id','type','life','truth_idx','lat','lon','P_pred','ukf'};
    assert(isequal(sort(fieldnames(trk)), sort(expected(:))), ...
        'Frame %d snapshot track is not lightweight', frame_id);
    assert(trk.type ~= params.HISTORY_TRACK, ...
        'Frame %d snapshot contains history track', frame_id);
    assert(isfield(trk.ukf, 'x') && isfield(trk.ukf, 'P') && ...
        isfield(trk.ukf, 'Q') && length(fieldnames(trk.ukf)) == 3, ...
        'Frame %d snapshot UKF is not lightweight', frame_id);
end

function validate_consumed_detections(dets, diag, frame_id)
    used = find(diag.used_det);
    for j = used
        assert(~dets(j).is_clutter && double(dets(j).aircraft_id) > 0, ...
            'Frame %d consumed clutter', frame_id);
        assert(double(dets(j).frameID) == frame_id, ...
            'Frame %d consumed stale/future detection', frame_id);
    end
    for r = 1:size(diag.TPmatch_result, 1)
        j = diag.TPmatch_result(r, 2);
        if j > 0
            assert(diag.association_used_det(j), ...
                'Frame %d TPmatch point not marked associated', frame_id);
        end
    end
end

function [hits, events] = record_confirmation_hits(hits, events, detList, frame_id, params)
    for i = 1:length(events)
        event = events(i);
        if ~strcmp(event.event, 'confirmed'), continue; end
        truth_id = double(event.truth_idx);
        if truth_id > length(hits), hits{truth_id} = []; end
        current_hit = false;
        start_frame = max(1, event.confirm_frame - params.oracle_TOLERANT_NUM + 1);
        hit_frames = [];
        for k = start_frame:event.confirm_frame
            dets = detList{k};
            for j = 1:length(dets)
                if ~dets(j).is_clutter && double(dets(j).aircraft_id) == truth_id
                    hit_frames(end+1) = k;
                    current_hit = current_hit || k == frame_id;
                    break;
                end
            end
        end
        assert(length(hit_frames) >= params.oracle_QUALIFY_NUM, ...
            'Frame %d confirmation lacks 3/7 evidence', frame_id);
        assert(current_hit, 'Frame %d confirmation was not triggered by a current hit', frame_id);
        hits{truth_id} = hit_frames;
    end
end

function validate_final_tracks(trackList, detList, params)
    for i = 1:length(trackList)
        trk = trackList{i};
        assert(isfield(trk, 'Type') && isfield(trk, 'Quality'), ...
            'Final track missing Nanyang fields');
        assert(trk.type == trk.Type && trk.quality == trk.Quality, ...
            'Final track alias mismatch');
        assert(iscell(trk.asscPointList), 'Final asscPointList must be cell');
        for j = 1:length(trk.asscPointList)
            dp = trk.asscPointList{j};
            assert(isstruct(dp) && ~dp.is_clutter, ...
                'Final track contains fabricated/clutter association');
            assert(double(dp.aircraft_id) == double(trk.truth_idx), ...
                'Final track history truth mismatch');
            frame_id = double(dp.frameID);
            assert(frame_id >= 1 && frame_id <= length(detList), ...
                'Final track associated point frame invalid');
            assert(detection_exists_exact(detList{frame_id}, dp), ...
                'Final track associated point differs from generated detection');
        end
        if trk.Type == params.HISTORY_TRACK
            assert(~isempty(trk.death_frame) && ~isempty(trk.death_reason), ...
                'History track missing death metadata');
            if strcmp(trk.death_reason, 'k_loss')
                assert(trk.SuccLossPointCnt >= params.tracker_K_loss, ...
                    'k_loss death below threshold');
            else
                assert(strcmp(trk.death_reason, 'truth_ended') && ...
                    isfield(params, 'oracle_truth_terminate_enable') && ...
                    params.oracle_truth_terminate_enable, ...
                    'Invalid truth-ended death');
            end
        else
            assert(isempty(trk.death_frame) && isempty(trk.death_reason), ...
                'Active final track has death metadata');
        end
    end
end

function tf = detection_exists_exact(dets, dp)
    tf = false;
    for j = 1:length(dets)
        if isequaln(dets(j), dp)
            tf = true;
            return;
        end
    end
end

function validate_lifecycle_events(events, params, frame_id)
    keys = {};
    for i = 1:length(events)
        event = events(i);
        key = sprintf('%s_%d', event.event, event.track_id);
        assert(~ismember(key, keys), 'Frame %d duplicate lifecycle event', frame_id);
        keys{end+1} = key;
        if strcmp(event.event, 'confirmed')
            assert(event.frameID == frame_id && ...
                event.AsscPointCnt >= params.oracle_QUALIFY_NUM, ...
                'Frame %d invalid confirmation event', frame_id);
            assert(event.birth_frame <= event.confirm_frame && ...
                event.confirm_frame == frame_id, ...
                'Frame %d invalid confirmation timing', frame_id);
            assert(event.confirm_frame - event.birth_frame + 1 <= ...
                params.oracle_TOLERANT_NUM, ...
                'Frame %d confirmation exceeds 3/7 window', frame_id);
            assert(event.TotalPointCnt == event.confirm_frame - event.birth_frame + 1, ...
                'Frame %d confirmation span mismatch', frame_id);
            assert(event.TotalLostPointCnt == ...
                event.TotalPointCnt - event.AsscPointCnt, ...
                'Frame %d confirmation counters mismatch', frame_id);
        elseif strcmp(event.event, 'died')
            assert(any(strcmp(event.death_reason, {'k_loss', 'truth_ended'})), ...
                'Frame %d invalid death event', frame_id);
            if strcmp(event.death_reason, 'truth_ended')
                assert(isfield(params, 'oracle_truth_terminate_enable') && ...
                    params.oracle_truth_terminate_enable, ...
                    'Frame %d truth termination occurred while disabled', frame_id);
            end
        else
            error('Frame %d unknown lifecycle event', frame_id);
        end
    end
end

function max_id = max_truth_id_from_events(diagList)
    max_id = 0;
    for k = 1:length(diagList)
        events = diagList{k}.lifecycle_events;
        for i = 1:length(events)
            max_id = max(max_id, double(events(i).truth_idx));
        end
    end
end
