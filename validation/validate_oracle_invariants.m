function validate_oracle_invariants(trackSnapshots, detList, diagList, params)
    assert(abs(params.detection_probability - 0.6) < eps, 'Pd hard constraint violated');
    assert(abs(params.false_alarm_rate - 0.001) < eps, 'Pfa hard constraint violated');
    assert(params.oracle_QUALIFY_NUM == 3 && params.oracle_TOLERANT_NUM == 7, ...
        'Oracle starter must remain strict 3/7');

    for k = 1:length(trackSnapshots)
        dets = detList{k};
        diag = diagList{k};
        n_dets = length(dets);
        assert(length(diag.association_used_det) == n_dets, 'Frame %d association mask length mismatch', k);
        assert(length(diag.starter_used_det) == n_dets, 'Frame %d starter mask length mismatch', k);
        assert(length(diag.used_det) == n_dets, 'Frame %d combined mask length mismatch', k);
        assert(~any(diag.association_used_det & diag.starter_used_det), 'Frame %d detection consumed twice', k);
        assert(isequal(diag.used_det, diag.association_used_det | diag.starter_used_det), ...
            'Frame %d combined mask mismatch', k);
        assert(isequal(diag.unused_det, find(~diag.used_det)), 'Frame %d unused indices mismatch', k);
        validate_consumed_detections(dets, diag, k);

        snap = trackSnapshots{k};
        if isempty(snap) || ~isfield(snap, 'trackList'), continue; end
        seen_ids = [];
        seen_truth = [];
        for i = 1:length(snap.trackList)
            trk = snap.trackList{i};
            assert(isfield(trk, 'Type') && isfield(trk, 'Quality'), 'Frame %d missing Nanyang fields', k);
            assert(trk.type == trk.Type && trk.quality == trk.Quality, 'Frame %d alias mismatch', k);
            assert(iscell(trk.asscPointList), 'Frame %d asscPointList must be cell', k);
            validate_associated_history(trk, detList, k);
            if trk.Type ~= params.HISTORY_TRACK
                assert(~ismember(trk.id, seen_ids), 'Frame %d duplicate active id', k);
                assert(~ismember(double(trk.truth_idx), seen_truth), 'Frame %d duplicate active truth_idx', k);
                seen_ids(end+1) = trk.id;
                seen_truth(end+1) = double(trk.truth_idx);
                assert(isempty(trk.death_frame) && isempty(trk.death_reason), ...
                    'Frame %d active track has death metadata', k);
            else
                assert(~isempty(trk.death_frame) && ~isempty(trk.death_reason), ...
                    'Frame %d history track missing death metadata', k);
                if strcmp(trk.death_reason, 'k_loss')
                    assert(trk.SuccLossPointCnt >= params.tracker_K_loss, ...
                        'Frame %d k_loss death below threshold', k);
                else
                    assert(strcmp(trk.death_reason, 'truth_ended'), ...
                        'Frame %d unknown death reason', k);
                end
            end
        end
        validate_lifecycle_events(diag.lifecycle_events, params, k);
    end
end

function validate_consumed_detections(dets, diag, frame_id)
    used = find(diag.used_det);
    for j = used
        assert(~dets(j).is_clutter && double(dets(j).aircraft_id) > 0, ...
            'Frame %d consumed clutter', frame_id);
    end
    for r = 1:size(diag.TPmatch_result, 1)
        j = diag.TPmatch_result(r, 2);
        if j > 0
            assert(diag.association_used_det(j), 'Frame %d TPmatch point not marked associated', frame_id);
        end
    end
end

function validate_associated_history(trk, detList, frame_id)
    for i = 1:length(trk.asscPointList)
        dp = trk.asscPointList{i};
        assert(isstruct(dp) && ~dp.is_clutter, 'Frame %d fabricated/clutter association history', frame_id);
        assert(double(dp.aircraft_id) == double(trk.truth_idx), 'Frame %d history truth mismatch', frame_id);
        fk = double(dp.frameID);
        assert(fk >= 1 && fk <= length(detList), 'Frame %d associated point frame invalid', frame_id);
        assert(detection_exists(detList{fk}, dp), 'Frame %d associated point not in generated detections', frame_id);
    end
end

function tf = detection_exists(dets, dp)
    tf = false;
    for j = 1:length(dets)
        if double(dets(j).aircraft_id) == double(dp.aircraft_id) && ...
                abs(dets(j).time_sec - dp.time_sec) < 1e-9 && ...
                abs(dets(j).prange - dp.prange) < 1e-6 && abs(dets(j).paz - dp.paz) < 1e-9
            tf = true;
            return;
        end
    end
end

function validate_lifecycle_events(events, params, frame_id)
    for i = 1:length(events)
        event = events(i);
        if strcmp(event.event, 'confirmed')
            assert(event.frameID == frame_id && event.AsscPointCnt >= params.oracle_QUALIFY_NUM, ...
                'Frame %d invalid confirmation event', frame_id);
            assert(event.birth_frame <= event.confirm_frame, 'Frame %d invalid confirmation timing', frame_id);
            assert(event.TotalPointCnt == event.confirm_frame - event.birth_frame + 1, ...
                'Frame %d confirmation span mismatch', frame_id);
            assert(event.TotalLostPointCnt == event.TotalPointCnt - event.AsscPointCnt, ...
                'Frame %d confirmation counters mismatch', frame_id);
        elseif strcmp(event.event, 'died')
            assert(any(strcmp(event.death_reason, {'k_loss', 'truth_ended'})), ...
                'Frame %d invalid death event', frame_id);
        else
            error('Frame %d unknown lifecycle event', frame_id);
        end
    end
end
