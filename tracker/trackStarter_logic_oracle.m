function [tempTrackList, valid_tracks, next_id, starter_used_original] = trackStarter_logic_oracle( ...
        tempTrackList, remainingPointList, pointOriginalIndex, sysPara, QUALIFY_NUM, TOLERANT_NUM, ...
        ukf_tpl, params, frame_id, next_id, truth_all, t_grid, activeTrackList, n_original_points)

    if nargin < 14
        n_original_points = 0;
        if ~isempty(pointOriginalIndex)
            n_original_points = max(pointOriginalIndex);
        end
    end
    n_targets = length(truth_all);
    tempTrackList = ensure_temp_track_list(tempTrackList, n_targets);
    valid_tracks = {};
    starter_used_original = false(1, n_original_points);

    for ac = 1:n_targets
        if has_active_truth(activeTrackList, ac, params.HISTORY_TRACK)
            tempTrackList(ac).pointHistory = struct('frameID', {}, 'point', {}, 'origIndex', {});
            tempTrackList(ac).missCount = 0;
            continue;
        end

        cand_idx = find_truth_points(remainingPointList, ac, starter_used_original, pointOriginalIndex);
        if ~isempty(cand_idx)
            j = cand_idx(1);
            original_index = pointOriginalIndex(j);
            starter_used_original(original_index) = true;
            tempTrackList(ac).pointHistory(end+1) = struct('frameID', frame_id, ...
                'point', remainingPointList(j), 'origIndex', original_index);
            tempTrackList(ac).missCount = 0;
        else
            tempTrackList(ac).pointHistory(end+1) = struct('frameID', frame_id, ...
                'point', [], 'origIndex', 0);
            tempTrackList(ac).missCount = tempTrackList(ac).missCount + 1;
        end

        if length(tempTrackList(ac).pointHistory) > TOLERANT_NUM
            tempTrackList(ac).pointHistory = tempTrackList(ac).pointHistory(end-TOLERANT_NUM+1:end);
        end

        real_hist = collect_real_history(tempTrackList(ac).pointHistory);
        if length(real_hist) >= QUALIFY_NUM
            det1 = real_hist(1).point;
            det2 = real_hist(end).point;
            newTrack = fun_create_new_track_oracle(det1, det2, ukf_tpl, params, ...
                frame_id, next_id, ac, real_hist);
            valid_tracks{end+1} = newTrack;
            next_id = next_id + 1;
            tempTrackList(ac).pointHistory = struct('frameID', {}, 'point', {}, 'origIndex', {});
            tempTrackList(ac).missCount = 0;
        end
    end
end

function tempTrackList = ensure_temp_track_list(tempTrackList, n_targets)
    empty_hist = struct('frameID', {}, 'point', {}, 'origIndex', {});
    if isempty(tempTrackList)
        tempTrackList = repmat(struct('truth_idx', [], 'pointHistory', empty_hist, 'missCount', 0), 1, n_targets);
    end
    for ac = 1:n_targets
        if length(tempTrackList) < ac || isempty(tempTrackList(ac).truth_idx)
            tempTrackList(ac).truth_idx = ac;
            tempTrackList(ac).pointHistory = empty_hist;
            tempTrackList(ac).missCount = 0;
        end
    end
end

function tf = has_active_truth(activeTrackList, ac, history_type)
    tf = false;
    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        type = trk.type;
        if isfield(trk, 'Type'), type = trk.Type; end
        if type ~= history_type && isfield(trk, 'truth_idx') && double(trk.truth_idx) == ac
            tf = true;
            return;
        end
    end
end

function idx = find_truth_points(pointList, ac, used_original, pointOriginalIndex)
    idx = [];
    for i = 1:length(pointList)
        original_index = pointOriginalIndex(i);
        if used_original(original_index) || ~isfield(pointList(i), 'aircraft_id')
            continue;
        end
        if double(pointList(i).aircraft_id) == ac && ...
                ~(isfield(pointList(i), 'is_clutter') && pointList(i).is_clutter)
            idx(end+1) = i;
        end
    end
end

function real_hist = collect_real_history(hist)
    real_hist = struct('frameID', {}, 'point', {}, 'origIndex', {});
    for i = 1:length(hist)
        if ~isempty(hist(i).point) && isstruct(hist(i).point) && isfield(hist(i).point, 'drange')
            real_hist(end+1) = hist(i);
        end
    end
end
