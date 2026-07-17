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
    active_truth = build_active_truth_index(activeTrackList, n_targets, params.HISTORY_TRACK);
    candidate_by_truth = build_candidate_index(remainingPointList, pointOriginalIndex, ...
        n_targets, n_original_points, frame_id);

    for ac = 1:n_targets
        if active_truth(ac)
            tempTrackList(ac).pointHistory = empty_history();
            tempTrackList(ac).missCount = 0;
            continue;
        end

        j = candidate_by_truth(ac);
        current_hit = j > 0;
        if current_hit
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
            tempTrackList(ac).pointHistory = ...
                tempTrackList(ac).pointHistory(end-TOLERANT_NUM+1:end);
        end

        real_hist = collect_real_history(tempTrackList(ac).pointHistory);
        if current_hit && length(real_hist) >= QUALIFY_NUM
            det1 = real_hist(1).point;
            det2 = real_hist(end).point;
            valid_tracks{end+1} = fun_create_new_track_oracle(det1, det2, ukf_tpl, ...
                params, frame_id, next_id, ac, real_hist);
            next_id = next_id + 1;
            tempTrackList(ac).pointHistory = empty_history();
            tempTrackList(ac).missCount = 0;
        end
    end
end

function hist = empty_history()
    hist = struct('frameID', {}, 'point', {}, 'origIndex', {});
end

function tempTrackList = ensure_temp_track_list(tempTrackList, n_targets)
    empty_hist = empty_history();
    if isempty(tempTrackList)
        tempTrackList = repmat(struct('truth_idx', [], ...
            'pointHistory', empty_hist, 'missCount', 0), 1, n_targets);
    end
    for ac = 1:n_targets
        if length(tempTrackList) < ac || isempty(tempTrackList(ac).truth_idx)
            tempTrackList(ac).truth_idx = ac;
            tempTrackList(ac).pointHistory = empty_hist;
            tempTrackList(ac).missCount = 0;
        end
    end
end

function active_truth = build_active_truth_index(activeTrackList, n_targets, history_type)
    active_truth = false(1, n_targets);
    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        if isfield(trk, 'Type')
            type = trk.Type;
        elseif isfield(trk, 'type')
            type = trk.type;
        else
            continue;
        end
        if type == history_type || ~isfield(trk, 'truth_idx') || ...
                ~isscalar(trk.truth_idx) || ~isfinite(double(trk.truth_idx))
            continue;
        end
        truth_id = double(trk.truth_idx);
        if truth_id >= 1 && truth_id <= n_targets && truth_id == floor(truth_id)
            active_truth(truth_id) = true;
        end
    end
end

function candidate_by_truth = build_candidate_index(pointList, pointOriginalIndex, ...
        n_targets, n_original_points, frame_id)
    candidate_by_truth = zeros(1, n_targets);
    for i = 1:length(pointList)
        if i > length(pointOriginalIndex)
            error('trackStarter_logic_oracle:indexMismatch', ...
                '剩余点迹与原始索引数量不一致');
        end
        original_index = pointOriginalIndex(i);
        if original_index < 1 || original_index > n_original_points
            error('trackStarter_logic_oracle:indexOutOfRange', '原始点迹索引越界');
        end
        dp = pointList(i);
        if ~isfield(dp, 'frameID') || double(dp.frameID) ~= double(frame_id) || ...
                ~isfield(dp, 'aircraft_id') || ~isscalar(dp.aircraft_id) || ...
                ~isfinite(double(dp.aircraft_id)) || ...
                (isfield(dp, 'is_clutter') && dp.is_clutter)
            continue;
        end
        truth_id = double(dp.aircraft_id);
        if truth_id >= 1 && truth_id <= n_targets && truth_id == floor(truth_id) && ...
                candidate_by_truth(truth_id) == 0
            candidate_by_truth(truth_id) = i;
        end
    end
end

function real_hist = collect_real_history(hist)
    real_hist = empty_history();
    for i = 1:length(hist)
        if ~isempty(hist(i).point) && isstruct(hist(i).point) && ...
                isfield(hist(i).point, 'drange')
            real_hist(end+1) = hist(i);
        end
    end
end
