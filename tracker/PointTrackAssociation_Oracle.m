function [TPmatch_result, singlePointsIndex, used_det] = PointTrackAssociation_Oracle( ...
        trackList, pointList, frame_id)
    n_tracks = length(trackList);
    n_points = length(pointList);
    TPmatch_result = [(1:n_tracks)', zeros(n_tracks, 1)];
    used_det = false(1, n_points);

    truth_ids = nan(1, n_tracks);
    max_truth_id = 0;
    for i = 1:n_tracks
        trk = trackList{i};
        if ~isfield(trk, 'truth_idx') || ~isscalar(trk.truth_idx) || ...
                ~isfinite(double(trk.truth_idx))
            continue;
        end
        truth_id = double(trk.truth_idx);
        if truth_id < 1 || truth_id ~= floor(truth_id)
            continue;
        end
        truth_ids(i) = truth_id;
        max_truth_id = max(max_truth_id, truth_id);
    end

    candidates = cell(1, max_truth_id);
    for j = 1:n_points
        dp = pointList(j);
        if ~is_current_real_detection(dp, frame_id)
            continue;
        end
        truth_id = double(dp.aircraft_id);
        if truth_id <= max_truth_id
            candidates{truth_id}(end+1) = j;
        end
    end

    for i = 1:n_tracks
        truth_id = truth_ids(i);
        if isnan(truth_id)
            continue;
        end
        candidate_indices = candidates{truth_id};
        best_j = 0;
        best_d = inf;
        for j = candidate_indices
            if used_det(j)
                continue;
            end
            d = oracle_point_distance(trackList{i}, pointList(j));
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end
        if best_j > 0
            TPmatch_result(i, 2) = best_j;
            used_det(best_j) = true;
        end
    end
    singlePointsIndex = find(~used_det);
end

function tf = is_current_real_detection(dp, frame_id)
    tf = isfield(dp, 'aircraft_id') && isscalar(dp.aircraft_id) && ...
        isfinite(double(dp.aircraft_id)) && double(dp.aircraft_id) >= 1 && ...
        isfield(dp, 'frameID') && isscalar(dp.frameID) && ...
        double(dp.frameID) == double(frame_id) && ...
        ~(isfield(dp, 'is_clutter') && dp.is_clutter);
end

function d = oracle_point_distance(trk, dp)
    if isfield(trk, 'x_pred') && numel(trk.x_pred) >= 3 && ...
            isfield(dp, 'lon') && isfield(dp, 'lat') && ...
            isfinite(dp.lon) && isfinite(dp.lat)
        d = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
    elseif isfield(trk, 'lon') && isfield(trk, 'lat') && ...
            isfield(dp, 'lon') && isfield(dp, 'lat') && ...
            isfinite(trk.lon) && isfinite(trk.lat) && ...
            isfinite(dp.lon) && isfinite(dp.lat)
        d = sphere_utils_haversine_distance(trk.lon, trk.lat, dp.lon, dp.lat);
    else
        d = 0;
    end
end
