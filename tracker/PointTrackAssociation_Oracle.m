function [TPmatch_result, singlePointsIndex, used_det] = PointTrackAssociation_Oracle(trackList, pointList, sysPara)
    n_tracks = length(trackList);
    n_points = length(pointList);
    TPmatch_result = zeros(n_tracks, 2);
    used_det = false(1, n_points);

    for i = 1:n_tracks
        TPmatch_result(i, :) = [i, 0];
        trk = trackList{i};
        if ~isfield(trk, 'truth_idx') || isempty(trk.truth_idx) || isnan(trk.truth_idx)
            continue;
        end
        best_j = 0;
        best_d = inf;
        for j = 1:n_points
            if used_det(j)
                continue;
            end
            dp = pointList(j);
            if ~isfield(dp, 'aircraft_id') || double(dp.aircraft_id) ~= double(trk.truth_idx)
                continue;
            end
            if isfield(dp, 'is_clutter') && dp.is_clutter
                continue;
            end
            d = oracle_point_distance(trk, dp);
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end
        if best_j > 0
            TPmatch_result(i, :) = [i, best_j];
            used_det(best_j) = true;
        end
    end
    singlePointsIndex = find(~used_det);
end

function d = oracle_point_distance(trk, dp)
    if isfield(trk, 'x_pred') && numel(trk.x_pred) >= 3 && isfield(dp, 'lon') && isfield(dp, 'lat') ...
            && ~isnan(dp.lon) && ~isnan(dp.lat)
        d = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
    elseif isfield(trk, 'lon') && isfield(trk, 'lat') && isfield(dp, 'lon') && isfield(dp, 'lat') ...
            && ~isnan(trk.lon) && ~isnan(trk.lat) && ~isnan(dp.lon) && ~isnan(dp.lat)
        d = sphere_utils_haversine_distance(trk.lon, trk.lat, dp.lon, dp.lat);
    else
        d = 0;
    end
end
