% =========================================================================
% track_management.m
% 航迹管理模块 — 过程式调度器，包含 JNN 关联和航迹质量管理。
% =========================================================================
%   assoc_pairs = track_management('associate', trackList, active_idx, detList, params)
%   trackList = track_management('quality', trackList, active_idx, params, frame_id)
% =========================================================================

function varargout = track_management(action, varargin)
    switch action
        case 'associate'
            varargout{1} = jnn_association(varargin{:});
        case 'quality'
            varargout{1} = manage_track_quality(varargin{:});
        otherwise
            error('track_management: unknown action ''%s''', action);
    end
end


% =========================================================================
% jnn_association — JNN 全局点迹-航迹关联
% =========================================================================
function assoc_pairs = jnn_association(trackList, active_idx, detList, params)
    n_tracks = length(active_idx);
    n_dets = length(detList);
    assoc_pairs = zeros(0, 2);
    if n_tracks == 0 || n_dets == 0, return; end

    cost = inf(n_tracks, n_dets);
    gate_threshold = params.gate_sigma^2 * 2;

    for i = 1:n_tracks
        trk = trackList{active_idx(i)};
        if ~isfield(trk, 'P_zz'), continue; end
        P_zz_2d = trk.P_zz(1:2, 1:2);
        if any(isnan(P_zz_2d(:))), continue; end
        z_pred = trk.z_pred;

        if trk.life <= 15
            geo_gate_m = 120000;
        else
            geo_gate_m = 80000;
        end
        if trk.missed > 0
            geo_gate_m = geo_gate_m + trk.missed * 15000;
        end

        for j = 1:n_dets
            dp = detList(j);
            if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end
            if isfield(dp, 'lat') && ~isnan(dp.lat) && ...
                    isfield(trk, 'x_pred')
                geo_dist = sphere_utils_haversine_distance(...
                    trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
                if geo_dist > geo_gate_m, continue; end
            end
            z_meas = [dp.drange; dp.daz];
            innov = z_meas - z_pred(1:2);
            if innov(2) > 180, innov(2) = innov(2) - 360;
            elseif innov(2) < -180, innov(2) = innov(2) + 360; end
            mahal = innov' * (P_zz_2d \ innov);
            if mahal < gate_threshold
                cost(i, j) = mahal;
            end
        end
    end

    available_trk = true(n_tracks, 1);
    available_pt  = true(n_dets, 1);
    while true
        best_val = inf;
        best_i = 0;
        best_j = 0;
        for i = 1:n_tracks
            if ~available_trk(i), continue; end
            for j = 1:n_dets
                if ~available_pt(j), continue; end
                if cost(i, j) < best_val
                    best_val = cost(i, j);
                    best_i = i;
                    best_j = j;
                end
            end
        end
        if isinf(best_val), break; end
        assoc_pairs(end+1, :) = [active_idx(best_i), best_j];
        available_trk(best_i) = false;
        available_pt(best_j) = false;
    end
end


% =========================================================================
% manage_track_quality — 航迹质量状态机
% =========================================================================
function trackList = manage_track_quality(trackList, active_idx, params, frame_id)
    TYPE_RELIABLE   = 1;
    TYPE_MAINTAIN   = 2;
    TYPE_TEMPORARY  = 6;
    TYPE_HISTORY    = 7;

    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        was_associated = ~isempty(trk.assoc_det);

        switch trk.type
            case TYPE_TEMPORARY
                if was_associated
                    trk.quality = min(trk.quality + 1, 15);
                    if trk.quality >= 10
                        trk.type = TYPE_RELIABLE;
                    end
                else
                    trk.quality = trk.quality - 1;
                    if trk.quality < 3
                        trk.type = TYPE_HISTORY;
                        trk.death_frame = frame_id;
                    end
                end
            case TYPE_RELIABLE
                if was_associated
                    trk.quality = min(trk.quality + 1, 15);
                else
                    trk.quality = trk.quality - 1;
                    if trk.quality < 8
                        trk.type = TYPE_MAINTAIN;
                    end
                end
            case TYPE_MAINTAIN
                if was_associated
                    trk.quality = min(trk.quality + 1, 15);
                    if trk.quality >= 10
                        trk.type = TYPE_RELIABLE;
                    end
                else
                    trk.quality = trk.quality - 1;
                    if trk.quality < 3
                        trk.type = TYPE_HISTORY;
                        trk.death_frame = frame_id;
                    end
                end
            case TYPE_HISTORY
                % absorb state, no change
        end

        if trk.type == TYPE_TEMPORARY && trk.missed >= params.tracker_K_loss
            trk.type = TYPE_HISTORY;
            trk.death_frame = frame_id;
        end

        trackList{t} = trk;
    end
end
