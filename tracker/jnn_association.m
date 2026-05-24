% =========================================================================
% jnn_association.m
% JNN 全局最近邻点迹-航迹关联
% =========================================================================
% 对所有活跃航迹与未使用点迹计算代价矩阵，通过贪心全局分配解决冲突：
%   1. 计算所有(track, point)对的马氏距离代价
%   2. 迭代选取最小代价对 → 分配 → 移除该行/列
%   3. 保证每个点迹最多关联一条航迹，每条航迹最多关联一个点迹
% =========================================================================

function assoc_pairs = jnn_association(trackList, active_idx, detList, params)
    n_tracks = length(active_idx);
    n_dets = length(detList);

    assoc_pairs = zeros(0, 2);
    if n_tracks == 0 || n_dets == 0, return; end

    % ---- 步骤1：计算代价矩阵 ----
    cost = inf(n_tracks, n_dets);
    gate_threshold = params.gate_sigma^2 * 2;

    for i = 1:n_tracks
        trk = trackList{active_idx(i)};
        if ~isfield(trk, 'P_zz'), continue; end
        P_zz_2d = trk.P_zz(1:2, 1:2);
        if any(isnan(P_zz_2d(:))), continue; end
        z_pred = trk.z_pred;

        % Adaptive geo gate + dynamic expansion on missed frames
        if trk.life <= 15
            geo_gate_m = 120000;  % 120km during UKF convergence
        else
            geo_gate_m = 80000;   % 80km for mature tracks
        end
        if trk.missed > 0
            geo_gate_m = geo_gate_m + trk.missed * 15000;
        end

        for j = 1:n_dets
            dp = detList(j);
            if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end

            % 地理距离预筛选：不同飞机的点迹可能落入同一波门
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

    % ---- 步骤2：贪心全局分配 ----
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
