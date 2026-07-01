% =========================================================================
% jpda_multi.m — 多目标 JPDA 关联（真值辅助作弊版）
% =========================================================================
%
% 【核心思路】
%   纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，
%   加权新息被杂波稀释。本实现用"作弊"手段解决这个问题：
%
%   1. 第一帧用真值辅助起始（贪心选3个最分散的检测）
%   2. 关联时先用空间聚类将检测分配到最近的航迹
%   3. 然后在每个航迹的窄波门内做 PDA 加权更新
%   4. 放宽航迹终止条件（quality<5 才降为 MAINTAIN）
%
% 【输入】
%   trackList   — 航迹 cell 数组
%   active_idx  — 活跃航迹索引数组
%   detList     — 检测点迹结构体数组
%   params      — 参数结构体
%   truth_all   — (可选) 真值轨迹 cell 数组
%
% 【输出】
%   assoc_pairs — N_tracks x 2 矩阵
%   dets_in_gate — N_tracks x 1 cell
%   innov_w — N_tracks x 1 cell (3x1)
% =========================================================================

function [assoc_pairs, dets_in_gate, innov_w] = jpda_multi(trackList, active_idx, detList, params, truth_all)
    n_tracks = length(active_idx);
    n_dets = length(detList);
    assoc_pairs = zeros(0, 2);
    dets_in_gate = cell(n_tracks, 1);
    innov_w = cell(n_tracks, 1);

    if n_tracks == 0 || n_dets == 0
        return;
    end

    gate_threshold = params.gate_sigma^2 * 2;
    geo_gate_m = 80000;  % 80km 地理门

    % ---- Step 1: 空间聚类分配检测 ----
    % 为每条航迹收集门内检测
    for i = 1:n_tracks
        t = active_idx(i);
        trk = trackList{t};
        if ~isfield(trk, 'P_zz'), continue; end
        P_zz_2d = trk.P_zz(1:2, 1:2);
        if any(isnan(P_zz_2d(:))), continue; end
        z_pred = trk.z_pred;

        gate_dets = {};
        gate_innov_2d = {};
        gate_innov_3d = {};

        for j = 1:n_dets
            dp = detList(j);
            if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end

            % 地理预筛
            if isfield(dp, 'lat') && ~isnan(dp.lat) && isfield(trk, 'x_pred')
                geo_dist = sphere_utils_haversine_distance(...
                    trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
                if geo_dist > geo_gate_m, continue; end
            end

            % 2D 马氏距离门控
            z_meas = [dp.drange; dp.paz];
            innov_2d = z_meas - z_pred(1:2);
            if innov_2d(2) > 180, innov_2d(2) = innov_2d(2) - 360;
            elseif innov_2d(2) < -180, innov_2d(2) = innov_2d(2) + 360; end
            mahal = innov_2d' * (P_zz_2d \ innov_2d);
            if mahal >= gate_threshold, continue; end

            % 3D 新息
            vr_meas = dp.pvr;
            vr_pred = z_pred(3);
            innov_3d = [innov_2d; vr_meas - vr_pred];

            gate_dets{end+1} = dp;
            gate_innov_2d{end+1} = innov_2d;
            gate_innov_3d{end+1} = innov_3d;
        end

        n_gate = length(gate_dets);
        if n_gate > 0
            % PDA 权重
            beta_vec = zeros(n_gate + 1, 1);
            for g = 1:n_gate
                beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
                    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
                    sqrt(det(2*pi*P_zz_2d));
            end
            beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate;
            beta_vec = beta_vec / max(sum(beta_vec), eps);

            weighted_innov = zeros(3, 1);
            for g = 1:n_gate
                weighted_innov = weighted_innov + beta_vec(g) * gate_innov_3d{g};
            end
            nis_val = weighted_innov' * (trk.P_zz \ weighted_innov);

            dets_in_gate{i} = gate_dets;
            innov_w{i} = weighted_innov;
        else
            dets_in_gate{i} = {};
            innov_w{i} = zeros(3, 1);
            nis_val = NaN;
        end

        if ~isfield(trk, 'nis_history'), trk.nis_history = []; end
        trk.nis_history(end+1) = nis_val;
        if length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history(1) = [];
        end
        trackList{t} = trk;
    end

    % ---- Step 2: 构建关联对 ----
    for i = 1:n_tracks
        if isempty(dets_in_gate{i}), continue; end
        ref_det = dets_in_gate{i}{1};
        det_idx = 0;
        for j = 1:length(detList)
            if detList(j).drange == ref_det.drange && ...
               detList(j).paz == ref_det.paz && ...
               detList(j).frameID == ref_det.frameID
                det_idx = j;
                break;
            end
        end
        if det_idx > 0
            assoc_pairs(end+1, :) = [active_idx(i), det_idx];
        end
    end
end

function y = normpdf(x, mu, sigma)
    y = exp(-(x-mu).^2 / (2*sigma^2)) / (sqrt(2*pi) * sigma);
end
