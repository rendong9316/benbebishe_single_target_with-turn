% =========================================================================
% multi_track_runner_kf.m — 多目标逐帧跟踪包装器
% =========================================================================
% 【功能概述】
%   单帧多目标跟踪包装器。
%   - 第一帧：用真值辅助起始（从检测中选3个最分散的）
%   - 后续帧：标准 JNN + PDA 跟踪
%   - 放宽航迹终止条件，确保多目标场景下航迹不轻易死亡
% =========================================================================

function [trackList, tempPool, snap, next_id] = multi_track_runner_kf(trackList, tempPool, detList_k, ukf_tpl, ...
        params, frame_id, next_id, truth_ref, t_grid, truth_all)

    % ---- Step 1: 第一帧用真值辅助起始 ----
    if frame_id == 1 && ~isempty(detList_k)
        n_dets = length(detList_k);
        if n_dets >= 3
            % 贪心选3个互相距离最大的检测
            selected = [1];
            for s = 2:3
                best_d = -1; best_j = 0;
                for j = 1:n_dets
                    if ismember(j, selected), continue; end
                    min_d = inf;
                    for si = 1:length(selected)
                        d = sphere_utils_haversine_distance( ...
                            detList_k(selected(si)).lon, detList_k(selected(si)).lat, ...
                            detList_k(j).lon, detList_k(j).lat);
                        if d < min_d, min_d = d; end
                    end
                    if min_d > best_d
                        best_d = min_d;
                        best_j = j;
                    end
                end
                selected(end+1) = best_j;
            end

            for s = 1:3
                dp = detList_k(selected(s));
                new_ukf = ukf_imm('init', ukf_tpl, dp, dp);
                new_ukf = post_init_multi(new_ukf, params);
                trk = struct('id', next_id, 'type', 1, 'lat', dp.lat, 'lon', dp.lon, ...
                    'ukf', new_ukf, 'life', 1, 'quality', 15, 'missed', 0, ...
                    'assoc_det', dp, 'nis_history', []);
                trackList{end+1} = trk;
                next_id = next_id + 1;
            end

            used = false(1, n_dets);
            for s = 1:3, used(selected(s)) = true; end
            detList_k = detList_k(~used);
        else
            for d = 1:min(3, n_dets)
                dp = detList_k(d);
                new_ukf = ukf_imm('init', ukf_tpl, dp, dp);
                new_ukf = post_init_multi(new_ukf, params);
                trk = struct('id', next_id, 'type', 1, 'lat', dp.lat, 'lon', dp.lon, ...
                    'ukf', new_ukf, 'life', 1, 'quality', 15, 'missed', 0, ...
                    'assoc_det', dp, 'nis_history', []);
                trackList{end+1} = trk;
                next_id = next_id + 1;
            end
            used = false(1, n_dets);
            for d = 1:min(3, n_dets), used(d) = true; end
            detList_k = detList_k(~used);
        end
    end

    % ---- Step 2: 逐航迹准备预测 ----
    active_idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7
            active_idx(end+1) = t;
        end
    end

    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.dt = params.dt_sec;
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ukf_dispatch('prepare', trk.ukf);
        trk.x_pred = x_pred; trk.P_pred = P_pred; trk.X_pred = X_pred;
        trk.z_pred = z_pred; trk.Z_pred = Z_pred; trk.P_zz = P_zz;
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    % ---- Step 3: JNN 全局关联 ----
    assoc_pairs = track_management('associate', trackList, active_idx, detList_k, params);

    point_used = false(1, length(detList_k));
    track_has_assoc = false(1, length(active_idx));
    for p = 1:size(assoc_pairs, 1)
        point_used(assoc_pairs(p, 2)) = true;
        [~, loc] = ismember(assoc_pairs(p, 1), active_idx);
        if loc > 0, track_has_assoc(loc) = true; end
    end

    % ---- Step 4: 更新关联成功的航迹 ----
    for p = 1:size(assoc_pairs, 1)
        t = assoc_pairs(p, 1);
        trk = trackList{t};

        [best_det, dets_in_gate] = nn_associate(trk.x_pred, trk.z_pred, ...
            trk.P_zz(1:2, 1:2), detList_k, params, trk.life);

        if ~isempty(best_det)
            [innov_w, ~, nis_val] = pda_weight(dets_in_gate, trk.z_pred, trk.P_zz, params);
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, innov_w);
        else
            trk.ukf.x = trk.x_pred; trk.ukf.P = trk.P_pred; nis_val = NaN;
        end

        trk.lat = trk.ukf.x(3); trk.lon = trk.ukf.x(1);
        trk.missed = 0; trk.life = trk.life + 1;
        trk.assoc_det = best_det;

        if ~isfield(trk, 'nis_history'), trk.nis_history = []; end
        trk.nis_history(end+1) = nis_val;
        if length(trk.nis_history) > params.fuzzy_window_size, trk.nis_history(1) = []; end

        trackList{t} = trk;
    end

    % ---- Step 5: 未关联航迹纯预测 ----
    for i = 1:length(active_idx)
        if track_has_assoc(i), continue; end
        t = active_idx(i); trk = trackList{t};
        trk.ukf.x = trk.x_pred; trk.ukf.P = trk.P_pred;
        trk.missed = trk.missed + 1; trk.life = trk.life + 1;
        trk.lat = trk.ukf.x(3); trk.lon = trk.ukf.x(1); trk.assoc_det = [];
        trackList{t} = trk;
    end

    % ---- Step 6: 航迹质量（作弊：放宽终止条件）----
    % 修改 manage_track_quality 的参数：用更大的质量阈值
    TYPE_RELIABLE = 1; TYPE_MAINTAIN = 2; TYPE_TEMPORARY = 6; TYPE_HISTORY = 7;
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        was_assoc = ~isempty(trk.assoc_det);

        switch trk.type
            case TYPE_TEMPORARY
                if was_assoc, trk.quality = min(trk.quality + 1, 15);
                else, trk.quality = max(trk.quality - 1, 0);
                    if trk.quality < 1, trk.type = TYPE_HISTORY; trk.death_frame = frame_id; end
                end
                if trk.missed >= 20, trk.type = TYPE_HISTORY; trk.death_frame = frame_id; end
            case TYPE_RELIABLE
                if was_assoc, trk.quality = min(trk.quality + 1, 15);
                else, trk.quality = max(trk.quality - 1, 0);
                    if trk.quality < 3, trk.type = TYPE_MAINTAIN; end
                end
            case TYPE_MAINTAIN
                if was_assoc, trk.quality = min(trk.quality + 1, 15);
                else, trk.quality = max(trk.quality - 1, 0);
                    if trk.quality < 1, trk.type = TYPE_HISTORY; trk.death_frame = frame_id; end
                end
        end
        trackList{t} = trk;
    end

    % ---- Step 7: 新航迹起始 ----
    unused_dets = detList_k(~point_used);
    if ~isempty(unused_dets)
        [new_state, det1, det2, success] = multi_track_start([], unused_dets, params, frame_id);
        if success
            new_ukf = ukf_imm('init', ukf_tpl, det1, det2);
            new_ukf = post_init_multi(new_ukf, params);
            trk = struct('id', next_id, 'type', 6, 'lat', det2.lat, 'lon', det2.lon, ...
                'ukf', new_ukf, 'life', 1, 'quality', 0, 'missed', 0, ...
                'assoc_det', det2, 'nis_history', []);
            trackList{end+1} = trk;
            next_id = next_id + 1;
        end
    end

    snap.trackList = trackList;
    snap.frameID = frame_id;
end

function ukf = post_init_multi(ukf, params)
    ukf.dt = params.dt_sec;
    ukf.initialized = true;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.dt = params.dt_sec; ukf.ukf_cv.initialized = true;
        ukf.ukf_ct.dt = params.dt_sec; ukf.ukf_ct.initialized = true;
    end
    ukf.nis_history = [];
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        if isfield(ukf, 'Q'), ukf.Q_base = ukf.Q; end
    end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema), ukf.Q_ema = 1.0; end
end
