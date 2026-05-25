% =========================================================================
% multi_track_manager.m
% =========================================================================
% 【功能概述】
%   多目标航迹管理主引擎，每帧执行完整的跟踪流水线：
%   分离活跃/历史航迹 → 批量UKF预测 → JNN全局关联 → 关联航迹更新
%   → 未关联航迹预测 → 质量管理 → 新航迹起始。
%
% 【调用关系】
%   主循环调用
%   子调用: find_active, track_management('associate',...),
%           ukf_jichu('predict',...), ukf_jichu('measurement',...),
%           ukf_jichu('prepare',...), nn_associate, pda_weight,
%           ukf_jichu('update',...),
%           track_management('quality',...), cleanup_stale
%   NOTE: Multi-target M/N track
%         initiation is not supported in this single-target project.
% =========================================================================

function [trackList, tempPool, trackSnapshot] = multi_track_manager(...
        trackList, tempPool, detList, ukf_tpl, params, frame_id)

    TYPE_HISTORY = 7;
    trackSnapshot = struct('frameID', frame_id, 'trackList', {{}});

    % =====================================================================
    % 特殊情况：当前帧无检测点迹 → 所有活跃航迹执行纯预测
    % =====================================================================
    if isempty(detList)
        for t = 1:length(trackList)
            trk = trackList{t};
            if trk.type == TYPE_HISTORY, continue; end

            trk.ukf.dt = params.dt_sec;
            [x_pred, P_pred, ~, trk.ukf] = ukf_jichu('predict', trk.ukf);

            trk.ukf.x = x_pred;
            trk.ukf.P = P_pred;
            trk.missed = trk.missed + 1;
            trk.life = trk.life + 1;
            trk.lat = x_pred(3);
            trk.lon = x_pred(1);
            trk.assoc_det = [];

            trackList{t} = trk;
        end
        active_idx = find_active(trackList);
        trackList = track_management('quality', trackList, active_idx, params, frame_id);
        trackSnapshot.trackList = trackList;
        return;
    end

    % ---- Step 1: 分离活跃航迹 ----
    active_idx = find_active(trackList);

    % ---- Step 2: 无活跃航迹 → 无处理 ----
    if isempty(active_idx)
        trackSnapshot.trackList = trackList;
        return;
    end

    % ---- Step 3: 批量UKF预测 ----
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.dt = params.dt_sec;

        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ukf_jichu('prepare', trk.ukf);

        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        trk.assoc_det = [];

        trackList{t} = trk;
    end

    % ---- Step 4: JNN全局点迹-航迹关联 ----
    assoc_pairs = track_management('associate', trackList, active_idx, detList, params);

    point_used = false(1, length(detList));
    track_has_assoc = false(1, length(active_idx));
    for p = 1:size(assoc_pairs, 1)
        point_used(assoc_pairs(p, 2)) = true;
        [~, loc] = ismember(assoc_pairs(p, 1), active_idx);
        if loc > 0, track_has_assoc(loc) = true; end
    end

    % ---- Step 5: 更新关联成功的航迹（模块化 PDA 更新） ----
    for p = 1:size(assoc_pairs, 1)
        t = assoc_pairs(p, 1);
        trk = trackList{t};

        % 使用 nn_associate 收集波门内所有点迹
        [best_det, dets_in_gate] = nn_associate(trk.x_pred, trk.z_pred, ...
            trk.P_zz(1:2, 1:2), detList, params, trk.life);

        if ~isempty(best_det)
            % PDA 加权新息
            [innov_w, ~, nis_val] = pda_weight(dets_in_gate, trk.z_pred, trk.P_zz, params);

            % UKF 纯 Kalman 更新
            [~, ~, trk.ukf] = ukf_jichu('update', trk.ukf, innov_w, trk.z_pred, ...
                trk.Z_pred, trk.X_pred, trk.x_pred, trk.P_pred, trk.P_zz);
        else
            % 漏检：使用 nn_associate 未找到门内点迹，执行纯预测
            trk.ukf.x = trk.x_pred;
            trk.ukf.P = trk.P_pred;
            nis_val = NaN;
        end

        trk.lat = trk.ukf.x(3);
        trk.lon = trk.ukf.x(1);
        trk.missed = 0;
        trk.life = trk.life + 1;
        trk.assoc_det = best_det;

        if ~isfield(trk, 'nis_history'), trk.nis_history = []; end
        trk.nis_history(end+1) = nis_val;
        if length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history(1) = [];
        end

        trackList{t} = trk;
    end

    % ---- Step 6: 更新未关联航迹（纯预测） ----
    for i = 1:length(active_idx)
        if track_has_assoc(i), continue; end
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.x = trk.x_pred;
        trk.ukf.P = trk.P_pred;
        trk.missed = trk.missed + 1;
        trk.life = trk.life + 1;
        trk.lat = trk.ukf.x(3);
        trk.lon = trk.ukf.x(1);
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    % ---- Step 7: 航迹质量状态机 ----
    trackList = track_management('quality', trackList, active_idx, params, frame_id);

    % ---- Step 8: 从未关联点迹起始新航迹 ----
    unused_dets = detList(~point_used);
    if ~isempty(unused_dets)
        tempPool = cleanup_stale(tempPool, frame_id, params.tracker_N);
    else
        tempPool = cleanup_stale(tempPool, frame_id, params.tracker_N);
    end

    trackSnapshot.trackList = trackList;
end


% =========================================================================
% find_active — 找出所有非HISTORY状态的航迹索引
% =========================================================================
function idx = find_active(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7
            idx(end+1) = t;
        end
    end
end


% =========================================================================
% cleanup_stale — 清理tempPool中超过N帧未更新的过期候选
% =========================================================================
function tempPool = cleanup_stale(tempPool, current_frame, N)
    keep = true(1, length(tempPool));
    for c = 1:length(tempPool)
        if current_frame - tempPool{c}.lastFrame > N
            keep(c) = false;
        end
    end
    tempPool = tempPool(keep);
end
