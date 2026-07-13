% =========================================================================
% track_matcher_dualgate.m — 双门限航迹关联算法
% =========================================================================
% 算法来源：开题报告《双门限航迹关联算法》
%
% 【核心思想】
%   分布式融合中，两个传感器各自输出独立航迹，融合中心需要判断两条航迹
%   是否来自同一真实目标。单次距离判断易受噪声/杂波/偏差干扰，故采用
%   两层分级判定 + 质量自适应修正。
%
% 【第一门限 T1：距离粗筛】
%   对每个时刻 i，计算 R1 航迹 r 与 R2 航迹 s 的位置距离 d_i。
%     - d_i < T1  → 通过，连续计数 k_count++
%     - d_i >= T1 → 不通过，k_count 立即归零
%
% 【第二门限 M：连续次数 + 方差校验】
%   当 k_count >= M（连续 M 个时刻通过距离门限），取最近 M 个匹配时刻
%   的位置差值序列，计算方差。
%     - var < var_thresh  → 长期稳定，判定关联
%     - var >= var_thresh → 短期巧合，判定不关联（计数器不清零，继续等）
%
% 【关联质量修正】
%   短航迹、低 SNR 航迹位置噪声大，T1 自适应放大：
%     life < 10  → T1 × 1.5
%     life 10-30 → T1 × 1.25
%     life > 30  → T1 × 1.0
%
% 【输入】
%   trackSnapshots_R1, trackSnapshots_R2 — [n_frames x 1] cell，每帧快照
%   params — 含双门限参数
%
% 【输出】
%   matched_pairs — struct 数组，每元素含 R1_track_id, R2_track_id,
%                   match_count, coexist_count, mean_dist_km, var_dist,
%                   match_frames, quality
% =========================================================================

function matched_pairs = track_matcher_dualgate(trackSnapshots_R1, trackSnapshots_R2, params)
    n_frames = length(trackSnapshots_R1);
    T1_base = get_param_local(params, 'dualgate_T1_km', 50);
    M = get_param_local(params, 'dualgate_M', 5);
    var_thresh = get_param_local(params, 'dualgate_var_km2', 100);
    coexist_thresh = get_param_local(params, 'dualgate_coexist_thresh', 3);

    % 收集 R1/R2 所有曾经活跃的航迹 ID
    r1_ids = collect_all_active_ids(trackSnapshots_R1);
    r2_ids = collect_all_active_ids(trackSnapshots_R2);

    matched_pairs = {};
    if isempty(r1_ids) || isempty(r2_ids)
        return;
    end

    % 预计算每条航迹的最大 life（用于质量修正）
    r1_max_life = compute_max_life(trackSnapshots_R1, r1_ids);
    r2_max_life = compute_max_life(trackSnapshots_R2, r2_ids);

    % 对每对 (r1_id, r2_id) 跑双门限判定
    for i = 1:length(r1_ids)
        r1_id = r1_ids(i);
        for j = 1:length(r2_ids)
            r2_id = r2_ids(j);

            % 质量修正后的 T1
            min_life = min(r1_max_life(i), r2_max_life(j));
            T1_adj = compute_quality_adjusted_T1(T1_base, min_life);

            % 逐帧距离序列
            dist_series = compute_per_frame_distance(...
                trackSnapshots_R1, trackSnapshots_R2, r1_id, r2_id, n_frames);

            % 双门限判定
            [matched, match_info] = dual_threshold_decide(...
                dist_series, T1_adj, M, var_thresh, coexist_thresh);

            if matched
                mp = struct(...
                    'R1_track_id', r1_id, ...
                    'R2_track_id', r2_id, ...
                    'match_count', match_info.n_match_frames, ...
                    'coexist_count', match_info.n_match_frames, ...
                    'match_ratio', match_info.n_match_frames / n_frames, ...
                    'mean_dist_km', match_info.mean_dist, ...
                    'mean_speed_diff', 0, ...
                    'mean_heading_diff', 0, ...
                    'var_dist', match_info.var_dist, ...
                    'match_frames', match_info.match_frames, ...
                    'quality', compute_quality_score(match_info.mean_dist, ...
                        match_info.n_match_frames, n_frames));
                matched_pairs{end+1} = mp;
            end
        end
    end

    % 转为 struct 数组
    if ~isempty(matched_pairs)
        matched_pairs = struct2table_vertcat(matched_pairs);
    else
        matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {});
    end

    % 互斥后处理：每条 R1 航迹只保留得分最高的 R2 匹配，反之亦然
    if get_param_local(params, 'dualgate_mutual_exclusion', true) && ...
       ~isempty(matched_pairs) && isfield(matched_pairs, 'quality')
        matched_pairs = apply_mutual_exclusion(matched_pairs);
    end

    fprintf('[双门限匹配] T1=%.1fkm M=%d var_thresh=%.1fkm² → 匹配 %d 对\n', ...
        T1_base, M, var_thresh, length(matched_pairs));
end


% =========================================================================
% apply_mutual_exclusion — 一对一互斥：每条 R1/R2 航迹只保留最佳匹配
% =========================================================================
function matched_pairs = apply_mutual_exclusion(matched_pairs)
    % 按 quality 降序排
    [~, idx] = sort([matched_pairs.quality], 'descend');
    matched_pairs = matched_pairs(idx);

    used_r1 = [];
    used_r2 = [];
    keep = false(length(matched_pairs), 1);
    for i = 1:length(matched_pairs)
        mp = matched_pairs(i);
        if ismember(mp.R1_track_id, used_r1) || ismember(mp.R2_track_id, used_r2)
            continue;
        end
        used_r1(end+1) = mp.R1_track_id;
        used_r2(end+1) = mp.R2_track_id;
        keep(i) = true;
    end
    matched_pairs = matched_pairs(keep);
end


% =========================================================================
% collect_all_active_ids — 收集快照中所有出现过的活跃航迹 ID
% =========================================================================
function ids = collect_all_active_ids(snaps)
    ids = [];
    for k = 1:length(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        trks = snaps{k}.trackList;
        for t = 1:length(trks)
            if trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
        ids(end+1) = trks{t}.id;
            end
        end
    end
    ids = unique(ids);
end


% =========================================================================
% compute_max_life — 每条航迹的最大 life 值
% =========================================================================
function max_life = compute_max_life(snaps, ids)
    max_life = zeros(size(ids));
    for k = 1:length(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        trks = snaps{k}.trackList;
        for t = 1:length(trks)
            trk = trks{t};
            if trk.type == 7 || isnan(trk.lat)
                continue;
            end
            idx = find(ids == trk.id, 1);
            if ~isempty(idx) && trk.life > max_life(idx)
                max_life(idx) = trk.life;
            end
        end
    end
end


% =========================================================================
% compute_quality_adjusted_T1 — 关联质量修正：短航迹 T1 放大
% =========================================================================
function T1_adj = compute_quality_adjusted_T1(T1_base, min_life)
    if min_life < 10
        factor = 1.5;
    elseif min_life < 30
        factor = 1.25;
    else
        factor = 1.0;
    end
    T1_adj = T1_base * factor;
end


% =========================================================================
% compute_per_frame_distance — 计算两条航迹在每帧的位置距离序列（km）
% =========================================================================
function dist_series = compute_per_frame_distance(snaps_R1, snaps_R2, r1_id, r2_id, n_frames)
    dist_series = nan(1, n_frames);
    for k = 1:n_frames
        r1_trk = find_track_by_id(snaps_R1{k}, r1_id);
        r2_trk = find_track_by_id(snaps_R2{k}, r2_id);
        if isempty(r1_trk) || isempty(r2_trk)
            continue;
        end
        if r1_trk.type == 7 || r2_trk.type == 7
            continue;
        end
        if isnan(r1_trk.lat) || isnan(r2_trk.lat)
            continue;
        end
        dist_series(k) = haversine_km_local(...
            r1_trk.lon, r1_trk.lat, r2_trk.lon, r2_trk.lat);
    end
end


% =========================================================================
% dual_threshold_decide — 双门限判定核心逻辑
% =========================================================================
function [matched, info] = dual_threshold_decide(dist_series, T1, M, var_thresh, coexist_thresh)
    matched = false;
    info = struct('n_match_frames', 0, 'mean_dist', NaN, ...
                  'var_dist', NaN, 'match_frames', []);

    n = length(dist_series);
    k_count = 0;
    first_pass_idx = 0;

    for i = 1:n
        if isnan(dist_series(i))
            % 任一航迹在该帧不存在，视为中断
            k_count = 0;
            first_pass_idx = 0;
            continue;
        end

        if dist_series(i) < T1
            % 通过第一门限
            if k_count == 0
                first_pass_idx = i;
            end
            k_count = k_count + 1;

            if k_count >= M
                % 进入第二门限方差校验：取最近 M 个通过时刻
                start_idx = i - M + 1;
                recent_M = dist_series(start_idx:i);
                var_D = var(recent_M);

                if var_D < var_thresh
                    % 通过方差校验 → 关联成功
                    % 扩展匹配区间到全部连续通过帧
                    ext_start = first_pass_idx;
                    ext_end = i;
                    while ext_end < n && ~isnan(dist_series(ext_end+1)) && ...
                          dist_series(ext_end+1) < T1
                        ext_end = ext_end + 1;
                    end
                    match_dists = dist_series(ext_start:ext_end);
                    match_dists = match_dists(~isnan(match_dists));
                    match_frames = (ext_start:ext_end)';

                    info.n_match_frames = length(match_frames);
                    info.mean_dist = mean(match_dists);
                    info.var_dist = var(match_dists);
                    info.match_frames = match_frames;
                    matched = true;

                    if info.n_match_frames < coexist_thresh
                        matched = false;
                    end
                    return;
                end
                % 方差未通过：k_count 不清零，继续等待后续时刻
            end
        else
            % 不通过第一门限 → k_count 立即归零
            k_count = 0;
            first_pass_idx = 0;
        end
    end
end


% =========================================================================
% compute_quality_score — 综合质量评分 0-100
% =========================================================================
function q = compute_quality_score(mean_dist, n_match, n_total)
    dist_score = 100 * exp(-mean_dist / 20);
    ratio_score = 100 * (n_match / max(n_total, 1));
    q = 0.5 * dist_score + 0.5 * ratio_score;
end


% =========================================================================
% find_track_by_id — 在快照中按 ID 找航迹
% =========================================================================
function trk = find_track_by_id(snap, tid)
    trk = [];
    if ~isfield(snap, 'trackList') || isempty(snap.trackList)
        return;
    end
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == tid
            trk = snap.trackList{t};
            return;
        end
    end
end


% =========================================================================
% haversine_km_local — Haversine 距离（km）
% =========================================================================
function d = haversine_km_local(lon1, lat1, lon2, lat2)
    R = 6371;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end


% =========================================================================
% struct2table_vertcat — cell of struct 转 struct 数组
% =========================================================================
function s = struct2table_vertcat(c)
    if isempty(c)
        s = struct();
        return;
    end
    s = c{1};
    for i = 2:length(c)
        s(end+1) = c{i};
    end
end


% =========================================================================
% get_param_local — 参数读取
% =========================================================================
function v = get_param_local(params, name, default)
    v = default;
    if isfield(params, name)
        v = params.(name);
    end
end
