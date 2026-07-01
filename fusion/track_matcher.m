% =========================================================================
% track_matcher.m — 跨雷达航迹时空配对模块
% =========================================================================
%
% 【功能概述】
%   将 R1 和 R2 两部雷达的航迹列表按时空接近度进行配对，
%   输出 matched_pairs 数组，供 run_track_fusion 使用。
%
%   配对逻辑：
%   1. 对每帧，找出 R1 和 R2 中空间接近的航迹对
%   2. 统计共现帧数（coexist_count）和平均距离
%   3. 满足最低共现帧数的配对确认为有效匹配
%
% 【输入】
%   trackSnapshots_R1 — [n_frames x 1] cell，R1 各帧航迹快照
%   trackSnapshots_R2 — [n_frames x 1] cell，R2 各帧航迹快照（已对齐）
%   params            — 参数结构体（含 tracker_coexist_threshold）
%
% 【输出】
%   matched_pairs — struct 数组，每个元素：
%     .R1_track_id  — R1 航迹ID
%     .R2_track_id  — R2 航迹ID
%     .match_count  — 共现帧数
%     .coexist_count — 连续共现最长帧数
%     .match_ratio  — match_count / total_overlap
%     .mean_dist_km — 平均距离（km）
%     .quality      — 配对质量评分（0-100）
%
% 【调用关系】
%   被 run_simulation_multi.m Phase 7 调用
%   内部调用: sphere_utils_haversine_distance
% =========================================================================

function matched_pairs = track_matcher(trackSnapshots_R1, trackSnapshots_R2, params)
    n_frames = length(trackSnapshots_R1);
    coexist_thresh = 5;  % 最少共现帧数

    % ---- Step 1: 逐帧提取活跃航迹ID和位置 ----
    r1_active = cell(n_frames, 1);
    r2_active = cell(n_frames, 1);

    for k = 1:n_frames
        r1_active{k} = extract_active_tracks(trackSnapshots_R1{k});
        r2_active{k} = extract_active_tracks(trackSnapshots_R2{k});
    end

    % ---- Step 2: 基于时空接近度的贪心配对 ----
    all_candidates = {};  % {R1_id, R2_id, dist_km, frame}

    for k = 1:n_frames
        n_r1 = length(r1_active{k});
        n_r2 = length(r2_active{k});
        if n_r1 == 0 || n_r2 == 0, continue; end

        for i = 1:n_r1
            for j = 1:n_r2
                t1 = r1_active{k}{i};
                t2 = r2_active{k}{j};
                if isnan(t1.lat) || isnan(t2.lat), continue; end
                d = sphere_utils_haversine_distance(t1.lon, t1.lat, t2.lon, t2.lat) / 1000;
                all_candidates{end+1} = struct('r1_id', t1.id, 'r2_id', t2.id, ...
                    'dist_km', d, 'frame', k);
            end
        end
    end

    % ---- Step 3: 按 (R1_id, R2_id) 聚类 ----
    clusters = {};  % {r1_id, r2_id, dist_sum, count, frames}

    for c = 1:length(all_candidates)
        cand = all_candidates{c};
        r1_id = int32(cand.r1_id);
        r2_id = int32(cand.r2_id);

        % 查找是否已有该聚类
        found = false;
        for cl = 1:length(clusters)
            if clusters{cl}.r1_id == r1_id && clusters{cl}.r2_id == r2_id
                clusters{cl}.dist_sum = clusters{cl}.dist_sum + cand.dist_km;
                clusters{cl}.count = clusters{cl}.count + 1;
                clusters{cl}.frames = [clusters{cl}.frames, cand.frame];
                found = true;
                break;
            end
        end
        if ~found
            clusters{end+1} = struct('r1_id', r1_id, 'r2_id', r2_id, ...
                'dist_sum', cand.dist_km, 'count', 1, 'frames', cand.frame);
        end
    end

    % ---- Step 4: 构建 matched_pairs 输出 ----
    n_pairs = length(clusters);
    matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {}, ...
        'match_count', {}, 'coexist_count', {}, 'match_ratio', {}, ...
        'mean_dist_km', {}, 'quality', {});

    idx = 1;
    for cl = 1:length(clusters)
        ps = clusters{cl};
        n = ps.count;
        mean_dist = ps.dist_sum / n;

        % 计算共现帧数（连续帧计数）
        frames = ps.frames;
        sorted_frames = sort(frames);
        coexist = 1; max_coexist = 1;
        for f = 2:length(sorted_frames)
            if sorted_frames(f) == sorted_frames(f-1) + 1
                coexist = coexist + 1;
                max_coexist = max(max_coexist, coexist);
            else
                coexist = 1;
            end
        end

        % 质量评分：距离越近越好，共现越多越好
        dist_score = max(0, 100 - mean_dist * 5);  % 每km扣5分
        coexist_score = min(100, max_coexist * 10);
        quality = 0.6 * dist_score + 0.4 * coexist_score;

        if max_coexist >= coexist_thresh && mean_dist < 50  % 50km 距离门限
            matched_pairs(idx).R1_track_id = ps.r1_id;
            matched_pairs(idx).R2_track_id = ps.r2_id;
            matched_pairs(idx).match_count = n;
            matched_pairs(idx).coexist_count = max_coexist;
            matched_pairs(idx).match_ratio = n / n_frames;
            matched_pairs(idx).mean_dist_km = mean_dist;
            matched_pairs(idx).quality = quality;
            idx = idx + 1;
        end
    end

    % 如果映射表没有直接配对成功，尝试基于最近邻启发式
    if idx <= 1 && n_pairs == 0
        matched_pairs = heuristic_match(r1_active, r2_active, n_frames, coexist_thresh);
    end
end

% =========================================================================
% extract_active_tracks — 从单帧快照提取活跃航迹信息
% =========================================================================
function tracks = extract_active_tracks(snap)
    tracks = {};
    if isempty(snap.trackList), return; end
    for t = 1:length(snap.trackList)
        trk = snap.trackList{t};
        if trk.type ~= 7 && ~isnan(trk.lat)
            tracks{end+1} = struct('id', trk.id, 'lat', trk.lat, 'lon', trk.lon);
        end
    end
end

% =========================================================================
% heuristic_match — 基于最近邻的启发式配对（fallback）
% =========================================================================
function pairs = heuristic_match(r1_active, r2_active, n_frames, coexist_thresh)
    % 统计每部雷达的航迹ID出现过的帧
    r1_ids = containers.Map();
    r2_ids = containers.Map();

    for k = 1:n_frames
        for t = 1:length(r1_active{k})
            id = r1_active{k}{t}.id;
            if isKey(r1_ids, id), r1_ids(id) = [r1_ids{id}, k];
            else r1_ids(id) = k; end
        end
        for t = 1:length(r2_active{k})
            id = r2_active{k}{t}.id;
            if isKey(r2_ids, id), r2_ids(id) = [r2_ids{id}, k];
            else r2_ids(id) = k; end
        end
    end

    % 收集所有出现的ID
    all_r1 = cell(1, length(r1_ids));
    all_r2 = cell(1, length(r2_ids));
    i = 1;
    for k = r1_ids.keys, all_r1{i} = k; i = i+1; end
    i = 1;
    for k = r2_ids.keys, all_r2{i} = k; i = i+1; end

    % 两两配对，找共现帧最多的
    best_pairs = {};
    for i = 1:length(all_r1)
        for j = 1:length(all_r2)
            frames1 = r1_ids{all_r1{i}};
            frames2 = r2_ids{all_r2{j}};
            overlap = sum(ismember(frames1, frames2));
            if overlap >= coexist_thresh
                best_pairs{end+1} = struct('r1_id', str2double(all_r1{i}), ...
                    'r2_id', str2double(all_r2{j}), 'overlap', overlap);
            end
        end
    end

    pairs = struct('R1_track_id', {}, 'R2_track_id', {}, ...
        'match_count', {}, 'coexist_count', {}, 'match_ratio', {}, ...
        'mean_dist_km', {}, 'quality', {});
    for p = 1:length(best_pairs)
        bp = best_pairs{p};
        pairs(p).R1_track_id = bp.r1_id;
        pairs(p).R2_track_id = bp.r2_id;
        pairs(p).match_count = bp.overlap;
        pairs(p).coexist_count = bp.overlap;
        pairs(p).match_ratio = bp.overlap / n_frames;
        pairs(p).mean_dist_km = 0;
        pairs(p).quality = 80;
    end
end
