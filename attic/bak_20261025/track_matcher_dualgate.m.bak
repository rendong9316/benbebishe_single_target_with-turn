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
    % 获取帧数，用于后续距离序列的长度设定
    n_frames = length(trackSnapshots_R1);
    % 从 params 中读取双门限算法参数，带默认值
    % T1_base: 第一门限距离阈值（km），默认 50m
    T1_base = get_param_local(params, 'dualgate_T1_km', 50);
    % M: 连续通过第一门限的最小帧数，默认 5 帧
    M = get_param_local(params, 'dualgate_M', 5);
    % var_thresh: 方差阈值（km^2），用于第二门限校验
    var_thresh = get_param_local(params, 'dualgate_var_km2', 100);
    % coexist_thresh: 最少共现帧数，关联结果需满足此最小持续时间
    coexist_thresh = get_param_local(params, 'dualgate_coexist_thresh', 3);

    % 收集 R1/R2 所有曾经活跃的航迹 ID
    % collect_all_active_ids 遍历所有帧快照，提取非 HISTORY 航迹的 ID
    r1_ids = collect_all_active_ids(trackSnapshots_R1);
    r2_ids = collect_all_active_ids(trackSnapshots_R2);

    matched_pairs = {};
    % 任一传感器无活跃航迹，直接返回空结果
    if isempty(r1_ids) || isempty(r2_ids)
        return;
    end

    % 预计算每条航迹的最大 life（用于质量修正）
    % compute_max_life 遍历所有帧快照，找出每条航迹出现过的最大 life 值
    r1_max_life = compute_max_life(trackSnapshots_R1, r1_ids);
    r2_max_life = compute_max_life(trackSnapshots_R2, r2_ids);

    % 对每对 (r1_id, r2_id) 跑双门限判定
    % 这是一个 O(N*M) 的笛卡尔积遍历，N 为 R1 航迹数，M 为 R2 航迹数
    for i = 1:length(r1_ids)
        r1_id = r1_ids(i);
        for j = 1:length(r2_ids)
            r2_id = r2_ids(j);

            % 质量修正后的 T1：取两条航迹中较小的 life 来决定修正因子
            % 短航迹（life 小）的 T1 会放大，降低关联门槛
            min_life = min(r1_max_life(i), r2_max_life(j));
            T1_adj = compute_quality_adjusted_T1(T1_base, min_life);

            % 逐帧距离序列：计算两条航迹在每一帧的位置距离
            % 如果某帧某条航迹不存在，对应位置为 NaN
            dist_series = compute_per_frame_distance(...
                trackSnapshots_R1, trackSnapshots_R2, r1_id, r2_id, n_frames);

            % 双门限判定核心：第一门限连续计数 + 第二门限方差校验
            % 返回 matched（是否关联）和 match_info（关联详情）
            [matched, match_info] = dual_threshold_decide(...
                dist_series, T1_adj, M, var_thresh, coexist_thresh);

            if matched
                % 构造匹配结果结构体
                mp = struct(...
                    'R1_track_id', r1_id, ...          % R1 侧航迹 ID
                    'R2_track_id', r2_id, ...          % R2 侧航迹 ID
                    'match_count', match_info.n_match_frames, ...  % 匹配帧数
                    'coexist_count', match_info.n_match_frames, ... % 共存帧数
                    'match_ratio', match_info.n_match_frames / n_frames, ... % 匹配比例
                    'mean_dist_km', match_info.mean_dist, ...  % 平均距离（km）
                    'mean_speed_diff', 0, ...  % 速度差（暂未实现）
                    'mean_heading_diff', 0, ...  % 航向差（暂未实现）
                    'var_dist', match_info.var_dist, ...  % 距离方差（km^2）
                    'match_frames', match_info.match_frames, ...  % 匹配帧号列表
                    'quality', compute_quality_score(match_info.mean_dist, ...  % 综合质量评分
                        match_info.n_match_frames, n_frames));
                % 追加到结果列表
                matched_pairs{end+1} = mp;
            end
        end
    end

    % 转为 struct 数组（cell of struct → struct array）
    % struct2table_vertcat 将元胞数组拼接为连续的结构体数组
    if ~isempty(matched_pairs)
        matched_pairs = struct2table_vertcat(matched_pairs);
    else
        % 空结果返回空结构体，保持字段名一致
        matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {});
    end

    % 互斥后处理：每条 R1 航迹只保留得分最高的 R2 匹配，反之亦然
    % 这是 1v1 匹配约束，避免一对多或多对一的歧义
    if get_param_local(params, 'dualgate_mutual_exclusion', true) && ...
       ~isempty(matched_pairs) && isfield(matched_pairs, 'quality')
        matched_pairs = apply_mutual_exclusion(matched_pairs);
    end

    % 打印匹配统计信息
    fprintf('[双门限匹配] T1=%.1fkm M=%d var_thresh=%.1fkm² → 匹配 %d 对\n', ...
        T1_base, M, var_thresh, length(matched_pairs));
end


% =========================================================================
% apply_mutual_exclusion — 一对一互斥：每条 R1/R2 航迹只保留最佳匹配
% =========================================================================
function matched_pairs = apply_mutual_exclusion(matched_pairs)
    % 按 quality 降序排，优先保留高质量匹配
    [~, idx] = sort([matched_pairs.quality], 'descend');
    matched_pairs = matched_pairs(idx);

    used_r1 = [];  % 已匹配的 R1 航迹 ID
    used_r2 = [];  % 已匹配的 R2 航迹 ID
    keep = false(length(matched_pairs), 1);  % 保留标记数组
    for i = 1:length(matched_pairs)
        mp = matched_pairs(i);
        % 如果 R1 或 R2 航迹已被更高评分的匹配占用，跳过
        if ismember(mp.R1_track_id, used_r1) || ismember(mp.R2_track_id, used_r2)
            continue;
        end
        % 标记这两个航迹已被使用
        used_r1(end+1) = mp.R1_track_id;
        used_r2(end+1) = mp.R2_track_id;
        keep(i) = true;
    end
    % 只保留未被跳过的匹配对
    matched_pairs = matched_pairs(keep);
end


% =========================================================================
% collect_all_active_ids — 收集快照中所有出现过的活跃航迹 ID
% =========================================================================
function ids = collect_all_active_ids(snaps)
    ids = [];
    % 遍历每一帧快照
    for k = 1:length(snaps)
        % 跳过没有 trackList 或 trackList 为空的帧
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        trks = snaps{k}.trackList;
        % 遍历该帧的所有航迹
        for t = 1:length(trks)
            % 排除 HISTORY 类型（type==7）和无效位置（NaN 经纬度）的航迹
            if trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
        ids(end+1) = trks{t}.id;
            end
        end
    end
    % 去重并排序
    ids = unique(ids);
end


% =========================================================================
% compute_max_life — 每条航迹的最大 life 值
% =========================================================================
function max_life = compute_max_life(snaps, ids)
    max_life = zeros(size(ids));
    % 遍历每一帧快照
    for k = 1:length(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        trks = snaps{k}.trackList;
        % 遍历该帧的所有航迹
        for t = 1:length(trks)
            trk = trks{t};
            % 跳过 HISTORY 航迹和无效位置的航迹
            if trk.type == 7 || isnan(trk.lat)
                continue;
            end
            % 查找该航迹 ID 在 ids 数组中的位置
            idx = find(ids == trk.id, 1);
            % 如果找到且当前 life 更大，更新最大值
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
    % 根据两条航迹中较小的 life 值决定 T1 修正因子
    % 短航迹（life 小）位置噪声大，放宽距离门限以提高关联召回率
    if min_life < 10
        factor = 1.5;    % 极短航迹：T1 放大 50%
    elseif min_life < 30
        factor = 1.25;   % 中等航迹：T1 放大 25%
    else
        factor = 1.0;    % 长航迹：不修正
    end
    T1_adj = T1_base * factor;
end


% =========================================================================
% compute_per_frame_distance — 计算两条航迹在每帧的位置距离序列（km）
% =========================================================================
function dist_series = compute_per_frame_distance(snaps_R1, snaps_R2, r1_id, r2_id, n_frames)
    % 初始化距离序列为 NaN（表示该帧无数据）
    dist_series = nan(1, n_frames);
    % 遍历每一帧
    for k = 1:n_frames
        % 在 R1 和 R2 的当前帧快照中分别查找对应 ID 的航迹
        r1_trk = find_track_by_id(snaps_R1{k}, r1_id);
        r2_trk = find_track_by_id(snaps_R2{k}, r2_id);
        % 任一条航迹在该帧不存在，跳过
        if isempty(r1_trk) || isempty(r2_trk)
            continue;
        end
        % 排除 HISTORY 航迹
        if r1_trk.type == 7 || r2_trk.type == 7
            continue;
        end
        % 排除位置无效的航迹
        if isnan(r1_trk.lat) || isnan(r2_trk.lat)
            continue;
        end
        % 计算两条航迹在经纬度上的 Haversine 距离（km）
        dist_series(k) = haversine_km_local(...
            r1_trk.lon, r1_trk.lat, r2_trk.lon, r2_trk.lat);
    end
end


% =========================================================================
% dual_threshold_decide — 双门限判定核心逻辑
% =========================================================================
function [matched, info] = dual_threshold_decide(dist_series, T1, M, var_thresh, coexist_thresh)
    % 双门限判定核心逻辑
    %
    % 第一门限（T1）：逐帧距离粗筛
    %   - d_i < T1 → 通过，连续计数 k_count++
    %   - d_i >= T1 → 不通过，k_count 归零
    %
    % 第二门限（M + 方差校验）：
    %   - 连续 M 帧通过 → 取最近 M 帧计算方差
    %   - var < var_thresh → 关联成功（长期稳定）
    %   - var >= var_thresh → 短期巧合，继续等待（k_count 不清零）
    %
    % 关联区间扩展：确认关联后，向前扩展到首次通过帧，向后扩展到
    %   连续通过帧的末端，得到完整的共现区间

    matched = false;  % 初始化判定结果为不匹配
    % 初始化返回信息结构体
    info = struct('n_match_frames', 0, 'mean_dist', NaN, ...
                  'var_dist', NaN, 'match_frames', []);

    n = length(dist_series);
    k_count = 0;           % 连续通过第一门限的帧数
    first_pass_idx = 0;    % 第一次通过第一门限的帧索引

    for i = 1:n
        if isnan(dist_series(i))
            % 任一航迹在该帧不存在 → 视为中断，计数器归零
            k_count = 0;
            first_pass_idx = 0;
            continue;
        end

        if dist_series(i) < T1
            % 通过第一门限
            if k_count == 0
                first_pass_idx = i;  % 记录首次通过的帧
            end
            k_count = k_count + 1;

            if k_count >= M
                % 进入第二门限：取最近 M 个通过时刻的距离计算方差
                start_idx = i - M + 1;
                recent_M = dist_series(start_idx:i);
                var_D = var(recent_M);

                if var_D < var_thresh
                    % 通过方差校验 → 关联成功
                    % 扩展匹配区间：从 first_pass_idx 延伸到连续通过的末端
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

                    % 最少共现帧数校验：关联区间必须至少持续 coexist_thresh 帧
                    if info.n_match_frames < coexist_thresh
                        matched = false;
                    end
                    return;
                end
                % 方差未通过 → k_count 不清零，继续等待后续时刻
                % 这意味着可能是短期巧合（两条不同航迹在某几帧偶然靠近）
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
    % 距离分数：距离越小分数越高，按指数衰减
    dist_score = 100 * exp(-mean_dist / 20);
    % 比例分数：匹配帧占总帧数的比例
    ratio_score = 100 * (n_match / max(n_total, 1));
    % 综合评分：距离和比例各占 50% 权重
    q = 0.5 * dist_score + 0.5 * ratio_score;
end


% =========================================================================
% find_track_by_id — 在快照中按 ID 找航迹
% =========================================================================
function trk = find_track_by_id(snap, tid)
    trk = [];
    % 快照中没有 trackList 或为空，直接返回
    if ~isfield(snap, 'trackList') || isempty(snap.trackList)
        return;
    end
    % 遍历航迹列表，查找 ID 匹配的航迹
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
    % 使用 Haversine 公式计算地球表面两点间的大圆距离（km）
    % 适用于经纬度坐标的距离计算
    R = 6371;  % 地球半径（km）
    dlat = deg2rad(lat2 - lat1);  % 纬度差转为弧度
    dlon = deg2rad(lon2 - lon1);  % 经度差转为弧度
    % Haversine 公式核心计算
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    % 钳位到 [0, 1] 防止浮点误差导致 acos 出错
    a = max(0, min(1, a));
    % 计算大圆距离
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
    % 逐个拼接 struct 数组元素
    s = c{1};
    for i = 2:length(c)
        s(end+1) = c{i};
    end
end


% =========================================================================
% get_param_local — 参数读取
% =========================================================================
function v = get_param_local(params, name, default)
    % 从 params 结构体中安全读取字段，不存在时返回默认值
    v = default;
    if isfield(params, name)
        v = params.(name);
    end
end
