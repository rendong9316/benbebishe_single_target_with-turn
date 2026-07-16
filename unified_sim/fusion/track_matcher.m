% =========================================================================
% track_matcher.m — 跨雷达航迹时空配对模块（优化版）
% =========================================================================
%
% 【功能概述】
%   将 R1 和 R2 两部雷达的航迹列表按时空接近度进行配对，
%   输出 matched_pairs 数组，供 run_track_fusion 使用。
%
%   配对逻辑（优化版）：
%   1. 多维特征：位置距离 + 速度相似度 + 航向相似度
%   2. 全局最优：每帧用匈牙利算法（枚举法）做全局最优分配
%   3. 时序统计：统计共现帧数、平均距离、匹配稳定性
%   4. 质量评分：综合距离、速度、共现帧数等因素
%
% 【输入】
%   trackSnapshots_R1 — [n_frames x 1] cell，R1 各帧航迹快照
%   trackSnapshots_R2 — [n_frames x 1] cell，R2 各帧航迹快照（已对齐）
%   params            — 参数结构体
%
% 【输出】
%   matched_pairs — struct 数组，每个元素：
%     .R1_track_id  — R1 航迹ID
%     .R2_track_id  — R2 航迹ID
%     .match_count  — 共现帧数
%     .coexist_count — 连续共现最长帧数
%     .match_ratio  — match_count / total_overlap
%     .mean_dist_km — 平均距离（km）
%     .mean_speed_diff — 平均速度差（m/s）
%     .mean_heading_diff — 平均航向差（度）
%     .quality      — 配对质量评分（0-100）
%
% 【调用关系】
%   被 run_simulation_multi.m Phase 7 调用
%   内部调用: sphere_utils_haversine_distance
% =========================================================================

function matched_pairs = track_matcher(trackSnapshots_R1, trackSnapshots_R2, params)
    n_frames = length(trackSnapshots_R1);
    method = 'simple';
    if isfield(params, 'track_matcher_method') && ~isempty(params.track_matcher_method)
        method = lower(params.track_matcher_method);
    end

    coexist_thresh = get_param_matcher(params, 'dualgate_coexist_thresh', 5);
    if strcmp(method, 'dualgate')
        dist_thresh_km = get_param_matcher(params, 'dualgate_T1_km', 35);
    else
        dist_thresh_km = get_param_matcher(params, 'track_matcher_dist_thresh_km', 50);
    end
    
    % 权重配置
    w_dist = 0.6;    % 距离权重
    w_speed = 0.25;  % 速度权重
    w_heading = 0.15; % 航向权重

    % ---- Step 1: 逐帧提取活跃航迹ID、位置、速度 ----
    r1_active = cell(n_frames, 1);
    r2_active = cell(n_frames, 1);

    for k = 1:n_frames
        r1_active{k} = extract_track_features(trackSnapshots_R1{k});
        r2_active{k} = extract_track_features(trackSnapshots_R2{k});
    end

    if strcmp(method, 'direct_single')
        matched_pairs = direct_single_match(r1_active, r2_active, n_frames);
        if ~isempty(matched_pairs)
            return;
        end
    end

    % ---- Step 2: 逐帧全局最优匹配 + 统计配对频次 ----
    % 使用配对计数器：key为"r1_id_r2_id"，value为匹配次数和累计距离等
    pair_stats = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    for k = 1:n_frames
        r1_tracks = r1_active{k};
        r2_tracks = r2_active{k};
        n_r1 = length(r1_tracks);
        n_r2 = length(r2_tracks);
        
        if n_r1 == 0 || n_r2 == 0, continue; end
        
        % 计算代价矩阵
        cost_matrix = zeros(n_r1, n_r2);
        for i = 1:n_r1
            for j = 1:n_r2
                cost_matrix(i, j) = compute_match_cost(r1_tracks{i}, r2_tracks{j}, ...
                    w_dist, w_speed, w_heading);
            end
        end
        
        % 全局最优分配（枚举法，适用于小规模问题）
        if n_r1 <= 4 && n_r2 <= 4
            assignment = optimal_assignment_enum(cost_matrix);
        else
            % 大规模用贪心近似
            assignment = greedy_assignment(cost_matrix);
        end
        
        % 统计配对结果
        for i = 1:length(assignment)
            j = assignment(i);
            if j == 0, continue; end % 未匹配
            
            r1_id = r1_tracks{i}.id;
            r2_id = r2_tracks{j}.id;
            
            % 检查距离门限
            dist_km = sphere_utils_haversine_distance(r1_tracks{i}.lon, r1_tracks{i}.lat, ...
                r2_tracks{j}.lon, r2_tracks{j}.lat) / 1000;
            if dist_km > dist_thresh_km, continue; end
            
            key = sprintf('%d_%d', r1_id, r2_id);
            if ~isKey(pair_stats, key)
                pair_stats(key) = struct(...
                    'r1_id', r1_id, ...
                    'r2_id', r2_id, ...
                    'count', 0, ...
                    'dist_sum', 0, ...
                    'speed_diff_sum', 0, ...
                    'heading_diff_sum', 0, ...
                    'frames', []);
            end
            
            s = pair_stats(key);
            s.count = s.count + 1;
            s.dist_sum = s.dist_sum + dist_km;
            s.speed_diff_sum = s.speed_diff_sum + ...
                compute_speed_diff(r1_tracks{i}, r2_tracks{j});
            s.heading_diff_sum = s.heading_diff_sum + ...
                compute_heading_diff(r1_tracks{i}, r2_tracks{j});
            s.frames = [s.frames, k];
            pair_stats(key) = s;
        end
    end

    % ---- Step 3: 构建 matched_pairs 输出 ----
    keys = pair_stats.keys();
    n_candidates = length(keys);
    
    candidates = struct('R1_track_id', {}, 'R2_track_id', {}, ...
        'match_count', {}, 'coexist_count', {}, 'match_ratio', {}, ...
        'mean_dist_km', {}, 'mean_speed_diff', {}, 'mean_heading_diff', {}, ...
        'quality', {});
    
    idx = 1;
    for k = 1:n_candidates
        s = pair_stats(keys{k});
        
        if s.count < coexist_thresh, continue; end
        
        mean_dist = s.dist_sum / s.count;
        mean_speed_diff = s.speed_diff_sum / s.count;
        mean_heading_diff = s.heading_diff_sum / s.count;
        
        % 计算连续共现最长帧数
        frames = s.frames;
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
        
        % 质量评分（0-100）
        % 距离评分：距离越近越好，50km时0分，0km时100分
        dist_score = max(0, 100 - mean_dist * 2);
        % 速度评分：速度差越小越好，50m/s时0分，0时100分
        speed_score = max(0, 100 - mean_speed_diff * 2);
        % 航向评分：航向差越小越好，90度时0分，0度时100分
        heading_score = max(0, 100 - mean_heading_diff * 1.1);
        % 共现评分：共现越多越好
        coexist_score = min(100, max_coexist * 5);
        
        quality = 0.5 * dist_score + 0.2 * speed_score + 0.15 * heading_score + 0.15 * coexist_score;
        
        candidates(idx).R1_track_id = s.r1_id;
        candidates(idx).R2_track_id = s.r2_id;
        candidates(idx).match_count = s.count;
        candidates(idx).coexist_count = max_coexist;
        candidates(idx).match_ratio = s.count / n_frames;
        candidates(idx).mean_dist_km = mean_dist;
        candidates(idx).mean_speed_diff = mean_speed_diff;
        candidates(idx).mean_heading_diff = mean_heading_diff;
        candidates(idx).quality = quality;
        idx = idx + 1;
    end
    
    % ---- Step 4: 全局最优配对（确保一一对应）----
    % 从候选配对中选择最优的一一对应集合
    matched_pairs = select_optimal_pairs(candidates);
    
    % 如果没有找到有效配对，用启发式方法兜底
    if isempty(matched_pairs)
        matched_pairs = heuristic_match(r1_active, r2_active, n_frames, coexist_thresh);
    end
end

% =========================================================================
% extract_track_features — 从单帧快照提取航迹特征（位置+速度+航向）
% =========================================================================
function tracks = extract_track_features(snap)
    tracks = {};
    if isempty(snap.trackList), return; end
    for t = 1:length(snap.trackList)
        trk = snap.trackList{t};
        if trk.type ~= 7 && ~isnan(trk.lat) && ~isempty(trk.ukf) && isfield(trk.ukf, 'x') && ~isempty(trk.ukf.x)
            x = trk.ukf.x;
            % 状态顺序：[lon; v_lon; lat; v_lat]
            lon = x(1);
            v_lon = x(2);
            lat = x(3);
            v_lat = x(4);
            
            % 计算速度大小（度/秒 -> 转换为 m/s 近似）
            % 1度纬度 ≈ 111km，1度经度 ≈ 111km * cos(lat)
            lat_rad = lat * pi / 180;
            v_lon_ms = v_lon * 111000 * cos(lat_rad); % 度/秒 -> 米/秒
            v_lat_ms = v_lat * 111000; % 度/秒 -> 米/秒
            speed = sqrt(v_lon_ms^2 + v_lat_ms^2);
            
            % 计算航向（度，正北为0，顺时针）
            heading = atan2(v_lon_ms, v_lat_ms) * 180 / pi;
            if heading < 0, heading = heading + 360; end
            
            tracks{end+1} = struct(...
                'id', trk.id, ...
                'lat', lat, ...
                'lon', lon, ...
                'v_lon', v_lon, ...
                'v_lat', v_lat, ...
                'speed_ms', speed, ...
                'heading_deg', heading);
        end
    end
end

% =========================================================================
% compute_match_cost — 计算两个航迹的匹配代价（越小越相似）
% =========================================================================
function cost = compute_match_cost(trk1, trk2, w_dist, w_speed, w_heading)
    % 距离代价（归一化到0-100）
    dist_km = sphere_utils_haversine_distance(trk1.lon, trk1.lat, trk2.lon, trk2.lat) / 1000;
    dist_cost = min(100, dist_km * 2); % 50km时100分
    
    % 速度代价（归一化到0-100）
    speed_diff = abs(trk1.speed_ms - trk2.speed_ms);
    speed_cost = min(100, speed_diff * 2); % 50m/s时100分
    
    % 航向代价（归一化到0-100）
    heading_diff = abs(trk1.heading_deg - trk2.heading_deg);
    if heading_diff > 180, heading_diff = 360 - heading_diff; end
    heading_cost = min(100, heading_diff * 1.1); % 约90度时100分
    
    % 综合代价
    cost = w_dist * dist_cost + w_speed * speed_cost + w_heading * heading_cost;
end

% =========================================================================
% compute_speed_diff — 计算两个航迹的速度差（m/s）
% =========================================================================
function diff = compute_speed_diff(trk1, trk2)
    diff = abs(trk1.speed_ms - trk2.speed_ms);
end

% =========================================================================
% compute_heading_diff — 计算两个航迹的航向差（度，0-180）
% =========================================================================
function diff = compute_heading_diff(trk1, trk2)
    diff = abs(trk1.heading_deg - trk2.heading_deg);
    if diff > 180, diff = 360 - diff; end
end

% =========================================================================
% optimal_assignment_enum — 枚举法求最优分配（适用于小规模）
% =========================================================================
function assignment = optimal_assignment_enum(cost_matrix)
    [n_row, n_col] = size(cost_matrix);
    
    % 确保行数 <= 列数
    if n_row > n_col
        cost_matrix = cost_matrix';
        transposed = true;
        [n_row, n_col] = size(cost_matrix);
    else
        transposed = false;
    end
    
    % 生成所有可能的排列
    cols = 1:n_col;
    perms_list = perms(cols);
    n_perms = size(perms_list, 1);
    
    best_cost = inf;
    best_assignment = zeros(1, n_row);
    
    for p = 1:n_perms
        perm = perms_list(p, 1:n_row);
        total_cost = 0;
        for i = 1:n_row
            total_cost = total_cost + cost_matrix(i, perm(i));
        end
        if total_cost < best_cost
            best_cost = total_cost;
            best_assignment = perm;
        end
    end
    
    if transposed
        % 转置回来
        assignment = zeros(1, n_col);
        for i = 1:n_row
            assignment(best_assignment(i)) = i;
        end
    else
        assignment = best_assignment;
    end
end

% =========================================================================
% greedy_assignment — 贪心分配（近似解，适用于大规模）
% =========================================================================
function assignment = greedy_assignment(cost_matrix)
    [n_row, n_col] = size(cost_matrix);
    assignment = zeros(1, n_row);
    used_col = false(1, n_col);
    
    % 按代价从小到大排序所有元素
    costs = [];
    rows = [];
    cols = [];
    for i = 1:n_row
        for j = 1:n_col
            costs(end+1) = cost_matrix(i, j);
            rows(end+1) = i;
            cols(end+1) = j;
        end
    end
    
    [~, order] = sort(costs);
    
    for k = 1:length(order)
        i = rows(order(k));
        j = cols(order(k));
        if assignment(i) == 0 && ~used_col(j)
            assignment(i) = j;
            used_col(j) = true;
        end
    end
end

% =========================================================================
% select_optimal_pairs — 从候选配对中选择最优的一一对应集合
% =========================================================================
function selected = select_optimal_pairs(candidates)
    if isempty(candidates)
        selected = struct();
        return;
    end
    
    n = length(candidates);
    
    % 按质量排序
    qualities = [candidates.quality];
    [~, order] = sort(qualities, 'descend');
    
    % 贪心选择：每次选质量最高的，然后排除已用的ID
    used_r1 = [];
    used_r2 = [];
    selected = struct('R1_track_id', {}, 'R2_track_id', {}, ...
        'match_count', {}, 'coexist_count', {}, 'match_ratio', {}, ...
        'mean_dist_km', {}, 'mean_speed_diff', {}, 'mean_heading_diff', {}, ...
        'quality', {});
    
    idx = 1;
    for k = 1:n
        c = candidates(order(k));
        if ~ismember(c.R1_track_id, used_r1) && ~ismember(c.R2_track_id, used_r2)
            selected(idx) = c;
            used_r1(end+1) = c.R1_track_id;
            used_r2(end+1) = c.R2_track_id;
            idx = idx + 1;
        end
    end
end

% =========================================================================
% direct_single_match — 单目标场景直接退化为一对一匹配
% =========================================================================
function pairs = direct_single_match(r1_active, r2_active, n_frames)
    r1_ids = collect_active_ids(r1_active);
    r2_ids = collect_active_ids(r2_active);
    pairs = struct('R1_track_id', {}, 'R2_track_id', {}, ...
        'match_count', {}, 'coexist_count', {}, 'match_ratio', {}, ...
        'mean_dist_km', {}, 'mean_speed_diff', {}, 'mean_heading_diff', {}, ...
        'quality', {});
    if length(r1_ids) ~= 1 || length(r2_ids) ~= 1
        return;
    end
    pairs(1).R1_track_id = r1_ids(1);
    pairs(1).R2_track_id = r2_ids(1);
    pairs(1).match_count = count_common_active_frames(r1_active, r2_active, r1_ids(1), r2_ids(1));
    pairs(1).coexist_count = pairs(1).match_count;
    pairs(1).match_ratio = pairs(1).match_count / max(n_frames, 1);
    pairs(1).mean_dist_km = 0;
    pairs(1).mean_speed_diff = 0;
    pairs(1).mean_heading_diff = 0;
    pairs(1).quality = 100;
end


function ids = collect_active_ids(active)
    ids = [];
    for k = 1:length(active)
        for t = 1:length(active{k})
            ids(end+1) = active{k}{t}.id;
        end
    end
    ids = unique(ids);
end


function n = count_common_active_frames(r1_active, r2_active, r1_id, r2_id)
    n = 0;
    for k = 1:length(r1_active)
        has_r1 = false;
        has_r2 = false;
        for i = 1:length(r1_active{k})
            if r1_active{k}{i}.id == r1_id
                has_r1 = true;
                break;
            end
        end
        for j = 1:length(r2_active{k})
            if r2_active{k}{j}.id == r2_id
                has_r2 = true;
                break;
            end
        end
        if has_r1 && has_r2
            n = n + 1;
        end
    end
end


function value = get_param_matcher(params, name, default_value)
    value = default_value;
    if isfield(params, name)
        value = params.(name);
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
            if isKey(r1_ids, id), r1_ids(id) = [r1_ids(id), k];
            else r1_ids(id) = k; end
        end
        for t = 1:length(r2_active{k})
            id = r2_active{k}{t}.id;
            if isKey(r2_ids, id), r2_ids(id) = [r2_ids(id), k];
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
        'mean_dist_km', {}, 'mean_speed_diff', {}, 'mean_heading_diff', {}, ...
        'quality', {});
    for p = 1:length(best_pairs)
        bp = best_pairs{p};
        pairs(p).R1_track_id = bp.r1_id;
        pairs(p).R2_track_id = bp.r2_id;
        pairs(p).match_count = bp.overlap;
        pairs(p).coexist_count = bp.overlap;
        pairs(p).match_ratio = bp.overlap / n_frames;
        pairs(p).mean_dist_km = 0;
        pairs(p).mean_speed_diff = 0;
        pairs(p).mean_heading_diff = 0;
        pairs(p).quality = 80;
    end
end
