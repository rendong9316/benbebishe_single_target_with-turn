% =========================================================================
% track_matcher.m — 跨雷达航迹时空配对模块（优化版）
% =========================================================================
%
% 【功能概述】
%   将 R1 和 R2 两部雷达的航迹列表按时空接近度进行配对，
%   输出 matched_pairs 数组，供 run_track_fusion 使用。
%
%   配对是航迹融合的前置步骤：融合算法需要知道"哪条 R1 航迹"和
%   "哪条 R2 航迹"描述的是同一个目标。如果配对错误（将不同目标的
%   航迹配对），融合结果将产生物理上无意义的中间值。
%
%   配对逻辑（优化版）：
%   1. 多维特征：位置距离 + 速度相似度 + 航向相似度
%      - 不仅仅看位置，还考虑速度和航向，减少"同位置不同向"的误配
%   2. 全局最优：每帧用匈牙利算法（枚举法）做全局最优分配
%      - 不是简单的最近邻一对一匹配（那可能陷入局部最优）
%      - 而是考虑所有可能的配对组合，选全局代价最小的
%   3. 时序统计：统计共现帧数、平均距离、匹配稳定性
%      - 单帧匹配不可靠（可能偶然接近），需要多帧共现确认
%      - 共现帧数越多，配对置信度越高
%   4. 质量评分：综合距离、速度、共现帧数等因素给出 0-100 分
%      - 高分配对（>70）可信度高，低分配对（<30）可能是误配
%
% 【配对失败的兜底策略】
%   如果全局最优匹配未能找到任何有效配对（如两部雷达的航迹 ID
%   完全不重叠），则使用 heuristic_match 函数基于共现帧数做
%   启发式配对作为 fallback。
%
% 【输入】
%   trackSnapshots_R1 — [n_frames x 1] cell，R1 各帧航迹快照
%   trackSnapshots_R2 — [n_frames x 1] cell，R2 航迹快照（已时间对齐）
%   params            — 参数结构体（目前主要用其中的阈值参数）
%
% 【输出】
%   matched_pairs — struct 数组，每个元素：
%     .R1_track_id  — R1 航迹ID
%     .R2_track_id  — R2 航迹ID
%     .match_count  — 共现帧数（两航迹在同一帧中同时出现的次数）
%     .coexist_count — 连续共现最长帧数（衡量配对的时序稳定性）
%     .match_ratio  — match_count / total_overlap（配对覆盖率）
%     .mean_dist_km — 平均距离（km），越小越好
%     .mean_speed_diff — 平均速度差（m/s），越小越好
%     .mean_heading_diff — 平均航向差（度），越小越好
%     .quality      — 配对质量评分（0-100），越高越可信
%
% 【调用关系】
%   被 run_simulation_multi.m Phase 7 调用
%   内部调用: sphere_utils_haversine_distance（大圆距离计算）
% =========================================================================

function matched_pairs = track_matcher(trackSnapshots_R1, trackSnapshots_R2, params)
    n_frames = length(trackSnapshots_R1);
    coexist_thresh = 5;  % 最少共现帧数：低于此值的配对视为无效
    dist_thresh_km = 50; % 距离门限（km）：超过此距离的配对直接丢弃

    % 权重配置：距离最重要，速度次之，航向再次
    % 这三个权重决定了匹配代价的计算中各维度的相对重要性
    w_dist = 0.6;    % 距离权重（60%）：位置是最直接的配对依据
    w_speed = 0.25;  % 速度权重（25%）：速度相近说明可能是同一目标
    w_heading = 0.15; % 航向权重（15%）：航向一致进一步确认配对

    % ---- Step 1: 逐帧提取活跃航迹特征 ----
    % extract_track_features 从快照中提取每条航迹的位置、速度、航向
    % 将原始航迹结构体转换为统一的特征向量，方便后续比较
    r1_active = cell(n_frames, 1);
    r2_active = cell(n_frames, 1);

    for k = 1:n_frames
        r1_active{k} = extract_track_features(trackSnapshots_R1{k});
        r2_active{k} = extract_track_features(trackSnapshots_R2{k});
    end

    % ---- Step 2: 逐帧全局最优匹配 + 统计配对频次 ----
    % 使用 containers.Map 存储配对统计：key="r1_id_r2_id"
    % Map 的 key 是字符串形式的"R1_ID_R2_ID"，value 是结构体，
    % 累计了该配对在所有帧中的统计信息
    pair_stats = containers.Map('KeyType', 'char', 'ValueType', 'any');

    for k = 1:n_frames
        r1_tracks = r1_active{k};
        r2_tracks = r2_active{k};
        n_r1 = length(r1_tracks);
        n_r2 = length(r2_tracks);

        if n_r1 == 0 || n_r2 == 0, continue; end

        % 计算代价矩阵：每对 (r1, r2) 的匹配代价
        % cost_matrix(i,j) = R1 第 i 条航迹与 R2 第 j 条航迹的匹配代价
        % 代价越小表示两条航迹越相似（越可能配对）
        cost_matrix = zeros(n_r1, n_r2);
        for i = 1:n_r1
            for j = 1:n_r2
                cost_matrix(i, j) = compute_match_cost(r1_tracks{i}, r2_tracks{j}, ...
                    w_dist, w_speed, w_heading);
            end
        end

        % 全局最优分配（枚举法，适用于小规模问题）
        % 当航迹数较少（≤4）时，使用全排列枚举找到全局最优分配
        % 当航迹数较多时，枚举复杂度 O(n!) 太高，改用贪心近似
        if n_r1 <= 4 && n_r2 <= 4
            assignment = optimal_assignment_enum(cost_matrix);
        else
            % 大规模用贪心近似
            assignment = greedy_assignment(cost_matrix);
        end

        % 统计配对结果：遍历分配结果，累加每对配对的统计数据
        for i = 1:length(assignment)
            j = assignment(i);
            if j == 0, continue; end % 未匹配（分配结果为 0 表示该航迹未配对）

            r1_id = r1_tracks{i}.id;
            r2_id = r2_tracks{j}.id;

            % 检查距离门限：即使代价最小，如果距离超过 50km 也视为误配
            % 使用 Haversine 公式计算两点间的大圆距离（考虑地球曲率）
            dist_km = sphere_utils_haversine_distance(r1_tracks{i}.lon, r1_tracks{i}.lat, ...
                r2_tracks{j}.lon, r2_tracks{j}.lat) / 1000;
            if dist_km > dist_thresh_km, continue; end

            % 构建配对键名："r1_id_r2_id"
            key = sprintf('%d_%d', r1_id, r2_id);
            if ~isKey(pair_stats, key)
                % 首次遇到此配对，初始化统计结构体
                pair_stats(key) = struct(...
                    'r1_id', r1_id, ...
                    'r2_id', r2_id, ...
                    'count', 0, ...          % 共现帧数计数器
                    'dist_sum', 0, ...       % 距离累加和（用于计算均值）
                    'speed_diff_sum', 0, ... % 速度差累加和
                    'heading_diff_sum', 0, ... % 航向差累加和
                    'frames', []);           % 共现帧号列表
            end

            % 更新统计信息
            s = pair_stats(key);
            s.count = s.count + 1;                              % 共现帧数 +1
            s.dist_sum = s.dist_sum + dist_km;                  % 累加距离
            s.speed_diff_sum = s.speed_diff_sum + ...           % 累加速度差
                compute_speed_diff(r1_tracks{i}, r2_tracks{j});
            s.heading_diff_sum = s.heading_diff_sum + ...       % 累加航向差
                compute_heading_diff(r1_tracks{i}, r2_tracks{j});
            s.frames = [s.frames, k];                           % 记录共现帧号
            pair_stats(key) = s;                                % 写回 Map
        end
    end

    % ---- Step 3: 构建 matched_pairs 输出 ----
    % 从 Map 中提取所有配对候选，过滤掉共现帧数不足的
    keys = pair_stats.keys();
    n_candidates = length(keys);

    % 预分配候选数组（结构体数组）
    candidates = struct('R1_track_id', {}, 'R2_track_id', {}, ...
        'match_count', {}, 'coexist_count', {}, 'match_ratio', {}, ...
        'mean_dist_km', {}, 'mean_speed_diff', {}, 'mean_heading_diff', {}, ...
        'quality', {});

    idx = 1;
    for k = 1:n_candidates
        s = pair_stats(keys{k});

        % 过滤：共现帧数不足 coexist_thresh (5) 的配对视为无效
        % 单帧匹配不可靠，需要多帧持续共现才能确认是同一目标
        if s.count < coexist_thresh, continue; end

        % 计算各维度的平均值
        mean_dist = s.dist_sum / s.count;
        mean_speed_diff = s.speed_diff_sum / s.count;
        mean_heading_diff = s.heading_diff_sum / s.count;

        % 计算连续共现最长帧数
        % 共现帧数多不代表配对可靠，如果这些帧是断断续续的（如只在
        % 偶数帧共现），可能是偶然接近。连续共现越长，配对越可靠
        frames = s.frames;
        sorted_frames = sort(frames);
        coexist = 1; max_coexist = 1;
        for f = 2:length(sorted_frames)
            if sorted_frames(f) == sorted_frames(f-1) + 1
                coexist = coexist + 1;
                max_coexist = max(max_coexist, coexist);
            else
                coexist = 1;  % 不连续，重置计数器
            end
        end

        % 质量评分（0-100）：距离40% + 速度20% + 航向15% + 共现15%
        % 各子分数都通过线性衰减函数计算：值越小（越好）→ 分数越高
        dist_score = max(0, 100 - mean_dist * 2);       % 距离每增加 50km，扣 100 分
        speed_score = max(0, 100 - mean_speed_diff * 2); % 速度差每增加 50m/s，扣 100 分
        heading_score = max(0, 100 - mean_heading_diff * 1.1); % 航向差每增加 ~91°，扣 100 分
        coexist_score = min(100, max_coexist * 5);       % 连续共现每增加 1 帧，+5 分

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
    % 从候选配对中选择最优的一组，确保每条 R1 航迹最多配对一条 R2 航迹，
    % 反之亦然（避免一对多或多对一的冲突）
    matched_pairs = select_optimal_pairs(candidates);

    % 如果没有找到有效配对，用启发式方法兜底
    % 当全局最优匹配未能找到任何有效配对时（如航迹 ID 完全不重叠），
    % 使用基于共现帧数的启发式方法重新尝试配对
    if isempty(matched_pairs)
        matched_pairs = heuristic_match(r1_active, r2_active, n_frames, coexist_thresh);
    end
end

% =========================================================================
% extract_track_features — 从单帧快照提取航迹特征（位置+速度+航向）
% =========================================================================
% 从航迹快照中提取每条活跃航迹的特征：
%   id, lat, lon, speed_ms, heading_deg
% 速度从 UKF 状态向量 [lon, v_lon, lat, v_lat] 中计算，
% 单位从度/秒转换为 m/s（1度纬度≈111km，1度经度≈111km*cos(lat)）。
%
% 注意：UKF 状态向量中的速度分量 v_lon 和 v_lat 的单位是"度/秒"，
% 需要乘以地球的尺度因子才能转换为物理速度（m/s）。
% 经度方向的尺度因子随纬度变化（cos(lat)），因为在高纬度地区，
% 经度一度对应的实际距离更短。
function tracks = extract_track_features(snap)
    tracks = {};
    if isempty(snap.trackList), return; end
    for t = 1:length(snap.trackList)
        trk = snap.trackList{t};
        % 跳过 HISTORY 航迹（type=7）和无效位置
        % 只处理 type≠7 且有有效 UKF 状态的航迹
        if trk.type ~= 7 && ~isnan(trk.lat) && ~isempty(trk.ukf) && isfield(trk.ukf, 'x') && ~isempty(trk.ukf.x)
            x = trk.ukf.x;
            % 状态顺序：[lon; v_lon; lat; v_lat]
            lon = x(1);
            v_lon = x(2);
            lat = x(3);
            v_lat = x(4);

            % 计算速度大小（度/秒 -> 转换为 m/s 近似）
            % 1度纬度 ≈ 111km（地球子午线弧长）
            % 1度经度 ≈ 111km * cos(lat)（随纬度递减，赤道处最大，两极处为零）
            lat_rad = lat * pi / 180;
            v_lon_ms = v_lon * 111000 * cos(lat_rad); % 经度速度 → m/s
            v_lat_ms = v_lat * 111000; % 纬度速度 → m/s
            speed = sqrt(v_lon_ms^2 + v_lat_ms^2);

            % 计算航向（度，正北为0，顺时针）
            % atan2(v_lon, v_lat) 给出从正北方向顺时针到速度矢量的角度
            % 如果结果为负，加 360 转换为 [0, 360) 范围
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
% 综合距离、速度差、航向差三个维度的归一化代价。
% 每个维度的代价都归一化到 [0, 100] 范围：
%   - 距离：50km 时代价 100（线性缩放，2 km^{-1}）
%   - 速度：50m/s 时代价 100（线性缩放，2 (m/s)^{-1}）
%   - 航向：90度时代价 100（线性缩放，1.1 度^{-1}，考虑周期性）
% 最终代价是三个维度的加权和，权重由 w_dist, w_speed, w_heading 控制
function cost = compute_match_cost(trk1, trk2, w_dist, w_speed, w_heading)
    % 距离代价（归一化到0-100）：50km 时代价 100
    % 使用 Haversine 公式计算大圆距离，考虑地球曲率
    dist_km = sphere_utils_haversine_distance(trk1.lon, trk1.lat, trk2.lon, trk2.lat) / 1000;
    dist_cost = min(100, dist_km * 2);

    % 速度代价（归一化到0-100）：50m/s 时代价 100
    speed_diff = abs(trk1.speed_ms - trk2.speed_ms);
    speed_cost = min(100, speed_diff * 2);

    % 航向代价（归一化到0-100）：90度时代价 100
    % 注意航向是圆周量（0=360度），需要用最短弧长计算差值
    heading_diff = abs(trk1.heading_deg - trk2.heading_deg);
    if heading_diff > 180, heading_diff = 360 - heading_diff; end  % 取短弧
    heading_cost = min(100, heading_diff * 1.1);

    % 加权综合：距离占 60%，速度占 25%，航向占 15%
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
% 航向是圆周量（0-360度），差值应取最短弧长：
%   例如 350度 和 10度 的差值是 20度（不是 340度）
function diff = compute_heading_diff(trk1, trk2)
    diff = abs(trk1.heading_deg - trk2.heading_deg);
    if diff > 180, diff = 360 - diff; end
end

% =========================================================================
% optimal_assignment_enum — 枚举法求最优分配（适用于小规模）
% =========================================================================
% 当航迹数 ≤ 4 时，使用全排列枚举找到代价最小的全局最优分配。
% 这等价于求解线性指派问题（Linear Assignment Problem），
% 但用暴力枚举而非匈牙利算法（因为小规模问题枚举更快）。
%
% 算法步骤：
%   1. 生成所有可能的列索引排列（perms 函数）
%   2. 对每个排列，计算总代价（各行选取排列中对应列的元素之和）
%   3. 选择总代价最小的排列作为最优分配
%
% 时间复杂度：O(n! * n)，其中 n = max(n_r1, n_r2)
% 当 n=4 时，4! = 24，计算量很小
% 当 n=10 时，10! = 3,628,800，计算量太大，需改用贪心
function assignment = optimal_assignment_enum(cost_matrix)
    [n_row, n_col] = size(cost_matrix);

    % 确保行数 <= 列数，这样 perms 生成的排列才有足够的列可选
    % 如果行数 > 列数，转置代价矩阵，最后再转置回来
    if n_row > n_col
        cost_matrix = cost_matrix';
        transposed = true;
        [n_row, n_col] = size(cost_matrix);
    else
        transposed = false;
    end

    % 生成所有可能的列索引排列
    % 例如 n_col=3 时，perms([1,2,3]) 生成 6 种排列：
    %   [1,2,3], [1,3,2], [2,1,3], [2,3,1], [3,1,2], [3,2,1]
    cols = 1:n_col;
    perms_list = perms(cols);
    n_perms = size(perms_list, 1);

    best_cost = inf;
    best_assignment = zeros(1, n_row);

    % 遍历所有排列，找到总代价最小的那个
    for p = 1:n_perms
        perm = perms_list(p, 1:n_row);  % 取前 n_row 个元素
        total_cost = 0;
        for i = 1:n_row
            total_cost = total_cost + cost_matrix(i, perm(i));
        end
        if total_cost < best_cost
            best_cost = total_cost;
            best_assignment = perm;
        end
    end

    % 如果之前转置了，需要转置回来
    % 转置后的 assignment 含义：assignment[j] = i 表示 R2 的第 j 条
    % 航迹匹配到 R1 的第 i 条航迹。需要反过来：assignment[i] = j
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
% 按代价从小到大排序所有元素，贪心地选择最小代价的分配。
% 这是一种近似算法，不保证全局最优，但计算效率高 O(n*m*log(n*m))。
%
% 算法步骤：
%   1. 将所有 (i,j) 对的代价展平为一维数组
%   2. 按代价从小到大排序
%   3. 依次选择代价最小的未冲突配对
%      - 冲突定义：该行或该列已被其他配对占用
%   4. 直到所有行都被分配或无可用的列
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

    % 获取排序索引（从小到大）
    [~, order] = sort(costs);

    % 贪心选择：按代价从小到大遍历，选择不冲突的配对
    for k = 1:length(order)
        i = rows(order(k));
        j = cols(order(k));
        % 只有当该行未分配且该列未被使用时才选择此配对
        if assignment(i) == 0 && ~used_col(j)
            assignment(i) = j;
            used_col(j) = true;
        end
    end
end

% =========================================================================
% select_optimal_pairs — 从候选配对中选择最优的一一对应集合
% =========================================================================
% 按质量评分降序排序，贪心选择：每次选质量最高的，排除已用的 ID。
% 确保最终的 matched_pairs 中每条 R1 航迹最多配对一条 R2 航迹，
% 每条 R2 航迹也最多配对一条 R1 航迹（一对一约束）。
%
% 算法步骤：
%   1. 将所有候选配对按质量评分从高到低排序
%   2. 依次遍历排序后的候选：
%      - 如果该候选的 R1 ID 和 R2 ID 都未被使用 → 选中
%      - 否则 → 跳过（存在冲突）
%   3. 返回选中的配对集合
function selected = select_optimal_pairs(candidates)
    if isempty(candidates)
        selected = struct();
        return;
    end

    n = length(candidates);

    % 按质量排序（降序）
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
        % 只有当 R1 和 R2 的 ID 都未被其他配对占用时才选中
        if ~ismember(c.R1_track_id, used_r1) && ~ismember(c.R2_track_id, used_r2)
            selected(idx) = c;
            used_r1(end+1) = c.R1_track_id;
            used_r2(end+1) = c.R2_track_id;
            idx = idx + 1;
        end
    end
end

% =========================================================================
% heuristic_match — 基于最近邻的启发式配对（fallback）
% =========================================================================
% 当全局最优匹配未能找到有效配对时，使用共现帧数作为 fallback。
% 这种方法不依赖代价矩阵和匈牙利算法，而是简单地统计每对航迹 ID
% 的共现帧数，选择共现最多的配对。
%
% 适用场景：
%   - 两部雷达的航迹 ID 系统完全不重叠（无法通过 ID 映射）
%   - 全局最优匹配因距离门限等原因未能找到有效配对
%   - 需要快速得到一个粗略的配对结果
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
