function [result] = tracklet_grouping(input_R1, input_R2, params)
% TRACKLET_GROUPING 动态遍历双站片段关系并归并为候选 group
%
% 这是片段关联的核心入口函数，负责:
%   1. 从输入（快照序列或片段数组）中提取航迹片段
%   2. 构建片段间的边（edge）：cross-radar overlap 边 或 intra-radar successor 边
%   3. 使用带约束的并查集将片段归并为 group
%
% 输入:
%   input_R1   — R1 的快照序列或已提取的片段数组
%   input_R2   — R2 的快照序列或已提取的片段数组
%   params     — 含双门限参数、片段分组参数
%
% 输出:
%   result     — struct，含 segments, edges, groups, fused_snapshots 等

    % 如果输入是字符串（如 'segments'），直接从 input_R2 获取片段
    if isstring(input_R1), input_R1 = char(input_R1); end
    if ischar(input_R1) && strcmpi(input_R1, 'segments')
        segments = input_R2;
    else
        % 否则从 R1 和 R2 的快照序列中分别提取片段，然后拼接
        segments = [build_faded_track_segments('extract', input_R1, [], 1), ...
            build_faded_track_segments('extract', input_R2, [], 2)];
    end

    % 如果没有片段，返回空结果
    if isempty(segments)
        result = empty_result(); return;
    end

    % 构建片段之间的关系边（cross-radar 和 intra-radar）
    [edges, diagnostics] = build_edges(segments, params);

    % 使用带约束的并查集将片段归并为 group
    groups = constrained_components(segments, edges, params);

    % 组装最终结果
    result = struct('segments', segments, 'edges', edges, 'groups', groups, ...
        'candidate_diagnostics', diagnostics, 'fused_snapshots', {{}}, ...
        'fusion_results', struct([]));
end

function result = empty_result()
% empty_result — 创建空的分组结果结构体
% 用于输入为空时的快速返回
result = struct('segments', struct([]), 'edges', struct([]), 'groups', struct([]), ...
    'candidate_diagnostics', struct([]), 'fused_snapshots', {{}}, 'fusion_results', struct([]));
end

function [edges, diagnostics] = build_edges(segments, params)
% build_edges — 构建片段间的关系边
%
% 遍历所有片段对 (i, j)，根据雷达 ID 是否相同选择:
%   - 不同雷达: cross_edge — 基于重叠区距离的双门限判定
%   - 同雷达: successor_edge — 基于时间先后和 Mahalanobis 距离的判定
%
% 输入:
%   segments — 片段数组
%   params   — 含双门限和 successor 门限参数
% 输出:
%   edges    — 通过的边数组
%   diagnostics — 所有边的判定诊断信息
edges = struct('a', {}, 'b', {}, 'edge_type', {}, 'score', {}, 'distance_km', {}, ...
    'distance_variance_km2', {}, 'coexist_frames', {}, 'gap_frames', {}, 'mahalanobis_d2', {}, 'reason', {});
diagnostics = struct('a', {}, 'b', {}, 'edge_type', {}, 'accepted', {}, 'reason', {});

% 遍历所有片段对 (i, j)，i < j
for i = 1:numel(segments)-1
    for j = i+1:numel(segments)
        % 如果两个片段来自不同雷达 → cross-edge（重叠判定）
        if segments(i).radar_id ~= segments(j).radar_id
            [ok, edge] = cross_edge(segments(i), segments(j), params);
            kind = 'overlap';
        else
            % 如果来自同一雷达 → successor-edge（前后衔接判定）
            [ok, edge] = successor_edge(segments(i), segments(j), params);
            kind = 'successor';
        end

        % 记录诊断信息（无论是否通过）
        diagnostics(end+1) = struct('a', i, 'b', j, 'edge_type', kind, ...
            'accepted', ok, 'reason', edge.reason); %#ok<AGROW>

        % 只有通过判定才加入边列表
        if ok
            edge.a = i; edge.b = j; edge.edge_type = kind;
            edges(end+1) = edge; %#ok<AGROW>
        end
    end
end
end

function [ok, edge] = cross_edge(a, b, params)
% cross_edge — 跨雷达片段的重叠判定
%
% 算法:
%   1. 计算两个片段的 effective_frames 交集（共现帧）
%   2. 对每对共现帧，计算两个片段对应航迹位置的 Haversine 距离
%   3. 对距离序列执行双门限判定
%   4. 计算综合评分（距离 + 方差）
%   5. 判定条件：双门限通过 AND 评分 >= 阈值
%
% 输入:
%   a, b — 两个来自不同雷达的片段
%   params — 含双门限和评分阈值参数
% 输出:
%   ok   — 是否通过判定
%   edge — 边结构体（含距离、评分、原因等）

    % effective区间含内部偶发漏检，末端tail已在提取阶段排除。
    % 计算两个片段的有效帧交集（共现帧）
    [common, ia, ib] = intersect(a.effective_frames, b.effective_frames);

    % 在 a 的 raw_frames 中找到 effective_frames 各元素的索引
    raw_a = arrayfun(@(frame) find(a.raw_frames == frame, 1), a.effective_frames);
    % 在 b 的 raw_frames 中找到 effective_frames 各元素的索引
    raw_b = arrayfun(@(frame) find(b.raw_frames == frame, 1), b.effective_frames);

    % 对每对共现帧，计算两个片段航迹位置的 Haversine 距离（km）
    % 使用 raw_a[ia[q]] 和 raw_b[ib[q]] 找到共现帧在原始帧序列中的索引
    d = arrayfun(@(q) sphere_utils_haversine_distance(...
        a.lons(raw_a(ia(q))), a.lats(raw_a(ia(q))), ...
        b.lons(raw_b(ib(q))), b.lats(raw_b(ib(q)))) / 1000, 1:numel(common));

    % 对距离序列执行双门限判定
    decision = dual_threshold_decide(common, d, params);

    % 综合评分：距离越小、方差越小，评分越高
    % 使用指数衰减函数：score = exp(-mean_dist/T1) * exp(-var/var_thresh)
    score = exp(-decision.mean_distance_km / max(params.dualgate_T1_km, eps)) * ...
        exp(-decision.distance_variance_km2 / max(params.dualgate_var_km2, eps));

    % 最终判定：双门限通过 AND 评分 >= 最小阈值
    ok = decision.accepted && score >= params.tracklet_cross_score_min;

    % 如果双门限通过但评分不够，修改拒绝原因为 SCORE_GATE
    if ~ok && strcmp(decision.reason, 'ACCEPTED'), decision.reason = 'SCORE_GATE'; end

    % 组装边结构体
    edge = struct('a', 0, 'b', 0, 'edge_type', 'overlap', 'score', score, ...
        'distance_km', decision.mean_distance_km, 'distance_variance_km2', decision.distance_variance_km2, ...
        'coexist_frames', decision.coexist_frames, 'gap_frames', 0, 'mahalanobis_d2', NaN, ...
        'reason', decision.reason);
end

function [ok, edge] = successor_edge(a, b, params)
% successor_edge — 同雷达片段的前后衔接判定
%
% 算法:
%   1. 确定时间先后关系（a 在前还是 b 在前）
%   2. 检查 gap 是否超过最大允许间隔
%   3. 计算前后片段末端的 Mahalanobis 距离
%   4. 用 CV 模型预测前一片段末端状态，与后一片段起始状态比较
%   5. 综合距离和 Mahalanobis 距离计算评分
%
% 输入:
%   a, b — 两个来自同一雷达的片段
%   params — 含 successor 门限参数
% 输出:
%   ok   — 是否通过判定
%   edge — 边结构体

    % 检查两个片段是否有时间先后关系
    if a.end_frame < b.start_frame
        earlier = a; later = b;
    elseif b.end_frame < a.start_frame
        earlier = b; later = a;
    else
        % 时间重叠，不能作为 successor 关系
        ok = false; edge = rejection_edge('NO_TIME_ORDER'); return;
    end

    % 计算 gap（两个片段之间的空闲帧数）
    gap = later.start_frame - earlier.end_frame - 1;

    % 如果 gap 超过最大允许间隔，拒绝
    if gap > params.tracklet_successor_max_gap_frames
        ok = false; edge = rejection_edge('GAP_GATE'); edge.gap_frames = gap; return;
    end

    % 找到 earlier 片段的最后一个 support 帧在 raw_frames 中的索引
    ia = find(earlier.raw_frames == earlier.last_support_frame, 1);
    % 找到 later 片段的第一个 support 帧在 raw_frames 中的索引
    ib = find(later.raw_frames == later.first_support_frame, 1);

    % 计算两个片段之间的时间差（秒）
    dt = (later.first_support_frame - earlier.last_support_frame) * params.dt_sec;

    % 用 CV（恒速）模型从 earlier 末端预测到 later 起始位置
    % propagate_cv 使用 F 矩阵和 Q 噪声进行状态传播
    [xp, Pp] = propagate_cv(earlier.states(:, ia), earlier.covariances(:, :, ia), dt, params);

    % 计算预测位置与 later 起始位置之间的 Haversine 距离（km）
    distance = sphere_utils_haversine_distance(xp(1), xp(3), later.lons(ib), later.lats(ib)) / 1000;

    % 合并预测协方差和 later 起始协方差，正则化后作为联合协方差
    S = regularize_cov(Pp + later.covariances(:, :, ib));

    % 计算状态差向量
    delta = later.states(:, ib) - xp;

    % 计算 Mahalanobis 距离
    mahal = delta' * (S \ delta);

    % 综合评分：距离越小、Mahalanobis 距离越小，评分越高
    score = exp(-distance / max(params.tracklet_successor_distance_km, eps)) * ...
        exp(-mahal / max(params.tracklet_successor_mahal_gate, eps));

    % 多级判定：距离门限 > Mahalanobis 门限 > 评分门限
    if distance >= params.tracklet_successor_distance_km
        ok = false; reason = 'DISTANCE_GATE';
    elseif mahal >= params.tracklet_successor_mahal_gate
        ok = false; reason = 'MAHALANOBIS_GATE';
    elseif score < params.tracklet_successor_score_min
        ok = false; reason = 'SCORE_GATE';
    else
        ok = true; reason = 'ACCEPTED';
    end

    % 组装边结构体
    edge = struct('a', 0, 'b', 0, 'edge_type', 'successor', 'score', score, ...
        'distance_km', distance, 'distance_variance_km2', NaN, 'coexist_frames', 0, ...
        'gap_frames', gap, 'mahalanobis_d2', mahal, 'reason', reason);
end

function edge = rejection_edge(reason)
% rejection_edge — 创建拒绝边的默认结构体
% 用于边判定失败时返回统一格式的拒绝标记
edge = struct('a', 0, 'b', 0, 'edge_type', 'successor', 'score', 0, 'distance_km', inf, ...
    'distance_variance_km2', inf, 'coexist_frames', 0, 'gap_frames', 0, ...
    'mahalanobis_d2', inf, 'reason', reason);
end

function groups = constrained_components(segments, edges, params)
% constrained_components — 使用带约束的并查集将片段归并为 group
%
% 算法:
%   1. 按边评分降序处理所有边
%   2. 对每条边，检查两端片段是否已连通（并查集 root 相同）
%   3. 如果不连通，检查合并是否满足约束条件（consistent）
%   4. 如果满足，执行 union 操作
%   5. 最后按连通分量提取 group
%
% 约束条件:
%   - 同一雷达的多个片段不能在 effective_frames 上有重叠
%   - 同一雷达的连续片段之间必须有 successor 边连接
%   - 跨雷达 group 至少需要一定数量的 overlap 边
%
% 输入:
%   segments — 片段数组
%   edges    — 边数组
%   params   — 含最小支持边数等参数
% 输出:
%   groups   — group 结构体数组
parent = 1:numel(segments);

% 按边评分降序排列，优先处理高质量边
[~, order] = sort([edges.score], 'descend');

% 贪心合并：按评分从高到低处理每条边
for q = order
    a = edges(q).a; b = edges(q).b;
    ra = root(parent, a); rb = root(parent, b);
    % 如果已经在同一集合中，跳过
    if ra == rb, continue; end

    % 找到 ra 和 rb 两个集合中的所有片段索引
    ma = find(arrayfun(@(x) root(parent, x) == ra, 1:numel(segments)));
    mb = find(arrayfun(@(x) root(parent, x) == rb, 1:numel(segments)));

    % 检查合并是否满足约束条件
    if consistent([ma, mb], segments, edges, params)
        % 合并两个集合：将 rb 的父节点设为 ra
        parent(rb) = ra;
    end
end

% 计算每个片段的根节点
roots = arrayfun(@(x) root(parent, x), 1:numel(segments));
ur = unique(roots);

% 初始化 group 结构体数组
groups = struct('group_id', {}, 'segment_indices', {}, 'support_edge_indices', {}, ...
    'start_frame', {}, 'end_frame', {}, 'is_isolated', {});

% 对每个连通分量，提取对应的片段和边
for g = 1:numel(ur)
    members = find(roots == ur(g));
    % 找出连接这些片段的所有边
    edge_idx = find(arrayfun(@(e) ismember(e.a, members) && ismember(e.b, members), edges));

    % 如果没有边且不允许孤立片段，跳过
    if isempty(edge_idx) && ~params.tracklet_group_allow_isolated, continue; end

    % 组装 group 信息
    groups(end+1) = struct('group_id', numel(groups)+1, 'segment_indices', members, ...
        'support_edge_indices', edge_idx, 'start_frame', min([segments(members).start_frame]), ...
        'end_frame', max([segments(members).end_frame]), 'is_isolated', isempty(edge_idx)); %#ok<AGROW>
end
end

function ok = consistent(members, segments, edges, params)
% consistent — 检查片段集合合并是否满足约束条件
%
% 约束1: 同一雷达的片段不能有 effective_frames 重叠
% 约束2: 同一雷达的连续片段之间必须有 successor 边
% 约束3: 跨雷达 group 至少需要一定数量的 overlap 边
ok = true;
for radar = 1:2
    % 找出该雷达在 members 中的所有片段索引
    idx = members([segments(members).radar_id] == radar);
    % 如果该雷达少于 2 个片段，无需检查
    if numel(idx) < 2, continue; end

    % 按 start_frame 排序
    [~, order] = sort([segments(idx).start_frame]);
    idx = idx(order);

    % 检查相邻片段之间是否满足约束
    for i = 1:numel(idx)-1
        current = idx(i); next = idx(i+1);

        % 约束1: 同一雷达的连续片段不能在 effective_frames 上有重叠
        if ~isempty(intersect(segments(current).effective_frames, segments(next).effective_frames))
            ok = false; return;
        end

        % 约束2: 必须有 successor 边连接
        has_successor = any(arrayfun(@(e) strcmp(e.edge_type, 'successor') && ...
            ((e.a == current && e.b == next) || (e.a == next && e.b == current)), edges));
        if ~has_successor, ok = false; return; end
    end
end

% 约束3: 如果配置了最小支持边数，检查跨雷达 overlap 边数量
if params.tracklet_group_min_support_edges > 0
    cross = sum(strcmp({edges.edge_type}, 'overlap') & arrayfun(@(e) ismember(e.a, members) && ismember(e.b, members), edges));
    if numel(unique([segments(members).radar_id])) > 1 && cross < params.tracklet_group_min_support_edges, ok = false; end
end
end

function [x_new, P_new] = propagate_cv(x, P, dt, params)
% propagate_cv — 用恒速（CV）模型预测状态
%
% 状态向量: [lon, lon_rate, lat, lat_rate]
% 状态转移矩阵 F:
%   [1 dt 0 0]
%   [0 1 0 0]
%   [0 0 1 dt]
%   [0 0 0 1]
%
% 过程噪声 Q = eye(4) * tracklet_prediction_q * max(|dt|, 1)
F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
Q = eye(4) * params.tracklet_prediction_q * max(abs(dt), 1);
% 状态传播: x_new = F * x
% 协方差传播: P_new = F * P * F' + Q（正则化后）
x_new = F*x; P_new = regularize_cov(F*P*F' + Q);
end

function r = root(parent, x)
% root — 并查集查找：找到节点 x 的根（路径压缩前的朴素实现）
r = x;
while parent(r) ~= r, r = parent(r); end
end
