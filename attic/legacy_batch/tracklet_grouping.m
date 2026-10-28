function result = tracklet_grouping(input_R1, input_R2, params)
% TRACKLET_GROUPING 基于图论的航迹片段分组与凝聚。
%
% 【问题描述】
%   在低检测概率（Pd=0.6）场景下，同一目标在两雷达上可能被切分为多个
%   独立航迹片段（segment）。这些片段在时空上连续，本应属于同一目标。
%   本函数的目标是将这些片段"聚合"成组（group），每组对应一个真实目标。
%
% 【算法流程】
%   1. 提取所有片段（support/tail/effective 标注）
%   2. 构建片段间边（edge）：
%      - 同站续接边（successor）：时间先后、空间连续 → 同一雷达的两个片段
%      - 跨站重叠边（overlap）：两雷达片段在时间和空间上有重叠
%      - 跨站交接边（handoff）：时间不重叠但满足交接条件
%   3. 枚举所有连通子集作为假设（hypothesis）
%   4. 用整数规划求解最优覆盖：每个片段必须恰好属于一个组
%   5. 计算最优解与次优解的间隙，判断是否存在歧义
%
% 【输入参数】
%   input_R1 — 字符串 'segments' 或 R1 快照序列
%   input_R2 — 如果 input_R1='segments' 则为片段数组；否则为 R2 快照序列
%   params   — 仿真参数（含 tracklet_* 前缀的分组参数）
%
% 【输出】
%   result.status    — 'SUCCESS' / 'GROUP_HYPOTHESIS_LIMIT_EXCEEDED' / 等
%   result.segments  — 所有片段数组
%   result.edges     — 所有构建的边
%   result.groups    — 最终分组结果
%   result.solver    — 整数规划求解器输出

if isstring(input_R1), input_R1 = char(input_R1); end  % 兼容 string 类型输入
if ischar(input_R1) && strcmpi(input_R1, 'segments')   % 如果第一个参数是'segments'字符串
    segments = input_R2;                                % 说明输入已经是片段数组
else
    % 从 R1 和 R2 快照中提取片段
    segments = [build_faded_track_segments('extract', input_R1, [], 1), ...
        build_faded_track_segments('extract', input_R2, [], 2)];
end

assert_truth_free_segments(segments);  % 确保片段中没有泄露真值信息
if isempty(segments)                    % 没有片段，直接返回
    result = empty_result('SUCCESS');
    return;
end

[edges, diagnostics] = build_edges(segments, params);  % 构建片段间的边
[hypotheses, hypothesis_status] = enumerate_hypotheses(segments, edges, params);  % 枚举假设
if ~strcmp(hypothesis_status, 'SUCCESS')  % 假设枚举失败（超限）
    result = empty_result(hypothesis_status);
    result.segments = segments;
    result.edges = edges;
    result.candidate_diagnostics = diagnostics;
    result.hypotheses = hypotheses;
    return;
end

% 用整数规划求解最优覆盖：每个片段恰好属于一个组
[groups, solver] = solve_hypothesis_cover(segments, hypotheses, params);
result = struct('status', solver.status, 'segments', segments, 'edges', edges, ...
    'candidate_diagnostics', diagnostics, 'hypotheses', hypotheses, ...
    'groups', groups, 'solver', solver, 'fused_snapshots', {{}}, ...
    'fusion_results', struct([]));
end

function result = empty_result(status)
% empty_result 创建空结果结构体（用于早期返回）
result = struct('status', status, 'segments', struct([]), 'edges', struct([]), ...
    'candidate_diagnostics', struct([]), 'hypotheses', struct([]), ...
    'groups', struct([]), 'solver', struct('status', status), ...
    'fused_snapshots', {{}}, 'fusion_results', struct([]));
end

function assert_truth_free_segments(segments)
% assert_truth_free_segments 确保片段结构体中没有泄露真值信息的字段。
% 分组算法应该是"盲"的——不能利用 truth_idx 来做决策。
forbidden = {'truth_idx', 'aircraft_id', 'target_id', 'expected_target_count'};
for i = 1:numel(forbidden)
    if isfield(segments, forbidden{i})
        error('tracklet_grouping:truthLeak', ...
            'Production segment field %s is forbidden', forbidden{i});
    end
end
end

function [edges, diagnostics] = build_edges(segments, params)
% build_edges 构建所有片段对之间的边（edge）。
%
% 边的类型：
%   successor — 同雷达站、时间先后连续的片段之间的续接边
%   overlap   — 不同雷达站、时空重叠的片段之间的重叠边
%   handoff   — 不同雷达站、时间不重叠但满足交接条件的交接边
%
% 算法：O(N^2) 遍历所有片段对，尝试构建三种类型的边

edges = empty_edges();             % 初始化边列表
diagnostics = empty_diagnostics(); % 初始化诊断信息
for i = 1:numel(segments)-1       % 遍历所有片段对 (i, j) with j > i
    for j = i+1:numel(segments)
        if segments(i).radar_id == segments(j).radar_id  % 同雷达站 → 尝试续接边
            [ok, edge] = temporal_edge(segments(i), segments(j), params, 'successor');
        else
            % 不同雷达站 → 先尝试重叠边
            [ok, edge] = overlap_edge(segments(i), segments(j), params);
            % 如果重叠边不通过且原因是"共现帧不足"，再尝试交接边
            if ~ok && strcmp(edge.reason, 'INSUFFICIENT_COEXISTENCE')
                [ok, edge] = temporal_edge(segments(i), segments(j), params, 'handoff');
            end
        end
        edge.a = i;              % 记录边的两端片段索引
        edge.b = j;
        diagnostics(end+1) = diagnostic_from_edge(edge, ok); %#ok<AGROW>  % 保存诊断信息
        if ok, edges(end+1) = edge; end %#ok<AGROW>  % 只保留通过的边
    end
end
end

function [ok, edge] = overlap_edge(a, b, params)
% overlap_edge 评估两个不同雷达站片段之间的重叠关系。
% 共现帧越多、距离越小，得分越高。
[common, ia, ib] = intersect(a.effective_frames, b.effective_frames);  % 找出共现帧
distances = zeros(1, numel(common));
for q = 1:numel(common)
    raw_a = find(a.raw_frames == a.effective_frames(ia(q)), 1);  % a 中共现帧的索引
    raw_b = find(b.raw_frames == b.effective_frames(ib(q)), 1);  % b 中共现帧的索引
    distances(q) = sphere_utils_haversine_distance( ...
        a.lons(raw_a), a.lats(raw_a), b.lons(raw_b), b.lats(raw_b)) / 1000;  % 共现帧的地面距离(km)
end
decision = dual_threshold_decide(common, distances, params);  % 对共现帧执行双门限判定
% 得分 = exp(-平均距离/T1) * exp(-方差/var_gate)，距离越小、方差越小得分越高
score = exp(-decision.mean_distance_km / max(params.dualgate_T1_km, eps)) * ...
    exp(-decision.distance_variance_km2 / max(params.dualgate_var_km2, eps));
ok = decision.accepted && score >= params.tracklet_cross_score_min;  % 双门限都通过才算成功
reason = decision.reason;
if ~ok && strcmp(reason, 'ACCEPTED'), reason = 'SCORE_GATE'; end  % 如果距离门限通过但得分不够
edge = make_edge('overlap', score, decision.mean_distance_km, ...
    decision.distance_variance_km2, decision.coexist_frames, 0, NaN, reason);
end

function [ok, edge] = temporal_edge(a, b, params, kind)
% temporal_edge 评估两个时间有序片段之间的续接/交接关系。
% kind='successor': 同雷达站的前后片段续接
% kind='handoff': 不同雷达站的时间接力交接
[ordered, earlier, later] = order_temporal(a, b);  % 确保 earlier 的时间在 later 之前
if ~ordered
    ok = false;
    edge = make_edge(kind, 0, inf, inf, 0, 0, inf, 'NO_TIME_ORDER');  % 时间重叠，无法排序
    return;
end

% 根据边类型选择门限参数
if strcmp(kind, 'successor')
    max_gap = params.tracklet_successor_max_gap_frames;  % 最大允许空缺帧数
    distance_gate = params.tracklet_successor_distance_km;  % 地理距离门限
    mahal_gate = params.tracklet_successor_mahal_gate;  % Mahalanobis 门限
    score_gate = params.tracklet_successor_score_min;  % 最低得分
else
    max_gap = params.tracklet_handoff_max_gap_frames;
    distance_gate = params.tracklet_handoff_distance_km;
    mahal_gate = params.tracklet_handoff_mahal_gate;
    score_gate = params.tracklet_handoff_score_min;
end

gap = later.first_support_frame - earlier.last_support_frame - 1;  % 两个片段之间的空缺帧数
if gap > max_gap
    ok = false;
    edge = make_edge(kind, 0, inf, inf, 0, gap, inf, 'GAP_GATE');  % 空缺太大
    return;
end

% 提取 earlier 片段最后一个支持帧的状态和 later 片段第一个支持帧的状态
ia = find(earlier.raw_frames == earlier.last_support_frame, 1);
ib = find(later.raw_frames == later.first_support_frame, 1);
dt = (later.first_support_frame - earlier.last_support_frame) * params.dt_sec;  % 时间差(秒)
% 用 CV 模型将 earlier 的状态外推到 later 的第一帧
[xp, Pp] = propagate_cv(earlier.states(:, ia), ...
    earlier.covariances(:, :, ia), dt, params);
distance = sphere_utils_haversine_distance( ...
    xp(1), xp(3), later.lons(ib), later.lats(ib)) / 1000;  % 外推位置与 later 实际位置的地理距离
S = regularize_cov(Pp + later.covariances(:, :, ib));  % 外推协方差 + later 协方差
delta = later.states(:, ib) - xp;  % 位置偏差向量
mahal = delta' * (S \ delta);  % Mahalanobis 距离平方
% 得分 = exp(-距离/distance_gate) * exp(-mahal/mahal_gate)
score = exp(-distance / max(distance_gate, eps)) * ...
    exp(-mahal / max(mahal_gate, eps));

if distance >= distance_gate
    ok = false; reason = 'DISTANCE_GATE';  % 地理距离超出门限
elseif mahal >= mahal_gate
    ok = false; reason = 'MAHALANOBIS_GATE';  % Mahalanobis 距离超出门限
elseif score < score_gate
    ok = false; reason = 'SCORE_GATE';  % 综合得分低于门限
else
    ok = true; reason = 'ACCEPTED';  % 全部通过
end
edge = make_edge(kind, score, distance, NaN, 0, gap, mahal, reason);
end

function [ordered, earlier, later] = order_temporal(a, b)
% order_temporal 判断两个片段的时间顺序。
% 如果 a 的最后支持帧在 b 的第一支持帧之前，则 earlier=a, later=b
% 反之 earlier=b, later=a
% 如果时间重叠（无法排序），返回 ordered=false
ordered = true;
if a.last_support_frame < b.first_support_frame
    earlier = a; later = b;
elseif b.last_support_frame < a.first_support_frame
    earlier = b; later = a;
else
    ordered = false; earlier = a; later = b;  % 时间重叠，无法排序
end
end

function edge = make_edge(kind, score, distance, variance, coexist, gap, mahal, reason)
% make_edge 构造边结构体
edge = struct('a', 0, 'b', 0, 'edge_type', kind, 'score', score, ...
    'distance_km', distance, 'distance_variance_km2', variance, ...
    'coexist_frames', coexist, 'gap_frames', gap, ...
    'mahalanobis_d2', mahal, 'reason', reason);
end

function edges = empty_edges()
% empty_edges 创建空边结构体数组（预定义字段）
edges = struct('a', {}, 'b', {}, 'edge_type', {}, 'score', {}, ...
    'distance_km', {}, 'distance_variance_km2', {}, ...
    'coexist_frames', {}, 'gap_frames', {}, 'mahalanobis_d2', {}, ...
    'reason', {});
end

function diagnostics = empty_diagnostics()
% empty_diagnostics 创建空诊断信息结构体数组（预定义字段）
diagnostics = struct('a', {}, 'b', {}, 'edge_type', {}, 'accepted', {}, ...
    'score', {}, 'distance_km', {}, 'distance_variance_km2', {}, ...
    'coexist_frames', {}, 'gap_frames', {}, 'mahalanobis_d2', {}, ...
    'reason', {});
end

function d = diagnostic_from_edge(edge, accepted)
% diagnostic_from_edge 将边结构体转换为诊断结构体（增加 accepted 字段）
d = edge;
d.accepted = logical(accepted);
d = orderfields(d, {'a','b','edge_type','accepted','score','distance_km', ...
    'distance_variance_km2','coexist_frames','gap_frames', ...
    'mahalanobis_d2','reason'});
end

function [hypotheses, status] = enumerate_hypotheses(segments, edges, params)
% enumerate_hypotheses 枚举所有可能的片段组合假设。
%
% 【算法】
%   1. 将片段视为图的节点，边视为连接
%   2. 找出所有连通分量（每个分量独立枚举假设）
%   3. 对每个连通分量，枚举所有非空子集
%   4. 过滤出"连通且一致"的子集作为假设
%   5. 按得分排序，超过数量限制时提前终止
%
% 【什么是"一致"？】
%   同一雷达站内的片段必须按时间顺序排列，相邻片段之间有 successor 边，
%   且时间上不重叠。不同雷达站之间可以有任意组合。

hypotheses = empty_hypotheses();  % 初始化假设列表
status = 'SUCCESS';               % 初始状态为成功
n = numel(segments);              % 片段总数
adj = false(n);                   % 邻接矩阵
for e = 1:numel(edges)            % 根据边填充邻接矩阵
    adj(edges(e).a, edges(e).b) = true;
    adj(edges(e).b, edges(e).a) = true;
end
components = graph_components(adj);  % 找出所有连通分量

for c = 1:numel(components)       % 对每个连通分量独立枚举
    members = components{c};
    if numel(members) >= 53       % 超过53个节点的图会产生 2^53 种组合，直接放弃
        status = 'GROUP_HYPOTHESIS_LIMIT_EXCEEDED';
        return;
    end
    masks = 1:(2^numel(members)-1);  % 所有非空子集的位掩码
    for mask = masks
        subset = members(logical(bitget(mask, 1:numel(members))));  % 解码位掩码
        if ~is_connected_subset(subset, adj), continue; end  % 子集必须连通
        if ~consistent_group(subset, segments, edges), continue; end  % 子集必须一致
        edge_idx = internal_edges(subset, edges);  % 子集内部的边索引
        score = sum([edges(edge_idx).score]);  % 子集得分 = 内部边得分之和
        hypotheses(end+1) = struct('hypothesis_id', numel(hypotheses)+1, ...
            'segment_indices', subset, 'support_edge_indices', edge_idx, ...
            'score', score, 'is_singleton', numel(subset) == 1); %#ok<AGROW>
        if numel(hypotheses) > params.tracklet_group_max_hypotheses  % 超过最大假设数
            status = 'GROUP_HYPOTHESIS_LIMIT_EXCEEDED';
            return;
        end
    end
end
end

function hypotheses = empty_hypotheses()
% empty_hypotheses 创建空假设结构体数组（预定义字段）
hypotheses = struct('hypothesis_id', {}, 'segment_indices', {}, ...
    'support_edge_indices', {}, 'score', {}, 'is_singleton', {});
end

function components = graph_components(adj)
% graph_components 用 BFS 找出无向图的所有连通分量
n = size(adj, 1);
seen = false(1, n);
components = {};
for start = 1:n
    if seen(start), continue; end
    queue = start; seen(start) = true; members = [];
    while ~isempty(queue)
        node = queue(1); queue(1) = [];
        members(end+1) = node; %#ok<AGROW>
        next = find(adj(node, :) & ~seen);  % 找到未访问的邻居
        seen(next) = true;
        queue = [queue, next]; %#ok<AGROW>
    end
    components{end+1} = sort(members); %#ok<AGROW>  % 排序保证可复现性
end
end

function tf = is_connected_subset(subset, adj)
% is_connected_subset 检查子集内的节点是否全部连通
% 使用 BFS 从第一个节点出发，如果能访问到所有子集节点则连通
if numel(subset) == 1, tf = true; return; end  % 单节点天然连通
local = adj(subset, subset);  % 子集内部的邻接子矩阵
seen = false(1, numel(subset)); seen(1) = true; queue = 1;
while ~isempty(queue)
    node = queue(1); queue(1) = [];
    next = find(local(node, :) & ~seen);  % 子集内部的未访问邻居
    seen(next) = true;
    queue = [queue, next]; %#ok<AGROW>
end
tf = all(seen);  % 所有子集节点都被访问到 → 连通
end

function tf = consistent_group(members, segments, edges)
% consistent_group 检查一个片段集合是否构成"一致的组"。
% 一致性条件：
%   1. 同一雷达站内的片段必须按时间顺序排列
%   2. 相邻片段之间必须有 successor 边
%   3. 同一雷达站内不能有时间重叠的片段
tf = true;
for radar = unique([segments(members).radar_id])  % 逐雷达站检查
    local = members([segments(members).radar_id] == radar);  % 该雷达站内的片段
    if numel(local) < 2, continue; end  % 少于2个片段无需检查
    [~, order] = sort([segments(local).first_support_frame]);  % 按时间排序
    local = local(order);
    for i = 1:numel(local)-1  % 检查相邻片段
        a = local(i); b = local(i+1);
        % 条件1: 相邻片段不能有有效帧重叠
        if ~isempty(intersect(segments(a).effective_frames, segments(b).effective_frames))
            tf = false; return;
        end
        % 条件2: 必须有 successor 边连接
        if ~has_edge(a, b, 'successor', edges)
            tf = false; return;
        end
    end
end
end

function tf = has_edge(a, b, kind, edges)
% has_edge 检查是否存在 a-b 之间指定类型的边
tf = any(arrayfun(@(e) strcmp(e.edge_type, kind) && ...
    ((e.a == a && e.b == b) || (e.a == b && e.b == a)), edges));
end

function idx = internal_edges(members, edges)
% internal_edges 找出连接 members 内部片段的边索引
if isempty(edges), idx = []; return; end
idx = find(arrayfun(@(e) ismember(e.a, members) && ismember(e.b, members), edges));
end

function [groups, solver] = solve_hypothesis_cover(segments, hypotheses, params)
% solve_hypothesis_cover 用整数规划求解最优片段覆盖。
%
% 【问题建模】
%   每个片段必须恰好属于一个组（覆盖约束）。
%   目标函数：最大化选中假设的得分总和。
%   由于得分相近时可能有多个等价最优解，我们额外计算次优解并比较间隙
%   来判断是否存在歧义（ambiguity）。
%
% 【整数规划模型】
%   变量: z[j] in {0, 1}  表示假设 j 是否被选中
%   约束: Aeq * z = ones(n, 1)  每个片段恰好属于一个假设
%   目标: maximize sum(hypotheses.score * z)
%
% 【歧义检测】
%   求解最优解后，添加约束"最多选 n-1 个最优假设"，再次求解。
%   如果次优解与最优解的间隙很小（< params.tracklet_group_ambiguity_margin），
%   则认为存在歧义，将这些假设标记为 AMBIGUOUS。

n = numel(segments); h = numel(hypotheses);  % 片段数和假设数
% 构建覆盖矩阵：Aeq[j,k]=1 表示假设k覆盖片段j
Aeq = zeros(n, h);
for q = 1:h, Aeq(hypotheses(q).segment_indices, q) = 1; end

% 目标函数：假设得分 + 稳定项（让得分相近的假设按ID排序，保证可复现性）
stable = (h:-1:1) * 1e-12;
objective = [hypotheses.score] + stable;
options = optimoptions('intlinprog', 'Display', 'off');  % 整数规划求解器选项
% 求解：全部变量0-1整数，等式约束（每个片段恰好一个组）
[z, fval, exitflag] = intlinprog(-objective, 1:h, [], [], ...
    Aeq, ones(n,1), zeros(h,1), ones(h,1), options);
if exitflag <= 0 || isempty(z)  % 求解失败
    groups = empty_groups();
    solver = struct('status', 'GROUP_SOLVER_FAILED', 'exitflag', exitflag, ...
        'objective', NaN, 'second_objective', NaN, 'optimality_gap', NaN, ...
        'selected_hypotheses', []);
    return;
end

selected = find(z > 0.5)';  % 选中的假设索引
best_score = -fval;          % 最优目标值（取负因为 intlinprog 是最小化）
second_score = -inf;
second_selected = [];
if ~isempty(selected)
    % 添加约束：最多选 numel(selected)-1 个最优假设中的假设
    A = zeros(1, h); A(selected) = 1;
    b = numel(selected) - 1;
    % 求解次优解
    [z2, fval2, exitflag2] = intlinprog(-objective, 1:h, A, b, ...
        Aeq, ones(n,1), zeros(h,1), ones(h,1), options);
    if exitflag2 > 0 && ~isempty(z2)
        second_score = -fval2;
        second_selected = find(z2 > 0.5)';
    end
end
gap = best_score - second_score;  % 最优与次优的间隙
ambiguous_hypotheses = [];
if isfinite(second_score) && gap < params.tracklet_group_ambiguity_margin  % 间隙太小 → 歧义
    ambiguous_hypotheses = setdiff(selected, intersect(selected, second_selected));
end

groups = empty_groups();
for q = selected
    hyp = hypotheses(q);
    members = hyp.segment_indices;
    radar_ids = unique([segments(members).radar_id]);
    status = 'CONFIRMED';  % 默认确认
    if hyp.is_singleton, status = 'SINGLE_SOURCE'; end  % 单源片段
    if ismember(q, ambiguous_hypotheses), status = 'AMBIGUOUS'; end  % 歧义
    by_radar = cell(1, max([segments.radar_id]));  % 按雷达分组的片段
    for radar = radar_ids
        by_radar{radar} = members([segments(members).radar_id] == radar);
    end
    groups(end+1) = struct('group_id', numel(groups)+1, ...
        'hypothesis_id', q, 'segment_indices', members, ...
        'segments_by_radar', {by_radar}, ...
        'support_edge_indices', hyp.support_edge_indices, ...
        'start_frame', min([segments(members).start_frame]), ...
        'end_frame', max([segments(members).end_frame]), ...
        'is_isolated', hyp.is_singleton, ...
        'is_single_source', numel(radar_ids) == 1, ...
        'score', hyp.score, 'status', status); %#ok<AGROW>
end
solver = struct('status', 'SUCCESS', 'exitflag', exitflag, ...
    'objective', best_score, 'second_objective', second_score, ...
    'optimality_gap', gap, 'selected_hypotheses', selected);
end

function groups = empty_groups()
% empty_groups 创建空分组结构体数组（预定义字段）
groups = struct('group_id', {}, 'hypothesis_id', {}, 'segment_indices', {}, ...
    'segments_by_radar', {}, 'support_edge_indices', {}, ...
    'start_frame', {}, 'end_frame', {}, 'is_isolated', {}, ...
    'is_single_source', {}, 'score', {}, 'status', {});
end

function [x_new, P_new] = propagate_cv(x, P, dt, params)
% propagate_cv 用 CV 模型将状态和协方差外推 dt 秒。
% 用于 temporal_edge 中从 earlier 片段的末帧外推到 later 片段的初帧
F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];  % CV 状态转移矩阵
Q = eye(4) * params.tracklet_prediction_q * max(abs(dt), 1);  % 过程噪声（极小值）
x_new = F * x;  % 状态外推
P_new = regularize_cov(F * P * F' + Q);  % 协方差外推 + 正则化
end
