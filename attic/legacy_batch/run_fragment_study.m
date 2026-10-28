function result = run_fragment_study(scenario_name, config)
% RUN_FRAGMENT_STUDY 人工制造双站互补碎片并研究 M:N 航迹凝聚。
% 不修改现有模块；truth_idx 仅用于测试夹具和外部评估。
%
% 工作流程：
%   1. 运行无融合基线（单站跟踪）
%   2. 人为制造 R1 断片 + R2 互补片段（碎片化）
%   3. R2 时间对齐到 R1
%   4. 片段分组 + 凝聚 + 融合
%   5. 真值外部评估（覆盖度、RMSE）
%   6. 可视化仪表盘

    % 参数检查：若调用方未传场景名，使用默认的转弯场景
if nargin < 1 || isempty(scenario_name)
    scenario_name = 'single_turn';
end
    % 参数检查：若调用方未传配置，使用默认配置
if nargin < 2 || isempty(config)
    config = default_config();
end

    % 合并用户传入的配置与默认配置（用户传入的字段覆盖默认值）
config = merge_config(default_config(), config);
    % 将所有子函数所在目录加入搜索路径
addpath(genpath('.'));
    % 初始化随机数生成器，保证碎片化操作的随机性可复现
rng(config.seed);

    % ====== 基线单站跟踪 ======
    % 调用 run_without_fusion 获得完整的单站跟踪结果（含航迹快照、真值等）
fprintf('========== 基线单站跟踪 ==========%s', newline);
baseline = run_without_fusion(scenario_name);

    % ====== 人工制造互补碎片 ======
    % 对基线航迹快照进行断裂操作：
    %   R1：在航迹中间挖一个 gap，产生两段独立航迹
    %   R2：裁剪为只覆盖 R1 gap 的区域，形成互补
fprintf('%s========== 人工制造互补碎片 ==========%s', newline, newline);
[fractured_R1, fractured_R2, fracture_plan] = manufacture_fragments( ...
    baseline.trackSnapshots_R1, baseline.trackSnapshots_R2, config);
    % 验证碎片化方案的正确性（gap 内 R1 无航迹、R2 有覆盖）
assert_fragment_plan(fractured_R1, fractured_R2, fracture_plan);
print_fragment_plan(fracture_plan);

    % ====== R2 时间对齐 ======
    % 将碎片化后的 R2 航迹时间对齐到 R1 的时间网格
fprintf('%s========== R2 时间对齐 ==========%s', newline, newline);
aligned_R2 = time_align_tracks(fractured_R2, baseline.params);

    % ====== M:N 航迹段凝聚与融合 ======
    % 将 R1/R2 的碎片段按时空邻近性分组，生成凝聚后的航迹
fprintf('%s========== M:N 航迹段凝聚与融合 ==========%s', newline, newline);
grouping = tracklet_grouping(fractured_R1, aligned_R2, baseline.params);

    % ====== 真值外部评估 ======
    % 对每个凝聚组评估：覆盖帧数、覆盖率、相对于最长单段的延长量、RMSE
fprintf('%s========== 真值外部评估 ==========%s', newline, newline);
evaluation = evaluate_groups(grouping, fractured_R1, fractured_R2, ...
    baseline.truthTrajs, baseline.scenario.t1_grid, baseline.params);
print_evaluation(evaluation);

    % ====== 可视化 ======
figure_paths = {};
    % 若配置要求显示或保存图片，则绘制碎片凝聚仪表盘
if config.show_figures || config.save_figures
    fprintf('%s========== 碎片凝聚过程可视化 ==========%s', newline, newline);
        % 构建视图输入结构体，将各阶段数据打包供绘图函数使用
    figure_paths = plot_fragment_study_dashboard(result_view_inputs(...
        baseline, fractured_R1, aligned_R2, fracture_plan, grouping, evaluation), config);
end

    % 组装返回结果
result = struct();
result.config = config;                           % 实验配置
result.baseline = baseline;                       % 基线单站跟踪结果
result.fractured_R1 = fractured_R1;               % R1 碎片化后的快照
result.fractured_R2 = fractured_R2;               % R2 碎片化后的快照
result.aligned_R2 = aligned_R2;                   % 时间对齐后的 R2
result.fracture_plan = fracture_plan;             % 碎片化方案（gap、裁剪范围等）
result.grouping = grouping;                       % 片段分组结果
result.evaluation = evaluation;                   % 外部评估指标
result.figure_paths = figure_paths;               % 生成的图片路径

    % 保存结果到 .mat 文件
if config.save_result
    if ~exist('results', 'dir'), mkdir('results'); end
    output_path = fullfile('results', sprintf('fragment_study_%s_%s.mat', ...
        scenario_name, datestr(now, 'yyyymmdd_HHMMSS')));
    save(output_path, '-struct', 'result');
    fprintf('结果已保存: %s%s', output_path, newline);
end
end

function config = merge_config(defaults, overrides)
    % 将 overrides 中的非空字段合并到 defaults 副本中
config = defaults;                                  % 先复制默认配置
if isempty(overrides), return; end                   % 若无覆盖则直接返回
names = fieldnames(overrides);                       % 获取所有需要覆盖的字段名
for i = 1:numel(names)
    config.(names{i}) = overrides.(names{i});        % 逐个字段覆盖
end
end

function views = result_view_inputs(baseline, fractured_R1, aligned_R2, plan, grouping, evaluation)
    % 为可视化仪表盘构建扁平化的视图输入结构体
    % 将 plan 中的每个碎片化方案映射到对应的评估数据和分组信息
views = struct('truth_idx', {}, 'truth', {}, 'plan', {}, 'r1_before', {}, ...
    'r1_after', {}, 'r2_middle', {}, 'fused', {}, 'group', {}, ...
    'group_segments', {}, 'group_edges', {}, 'overlap1', {}, 'overlap2', {}, ...
    'evaluation', {});
for q = 1:numel(plan)
    p = plan(q);                                        % 第 q 个碎片化方案
        % 在评估结果中找到对应 truth_idx 的条目
    candidates = find([evaluation.truth_idx] == p.truth_idx);
    if isempty(candidates), continue; end
        % 选取覆盖帧数最多的那个评估条目作为代表
    [~, best] = max([evaluation(candidates).coverage_frames]);
    ev = evaluation(candidates(best));
        % 根据 group_id 找到对应的分组
    group_idx = find([grouping.groups.group_id] == ev.group_id, 1);
    if isempty(group_idx), continue; end
    group = grouping.groups(group_idx);
    segs = grouping.segments(group.segment_indices);    % 该组包含的片段
    before = find_segment(segs, 1, p.r1_original_id);   % R1 gap 前的片段
    after = find_segment(segs, 1, p.r1_new_id);         % R1 gap 后的片段
    middle = find_segment(segs, 2, p.r2_original_id);   % R2 互补片段
        % 提取组内跨雷达边（overlap 类型的边）
    edges = grouping.edges(arrayfun(@(e) ismember(e.a, group.segment_indices) && ...
        ismember(e.b, group.segment_indices), grouping.edges));
    common1 = intersect(before.frames, middle.frames);  % R1-before 与 R2 的共现帧
    common2 = intersect(after.frames, middle.frames);   % R1-after 与 R2 的共现帧
    fused = collect_fused(grouping.fused_snapshots, group.group_id); % 融合快照
        % 组装视图条目
    views(end+1) = struct('truth_idx', p.truth_idx, ... %#ok<AGROW>
        'truth', baseline.truthTrajs{p.truth_idx}, 'plan', p, ...
        'r1_before', before, 'r1_after', after, 'r2_middle', middle, ...
        'fused', fused, 'group', group, 'group_segments', segs, ...
        'group_edges', edges, 'overlap1', common1, 'overlap2', common2, ...
        'evaluation', ev);
end
end

function seg = find_segment(segs, radar_id, track_id)
    % 从片段列表中查找指定雷达 ID 和航迹 ID 对应的片段
seg = struct('frames', [], 'lats', [], 'lons', [], 'start_frame', [], 'end_frame', [], 'radar_id', radar_id, 'track_id', track_id);
idx = find([segs.radar_id] == radar_id & [segs.track_id] == track_id, 1);
if ~isempty(idx), seg = segs(idx); end
end

function fused = collect_fused(snapshots, group_id)
    % 从融合快照中收集属于指定 group_id 的融合航迹（经纬度、帧号、来源）
fused = struct('frames', [], 'lat', [], 'lon', [], 'source', {{}});
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if trk.group_id == group_id
            fused.frames(end+1) = k; %#ok<AGROW>
            fused.lat(end+1) = trk.lat; %#ok<AGROW>
            fused.lon(end+1) = trk.lon; %#ok<AGROW>
            fused.source{end+1} = trk.source; %#ok<AGROW>
        end
    end
end
end

function config = default_config()
    % 碎片化实验的默认配置参数
config = struct();
config.seed = 42;                                       % 随机种子
config.r1_gap_range = [2, 5];                           % R1 gap 的帧数范围（最小~最大）
config.r2_start_fraction_range = [0.10, 0.25];          % R2 保留区间的起始分数范围
config.r2_end_fraction_range = [0.75, 0.90];            % R2 保留区间的结束分数范围
config.min_segment_frames = 8;                          % 每个片段的最小帧数
config.new_id_offset = 1000;                            % 新航迹 ID 的偏移量（避免与原 ID 冲突）
config.save_result = true;                              % 是否保存 .mat 结果
config.show_figures = true;                             % 是否显示图窗
config.save_figures = true;                             % 是否保存图片
config.figure_visible = 'on';                           % 图窗可见性
config.figure_dpi = 180;                                % 图片导出 DPI
config.output_root = fullfile('results', 'fragment_study'); % 输出目录
config.min_overlap_frames = 3;                          % 共现帧的最小阈值
end

function [out_R1, out_R2, plan] = manufacture_fragments(in_R1, in_R2, config)
    % 人工制造互补碎片：
    %   R1: 在原航迹中间挖一个 gap，产生两段航迹
    %   R2: 裁剪为一个覆盖 R1 gap 的短片段
    % 返回：碎片化后的快照 + 碎片化方案（plan）
out_R1 = clone_snapshots(in_R1);                        % 深拷贝 R1 快照
out_R2 = clone_snapshots(in_R2);                        % 深拷贝 R2 快照
truth_ids = unique([collect_truth_ids(in_R1), collect_truth_ids(in_R2)]); % 所有涉及的 truth_idx
max_id = max([collect_track_ids(in_R1), collect_track_ids(in_R2), 0]);  % 当前最大航迹 ID
next_id = max_id + config.new_id_offset;                % 新 ID 起始值
plan = struct('truth_idx', {}, 'r1_original_id', {}, 'r1_new_id', {}, ...
    'r1_gap_start', {}, 'r1_gap_end', {}, 'r2_keep_start', {}, ...
    'r2_keep_end', {}, 'r2_original_id', {});           % 预分配碎片方案结构体

for q = 1:numel(truth_ids)                               % 遍历每个目标
    truth_idx = truth_ids(q);
        % 获取该目标在 R1/R2 上的活跃帧和航迹 ID
    [r1_frames, r1_id] = active_frames_for_truth(in_R1, truth_idx);
    [r2_frames, r2_id] = active_frames_for_truth(in_R2, truth_idx);
        % 若航迹太短则跳过（至少需要 2*min_segment_frames + max_gap 帧）
    if numel(r1_frames) < 2 * config.min_segment_frames + config.r1_gap_range(2) || ...
            numel(r2_frames) < 2 * config.min_segment_frames
        continue;
    end

        % --- R1 gap 生成 ---
    gap = randi(config.r1_gap_range);                   % 随机选择 gap 宽度
    lower = r1_frames(1) + config.min_segment_frames;   % gap 最早起始帧
    upper = r1_frames(end) - config.min_segment_frames - gap + 1;  % gap 最晚起始帧
    gap_start = randi([lower, upper]);                  % 随机选择 gap 起始帧
    gap_end = gap_start + gap - 1;                      % gap 结束帧
    new_id = next_id;                                   % 分配新航迹 ID
    next_id = next_id + 1;                              % ID 递增

        % --- R2 裁剪区间 ---
    r2_span = r2_frames(end) - r2_frames(1) + 1;        % R2 航迹总跨度
    start_fraction = random_between(config.r2_start_fraction_range);  % 随机起始分数
    end_fraction = random_between(config.r2_end_fraction_range);      % 随机结束分数
    keep_start = max(r2_frames(1), floor(r2_frames(1) + start_fraction * r2_span));
    keep_end = min(r2_frames(end), ceil(r2_frames(1) + end_fraction * r2_span));
        % 确保 R2 保留区间与 R1 gap 有重叠（空间分集条件）
    keep_start = min(keep_start, gap_start - 1);
    keep_end = max(keep_end, gap_end + 1);

        % 执行 R1 分裂和 R2 裁剪
    out_R1 = split_track(out_R1, truth_idx, r1_id, gap_start, gap_end, new_id);
    out_R2 = crop_track(out_R2, truth_idx, r2_id, keep_start, keep_end);

        % 记录碎片化方案
    plan(end+1) = struct('truth_idx', truth_idx, ... %#ok<AGROW>
        'r1_original_id', r1_id, 'r1_new_id', new_id, ...
        'r1_gap_start', gap_start, 'r1_gap_end', gap_end, ...
        'r2_keep_start', keep_start, 'r2_keep_end', keep_end, ...
        'r2_original_id', r2_id);
end
end

function snapshots = clone_snapshots(input)
    % 深拷贝快照结构体：复制每个帧的 trackList 和 frameID
snapshots = cell(size(input));
for k = 1:numel(input)
    snap = input{k};
    tracks = {};
    if ~isempty(snap) && isfield(snap, 'trackList')
        tracks = snap.trackList;
    end
    snapshots{k} = struct('trackList', {tracks}, 'frameID', snap.frameID);
end
end

function ids = collect_truth_ids(snapshots)
    % 从快照中收集所有非终止航迹的 truth_idx（去重）
ids = [];
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx')
            ids(end+1) = double(trk.truth_idx); %#ok<AGROW>
        end
    end
end
ids = unique(ids);
end

function ids = collect_track_ids(snapshots)
    % 从快照中收集所有航迹的 id（去重）
ids = [];
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && isfield(trk, 'id')
            ids(end+1) = double(trk.id); %#ok<AGROW>
        end
    end
end
end

function [frames, track_id] = active_frames_for_truth(snapshots, truth_idx)
    % 找出指定 truth_idx 的航迹在各帧中的活跃帧号和航迹 ID
frames = [];
track_id = NaN;
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx') && ...
                double(trk.truth_idx) == truth_idx
            frames(end+1) = k; %#ok<AGROW>
            if isnan(track_id), track_id = double(trk.id); end
            break;                                          % 每帧只取第一个匹配项
        end
    end
end
end

function snapshots = split_track(snapshots, truth_idx, original_id, gap_start, gap_end, new_id)
    % 将指定 truth_idx + original_id 的航迹在 [gap_start, gap_end] 范围内断开
    % gap 之后的部分赋予 new_id，形成第二段航迹
for k = 1:numel(snapshots)
    tracks = snapshots{k}.trackList;
    keep = true(1, numel(tracks));                        % 标记哪些航迹保留
    for t = 1:numel(tracks)
        trk = tracks{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx') && ...
                double(trk.truth_idx) == truth_idx && double(trk.id) == original_id
            if k >= gap_start && k <= gap_end
                keep(t) = false;                          % gap 帧内：移除该航迹
            elseif k > gap_end
                trk.id = new_id;                          % gap 之后：赋予新 ID
                tracks{t} = trk;
            end
        end
    end
    snapshots{k}.trackList = tracks(keep);                % 更新帧内的航迹列表
end
end

function snapshots = crop_track(snapshots, truth_idx, original_id, keep_start, keep_end)
    % 裁剪航迹：只保留 [keep_start, keep_end] 范围内的帧
for k = 1:numel(snapshots)
    tracks = snapshots{k}.trackList;
    keep = true(1, numel(tracks));
    for t = 1:numel(tracks)
        trk = tracks{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx') && ...
                double(trk.truth_idx) == truth_idx && double(trk.id) == original_id && ...
                (k < keep_start || k > keep_end)          % 超出保留范围的帧
            keep(t) = false;
        end
    end
    snapshots{k}.trackList = tracks(keep);
end
end

function value = random_between(range)
    % 在 [range(1), range(2)] 均匀随机采样
value = range(1) + rand() * (range(2) - range(1));
end

function assert_fragment_plan(r1, r2, plan)
    % 验证碎片化方案的正确性：
    %   1. R1 在 gap 前后各有独立航迹 ID
    %   2. R1 gap 内无航迹
    %   3. R2 在 gap 帧内有覆盖（空间分集条件）
assert(~isempty(plan), '未生成任何碎片计划');
for q = 1:numel(plan)
    p = plan(q);
    ids_r1 = ids_for_truth(r1, p.truth_idx);
    assert(ismember(p.r1_original_id, ids_r1) && ismember(p.r1_new_id, ids_r1), ...
        'R1 未形成两个独立航迹 ID');
    for k = p.r1_gap_start:p.r1_gap_end
        assert(~has_truth_track(r1{k}, p.truth_idx), 'R1 gap 内仍有目标航迹');
        assert(has_truth_track(r2{k}, p.truth_idx), 'R2 未覆盖 R1 gap，空间分集条件不成立');
    end
end
end

function ids = ids_for_truth(snapshots, truth_idx)
    % 收集指定 truth_idx 的所有航迹 ID
ids = [];
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && isfield(trk, 'truth_idx') && double(trk.truth_idx) == truth_idx
            ids(end+1) = double(trk.id); %#ok<AGROW>
        end
    end
end
ids = unique(ids);
end

function tf = has_truth_track(snap, truth_idx)
    % 判断单帧快照中是否存在指定 truth_idx 的航迹
tf = false;
for t = 1:numel(snap.trackList)
    trk = snap.trackList{t};
    if ~isempty(trk) && isfield(trk, 'truth_idx') && double(trk.truth_idx) == truth_idx
        tf = true;
        return;
    end
end
end

function print_fragment_plan(plan)
    % 打印碎片化方案的摘要信息
for q = 1:numel(plan)
    p = plan(q);
    fprintf('目标%d: R1 #%d -> gap[%d,%d] -> #%d；R2 #%d 保留[%d,%d]%s', ...
        p.truth_idx, p.r1_original_id, p.r1_gap_start, p.r1_gap_end, ...
        p.r1_new_id, p.r2_original_id, p.r2_keep_start, p.r2_keep_end, newline);
end
end

function evaluation = evaluate_groups(grouping, r1, r2, truthTrajs, t_grid, params)
    % 对每个凝聚组计算外部评估指标：覆盖率、延长帧数、RMSE
evaluation = struct('group_id', {}, 'truth_idx', {}, 'segment_count', {}, ...
    'coverage_frames', {}, 'truth_frames', {}, 'coverage_ratio', {}, ...
    'best_single_frames', {}, 'extension_frames', {}, 'rmse_km', {});
for g = 1:numel(grouping.groups)
    group = grouping.groups(g);
    segment_indices = group.segment_indices;
    truth_votes = [];                                       % 投票统计：每个片段贡献一个 truth_idx
    single_lengths = [];
    for s = segment_indices
        seg = grouping.segments(s);
        truth_votes = [truth_votes, truth_labels_for_segment(seg, r1, r2)]; %#ok<AGROW>
        single_lengths(end+1) = numel(seg.frames); %#ok<AGROW>
    end
    truth_votes = truth_votes(truth_votes > 0);             % 过滤掉无效投票
    if isempty(truth_votes), continue; end
    truth_idx = mode(truth_votes);                          % 众数投票确定该组的真值目标
        % 计算融合航迹与真值的误差
    [errors, covered] = group_errors(grouping.fused_snapshots, g, truthTrajs{truth_idx}, t_grid);
    evaluation(end+1) = struct('group_id', group.group_id, 'truth_idx', truth_idx, ... %#ok<AGROW>
        'segment_count', numel(segment_indices), 'coverage_frames', covered, ...
        'truth_frames', numel(t_grid), 'coverage_ratio', covered / numel(t_grid), ...
        'best_single_frames', max(single_lengths), ...
        'extension_frames', covered - max(single_lengths), ...
        'rmse_km', sqrt(mean(errors.^2)));
end
end

function labels = truth_labels_for_segment(seg, r1, r2)
    % 从片段所在的雷达快照中提取 truth_idx（用于投票）
labels = [];
if seg.radar_id == 1, snapshots = r1; else, snapshots = r2; end
for k = seg.frames
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if double(trk.id) == seg.track_id && isfield(trk, 'truth_idx')
            labels(end+1) = double(trk.truth_idx); %#ok<AGROW>
        end
    end
end
end

function [errors, covered] = group_errors(fused, group_id, truth, t_grid)
    % 计算指定组内融合航迹与真值的逐帧误差
errors = [];
covered = 0;
for k = 1:min(numel(fused), numel(t_grid))
    tracks = fused{k}.trackList;
    for t = 1:numel(tracks)
        trk = tracks{t};
        if trk.group_id ~= group_id, continue; end
            % 插值得到真值在当前帧的经纬度
        truth_lon = interp1(truth.time_sec, truth.lon, t_grid(k), 'linear', NaN);
        truth_lat = interp1(truth.time_sec, truth.lat, t_grid(k), 'linear', NaN);
        if isfinite(truth_lon) && isfinite(truth_lat)
            errors(end+1) = haversine_km(trk.lat, trk.lon, truth_lat, truth_lon); %#ok<AGROW>
            covered = covered + 1;
        end
    end
end
end

function print_evaluation(evaluation)
    % 打印每个凝聚组的评估结果
for i = 1:numel(evaluation)
    e = evaluation(i);
    fprintf('Group%d -> 目标%d: %d段, 覆盖=%d帧(%.1f%%), 比最长单段延长=%d帧, RMSE=%.2fkm%s', ...
        e.group_id, e.truth_idx, e.segment_count, e.coverage_frames, ...
        100*e.coverage_ratio, e.extension_frames, e.rmse_km, newline);
end
end

function d = haversine_km(lat1, lon1, lat2, lon2)
    % 计算两点间的大圆距离（Haversine 公式），单位为 km
R = 6371.0088;                                              % 地球半径
dlat = deg2rad(lat2-lat1);                                  % 纬度差（弧度）
dlon = deg2rad(lon2-lon1);                                  % 经度差（弧度）
a = sin(dlat/2)^2 + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2)^2;
d = 2*R*atan2(sqrt(a), sqrt(max(0, 1-a)));                 % Haversine 公式
end
