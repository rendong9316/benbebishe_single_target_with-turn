function [det_faded, plan, validation, status] = plan_controlled_fragmentation( ...
    detList, baseline_tracks, baseline_snapshots, ukf_tpl, tracker_params, ...
    truth_all, t_grid, radar_id, params)
% PLAN_CONTROLLED_FRAGMENTATION 构建可控的精确K片段衰落夹具。
%
% 【功能概述】
%   本函数是"可控碎片实验"的核心引擎。它的目标很明确：通过系统地移除
%   目标检测，在每部雷达上精确制造出指定数量的航迹片段（segments）。
%   例如，若希望每个目标在每部雷达上产生恰好2个片段，函数会使用回溯搜索
%   找到合适的衰落窗口，使得航迹在窗口内因缺乏检测而死亡，随后在新检测
%   到来时重新起始，从而形成两个独立片段。
%
% 【为什么需要可控碎片？】
%   真实外辐射源雷达中，电离层衰落会导致检测随机丢失，航迹自然断裂。
%   但为了做科学研究（比如对比"片段凝聚前后的RMSE"），我们需要精确
%   知道"断了几次、在哪断、断了多少帧"。手动调参无法保证可复现性，
%   所以本函数用确定性搜索+随机种子来构建可复现的衰落方案。
%
% 【算法流程】
%   1. 对每个目标，先运行一次完整跟踪得到基线航迹（期望1个片段）
%   2. 在基线航迹的"支持帧"（有关联检测的帧）之后寻找合法窗口
%   3. 在窗口内移除目标检测 → 航迹死亡 → 新航迹起始 → 片段数+1
%   4. 递归搜索直到达到目标片段数
%   5. 用最终检测列表重新运行跟踪，验证片段数是否精确匹配
%
% 【输入参数】
%   detList            — 逐帧检测列表（cell数组），会被修改以施加衰落
%   baseline_tracks    — 基线航迹列表（目标1的）
%   baseline_snapshots — 基线航迹快照序列
%   ukf_tpl            — UKF滤波器模板
%   tracker_params     — 跟踪器参数
%   truth_all          — 真值航迹（用于确定每个检测属于哪个目标）
%   t_grid             — 时间网格（帧号→秒）
%   radar_id           — 雷达编号（1或2）
%   params             — 全局仿真参数（含 fragmentation 子结构）
%
% 【输出】
%   det_faded   — 施加衰落后的检测列表（部分目标检测被移除）
%   plan        — 衰落方案详情，包含每个事件的窗口、候选数、移除检测数等
%   validation  — 验证结果，每个目标的期望片段数 vs 实际片段数
%   status      — 'SUCCESS' 或错误描述（如搜索空间耗尽、片段数不匹配）

frag = validate_fragmentation_config(params, tracker_params); % 验证碎片化配置参数的合法性
desired = frag.segments_per_target_per_radar;                  % 每个目标每雷达期望的片段数
if ~frag.enabled, desired = 1; end                            % 若未启用则恢复为1个片段（无衰落）
seed = frag.seed_r1;                                           % R1使用的随机种子
if radar_id == 2, seed = frag.seed_r2; end                    % R2使用独立种子
stream = RandStream('mt19937ar', 'Seed', seed);                % 创建可复现的随机数流

events = empty_events();                                       % 初始化衰落事件列表
det_faded = detList;                                           % 衰落后的检测列表（初始等于输入）
nodes = 0;                                                     % 回溯搜索的候选节点计数
n_targets = numel(truth_all);                                  % 目标总数
status = 'SUCCESS';                                            % 初始状态为成功

for target_id = 1:n_targets                                    % 逐目标处理
    if target_id == 1
        tracks = baseline_tracks;                              % 第一个目标用传入的基线航迹
        snapshots = baseline_snapshots;                        % 第一个目标用传入的基线快照
    else
        % 后续目标：用当前（已被前序目标衰落修改过的）检测列表重新运行跟踪
        % false 表示不打印 verbose 日志
        [tracks, ~, snapshots] = run_oracle_tracker_sequence( ...
            det_faded, ukf_tpl, tracker_params, truth_all, t_grid, false);
    end

    % 提取当前目标的有效片段（需满足 support/effective 帧数要求）
    [base_segments, ~] = valid_target_segments( ...
        snapshots, tracks, radar_id, target_id, frag);
    if numel(base_segments) ~= 1
        % 基线应该有且仅有1个片段（未衰落状态），否则夹具不可用
        status = sprintf('BASELINE_TARGET_UNTRACKED_R%d_T%d', radar_id, target_id);
        break;
    end

    % 递归搜索：在候选窗口中寻找能使片段数从 depth+1 增加到 depth+2 的位置
    [ok, det_next, target_events, nodes] = search_target_plan( ...
        det_faded, tracks, snapshots, ukf_tpl, tracker_params, truth_all, ...
        t_grid, radar_id, target_id, desired, 0, -inf, stream, frag, nodes);
    if ~ok
        % 搜索失败：要么搜索空间耗尽，要么找不到满足条件的窗口
        status = sprintf('FRAGMENT_PLAN_UNSATISFIABLE_R%d_T%d', radar_id, target_id);
        if nodes >= frag.max_search_nodes
            status = sprintf('FRAGMENT_PLAN_SEARCH_EXHAUSTED_R%d_T%d', radar_id, target_id);
        end
        break;
    end
    det_faded = det_next;                                      % 更新检测列表（叠加当前目标的衰落）
    events = [events, target_events];                          % 累积衰落事件
end

% 用最终检测列表运行完整跟踪，得到最终航迹
[final_tracks, ~, final_snapshots] = run_oracle_tracker_sequence( ...
    det_faded, ukf_tpl, tracker_params, truth_all, t_grid, false);
% 验证每个目标的实际片段数是否与期望匹配
validation = validate_counts(final_snapshots, final_tracks, radar_id, ...
    n_targets, desired, frag);

% 如果要求精确匹配但有不匹配的目标，标记为失败
if strcmp(status, 'SUCCESS') && frag.require_exact_count
    bad = find([validation.actual_segments] ~= desired, 1);
    if ~isempty(bad)
        status = sprintf('FRAGMENT_COUNT_MISMATCH_R%d_T%d', ...
            radar_id, validation(bad).truth_idx);
    end
end

% 为每个事件分配递增的 event_id
for i = 1:numel(events), events(i).event_id = i; end
% 在航迹信息上标注事件（如死亡帧、重启航迹ID等）
events = annotate_events(events, final_tracks);
% 组装衰落方案结构体
plan = struct('radar_id', radar_id, 'seed', seed, ...
    'desired_segments_per_target', desired, 'target_count', n_targets, ...
    'search_nodes', nodes, 'events', events, 'status', status);
end

function [ok, det_out, events, nodes] = search_target_plan( ...
    det_in, tracks, snapshots, ukf_tpl, tracker_params, truth_all, t_grid, ...
    radar_id, target_id, desired, depth, last_window_end, stream, frag, nodes)
% search_target_plan 递归回溯搜索：寻找能使目标产生期望片段数的衰落窗口。
%
% 【递归逻辑】
%   depth=0: 当前有1个片段，需要在目标片段数基础上再制造1个（即从K=1搜索到K=2）
%   depth=1: 当前有2个片段，继续搜索第3个窗口
%   ...
%   depth=desired-1: 已达到目标片段数，递归终止
%
%   每一步：枚举所有合法的衰落窗口起始帧 → 移除检测 → 重新跟踪 → 检查片段数
%   如果片段数增加了1，则递归进入下一步；否则尝试下一个候选窗口

events = empty_events();               % 初始化本层返回的事件列表
det_out = det_in;                      % 初始化输出检测列表（等于输入）
[segments, ~] = valid_target_segments( ...
    snapshots, tracks, radar_id, target_id, frag);  % 提取当前目标的有效片段
expected = depth + 1;                  % 期望的片段数 = 当前深度 + 1
if numel(segments) ~= expected
    ok = false;                        % 片段数不符合当前深度的期望值，剪枝
    return;
end
if expected == desired
    ok = true;                         % 已达到目标片段数，搜索成功
    return;
end

% 寻找合法的衰落窗口起始帧（不能在已使用的窗口内，不能超出检测列表末尾）
candidates = legal_candidate_starts( ...
    det_in, tracks, target_id, last_window_end, frag);
if isempty(candidates)
    ok = false;                        % 没有合法候选，回溯
    return;
end
% 随机打乱候选顺序（使用可复现的随机流），避免搜索陷入局部最优
order = randperm(stream, numel(candidates));
candidates = candidates(order);

for q = 1:numel(candidates)            % 逐个尝试候选窗口
    if nodes >= frag.max_search_nodes
        ok = false;                    % 搜索节点数达到上限，停止搜索
        return;
    end
    nodes = nodes + 1;                 % 计数+1
    % 构造衰落窗口：从候选起始帧开始，长度为 fade_length_frames
    window = candidates(q):(candidates(q) + frag.fade_length_frames - 1);
    % 应用衰落：移除窗口内属于目标_id的检测
    [trial_det, removed] = apply_target_window(det_in, target_id, window);
    if removed == 0, continue; end     % 窗口内没有目标检测，跳过

    % 用修改后的检测列表重新运行跟踪，检查片段数是否增加
    [trial_tracks, ~, trial_snapshots] = run_oracle_tracker_sequence( ...
        trial_det, ukf_tpl, tracker_params, truth_all, t_grid, false);
    [trial_segments, ~] = valid_target_segments( ...
        trial_snapshots, trial_tracks, radar_id, target_id, frag);
    if numel(trial_segments) ~= expected + 1, continue; end  % 片段数未增加，剪枝

    % 递归搜索下一个窗口
    [child_ok, child_det, child_events, nodes] = search_target_plan( ...
        trial_det, trial_tracks, trial_snapshots, ukf_tpl, tracker_params, ...
        truth_all, t_grid, radar_id, target_id, desired, depth + 1, ...
        window(end), stream, frag, nodes);
    if child_ok
        event = make_event(numel(child_events) + 1, radar_id, target_id, ...
            depth + 1, window, numel(candidates), removed);
        det_out = child_det;           % 返回成功路径上的最终检测列表
        events = [event, child_events];% 累积事件链
        ok = true;                     % 本层搜索成功
        return;                        % 找到一条可行路径，立即返回
    end
end
ok = false;                            % 所有候选都试过了，返回失败
end

function candidates = legal_candidate_starts(detList, tracks, target_id, last_end, frag)
% legal_candidate_starts 找出所有合法的衰落窗口起始帧。
%
% 合法条件（全部满足才算一个候选）：
%   1. 起始帧 > last_end（不与上一个衰落窗口重叠）
%   2. 起始帧 + 窗口长度 + 最小有效帧数 ≤ 总帧数（不能超出检测列表末尾）
%   3. 起始帧前一帧必须是该目标的"支持帧"（有关联检测的帧）
%   4. 起始帧前的支持帧中，至少有 min_support_frames 帧有目标检测
%   5. 从该支持帧到窗口结束，至少有 min_effective_frames 帧有目标检测
%   6. 窗口内完全没有目标检测（否则衰落无效）
%   7. 窗口结束后存在足够的检测证据来形成新航迹

candidates = [];                        % 初始化候选列表
n_frames = numel(detList);              % 总帧数
target_tracks = tracks_for_truth(tracks, target_id);  % 筛选出属于目标_id的航迹
if isempty(target_tracks), return; end  % 目标没有被跟踪，直接返回

% 找出目标航迹中"最新"的那一段的支持帧（用于确定窗口起始位置）
latest_first = -inf;                    % 最新航迹段的起始帧号
latest_frames = [];                     % 最新航迹段的所有支持帧
for i = 1:numel(target_tracks)
    frames = association_frames_local(target_tracks{i});  % 提取航迹的关联帧号
    if isempty(frames), continue; end
    if min(frames) > latest_first       % 找到起始帧最大的航迹段
        latest_first = min(frames);
        latest_frames = frames;
    end
end
if isempty(latest_frames), return; end  % 没有找到有效航迹段

future_margin = frag.fade_length_frames + frag.min_effective_frames - 1;
% 遍历最新航迹段的每个支持帧，尝试在其后开启一个新窗口
for q = 1:numel(latest_frames)
    support_frame = latest_frames(q);   % 当前支持帧
    start_frame = support_frame + 1;    % 窗口从支持帧的下一帧开始
    % 条件1: 不与上一个衰落窗口重叠
    if start_frame <= last_end, continue; end
    % 条件2: 窗口 + 最小有效帧数不超出检测列表末尾
    if start_frame + future_margin > n_frames, continue; end

    used = latest_frames(latest_frames <= support_frame);  % 当前支持帧之前的所有支持帧
    % 条件3: 当前支持帧之前至少有 min_support_frames 个支持帧（保证前一段航迹足够长）
    if numel(used) < frag.min_support_frames, continue; end
    % 条件4: 从航迹第一段到 support_frame 至少有 min_effective_frames 个有效帧
    if support_frame - latest_frames(1) + 1 < frag.min_effective_frames, continue; end

    window = start_frame:(start_frame + frag.fade_length_frames - 1);  % 衰落窗口范围
    % 条件5: 窗口内完全没有目标检测（否则衰落没有效果）
    if count_target_detections(detList, target_id, window) == 0, continue; end
    % 条件6: 窗口结束后有足够的检测证据能形成新航迹
    if ~has_restart_evidence(detList, target_id, window(end), frag), continue; end
    candidates(end+1) = start_frame;    % 通过所有检查，加入候选列表
end
candidates = unique(candidates);        % 去重
end

function tf = has_restart_evidence(detList, target_id, window_end, frag)
% has_restart_evidence 检查衰落窗口结束后是否有足够的检测证据能形成新航迹。
%
% 逻辑：在 window_end 之后扫描检测列表，找到一个"密集检测区域"，
% 该区域内至少有 min_support_frames 个目标检测，且该区域持续时间
% 至少达到 min_effective_frames 帧。这说明窗口结束后目标检测恢复了，
% 新航迹可以由此起始。

frames = target_detection_frames(detList, target_id);  % 目标_id的所有检测帧号
frames = frames(frames > window_end);                   % 只关心窗口结束后的检测
tf = false;                                             % 默认返回false
for i = 1:numel(frames)                                 % 逐个检测帧检查
    % 在当前帧附近±6帧范围内统计目标检测数
    hits = frames(frames >= frames(i) & frames <= frames(i) + 6);
    if numel(hits) < frag.min_support_frames, continue; end  % 检测不够密集，跳过
    % 从当前检测到列表末尾的跨度是否足够长
    if frames(end) - frames(i) + 1 >= frag.min_effective_frames
        tf = true;                                       % 找到足够的重启证据
        return;
    end
end
end

function [det_out, removed] = apply_target_window(det_in, target_id, window)
% apply_target_window 在指定窗口内移除属于 target_id 的所有检测。
%
% 这是衰落操作的核心：遍历窗口内的每一帧，将该帧中所有非杂波且
% aircraft_id == target_id 的检测点迹删除。返回被移除的检测数量。

det_out = det_in;             % 输出检测列表初始等于输入
removed = 0;                  % 被移除的检测计数器
for k = window                 % 遍历衰落窗口内的每一帧
    dets = det_out{k};        % 取出当前帧的所有检测
    keep = true(1, numel(dets));  % 初始化保留掩码（默认全部保留）
    for d = 1:numel(dets)
        % 判断条件：非杂波 + 有aircraft_id字段 + aircraft_id匹配target_id
        is_target = ~dets(d).is_clutter && isfield(dets(d), 'aircraft_id') && ...
            double(dets(d).aircraft_id) == target_id;
        if is_target
            keep(d) = false;  % 标记为不保留（移除）
            removed = removed + 1;
        end
    end
    det_out{k} = dets(keep);  % 应用保留掩码，更新该帧检测列表
end
end

function [valid, valid_indices] = valid_target_segments( ...
    snapshots, tracks, radar_id, target_id, frag)
% valid_target_segments 筛选出属于指定目标的有效片段。
%
% 有效性条件：
%   1. 片段关联的航迹的 truth_idx 等于 target_id
%   2. 片段至少有 min_support_frames 个支持帧（有关联检测的帧）
%   3. 片段至少有 min_effective_frames 个有效帧（support区间内的帧）

segments = build_faded_track_segments('extract', snapshots, tracks, radar_id);  % 提取该雷达的所有片段
valid_indices = [];
for i = 1:numel(segments)
    trk = find_track_by_id(tracks, segments(i).track_id);  % 根据track_id找到航迹
    % 航迹不存在或truth_idx不匹配 → 不属于当前目标，跳过
    if isempty(trk) || ~isfield(trk, 'truth_idx') || ...
            double(trk.truth_idx) ~= target_id
        continue;
    end
    % 支持帧数不足 → 片段质量太差，跳过
    if numel(segments(i).support_frames) < frag.min_support_frames, continue; end
    % 有效帧数不足 → 片段太短，跳过
    if numel(segments(i).effective_frames) < frag.min_effective_frames, continue; end
    valid_indices(end+1) = i; %#ok<AGROW>  % 通过所有检查，记录索引
end
valid = segments(valid_indices);  % 返回有效片段数组
end

function validation = validate_counts(snapshots, tracks, radar_id, n_targets, desired, frag)
% validate_counts 验证每个目标的实际片段数是否与期望值匹配。
% 用于衰落方案构建完成后确认夹具是否成功。

validation = struct('radar_id', {}, 'truth_idx', {}, 'desired_segments', {}, ...
    'actual_segments', {}, 'segment_indices', {}, 'status', {});
for target_id = 1:n_targets                         % 逐目标检查
    % 提取该目标在当前雷达上的有效片段
    [segments, indices] = valid_target_segments( ...
        snapshots, tracks, radar_id, target_id, frag);
    item_status = 'SUCCESS';                         % 初始状态为成功
    if numel(segments) ~= desired, item_status = 'COUNT_MISMATCH'; end  % 片段数不匹配
    validation(end+1) = struct('radar_id', radar_id, 'truth_idx', target_id, ...
        'desired_segments', desired, 'actual_segments', numel(segments), ...
        'segment_indices', indices, 'status', item_status); %#ok<AGROW>
end
end

function events = annotate_events(events, tracks)
% annotate_events 为衰落事件补充航迹信息。
% 每个衰落事件包含：衰落前航迹ID、死亡帧、重启航迹ID、确认帧。
% 这些信息需要从最终航迹列表中反查。

for e = 1:numel(events)
    target_tracks = tracks_for_truth(tracks, events(e).fixture_truth_idx);  % 目标的所有航迹
    pre_id = NaN; death_frame = NaN; restart_id = NaN; confirm_frame = NaN;
    for i = 1:numel(target_tracks)
        trk = target_tracks{i};
        frames = association_frames_local(trk);  % 该航迹关联的帧号列表
        if isempty(frames), continue; end
        % 找到衰落窗口之前的航迹（最大帧号 < 窗口起始帧）
        if min(frames) < events(e).window(1) && max(frames) < events(e).window(2)
            if isfield(trk, 'id'), pre_id = double(trk.id); end
            if isfield(trk, 'death_frame'), death_frame = double(trk.death_frame); end
        % 找到衰落窗口之后的第一条新航迹
        elseif min(frames) > events(e).window(2) && isnan(restart_id)
            if isfield(trk, 'id'), restart_id = double(trk.id); end
            if isfield(trk, 'confirm_frame'), confirm_frame = double(trk.confirm_frame); end
        end
    end
    events(e).pre_track_id = pre_id;            % 衰落前航迹ID
    events(e).death_frame = death_frame;        % 航迹死亡帧号
    events(e).restart_track_id = restart_id;    % 新航迹ID
    events(e).confirm_frame = confirm_frame;    % 新航迹确认帧号
end
end

function event = make_event(event_id, radar_id, target_id, ordinal, window, candidate_count, removed)
% make_event 构造一个衰落事件的结构体。
% ordinal 表示这是目标的第几个衰落事件（从1开始计数）。
event = struct('event_id', event_id, 'radar_id', radar_id, ...
    'fixture_truth_idx', target_id, 'ordinal', ordinal, ...
    'candidate_count', candidate_count, 'window', [window(1), window(end)], ...
    'removed_detection_count', removed, 'pre_track_id', NaN, ...
    'death_frame', NaN, 'restart_track_id', NaN, 'confirm_frame', NaN);
end

function events = empty_events()
% empty_events 创建空事件结构体数组（预定义所有字段名）
% 用于初始化 events 数组，确保后续 append 时字段结构一致
events = struct('event_id', {}, 'radar_id', {}, 'fixture_truth_idx', {}, ...
    'ordinal', {}, 'candidate_count', {}, 'window', {}, ...
    'removed_detection_count', {}, 'pre_track_id', {}, 'death_frame', {}, ...
    'restart_track_id', {}, 'confirm_frame', {});
end

function tracks_out = tracks_for_truth(tracks, target_id)
% tracks_for_truth 从航迹列表中筛选出 truth_idx 匹配 target_id 的航迹
tracks_out = {};
for i = 1:numel(tracks)
    trk = tracks{i};
    if isfield(trk, 'truth_idx') && double(trk.truth_idx) == target_id
        tracks_out{end+1} = trk; %#ok<AGROW>
    end
end
end

function trk = find_track_by_id(tracks, track_id)
% find_track_by_id 从航迹列表中按ID查找航迹
trk = [];
for i = 1:numel(tracks)
    if double(tracks{i}.id) == double(track_id)
        trk = tracks{i};
        return;
    end
end
end

function frames = association_frames_local(trk)
% association_frames_local 从航迹的 asscPointList 中提取所有关联帧号
frames = [];
if ~isfield(trk, 'asscPointList'), return; end
for i = 1:numel(trk.asscPointList)
    dp = trk.asscPointList{i};
    if ~isempty(dp) && isfield(dp, 'frameID')
        frames(end+1) = double(dp.frameID); %#ok<AGROW>
    end
end
frames = unique(frames);  % 去重
end

function frames = target_detection_frames(detList, target_id)
% target_detection_frames 找出目标在所有帧中有检测的帧号列表
frames = [];
for k = 1:numel(detList)
    if count_target_detections(detList, target_id, k) > 0
        frames(end+1) = k; %#ok<AGROW>
    end
end
end

function n = count_target_detections(detList, target_id, frames)
% count_target_detections 统计指定帧中属于 target_id 的非杂波检测数量
n = 0;
for k = frames
    dets = detList{k};
    for d = 1:numel(dets)
        % 非杂波 + 有aircraft_id字段 + aircraft_id匹配target_id
        if ~dets(d).is_clutter && isfield(dets(d), 'aircraft_id') && ...
                double(dets(d).aircraft_id) == target_id
            n = n + 1;
        end
    end
end
end

function frag = validate_fragmentation_config(params, tracker_params)
% validate_fragmentation_config 验证碎片化配置参数的合法性和一致性。
%
% 检查三个级别的约束：
%   1. 必需字段是否存在
%   2. 计数参数是否为正整数
%   3. 碎片化约束是否与跟踪器生命周期约束兼容（衰落窗口必须足够长以使航迹死亡）

if ~isfield(params, 'fragmentation') || ~isstruct(params.fragmentation)
    error('plan_controlled_fragmentation:missingConfig', ...
        'params.fragmentation is required');
end
frag = params.fragmentation;
% 检查必需字段
required = {'enabled', 'segments_per_target_per_radar', 'require_exact_count', ...
    'fade_length_frames', 'min_effective_frames', 'min_support_frames', ...
    'seed_r1', 'seed_r2', 'max_search_nodes'};
for i = 1:numel(required)
    if ~isfield(frag, required{i})
        error('plan_controlled_fragmentation:missingConfig', ...
            'Missing fragmentation.%s', required{i});
    end
end
% 检查计数参数是否为正整数
positive_ints = [frag.segments_per_target_per_radar, frag.fade_length_frames, ...
    frag.min_effective_frames, frag.min_support_frames, frag.max_search_nodes];
if any(~isfinite(positive_ints)) || any(positive_ints < 1) || ...
        any(positive_ints ~= floor(positive_ints))
    error('plan_controlled_fragmentation:invalidConfig', ...
        'Fragmentation counts must be positive integers');
end
% 约束1: 衰落窗口长度必须 ≥ tracker_K_loss（否则航迹不会自然死亡）
% 约束2: 最小支持帧数必须 ≥ oracle_QUALIFY_NUM（否则起始器会确认虚假航迹）
if frag.fade_length_frames < tracker_params.tracker_K_loss || ...
        frag.min_support_frames < tracker_params.oracle_QUALIFY_NUM
    error('plan_controlled_fragmentation:invalidConfig', ...
        'Fragmentation lifecycle constraints are weaker than the Oracle tracker');
end
% 约束3: 最小有效帧数必须 ≥ dualgate_M（否则匹配器会接受太短的片段）
if isfield(tracker_params, 'dualgate_M') && ...
        frag.min_effective_frames < tracker_params.dualgate_M
    error('plan_controlled_fragmentation:invalidConfig', ...
        'fragmentation.min_effective_frames must be at least dualgate_M');
end
end
