function [det_faded, fade, status] = build_faded_track_segments(action, varargin)
% BUILD_FADED_TRACK_SEGMENTS 随机衰落、support标注与动态片段提取。
%
% 这是片段管理的主调度函数，根据 action 参数分发到不同的子功能：
%   'apply_fade' — 对指定目标的检测点迹施加随机衰落（移除窗口内的目标检测）
%   'extract'    — 从快照序列中提取航迹片段（segment），标注 support/tail/effective 帧
%
% 输入:
%   action       — 字符串，操作类型 ('apply_fade' 或 'extract')
%   varargin     — 可变参数，根据 action 不同传递不同参数
%
% 输出:
%   det_faded    — 应用衰落后的检测列表，或提取的片段数组
%   fade         — 衰落配置结构体（仅 apply_fade 模式）
%   status       — 操作状态字符串 ('SUCCESS' 或错误信息)
switch lower(action)
    case 'apply_fade'
        % 随机衰落：在指定时间窗口内移除目标检测
        [det_faded, fade, status] = apply_fade(varargin{:});
    case 'extract'
        % 片段提取：从航迹快照中提取连续的片段
        det_faded = extract_segments(varargin{:});
        fade = [];
        status = 'SUCCESS';
    otherwise
        % 未知的 action，抛出错误
        error('build_faded_track_segments:unknownAction', '未知操作: %s', action);
end
end

function [output, fade, status] = apply_fade(detList, baseline_snapshots, baseline_tracks, target_id, window_length, seed, radar_id)
% apply_fade — 对指定目标在随机选择的窗口内施加衰落（移除检测）
%
% 算法流程:
%   1. 从 baseline_tracks 和 baseline_snapshots 中找出合法窗口的起始帧
%      （支持帧且活跃帧）
%   2. 从候选起始帧中随机选择一个（使用指定 seed）
%   3. 在选中的窗口内，移除所有属于 target_id 的目标检测
%   4. 返回修改后的检测列表和衰落配置信息
%
% 输入:
%   detList          — cell 数组，每帧的检测点迹列表
%   baseline_snapshots — 基线航迹快照序列
%   baseline_tracks    — 基线航迹列表
%   target_id          — 要施加衰落的目标编号
%   window_length      — 衰落窗口长度（帧数）
%   seed               — 随机种子
%   radar_id           — 雷达编号
%
% 输出:
%   output           — 衰落后的检测列表
%   fade             — 衰落配置结构体
%   status           — 操作状态

    % 初始化输出为输入（后续就地修改）
    output = detList;

    % 从基线航迹中提取 target_id 对应的 support 帧（有关联点迹的帧）
    support = support_frames_for_target(baseline_tracks, target_id);

    % 从基线快照中提取 target_id 对应的 active 帧（航迹活跃的帧）
    active = active_frames_for_target(baseline_snapshots, target_id);

    % 获取检测列表的总帧数
    n_frames = numel(detList);

    % 枚举所有可能的窗口起始帧（从第2帧开始，避免覆盖第1帧）
    % 候选起始帧必须同时是 support 帧和 active 帧
    candidates = [];
    for start_frame = 2:(n_frames - window_length + 1)
        % 检查 start_frame-1 是否同时在 support 和 active 集合中
        if ismember(start_frame - 1, support) && ismember(start_frame - 1, active)
            candidates(end+1) = start_frame; %#ok<AGROW>
        end
    end

    % 构建衰落配置结构体，记录本次衰落的元数据
    fade = struct('radar_id', radar_id, 'seed', seed, 'candidate_count', numel(candidates), ...
        'candidate_starts', candidates, 'window', [], 'removed_target_detections', 0, ...
        'target_id', target_id);

    % 如果没有合法候选窗口，返回错误状态
    if isempty(candidates)
        status = sprintf('NO_LEGAL_FADE_WINDOW_R%d', radar_id);
        return;
    end

    % 使用指定 seed 创建随机数流，从中随机选择一个候选起始帧
    stream = RandStream('mt19937ar', 'Seed', seed);
    start_frame = candidates(randi(stream, numel(candidates)));

    % 确定衰落窗口的帧号范围
    frames = start_frame:(start_frame + window_length - 1);

    % 计数器：记录被移除的目标检测数量
    removed = 0;

    % 遍历窗口内每一帧，移除属于 target_id 的检测点迹
    for k = frames
        dets = output{k};  % 取出当前帧的所有检测
        keep = true(1, numel(dets));  % 初始化保留掩码（全部保留）
        for d = 1:numel(dets)
            % 判断当前检测是否属于目标：非杂波 + 有 aircraft_id 字段 + aircraft_id 匹配 target_id
            is_target = ~dets(d).is_clutter && isfield(dets(d), 'aircraft_id') && ...
                double(dets(d).aircraft_id) == target_id;
            if is_target
                keep(d) = false;  % 标记为不保留
                removed = removed + 1;  % 计数 +1
            end
        end
        output{k} = dets(keep);  % 应用保留掩码，更新该帧检测列表
    end

    % 记录衰落窗口的起止帧和移除的检测数量
    fade.window = [frames(1), frames(end)];
    fade.removed_target_detections = removed;
    status = 'SUCCESS';
end

function segments = extract_segments(snapshots, trackList, radar_id)
% extract_segments — 从航迹快照序列中提取连续片段（segment）
%
% 算法流程:
%   1. 构建 support 映射：每条航迹的关联点迹帧号列表
%   2. 遍历所有帧的快照，按航迹 ID 分组，检测航迹的连续活跃区间
%   3. 对每个片段，计算 support_frames（有关联的帧）、effective_frames
%      （support 区间内的帧）、tail_frames（support 之后的帧）
%   4. 过滤掉没有 support 帧的空片段
%
% 输入:
%   snapshots  — 帧快照序列
%   trackList  — 航迹列表（可选，用于构建 support 映射）
%   radar_id   — 雷达编号
%
% 输出:
%   segments   — 片段结构体数组

    % 从航迹列表构建 support 映射（航迹ID → 关联帧号列表）
    support_map = build_support_map(trackList);

    % 如果 trackList 为空，后续将推断所有帧为 support 帧
    infer_all_support = isempty(trackList);

    % 初始化空片段数组（预定义字段结构）
    segments = empty_segments();

    % active 字典：key="Tk"（k为航迹ID），value=segments 数组中的索引
    % 用于跟踪每条航迹当前正在构建的片段
    active = struct();

    % 遍历所有帧的快照
    for k = 1:numel(snapshots)
        snap = snapshots{k};  % 取出第 k 帧的快照
        % 跳过空快照或没有 trackList 字段的快照
        if isempty(snap) || ~isfield(snap, 'trackList'), continue; end

        % 遍历该帧的所有航迹
        for t = 1:numel(snap.trackList)
            trk = snap.trackList{t};
            % 跳过空航迹、HISTORY 航迹(type==7)、无 UKF 状态或状态为空的航迹
            if isempty(trk) || trk.type == 7 || ~isfield(trk, 'ukf') || isempty(trk.ukf) || isempty(trk.ukf.x), continue; end

            % 构造航迹的唯一键名，如 "T1", "T2"
            key = sprintf('T%d', double(trk.id));

            % 检查该航迹是否已有活跃片段：
            % 条件1: active 字典中不存在该 key
            % 条件2: 该航迹的片段在上一帧断开（online_end_frame != k-1）
            if ~isfield(active, key) || segments(active.(key)).online_end_frame ~= k - 1
                % 航迹片段断开或首次出现 → 创建新片段
                segments(end+1) = make_segment(numel(segments) + 1, radar_id, trk, k); %#ok<AGROW>
                active.(key) = numel(segments);  % 记录该片段在 segments 数组中的索引
            else
                % 航迹连续活跃 → 将当前帧状态追加到已有片段
                idx = active.(key);
                segments(idx) = append_state(segments(idx), trk, k);
            end
        end
    end

    % 后处理：为每个片段计算 support_frames、effective_frames、tail_frames
    for i = 1:numel(segments)
        % 从 support_map 中查找该片段的关联帧号列表
        key = sprintf('T%d', segments(i).track_id);
        if isKey(support_map, key), all_support = support_map(key); else, all_support = []; end

        % 如果 trackList 为空，推断所有 raw_frames 为 support 帧
        if infer_all_support, all_support = segments(i).raw_frames; end

        % support = raw_frames 与 all_support 的交集
        % 即：既在航迹活跃期间、又有实际关联点迹的帧
        support = intersect(segments(i).raw_frames, all_support);

        % 将 support 帧列表存入片段
        segments(i).support_frames = support;

        % 如果没有 support 帧，标记所有字段为空/NaN
        if isempty(support)
            segments(i).effective_frames = [];
            segments(i).tail_frames = segments(i).raw_frames;
            segments(i).first_support_frame = NaN;
            segments(i).last_support_frame = NaN;
            segments(i).start_frame = NaN;
            segments(i).end_frame = NaN;
        else
            % 有 support 帧：
            %   effective_frames = raw_frames 中落在 [first_support, last_support] 区间内的帧
            %   tail_frames = raw_frames 中在 last_support 之后的帧
            first_support = support(1); last_support = support(end);
            segments(i).effective_frames = segments(i).raw_frames(segments(i).raw_frames >= first_support & segments(i).raw_frames <= last_support);
            segments(i).tail_frames = segments(i).raw_frames(segments(i).raw_frames > last_support);
            segments(i).first_support_frame = first_support;
            segments(i).last_support_frame = last_support;
            segments(i).start_frame = first_support;
            segments(i).end_frame = last_support;
        end
        segments(i).support_mask = ismember(segments(i).raw_frames, segments(i).support_frames);
        segments(i).coast_mask = ismember(segments(i).raw_frames, segments(i).tail_frames);
    end

    % 过滤掉没有 support 帧的空片段（这些片段没有实际关联数据）
    segments = segments(arrayfun(@(s) ~isempty(s.support_frames), segments));

    % 为每个片段分配序号 ID（从 1 开始连续编号）
    for i = 1:numel(segments), segments(i).segment_id = i; end
end

function segments = empty_segments()
% empty_segments — 创建空片段结构体数组（预定义所有字段名）
% 用于初始化 segments 数组，确保后续 append 时字段结构一致
segments = struct('segment_id', {}, 'radar_id', {}, 'track_id', {}, 'raw_frames', {}, ...
    'effective_frames', {}, 'support_frames', {}, 'tail_frames', {}, ...
    'support_mask', {}, 'coast_mask', {}, ...
    'first_support_frame', {}, 'last_support_frame', {}, 'online_end_frame', {}, ...
    'start_frame', {}, 'end_frame', {}, 'states', {}, 'covariances', {}, ...
    'pred_covariances', {}, 'process_noises', {}, 'lats', {}, 'lons', {});
end

function seg = make_segment(id, radar_id, trk, frame)
% make_segment — 创建一个新的片段结构体
%
% 输入:
%   id     — 片段编号
%   radar_id — 雷达编号
%   trk    — 航迹结构体
%   frame  — 当前帧号
%
% 输出:
%   seg    — 片段结构体，包含初始状态、协方差、噪声等信息
seg = struct('segment_id', id, 'radar_id', radar_id, 'track_id', double(trk.id), ...
    'raw_frames', frame, 'effective_frames', [], 'support_frames', [], 'tail_frames', [], ...
    'support_mask', false, 'coast_mask', false, ...
    'first_support_frame', NaN, 'last_support_frame', NaN, 'online_end_frame', frame, ...
    'start_frame', NaN, 'end_frame', NaN, 'states', trk.ukf.x, ...
    'covariances', trk.ukf.P, 'pred_covariances', track_pred_cov(trk), ...
    'process_noises', trk.ukf.Q, 'lats', trk.lat, 'lons', trk.lon);
end

function seg = append_state(seg, trk, frame)
% append_state — 将当前帧的航迹状态追加到已有片段的末尾
%
% 追加的内容包括:
%   raw_frames: 帧号
%   online_end_frame: 最新在线帧号
%   states: UKF 状态向量（列追加）
%   covariances: 协方差矩阵（第三维追加）
%   pred_covariances: 预测协方差（第三维追加）
%   process_noises: 过程噪声（第三维追加）
%   lats/lons: 经纬度
seg.raw_frames(end+1) = frame;
seg.online_end_frame = frame;
seg.states(:, end+1) = trk.ukf.x;
seg.covariances(:, :, end+1) = trk.ukf.P;
seg.pred_covariances(:, :, end+1) = track_pred_cov(trk);
seg.process_noises(:, :, end+1) = trk.ukf.Q;
seg.lats(end+1) = trk.lat;
seg.lons(end+1) = trk.lon;
end

function P = track_pred_cov(trk)
% track_pred_cov — 获取航迹的预测协方差
% 优先使用 trk.P_pred（如果有），否则回退到 trk.ukf.P
if isfield(trk, 'P_pred') && ~isempty(trk.P_pred)
    P = trk.P_pred;
else
    P = trk.ukf.P;
end
end

function map = build_support_map(trackList)
% build_support_map — 从航迹列表构建"航迹ID → 关联帧号列表"映射
%
% 遍历每条航迹的 asscPointList，提取每个关联点迹的 frameID，
% 去重后作为该航迹的 support 帧号列表。
% 返回 containers.Map 对象，key="Tk"（k为航迹ID），value=帧号数组
map = containers.Map('KeyType', 'char', 'ValueType', 'any');
for i = 1:numel(trackList)
    trk = trackList{i};
    key = sprintf('T%d', double(trk.id));
    frames = [];
    % 遍历该航迹的关联点迹历史
    if isfield(trk, 'asscPointList')
        for j = 1:numel(trk.asscPointList)
            dp = trk.asscPointList{j};
            % 提取点迹的 frameID
            if ~isempty(dp) && isfield(dp, 'frameID'), frames(end+1) = double(dp.frameID); end %#ok<AGROW>
        end
    end
    % 去重后存入映射
    map(key) = unique(frames);
end
end

function frames = support_frames_for_target(trackList, target_id)
% support_frames_for_target — 从航迹列表中提取 target_id 对应的 support 帧号
%
% 遍历 trackList，找出 truth_idx == target_id 的航迹，
% 收集其 asscPointList 中所有点迹的 frameID，去重后返回。
frames = [];
for i = 1:numel(trackList)
    trk = trackList{i};
    % 跳过没有 truth_idx、truth_idx 不匹配 target_id、或没有 asscPointList 的航迹
    if ~isfield(trk, 'truth_idx') || double(trk.truth_idx) ~= target_id || ~isfield(trk, 'asscPointList'), continue; end
    % 收集该航迹所有关联点迹的 frameID
    for j = 1:numel(trk.asscPointList)
        dp = trk.asscPointList{j};
        if ~isempty(dp) && isfield(dp, 'frameID'), frames(end+1) = double(dp.frameID); end %#ok<AGROW>
    end
end
% 去重排序
frames = unique(frames);
end

function frames = active_frames_for_target(snapshots, target_id)
% active_frames_for_target — 从快照序列中提取 target_id 对应的活跃帧号
%
% 遍历所有帧的快照，找出 truth_idx == target_id 且 type ~= 7（非HISTORY）的航迹，
% 记录这些航迹出现的帧号。
frames = [];
for k = 1:numel(snapshots)
    snap = snapshots{k};
    % 跳过空快照或没有 trackList 的快照
    if isempty(snap) || ~isfield(snap, 'trackList'), continue; end
    for i = 1:numel(snap.trackList)
        trk = snap.trackList{i};
        % 条件：非HISTORY航迹 + 有truth_idx + truth_idx匹配target_id
        if trk.type ~= 7 && isfield(trk, 'truth_idx') && double(trk.truth_idx) == target_id
            frames(end+1) = k; break; %#ok<AGROW>
        end
    end
end
end
