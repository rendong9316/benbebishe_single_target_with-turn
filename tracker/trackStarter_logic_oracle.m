function [tempTrackList, valid_tracks, next_id, starter_used_original] = trackStarter_logic_oracle( ...
        tempTrackList, remainingPointList, pointOriginalIndex, ukf_tpl, params, ...
        frame_id, next_id, truth_all, ~, activeTrackList, n_original_points)

    % Oracle 模式航迹起始模块 — 基于可配置滑窗的确认机制
    %
    % 核心逻辑：
    %   1. 按真值 ID 维护独立的滑窗缓冲区（每个目标一个 tempTrackEntry）
    %   2. 每帧：若有点迹命中 → 记录到对应真值的滑窗；若无 → 记空
    %   3. 滑窗长度和最少命中数由 params 中的 Oracle 起始参数配置
    %   4. 确认时使用两点法初始化 UKF：最早有效检测 + 当前帧检测
    %
    % 与普通起始器的区别：
    %   - Oracle 模式下已知每个候选点迹的 aircraft_id（真值身份），
    %     无需聚类或 GNN，直接按真值 ID 分组
    %   - 起始不依赖距离门限，而是依赖配置窗口内的真实命中次数
    %   - 每个真值 ID 有独立的滑窗，互不干扰

    % ---- 兼容省略 n_original_points 的调用 ----
    % 未传 n_original_points 时，从 pointOriginalIndex 中推导
    if nargin < 11
        n_original_points = 0;
        if ~isempty(pointOriginalIndex)
            n_original_points = max(pointOriginalIndex);
        end
    end

    % ---- 从唯一配置源读取并验证起始参数 ----
    % validate_starter_params 检查 oracle_QUALIFY_NUM 和 oracle_TOLERANT_NUM
    % 确保它们是正整数且 QUALIFY_NUM <= TOLERANT_NUM
    [QUALIFY_NUM, TOLERANT_NUM] = validate_starter_params(params);

    % 真值目标总数
    n_targets = length(truth_all);

    % 确保 tempTrackList 结构完整（每个真值目标一个条目）
    % ensure_temp_track_list 会为缺失的目标创建空的滑窗条目
    tempTrackList = ensure_temp_track_list(tempTrackList, n_targets);

    % 初始化本帧确认的新航迹列表
    valid_tracks = {};

    % 初始化起始消耗掩码（记录哪些原始点迹被起始模块消耗了）
    starter_used_original = false(1, n_original_points);

    % 找出哪些真值目标已有活跃航迹（有 → 跳过该目标的起始）
    % build_active_truth_index 遍历 activeTrackList，标记已有航迹的真值 ID
    active_truth = build_active_truth_index(activeTrackList, n_targets, params.HISTORY_TRACK);

    % 按真值 ID 分组剩余点迹，每个真值最多取一个最近候选
    % build_candidate_index 遍历 remainingPointList，按 aircraft_id 分组，
    % 返回 candidate_by_truth[真值ID] = 点在 remainingPointList 中的索引
    candidate_by_truth = build_candidate_index(remainingPointList, pointOriginalIndex, ...
        n_targets, n_original_points, frame_id);

    % ---- 逐目标处理滑窗 ----
    % 对每个真值目标独立维护一个滑窗缓冲区
    for ac = 1:n_targets
        % 若该真值目标已有活跃航迹，清空其滑窗（防止重复起始）
        % 这一步确保已跟踪的目标不会因为滑窗中有残留检测而被误起始
        if active_truth(ac)
            tempTrackList(ac).pointHistory = empty_history();
            tempTrackList(ac).missCount = 0;
            continue;
        end

        % 检查本帧是否有该目标的候选点迹
        % candidate_by_truth(ac) 非零表示本帧有该目标的检测
        j = candidate_by_truth(ac);
        current_hit = j > 0;

        if current_hit
            % 命中：记录点迹到滑窗，同时标记原始索引为已消耗
            % 将点迹信息（帧号、点迹结构、原始索引）追加到滑窗末尾
            original_index = pointOriginalIndex(j);
            starter_used_original(original_index) = true;
            tempTrackList(ac).pointHistory(end+1) = struct('frameID', frame_id, ...
                'point', remainingPointList(j), 'origIndex', original_index);
            % 重置漏检计数（命中了，漏检归零）
            tempTrackList(ac).missCount = 0;
        else
            % 未命中：记录空条目（区分"漏检"和"滑窗溢出"）
            % 空条目用于保持滑窗的时间连续性，missCount 递增
            tempTrackList(ac).pointHistory(end+1) = struct('frameID', frame_id, ...
                'point', [], 'origIndex', 0);
            tempTrackList(ac).missCount = tempTrackList(ac).missCount + 1;
        end

        % 滑动窗口截断：保持最近 TOLERANT_NUM 个物理帧
        % 超出窗口的旧检测被丢弃，防止滑窗无限增长
        if length(tempTrackList(ac).pointHistory) > TOLERANT_NUM
            tempTrackList(ac).pointHistory = ...
                tempTrackList(ac).pointHistory(end-TOLERANT_NUM+1:end);
        end

        % ---- 可配置窗口确认逻辑 ----
        % 收集滑窗中的真实检测（非空点迹）
        % collect_real_history 过滤掉空条目和无效结构，只保留有效检测
        real_hist = collect_real_history(tempTrackList(ac).pointHistory);

        % 当前帧命中且窗口内真实检测数达到配置阈值时触发确认
        if current_hit && length(real_hist) >= QUALIFY_NUM
            % 两点法初始化：最早检测 + 当前检测
            % det1 提供初始位置，det2 提供最新位置，两者差分得到初始速度
            det1 = real_hist(1).point;
            det2 = real_hist(end).point;

            % 创建新航迹：UKF 初始化 + 航迹结构体组装
            % fun_create_new_track_oracle 内部会验证 real_hist 的有效性
            valid_tracks{end+1} = fun_create_new_track_oracle(det1, det2, ukf_tpl, ...
                params, frame_id, next_id, ac, real_hist);
            next_id = next_id + 1;

            % 确认后清空滑窗，等待下一次起始
            tempTrackList(ac).pointHistory = empty_history();
            tempTrackList(ac).missCount = 0;
        end
    end
end

function [qualify_num, tolerant_num] = validate_starter_params(params)
    % 起始参数只允许由 params 提供，避免调用方另传阈值造成配置分叉
    % 检查 oracle_QUALIFY_NUM 和 oracle_TOLERANT_NUM 是否存在
    required = {'oracle_QUALIFY_NUM', 'oracle_TOLERANT_NUM'};
    for i = 1:length(required)
        if ~isfield(params, required{i})
            error('trackStarter_logic_oracle:missingConfig', ...
                '缺少 Oracle 航迹起始参数 %s', required{i});
        end
    end

    % 读取参数值
    qualify_num = params.oracle_QUALIFY_NUM;
    tolerant_num = params.oracle_TOLERANT_NUM;

    % 验证 qualify_num 是正整数
    valid_qualify = isnumeric(qualify_num) && isscalar(qualify_num) && ...
        isfinite(qualify_num) && qualify_num >= 1 && qualify_num == floor(qualify_num);
    % 验证 tolerant_num 是正整数
    valid_tolerant = isnumeric(tolerant_num) && isscalar(tolerant_num) && ...
        isfinite(tolerant_num) && tolerant_num >= 1 && tolerant_num == floor(tolerant_num);

    % 如果验证失败或 QUALIFY_NUM > TOLERANT_NUM，报错
    if ~valid_qualify || ~valid_tolerant || qualify_num > tolerant_num
        error('trackStarter_logic_oracle:invalidConfig', ...
            'Oracle 起始参数必须为正整数且 QUALIFY_NUM 不大于 TOLERANT_NUM');
    end
end

function hist = empty_history()
    % 创建空的历史记录结构体数组（预定义字段但不含元素）
    % 用于初始化 tempTrackList 各条目的 pointHistory 字段
    hist = struct('frameID', {}, 'point', {}, 'origIndex', {});
end

function tempTrackList = ensure_temp_track_list(tempTrackList, n_targets)
    % 确保 tempTrackList 包含所有 n_targets 个目标条目
    % 如果 tempTrackList 为空，创建全空数组；
    % 如果已有部分条目但不足 n_targets，补齐缺失的条目
    empty_hist = empty_history();
    if isempty(tempTrackList)
        % 使用 repmat 创建 n_targets 个相同的空条目
        tempTrackList = repmat(struct('truth_idx', [], ...
            'pointHistory', empty_hist, 'missCount', 0), 1, n_targets);
    end
    % 逐个检查是否缺少条目，补充缺失的
    for ac = 1:n_targets
        if length(tempTrackList) < ac || isempty(tempTrackList(ac).truth_idx)
            tempTrackList(ac).truth_idx = ac;
            tempTrackList(ac).pointHistory = empty_hist;
            tempTrackList(ac).missCount = 0;
        end
    end
end

function active_truth = build_active_truth_index(activeTrackList, n_targets, history_type)
    % 从活跃航迹列表中提取已关联的真值 ID
    % 遍历 activeTrackList，对于每条非 HISTORY 且有有效 truth_idx 的航迹，
    % 将其 truth_idx 标记为 true，表示该真值目标已有航迹在跟踪
    active_truth = false(1, n_targets);
    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        % 兼容 Type 和 type 两种字段名
        if isfield(trk, 'Type')
            type = trk.Type;
        elseif isfield(trk, 'type')
            type = trk.type;
        else
            continue;
        end
        % 跳过 HISTORY 航迹和无 truth_idx 的航迹
        if type == history_type || ~isfield(trk, 'truth_idx') || ...
                ~isscalar(trk.truth_idx) || ~isfinite(double(trk.truth_idx))
            continue;
        end
        truth_id = double(trk.truth_idx);
        % 验证 truth_id 在有效范围内且为整数
        if truth_id >= 1 && truth_id <= n_targets && truth_id == floor(truth_id)
            active_truth(truth_id) = true;
        end
    end
end

function candidate_by_truth = build_candidate_index(pointList, pointOriginalIndex, ...
        n_targets, n_original_points, frame_id)
    % 按真值 ID 对剩余点迹分组，返回每个真值对应的候选点迹索引
    % 遍历 remainingPointList，筛选出当前帧的有效检测（非杂波、非过期），
    % 按 aircraft_id 归类到 candidate_by_truth 数组中
    % 每个真值 ID 最多保留一个候选点迹（最近邻原则）
    candidate_by_truth = zeros(1, n_targets);
    for i = 1:length(pointList)
        % 安全检查：确保点迹索引与原始索引数组长度匹配
        if i > length(pointOriginalIndex)
            error('trackStarter_logic_oracle:indexMismatch', ...
                '剩余点迹与原始索引数量不一致');
        end
        original_index = pointOriginalIndex(i);
        % 验证原始索引在有效范围内
        if original_index < 1 || original_index > n_original_points
            error('trackStarter_logic_oracle:indexOutOfRange', '原始点迹索引越界');
        end
        dp = pointList(i);
        % 筛选条件：必须是当前帧、有有效 aircraft_id、非杂波
        if ~isfield(dp, 'frameID') || double(dp.frameID) ~= double(frame_id) || ...
                ~isfield(dp, 'aircraft_id') || ~isscalar(dp.aircraft_id) || ...
                ~isfinite(double(dp.aircraft_id)) || ...
                (isfield(dp, 'is_clutter') && dp.is_clutter)
            continue;
        end
        truth_id = double(dp.aircraft_id);
        % 将点迹归入对应真值 ID 的候选槽位（每个真值只取第一个有效检测）
        if truth_id >= 1 && truth_id <= n_targets && truth_id == floor(truth_id) && ...
                candidate_by_truth(truth_id) == 0
            candidate_by_truth(truth_id) = i;
        end
    end
end

function real_hist = collect_real_history(hist)
    % 从滑窗历史中收集真实检测（非空点迹）
    % 遍历滑窗条目，过滤掉空点迹（漏检帧）和无效结构，
    % 只保留包含 drange 字段的有效检测
    real_hist = empty_history();
    for i = 1:length(hist)
        if ~isempty(hist(i).point) && isstruct(hist(i).point) && ...
                isfield(hist(i).point, 'drange')
            real_hist(end+1) = hist(i);
        end
    end
end
