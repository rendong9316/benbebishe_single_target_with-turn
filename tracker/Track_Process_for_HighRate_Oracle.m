function [trackList, tempTrackList, snap, next_id, diagInfo] = Track_Process_for_HighRate_Oracle( ...
        trackList, tempTrackList, pointList, ukf_tpl, params, frame_id, next_id, truth_all, t_grid)

    % ---- 初始化：空列表保护 ----
    % 确保 trackList 和 tempTrackList 非空，避免后续索引报错
    if isempty(trackList), trackList = {}; end
    if isempty(tempTrackList), tempTrackList = struct([]); end

    % ---- 点迹规范化：补齐缺失字段 ----
    % 确保每个点迹都有 aircraft_id 和 is_clutter 字段，
    % 缺少时根据 aircraft_id==0 自动推断 is_clutter
    pointList = normalize_point_list(pointList);
    n_points = length(pointList);
    history_type = params.HISTORY_TRACK;

    % ================================================================
    % 阶段1：航迹生命周期管理 — 真值终止检测
    % ================================================================
    % 保存处理前的航迹副本，用于后续检测航迹死亡事件
    before_truth_termination = trackList;

    % 如果启用了真值终止开关，遍历所有航迹检查关联的真实目标
    % 是否已结束（当前帧时间 > 真值航迹最后一帧时间）。
    % 若真值已结束，将该航迹标记为 HISTORY 类型并记录死亡原因。
    % 这一步确保航迹不会在目标消失后继续无限期存活。
    if is_truth_termination_enabled(params)
        trackList = terminate_finished_truth( ...
            trackList, truth_all, t_grid, frame_id, history_type);
    end

    % 收集真值终止阶段的生命周期事件（死亡事件）
    truth_events = collect_lifecycle_events( ...
        before_truth_termination, trackList, frame_id, history_type);

    % 将航迹分为活跃和历史两类，HISTORY 航迹不参与后续处理
    [activeTrackList, historyTrackList] = partition_tracks(trackList, history_type);

    % ================================================================
    % 阶段2：UKF 预测 — 对所有活跃航迹执行一步预测
    % ================================================================
    % 对每条活跃航迹，调用对应 UKF 滤波器的 'prepare' action，
    % 完成 Sigma 点生成、状态预测、量测预测及协方差计算。
    % 预测结果存入 trk.x_pred / trk.P_pred 等字段，供后续关联使用。
    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        % 注入时间步长和生命计数（用于自适应 Q 的成熟度检查）
        trk.ukf.dt = params.dt_sec;
        trk.trk.life_count = trk.life + 1;
        % 调用 UKF prepare：返回 7 个输出
        %   x_pred, P_pred — 预测状态和协方差
        %   X_pred         — 预测后的 Sigma 点矩阵
        %   z_pred         — 预测量测（[Rg; az; vd]）
        %   Z_pred         — 预测量测的 Sigma 点
        %   P_zz           — 量测自协方差（含 R）
        %   ukf            — 更新后的 UKF 内部状态
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ...
            ukf_dispatch('prepare', trk.ukf);
        % 将预测结果挂载到航迹结构上，供关联模块使用
        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        activeTrackList{i} = trk;
    end

    % 保存预测后的航迹副本，用于检测更新阶段的死亡事件
    before_update = activeTrackList;

    % ================================================================
    % 阶段3：Oracle 点迹-航迹关联
    % ================================================================
    % PointTrackAssociation_Oracle 的核心逻辑：
    %   1. 从航迹中提取 truth_idx 字段，建立"真值ID→航迹索引"映射
    %   2. 从点迹中提取 aircraft_id 字段，按真值 ID 分组候选点迹
    %   3. 对每条航迹，在其对应真值 ID 的候选点迹中，
    %      选择与预测位置距离最近的未用点迹进行关联
    %   4. 返回关联结果 TPmatch_result（[track_idx, point_idx]）
    %      和未被关联的点迹索引 singlePointsIndex
    % 关键点：Oracle 模式下，关联不依赖马氏距离门限，
    %   而是直接用真值 ID 筛选候选点迹，然后在候选中找最近邻。
    %   这等同于"已知真值身份的最近邻匹配"。
    [TPmatch_result, singlePointsIndex, association_used_det] = ...
        PointTrackAssociation_Oracle(activeTrackList, pointList, frame_id);

    % ================================================================
    % 阶段4：UKF 更新 — 按关联结果更新每条航迹
    % ================================================================
    % Fun_UpdateTrackByAsscResult_Oracle 的处理逻辑：
    %   - 对有关联点迹的航迹：计算新息 innov = z_meas - z_pred，
    %     调用 ukf_dispatch('update', innov) 执行 Kalman 更新
    %   - 对未关联的航迹：调用 ukf_dispatch('update', []) 保留预测状态
    %   - 同时记录 NIS（归一化新息平方）用于后续自适应 Q
    %   - 调用质量管理系统更新 Quality、type 等字段
    activeTrackList = Fun_UpdateTrackByAsscResult_Oracle( ...
        activeTrackList, pointList, TPmatch_result, params, frame_id);

    % 收集更新阶段的生命周期事件（航迹从活跃转入历史）
    update_events = collect_lifecycle_events( ...
        before_update, activeTrackList, frame_id, history_type);

    % 将已转为 HISTORY 的航迹从 active 列表中分离出来
    [stillActiveTrackList, newlyHistoryTrackList] = ...
        partition_tracks(activeTrackList, history_type);

    % 追加到历史航迹列表
    historyTrackList = [historyTrackList, newlyHistoryTrackList];

    % ================================================================
    % 阶段5：从未关联点迹中分离出剩余点迹，送入航迹起始模块
    % ================================================================
    % 从原始点迹列表中移除已被航迹关联消耗的点迹，
    % 剩余点迹（new targets + 虚警）送入 trackStarter
    [remainingPointList, pointOriginalIndex] = ...
        fun_remove_assc_pts_from_pointlist_oracle(pointList, association_used_det);

    % trackStarter_logic_oracle 的核心逻辑：
    %   1. 按真值 ID 对剩余点迹分组
    %   2. 对每个未激活的真值目标，维护一个滑动窗口（TOLERANT_NUM=7）
    %      的候选点迹历史
    %   3. 当窗口内真实命中数 >= QUALIFY_NUM=3 时，触发航迹确认
    %   4. 使用两点法初始化 UKF（det1=最早有效检测, det2=当前帧）
    %      并创建新的 RELIABLE_TRACK
    %   5. 返回确认的新航迹 valid_tracks 和起始模块消耗的点迹掩码
    [tempTrackList, valid_tracks, next_id, starter_used_det] = ...
        trackStarter_logic_oracle(tempTrackList, remainingPointList, ...
        pointOriginalIndex, params, params.oracle_QUALIFY_NUM, ...
        params.oracle_TOLERANT_NUM, ukf_tpl, params, frame_id, next_id, ...
        truth_all, t_grid, stillActiveTrackList, n_points);

    % 收集起始确认事件
    confirm_events = collect_confirmation_events(valid_tracks, frame_id);

    % ================================================================
    % 阶段6：合并结果，构建输出
    % ================================================================
    % 合并关联和起始消耗的点迹掩码，得到本帧所有被使用的点迹
    used_det = association_used_det | starter_used_det;

    % 最终航迹列表 = 仍活跃的 + 新确认的 + 历史的
    trackList = [stillActiveTrackList, valid_tracks, historyTrackList];

    % 构建本帧航迹快照（仅保留精简字段，供可视化/融合/诊断使用）
    snap = make_snap([stillActiveTrackList, valid_tracks], frame_id);

    % 组装诊断信息，包含：
    %   - 关联结果（TPmatch_result）
    %   - 点迹消耗掩码（association/starter/used/unused）
    %   - 生命周期事件（真值终止 + 更新 + 确认三类事件）
    diagInfo = struct('frameID', frame_id, 'n_points', n_points, ...
        'TPmatch_result', TPmatch_result, ...
        'association_used_det', association_used_det, ...
        'starter_used_det', starter_used_det, 'used_det', used_det, ...
        'singlePointsIndex', singlePointsIndex, 'unused_det', find(~used_det), ...
        'lifecycle_events', [truth_events, update_events, confirm_events]);
end

function enabled = is_truth_termination_enabled(params)
    % 检查是否启用了真值终止功能
    % 从 params 中读取 oracle_truth_terminate_enable 字段，
    % 确保其为标量逻辑值
    enabled = isfield(params, 'oracle_truth_terminate_enable') && ...
        isscalar(params.oracle_truth_terminate_enable) && ...
        logical(params.oracle_truth_terminate_enable);
end

function [active, history] = partition_tracks(trackList, history_type)
    % 将航迹列表按类型拆分为活跃和历史两类
    % 遍历 trackList，根据 Type 字段是否为 HISTORY_TRACK 分类
    active = {};
    history = {};
    for i = 1:length(trackList)
        if get_track_type(trackList{i}) == history_type
            history{end+1} = trackList{i};
        else
            active{end+1} = trackList{i};
        end
    end
end

function pointList = normalize_point_list(pointList)
    % 规范化点迹列表：确保每个点迹都有 aircraft_id 和 is_clutter 字段
    % 如果缺少 aircraft_id，设为 0（杂波）；如果缺少 is_clutter，
    % 根据 aircraft_id==0 自动推断
    if isempty(pointList), pointList = []; return; end
    for i = 1:length(pointList)
        if ~isfield(pointList(i), 'aircraft_id')
            pointList(i).aircraft_id = int32(0);
        end
        if ~isfield(pointList(i), 'is_clutter')
            pointList(i).is_clutter = double(pointList(i).aircraft_id) == 0;
        end
    end
end

function trackList = terminate_finished_truth( ...
        trackList, truth_all, t_grid, frame_id, history_type)
    % 真值终止检测：遍历所有航迹，检查其关联的真值目标是否已结束
    % 若真值航迹的最后一帧时间 < 当前雷达帧对应时间，将该航迹转为 HISTORY
    % 这一步防止目标已消失但航迹仍在跟踪的情况
    if isempty(truth_all) || frame_id > length(t_grid), return; end
    t_now = t_grid(frame_id);  % 当前帧对应的绝对时间
    for i = 1:length(trackList)
        trk = trackList{i};
        % 跳过已是 HISTORY 的航迹和无 truth_idx 的航迹
        if get_track_type(trk) == history_type || ~isfield(trk, 'truth_idx')
            continue;
        end
        truth_id = double(trk.truth_idx);
        % 验证 truth_idx 的有效性：必须是 1~length(truth_all) 范围内的整数
        if ~isscalar(truth_id) || ~isfinite(truth_id) || truth_id < 1 || ...
                truth_id > length(truth_all) || isempty(truth_all{truth_id})
            continue;
        end
        % 关键判断：当前雷达帧时间已超过该真值航迹的最后一帧时间
        % truth_all{truth_id}(end, 5) 是真值轨迹最后一行的时间戳（第5列）
        if t_now > truth_all{truth_id}(end, 5)
            trk.Type = history_type;
            trk.type = history_type;
            trk.death_frame = frame_id;
            trk.death_reason = 'truth_ended';  % 区别于 K_loss 的死亡原因
            trackList{i} = trk;
        end
    end
end

function events = collect_lifecycle_events(before, after, frame_id, history_type)
    % 比较 before 和 after 两个时刻的航迹列表，收集从活跃转为 HISTORY 的事件
    % 使用航迹 ID 做映射，检测哪些航迹在本帧死亡
    events = empty_events();
    if isempty(after), return; end
    % 建立 before 时刻各航迹 ID 对应的类型映射表
    before_type_by_id = nan(1, max_track_id(before));
    for i = 1:length(before)
        id = before{i}.id;
        if is_valid_id(id)
            before_type_by_id(id) = get_track_type(before{i});
        end
    end
    % 遍历 after 时刻的航迹，找出从非 HISTORY 变为 HISTORY 的航迹
    for i = 1:length(after)
        trk = after{i};
        id = trk.id;
        if ~is_valid_id(id) || id > length(before_type_by_id) || ...
                isnan(before_type_by_id(id)) || ...
                before_type_by_id(id) == history_type || ...
                get_track_type(trk) ~= history_type
            continue;
        end
        events(end+1) = make_event('died', trk, frame_id);
    end
end

function max_id = max_track_id(trackList)
    % 遍历航迹列表，返回最大的航迹 ID
    % 用于分配 before_type_by_id 数组的大小
    max_id = 0;
    for i = 1:length(trackList)
        id = trackList{i}.id;
        if is_valid_id(id), max_id = max(max_id, id); end
    end
end

function tf = is_valid_id(id)
    % 检查 id 是否为有效航迹 ID：标量、有限、>=1 且为整数
    tf = isscalar(id) && isfinite(id) && id >= 1 && id == floor(id);
end

function events = collect_confirmation_events(valid_tracks, frame_id)
    % 收集本帧确认的所有新航迹事件，标记为 'confirmed'
    events = empty_events();
    for i = 1:length(valid_tracks)
        events(end+1) = make_event('confirmed', valid_tracks{i}, frame_id);
    end
end

function events = empty_events()
    % 初始化空的事件结构体数组，预定义所有字段
    % 每个事件包含：事件类型、航迹ID、真值索引、帧号、出生/确认/死亡帧等
    events = struct('event', {}, 'track_id', {}, 'truth_idx', {}, ...
        'frameID', {}, 'birth_frame', {}, 'confirm_frame', {}, ...
        'death_frame', {}, 'death_reason', {}, 'Quality', {}, ...
        'SuccLossPointCnt', {}, 'TotalPointCnt', {}, ...
        'AsscPointCnt', {}, 'TotalLostPointCnt', {});
end

function event = make_event(name, trk, frame_id)
    % 根据航迹结构和事件名称构造一个生命周期事件
    % name: 'died' 或 'confirmed'
    % 从 trk 中提取各类字段填充事件结构体
    event = struct('event', name, 'track_id', trk.id, ...
        'truth_idx', trk.truth_idx, 'frameID', frame_id, ...
        'birth_frame', field_or(trk, 'birth_frame', NaN), ...
        'confirm_frame', field_or(trk, 'confirm_frame', NaN), ...
        'death_frame', field_or(trk, 'death_frame', NaN), ...
        'death_reason', field_or(trk, 'death_reason', ''), ...
        'Quality', trk.Quality, ...
        'SuccLossPointCnt', trk.SuccLossPointCnt, ...
        'TotalPointCnt', trk.TotalPointCnt, ...
        'AsscPointCnt', trk.AsscPointCnt, ...
        'TotalLostPointCnt', trk.TotalLostPointCnt);
end

function value = field_or(s, name, default_value)
    % 安全读取结构体字段：如果字段存在则返回值，否则返回默认值
    if isfield(s, name), value = s.(name); else, value = default_value; end
end

function type = get_track_type(trk)
    % 获取航迹类型，兼容 Type 和 type 两种字段名（新旧代码兼容）
    if isfield(trk, 'Type'), type = trk.Type; else, type = trk.type; end
end

function snap = make_snap(activeTrackList, frame_id)
    % 构建航迹快照：仅保留精简字段供可视化/融合/诊断使用
    % 从完整航迹结构体中提取 id、type、life、truth_idx、经纬度、
    % 预测协方差 P_pred 和 UKF 的 x/P/Q 三个核心字段
    slim_tracks = cell(1, length(activeTrackList));
    for i = 1:length(activeTrackList)
        trk = activeTrackList{i};
        % 提取 UKF 核心状态
        slim_ukf = struct('x', trk.ukf.x, 'P', trk.ukf.P, 'Q', trk.ukf.Q);
        % 优先使用预测协方差，不存在则回退到当前协方差
        if isfield(trk, 'P_pred') && ~isempty(trk.P_pred)
            P_pred = trk.P_pred;
        else
            P_pred = trk.ukf.P;
        end
        slim_tracks{i} = struct('id', trk.id, 'type', get_track_type(trk), ...
            'life', trk.life, 'truth_idx', trk.truth_idx, ...
            'lat', trk.lat, 'lon', trk.lon, 'P_pred', P_pred, ...
            'ukf', slim_ukf);
    end
    snap = struct('trackList', {slim_tracks}, 'frameID', frame_id);
end
