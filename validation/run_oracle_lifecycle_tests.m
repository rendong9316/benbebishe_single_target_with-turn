% =========================================================================
% validation/run_oracle_lifecycle_tests.m — Oracle 航迹生命周期测试套件
% =========================================================================
%
% 【功能概述】
%   本文件验证 Oracle 航迹处理模块的关键行为不变量。每个测试用例构造
%   最小化的输入场景，逐帧调用 Track_Process_for_HighRate_Oracle 或
%   其子函数，然后用 assert 验证状态转移是否符合预期。
%
% 【测试的不变量清单】
%   1. K_loss 连续漏检终止：SuccLossPointCnt >= 8 时航迹转为 HISTORY
%      验证：创建空航迹，连续 8 帧无关联 → 第 8 帧 Type 应变为 HISTORY
%   2. 可配置滑窗起始：窗口内至少配置数量的帧有检测才确认航迹
%      验证：默认配置从 params.oracle_* 读取，并覆盖 2/4、4/8 回归场景
%   3. 当前帧命中要求：确认必须由当前帧检测触发，不能仅凭历史滑窗确认
%      验证：预加载足够历史后，当前帧无检测仍不得确认
%   4. 关联帧检查和候选去重：同一帧内同一航迹只关联一个点迹
%      验证：构造过期帧、远距离、近距离、杂波四种候选 → 应选最近的有效帧
%   5. 真值终止开关：truth_terminate_enable=true 时真值结束 → HISTORY
%      验证：真值只有 2 帧，第 3 帧时应转为 HISTORY
%   6. 无效起始历史拒绝：起始历史不满足配置要求时抛出异常
%      验证：传入空的 real_hist → 应抛出 invalidHistory 异常
%
% 【测试方法】
%   使用 fixture_track 和 fixture_detection 构造最小化测试用例，
%   逐帧调用 Track_Process_for_HighRate_Oracle 或子函数，
%   然后用 assert 验证状态转移是否符合预期。
% =========================================================================
function run_oracle_lifecycle_tests()
    % 加载仿真参数
    params = simulation_params_oracle();
    % 显式设置 K_loss 为 8（与参数中定义一致，双重保险）
    params.tracker_K_loss = 8;

    % ==================== 测试1：K_loss 连续漏检终止 ====================
    % 创建一个空航迹，连续 8 帧无关联 → 应转为 HISTORY
    % 这是最基础的航迹终止逻辑验证
    track = fixture_track(params);
    % 前 7 帧：持续调用质量管理和信息更新函数，但无关联点迹
    for k = 1:7
        % 传入空关联点迹列表 []，模拟连续漏检
        track = fun_track_quality_management_and_info_completion_oracle(track, [], params, params, k);
        % 断言：前 7 帧航迹不应终止（Type 不应变为 HISTORY）
        assert(track.Type ~= params.HISTORY_TRACK);
    end
    % 第 8 帧：达到 K_loss 阈值，应终止
    track = fun_track_quality_management_and_info_completion_oracle(track, [], params, params, 8);
    assert(track.Type == params.HISTORY_TRACK);       % 类型应转为 HISTORY
    assert(strcmp(track.death_reason, 'k_loss'));     % 死亡原因应为 k_loss
    assert(track.death_frame == 8);                    % 死亡帧号为 8
    assert(track.SuccLossPointCnt == 8);               % 连续漏检计数应为 8

    % ==================== 测试2：当前配置滑窗起始 ====================
    % 检测序列和期望结果均由 params.oracle_* 推导，不假设固定参数组合
    qualify_num = params.oracle_QUALIFY_NUM;
    tolerant_num = params.oracle_TOLERANT_NUM;
    tempTrackList = struct([]);  % 初始临时航迹列表为空
    truth_all = fixture_truth_through(tolerant_num);
    % 创建 UKF 模板（使用 radar_params 获取 R1 的专属参数）
    ukf_tpl = ukf_jichu('create', radar_params(params, 1), 113, 33.5, 109, 33.5, params.dt_sec);
    next_id = 1;  % 航迹 ID 计数器
    % 在整个配置窗口内均匀安排恰好 Q 次命中，最后一次位于窗口末帧
    frames_with_detection = unique(round(linspace(1, tolerant_num, qualify_num)));
    assert(length(frames_with_detection) == qualify_num);
    confirm_frame = frames_with_detection(end);
    created = {};  % 记录新生成的航迹

    for frame_id = 1:confirm_frame
        if ismember(frame_id, frames_with_detection)
            % 有检测的帧：构造一个 fixture 检测点迹
            dp = fixture_detection(frame_id, 1);
            remaining = dp;                % 剩余待关联点迹
            original_index = 1;            % 原始索引
            n_original = 1;                % 原始点迹数量
        else
            % 无检测的帧：空列表
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        % 调用起始器逻辑：传入临时航迹列表、剩余点迹、参数等
        [tempTrackList, new_tracks, next_id, mask] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:params.dt_sec:((confirm_frame-1)*params.dt_sec), {}, n_original);
        if frame_id < confirm_frame
            % 确认帧之前尚未满足配置条件，不应产生新航迹
            assert(isempty(new_tracks));
        else
            % 配置要求的最后一次命中到达后，应产生一条新航迹
            created = new_tracks;
            % 断言：mask 长度为 1 且为 true（一个点迹被消耗）
            assert(length(mask) == 1 && mask(1));
        end
    end
    % 断言：总共只创建了 1 条航迹
    assert(length(created) == 1);
    trk = created{1};
    % 断言：航迹类型为 RELIABLE_TRACK（可靠航迹）
    assert(trk.Type == params.RELIABLE_TRACK);
    expected_span = confirm_frame - frames_with_detection(1) + 1;
    assert(trk.birth_frame == frames_with_detection(1) && trk.confirm_frame == confirm_frame);
    assert(trk.TotalPointCnt == expected_span && ...
        trk.AsscPointCnt == qualify_num && ...
        trk.TotalLostPointCnt == expected_span - qualify_num);
    assert(trk.life == expected_span && length(trk.asscPointList) == qualify_num);

    % ==================== 测试3：默认配置滑窗不满足 ====================
    % 第三次命中落在配置窗口之外，旧证据淘汰后窗口内命中不足
    % 验证：即使累计命中达到配置阈值，当前窗口证据不足也不确认
    tempTrackList = struct([]);
    next_id = 1;
    hits = [1, 2, tolerant_num + 1];  % 第三次命中时帧1证据已淘汰
    for frame_id = 1:(tolerant_num + 1)
        if ismember(frame_id, hits)
            remaining = fixture_detection(frame_id, 1);
            original_index = 1;
            n_original = 1;
        else
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        % 调用起始器逻辑
        [tempTrackList, new_tracks, next_id] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:30:210, {}, n_original);
        % 断言：全程不应产生任何航迹（检测太稀疏，窗口内命中不足）
        assert(isempty(new_tracks));
    end

    % ==================== 运行其他测试用例 ====================
    % 测试4：确认必须由当前帧命中触发
    test_current_hit_required(params, ukf_tpl, truth_all);
    % 测试5-6：2/4 与 4/8 可配置起始端到端回归
    test_configurable_starter_end_to_end(params, ukf_tpl, 2, 4);
    test_configurable_starter_end_to_end(params, ukf_tpl, 4, 8);
    % 测试7：关联帧检查和候选去重
    test_association_frame_and_duplicate_candidates(params);
    % 测试8：真值终止开关
    test_truth_termination_switch(params, ukf_tpl);
    % 测试9：无效起始历史拒绝
    test_invalid_creation_history(params, ukf_tpl);

    % 所有测试通过
    disp('oracle lifecycle tests ok');
end

% =========================================================================
% test_current_hit_required — 确认必须由当前帧命中触发
% =========================================================================
% 【测试目的】
%   验证 Oracle 起始器的确认逻辑：即使滑窗内已有足够历史命中，
%   如果当前帧没有新检测，也不能确认航迹。
%   这是为了防止"延迟确认"——即过去某时刻的滑窗满足条件，
%   但在后续帧才确认，此时航迹已经过时。
% =========================================================================
function test_current_hit_required(params, ukf_tpl, truth_all)
    tempTrackList = struct([]);
    next_id = 1;
    qualify_num = params.oracle_QUALIFY_NUM;
    % 前 Q 帧有检测，第 Q+1 帧无检测
    for frame_id = 1:(qualify_num + 1)
        if frame_id <= qualify_num
            remaining = fixture_detection(frame_id, 1);
            original_index = 1;
            n_original = 1;
        else
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        [tempTrackList, new_tracks, next_id] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:30:120, {}, n_original);
        if frame_id < qualify_num
            % 第 1 至 Q-1 帧：滑窗内命中不足，不确认
            assert(isempty(new_tracks));
        elseif frame_id == qualify_num
            % 第 Q 帧：窗口内达到配置命中数，满足确认条件
            assert(length(new_tracks) == 1);
            tempTrackList = struct([]);
        else
            % 第 Q+1 帧：无新检测，即使滑窗内仍有历史命中也不确认
            assert(isempty(new_tracks));
        end
    end

    % ===== 边界测试：预加载滑窗但当前帧无检测 =====
    % 手动构造一个预加载的临时航迹，其 pointHistory 包含帧 1,2,3 的检测
    preloaded = struct('truth_idx', 1, 'pointHistory', ...
        struct('frameID', {1,2,3}, ...
        'point', {fixture_detection(1,1), fixture_detection(2,1), fixture_detection(3,1)}, ...
        'origIndex', {1,1,1}), 'missCount', 0);
    % 在第 4 帧调用起始器，但传入空检测列表
    [~, new_tracks] = trackStarter_logic_oracle(preloaded, [], [], ...
        ukf_tpl, params, 4, 1, truth_all, 0:30:120, {}, 0);
    % 断言：当前帧无检测，不应确认（即使滑窗已满）
    assert(isempty(new_tracks));
end

% =========================================================================
% test_configurable_starter_end_to_end — 可配置 Oracle 起始端到端回归
% =========================================================================
% 对明确的 Q/T 组合逐帧调用主处理函数，覆盖确认阈值、窗口边界、
% 旧证据淘汰和当前帧命中要求。成功场景还交给不变量验证器复核。
function test_configurable_starter_end_to_end(base_params, ukf_tpl, qualify_num, tolerant_num)
    params = base_params;
    params.oracle_QUALIFY_NUM = qualify_num;
    params.oracle_TOLERANT_NUM = tolerant_num;

    % Q-1 次命中不足以确认。
    [tracks, ~, ~, ~, diags] = run_oracle_frames( ...
        params, ukf_tpl, 1:(qualify_num-1), qualify_num-1);
    assert(isempty(tracks));
    assert(~has_lifecycle_event(diags, 'confirmed'));

    % 配置窗口跨度恰好为 T，并在第 Q 次（当前帧）命中时确认。
    exact_span_hits = [1:(qualify_num-1), tolerant_num];
    [finalTrackList, ~, snapshots, detList, diagList] = run_oracle_frames( ...
        params, ukf_tpl, exact_span_hits, tolerant_num);
    assert(length(finalTrackList) == 1);
    trk = finalTrackList{1};
    assert(trk.birth_frame == 1);
    assert(trk.confirm_frame == tolerant_num);
    assert(trk.AsscPointCnt == qualify_num);
    assert(trk.TotalPointCnt == tolerant_num);
    assert(trk.TotalLostPointCnt == tolerant_num - qualify_num);
    assert(length(trk.asscPointList) == qualify_num);
    assert(trk.Type == params.RELIABLE_TRACK);
    for i = 1:length(exact_span_hits)
        hit_frame = exact_span_hits(i);
        assert(length(diagList{hit_frame}.starter_used_det) == 1 && ...
            diagList{hit_frame}.starter_used_det(1));
        assert(isequaln(trk.asscPointList{i}, detList{hit_frame}(1)));
    end
    assert(length(diagList{tolerant_num}.starter_used_det) == 1 && ...
        diagList{tolerant_num}.starter_used_det(1));
    events = diagList{tolerant_num}.lifecycle_events;
    confirmed = events(strcmp({events.event}, 'confirmed'));
    assert(length(confirmed) == 1);
    assert(confirmed.track_id == trk.id && ...
        confirmed.birth_frame == 1 && ...
        confirmed.confirm_frame == tolerant_num && ...
        confirmed.AsscPointCnt == qualify_num && ...
        confirmed.TotalPointCnt == tolerant_num && ...
        confirmed.TotalLostPointCnt == tolerant_num - qualify_num);
    validate_oracle_invariants(snapshots, detList, diagList, params, finalTrackList);

    % 跨度 T+1：第一帧旧证据被淘汰，当前窗口只剩 Q-1 次命中。
    expired_hits = [1:(qualify_num-1), tolerant_num + 1];
    [tracks, ~, ~, ~, diags] = run_oracle_frames( ...
        params, ukf_tpl, expired_hits, tolerant_num + 1);
    assert(isempty(tracks));
    assert(~has_lifecycle_event(diags, 'confirmed'));

    % 预加载达到 Q 次的实际检测历史，但当前帧无命中，仍不得确认。
    preloaded_detList = cell(1, qualify_num + 1);
    point_history = struct('frameID', {}, 'point', {}, 'origIndex', {});
    for k = 1:qualify_num
        preloaded_detList{k} = fixture_detection(k, 1);
        point_history(end+1) = struct('frameID', k, ...
            'point', preloaded_detList{k}, 'origIndex', 1);
    end
    preloaded_detList{qualify_num + 1} = struct([]);
    preloaded = struct('truth_idx', 1, 'pointHistory', point_history, 'missCount', 0);
    t_grid = 0:30:(qualify_num * 30);
    truth_all = fixture_truth_through(qualify_num + 1);
    [tracks, ~, ~, ~, diag] = Track_Process_for_HighRate_Oracle( ...
        {}, preloaded, preloaded_detList{qualify_num + 1}, ukf_tpl, params, ...
        qualify_num + 1, 1, truth_all, t_grid);
    assert(isempty(tracks));
    assert(isempty(diag.lifecycle_events));
end

% 逐帧执行完整 Oracle 主处理链并收集验证器所需产物。
function [trackList, tempTrackList, snapshots, detList, diagList] = ...
        run_oracle_frames(params, ukf_tpl, hit_frames, n_frames)
    trackList = {};
    tempTrackList = struct([]);
    snapshots = cell(1, n_frames);
    detList = cell(1, n_frames);
    diagList = cell(1, n_frames);
    next_id = 1;
    truth_all = fixture_truth_through(n_frames);
    t_grid = 0:30:((n_frames-1)*30);
    for frame_id = 1:n_frames
        if ismember(frame_id, hit_frames)
            detList{frame_id} = fixture_detection(frame_id, 1);
        else
            detList{frame_id} = struct([]);
        end
        [trackList, tempTrackList, snapshots{frame_id}, next_id, diagList{frame_id}] = ...
            Track_Process_for_HighRate_Oracle(trackList, tempTrackList, ...
            detList{frame_id}, ukf_tpl, params, frame_id, next_id, truth_all, t_grid);
    end
end

% 构造覆盖完整测试帧的真值，避免确认后立即触发 truth_ended。
function truth_all = fixture_truth_through(n_frames)
    end_time = max(0, (n_frames - 1) * 30);
    truth_all = {[0, 0, 0, 0, 0; n_frames, n_frames, 0, 0, end_time]};
end

function tf = has_lifecycle_event(diagList, event_name)
    tf = false;
    for k = 1:length(diagList)
        events = diagList{k}.lifecycle_events;
        if ~isempty(events) && any(strcmp({events.event}, event_name))
            tf = true;
            return;
        end
    end
end

% =========================================================================
% test_association_frame_and_duplicate_candidates — 关联帧检查和候选去重
% =========================================================================
% 【测试目的】
%   验证 PointTrackAssociation_Oracle 函数在存在多种非法候选时的行为：
%     1. 过期帧（frameID < 当前帧）→ 应被拒绝
%     2. 远距离点迹（超出关联门限）→ 应被拒绝
%     3. 杂波（is_clutter=true）→ 应被拒绝
%     4. 合法近距离点迹 → 应被选中
%   同时验证候选去重：同一帧内同一航迹只关联一个点迹。
% =========================================================================
function test_association_frame_and_duplicate_candidates(params)
    % 构造测试航迹
    trk = fixture_track(params);
    trk.x_pred = [129.02; 0; 31.02; 0];  % 预测位置（经度≈129, 纬度≈31）
    % 构造四个候选点迹：
    stale = fixture_detection(1, 1);       % 过期帧（frame_id=1，当前帧=2）
    current_far = fixture_detection(2, 1); current_far.lon = 130; current_far.lat = 32;
    % 远距离点迹（距离航迹预测位置很远）
    current_near = fixture_detection(2, 1); current_near.lon = 129.021; current_near.lat = 31.021;
    % 近距离点迹（非常接近航迹预测位置）
    clutter = fixture_detection(2, 0); clutter.is_clutter = true;
    % 杂波点迹

    % 调用 Oracle 点迹-航迹关联函数
    % 输入：[一个航迹], [四个候选点迹], 当前帧号=2
    [matches, remaining, used] = PointTrackAssociation_Oracle( ...
        {trk}, [stale, current_far, current_near, clutter], 2);
    % 断言：第 3 个点迹（current_near）被关联到航迹 1
    assert(matches(1,2) == 3);
    % 断言：只有第 3 个点迹被消耗（used mask 中只有索引 3 为 true）
    assert(isequal(find(used), 3));
    % 断言：剩余点迹为 [stale, current_far, clutter]（索引 1,2,4）
    assert(isequal(remaining, [1,2,4]));
end

% =========================================================================
% test_truth_termination_switch — 真值终止开关测试
% =========================================================================
% 【测试目的】
%   验证 oracle_truth_terminate_enable 参数的行为：
%     - 开启时：真值轨迹结束后，航迹应转为 HISTORY 并标记 death_reason='truth_ended'
%     - 关闭时：即使真值已结束，航迹也应保持原状态
% =========================================================================
function test_truth_termination_switch(params, ukf_tpl)
    % 真值航迹只有 2 帧（t=0 和 t=30），当前帧 t=60 已超出真值范围
    truth_all = {[0,0,0,0,0; 1,1,0,0,30]};
    t_grid = [0,30,60];  % 三个时间点

    % 构造测试航迹并初始化 UKF
    trk = fixture_track(params);
    trk.ukf = ukf_tpl;
    % 初始化 UKF（用两个检测点迹做初始化滤波）
    trk.ukf = ukf_dispatch('init', trk.ukf, ...
        fixture_detection(1,1), fixture_detection(2,1));
    % 预加载关联点迹列表
    trk.asscPointList = {fixture_detection(1,1), fixture_detection(2,1)};

    % ===== 开启真值终止 =====
    enabled = params;
    enabled.oracle_truth_terminate_enable = true;
    % 调用 Oracle 主处理函数
    [tracks, ~, ~, ~, diag] = Track_Process_for_HighRate_Oracle( ...
        {trk}, struct([]), [], ukf_tpl, enabled, 3, 2, truth_all, t_grid);
    % 断言：航迹应转为 HISTORY
    assert(tracks{1}.Type == params.HISTORY_TRACK);
    % 断言：死亡原因为 'truth_ended'
    assert(strcmp(tracks{1}.death_reason, 'truth_ended'));
    % 断言：生命周期事件中有一条记录
    assert(length(diag.lifecycle_events) == 1);

    % ===== 关闭真值终止 =====
    disabled = params;
    disabled.oracle_truth_terminate_enable = false;
    [tracks, ~, ~, ~, diag] = Track_Process_for_HighRate_Oracle( ...
        {trk}, struct([]), [], ukf_tpl, disabled, 3, 2, truth_all, t_grid);
    % 断言：航迹不应转为 HISTORY
    assert(tracks{1}.Type ~= params.HISTORY_TRACK);
    % 断言：无生命周期事件
    assert(isempty(diag.lifecycle_events));
end

% =========================================================================
% test_invalid_creation_history — 无效起始历史拒绝
% =========================================================================
% 【测试目的】
%   验证 fun_create_new_track_oracle 函数在接收到空的 real_hist
%   （即没有足够的历史检测证据）时，应抛出 invalidHistory 异常。
%   这是安全保护机制：防止无依据地创建航迹。
% =========================================================================
function test_invalid_creation_history(params, ukf_tpl)
    failed = false;
    try
        % 传入空的 real_hist（最后一个参数 struct([])）
        % 这表示没有任何历史检测证据，函数应拒绝创建航迹并抛出异常
        fun_create_new_track_oracle(fixture_detection(1,1), ...
            fixture_detection(2,1), ukf_tpl, params, 2, 1, 1, struct([]));
    catch exception
        % 捕获异常并验证其标识符是否为 invalidHistory
        failed = strcmp(exception.identifier, ...
            'fun_create_new_track_oracle:invalidHistory');
    end
    % 断言：确实捕获到了预期的异常
    assert(failed);
end

% =========================================================================
% fixture_track — 构造最小化测试航迹
% =========================================================================
% 创建一个结构体化的空航迹，包含所有必需字段，
% 用于测试中避免依赖完整的航迹创建流程。
function track = fixture_track(params)
    % 构造最小 UKF 结构
    ukf = struct('x', zeros(4,1), 'P', eye(4), 'Q', eye(4));
    % 组装航迹结构体：
    % id=1, truth_idx=1, Type=RELIABLE_TRACK, Quality=1
    % 所有计数器初始化为 0
    track = struct('id', 1, 'truth_idx', 1, 'Type', params.RELIABLE_TRACK, ...
        'type', params.RELIABLE_TRACK, 'Quality', 1, 'quality', 1, ...
        'TotalPointCnt', 0, 'AsscPointCnt', 0, 'TotalLostPointCnt', 0, ...
        'SuccLossPointCnt', 0, 'missed', 0, 'life', 0, 'death_frame', [], ...
        'death_reason', '', 'ukf', ukf, 'smoothPointList', {{}}, ...
        'outputPointList', {{}}, 'isNewTrack', 0);
end

% =========================================================================
% fixture_detection — 构造最小化测试点迹
% =========================================================================
% 创建一个结构体化的检测点迹，包含所有必需字段。
% 位置随 frame_id 线性变化，方便测试中追踪。
function dp = fixture_detection(frame_id, aircraft_id)
    % 构造检测点迹结构体：
    % frameID: 帧号
    % time_sec: 对应时间（(frame_id-1)*30 秒）
    % prange/paz/pvr: 原始距离/方位/多普勒（随 frame_id 递增）
    % range_meas/azimuth_meas/radial_vel_meas: 校准后的量测值
    % drange/daz: 偏差量
    % lat/lon: 转换后的经纬度（随 frame_id 递增）
    % is_clutter: 是否为杂波（由 aircraft_id=0 判断）
    % aircraft_id: 所属飞机编号（int32 类型）
    dp = struct('frameID', frame_id, 'time_sec', (frame_id-1)*30, ...
        'prange', 1.5e6 + frame_id*1000, 'paz', 90 + frame_id*0.01, 'pvr', 100, ...
        'range_meas', 1.5e6 + frame_id*1000, 'azimuth_meas', 90 + frame_id*0.01, ...
        'radial_vel_meas', 100, 'drange', 1.5e6 + frame_id*1000, ...
        'daz', 90 + frame_id*0.01, 'lat', 31 + frame_id*0.01, ...
        'lon', 129 + frame_id*0.01, 'is_clutter', false, ...
        'aircraft_id', int32(aircraft_id));
end
