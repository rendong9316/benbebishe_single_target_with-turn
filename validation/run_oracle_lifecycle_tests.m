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
%   2. 3/7 滑窗起始：窗口内至少 3 帧有检测才确认航迹
%      验证：在帧 1,3,5 有检测 → 第 5 帧确认（窗口 [1,5] 内有 3 次命中）
%   3. 当前帧命中要求：确认必须由当前帧检测触发，不能仅凭历史滑窗确认
%      验证：在第 3 帧确认后，第 4 帧无新检测 → 不应再次确认
%   4. 关联帧检查和候选去重：同一帧内同一航迹只关联一个点迹
%      验证：构造过期帧、远距离、近距离、杂波四种候选 → 应选最近的有效帧
%   5. 真值终止开关：truth_terminate_enable=true 时真值结束 → HISTORY
%      验证：真值只有 2 帧，第 3 帧时应转为 HISTORY
%   6. 无效起始历史拒绝：起始历史不满足 3/7 要求时抛出异常
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

    % ==================== 测试2：3/7 滑窗起始 ====================
    % 在帧 1,3,5 有检测 → 第 5 帧确认
    % 窗口 [1,5] 共 5 帧，包含 3 次真实命中（满足 QUALIFY_NUM=3）
    % 窗口跨度 5 ≤ TOLERANT_NUM=7（满足窗口大小约束）
    tempTrackList = struct([]);  % 初始临时航迹列表为空
    % 真值轨迹：飞机从 (0,0) 开始，每帧移动 (1,1)
    truth_all = {[0, 0, 0, 0, 0; 1, 1, 0, 0, 100]};
    % 创建 UKF 模板（使用 radar_params 获取 R1 的专属参数）
    ukf_tpl = ukf_jichu('create', radar_params(params, 1), 113, 33.5, 109, 33.5, params.dt_sec);
    next_id = 1;  % 航迹 ID 计数器
    frames_with_detection = [1, 3, 5];  % 有检测的帧号
    created = {};  % 记录新生成的航迹

    for frame_id = 1:5
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
            tempTrackList, remaining, original_index, params, 3, 7, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:30:120, {}, n_original);
        if frame_id < 5
            % 前 4 帧：尚未满足 3/7 确认条件，不应产生新航迹
            assert(isempty(new_tracks));
        else
            % 第 5 帧：满足确认条件，应产生一个新航迹
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
    % 断言：birth_frame=1（第一次检测在帧1），confirm_frame=5
    assert(trk.birth_frame == 1 && trk.confirm_frame == 5);
    % 断言：TotalPointCnt=5（5 帧都有记录），AsscPointCnt=3（3 帧有关联）
    % TotalLostPointCnt=2（2 帧漏检），life=5（寿命 5 帧）
    assert(trk.TotalPointCnt == 5 && trk.AsscPointCnt == 3 && trk.TotalLostPointCnt == 2);
    assert(trk.life == 5 && length(trk.asscPointList) == 3);

    % ==================== 测试3：3/7 滑窗不满足（检测间隔太稀疏）====================
    % 在帧 1,2,8 有检测 → 第 8 帧时窗口 [2,8] 内只有 2 次命中（不满足 ≥3）
    % 验证：即使总共有 3 次检测，但窗口内不足则不确认
    tempTrackList = struct([]);
    next_id = 1;
    hits = [1, 2, 8];  % 检测帧号
    for frame_id = 1:8
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
            tempTrackList, remaining, original_index, params, 3, 7, ukf_tpl, params, ...
            frame_id, next_id, truth_all, 0:30:210, {}, n_original);
        % 断言：全程不应产生任何航迹（检测太稀疏，窗口内命中不足）
        assert(isempty(new_tracks));
    end

    % ==================== 运行其他测试用例 ====================
    % 测试3：确认必须由当前帧命中触发
    test_current_hit_required(params, ukf_tpl, truth_all);
    % 测试4：关联帧检查和候选去重
    test_association_frame_and_duplicate_candidates(params);
    % 测试5：真值终止开关
    test_truth_termination_switch(params, ukf_tpl);
    % 测试6：无效起始历史拒绝
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
    % 在帧 1,2,3 有检测，第 4 帧无检测
    for frame_id = 1:4
        if frame_id <= 3
            remaining = fixture_detection(frame_id, 1);
            original_index = 1;
            n_original = 1;
        else
            remaining = [];
            original_index = [];
            n_original = 0;
        end
        [tempTrackList, new_tracks, next_id] = trackStarter_logic_oracle( ...
            tempTrackList, remaining, original_index, params, 3, 7, ukf_tpl, ...
            params, frame_id, next_id, truth_all, 0:30:120, {}, n_original);
        if frame_id < 3
            % 第 1-2 帧：滑窗内命中不足，不确认
            assert(isempty(new_tracks));
        elseif frame_id == 3
            % 第 3 帧：滑窗 [1,3] 内有 3 次命中，满足确认条件
            assert(length(new_tracks) == 1);
        else
            % 第 4 帧：无新检测，即使滑窗内仍有历史命中也不确认
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
    [~, new_tracks] = trackStarter_logic_oracle(preloaded, [], [], params, 3, 7, ...
        ukf_tpl, params, 4, 1, truth_all, 0:30:120, {}, 0);
    % 断言：当前帧无检测，不应确认（即使滑窗已满）
    assert(isempty(new_tracks));
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
