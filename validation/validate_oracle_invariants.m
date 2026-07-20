% =========================================================================
% validate_oracle_invariants.m — Oracle 航迹处理不变量验证器
% =========================================================================
%
% 【功能概述】
%   本函数在仿真运行结束后，对整个处理链路输出的中间结果进行全面验证，
%   确保 Oracle 航迹处理模块的所有关键约束在整个仿真过程中未被破坏。
%
% 【验证的不变量清单】
%   不变量1：雷达硬约束 —— Pd=0.6, Pfa=0.001 不可修改
%   不变量2：Oracle 起始参数合法，且确认行为遵守配置的滑窗规则
%   不变量3：每个点迹只能被消耗一次（关联消耗和起始消耗互斥，不可重复）
%   不变量4：关联消耗的点迹必须是真实检测（非杂波、非过期帧、aircraft_id>0）
%   不变量5：快照中每条航迹的精简字段格式正确（不含冗余数据）
%   不变量6：快照中无重复的 active ID 或 truth_idx
%   不变量7：确认事件必须有配置要求的窗口证据且由当前帧命中触发
%   不变量8：最终航迹的 asscPointList 必须全部对应真实存在的检测（无虚构）
%
% 【输入参数】
%   trackSnapshots — cell 数组，每帧的航迹快照
%   detList        — cell 数组，每帧的检测点迹
%   diagList       — cell 数组，每帧的诊断信息（含 used_det, lifecycle_events 等）
%   params         — 参数结构体
%   finalTrackList — 最终航迹列表（可选，默认空）
% =========================================================================
function validate_oracle_invariants(trackSnapshots, detList, diagList, params, finalTrackList)
    % Oracle 不变量验证 — 确保航迹处理过程中的关键约束不被破坏
    %
    % 验证的不变量：
    %   1. 雷达硬约束：Pd=0.6, Pfa=0.001 不可修改
    %   2. Oracle 起始参数合法，且确认行为遵守配置的滑窗规则
    %   3. 每个点迹只能被消耗一次（关联或起始，不可重复）
    %   4. 关联消耗的点迹必须是真实检测（非杂波、非过期帧）
    %   5. 快照中每条航迹的精简字段格式正确
    %   6. 快照中无重复的 active ID 或 truth_idx
    %   7. 确认事件必须有配置要求的窗口证据且由当前帧命中触发
    %   8. 最终航迹的 asscPointList 必须全部对应真实存在的检测

    % 如果调用者未传 finalTrackList，设为空 cell
    if nargin < 5, finalTrackList = {}; end

    % ==================== 不变量1：雷达硬约束验证 ====================
    % 验证 Pd 严格等于 0.6（使用 eps 容差避免浮点误差）
    assert(abs(params.detection_probability - 0.6) < eps, 'Pd hard constraint violated');
    % 验证 Pfa 严格等于 0.001
    assert(abs(params.false_alarm_rate - 0.001) < eps, 'Pfa hard constraint violated');
    % ==================== 不变量2：Oracle 起始配置合法性 ====================
    validate_starter_config(params);

    % 初始化确认事件证据收集器
    % 按 truth_id 索引，每个元素是一个 cell，记录该飞机的确认帧历史
    confirmation_hits = cell(1, max(1, max_truth_id_from_events(diagList)));

    % ==================== 逐帧遍历验证 ====================
    for k = 1:length(trackSnapshots)
        dets = detList{k};          % 当前帧的所有检测点迹
        diag = diagList{k};         % 当前帧的诊断信息
        n_dets = length(dets);      % 当前帧检测总数

        % ==================== 不变量3：掩码长度一致性 ====================
        % association_used_det 的长度应等于当前帧检测数
        assert(length(diag.association_used_det) == n_dets, ...
            'Frame %d association mask length mismatch', k);
        % starter_used_det 的长度也应等于当前帧检测数
        assert(length(diag.starter_used_det) == n_dets, ...
            'Frame %d starter mask length mismatch', k);
        % used_det（合并掩码）的长度也应等于当前帧检测数
        assert(length(diag.used_det) == n_dets, ...
            'Frame %d combined mask length mismatch', k);

        % ==================== 不变量3：关联和起始互斥 ====================
        % 关联消耗和起始消耗不能作用于同一个点迹（一个点迹不能被重复使用）
        % & 是按位与，any(...) 检查结果中是否有 true
        assert(~any(diag.association_used_det & diag.starter_used_det), ...
            'Frame %d detection consumed twice', k);

        % ==================== 不变量3：combined mask 是两者的并集 ====================
        % used_det 应该等于 association_used_det 和 starter_used_det 的逻辑或
        assert(isequal(diag.used_det, ...
            diag.association_used_det | diag.starter_used_det), ...
            'Frame %d combined mask mismatch', k);

        % ==================== unused_det 验证 ====================
        % unused_det 应该是 used_det 的补集（未被消耗的点迹索引）
        assert(isequal(diag.unused_det, find(~diag.used_det)), ...
            'Frame %d unused indices mismatch', k);

        % ==================== 不变量4：被消耗点迹合法性检查 ====================
        % 调用子函数验证所有被消耗的点迹都是真实检测
        validate_consumed_detections(dets, diag, k);

        % ==================== 快照结构验证 ====================
        snap = trackSnapshots{k};
        % 验证快照是结构体，包含 trackList 和 frameID 字段
        % 且 frameID 等于当前帧号 k
        assert(isstruct(snap) && isfield(snap, 'trackList') && ...
            isfield(snap, 'frameID') && snap.frameID == k, ...
            'Frame %d invalid snapshot', k);

        % ==================== 不变量6：无重复 ID 或 truth_idx ====================
        seen_ids = [];    % 记录已见过的航迹 ID
        seen_truth = [];  % 记录已见过的 truth_idx
        for i = 1:length(snap.trackList)
            trk = snap.trackList{i};
            % ==================== 不变量5：精简字段格式验证 ====================
            % 调用子函数验证快照中的航迹只包含必要字段
            validate_slim_track(trk, params, k);
            % 断言：当前航迹 ID 未在之前见过（无重复 active ID）
            assert(~ismember(trk.id, seen_ids), 'Frame %d duplicate active id', k);
            % 断言：当前航迹的 truth_idx 未在之前见过（无重复 truth_idx）
            assert(~ismember(double(trk.truth_idx), seen_truth), ...
                'Frame %d duplicate active truth_idx', k);
            % 记录已见的 ID 和 truth_idx
            seen_ids(end+1) = trk.id;
            seen_truth(end+1) = double(trk.truth_idx);
        end

        % ==================== 收集确认事件证据 ====================
        % 从当前帧的生命周期事件中提取确认相关信息
        % 调用 record_confirmation_hits 收集配置滑窗内的命中证据
        [confirmation_hits, frame_events] = record_confirmation_hits( ...
            confirmation_hits, diag.lifecycle_events, detList, k, params);
        % 验证生命周期事件的合法性
        validate_lifecycle_events(frame_events, params, k);
    end

    % ==================== 验证最终航迹列表的不变量 ====================
    validate_final_tracks(finalTrackList, detList, params);
end

function validate_starter_config(params)
    % 验证可配置的确认数和窗口长度，不限制为某个固定组合
    required = {'oracle_QUALIFY_NUM', 'oracle_TOLERANT_NUM'};
    for i = 1:length(required)
        assert(isfield(params, required{i}), ...
            'Oracle starter config missing field %s', required{i});
    end

    qualify_num = params.oracle_QUALIFY_NUM;
    tolerant_num = params.oracle_TOLERANT_NUM;
    valid_qualify = isnumeric(qualify_num) && isscalar(qualify_num) && ...
        isfinite(qualify_num) && qualify_num >= 1 && qualify_num == floor(qualify_num);
    valid_tolerant = isnumeric(tolerant_num) && isscalar(tolerant_num) && ...
        isfinite(tolerant_num) && tolerant_num >= 1 && tolerant_num == floor(tolerant_num);
    assert(valid_qualify && valid_tolerant && qualify_num <= tolerant_num, ...
        'Oracle starter config must use positive integers with QUALIFY_NUM <= TOLERANT_NUM');
end

% =========================================================================
% validate_slim_track — 验证快照航迹的精简字段格式
% =========================================================================
% 【功能】
%   快照中的航迹应该是轻量级的，只包含必要的显示/存储字段。
%   此函数验证快照航迹恰好包含以下字段：
%     id, type, life, truth_idx, lat, lon, P_pred, ukf
%   其中 ukf 内部也只应包含 x, P, Q 三个字段。
%   这样可以确保快照不会携带大量冗余数据（如完整的 asscPointList、
%   smoothPointList 等），节省内存和磁盘空间。
%
% 【输入】
%   trk       — 快照中的航迹结构体
%   params    — 参数结构体（用于检查 HISTORY_TRACK 常量）
%   frame_id  — 当前帧号（用于错误信息）
% =========================================================================
function validate_slim_track(trk, params, frame_id)
    % 定义期望的字段列表
    expected = {'id','type','life','truth_idx','lat','lon','P_pred','ukf'};
    % 验证字段名排序后与期望完全一致（顺序可能不同，但内容必须相同）
    assert(isequal(sort(fieldnames(trk)), sort(expected(:))), ...
        'Frame %d snapshot track is not lightweight', frame_id);
    % 验证快照中不包含 HISTORY_TRACK 类型的航迹（历史航迹不应出现在快照中）
    assert(trk.type ~= params.HISTORY_TRACK, ...
        'Frame %d snapshot contains history track', frame_id);
    % 允许固定大小的逐帧滤波诊断，不携带完整历史和内部缓存。
    expected_ukf = {'x','P','Q','Q_ema','mu','model_nis','log_likelihood'};
    assert(isequal(sort(fieldnames(trk.ukf)), sort(expected_ukf(:))), ...
        'Frame %d snapshot UKF is not lightweight', frame_id);
end

% =========================================================================
% validate_consumed_detections — 验证被消耗点迹的合法性
% =========================================================================
% 【功能】
%   确保所有被关联或起始消耗的点迹满足以下条件：
%     1. 是真实目标检测（不是杂波，aircraft_id > 0）
%     2. 是当前帧检测（frameID 等于当前帧号，不是过期帧或未来帧）
%     3. TPmatch 结果中标记的点迹确实已被 association_used_det 标记
%
% 【不变量4 的具体实现】
%   如果这里有断言失败，说明系统中存在"虚构检测"的问题——
%   即某个点迹被当作真实检测消耗了，但它实际上是杂波或过期数据。
% =========================================================================
function validate_consumed_detections(dets, diag, frame_id)
    % 找出所有被消耗的点迹索引
    used = find(diag.used_det);
    % 遍历每个被消耗的点迹
    for j = used
        % 断言：该点迹不是杂波，且 aircraft_id > 0（真实目标）
        assert(~dets(j).is_clutter && double(dets(j).aircraft_id) > 0, ...
            'Frame %d consumed clutter', frame_id);
        % 断言：该点迹的 frameID 等于当前帧号（不是过期或未来帧）
        assert(double(dets(j).frameID) == frame_id, ...
            'Frame %d consumed stale/future detection', frame_id);
    end
    % 验证 TPmatch 结果的一致性
    % TPmatch_result 的每一行是 [track_idx, det_idx]
    for r = 1:size(diag.TPmatch_result, 1)
        j = diag.TPmatch_result(r, 2);  % 取检测索引
        if j > 0
            % 断言：该检测在 association_used_det 中被标记为已使用
            assert(diag.association_used_det(j), ...
                'Frame %d TPmatch point not marked associated', frame_id);
        end
    end
end

% =========================================================================
% record_confirmation_hits — 收集确认事件的配置窗口证据
% =========================================================================
% 【功能】
%   遍历当前帧的所有生命周期事件，对于每个 'confirmed' 事件：
%     1. 回溯检查配置窗口内的检测命中情况
%     2. 验证窗口内真实检测命中数达到配置阈值
%     3. 验证当前帧确实有一次命中（确认必须由当前帧触发）
%
% 【返回值】
%   hits — 更新后的确认事件证据收集器（按 truth_id 索引）
%   events — 当前帧的生命周期事件列表（供后续验证使用）
% =========================================================================
function [hits, events] = record_confirmation_hits(hits, events, detList, frame_id, params)
    % 遍历当前帧的所有生命周期事件
    for i = 1:length(events)
        event = events(i);
        % 只处理 'confirmed' 类型的事件
        if ~strcmp(event.event, 'confirmed'), continue; end
        % 获取该确认事件对应的真值飞机 ID
        truth_id = double(event.truth_idx);
        % 如果 truth_id 超出 hits 数组范围，扩展数组
        if truth_id > length(hits), hits{truth_id} = []; end
        current_hit = false;  % 标记当前帧是否有命中
        % 计算配置滑窗的起始帧号
        start_frame = max(1, event.confirm_frame - params.oracle_TOLERANT_NUM + 1);
        hit_frames = [];  % 记录窗口内的命中帧号
        % 遍历滑窗内的每一帧
        for k = start_frame:event.confirm_frame
            dets = detList{k};  % 该帧的所有检测
            % 遍历该帧的所有检测，寻找属于该 truth_id 的真实检测
            for j = 1:length(dets)
                % 检查条件：非杂波 且 aircraft_id 匹配
                if ~dets(j).is_clutter && double(dets(j).aircraft_id) == truth_id
                    hit_frames(end+1) = k;  % 记录命中帧号
                    % 如果该命中帧就是确认帧，标记 current_hit
                    current_hit = current_hit || k == frame_id;
                    break;  % 每帧最多算一次命中
                end
            end
        end
        % 断言：窗口内命中次数达到配置的确认阈值
        assert(length(hit_frames) >= params.oracle_QUALIFY_NUM, ...
            'Frame %d confirmation lacks configured window evidence (%d/%d)', ...
            frame_id, params.oracle_QUALIFY_NUM, params.oracle_TOLERANT_NUM);
        % 断言：确认必须由当前帧命中触发
        assert(current_hit, 'Frame %d confirmation was not triggered by a current hit', frame_id);
        % 记录该飞机的确认证据
        hits{truth_id} = hit_frames;
    end
end

% =========================================================================
% validate_final_tracks — 验证最终航迹列表的不变量
% =========================================================================
% 【功能】
%   在仿真结束后，对所有最终航迹（包括活跃的和历史的）进行完整性验证：
%     1. 南阳式字段齐全（Type, Quality, asscPointList）
%     2. 别名一致性（type==Type, quality==Quality）
%     3. 关联点迹全部对应真实存在的检测（无虚构）
%     4. 历史航迹有死亡元数据，活跃航迹无死亡元数据
%     5. k_loss 死亡的航迹满足连续漏检阈值
%
% 【不变量8 的具体实现】
%   通过 detection_exists_exact 函数验证每个关联点迹确实在 detList
%   对应帧中存在，确保 Oracle 没有"凭空捏造"检测数据。
% =========================================================================
function validate_final_tracks(trackList, detList, params)
    for i = 1:length(trackList)
        trk = trackList{i};
        % 不变量：南阳式字段齐全
        assert(isfield(trk, 'Type') && isfield(trk, 'Quality'), ...
            'Final track missing Nanyang fields');
        % 不变量：别名一致性
        assert(trk.type == trk.Type && trk.quality == trk.Quality, ...
            'Final track alias mismatch');
        % 不变量：asscPointList 必须是 cell 数组
        assert(iscell(trk.asscPointList), 'Final asscPointList must be cell');
        % 遍历每条航迹的关联点迹列表
        for j = 1:length(trk.asscPointList)
            dp = trk.asscPointList{j};
            % 不变量：关联点迹必须是真实检测（非虚构、非杂波）
            assert(isstruct(dp) && ~dp.is_clutter, ...
                'Final track contains fabricated/clutter association');
            % 不变量：关联点迹的 aircraft_id 必须与航迹的 truth_idx 一致
            assert(double(dp.aircraft_id) == double(trk.truth_idx), ...
                'Final track history truth mismatch');
            % 验证关联点迹的帧号在有效范围内
            frame_id = double(dp.frameID);
            assert(frame_id >= 1 && frame_id <= length(detList), ...
                'Final track associated point frame invalid');
            % 不变量：关联点迹必须与 detList 中对应帧的实际检测完全一致
            assert(detection_exists_exact(detList{frame_id}, dp), ...
                'Final track associated point differs from generated detection');
        end
        % 区分历史航迹和活跃航迹的验证
        if trk.Type == params.HISTORY_TRACK
            % 历史航迹必须有死亡元数据
            assert(~isempty(trk.death_frame) && ~isempty(trk.death_reason), ...
                'History track missing death metadata');
            % 验证死亡原因的合理性
            if strcmp(trk.death_reason, 'k_loss')
                % k_loss 终止：连续漏检计数应 >= tracker_K_loss
                assert(trk.SuccLossPointCnt >= params.tracker_K_loss, ...
                    'k_loss death below threshold');
            else
                % 其他原因（如 truth_ended）需要验证参数设置
                assert(strcmp(trk.death_reason, 'truth_ended') && ...
                    isfield(params, 'oracle_truth_terminate_enable') && ...
                    params.oracle_truth_terminate_enable, ...
                    'Invalid truth-ended death');
            end
        else
            % 活跃航迹不应有死亡元数据
            assert(isempty(trk.death_frame) && isempty(trk.death_reason), ...
                'Active final track has death metadata');
        end
    end
end

% =========================================================================
% detection_exists_exact — 检查检测点迹是否存在于列表中
% =========================================================================
% 【功能】
%   在 detList 的某一帧中精确查找 dp 点迹是否存在。
%   使用 isequaln 进行深度比较（能处理 NaN 和 struct 嵌套）。
%   这是不变量8 的核心验证函数——确保最终航迹的 asscPointList
%   中的所有点迹都能在原始检测列表中找到对应项。
% =========================================================================
function tf = detection_exists_exact(dets, dp)
    tf = false;  % 默认不存在
    for j = 1:length(dets)
        % isequaln 是 isequal 的 NaN-safe 版本
        % 对 struct 进行递归深度比较
        if isequaln(dets(j), dp)
            tf = true;
            return;
        end
    end
end

% =========================================================================
% validate_lifecycle_events — 验证生命周期事件的合法性
% =========================================================================
% 【功能】
%   验证当前帧的所有生命周期事件（confirmed/died）的语义正确性：
%     - confirmed 事件：frameID 匹配、AsscPointCnt≥3、时间窗口≤7、计数器一致
%     - died 事件：death_reason 只能是 k_loss 或 truth_ended
%     - 无重复事件（同一 track_id 在同一帧不能有同类型事件）
% =========================================================================
function validate_lifecycle_events(events, params, frame_id)
    keys = {};  % 记录已见过的事件键（event_trackId），用于去重检查
    for i = 1:length(events)
        event = events(i);
        % 构造事件唯一键
        key = sprintf('%s_%d', event.event, event.track_id);
        % 断言：同一帧内不能有重复事件
        assert(~ismember(key, keys), 'Frame %d duplicate lifecycle event', frame_id);
        keys{end+1} = key;
        % 根据事件类型分别验证
        if strcmp(event.event, 'confirmed')
            % 确认事件：frameID 必须等于当前帧
            assert(event.frameID == frame_id && ...
                event.AsscPointCnt >= params.oracle_QUALIFY_NUM, ...
                'Frame %d invalid confirmation event', frame_id);
            % 确认帧必须在出生帧之后
            assert(event.birth_frame <= event.confirm_frame && ...
                event.confirm_frame == frame_id, ...
                'Frame %d invalid confirmation timing', frame_id);
            % 确认帧与出生帧的物理跨度不能超过配置窗口
            assert(event.confirm_frame - event.birth_frame + 1 <= ...
                params.oracle_TOLERANT_NUM, ...
                'Frame %d confirmation exceeds configured window (%d)', ...
                frame_id, params.oracle_TOLERANT_NUM);
            % TotalPointCnt 应该等于窗口跨度
            assert(event.TotalPointCnt == event.confirm_frame - event.birth_frame + 1, ...
                'Frame %d confirmation span mismatch', frame_id);
            % TotalLostPointCnt 应该等于 TotalPointCnt - AsscPointCnt
            assert(event.TotalLostPointCnt == ...
                event.TotalPointCnt - event.AsscPointCnt, ...
                'Frame %d confirmation counters mismatch', frame_id);
        elseif strcmp(event.event, 'died')
            % 死亡事件：death_reason 只能是 k_loss 或 truth_ended
            assert(any(strcmp(event.death_reason, {'k_loss', 'truth_ended'})), ...
                'Frame %d invalid death event', frame_id);
            % 如果是 truth_ended 死亡，必须确保参数中开启了真值终止
            if strcmp(event.death_reason, 'truth_ended')
                assert(isfield(params, 'oracle_truth_terminate_enable') && ...
                    params.oracle_truth_terminate_enable, ...
                    'Frame %d truth termination occurred while disabled', frame_id);
            end
        else
            % 未知事件类型，直接报错
            error('Frame %d unknown lifecycle event', frame_id);
        end
    end
end

% =========================================================================
% max_truth_id_from_events — 从事件列表中提取最大 truth_id
% =========================================================================
% 【功能】
%   遍历整个 diagList，找到所有生命周期事件中出现的最大 truth_idx。
%   这个值用于确定 confirmation_hits 数组的大小。
%   如果没有任何事件，返回 0。
% =========================================================================
function max_id = max_truth_id_from_events(diagList)
    max_id = 0;  % 初始化为 0
    for k = 1:length(diagList)
        events = diagList{k}.lifecycle_events;  % 当前帧的生命周期事件
        for i = 1:length(events)
            % 更新最大值
            max_id = max(max_id, double(events(i).truth_idx));
        end
    end
end
