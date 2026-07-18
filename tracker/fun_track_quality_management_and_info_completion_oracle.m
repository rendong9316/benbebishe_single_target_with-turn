function track = fun_track_quality_management_and_info_completion_oracle(track, asscPoint, sysPara, params, frame_id)
    % 航迹质量管理和状态转移 — 南阳式 4 态状态机
    %
    % 航迹类型（type 字段值）：
    %   1 = RELIABLE_TRACK   (可靠航迹)
    %   2 = MAINTAIN_TRACK   (维持航迹，质量下降但未终止)
    %   6 = TEMPORARY_TRACK  (临时航迹，起始中)
    %   7 = HISTORY_TRACK    (历史航迹，已终止)
    %
    % 状态转移规则：
    %   RELIABLE → MAINTAIN:  关联成功但 Quality 降至低于维持阈值
    %   MAINTAIN → RELIABLE:  重新获得关联且 Quality 恢复到确认阈值以上
    %   TEMPORARY → RELIABLE: 起始确认
    %   任何 → HISTORY:       SuccLossPointCnt >= K_loss (连续漏检)
    %
    % 质量管理机制：
    %   关联成功 → Quality+1 (上限 max_quality)
    %   关联失败 → Quality-1, SuccLossPointCnt+1

    % ---- 初始化缺失字段（防御性编程） ----
    % 确保航迹结构体中包含所有必需的质量管理字段，
    % 防止因字段缺失导致后续逻辑出错
    if ~isfield(track, 'TotalPointCnt'), track.TotalPointCnt = 0; end
    if ~isfield(track, 'AsscPointCnt'), track.AsscPointCnt = 0; end
    if ~isfield(track, 'TotalLostPointCnt'), track.TotalLostPointCnt = 0; end
    if ~isfield(track, 'SuccLossPointCnt'), track.SuccLossPointCnt = 0; end
    % Quality 字段可能有 Quality 或 quality 两种命名（兼容旧代码）
    % 如果 Quality 字段不存在，从 quality 读取或取默认确认阈值
    if ~isfield(track, 'Quality'), track.Quality = get_field_or_default(track, 'quality', params.oracle_confirm_quality); end
    if ~isfield(track, 'life'), track.life = 0; end
    if ~isfield(track, 'death_reason'), track.death_reason = ''; end
    if ~isfield(track, 'death_frame'), track.death_frame = []; end

    % ---- 统计更新 ----
    % 判断本帧是否有关联点迹：asscPoint 非空表示有关联
    has_assoc = ~isempty(asscPoint);
    % 总存活帧数和航迹寿命各 +1（无论是否关联都计数）
    track.TotalPointCnt = track.TotalPointCnt + 1;
    track.life = track.life + 1;

    if has_assoc
        % 有关联：关联计数+1，连续漏检清零，Quality+1（上限 params.oracle_max_quality）
        % updateFlag=1 表示本帧航迹已更新，可用于后续诊断
        track.AsscPointCnt = track.AsscPointCnt + 1;
        track.SuccLossPointCnt = 0;
        track.updateFlag = 1;
        track.Quality = min(track.Quality + 1, params.oracle_max_quality);
    else
        % 无关联：总漏检数+1，连续漏检数+1，Quality 递减
        % 递减幅度由 params.oracle_loss_quality_penalty 控制
        track.TotalLostPointCnt = track.TotalLostPointCnt + 1;
        track.SuccLossPointCnt = track.SuccLossPointCnt + 1;
        track.updateFlag = 0;
        % Quality 不能低于 0，使用 max 钳位
        track.Quality = max(track.Quality - params.oracle_loss_quality_penalty, 0);
    end

    % ---- 状态转移 ----
    % 记录转移前的类型，用于后续检测从活跃转为历史的时刻
    old_type = track.Type;
    % 调用 transition_type 函数，根据当前状态和关联情况决定新类型
    track.Type = transition_type(track, has_assoc, params);

    % 记录从活跃转为历史的时刻（死亡事件）
    % 只有当类型从非 HISTORY 变为 HISTORY 时才记录
    if old_type ~= params.HISTORY_TRACK && track.Type == params.HISTORY_TRACK
        if isempty(track.death_frame)
            track.death_frame = frame_id;
        end
        if isempty(track.death_reason)
            track.death_reason = 'k_loss';  % 连续 K_loss 帧未关联
        end
    end

    % 同步别名字段（Quality/type/quality 双写，兼容新旧代码）
    % type 是 Type 的小写别名，missed 是 SuccLossPointCnt 的别名
    track.type = track.Type;
    track.quality = track.Quality;
    track.missed = track.SuccLossPointCnt;

    % ---- 更新航迹位置字段（从 UKF 状态提取） ----
    % 将 UKF 估计的经纬度同步到航迹结构体顶层
    % 同时生成平滑点（smoothPoint）并追加到列表中
    % UKF 状态 x 的结构：[lon, vel_east, lat, vel_north, ...]
    if isfield(track, 'ukf') && isfield(track.ukf, 'x') && numel(track.ukf.x) >= 3
        track.lat = track.ukf.x(3);  % 纬度是第 3 个状态分量
        track.lon = track.ukf.x(1);  % 经度是第 1 个状态分量
        outPoint = struct('frameID', frame_id, 'lon', track.lon, 'lat', track.lat);
        if ~isfield(track, 'smoothPointList') || isempty(track.smoothPointList)
            track.smoothPointList = {outPoint};
        else
            track.smoothPointList{end+1} = outPoint;
        end
        track.outputPointList = track.smoothPointList;
    end
    track.isNewTrack = 0;  % 起始完成后清除新航迹标记（下一帧不再视为新航迹）
end

function type = transition_type(track, has_assoc, params)
    % 航迹状态转移逻辑
    % 输入: track.Type (当前类型), has_assoc (本帧是否有关联)
    % 输出: type (转移后的类型)
    type = track.Type;

    % HISTORY 航迹不可逆：一旦转入历史状态就不再改变
    % 这是防呆设计，避免已终止航迹被意外复活
    if type == params.HISTORY_TRACK
        return;
    end

    % 连续漏检超过 K_loss → 转入 HISTORY
    % 这是最优先的判断，因为连续漏检意味着航迹已不可靠
    if track.SuccLossPointCnt >= params.tracker_K_loss
        type = params.HISTORY_TRACK;
        return;
    end

    % 质量驱动的中间态转移：
    %   RELIABLE → MAINTAIN:  Quality 降到维持阈值以下
    %   MAINTAIN → RELIABLE: 重新获得关联且 Quality 恢复到确认阈值以上
    %   TEMPORARY → RELIABLE: 起始确认（Quality 达到确认阈值）
    % 注意：这三个判断是互斥的，用 elseif 链实现
    if type == params.RELIABLE_TRACK && track.Quality < params.oracle_maintain_quality
        % 可靠航迹质量下降到维持阈值以下 → 降级为维持航迹
        % 维持航迹仍可跟踪，但优先级降低
        type = params.MAINTAIN_TRACK;
    elseif type == params.MAINTAIN_TRACK && has_assoc && track.Quality >= params.oracle_confirm_quality
        % 维持航迹重新获得关联且质量恢复到确认阈值以上 → 升级为可靠航迹
        type = params.RELIABLE_TRACK;
    elseif type == params.TEMPORARY_TRACK && has_assoc && track.Quality >= params.oracle_confirm_quality
        % 临时航迹（起始中）质量达到确认阈值 → 确认为可靠航迹
        type = params.RELIABLE_TRACK;
    end
end

function v = get_field_or_default(s, name, default_value)
    % 安全读取结构体字段：如果字段存在则返回值，否则返回默认值
    % 用于兼容新旧代码中 Quality/quality 字段名不一致的问题
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
