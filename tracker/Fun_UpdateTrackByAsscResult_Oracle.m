function trackList = Fun_UpdateTrackByAsscResult_Oracle(trackList, pointList, TPmatch_result, params, frame_id)
    % Oracle 模式航迹更新（按关联结果）
    %
    % 根据 PointTrackAssociation_Oracle 的关联结果，对每条航迹执行
    % UKF 更新或纯预测。有关联点迹的航迹执行 Kalman 校正，无关联的
    % 航迹保留预测状态。同时更新 NIS 历史、关联点迹列表和航迹质量。
    %
    % 输入：
    %   trackList      — 航迹列表（预测后状态）
    %   pointList      — 本帧所有检测点迹
    %   TPmatch_result — [track_idx, point_idx] 关联结果
    %   params         — 仿真参数
    %   frame_id       — 当前帧编号
    % 输出：
    %   trackList      — 更新后的航迹列表

    % 遍历关联结果，对每条航迹执行更新或纯预测
    % TPmatch_result 每行是一个关联对：[航迹索引, 点迹索引]
    for r = 1:size(TPmatch_result, 1)
        ti = TPmatch_result(r, 1);  % 航迹索引（TPmatch_result 第 1 列）
        pi = TPmatch_result(r, 2);  % 点迹索引（0=未关联）
        if ti < 1 || ti > length(trackList)
            continue;  % 跳过无效的航迹索引
        end
        trk = trackList{ti};

        % 确保 nis_history 和 asscPointList 字段存在
        % 防御性编程：防止新航迹或异常航迹缺少这些字段
        if ~isfield(trk, 'nis_history') || isempty(trk.nis_history)
            trk.nis_history = [];
        end
        if ~isfield(trk, 'asscPointList') || isempty(trk.asscPointList)
            trk.asscPointList = {};
        end

        if pi > 0
            % ---- 有关联点迹：计算新息并执行 Kalman 更新 ----
            dp = pointList(pi);
            % 新息 = 校准后量测 - 预测量测
            %   drange: 群距离新息（米）= 实测距离 - 预测距离
            %   daz:    方位角新息（度，经 wrap 到 [-180, 180]）
            %   radial_vel: 多普勒新息（m/s）= 实测径向速度 - 预测径向速度
            % 三个新息组成 3x1 向量，对应 UKF 的量测维度
            innov = [dp.drange - trk.z_pred(1); wrap_angle_oracle(dp.daz - trk.z_pred(2)); dp.radial_vel_meas - trk.z_pred(3)];

            % 调用 UKF update：根据滤波器类型自动路由到 IMM/自适应/基础 UKF
            % innov_w 非空 → 执行 Kalman 更新，用新息修正预测状态
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, innov);

            % 记录关联点迹和 NIS（归一化新息平方）
            % NIS = innov' * P_zz^{-1} * innov，用于检测滤波器是否一致
            % 如果 NIS 过大，说明量测与预测不一致，可能存在异常
            trk.assoc_det = dp;
            trk.asscPointList{end+1} = dp;
            trk.nis_history(end+1) = safe_nis(innov, trk.P_zz);

            % 航迹质量管理和状态转移（含 Quality 增减、type 转换等）
            trk = fun_track_quality_management_and_info_completion_oracle(trk, dp, params, params, frame_id);
        else
            % ---- 未关联点迹：纯预测帧，保留预测状态 ----
            % ukf_dispatch('update', [], ...) 内部检测到 innov_w 为空时，
            % 直接将状态设为预测值，不执行 Kalman 校正
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
            trk.assoc_det = [];
            trk.nis_history(end+1) = NaN;  % 未关联帧 NIS 标记为 NaN

            % 航迹质量管理和状态转移（传入 [] 表示无关联点迹）
            trk = fun_track_quality_management_and_info_completion_oracle(trk, [], params, params, frame_id);
        end

        % 滑动窗口截断：只保留最近 fuzzy_window_size 个 NIS 值
        % 用于自适应 Q 的模糊推理，避免历史数据淹没当前状态
        % 过长的 NIS 历史会增加计算负担且降低对近期状态的敏感性
        if isfield(params, 'fuzzy_window_size') && length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history = trk.nis_history(end-params.fuzzy_window_size+1:end);
        end
        trackList{ti} = trk;
    end
end

function a = wrap_angle_oracle(a)
    % 角度归一化：将角度差 wrap 到 [-180, 180] 范围
    % 方位角是周期性量，-180 度和 180 度等价，需要进行 wrap 处理
    % 避免角度跳变导致新息计算错误
    % 使用 while 循环而非 mod，因为方位角偏差可能远大于 360 度
    while a > 180
        a = a - 360;
    end
    while a < -180
        a = a + 360;
    end
end

function nis = safe_nis(innov, P_zz)
    % 安全计算 NIS（归一化新息平方），出错时返回 NaN
    % NIS = innov' * P_zz^{-1} * innov
    % P_zz 是量测自协方差矩阵，可能奇异或病态，需要 try-catch 保护
    try
        nis = innov' * (P_zz \ innov);
    catch
        nis = NaN;
    end
end
