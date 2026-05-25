% =========================================================================
% multi_track_manager.m
% =========================================================================
% 【功能概述】
%   多目标航迹管理主引擎，每帧执行完整的跟踪流水线：
%   分离活跃/历史航迹 → 批量UKF预测 → JNN全局关联 → 关联航迹更新
%   → 未关联航迹预测 → 质量管理 → 新航迹起始。
%
% 【数学原理】
%   整体框架遵循经典的多目标跟踪（MTT）流水线架构:
%
%   1. UKF预测 (Unscented Kalman Filter Predict):
%      通过Sigma点传播状态方程，得到先验状态 x_pred 和协方差 P_pred，
%      同时计算量测预测 z_pred 及其协方差 P_zz。
%      Sigma点: X_i = x + sqrt((n+lambda)*P) 的列 ±
%      P_zz = R + sum_i Wc_i * (Z_i - z_pred) * (Z_i - z_pred)'
%
%   2. JNN全局关联:
%      计算所有(活跃航迹, 未用点迹)对的马氏距离代价矩阵，
%      贪心求解全局最优一对一配对。
%
%   3. PDA更新 (Probabilistic Data Association):
%      对关联上的航迹，收集波门内所有点迹，用PDA算法计算
%      各点迹的后验关联概率 beta_i，加权融合更新状态。
%
%   4. 模糊自适应Q:
%      基于NIS (Normalized Innovation Squared) 历史，用模糊逻辑
%      在线调整过程噪声协方差矩阵Q，适应目标机动。
%
% 【输入参数】
%   trackList - cell数组，所有航迹结构体
%   tempPool  - cell数组，临时点迹候选池（用于M/N起始）
%   detList   - 结构体数组，当前帧所有检测点迹
%   ukf_tpl   - UKF模板结构体（含初始状态、协方差等）
%   params    - 参数结构体，需包含:
%               .dt_sec: 帧时间间隔（秒）
%               .gate_sigma: 波门Sigma倍数
%               .tracker_N: M/N起始滑窗帧数N
%               .tracker_M: M/N起始最少点迹数M
%               .tracker_K_loss: 暂定航迹最大漏检帧数
%               .fuzzy_window_size: 模糊自适应窗口大小
%               .use_fuzzy_adaptive: 是否启用模糊自适应Q
%   frame_id  - 当前帧编号
%
% 【输出】
%   trackList    - 更新后的航迹列表
%   tempPool     - 更新后的临时候选池
%   trackSnapshot - 当前帧航迹快照（用于评估和可视化）
%
% 【调用关系】
%   主循环调用
%   子调用: find_active, jnn_association, ukf_predict_step,
%           ukf_measurement_model, ukf_pda_update, ukf_fuzzy_adapt,
%           manage_track_quality, track_starter_mofn, cleanup_stale
% =========================================================================

function [trackList, tempPool, trackSnapshot] = multi_track_manager(...
        trackList, tempPool, detList, ukf_tpl, params, frame_id)

    TYPE_HISTORY = 7;
    % 创建当前帧的快照结构体
    trackSnapshot = struct('frameID', frame_id, 'trackList', {{}});

    % =====================================================================
    % 特殊情况：当前帧无检测点迹
    % 所有活跃航迹执行纯预测（只做UKF预测，不更新），质量衰减
    % =====================================================================
    if isempty(detList)
        for t = 1:length(trackList)
            trk = trackList{t};
            if trk.type == TYPE_HISTORY, continue; end  % 跳过历史航迹

            % UKF预测步骤（无观测更新）
            trk.ukf.dt = params.dt_sec;
            [x_pred, P_pred, ~, trk.ukf] = ukf_predict_step(trk.ukf);

            % 将预测值写回UKF状态
            trk.ukf.x = x_pred;
            trk.ukf.P = P_pred;

            % 更新航迹元数据
            trk.missed = trk.missed + 1;  % 漏检计数+1
            trk.life = trk.life + 1;      % 生命周期+1
            trk.lat = x_pred(3);          % 更新纬度（状态向量第3分量）
            trk.lon = x_pred(1);          % 更新经度（状态向量第1分量）
            trk.assoc_det = [];           % 无关联量测

            trackList{t} = trk;
        end
        % 运行质量状态机（纯预测也会影响质量）
        active_idx = find_active(trackList);
        trackList = track_management.manage_track_quality(trackList, active_idx, params, frame_id);
        trackSnapshot.trackList = trackList;
        return;
    end

    % ---- Step 1: 分离活跃航迹（type≠7即HISTORY） ----
    active_idx = find_active(trackList);

    % ---- Step 2: 无活跃航迹 → 直接尝试M/N起始 ----
    if isempty(active_idx)
        [trackList, tempPool] = track_starter_mofn(trackList, tempPool, ...
            detList, ukf_tpl, params, frame_id);
        trackSnapshot.trackList = trackList;
        return;
    end

    % ---- Step 3: 批量UKF预测 ----
    % 对所有活跃航迹执行预测步骤，计算先验估计和量测预测
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};

        % 设置时间步长
        trk.ukf.dt = params.dt_sec;

        % UKF预测：传播状态和协方差
        %   x_pred: 预测状态向量
        %   P_pred: 预测状态协方差
        %   X_pred: Sigma点矩阵
        [x_pred, P_pred, X_pred, trk.ukf] = ukf_predict_step(trk.ukf);

        % 计算预测状态下的量测预测
        z_pred = ukf_measurement_model(trk.ukf, x_pred);

        % 通过Sigma点传播计算量测预测协方差 P_zz
        % 这个过程考虑了状态不确定性到量测空间的传播
        Z_pred = zeros(trk.ukf.m, 2*trk.ukf.n + 1);
        for s = 1:(2*trk.ukf.n + 1)
            Z_pred(:, s) = ukf_measurement_model(trk.ukf, X_pred(:, s));
        end

        % P_zz = R + sum_i Wc(i) * (Z_i - z_pred) * (Z_i - z_pred)'
        % 其中 R 为量测噪声协方差，Wc 为Sigma点权重
        P_zz = trk.ukf.R;
        for s = 1:(2*trk.ukf.n + 1)
            dz = Z_pred(:, s) - z_pred;
            P_zz = P_zz + trk.ukf.Wc(s) * (dz * dz');
        end

        % 安全回退：若P_zz数值异常（NaN），退化为仅用R
        if any(isnan(P_zz(:)))
            P_zz = trk.ukf.R;
        end

        % 存储预测结果到航迹结构体，供后续关联使用
        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        trk.assoc_det = [];  % 初始化关联状态为空

        trackList{t} = trk;
    end

    % ---- Step 4: JNN全局点迹-航迹关联 ----
    % 调用track_management.jnn_association计算全局最佳一对一配对
    assoc_pairs = track_management.jnn_association(trackList, active_idx, detList, params);

    % 标记点迹使用状态和航迹关联状态
    point_used = false(1, length(detList));
    track_has_assoc = false(1, length(active_idx));
    for p = 1:size(assoc_pairs, 1)
        point_used(assoc_pairs(p, 2)) = true;
        [~, loc] = ismember(assoc_pairs(p, 1), active_idx);
        if loc > 0, track_has_assoc(loc) = true; end
    end

    % ---- Step 5: 更新关联成功的航迹（UKF + PDA + 模糊自适应Q） ----
    for p = 1:size(assoc_pairs, 1)
        t = assoc_pairs(p, 1);   % 航迹全局索引
        d = assoc_pairs(p, 2);   % 点迹索引
        trk = trackList{t};
        det = detList(d);

        % ---- 收集波门内所有点迹（用于PDA多假设更新） ----
        % 初始集合包含关联到的那个点迹
        dets_in_gate = {det};
        P_zz_2d = trk.P_zz(1:2, 1:2);
        gate_threshold = params.gate_sigma^2 * 2;

        % 遍历其他未使用的点迹，收集波门内的
        for j = 1:length(detList)
            if j == d || point_used(j), continue; end  % 跳过已用点迹
            dp = detList(j);
            if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end

            % 计算马氏距离判断是否在波门内
            z_m = [dp.drange; dp.daz];
            innov = z_m - trk.z_pred(1:2);
            if innov(2) > 180, innov(2) = innov(2) - 360;
            elseif innov(2) < -180, innov(2) = innov(2) + 360; end
            if innov' * (P_zz_2d \ innov) < gate_threshold
                dets_in_gate{end+1} = dp;  % 加入门内点迹集合
            end
        end

        % ---- PDA更新 ----
        % ukf_pda_update 执行概率数据关联UKF更新:
        %   1. 计算各门内点迹的后验关联概率 beta_i
        %   2. 对各点迹计算Kalman更新，加权融合
        %   3. 返回更新后的UKF状态
        [~, ~, trk.ukf, best, nis_val] = ukf_pda_update(trk.ukf, dets_in_gate, ...
            trk.z_pred, trk.Z_pred, trk.X_pred, trk.x_pred, trk.P_pred, ...
            trk.P_zz, params);

        % ---- 更新航迹元数据 ----
        trk.lat = trk.ukf.x(3);
        trk.lon = trk.ukf.x(1);
        trk.missed = 0;              % 关联成功，清零漏检计数
        trk.life = trk.life + 1;
        trk.assoc_det = best;        % 记录最佳关联量测

        % ---- 维护NIS历史（用于模糊自适应Q） ----
        trk.nis_history(end+1) = nis_val;
        if length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history(1) = [];  % 滑动窗口，移除最旧值
        end

        % ---- 模糊自适应Q ----
        % 基于NIS历史，用模糊逻辑调整过程噪声协方差Q
        % 原理: NIS持续偏高 → 目标可能有未建模机动 → 增大Q
        %       NIS正常偏低 → 模型匹配良好 → 保持或减小Q
        if params.use_fuzzy_adaptive
            trk.ukf = ukf_fuzzy_adapt(trk.ukf, trk.nis_history, trk.life, params);
        end

        trackList{t} = trk;
    end

    % ---- Step 6: 更新未关联航迹（纯预测，无观测更新） ----
    for i = 1:length(active_idx)
        if track_has_assoc(i), continue; end  % 已关联的跳过
        t = active_idx(i);
        trk = trackList{t};

        % 仅使用预测值更新状态（不融合观测）
        trk.ukf.x = trk.x_pred;
        trk.ukf.P = trk.P_pred;
        trk.missed = trk.missed + 1;   % 漏检计数+1
        trk.life = trk.life + 1;
        trk.lat = trk.ukf.x(3);
        trk.lon = trk.ukf.x(1);
        trk.assoc_det = [];

        trackList{t} = trk;
    end

    % ---- Step 7: 航迹质量状态机 ----
    % 根据本帧关联结果更新各航迹的状态转移
    trackList = track_management.manage_track_quality(trackList, active_idx, params, frame_id);

    % ---- Step 8: 从未关联点迹起始新航迹（M/N逻辑） ----
    unused_dets = detList(~point_used);
    if ~isempty(unused_dets)
        % 用剩余点迹尝试M/N航迹起始
        [trackList, tempPool] = track_starter_mofn(trackList, tempPool, ...
            unused_dets, ukf_tpl, params, frame_id);
    else
        % 无剩余点迹时仍然需要清理过期候选
        tempPool = cleanup_stale(tempPool, frame_id, params.tracker_N);
    end

    trackSnapshot.trackList = trackList;
end


% =========================================================================
% 辅助函数: find_active
% 找出所有非HISTORY状态的航迹索引
% =========================================================================
function idx = find_active(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7  % type 7 = HISTORY
            idx(end+1) = t;
        end
    end
end


% =========================================================================
% 辅助函数: cleanup_stale
% 清理tempPool中超过N帧未更新的过期候选
% =========================================================================
function tempPool = cleanup_stale(tempPool, current_frame, N)
    keep = true(1, length(tempPool));
    for c = 1:length(tempPool)
        if current_frame - tempPool{c}.lastFrame > N
            keep(c) = false;
        end
    end
    tempPool = tempPool(keep);
end
