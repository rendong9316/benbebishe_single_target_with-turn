function decision = dual_threshold_decide(frames, distances_km, params)
% DUAL_THRESHOLD_DECIDE 对连续有效帧执行距离和方差双门限判定。
%
% 输入:
%   frames        — 帧号数组（可能未排序）
%   distances_km  — 对应帧的距离值（km）
%   params        — 含 dualgate_M, dualgate_T1_km, dualgate_var_km2 等参数
%
% 输出:
%   decision      — struct，含 accepted, reason, accepted_window 等字段
%
% 算法流程:
%   1. 帧号排序（如未排序）
%   2. 逐帧扫描：距离 < T1 且帧号连续 → 连续计数 +1
%   3. 连续计数 >= M 时：计算最近 M 帧距离方差
%   4. 方差 < 门限 → 判定为关联
%   5. 附加共存帧数校验和拒绝原因诊断

    % 将 frames 强制转为行向量，方便后续索引操作
    frames = frames(:)';

    % 如果帧号未排序（存在递减），先按帧号升序排列
    % 同时同步打乱 distances_km 的顺序，保持帧号与距离的对应关系
    if any(diff(frames) < 0)
        [frames, order] = sort(frames);
        distances_km = distances_km(order);
    end

    % 从 params 中读取第二门限参数：连续帧数要求
    M = params.dualgate_M;
    % 第一门限：距离阈值（km）
    T1 = params.dualgate_T1_km;
    % 方差门限：连续 M 帧距离的方差上限
    var_gate = params.dualgate_var_km2;
    % 共存帧数下限：取双门限共存阈值和片段交叉最小重叠帧数的较大值
    coexist_gate = max(params.dualgate_coexist_thresh, params.tracklet_cross_min_overlap_frames);

    % 初始化变量：
    %   run_length — 当前连续满足条件的帧数
    %   best_run   — 历史上达到的最大连续长度
    %   accepted   — 是否通过双门限判定
    %   accepted_window — 通过判定时对应的帧号数组
    run_length = 0; best_run = 0; accepted = false; accepted_window = [];

    % 逐帧扫描：检查距离是否低于 T1 且帧号连续
    for i = 1:numel(frames)
        % 条件1: 距离低于第一门限
        % 条件2: 是第一帧或帧号连续（frames[i] == frames[i-1] + 1）
        if distances_km(i) < T1 && (i == 1 || frames(i) == frames(i-1) + 1)
            % 距离合格且帧号连续 → 连续计数 +1
            run_length = run_length + 1;
        elseif distances_km(i) < T1
            % 距离合格但帧号不连续 → 连续计数重置为 1（当前帧重新开始）
            run_length = 1;
        else
            % 距离不合格 → 连续计数清零
            run_length = 0;
        end
        % 更新历史最大连续长度
        best_run = max(best_run, run_length);
        % 当连续长度达到 M 时，进入方差校验
        if run_length >= M
            % 取最近 M 帧的索引
            idx = (i-M+1):i;
            % 计算这 M 帧距离的方差（biased estimator, n 分母）
            % 方差小说明距离稳定，判定为同一目标的连续片段
            if var(distances_km(idx), 1) < var_gate
                accepted = true;
                accepted_window = frames(idx);
                break;
            end
        end
    end

    % 判定结果分析：根据各种条件给出拒绝原因
    if numel(frames) < coexist_gate
        % 总帧数不足共存阈值 → 拒绝
        accepted = false; reason = 'INSUFFICIENT_COEXISTENCE';
    elseif accepted
        % 通过了双门限判定
        reason = 'ACCEPTED';
    elseif best_run < M
        % 最大连续长度未达到 M → 距离门限或帧连续性不满足
        reason = 'DISTANCE_OR_CONTINUITY_GATE';
    else
        % 连续长度够了但方差太大 → 距离波动剧烈，判定不关联
        reason = 'VARIANCE_GATE';
    end

    % 组装返回结构体，包含判定结果和诊断信息
    decision = struct('accepted', accepted, 'reason', reason, 'coexist_frames', numel(frames), ...
        'best_consecutive_run', best_run, 'accepted_window', accepted_window, ...
        'mean_distance_km', mean_or_inf(distances_km), 'distance_variance_km2', var_or_inf(distances_km));
end

function value = mean_or_inf(x)
    % 安全求均值：空数组返回 inf，非空返回 mean(x)
    if isempty(x), value = inf; else, value = mean(x); end
end

function value = var_or_inf(x)
    % 安全求方差：空数组返回 inf，非空返回 var(x, 1)（biased estimator）
    if isempty(x), value = inf; else, value = var(x, 1); end
end
