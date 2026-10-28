function S = init_strategy(method, params)
%INIT_STRATEGY 初始化在线跨站关联策略
%
%   S = online_assoc.init_strategy(method, params)
%
%   method ∈ {'dualgate','nn','sequential','mewf'}
%
%   返回的状态 S 跨帧持有，每帧由 update_candidates 推进。
%   各方法的差别只在"候选怎么产生"：
%     dualgate   — 三判据（位置 + 马氏 + 速度一致性），逐帧硬门限
%     nn         — 朴素最近邻，无门限（基线）
%     sequential — 共现帧马氏通过率达标（基线）
%     mewf       — 四项证据滚动累积，综合分判决
%
%   策略只负责产候选，不做 1v1 指派，也不做 M/N 确认——
%   那两件事由融合中心（tracker/central_track_manager.m）统一执行。

    method = lower(char(method));
    valid = {'dualgate', 'nn', 'sequential', 'mewf'};
    if ~any(strcmp(method, valid))
        error('online_assoc:badMethod', ...
            '未知关联策略 "%s"，可选：%s', method, strjoin(valid, ', '));
    end

    S = struct();
    S.method  = method;
    S.params  = params;
    S.acc     = online_assoc.empty_acc();
end
