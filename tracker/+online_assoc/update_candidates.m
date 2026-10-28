function [cands, S] = update_candidates(S, local1, local2, k, params)
%UPDATE_CANDIDATES 逐帧推进关联策略，产出候选对
%
%   [cands, S] = online_assoc.update_candidates(S, local1, local2, k, params)
%
%   local1 / local2 — 本帧两站的局部航迹数组，元素字段：
%       id, x(4x1), P(4x4), truth_idx, life
%   cands — 候选对结构体数组，字段：
%       r1_id, r2_id   两站航迹 ID
%       score         越大越可信
%       dist_km       本帧 Haversine 距离
%       d2            本帧马氏距离平方
%       vdiff         本帧速度差模 (km/s)
%       vtol          本帧速度容差 (km/s)
%       n_coexist     截至本帧的共现帧数
%
%   只返回"通过本方法自身门限"的候选；1v1 指派与 M/N 确认由融合中心做。

    switch S.method
        case 'nn'
            [cands, S] = online_strategies.nn(S, local1, local2, k, params);
        case 'sequential'
            [cands, S] = online_strategies.sequential(S, local1, local2, k, params);
        case 'mewf'
            [cands, S] = online_strategies.mewf(S, local1, local2, k, params);
        otherwise   % dualgate
            [cands, S] = online_strategies.dualgate(S, local1, local2, k, params);
    end
end
