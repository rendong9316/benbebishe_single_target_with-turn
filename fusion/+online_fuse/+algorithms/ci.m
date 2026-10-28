% =========================================================================
% ci.m — 协方差交叉融合（CI, Covariance Intersection）
% =========================================================================
%
% 【功能概述】
%   CI 在不假设两站估计相互独立、也不要求已知互协方差的前提下给出保守融合：
%
%       P_fused^{-1} = w * P1^{-1} + (1-w) * P2^{-1}
%       x_fused     = P_fused * ( w*P1^{-1}*x1 + (1-w)*P2^{-1}*x2 )
%
%   权重 w 通过最小化 det(P_fused) 的一维搜索确定。
%
% 【保守性说明】
%   对任意 w 属于 [0,1]，信息矩阵 w*P1^{-1} + (1-w)*P2^{-1} 在半正定序上
%   不大于 P1^{-1} + P2^{-1} = P_SCC^{-1}；
%   取逆反序得 P_fused 在半正定序上不小于 P_SCC，
%   即 trace(P_fused) 不小于 trace(P_SCC)。
%   这正是 CI 相对 SCC 更保守（不确定性更大）的原因。
%
% 【输入参数】
%   x1, P1 — 站 1 的状态估计（4x1）与协方差（4x4）
%   x2, P2 — 站 2 的状态估计（4x1）与协方差（4x4）
%
% 【输出参数】
%   x_fused — [4x1] 融合后的状态估计
%   P_fused — [4x4] 融合后的误差协方差
%   w       — 选出的权重（站 1 信息矩阵的占比）
%
% 【与批处理线的一致性】
%   权重定义与 fusion/track_fusion_algorithms.m 的 fuse_ci 完全一致，
%   保证在线版与历史批处理结果可比。
% =========================================================================

function [x_fused, P_fused, w] = ci(x1, P1, x2, P2)
    % 数值安全
    P1 = online_fuse.regularize_cov(P1);
    P2 = online_fuse.regularize_cov(P2);

    % 预计算信息矩阵，优化循环里只做线性组合
    eye4 = eye(size(P1, 1));
    I1 = mldivide(P1, eye4);
    I2 = mldivide(P2, eye4);

    % 一维搜索：最小化 det(P_fused) = 1 / det(w*I1 + (1-w)*I2)
    obj = @(ww) ci_cost(ww, I1, I2);
    w = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));

    % 用最优权重执行融合
    P_inv = w * I1 + (1 - w) * I2;
    P_fused = mldivide(P_inv, eye4);
    x_fused = P_fused * (w * (I1 * x1) + (1 - w) * (I2 * x2));

    P_fused = online_fuse.regularize_cov(P_fused);
end

% -------------------------------------------------------------------------
function cost = ci_cost(w, I1, I2)
%CI_COST CI 的优化目标：融合协方差的行列式
%   det(P_fused) = 1 / det(w*I1 + (1-w)*I2)
%   行列式越小，融合不确定度椭球体积越小。
    P_inv = w * I1 + (1 - w) * I2;
    cost = 1 / det(P_inv);
end
