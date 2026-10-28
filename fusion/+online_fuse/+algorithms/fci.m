% =========================================================================
% fci.m — 快速协方差交叉融合（FCI, Fast CI）
% =========================================================================
%
% 【功能概述】
%   FCI 是 CI 的闭式近似：不做迭代搜索，直接由两站协方差的迹解析确定权重。
%
%       tr1_inv = 1 / trace(P1),  tr2_inv = 1 / trace(P2)
%       w       = tr1_inv / (tr1_inv + tr2_inv)
%
%   精度越高的站（迹越小）获得的权重越大。
%   融合公式与 CI 完全相同：
%       P_fused^{-1} = w * P1^{-1} + (1-w) * P2^{-1}
%
% 【与 CI 的权衡】
%   CI 最小化 det(P_fused)（椭球体积），FCI 用迹近似（总不确定度），
%   FCI 融合结果通常略保守于 CI，但计算量低一个数量级。
%   由于 w 属于 (0,1)，同样满足 P_fused 在半正定序上不小于 P_SCC
%   （保守性不打折）。
%
% 【输入参数】
%   x1, P1 — 站 1 的状态估计（4x1）与协方差（4x4）
%   x2, P2 — 站 2 的状态估计（4x1）与协方差（4x4）
%
% 【输出参数】
%   x_fused — [4x1] 融合后的状态估计
%   P_fused — [4x4] 融合后的误差协方差
%   w       — 解析权重（站 1 信息矩阵的占比）
%
% 【与批处理线的一致性】
%   权重定义与 fusion/track_fusion_algorithms.m 的 fuse_fci 完全一致。
% =========================================================================

function [x_fused, P_fused, w] = fci(x1, P1, x2, P2)
    % 数值安全
    P1 = online_fuse.regularize_cov(P1);
    P2 = online_fuse.regularize_cov(P2);
    eye4 = eye(size(P1, 1));

    % 迹的倒数之比 得到解析权重
    tr1_inv = 1 / trace(P1);
    tr2_inv = 1 / trace(P2);
    w = tr1_inv / (tr1_inv + tr2_inv);

    % 与 CI 相同的融合公式
    I1 = mldivide(P1, eye4);
    I2 = mldivide(P2, eye4);
    P_inv = w * I1 + (1 - w) * I2;
    P_fused = mldivide(P_inv, eye4);
    x_fused = P_fused * (w * (I1 * x1) + (1 - w) * (I2 * x2));

    P_fused = online_fuse.regularize_cov(P_fused);
end
