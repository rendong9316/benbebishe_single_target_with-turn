% =========================================================================
% scc.m — 简单凸组合融合（SCC, Simple Convex Combination）
% =========================================================================
%
% 【功能概述】
%   在两站估计误差相互独立的假设下做信息（精度）矩阵相加：
%
%       P_fused^{-1} = P1^{-1} + P2^{-1}
%       x_fused     = P_fused * ( P1^{-1}*x1 + P2^{-1}*x2 )
%
%   当 P1 = P2 = P 时退化为等权平均：
%       x_fused = (x1 + x2) / 2,  P_fused = P / 2
%
% 【独立性假设的代价】
%   两站若共享部分量测或受到相同干扰，P_fused 会过度自信（偏小）。
%   这是第 5 章分析 BC / CI / FCI 存在意义的出发点。
%
% 【数值实现】
%   用 mldivide(A,B)（反斜杠除法的函数形式）代替显式求逆，数值更稳定。
%
% 【输入参数】
%   x1, P1 — 站 1 的状态估计（4x1）与协方差（4x4）
%   x2, P2 — 站 2 的状态估计（4x1）与协方差（4x4）
%
% 【输出参数】
%   x_fused — [4x1] 融合后的状态估计
%   P_fused — [4x4] 融合后的误差协方差
% =========================================================================

function [x_fused, P_fused] = scc(x1, P1, x2, P2)
    % 数值安全：先正则化，保证对称正定
    P1 = online_fuse.regularize_cov(P1);
    P2 = online_fuse.regularize_cov(P2);

    % 信息矩阵相加：P_fused = (P1^{-1} + P2^{-1})^{-1}
    eye4 = eye(size(P1, 1));
    P_inv = mldivide(P1, eye4) + mldivide(P2, eye4);
    P_fused = mldivide(P_inv, eye4);

    % 融合状态：信息状态之和再乘融合协方差
    x_fused = P_fused * (mldivide(P1, x1) + mldivide(P2, x2));

    % 输出再正则化，保证下游可用
    P_fused = online_fuse.regularize_cov(P_fused);
end
