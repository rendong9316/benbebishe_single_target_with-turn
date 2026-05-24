% =========================================================================
% scc_fuse.m
% Simple Convex Combination (SCC) 单帧融合
% =========================================================================
% 假设两源估计误差独立, 信息矩阵直接相加:
%   P_fused^{-1} = P1^{-1} + P2^{-1}
%   x_fused = P_fused * (P1^{-1}*x1 + P2^{-1}*x2)
% =========================================================================

function [x_fused, P_fused] = scc_fuse(x1, P1, x2, P2)
    P1 = regularize_cov(P1);
    P2 = regularize_cov(P2);

    P_fused_inv = inv(P1) + inv(P2);
    P_fused = inv(P_fused_inv);
    x_fused = P_fused * (P1 \ x1 + P2 \ x2);
    P_fused = regularize_cov(P_fused);
end
