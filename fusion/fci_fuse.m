% =========================================================================
% fci_fuse.m
% Fast Covariance Intersection (FCI) 单帧融合
% =========================================================================
% 无需迭代优化, 用迹的倒数做权重:
%   w = tr(P1)^{-1} / (tr(P1)^{-1} + tr(P2)^{-1})
%   P_fused^{-1} = w*P1^{-1} + (1-w)*P2^{-1}
%   x_fused = P_fused * (w*P1^{-1}*x1 + (1-w)*P2^{-1}*x2)
% =========================================================================

function [x_fused, P_fused, w_fci] = fci_fuse(x1, P1, x2, P2)
    P1 = regularize_cov(P1);
    P2 = regularize_cov(P2);

    tr1_inv = 1 / trace(P1);
    tr2_inv = 1 / trace(P2);
    w_fci = tr1_inv / (tr1_inv + tr2_inv);

    P1_inv = inv(P1);
    P2_inv = inv(P2);

    P_fused_inv = w_fci * P1_inv + (1 - w_fci) * P2_inv;
    P_fused = inv(P_fused_inv);
    x_fused = P_fused * (w_fci * P1_inv * x1 + (1 - w_fci) * P2_inv * x2);
    P_fused = regularize_cov(P_fused);
end
