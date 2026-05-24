% =========================================================================
% ci_fuse.m
% Covariance Intersection (CI) 单帧融合
% =========================================================================
% 无需互协方差, 保守融合:
%   P_fused^{-1} = w*P1^{-1} + (1-w)*P2^{-1}
%   x_fused = P_fused * (w*P1^{-1}*x1 + (1-w)*P2^{-1}*x2)
% w 通过 fminbnd 最小化 det(P_fused) 求取
% =========================================================================

function [x_fused, P_fused, w_opt] = ci_fuse(x1, P1, x2, P2)
    P1 = regularize_cov(P1);
    P2 = regularize_cov(P2);

    P1_inv = inv(P1);
    P2_inv = inv(P2);

    % 优化w以最小化det(P_fused)
    obj = @(w) ci_cost(w, P1_inv, P2_inv);
    w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));

    P_fused_inv = w_opt * P1_inv + (1 - w_opt) * P2_inv;
    P_fused = inv(P_fused_inv);
    x_fused = P_fused * (w_opt * P1_inv * x1 + (1 - w_opt) * P2_inv * x2);
    P_fused = regularize_cov(P_fused);
end

function cost = ci_cost(w, P1_inv, P2_inv)
    P_inv = w * P1_inv + (1 - w) * P2_inv;
    cost = 1 / det(P_inv);  % = det(P), 越小越好
end
