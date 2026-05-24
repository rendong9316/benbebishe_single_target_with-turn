% =========================================================================
% bc_fuse.m
% Bar-Shalom-Campo (BC) 单帧融合
% =========================================================================
% 考虑互协方差 P12, 精确融合:
%   S = P1 + P2 - P12 - P12'
%   x_fused = x1 + (P1 - P12) * inv(S) * (x2 - x1)
%   P_fused = P1 - (P1 - P12) * inv(S) * (P1 - P12')
%
% 互协方差递推 (在融合循环中维护):
%   预测: P12_pred = F * P12 * F' + Q
%   更新: P12_new = (I-K1*H1) * P12_pred * (I-K2*H2)'
%        ≈ P1_new * inv(P1_pred) * P12_pred * inv(P2_pred)' * P2_new'
% =========================================================================

function [x_fused, P_fused] = bc_fuse(x1, P1, x2, P2, P12)
    P1 = regularize_cov(P1);
    P2 = regularize_cov(P2);

    if nargin < 5 || isempty(P12)
        P12 = zeros(size(P1));
    end

    S = P1 + P2 - P12 - P12';
    S = regularize_cov(S);

    K_bc = (P1 - P12) / S;

    x_fused = x1 + K_bc * (x2 - x1);
    P_fused = P1 - K_bc * (P1 - P12');
    P_fused = regularize_cov(P_fused);
end
