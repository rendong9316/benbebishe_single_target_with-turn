% =========================================================================
% bc.m — Bar-Shalom-Campo 融合（BC，考虑互协方差）
% =========================================================================
%
% 【功能概述】
%   在两站估计之间存在相关性时，用互协方差 P12 修正融合增益：
%
%       S        = P1 + P2 - P12 - P12'
%       K_bc     = (P1 - P12) * S^{-1}
%       x_fused  = x1 + K_bc * (x2 - x1)
%       P_fused  = P1 - K_bc * (P1 - P12')
%
%   与 SCC 的差别完全来自 P12：
%     - P12 = 0 时 BC 精确退化为 SCC（x 为算术平均，P 为 P/2）；
%     - P12 非零时 BC 才真正区别于 SCC。
%   因此"P12 是否被真正递推出来"是 BC 有无意义的关键，
%   也是批处理线退化为 SCC 的根因所在。
%
% 【输入参数】
%   x1, P1 — 站 1 的状态估计（4x1）与协方差（4x4）
%   x2, P2 — 站 2 的状态估计（4x1）与协方差（4x4）
%   P12    — [4x4] 两站估计误差的互协方差（可为空 → 视作 0）
%
% 【输出参数】
%   x_fused — [4x1] 融合后的状态估计
%   P_fused — [4x4] 融合后的误差协方差
%
% 【参考文献】
%   Bar-Shalom, Y., Campo, L., "The Effect of the Common Process Noise on the
%   Two-Sensor Fused-Track Covariance", IEEE T-AES, 1986.
% =========================================================================

function [x_fused, P_fused] = bc(x1, P1, x2, P2, P12)
    % 数值安全
    P1 = online_fuse.regularize_cov(P1);
    P2 = online_fuse.regularize_cov(P2);
    if isempty(P12)
        P12 = zeros(size(P1));
    end

    % 新息协方差 S = P1 + P2 - P12 - P12'
    % 加上 P12' 是为了强制 S 对称（P12 数值上可能轻微不对称）
    S = P1 + P2 - P12 - P12';
    S = online_fuse.regularize_cov(S);

    % BC 融合增益：右除等价于 (P1 - P12) * inv(S)
    K_bc = (P1 - P12) / S;

    % 融合状态：x1 + K * 新息
    x_fused = x1 + K_bc * (x2 - x1);

    % 融合协方差：P1 - K * (P1 - P12')
    P_fused = P1 - K_bc * (P1 - P12');
    P_fused = online_fuse.regularize_cov(P_fused);
end
