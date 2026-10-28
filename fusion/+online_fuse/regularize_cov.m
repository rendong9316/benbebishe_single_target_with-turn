% =========================================================================
% regularize_cov.m — 协方差正则化（强制对称正定）
% =========================================================================
%
% 【功能概述】
%   对任意协方差矩阵 P 执行正则化，保证输出严格对称正定。
%   这是本包内部的独立实现（与 fusion/regularize_cov.m 同算法），
%   使 fusion/+online_fuse/** 自包含，不依赖批处理线的文件是否被归档。
%
% 【算法步骤】
%   1. 强制对称化：P_sym = (P + P')/2
%   2. 特征值分解：P_sym = V * diag(d) * V'
%   3. 双阈值裁剪：d_clip = max(d, max(min_eig, 1e-6*max(d)))
%   4. 重构：P_reg = V * diag(d_clip) * V'
%
% 【输入参数】
%   P       — [NxN] 待正则化的协方差矩阵
%   min_eig — [可选] 绝对特征值下限，默认 1e-12
%
% 【输出参数】
%   P_reg   — [NxN] 正则化后的协方差矩阵
% =========================================================================

function P_reg = regularize_cov(P, min_eig)
    if nargin < 2
        min_eig = 1e-12;
    end

    % NaN / Inf 守卫：上游已崩溃时退化为单位阵
    if isempty(P) || numel(P) < 4
        P_reg = eye(4);
        return;
    end
    if any(isnan(P(:))) || any(isinf(P(:)))
        P_reg = eye(size(P, 1));
        return;
    end

    % 步骤 1：强制对称化
    P_sym = (P + P') / 2.0;

    % 步骤 2：特征值分解
    [V, D] = eig(P_sym);
    d = diag(D);

    % 步骤 3：双阈值裁剪（绝对 + 相对）
    min_allowed = max(min_eig, 1e-6 * max(d));
    d_clip = max(d, min_allowed);

    % 步骤 4：重构
    P_reg = V * diag(d_clip) * V';
    P_reg = (P_reg + P_reg') / 2;
end
