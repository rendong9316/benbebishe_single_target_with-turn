% =========================================================================
% pda_weight.m
% =========================================================================
% 功能概要：
%   PDA (Probabilistic Data Association) β 权重计算 — 纯过程式函数。
%   根据波门内点迹集计算关联概率 β_i，构造 3D 加权新息向量。
%   不含任何 UKF K/P 更新数学——那部分留给 ukf_jichu('update', ...)。
%
% 输入：
%   dets_in_gate - cell 数组，波门内所有点迹结构体
%   z_pred       - 预测量测 [range; azimuth; radial_vel] (3x1)
%   P_zz         - 量测协方差矩阵 (3x3)
%   params       - 参数结构体（含 pda_pd_gate, detection_probability,
%                  pda_clutter_intensity 等）
%
% 输出：
%   innov_weighted - 加权新息向量 (3x1)，已做方位角包裹处理
%   beta_vec       - 各点迹的关联概率 (1xm)
%   nis_2d         - 2D NIS 值（最高权重对应点迹的马氏距离）
% =========================================================================

function [innov_weighted, beta_vec, nis_2d] = pda_weight(dets_in_gate, z_pred, P_zz, params)

    m = length(dets_in_gate);

    % 防御：确保 P_zz 是 3x3 矩阵
    if ~isequal(size(P_zz), [3, 3])
        if ~isempty(dets_in_gate)
            dp = dets_in_gate{1};
            z_actual = [dp.drange; dp.daz; dp.radial_vel_meas];
            innov_weighted = z_actual - z_pred;
            if innov_weighted(2) > 180
                innov_weighted(2) = innov_weighted(2) - 360;
            elseif innov_weighted(2) < -180
                innov_weighted(2) = innov_weighted(2) + 360;
            end
        else
            innov_weighted = zeros(3, 1);
        end
        beta_vec = 1;
        nis_2d = 0;
        return;
    end
    P_zz_2d = P_zz(1:2, 1:2);

    % ---- Step 1: 提取各量测的 3D 向量与 2D 新息 ----
    z_meas_3d = zeros(3, m);
    innov_2d = zeros(2, m);
    mahal_2d = zeros(1, m);
    for i = 1:m
        dp = dets_in_gate{i};
        z_meas_3d(:, i) = [dp.drange; dp.daz; dp.radial_vel_meas];
        innov_2d(:, i) = z_meas_3d(1:2, i) - z_pred(1:2);
        if innov_2d(2, i) > 180
            innov_2d(2, i) = innov_2d(2, i) - 360;
        elseif innov_2d(2, i) < -180
            innov_2d(2, i) = innov_2d(2, i) + 360;
        end
        mahal_2d(i) = innov_2d(:, i)' * (P_zz_2d \ innov_2d(:, i));
    end

    % ---- Step 2: 计算 PDA 关联概率 β_i（含 m=1 情况，不再退化） ----
    Pg = params.pda_pd_gate;
    Pd = params.detection_probability;
    alpha = Pd * Pg;

    det_Pzz_2d = det(P_zz_2d);
    V_norm = 2 * pi * sqrt(det_Pzz_2d);
    lambda = params.pda_clutter_intensity;
    b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);

    e = zeros(1, m);
    for i = 1:m
        e(i) = exp(-0.5 * mahal_2d(i));
    end

    beta_vec = zeros(1, m);
    for i = 1:m
        beta_vec(i) = e(i) / (b + sum(e));
    end

    % ---- Step 3: 构造 3D 完整新息向量并计算加权新息 ----
    innov_3d = zeros(3, m);
    for i = 1:m
        innov_3d(:, i) = z_meas_3d(:, i) - z_pred;
        if innov_3d(2, i) > 180
            innov_3d(2, i) = innov_3d(2, i) - 360;
        elseif innov_3d(2, i) < -180
            innov_3d(2, i) = innov_3d(2, i) + 360;
        end
    end
    innov_weighted = innov_3d * beta_vec';

    % ---- Step 4: 选取最高权重量测对应的 NIS 值 ----
    [~, best_idx] = max(beta_vec);
    nis_2d = mahal_2d(best_idx);
end
