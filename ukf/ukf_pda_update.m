% =========================================================================
% ukf_pda_update.m
% 栅格加权PDA (Probabilistic Data Association) UKF更新
% =========================================================================
% 功能说明：
%   替代传统的最近邻(NN)关联，使用概率数据关联方法对波门内所有点迹按
%   Gaussian似然加权，合成等效量测进行UKF更新。核心优势：
%     1. 不"孤注一掷"选最近点迹，避免杂波恰好比目标更近时的误关联
%     2. 波门内多点迹按概率权重共同参与状态更新，更稳健
%     3. beta_0项保留"目标漏检"的可能性，避免强制关联杂波
%
% PDA更新公式：
%   β_0 = (1-Pd*Pg) / ((1-Pd*Pg) + Pd*Pg*Σe_i)     -- 无量测为目标的概率
%   β_i = Pd*Pg*e_i / ((1-Pd*Pg) + Pd*Pg*Σe_i)     -- 量测i为目标的概率
%   e_i = exp(-0.5*mahal_i) / (2π√|P_zz_2d|)       -- Gaussian似然
%   ν_weighted = Σ β_i * ν_i                         -- 加权新息(3D)
%   x_new = x_pred + K * ν_weighted                  -- 状态更新
%   P_new = β_0*P_pred + (1-β_0)*P_c + K*P_spread*K' -- 协方差更新
% =========================================================================

function [lon, lat, ukf, best_det, nis_2d] = ukf_pda_update(ukf, dets_in_gate, ...
        z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz, params)

    m = length(dets_in_gate);

    % 防御：确保P_zz是3×3矩阵
    if ~isequal(size(P_zz), [3, 3])
        % P_zz维度异常，退化为单量测标准UKF更新
        [lon, lat, ukf] = ukf_filter_update(ukf, dets_in_gate{1});
        best_det = dets_in_gate{1};
        nis_2d = 0;
        return;
    end
    P_zz_2d = P_zz(1:2, 1:2);

    % ---- 步骤1：提取各量测的3D向量与2D新息 ----
    z_meas_3d = zeros(3, m);
    innov_2d = zeros(2, m);
    mahal_2d = zeros(1, m);
    for i = 1:m
        dp = dets_in_gate{i};
        z_meas_3d(:,i) = [dp.drange; dp.daz; dp.radial_vel_meas];
        innov_2d(:,i) = z_meas_3d(1:2,i) - z_pred(1:2);
        if innov_2d(2,i) > 180, innov_2d(2,i) = innov_2d(2,i) - 360;
        elseif innov_2d(2,i) < -180, innov_2d(2,i) = innov_2d(2,i) + 360; end
        mahal_2d(i) = innov_2d(:,i)' * (P_zz_2d \ innov_2d(:,i));
    end

    % ---- 步骤2：单量测退化 → 标准UKF更新 ----
    if m == 1
        [lon, lat, ukf] = ukf_filter_update(ukf, dets_in_gate{1});
        best_det = dets_in_gate{1};
        nis_2d = mahal_2d(1);
        return;
    end

    % ---- 步骤3：计算PDA关联概率 ----
    % 使用标准参数化PDA公式:
    %   e_i = exp(-0.5 * mahal_i)          -- 无单位似然
    %   b = λ * 2π√|S| * (1-Pd*Pg)/(Pd*Pg) -- 杂波有效权重
    %   β_i = e_i / (b + Σe_j)             -- 量测i为目标概率
    %   β_0 = b / (b + Σe_j)               -- 无量测为目标概率
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

    beta_0 = b / (b + sum(e));
    beta = zeros(1, m);
    for i = 1:m
        beta(i) = e(i) / (b + sum(e));
    end

    % ---- 步骤4：构造3D完整新息向量并计算加权新息 ----
    innov_3d = zeros(3, m);
    for i = 1:m
        innov_3d(:,i) = z_meas_3d(:,i) - z_pred;
        if innov_3d(2,i) > 180, innov_3d(2,i) = innov_3d(2,i) - 360;
        elseif innov_3d(2,i) < -180, innov_3d(2,i) = innov_3d(2,i) + 360; end
    end
    weighted_innov = innov_3d * beta';

    % ---- 步骤5：计算互协方差P_xz与卡尔曼增益 ----
    P_xz = zeros(ukf.n, ukf.m);
    for i = 1:(2 * ukf.n + 1)
        dz = Z_pred(:, i) - z_pred;
        dx = X_pred(:, i) - x_pred;
        P_xz = P_xz + ukf.Wc(i) * (dx * dz');
    end

    try
        K = P_xz / P_zz;
    catch
        K = P_xz * pinv(P_zz);
    end

    % ---- 步骤6：PDA状态更新 ----
    ukf.x = x_pred + K * weighted_innov;

    if any(isnan(ukf.x)) || any(isinf(ukf.x))
        ukf.x = x_pred;
        ukf.P = P_pred;
        lon = ukf.x(1);
        lat = ukf.x(3);
        [~, best_idx] = min(mahal_2d);
        best_det = dets_in_gate{best_idx};
        nis_2d = mahal_2d(best_idx);
        return;
    end

    % ---- 步骤7：PDA协方差更新（含新息散布项） ----
    P_c = P_pred - K * P_zz * K';

    innov_spread = zeros(ukf.m, ukf.m);
    for i = 1:m
        innov_spread = innov_spread + beta(i) * (innov_3d(:,i) * innov_3d(:,i)');
    end
    innov_spread = innov_spread - weighted_innov * weighted_innov';

    ukf.P = beta_0 * P_pred + (1 - beta_0) * P_c + K * innov_spread * K';
    ukf.P = regularize_cov(ukf.P);

    % ---- 步骤8：选取最高权重量测作为快照参考 ----
    [~, best_idx] = max(beta);
    best_det = dets_in_gate{best_idx};
    nis_2d = mahal_2d(best_idx);

    lon = ukf.x(1);
    lat = ukf.x(3);
end
