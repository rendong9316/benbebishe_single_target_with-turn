% =========================================================================
% ukf_fuzzy_adapt.m
% 模糊自适应Q调节
% =========================================================================
% 功能说明：
%   根据归一化新息平方(NIS)的滑动平均值判断UKF当前工作状态，使用模糊
%   推理系统自适应调节过程噪声协方差Q矩阵的缩放因子。
%
% 原理：
%   NIS = innov_2d' * inv(P_zz_2d) * innov_2d ~ χ²(2)
%   理论均值 E[NIS] = 2
%
%   当NIS持续偏大 → 滤波器发散/模型失配 → 增大Q（更信任量测）
%   当NIS持续偏小 → 滤波器过度自信     → 减小Q（更信任模型，更平滑）
%
% 模糊系统：
%   输入：NIS_ratio = NIS_avg / 2（归一化到理论均值）
%   输出：Q缩放因子（连续值，带EMA平滑避免剧烈跳变）
%
% 输入：
%   ukf         — UKF结构体（含Q, Q_base, Q_ema字段）
%   nis_history — NIS滑动窗口值（向量）
%   params      — 仿真参数结构体
% 输出：
%   ukf         — 更新后的UKF结构体（Q和Q_ema已调整）
% =========================================================================

function ukf = ukf_fuzzy_adapt(ukf, nis_history, track_life, params)
    % ---- 初始化Q_ema ----
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end

    % ---- 航迹未成熟（前12帧为UKF收敛期）→ 不调节 ----
    if track_life < 12
        return;
    end

    % ---- 无足够历史 → 返回当前Q ----
    if isempty(nis_history) || length(nis_history) < 3
        return;
    end

    % ---- 步骤1：计算滑动平均NIS ----
    nis_avg = mean(nis_history);

    % NIS归一化：理论值 E[χ²(2)] = 2
    nis_ratio = nis_avg / 2.0;

    % ---- 步骤2：模糊隶属度计算（三角形隶属函数） ----
    mu_VS = trimf_val(nis_ratio, 0.0, 0.0, 0.4);   % Very Small
    mu_S  = trimf_val(nis_ratio, 0.2, 0.5, 0.8);   % Small
    mu_M  = trimf_val(nis_ratio, 0.6, 1.0, 1.5);   % Medium
    mu_L  = trimf_val(nis_ratio, 1.3, 2.0, 3.0);   % Large
    mu_VL = trimf_val(nis_ratio, 2.5, 4.0, 4.0);   % Very Large

    % ---- 步骤3：Sugeno输出常数 ----
    out_Decrease       = 0.6;    % 减小Q
    out_SlightDecrease = 0.8;    % 略减小Q
    out_Maintain       = 1.0;    % 保持Q
    out_Increase       = 1.8;    % 增大Q
    out_RapidIncrease  = 3.0;    % 大幅增大Q

    % ---- 步骤4：Sugeno加权平均解模糊 ----
    total_mu = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total_mu < 1e-10
        factor_raw = 1.0;
    else
        factor_raw = (mu_VS * out_Decrease + mu_S * out_SlightDecrease + ...
                      mu_M * out_Maintain + mu_L * out_Increase + ...
                      mu_VL * out_RapidIncrease) / total_mu;
    end

    % ---- 步骤5：因子裁剪 ----
    factor_raw = max(0.6, min(3.0, factor_raw));

    % ---- 步骤6：EMA平滑（η=0.35，保守平滑） ----
    eta = 0.35;
    ukf.Q_ema = eta * factor_raw + (1 - eta) * ukf.Q_ema;

    % ---- 步骤7：缩放Q矩阵 ----
    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;  % 接近1.0时保持基准Q
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end
end

% =========================================================================
% 三角形隶属函数求值
% trimf(x, a, b, c): 三角形顶点在 (a,0)→(b,1)→(c,0)
% =========================================================================
function mu = trimf_val(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
