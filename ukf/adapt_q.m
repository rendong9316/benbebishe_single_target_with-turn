% =========================================================================
% adapt_q.m — 通用自适应 Q 调整函数
% =========================================================================
% 整合模糊自适应 Q 和机动自适应 Q，支持乘法叠加融合策略。
% 可被 ukf_zishiying 和 ukf_imm（CV/CT 子模型）复用。
%
% 输入:
%   ukf    - UKF 结构体（含 nis_history, Q_base, Q_ema, params 等字段）
%   params - 参数结构体
%   mode   - 'zishiying'（含机动检测）或 'fuzzy_only'（仅模糊）
%
% 输出:
%   ukf    - 更新后的 UKF 结构体（Q 和 Q_ema 已调整）
% =========================================================================

function ukf = adapt_q(ukf, params, mode)
    if nargin < 3
        mode = 'zishiying';
    end

    % ---- 基础检查 ----
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history) || length(ukf.nis_history) < 3
        return;
    end

    mature_frames = 12;
    if isfield(params, 'maneuver_mature_frames')
        mature_frames = params.maneuver_mature_frames;
    end
    if ~isfield(ukf, 'life_count') || ukf.life_count < mature_frames
        return;
    end

    % ---- 初始化字段 ----
    if ~isfield(ukf, 'maneuver_active'), ukf.maneuver_active = false; end
    if ~isfield(ukf, 'maneuver_counter'), ukf.maneuver_counter = 0; end
    if ~isfield(ukf, 'maneuver_recovery'), ukf.maneuver_recovery = 0; end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end

    nis_history = ukf.nis_history;

    % ---- 模糊自适应 Q ----
    nis_avg = mean(nis_history);
    nis_ratio = nis_avg / 2.0;

    mu_VS = trimf_val_adaptq(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val_adaptq(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val_adaptq(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val_adaptq(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val_adaptq(nis_ratio, 2.5, 4.0, 4.0);

    out_Decrease       = 0.6;
    out_SlightDecrease = 0.8;
    out_Maintain       = 1.0;
    out_Increase       = 1.8;
    out_RapidIncrease  = 3.0;

    total_mu = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total_mu < 1e-10
        factor_fuzzy = 1.0;
    else
        factor_fuzzy = (mu_VS * out_Decrease + mu_S * out_SlightDecrease + ...
                       mu_M * out_Maintain + mu_L * out_Increase + ...
                       mu_VL * out_RapidIncrease) / total_mu;
    end

    % ---- 机动自适应 Q（仅 mode='zishiying' 时启用） ----
    if strcmp(mode, 'fuzzy_only')
        factor_raw = factor_fuzzy;
        return;
    end

    % 机动检测: 短时 vs 长时 NIS 趋势
    % 机动检测阈值（更灵敏）
    nis_ratio_thresh = 1.10;    % 原1.25 → 1.10
    nis_short_abs = 2.0;        % 原2.8 → 2.0
    nis_long_abs = 2.5;         % 原3.2 → 2.5
    recovery_frames = 6;        % 原4 → 6（更宽容的恢复）
    max_duration = 80;          % 原50 → 80（更长的机动窗口）

    if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
    if isfield(params, 'maneuver_nis_short_thresh'), nis_short_abs = params.maneuver_nis_short_thresh; end
    if isfield(params, 'maneuver_nis_long_thresh'), nis_long_abs = params.maneuver_nis_long_thresh; end
    if isfield(params, 'maneuver_recovery_frames'), recovery_frames = params.maneuver_recovery_frames; end
    if isfield(params, 'maneuver_max_duration'), max_duration = params.maneuver_max_duration; end

    win_short = min(3, length(nis_history));
    nis_short = mean(nis_history(end-win_short+1:end));
    nis_long  = mean(nis_history);

    maneuver_detected = false;
    if (nis_short > nis_long * nis_ratio_thresh && nis_short > nis_short_abs) || nis_long > nis_long_abs
        maneuver_detected = true;
    end

    if ~ukf.maneuver_active
        if maneuver_detected
            ukf.maneuver_active = true;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
    else
        ukf.maneuver_counter = ukf.maneuver_counter + 1;
        if ~maneuver_detected
            ukf.maneuver_recovery = ukf.maneuver_recovery + 1;
        else
            ukf.maneuver_recovery = 0;
        end
        if ukf.maneuver_recovery >= recovery_frames
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
        if ukf.maneuver_counter > max_duration
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
    end

    % 机动 Q 提升因子（更激进）
    q_boost_init = 2.0;
    q_boost_mid = 3.0;
    q_boost_max = 5.0;
    if isfield(params, 'maneuver_q_boost_init'), q_boost_init = params.maneuver_q_boost_init; end
    if isfield(params, 'maneuver_q_boost_mid'), q_boost_mid = params.maneuver_q_boost_mid; end
    if isfield(params, 'maneuver_q_boost_max'), q_boost_max = params.maneuver_q_boost_max; end

    if ukf.maneuver_active
        if ukf.maneuver_counter < 5
            maneuver_target = q_boost_init + ukf.maneuver_counter * 0.2;
        elseif ukf.maneuver_counter < 15
            maneuver_target = q_boost_mid + (ukf.maneuver_counter - 5) * 0.08;
        else
            maneuver_target = q_boost_max;
        end
        % 乘法叠加: 模糊负责基线校准, 机动负责脉冲增强
        factor_raw = factor_fuzzy * maneuver_target;
    else
        factor_raw = factor_fuzzy;
    end

    factor_raw = max(0.5, min(4.0, factor_raw));

    % EMA 平滑
    ema_eta = 0.20;
    if isfield(params, 'maneuver_ema_eta'), ema_eta = params.maneuver_ema_eta; end
    ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;

    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end
end


% =========================================================================
% trimf_val_adaptq — 三角形隶属函数求值
% =========================================================================
function mu = trimf_val_adaptq(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
