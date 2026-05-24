% =========================================================================
% ukf_maneuver_adapt.m
% 机动自适应UKF — 趋势检测 + Q动态提升
% =========================================================================
% 原理:
%   1. 比较短时NIS均值 vs 长时NIS均值 — 检测渐变机动 (如缓转弯)
%   2. 短时均值 > 长时均值×1.5 → 判定为目标机动
%   3. 机动期间Q提升5-8倍, 机动结束后逐步回到模糊自适应水平
%
% 与基础模糊自适应Q的区别:
%   基础版: 连续平滑调节Q (因子0.6-3.0), 对渐变机动响应弱
%   本模块: 趋势检测+离散提升Q (因子5-8), 对转弯等机动响应快
% =========================================================================

function ukf = ukf_maneuver_adapt(ukf, nis_history, innov_history, track_life, params)
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
    if ~isfield(ukf, 'maneuver_active')
        ukf.maneuver_active = false;
    end
    if ~isfield(ukf, 'maneuver_counter')
        ukf.maneuver_counter = 0;
    end
    if ~isfield(ukf, 'maneuver_recovery')
        ukf.maneuver_recovery = 0;
    end

    if track_life < 12 || isempty(nis_history) || length(nis_history) < 6
        return;
    end

    % ---- 机动检测: 短时 vs 长时 NIS 趋势比较 ----
    win_short = min(3, length(nis_history));
    nis_short = mean(nis_history(end-win_short+1:end));
    nis_long  = mean(nis_history);

    % 机动判定: 短时NIS明显高于长时 (趋势上升+绝对值偏高)
    maneuver_detected = false;
    if (nis_short > nis_long * 1.25 && nis_short > 2.8) || nis_long > 3.2
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
        % 连续4帧趋势正常 → 结束机动
        if ukf.maneuver_recovery >= 4
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
        % 机动持续超过50帧, 强制结束
        if ukf.maneuver_counter > 50
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
    end

    % ---- 基础模糊自适应 (与原有逻辑一致) ----
    nis_avg = mean(nis_history);
    nis_ratio = nis_avg / 2.0;

    mu_VS = trimf_val_mv(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val_mv(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val_mv(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val_mv(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val_mv(nis_ratio, 2.5, 4.0, 4.0);

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

    % ---- 机动Q提升因子 (渐进式, 避免突然跳变) ----
    if ukf.maneuver_active
        % 渐进提升Q: ramp从1.5→2.5→3.5, 而非瞬间跳到7倍
        if ukf.maneuver_counter < 5
            maneuver_target = 1.5 + ukf.maneuver_counter * 0.2;  % 1.5→2.3
        elseif ukf.maneuver_counter < 15
            maneuver_target = 2.3 + (ukf.maneuver_counter - 5) * 0.08;  % 2.3→3.1
        else
            maneuver_target = 3.5;  % 最大3.5倍
        end
        factor_raw = max(factor_fuzzy, maneuver_target);
    else
        factor_raw = factor_fuzzy;
    end

    factor_raw = max(0.5, min(4.0, factor_raw));

    % ---- EMA平滑 (η=0.20, 比基础版慢得多, 防止跳变) ----
    eta = 0.20;
    ukf.Q_ema = eta * factor_raw + (1 - eta) * ukf.Q_ema;

    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end

    % ---- 记录机动方向 (供诊断) ----
    if ukf.maneuver_active && ~isempty(innov_history)
        recent_innov = innov_history{end};
        innov_mag = sqrt(recent_innov(1)^2 + recent_innov(2)^2);
        if innov_mag > 1e-6
            ukf.maneuver_direction = recent_innov / innov_mag;
        end
    end
end

function mu = trimf_val_mv(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
