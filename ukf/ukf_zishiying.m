% =========================================================================
% ukf_zishiying.m
% =========================================================================
% 功能概要：
%   自适应 UKF — 过程式调度器，封装 ukf_jichu + 自动自适应 Q。
%   'update' 动作接收外部准备好的新息和预测统计，内部委托
%   ukf_jichu('update', ...) 完成纯 Kalman 数学，再施加模糊自适应 Q。
%
% 公共 actions：
%   ukf = ukf_zishiying('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
%   ukf = ukf_zishiying('init', ukf_tpl, meas1, meas2)
%   [lon, lat, ukf] = ukf_zishiying('update', ukf, innov, z_pred, Z_pred, ...
%       X_pred, x_pred, P_pred, P_zz, params)
%      纯 Kalman 更新 + 机动自适应 Q（模糊自适应 + 机动检测 + Q 提升）
% =========================================================================

function varargout = ukf_zishiying(action, varargin)
    switch action
        case 'create'
            varargout{1} = ukf_jichu('create', varargin{:});
        case 'init'
            varargout{1} = ukf_jichu('init', varargin{:});
        case 'update'
            % varargin = {ukf, innov, z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz, params}
            [lon, lat, ukf] = ukf_jichu('update', varargin{1:8});
            ukf = apply_maneuver_adapt_post(ukf, varargin{9});
            varargout = {lon, lat, ukf};
        otherwise
            error('ukf_zishiying: unknown action ''%s''', action);
    end
end


% =========================================================================
% apply_maneuver_adapt_post — 机动自适应后处理
% 综合: 机动预检测(suspect_counter/渐进波门) + 机动自适应 Q 更新
% =========================================================================
function ukf = apply_maneuver_adapt_post(ukf, params)
    % 航迹未成熟 → 不处理
    mature_frames = 12;
    if isfield(params, 'maneuver_mature_frames')
        mature_frames = params.maneuver_mature_frames;
    end
    if ~isfield(ukf, 'life_count') || ukf.life_count < mature_frames
        return;
    end

    % ---- 初始化机动相关字段 ----
    if ~isfield(ukf, 'maneuver_active'), ukf.maneuver_active = false; end
    if ~isfield(ukf, 'maneuver_counter'), ukf.maneuver_counter = 0; end
    if ~isfield(ukf, 'maneuver_recovery'), ukf.maneuver_recovery = 0; end
    if ~isfield(ukf, 'suspect_counter'), ukf.suspect_counter = 0; end
    if ~isfield(ukf, 'innov_history'), ukf.innov_history = {}; end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end

    % ---- 无足够历史 → 返回 ----
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history) || length(ukf.nis_history) < 3
        return;
    end

    % ---- 读取机动检测阈值参数（从params读取，有默认值） ----
    nis_ratio_thresh = 1.25;
    nis_short_abs = 2.8;
    nis_long_abs = 3.2;
    recovery_frames = 4;
    max_duration = 50;
    if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
    if isfield(params, 'maneuver_nis_short_thresh'), nis_short_abs = params.maneuver_nis_short_thresh; end
    if isfield(params, 'maneuver_nis_long_thresh'), nis_long_abs = params.maneuver_nis_long_thresh; end
    if isfield(params, 'maneuver_recovery_frames'), recovery_frames = params.maneuver_recovery_frames; end
    if isfield(params, 'maneuver_max_duration'), max_duration = params.maneuver_max_duration; end

    % ---- 机动检测: 短时 vs 长时 NIS 趋势比较 ----
    nis_history = ukf.nis_history;
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

    % ---- 机动预检测：预扫描宽门限量测 (仅非机动状态) ----
    wide_gate_mult = 1.8;
    suspect_thresh = 2;
    if isfield(params, 'maneuver_wide_gate_mult'), wide_gate_mult = params.maneuver_wide_gate_mult; end
    if isfield(params, 'maneuver_suspect_thresh'), suspect_thresh = params.maneuver_suspect_thresh; end

    if ~ukf.maneuver_active && isfield(ukf, 'last_x_pred') && isfield(ukf, 'last_z_pred') ...
            && isfield(ukf, 'last_P_zz') && isfield(ukf, 'last_det_list')
        if ukf.suspect_counter < 0
            ukf.suspect_counter = 0;
        end
        x_pred = ukf.last_x_pred;
        z_pred = ukf.last_z_pred;
        P_zz = ukf.last_P_zz;
        dets = ukf.last_det_list;

        wide_gate = (params.gate_sigma * wide_gate_mult)^2 * 2;
        any_in_wide = false;
        for d = 1:length(dets)
            dp = dets(d);
            if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
            geo_d = sphere_utils_haversine_distance(x_pred(1), x_pred(3), dp.lon, dp.lat);
            if geo_d > 120000, continue; end
            z_m = [dp.drange; dp.daz];
            inno = z_m - z_pred(1:2);
            if inno(2) > 180, inno(2) = inno(2) - 360;
            elseif inno(2) < -180, inno(2) = inno(2) + 360; end
            if inno' * (P_zz(1:2,1:2) \ inno) < wide_gate
                any_in_wide = true; break;
            end
        end

        if any_in_wide
            ukf.suspect_counter = ukf.suspect_counter + 1;
        else
            ukf.suspect_counter = max(0, ukf.suspect_counter - 1);
        end

        if ukf.suspect_counter >= suspect_thresh
            ukf.maneuver_active = true;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
    end

    % ---- 记录新息历史（供机动检测用） ----
    if isfield(ukf, 'last_innov') && ~isempty(ukf.last_innov)
        ukf.innov_history{end+1} = ukf.last_innov;
        if length(ukf.innov_history) > 10
            ukf.innov_history(1) = [];
        end
    end

    % ---- 模糊自适应 Q ----
    nis_avg = mean(nis_history);
    nis_ratio = nis_avg / 2.0;

    mu_VS = trimf_val_maneuver(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val_maneuver(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val_maneuver(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val_maneuver(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val_maneuver(nis_ratio, 2.5, 4.0, 4.0);

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

    % ---- 读取机动 Q 提升参数 ----
    q_boost_init = 1.5;
    q_boost_mid = 2.3;
    q_boost_max = 3.5;
    if isfield(params, 'maneuver_q_boost_init'), q_boost_init = params.maneuver_q_boost_init; end
    if isfield(params, 'maneuver_q_boost_mid'), q_boost_mid = params.maneuver_q_boost_mid; end
    if isfield(params, 'maneuver_q_boost_max'), q_boost_max = params.maneuver_q_boost_max; end

    % ---- 机动 Q 提升因子 (渐进式, 避免突然跳变) ----
    if ukf.maneuver_active
        if ukf.maneuver_counter < 5
            maneuver_target = q_boost_init + ukf.maneuver_counter * 0.2;
        elseif ukf.maneuver_counter < 15
            maneuver_target = q_boost_mid + (ukf.maneuver_counter - 5) * 0.08;
        else
            maneuver_target = q_boost_max;
        end
        factor_raw = max(factor_fuzzy, maneuver_target);
    else
        factor_raw = factor_fuzzy;
    end

    factor_raw = max(0.5, min(4.0, factor_raw));

    % ---- EMA 平滑 ----
    ema_eta = 0.20;
    if isfield(params, 'maneuver_ema_eta'), ema_eta = params.maneuver_ema_eta; end
    ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;

    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end

    % ---- 记录机动方向 (供诊断) ----
    if ukf.maneuver_active && ~isempty(ukf.innov_history)
        recent_innov = ukf.innov_history{end};
        innov_mag = sqrt(recent_innov(1)^2 + recent_innov(2)^2);
        if innov_mag > 1e-6
            ukf.maneuver_direction = recent_innov / innov_mag;
        end
    end
end


% =========================================================================
% trimf_val_maneuver — 三角形隶属函数求值（机动版本）
% trimf(x, a, b, c): 三角形顶点在 (a,0)→(b,1)→(c,0)
% =========================================================================
function mu = trimf_val_maneuver(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
