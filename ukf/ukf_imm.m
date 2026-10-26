% =========================================================================
% ukf_imm.m -- IMM (Interacting Multiple Model) UKF Filter (5 models version)
% =========================================================================
% Function: varargout = ukf_imm(action, varargin)
% Dispatches to create_imm, init_imm, prepare_imm, update_imm
%
% This implements a 5-model IMM with:
%   Model 1: CV (constant velocity) - turn rate = 0
%   Model 2: CTL-slow (left turn) - turn rate = +0.7 deg/s
%   Model 3: CTL-fast (left turn) - turn rate = +1.5 deg/s
%   Model 4: CTR-fast (right turn) - turn rate = -1.5 deg/s
%   Model 5: CTR-slow (right turn) - turn rate = -0.7 deg/s

function varargout = ukf_imm(action, varargin)
    switch action
        case 'create'
            varargout{1} = create_imm(varargin{:});
        case 'init'
            varargout{1} = init_imm(varargin{:});
        case 'prepare'
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}, ...
             varargout{5}, varargout{6}, varargout{7}] = prepare_imm(varargin{:});
        case 'update'
            [varargout{1}, varargout{2}, varargout{3}] = update_imm(varargin{:});
        otherwise
            error('ukf_imm: unknown action %s', action);
    end
end

% =========================================================================
% create_imm -- Create IMM UKF template (5 models)
% =========================================================================
function imm = create_imm(params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
    imm.params = params;
    imm.radar_lon = radar_lon;
    imm.radar_lat = radar_lat;
    imm.tx_lon = tx_lon;
    imm.tx_lat = tx_lat;
    imm.dt = dt;

    % Turn rates in rad/s for 5 models
    % 【2026-10-26】改为可配置：实测真实 ADS-B 航迹转弯率中位 0.012 °/s、
    % p90 0.23、p99 0.66，而原默认 0.7/1.5 明显偏快（对应 p99 与极端机动）。
    % 通过 params.imm_ct_omega_*_deg 可配置，默认保持 0.7/1.5 向后兼容。
    omega_slow_deg = get_param_imm(params, 'imm_ct_omega_slow_deg', 0.7);
    omega_fast_deg = get_param_imm(params, 'imm_ct_omega_fast_deg', 1.5);
    omega_slow_rad = deg2rad(omega_slow_deg);
    omega_fast_rad = deg2rad(omega_fast_deg);

    % Create 5 UKF instances
    % Model 1: CV
    ukf_cv = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_cv.model_type = 'CV';
    ukf_cv.turn_rate_rad_per_sec = 0.0;

    % Model 2: CTL slow (left turn, +0.7 deg/s)
    ukf_ct_slow = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_ct_slow.model_type = 'CT';
    ukf_ct_slow.turn_rate_rad_per_sec = omega_slow_rad;

    % Model 3: CTL fast (left turn, +1.5 deg/s)
    ukf_ct_fast = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_ct_fast.model_type = 'CT';
    ukf_ct_fast.turn_rate_rad_per_sec = omega_fast_rad;

    % Model 4: CTR fast (right turn, -1.5 deg/s)
    ukf_ctr_fast = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_ctr_fast.model_type = 'CT';
    ukf_ctr_fast.turn_rate_rad_per_sec = -omega_fast_rad;

    % Model 5: CTR slow (right turn, -0.7 deg/s)
    ukf_ctr_slow = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_ctr_slow.model_type = 'CT';
    ukf_ctr_slow.turn_rate_rad_per_sec = -omega_slow_rad;

    imm.ukf_cv = ukf_cv;
    imm.ukf_ct_slow = ukf_ct_slow;
    imm.ukf_ct_fast = ukf_ct_fast;
    imm.ukf_ctr_fast = ukf_ctr_fast;
    imm.ukf_ctr_slow = ukf_ctr_slow;
    imm.model_fields = {'ukf_cv', 'ukf_ct_slow', 'ukf_ct_fast', ...
        'ukf_ctr_fast', 'ukf_ctr_slow'};

    % Markov transition matrix (5x5)
    p_cv_ct = get_param_imm(params, 'imm_Pi_CV_to_CT', 0.001);
    p_ct_cv = get_param_imm(params, 'imm_Pi_CT_to_CV', 0.001);
    adapt_mode_local = get_param_imm(params, 'imm_adapt_mode', '3in1');
    if strcmp(adapt_mode_local, '3in1')
        p_cv_ct = get_param_imm(params, 'imm_slow_Pi_CV_to_CT', p_cv_ct);
        p_ct_cv = get_param_imm(params, 'imm_slow_Pi_CT_to_CV', p_ct_cv);
    end
    if isfield(params, 'imm_cv_dwell_time_sec')
        p_cv_ct = 1 - exp(-dt / params.imm_cv_dwell_time_sec);
    end
    if isfield(params, 'imm_ct_dwell_time_sec')
        p_ct_cv = 1 - exp(-dt / params.imm_ct_dwell_time_sec);
    end
    validate_transition_probability(p_cv_ct, 'CV_to_CT');
    validate_transition_probability(p_ct_cv, 'CT_to_CV');

    % Keep the configured CV-to-CT value as the total entry probability.
    % CT rows split the non-CV probability between staying in the same model
    % and moving to one of the other CT candidates.
    ct_switch_total = get_param_imm(params, 'imm_Pi_CT_to_CT', 0.003);
    validate_transition_probability(ct_switch_total, 'CT_to_CT');
    if p_ct_cv + ct_switch_total > 1
        error('ukf_imm:invalidTransitionRow', ...
            'imm_Pi_CT_to_CV + imm_Pi_CT_to_CT must not exceed 1.');
    end

    P = zeros(5, 5);
    P(1, 1) = 1 - p_cv_ct;
    P(1, 2:5) = p_cv_ct / 4;
    for m = 2:5
        P(m, 1) = p_ct_cv;
        other_ct = setdiff(2:5, m);
        P(m, other_ct) = ct_switch_total / numel(other_ct);
        P(m, m) = 1 - p_ct_cv - ct_switch_total;
    end
    imm.Pi = P;

    % Optional adaptive CT-to-CV feedback uses the same base for all CT models.
    if isfield(params, 'k_turnrate') && params.k_turnrate > 0
        imm.adaptive_k = params.k_turnrate;
        imm.base_p_ct_cv = p_ct_cv;
    end

    % Initial model probabilities
    if isfield(params, 'imm_mu_init_CV')
        mu_cv = params.imm_mu_init_CV;
        mu_rest = (1 - mu_cv) / 4;
        imm.mu = [mu_cv; mu_rest; mu_rest; mu_rest; mu_rest];
    else
        imm.mu = ones(5, 1) / 5;
    end

    % IMM-IPDA parameters
    imm.Pd = params.detection_probability;
    imm.Pg = params.pda_pd_gate;
    imm.Pd_Pg = imm.Pd * imm.Pg;
    imm.L_no_det = 1.0 - imm.Pd_Pg;

    % Probability bounds
    imm.mu_min = 0.02;
    imm.mu_max = 0.95;

    % Model count
    imm.M = 5;

    % Capability flags
    imm.filter_type = 'imm';
    imm.imm_adapt_mode = get_imm_adapt_mode(imm);
    imm.capability = struct('adaptive_q', strcmp(imm.imm_adapt_mode, '3in1'), ...
                            'imm', true, ...
                            'models', {{'CV', 'CTL-slow', 'CTL-fast', 'CTR-fast', 'CTR-slow'}});

    imm.initialized = false;
    imm.cache = [];
    % 航迹年龄计数器：每次 prepare 自增，供三段式 Q 调度使用。
    % 不依赖 tracker 层的 life 字段（该字段在 IMM 内从未被赋值）。
    imm.age = 0;
    % 上一帧是否获得了观测更新（true/false），供漏检帧 Q 膨胀使用
    imm.last_updated = true;
end

% =========================================================================
% init_imm -- Initialize all 5 UKFs
% =========================================================================
function imm = init_imm(imm, meas1, meas2, varargin)
    history_args = varargin;

    imm.ukf_cv = ukf_jichu('init', imm.ukf_cv, meas1, meas2, history_args{:});
    imm.ukf_cv.dt = imm.dt;
    imm.ukf_cv.initialized = true;
    imm.ukf_cv.Q_base = imm.ukf_cv.Q;
    imm.ukf_cv.Q_ema = 1.0;
    imm.ukf_cv.transient_nis_ewma = 0.0;
    imm.ukf_cv.nis_history = [];

    imm.ukf_ct_slow = ukf_jichu('init', imm.ukf_ct_slow, meas1, meas2, history_args{:});
    imm.ukf_ct_slow.dt = imm.dt;
    imm.ukf_ct_slow.initialized = true;
    imm.ukf_ct_slow.Q_base = imm.ukf_ct_slow.Q;
    imm.ukf_ct_slow.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
    imm.ukf_ct_slow.nis_history = [];

    imm.ukf_ct_fast = ukf_jichu('init', imm.ukf_ct_fast, meas1, meas2, history_args{:});
    imm.ukf_ct_fast.dt = imm.dt;
    imm.ukf_ct_fast.initialized = true;
    imm.ukf_ct_fast.Q_base = imm.ukf_ct_fast.Q;
    imm.ukf_ct_fast.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
    imm.ukf_ct_fast.nis_history = [];

    imm.ukf_ctr_fast = ukf_jichu('init', imm.ukf_ctr_fast, meas1, meas2, history_args{:});
    imm.ukf_ctr_fast.dt = imm.dt;
    imm.ukf_ctr_fast.initialized = true;
    imm.ukf_ctr_fast.Q_base = imm.ukf_ctr_fast.Q;
    imm.ukf_ctr_fast.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
    imm.ukf_ctr_fast.nis_history = [];

    imm.ukf_ctr_slow = ukf_jichu('init', imm.ukf_ctr_slow, meas1, meas2, history_args{:});
    imm.ukf_ctr_slow.dt = imm.dt;
    imm.ukf_ctr_slow.initialized = true;
    imm.ukf_ctr_slow.Q_base = imm.ukf_ctr_slow.Q;
    imm.ukf_ctr_slow.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
    imm.ukf_ctr_slow.nis_history = [];

    imm.mu = initial_mode_probabilities(imm.params);
    imm.imm_adapt_mode = get_imm_adapt_mode(imm);
    imm.capability.adaptive_q = strcmp(imm.imm_adapt_mode, '3in1');
    imm.initialized = true;
    imm.nis_history = [];
    imm.mu_history = zeros(0, 5);
    imm.log_likelihood_history = zeros(0, 5);
    imm.x = imm.ukf_cv.x;
    imm.P = imm.ukf_cv.P;
    imm.Q = imm.ukf_cv.Q;
end

% =========================================================================
% prepare_imm -- IMM prediction step
% =========================================================================
function [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, imm] = prepare_imm(imm)
    M = imm.M;
    Pi = imm.Pi;
    mu = imm.mu;
    ukf_models = {imm.ukf_cv, imm.ukf_ct_slow, imm.ukf_ct_fast, imm.ukf_ctr_fast, imm.ukf_ctr_slow};

    % ---- 航迹年龄自增（供 Q 调度使用）----
    if ~isfield(imm, 'age') || ~isfinite(imm.age)
        imm.age = 0;
    end
    imm.age = imm.age + 1;

    % 注意：Q 调度必须放在 IMM 混合之后（否则 Q_ema 会被混合结果覆盖），
    % 见下方 "Apply adaptive Q before prediction" 处。

    % Step 1: Mixing
    local_Pi = Pi;
    if isfield(imm, 'adaptive_k') && imm.adaptive_k > 0
        ct_mu = sum(mu(2:end));
        base_p = imm.base_p_ct_cv;
        mode_local = 'exp_decay';
        if isfield(imm.params, 'imm_adaptive_mode')
            mode_local = imm.params.imm_adaptive_mode;
        end

        if strcmp(mode_local, 'exp_decay')
            power = get_param_imm(imm.params, 'power', 1.5);
            kp = imm.adaptive_k;
            adj_lev = base_p * exp(-kp * ct_mu^power);
        elseif strcmp(mode_local, 'sigmoid')
            threshold = get_param_imm(imm.params, 'threshold', 0.4);
            power = get_param_imm(imm.params, 'power', 1.0);
            kp = imm.adaptive_k * 3;
            adj_lev = base_p / (1 + exp(kp * ((ct_mu^power) - threshold)));
        else
            adj_lev = base_p / (1 + imm.adaptive_k * ct_mu);
        end

        adj_lev = max(adj_lev, 0.05 * base_p);
        adj_lev = min(adj_lev, base_p);

        % 【2026-10-25 修复】原实现只改写第 1 列与对角元，CT 行残留的
        % CT→CT 非对角元未清零，导致转移矩阵行和 = 1 + ct_switch_total ≈ 1.003，
        % 破坏马尔可夫性。此处按行重排并保持行和严格为 1。
        ct_switch = get_param_imm(imm.params, 'imm_Pi_CT_to_CT', 0.003);
        for m = 2:M
            other = setdiff(2:M, m);
            local_Pi(m,1) = adj_lev;
            local_Pi(m,other) = ct_switch / numel(other);
            local_Pi(m,m) = 1 - adj_lev - ct_switch;
        end
    end

    c_bar = local_Pi' * mu;
    mu_mix = zeros(M, M);
    for i = 1:M
        for j = 1:M
            mu_mix(i, j) = local_Pi(i, j) * mu(i) / max(c_bar(j), 1e-12);
        end
    end

    x_mix = repmat({zeros(4,1)}, 1, M);
    P_mix = repmat({zeros(4,4)}, 1, M);
    for j = 1:M
        for i = 1:M
            x_mix{j} = x_mix{j} + mu_mix(i, j) * ukf_models{i}.x;
        end
        for i = 1:M
            dx = ukf_models{i}.x - x_mix{j};
            P_mix{j} = P_mix{j} + mu_mix(i, j) * (ukf_models{i}.P + dx * dx');
        end
    end

    for m = 1:M
        ukf_models{m}.x = x_mix{m};
        ukf_models{m}.P = regularize_imm(P_mix{m});
    end

    imm.ukf_cv = ukf_models{1};
    imm.ukf_ct_slow = ukf_models{2};
    imm.ukf_ct_fast = ukf_models{3};
    imm.ukf_ctr_fast = ukf_models{4};
    imm.ukf_ctr_slow = ukf_models{5};

    % Apply adaptive Q before prediction
    if isfield(imm, 'life_count')
        for m = 1:M
            ukf_models{m}.life_count = imm.life_count;
        end
    end

    % ---- 三段式过程噪声调度（必须位于混合之后，否则 Q_ema 被覆盖）----
    % 起始段用大 Q 快速建立速度估计，稳态段用小 Q 保持平滑。
    % 依据：实测速度误差 80 m/s 需 40 帧才收敛（Q_vel 仅 0.77 m/s/帧）。
    if get_param_imm(imm.params, 'imm_q_schedule_enable', 0)
        q_sched = get_param_imm(imm.params, 'imm_q_stage3_scale', 1);
        if imm.age <= get_param_imm(imm.params, 'imm_q_stage1_frames', 5)
            q_sched = get_param_imm(imm.params, 'imm_q_stage1_scale', 30);
        elseif imm.age <= get_param_imm(imm.params, 'imm_q_stage2_frames', 15)
            q_sched = get_param_imm(imm.params, 'imm_q_stage2_scale', 6);
        end
        % 漏检帧膨胀：上一帧无观测更新，说明目标可能机动或量测断续
        if isfield(imm, 'last_updated') && ~imm.last_updated
            q_sched = q_sched * get_param_imm(imm.params, 'imm_q_miss_scale', 3);
        end
        ct_q = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 4.5);
        imm.ukf_cv.Q_ema        = q_sched;
        imm.ukf_ct_slow.Q_ema   = q_sched * ct_q;
        imm.ukf_ct_fast.Q_ema   = q_sched * ct_q;
        imm.ukf_ctr_fast.Q_ema  = q_sched * ct_q;
        imm.ukf_ctr_slow.Q_ema  = q_sched * ct_q;
        imm.q_schedule_current  = q_sched;   % 记录，供诊断与论文绘图
    end
    if isfield(imm.params, 'use_fuzzy_adaptive') && imm.params.use_fuzzy_adaptive
        adapt_mode = get_imm_adapt_mode(imm);
        if strcmp(adapt_mode, '3in1')
            imm.ukf_cv = apply_transient_q_imm(imm.ukf_cv, imm.params);
            imm.ukf_ct_slow.Q = imm.ukf_ct_slow.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ct_slow.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ct_fast.Q = imm.ukf_ct_fast.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ct_fast.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ctr_fast.Q = imm.ukf_ctr_fast.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ctr_fast.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ctr_slow.Q = imm.ukf_ctr_slow.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ctr_slow.Q_ema = get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
        elseif strcmp(adapt_mode, 'fuzzy_only')
            for m = 1:M
                ukf_models{m} = adapt_q(ukf_models{m}, imm.params, 'fuzzy_only');
            end
        end
    end

    % Step 2: Independent prediction for each model
    [x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, imm.ukf_cv] = ukf_jichu('prepare', imm.ukf_cv);
    [x_pred_ct_slow, P_pred_ct_slow, X_pred_ct_slow, z_pred_ct_slow, Z_pred_ct_slow, P_zz_ct_slow, imm.ukf_ct_slow] = ukf_jichu('prepare', imm.ukf_ct_slow);
    [x_pred_ct_fast, P_pred_ct_fast, X_pred_ct_fast, z_pred_ct_fast, Z_pred_ct_fast, P_zz_ct_fast, imm.ukf_ct_fast] = ukf_jichu('prepare', imm.ukf_ct_fast);
    [x_pred_ctr_fast, P_pred_ctr_fast, X_pred_ctr_fast, z_pred_ctr_fast, Z_pred_ctr_fast, P_zz_ctr_fast, imm.ukf_ctr_fast] = ukf_jichu('prepare', imm.ukf_ctr_fast);
    [x_pred_ctr_slow, P_pred_ctr_slow, X_pred_ctr_slow, z_pred_ctr_slow, Z_pred_ctr_slow, P_zz_ctr_slow, imm.ukf_ctr_slow] = ukf_jichu('prepare', imm.ukf_ctr_slow);

    preds = {x_pred_cv, x_pred_ct_slow, x_pred_ct_fast, x_pred_ctr_fast, x_pred_ctr_slow};
    P_preds = {P_pred_cv, P_pred_ct_slow, P_pred_ct_fast, P_pred_ctr_fast, P_pred_ctr_slow};
    X_preds = {X_pred_cv, X_pred_ct_slow, X_pred_ct_fast, X_pred_ctr_fast, X_pred_ctr_slow};
    z_preds = {z_pred_cv, z_pred_ct_slow, z_pred_ct_fast, z_pred_ctr_fast, z_pred_ctr_slow};
    Z_preds = {Z_pred_cv, Z_pred_ct_slow, Z_pred_ct_fast, Z_pred_ctr_fast, Z_pred_ctr_slow};
    P_zz_vals = {P_zz_cv, P_zz_ct_slow, P_zz_ct_fast, P_zz_ctr_fast, P_zz_ctr_slow};

    % Step 3: Combined prediction
    x_pred_comb = c_bar(1)*preds{1} + c_bar(2)*preds{2} + c_bar(3)*preds{3} + ...
                  c_bar(4)*preds{4} + c_bar(5)*preds{5};
    P_pred_comb = combine_cov_imm(preds, P_preds, c_bar, x_pred_comb);

    z_pred_comb = c_bar(1)*z_preds{1} + c_bar(2)*z_preds{2} + c_bar(3)*z_preds{3} + ...
                  c_bar(4)*z_preds{4} + c_bar(5)*z_preds{5};
    z_pred_comb(2) = circular_mean_imm([z_preds{1}(2), z_preds{2}(2), z_preds{3}(2), z_preds{4}(2), z_preds{5}(2)], c_bar);

    P_zz_comb = combine_meas_cov_imm(z_preds, P_zz_vals, c_bar, z_pred_comb);

    x_pred = x_pred_comb;
    z_pred = z_pred_comb;
    P_zz = P_zz_comb;
    P_pred = P_pred_comb;
    X_pred = X_preds{1};
    Z_pred = Z_preds{1};

    imm.cache = struct(...
        'x_pred_cv', x_pred_cv, 'x_pred_ct_slow', x_pred_ct_slow, 'x_pred_ct_fast', x_pred_ct_fast, ...
        'x_pred_ctr_fast', x_pred_ctr_fast, 'x_pred_ctr_slow', x_pred_ctr_slow, ...
        'P_pred_cv', P_pred_cv, 'P_pred_ct_slow', P_pred_ct_slow, 'P_pred_ct_fast', P_pred_ct_fast, ...
        'P_pred_ctr_fast', P_pred_ctr_fast, 'P_pred_ctr_slow', P_pred_ctr_slow, ...
        'X_pred_cv', X_pred_cv, 'X_pred_ct_slow', X_pred_ct_slow, 'X_pred_ct_fast', X_pred_ct_fast, ...
        'X_pred_ctr_fast', X_pred_ctr_fast, 'X_pred_ctr_slow', X_pred_ctr_slow, ...
        'z_pred_cv', z_pred_cv, 'z_pred_ct_slow', z_pred_ct_slow, 'z_pred_ct_fast', z_pred_ct_fast, ...
        'z_pred_ctr_fast', z_pred_ctr_fast, 'z_pred_ctr_slow', z_pred_ctr_slow, ...
        'Z_pred_cv', Z_pred_cv, 'Z_pred_ct_slow', Z_pred_ct_slow, 'Z_pred_ct_fast', Z_pred_ct_fast, ...
        'Z_pred_ctr_fast', Z_pred_ctr_fast, 'Z_pred_ctr_slow', Z_pred_ctr_slow, ...
        'P_zz_cv', P_zz_cv, 'P_zz_ct_slow', P_zz_ct_slow, 'P_zz_ct_fast', P_zz_ct_fast, ...
        'P_zz_ctr_fast', P_zz_ctr_fast, 'P_zz_ctr_slow', P_zz_ctr_slow, ...
        'x_pred_comb', x_pred_comb, 'z_pred_comb', z_pred_comb, 'P_zz_comb', P_zz_comb, ...
        'c_bar', c_bar);
end

% =========================================================================
% update_imm -- IMM update step
% =========================================================================
function [lon, lat, imm] = update_imm(imm, innov_w)
    cache = imm.cache;
    nz = imm.ukf_cv.m;

    % 记录本帧是否获得观测更新，供下一帧 prepare 的漏检 Q 膨胀使用
    imm.last_updated = ~isempty(innov_w);

    if isempty(innov_w)
        imm.ukf_cv = keep_prediction(imm.ukf_cv, cache, 'cv');
        imm.ukf_ct_slow = keep_prediction(imm.ukf_ct_slow, cache, 'ct_slow');
        imm.ukf_ct_fast = keep_prediction(imm.ukf_ct_fast, cache, 'ct_fast');
        imm.ukf_ctr_fast = keep_prediction(imm.ukf_ctr_fast, cache, 'ctr_fast');
        imm.ukf_ctr_slow = keep_prediction(imm.ukf_ctr_slow, cache, 'ctr_slow');
        log_likelihood = zeros(5, 1);
    else
        z_weighted = innov_w + cache.z_pred_comb;

        innov_cv = z_weighted - cache.z_pred_cv;
        innov_ct_slow = z_weighted - cache.z_pred_ct_slow;
        innov_ct_fast = z_weighted - cache.z_pred_ct_fast;
        innov_ctr_fast = z_weighted - cache.z_pred_ctr_fast;
        innov_ctr_slow = z_weighted - cache.z_pred_ctr_slow;

        if abs(innov_cv(2)) > 180
            innov_cv(2) = innov_cv(2) - 360 * round(innov_cv(2) / 360);
        end
        if abs(innov_ct_slow(2)) > 180
            innov_ct_slow(2) = innov_ct_slow(2) - 360 * round(innov_ct_slow(2) / 360);
        end
        if abs(innov_ct_fast(2)) > 180
            innov_ct_fast(2) = innov_ct_fast(2) - 360 * round(innov_ct_fast(2) / 360);
        end
        if abs(innov_ctr_fast(2)) > 180
            innov_ctr_fast(2) = innov_ctr_fast(2) - 360 * round(innov_ctr_fast(2) / 360);
        end
        if abs(innov_ctr_slow(2)) > 180
            innov_ctr_slow(2) = innov_ctr_slow(2) - 360 * round(innov_ctr_slow(2) / 360);
        end

        [~, ~, imm.ukf_cv] = ukf_jichu('update', imm.ukf_cv, innov_cv);
        [~, ~, imm.ukf_ct_slow] = ukf_jichu('update', imm.ukf_ct_slow, innov_ct_slow);
        [~, ~, imm.ukf_ct_fast] = ukf_jichu('update', imm.ukf_ct_fast, innov_ct_fast);
        [~, ~, imm.ukf_ctr_fast] = ukf_jichu('update', imm.ukf_ctr_fast, innov_ctr_fast);
        [~, ~, imm.ukf_ctr_slow] = ukf_jichu('update', imm.ukf_ctr_slow, innov_ctr_slow);

        nis_cv_val = innov_cv' * (cache.P_zz_cv \ innov_cv);
        nis_ct_slow_val = innov_ct_slow' * (cache.P_zz_ct_slow \ innov_ct_slow);
        nis_ct_fast_val = innov_ct_fast' * (cache.P_zz_ct_fast \ innov_ct_fast);
        nis_ctr_fast_val = innov_ctr_fast' * (cache.P_zz_ctr_fast \ innov_ctr_fast);
        nis_ctr_slow_val = innov_ctr_slow' * (cache.P_zz_ctr_slow \ innov_ctr_slow);

        if ~isnan(nis_cv_val)
            imm.ukf_cv.nis_history(end+1) = nis_cv_val;
        end
        if ~isnan(nis_ct_slow_val)
            imm.ukf_ct_slow.nis_history(end+1) = nis_ct_slow_val;
        end
        if ~isnan(nis_ct_fast_val)
            imm.ukf_ct_fast.nis_history(end+1) = nis_ct_fast_val;
        end
        if ~isnan(nis_ctr_fast_val)
            imm.ukf_ctr_fast.nis_history(end+1) = nis_ctr_fast_val;
        end
        if ~isnan(nis_ctr_slow_val)
            imm.ukf_ctr_slow.nis_history(end+1) = nis_ctr_slow_val;
        end
        imm.nis_history = imm.ukf_cv.nis_history;

        log_likelihood = [gaussian_log_likelihood_imm(cache.P_zz_cv, innov_cv, nz); ...
                          gaussian_log_likelihood_imm(cache.P_zz_ct_slow, innov_ct_slow, nz); ...
                          gaussian_log_likelihood_imm(cache.P_zz_ct_fast, innov_ct_fast, nz); ...
                          gaussian_log_likelihood_imm(cache.P_zz_ctr_fast, innov_ctr_fast, nz); ...
                          gaussian_log_likelihood_imm(cache.P_zz_ctr_slow, innov_ctr_slow, nz)];
    end

    % Bayesian model probability update
    log_weight = log_likelihood + log(max(cache.c_bar, realmin));
    log_weight = log_weight - max(log_weight);
    weighted_likelihood = exp(log_weight);
    mu_new = weighted_likelihood / sum(weighted_likelihood);
    imm.mu = project_bounded_probability_imm(mu_new, imm.mu_min, imm.mu_max);

    % State combination
    x_comb = imm.mu(1)*imm.ukf_cv.x + imm.mu(2)*imm.ukf_ct_slow.x + imm.mu(3)*imm.ukf_ct_fast.x + ...
             imm.mu(4)*imm.ukf_ctr_fast.x + imm.mu(5)*imm.ukf_ctr_slow.x;
    lon = x_comb(1);
    lat = x_comb(3);

    imm.x = x_comb;
    imm.P = combine_cov_imm({imm.ukf_cv.x, imm.ukf_ct_slow.x, imm.ukf_ct_fast.x, imm.ukf_ctr_fast.x, imm.ukf_ctr_slow.x}, ...
                            {imm.ukf_cv.P, imm.ukf_ct_slow.P, imm.ukf_ct_fast.P, imm.ukf_ctr_fast.P, imm.ukf_ctr_slow.P}, ...
                            imm.mu, x_comb);
    imm.Q = imm.mu(1)*imm.ukf_cv.Q + imm.mu(2)*imm.ukf_ct_slow.Q + imm.mu(3)*imm.ukf_ct_fast.Q + ...
            imm.mu(4)*imm.ukf_ctr_fast.Q + imm.mu(5)*imm.ukf_ctr_slow.Q;
    imm.Q_ema = imm.mu(1)*imm.ukf_cv.Q_ema + imm.mu(2)*imm.ukf_ct_slow.Q_ema + imm.mu(3)*imm.ukf_ct_fast.Q_ema + ...
                imm.mu(4)*imm.ukf_ctr_fast.Q_ema + imm.mu(5)*imm.ukf_ctr_slow.Q_ema;
    imm.mu_history(end+1, :) = imm.mu';
    imm.log_likelihood_history(end+1, :) = log_likelihood';
end

function mean_deg = circular_mean_imm(angles_deg, weights)
    % Compute circular mean of angles_deg weighted by weights
    weights = weights(:) / sum(weights(:));  % Column vector
    angles_deg = angles_deg(:);             % Column vector
    reference = angles_deg(1);
    delta = angles_deg - reference;         % Column vector
    delta = mod(delta + 180.0, 360.0) - 180.0;  % Vectorized wrap
    mean_deg = mod(reference + weights' * delta, 360.0);  % Dot product: (1xN)*(Nx1) = scalar
end

function angle = wrap_angle_imm(angle)
    angle = mod(angle + 180.0, 360.0) - 180.0;
end

function value = gaussian_log_likelihood_imm(P, innov, dimension)
    P = (P + P') / 2;
    [L, flag] = chol(P, 'lower');
    if flag ~= 0
        P = regularize_imm(P);
        [L, flag] = chol(P, 'lower');
    end
    if flag ~= 0
        value = -realmax;
        return;
    end
    whitened = L \ innov;
    log_det = 2 * sum(log(diag(L)));
    value = -0.5 * (dimension * log(2*pi) + log_det + whitened' * whitened);
end

function probability = project_bounded_probability_imm(probability, lower, upper)
    probability = max(0, probability(:));
    if sum(probability) <= 0
        probability(:) = 1 / numel(probability);
    else
        probability = probability / sum(probability);
    end
    probability = min(upper, max(lower, probability));
    for iteration = 1:10
        residual = 1 - sum(probability);
        if abs(residual) < 1e-14
            break;
        end
        if residual > 0
            capacity = max(0, upper - probability);
        else
            capacity = max(0, probability - lower);
        end
        if sum(capacity) <= 0
            break;
        end
        probability = probability + residual * capacity / sum(capacity);
        probability = min(upper, max(lower, probability));
    end
    probability = probability / sum(probability);
end

function P_comb = combine_cov_imm(x_models, P_models, mu, x_comb)
    P_comb = zeros(size(P_models{1}));
    for i = 1:length(mu)
        dx = x_models{i} - x_comb;
        P_comb = P_comb + mu(i) * (P_models{i} + dx * dx');
    end
    P_comb = regularize_imm(P_comb);
end

function P_reg = regularize_imm(P)
    P_reg = (P + P') / 2;
    if any(isnan(P_reg(:))) || any(isinf(P_reg(:)))
        P_reg = eye(size(P_reg)) * 1e-6;
        return;
    end
    min_eig = 1e-12;
    [V, D] = eig(P_reg);
    d = diag(D);
    d(d < min_eig) = min_eig;
    P_reg = V * diag(d) * V';
    P_reg = (P_reg + P_reg') / 2;
end

function P_zz_comb = combine_meas_cov_imm(z_models, P_zz_models, mu, z_comb)
    P_zz_comb = zeros(size(P_zz_models{1}));
    for i = 1:length(mu)
        dz = z_models{i} - z_comb;
        if length(dz) >= 2 && abs(dz(2)) > 180
            dz(2) = dz(2) - 360 * round(dz(2) / 360);
        end
        P_zz_comb = P_zz_comb + mu(i) * (P_zz_models{i} + dz * dz');
    end
    P_zz_comb = (P_zz_comb + P_zz_comb') / 2;
end

function adapt_mode = get_imm_adapt_mode(imm)
    adapt_mode = '3in1';
    if isfield(imm, 'imm_adapt_mode')
        adapt_mode = imm.imm_adapt_mode;
    elseif isfield(imm, 'params') && isfield(imm.params, 'imm_adapt_mode')
        adapt_mode = imm.params.imm_adapt_mode;
    end
end

function ukf = apply_transient_q_imm(ukf, params)
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end
    ukf.Q = ukf.Q_base;
    ukf.Q_ema = 1.0;

    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history)
        return;
    end

    nis_now = ukf.nis_history(end);
    nis_start = get_param_imm(params, 'imm_transient_nis_start', 3.0);
    nis_full = get_param_imm(params, 'imm_transient_nis_full', 12.0);
    gain_max = get_param_imm(params, 'imm_transient_gain_max', 5.0);
    ewma_alpha = get_param_imm(params, 'imm_transient_ewma_alpha', 0.65);

    if ~isfield(ukf, 'transient_nis_ewma') || isempty(ukf.transient_nis_ewma)
        ukf.transient_nis_ewma = 0.0;
    end

    nis_excess = max(0.0, nis_now - nis_start);
    ukf.transient_nis_ewma = ewma_alpha * nis_excess + (1.0 - ewma_alpha) * ukf.transient_nis_ewma;
    if ukf.transient_nis_ewma <= 0
        return;
    end

    nis_span = max(nis_full - nis_start, 1e-6);
    gain_ratio = min(1.0, ukf.transient_nis_ewma / nis_span);
    q_gain = 1.0 + (gain_max - 1.0) * gain_ratio;
    ukf.Q = ukf.Q_base * q_gain;
    ukf.Q_ema = q_gain;
end

function value = get_param_imm(params, field_name, default_value)
    value = default_value;
    if isfield(params, field_name)
        value = params.(field_name);
    end
end

function mu = initial_mode_probabilities(params)
    mu_cv = get_param_imm(params, 'imm_mu_init_CV', 0.5);
    mu_rest = (1 - mu_cv) / 4;
    mu = [mu_cv; mu_rest; mu_rest; mu_rest; mu_rest];
end

function validate_transition_probability(value, name)
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || ...
            value < 0 || value > 1
        error('ukf_imm:invalidProbability', '%s must be a finite scalar in [0, 1].', name);
    end
end

function ukf = keep_prediction(ukf, cache, model)
    switch model
        case 'cv'
            ukf.x = cache.x_pred_cv;
            ukf.P = cache.P_pred_cv;
        case 'ct_slow'
            ukf.x = cache.x_pred_ct_slow;
            ukf.P = cache.P_pred_ct_slow;
        case 'ct_fast'
            ukf.x = cache.x_pred_ct_fast;
            ukf.P = cache.P_pred_ct_fast;
        case 'ctr_fast'
            ukf.x = cache.x_pred_ctr_fast;
            ukf.P = cache.P_pred_ctr_fast;
        case 'ctr_slow'
            ukf.x = cache.x_pred_ctr_slow;
            ukf.P = cache.P_pred_ctr_slow;
    end
end
