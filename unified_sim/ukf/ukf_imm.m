% =========================================================================
% ukf_imm.m — IMM（交互多模型）UKF 滤波器
% =========================================================================
% 【功能概述】
%   封装 CV（匀速）和 CT（协调转弯）双模型 IMM 滤波的纯数学逻辑。
%   对外暴露与 ukf_jichu / ukf_zishiying 完全一致的 action 接口：
%     create → init → prepare → update
%   内部维护两个独立 UKF 实例 + 模型概率 + Markov 转移矩阵。
%
% 【IMM 循环（每帧）】
%   prepare:
%     1. 模型混合（Mixing）  — 各模型用混合初始状态
%     2. 双模型独立预测       — CV/CT 各自 UKF prepare
%     3. 组合输出             — 返回 mu 加权组合给 tracker 做关联
%   update:
%     1. 重构加权量测         — z_weighted = innov_w + z_pred_comb
%     2. 各模型新息分解       — innov_cv, innov_ct
%     3. 各模型独立更新       — 委托 ukf_jichu('update', ...)
%     4. Pd-IPDA 似然度      — 文献: Musicki 2008, IEEE T-AES
%     5. 贝叶斯概率更新       — mu_new ∝ L ⊙ (Pi' × mu)
%     6. 概率钳位 [0.02,0.95]
%     7. 状态组合             — x_comb = Σ mu_i * x_i
%     8. 自适应 Q            — 各模型 NIS 模糊 Q
%
% 【数学模型】
%   CV: F_CV = [1 Δt 0 0; 0 1 0 0; 0 0 1 Δt; 0 0 0 1]
%   CT: F_CT(ω) = [1, sin(ωΔt)/ω, 0, -(1-cos(ωΔt))/ω; ...]
%   Markov Pi: [p_cc, 1-p_cc; 1-p_tt, p_tt] 默认 [0.90, 0.10; 0.10, 0.90]
%
% 【统一接口】
%   ukf = ukf_imm('create',  params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
%   ukf = ukf_imm('init',    ukf, meas1, meas2)
%   [x_pred, z_pred, P_zz, ukf] = ukf_imm('prepare', ukf)
%   [lon, lat, ukf] = ukf_imm('update',  ukf, innov_w)  % innov_w=[]→纯预测
% =========================================================================

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
            error('ukf_imm: unknown action ''%s''', action);
    end
end


% =========================================================================
% create_imm — 创建 IMM UKF 模板
% params 需含: imm_turn_rate_rad_per_sec（CT 转弯率 rad/s）
% =========================================================================
function imm = create_imm(params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
    % ---- 第1部分：保存基础参数 ----
    imm.params = params;
    imm.radar_lon = radar_lon;
    imm.radar_lat = radar_lat;
    imm.tx_lon = tx_lon;
    imm.tx_lat = tx_lat;
    imm.dt = dt;

    % ---- 第2部分：创建 CV 模型 UKF ----
    ukf_cv = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);

    % ---- 第3部分：创建 CT 模型 UKF ----
    ukf_ct = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_ct.model_type = 'CT';
    if isfield(params, 'imm_turn_rate_rad_per_sec')
        ukf_ct.turn_rate_rad_per_sec = params.imm_turn_rate_rad_per_sec;
    else
        ukf_ct.turn_rate_rad_per_sec = 1.0 * pi / 180;  % 默认 1°/s
    end

    imm.ukf_cv = ukf_cv;
    imm.ukf_ct = ukf_ct;

    % ---- 第4部分：Markov 转移概率矩阵 ----
    if isfield(params, 'imm_Pi_CV_to_CT') && isfield(params, 'imm_Pi_CT_to_CV')
        p_cv_ct = params.imm_Pi_CV_to_CT;
        p_ct_cv = params.imm_Pi_CT_to_CV;
        adapt_mode_local = '3in1';
        if isfield(params, 'imm_adapt_mode'), adapt_mode_local = params.imm_adapt_mode; end
        if strcmp(adapt_mode_local, '3in1')
            if isfield(params, 'imm_slow_Pi_CV_to_CT'), p_cv_ct = params.imm_slow_Pi_CV_to_CT; end
            if isfield(params, 'imm_slow_Pi_CT_to_CV'), p_ct_cv = params.imm_slow_Pi_CT_to_CV; end
        end
        imm.Pi = [1-p_cv_ct, p_cv_ct;
                  p_ct_cv, 1-p_ct_cv];
    else
        imm.Pi = [0.90, 0.10;
                  0.10, 0.90];
    end

    % ---- 第5部分：初始模型概率 ----
    if isfield(params, 'imm_mu_init_CV')
        mu_cv = params.imm_mu_init_CV;
        imm.mu = [mu_cv; 1 - mu_cv];
    else
        imm.mu = [0.5; 0.5];
    end

    % ---- 第6部分：IMM-IPDA 检测参数（Musicki 2008） ----
    imm.Pd = params.detection_probability;
    imm.Pg = params.pda_pd_gate;
    imm.Pd_Pg = imm.Pd * imm.Pg;
    imm.L_no_det = 1.0 - imm.Pd_Pg;

    % ---- 第7部分：概率钳位 ----
    imm.mu_min = 0.02;
    imm.mu_max = 0.95;

    % ---- 第8部分：模型数量 ----
    imm.M = 2;

    % ---- 第9部分：滤波器能力标记 ----
    imm.filter_type = 'imm';
    imm.imm_adapt_mode = get_imm_adapt_mode(imm);
    imm.capability = struct('adaptive_q', strcmp(imm.imm_adapt_mode, '3in1'), ...
                            'imm', true, ...
                            'models', {{'CV', 'CT'}});

    % ---- 第10部分：初始化标志 ----
    imm.initialized = false;
    imm.cache = [];
end


% =========================================================================
% init_imm — 两点差分初始化两个 UKF
% =========================================================================
function imm = init_imm(imm, meas1, meas2)
    imm.ukf_cv = ukf_jichu('init', imm.ukf_cv, meas1, meas2);
    imm.ukf_cv.dt = imm.dt;
    imm.ukf_cv.initialized = true;
    imm.ukf_cv.Q_base = imm.ukf_cv.Q;
    imm.ukf_cv.Q_ema = 1.0;
    imm.ukf_cv.transient_nis_ewma = 0.0;
    imm.ukf_cv.nis_history = [];  % 重起始时清空

    imm.ukf_ct = ukf_jichu('init', imm.ukf_ct, meas1, meas2);
    imm.ukf_ct.dt = imm.dt;
    imm.ukf_ct.initialized = true;
    imm.ukf_ct.Q_base = imm.ukf_ct.Q;
    if strcmp(get_imm_adapt_mode(imm), '3in1')
        imm.ukf_ct.Q = imm.ukf_ct.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
    end
    imm.ukf_ct.Q_ema = 1.0;
    imm.ukf_ct.nis_history = [];  % 重起始时清空

    imm.mu = [0.5; 0.5];
    imm.imm_adapt_mode = get_imm_adapt_mode(imm);
    imm.capability.adaptive_q = strcmp(imm.imm_adapt_mode, '3in1');
    imm.initialized = true;
    imm.nis_history = [];     % 镜像 CV 的 NIS，供诊断代码读取
    imm.mu_history = zeros(0, 2);  % 模型概率历史 [n_frames × 2]
    imm.x = imm.ukf_cv.x;     % 顶层组合状态（供时间对齐/融合）
    imm.P = imm.ukf_cv.P;     % 顶层组合协方差
    imm.Q = imm.ukf_cv.Q;     % 代表性过程噪声（供时间对齐）
end


% =========================================================================
% prepare_imm — IMM 预测步：混合 → 双预测 → 组合
% 返回 7 个输出（与 ukf_jichu.prepare 接口兼容）
% tracker 取用: x_pred(输出1), z_pred(输出4), P_zz(输出6), imm(输出7)
% =========================================================================
function [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, imm] = prepare_imm(imm)
    M = imm.M;
    Pi = imm.Pi;
    mu = imm.mu;
    ukf_cv = imm.ukf_cv;
    ukf_ct = imm.ukf_ct;

    % ---- Step 1: 模型混合（Mixing） ----
    c_bar = Pi' * mu;
    mu_mix = zeros(M, M);
    for i = 1:M
        for j = 1:M
            mu_mix(i, j) = Pi(i, j) * mu(i) / max(c_bar(j), 1e-12);
        end
    end

    ukf_models = {ukf_cv, ukf_ct};
    x_mix = {zeros(4,1), zeros(4,1)};
    P_mix = {zeros(4,4), zeros(4,4)};
    for j = 1:M
        for i = 1:M
            x_mix{j} = x_mix{j} + mu_mix(i, j) * ukf_models{i}.x;
        end
        for i = 1:M
            dx = ukf_models{i}.x - x_mix{j};
            P_mix{j} = P_mix{j} + mu_mix(i, j) * (ukf_models{i}.P + dx * dx');
        end
    end
    imm.ukf_cv.x = x_mix{1};  imm.ukf_cv.P = regularize_cov_imm(P_mix{1});
    imm.ukf_ct.x = x_mix{2};  imm.ukf_ct.P = regularize_cov_imm(P_mix{2});

    % ---- Step 1.5: 自适应 Q 在预测前施加，使 Q 变化影响当前帧的似然度 ----
    if isfield(imm, 'life_count')
        imm.ukf_cv.life_count = imm.life_count;
        imm.ukf_ct.life_count = imm.life_count;
    end
    if isfield(imm.params, 'use_fuzzy_adaptive') && imm.params.use_fuzzy_adaptive
        adapt_mode = get_imm_adapt_mode(imm);

        if strcmp(adapt_mode, '3in1')
            imm.ukf_cv = apply_transient_q_imm(imm.ukf_cv, imm.params);
            imm.ukf_ct.Q = imm.ukf_ct.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ct.Q_ema = 1.0;
        elseif strcmp(adapt_mode, 'fuzzy_only')
            imm.ukf_cv = adapt_q(imm.ukf_cv, imm.params, 'fuzzy_only');
            imm.ukf_ct = adapt_q(imm.ukf_ct, imm.params, 'fuzzy_only');
        end
    end

    % ---- Step 2: 各模型独立预测 ----
    [x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, imm.ukf_cv] = ...
        ukf_jichu('prepare', imm.ukf_cv);
    [x_pred_ct, P_pred_ct, X_pred_ct, z_pred_ct, Z_pred_ct, P_zz_ct, imm.ukf_ct] = ...
        ukf_jichu('prepare', imm.ukf_ct);

    % ---- Step 3: 计算组合预测（内部使用） ----
    x_pred_comb = mu(1) * x_pred_cv + mu(2) * x_pred_ct;
    P_pred_comb = combine_cov_imm({x_pred_cv, x_pred_ct}, {P_pred_cv, P_pred_ct}, mu, x_pred_comb);
    z_pred_comb = mu(1) * z_pred_cv + mu(2) * z_pred_ct;
    P_zz_comb = combine_meas_cov_imm({z_pred_cv, z_pred_ct}, {P_zz_cv, P_zz_ct}, mu, z_pred_comb);

    % ---- Step 4: 返回组合预测给 tracker，关联中心与 IMM 后验一致 ----
    x_pred = x_pred_comb;
    z_pred = z_pred_comb;
    P_zz = P_zz_comb;

    % ---- Step 5: 占位输出（接口兼容，tracker 不使用） ----
    P_pred = P_pred_comb;
    X_pred = X_pred_cv;
    Z_pred = Z_pred_cv;

    % ---- Step 6: 缓存所有中间结果 ----
    imm.cache = struct(...
        'x_pred_cv', x_pred_cv, 'x_pred_ct', x_pred_ct, ...
        'P_pred_cv', P_pred_cv, 'P_pred_ct', P_pred_ct, ...
        'X_pred_cv', X_pred_cv, 'X_pred_ct', X_pred_ct, ...
        'z_pred_cv', z_pred_cv, 'z_pred_ct', z_pred_ct, ...
        'Z_pred_cv', Z_pred_cv, 'Z_pred_ct', Z_pred_ct, ...
        'P_zz_cv', P_zz_cv, 'P_zz_ct', P_zz_ct, ...
        'x_pred_comb', x_pred_comb, 'z_pred_comb', z_pred_comb, 'P_zz_comb', P_zz_comb, ...
        'c_bar', c_bar);
end


% =========================================================================
% update_imm — IMM 更新步：双模型更新 → 似然 → 概率 → 组合
% innov_w = PDA 加权新息（相对于组合 z_pred），[] = 纯预测
% =========================================================================
function [lon, lat, imm] = update_imm(imm, innov_w)
    cache = imm.cache;
    nz = imm.ukf_cv.m;

    % ---- 纯预测帧 ----
    if isempty(innov_w)
        % 两个模型均保留预测状态
        imm.ukf_cv = keep_prediction(imm.ukf_cv, cache, 'cv');
        imm.ukf_ct = keep_prediction(imm.ukf_ct, cache, 'ct');
        L_cv = imm.L_no_det;
        L_ct = imm.L_no_det;
    else
        % ---- 重建加权量测（innov_w 相对于 tracker 使用的组合 z_pred） ----
        z_weighted = innov_w + cache.z_pred_comb;

        % ---- 各模型新息 ----
        innov_cv = z_weighted - cache.z_pred_cv;
        innov_ct = z_weighted - cache.z_pred_ct;
        if abs(innov_cv(2)) > 180
            innov_cv(2) = innov_cv(2) - 360 * round(innov_cv(2) / 360);
        end
        if abs(innov_ct(2)) > 180
            innov_ct(2) = innov_ct(2) - 360 * round(innov_ct(2) / 360);
        end

        % ---- 各模型独立更新 ----
        [~, ~, imm.ukf_cv] = ukf_jichu('update', imm.ukf_cv, innov_cv);
        [~, ~, imm.ukf_ct] = ukf_jichu('update', imm.ukf_ct, innov_ct);

        % ---- 记录各模型 NIS ----
        nis_cv_val = innov_cv' * (cache.P_zz_cv \ innov_cv);
        nis_ct_val = innov_ct' * (cache.P_zz_ct \ innov_ct);
        if ~isnan(nis_cv_val)
            imm.ukf_cv.nis_history(end+1) = nis_cv_val;
        end
        if ~isnan(nis_ct_val)
            imm.ukf_ct.nis_history(end+1) = nis_ct_val;
        end
        imm.nis_history = imm.ukf_cv.nis_history;  % 镜像供诊断

        % ---- Pd-IPDA 似然度（Musicki 2008） ----
        log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
        L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
        log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_ct), 1e-30)));
        L_ct = imm.Pd_Pg * exp(log_norm - 0.5 * nis_ct_val);

        % 3in1 模式保持 IMM 原生似然更新，不用自适应 Q 反向改写模型概率。
    end

    % ---- 贝叶斯模型概率更新 ----
    c_total = L_cv * cache.c_bar(1) + L_ct * cache.c_bar(2);
    if c_total > 1e-30
        mu_new = [L_cv * cache.c_bar(1); L_ct * cache.c_bar(2)] / c_total;
    else
        mu_new = imm.mu;
    end
    imm.mu = max(imm.mu_min, min(imm.mu_max, mu_new));
    imm.mu = imm.mu / sum(imm.mu);

    % ---- 组合状态 ----
    x_comb = imm.mu(1) * imm.ukf_cv.x + imm.mu(2) * imm.ukf_ct.x;
    lon = x_comb(1);
    lat = x_comb(3);

    % ---- 更新顶层状态（供时间对齐/融合/诊断使用） ----
    imm.x = x_comb;
    imm.P = combine_cov_imm({imm.ukf_cv.x, imm.ukf_ct.x}, {imm.ukf_cv.P, imm.ukf_ct.P}, imm.mu, x_comb);
    imm.Q = imm.mu(1) * imm.ukf_cv.Q + imm.mu(2) * imm.ukf_ct.Q;
    imm.Q_ema = imm.mu(1) * imm.ukf_cv.Q_ema + imm.mu(2) * imm.ukf_ct.Q_ema;
    imm.mu_history(end+1, :) = imm.mu';
end


function P_comb = combine_cov_imm(x_models, P_models, mu, x_comb)
    P_comb = zeros(size(P_models{1}));
    for i = 1:length(mu)
        dx = x_models{i} - x_comb;
        P_comb = P_comb + mu(i) * (P_models{i} + dx * dx');
    end
    P_comb = regularize_cov_imm(P_comb);
end


function P_reg = regularize_cov_imm(P)
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


% =========================================================================
% keep_prediction — 纯预测：保留模型预测状态
% =========================================================================
function ukf = keep_prediction(ukf, cache, model)
    switch model
        case 'cv'
            ukf.x = cache.x_pred_cv;
            ukf.P = cache.P_pred_cv;
        case 'ct'
            ukf.x = cache.x_pred_ct;
            ukf.P = cache.P_pred_ct;
    end
end
