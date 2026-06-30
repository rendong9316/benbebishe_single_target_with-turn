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

    % ---- 第9部分：初始化标志 ----
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
    imm.ukf_cv.nis_history = [];  % 重起始时清空

    imm.ukf_ct = ukf_jichu('init', imm.ukf_ct, meas1, meas2);
    imm.ukf_ct.dt = imm.dt;
    imm.ukf_ct.initialized = true;
    imm.ukf_ct.Q_base = imm.ukf_ct.Q;
    imm.ukf_ct.Q_ema = 1.0;
    imm.ukf_ct.nis_history = [];  % 重起始时清空

    imm.mu = [0.5; 0.5];
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
    imm.ukf_cv.x = x_mix{1};  imm.ukf_cv.P = P_mix{1};
    imm.ukf_ct.x = x_mix{2};  imm.ukf_ct.P = P_mix{2};

    % ---- Step 2: 各模型独立预测 ----
    [x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, imm.ukf_cv] = ...
        ukf_jichu('prepare', imm.ukf_cv);
    [x_pred_ct, P_pred_ct, X_pred_ct, z_pred_ct, Z_pred_ct, P_zz_ct, imm.ukf_ct] = ...
        ukf_jichu('prepare', imm.ukf_ct);

    % ---- Step 3: 计算组合预测（内部使用） ----
    x_pred_comb = mu(1) * x_pred_cv + mu(2) * x_pred_ct;
    z_pred_comb = mu(1) * z_pred_cv + mu(2) * z_pred_ct;
    P_zz_comb = 0.5 * P_zz_cv + 0.5 * P_zz_ct;

    % ---- Step 4: 返回 CV 模型预测给 tracker（门中心更可靠，不依赖 mu 收敛） ----
    x_pred = x_pred_cv;
    z_pred = z_pred_cv;
    P_zz = P_zz_cv;

    % ---- Step 5: 占位输出（接口兼容，tracker 不使用） ----
    P_pred = P_pred_cv;  % 仅占位
    X_pred = X_pred_cv;  % 仅占位
    Z_pred = Z_pred_cv;  % 仅占位

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
    M = imm.M;
    nz = imm.ukf_cv.m;

    % ---- 纯预测帧 ----
    if isempty(innov_w)
        % 两个模型均保留预测状态
        imm.ukf_cv = keep_prediction(imm.ukf_cv, cache, 'cv');
        imm.ukf_ct = keep_prediction(imm.ukf_ct, cache, 'ct');
        L_cv = imm.L_no_det;
        L_ct = imm.L_no_det;
    else
        % ---- 重建加权量测（innov_w 相对于 tracker 使用的 z_pred，即 CV 预测） ----
        z_weighted = innov_w + cache.z_pred_cv;

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
    imm.P = imm.mu(1) * imm.ukf_cv.P + imm.mu(2) * imm.ukf_ct.P;
    imm.mu_history(end+1, :) = imm.mu';

    % ---- 自适应 Q（仅 CV，life>12，与旧 imm_tracker 完全一致） ----
    if imm.params.use_fuzzy_adaptive && isfield(imm, 'life_count') ...
            && imm.life_count > 12 && isfield(imm.ukf_cv, 'nis_history')
        imm.ukf_cv = apply_fuzzy_adapt_imm(imm.ukf_cv, imm.params);
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


% =========================================================================
% apply_fuzzy_adapt_imm — 模糊自适应 Q（IMM 内部，与 single_track_runner 同步）
% =========================================================================
function ukf = apply_fuzzy_adapt_imm(ukf, params)
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history)
        return;
    end

    nis_history = ukf.nis_history;

    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end

    nis_avg = mean(nis_history);
    nis_ratio = nis_avg / 2.0;

    mu_VS = trimf_val_imm(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val_imm(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val_imm(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val_imm(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val_imm(nis_ratio, 2.5, 4.0, 4.0);

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

    factor_raw = max(0.5, min(4.0, factor_fuzzy));

    ema_eta = 0.20;
    if isfield(params, 'fuzzy_ema_eta'), ema_eta = params.fuzzy_ema_eta; end
    ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;

    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end
end


% =========================================================================
% trimf_val_imm — 三角形隶属函数求值
% =========================================================================
function mu = trimf_val_imm(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
