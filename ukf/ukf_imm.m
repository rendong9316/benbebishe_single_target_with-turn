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
%        利用 Markov 转移概率和当前模型概率，计算混合初始状态
%        x_mix_j = Σ_i μ(i|j) * x_i，其中 μ(i|j) = P_ij * μ_i / c_bar_j
%     2. 双模型独立预测       — CV/CT 各自 UKF prepare
%        混合后的状态分别输入 CV 和 CT 两个 UKF 实例做预测
%     3. 组合输出             — 返回 mu 加权组合给 tracker 做关联
%        状态和量测统计都按当前模型概率 mu 加权组合
%   update:
%     1. 重构加权量测         — z_weighted = innov_w + z_pred_comb
%        将 PDA 加权新息还原为绝对量测值
%     2. 各模型新息分解       — innov_cv, innov_ct
%        计算每个模型相对于自身预测的新息
%     3. 各模型独立更新       — 委托 ukf_jichu('update', ...)
%     4. Pd-IPDA 似然度      — 文献: Musicki 2008, IEEE T-AES
%        考虑检测概率 Pd 和门内概率 Pg 的似然度计算
%     5. 贝叶斯概率更新       — mu_new ∝ L ⊙ (Pi' × mu)
%        用似然度更新模型概率
%     6. 概率钳位 [0.02,0.95]
%        防止概率坍缩到 0 或 1
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
    % 动作分发：将外部 action 请求转发到对应的内部函数
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
% 此函数创建一个完整的 IMM 滤波器结构体，包含：
%   - 两个独立 UKF 实例（CV 模型和 CT 模型）
%   - Markov 转移概率矩阵 Pi
%   - 初始模型概率 mu
%   - IMM-IPDA 检测参数
%   - 概率钳位边界
function imm = create_imm(params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
    % ---- 第1部分：保存基础参数 ----
    % 将雷达位置、发射机位置、时间间隔等存储到结构体中
    imm.params = params;
    imm.radar_lon = radar_lon;
    imm.radar_lat = radar_lat;
    imm.tx_lon = tx_lon;
    imm.tx_lat = tx_lat;
    imm.dt = dt;

    % ---- 第2部分：创建 CV 模型 UKF ----
    % CV（Constant Velocity）模型假设目标匀速直线运动
    ukf_cv = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);

    % ---- 第3部分：创建 CT 模型 UKF ----
    % CT（Constant Turn）模型假设目标以恒定转弯率运动
    ukf_ct = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
    ukf_ct.model_type = 'CT';  % 标记为 CT 模型
    % 读取转弯率参数，默认 1°/s（常用于民航目标的典型转弯率）
    if isfield(params, 'imm_turn_rate_rad_per_sec')
        ukf_ct.turn_rate_rad_per_sec = params.imm_turn_rate_rad_per_sec;
    else
        ukf_ct.turn_rate_rad_per_sec = 1.0 * pi / 180;  % 默认 1°/s
    end

    imm.ukf_cv = ukf_cv;
    imm.ukf_ct = ukf_ct;

    % ---- 第4部分：Markov 转移概率矩阵 ----
    % Pi 是一个 2x2 矩阵，描述模型间转移概率：
    %   Pi(1,1) = P(CV→CV)  当前模型为 CV 时继续保持 CV 的概率
    %   Pi(1,2) = P(CV→CT)  当前模型为 CV 时转为 CT 的概率
    %   Pi(2,1) = P(CT→CV)  当前模型为 CT 时转为 CV 的概率
    %   Pi(2,2) = P(CT→CT)  当前模型为 CT 时继续保持 CT 的概率
    if isfield(params, 'imm_Pi_CV_to_CT') && isfield(params, 'imm_Pi_CT_to_CV')
        p_cv_ct = params.imm_Pi_CV_to_CT;
        p_ct_cv = params.imm_Pi_CT_to_CV;
        adapt_mode_local = '3in1';
        if isfield(params, 'imm_adapt_mode'), adapt_mode_local = params.imm_adapt_mode; end
        % 3in1 模式使用慢速转移概率（减少模型切换频率）
        if strcmp(adapt_mode_local, '3in1')
            if isfield(params, 'imm_slow_Pi_CV_to_CT'), p_cv_ct = params.imm_slow_Pi_CV_to_CT; end
            if isfield(params, 'imm_slow_Pi_CT_to_CV'), p_ct_cv = params.imm_slow_Pi_CT_to_CV; end
        end
        imm.Pi = [1-p_cv_ct, p_cv_ct;
                  p_ct_cv, 1-p_ct_cv];
    else
        % 默认转移矩阵：90% 概率保持当前模型，10% 概率切换到另一模型
        imm.Pi = [0.90, 0.10;
                  0.10, 0.90];
    end

    % ---- 第5部分：初始模型概率 ----
    % mu 是每个模型的先验概率，初始化为均匀分布或从参数读取
    if isfield(params, 'imm_mu_init_CV')
        mu_cv = params.imm_mu_init_CV;
        imm.mu = [mu_cv; 1 - mu_cv];
    else
        % 默认：CV 和 CT 各占 50% 先验概率
        imm.mu = [0.5; 0.5];
    end

    % ---- 第6部分：IMM-IPDA 检测参数（Musicki 2008） ----
    % IPDA（Integrated Probabilistic Data Association）将检测概率
    % 和门内概率纳入似然度计算，比传统 IMM 更精确
    imm.Pd = params.detection_probability;      % 目标检测概率（通常 0.6-0.99）
    imm.Pg = params.pda_pd_gate;                % 门内概率（通常 0.95-0.99）
    imm.Pd_Pg = imm.Pd * imm.Pg;                % 联合检测概率
    imm.L_no_det = 1.0 - imm.Pd_Pg;             % 未检测时的似然度基线

    % ---- 第7部分：概率钳位 ----
    % 防止模型概率坍缩到 0 或 1（一旦某模型概率为 0，就永远无法恢复）
    imm.mu_min = 0.02;   % 最小概率 2%，保证每个模型都有生存机会
    imm.mu_max = 0.95;   % 最大概率 95%，保留一定的模型探索空间

    % ---- 第8部分：模型数量 ----
    imm.M = 2;  % 双模型 IMM

    % ---- 第9部分：滤波器能力标记 ----
    % 记录滤波器支持的自适应模式和模型信息
    imm.filter_type = 'imm';
    imm.imm_adapt_mode = get_imm_adapt_mode(imm);
    imm.capability = struct('adaptive_q', strcmp(imm.imm_adapt_mode, '3in1'), ...
                            'imm', true, ...
                            'models', {{'CV', 'CT'}});

    % ---- 第10部分：初始化标志 ----
    imm.initialized = false;
    imm.cache = [];  % 缓存 prepare 阶段的中间结果，供 update 使用
end


% =========================================================================
% init_imm — 两点差分初始化两个 UKF
% =========================================================================
% 使用两次量测初始化 CV 和 CT 两个 UKF 模型的状态和协方差
% 同时设置自适应 Q 相关的初始状态
function imm = init_imm(imm, meas1, meas2)
    % 初始化 CV 模型 UKF
    imm.ukf_cv = ukf_jichu('init', imm.ukf_cv, meas1, meas2);
    imm.ukf_cv.dt = imm.dt;
    imm.ukf_cv.initialized = true;
    imm.ukf_cv.Q_base = imm.ukf_cv.Q;  % 保存基线 Q 用于自适应缩放
    imm.ukf_cv.Q_ema = 1.0;            % EMA 初始值为 1.0（无缩放）
    imm.ukf_cv.transient_nis_ewma = 0.0;  % 瞬态 NIS 的 EWMA 初始值
    imm.ukf_cv.nis_history = [];  % 重起始时清空 NIS 历史

    % 初始化 CT 模型 UKF
    imm.ukf_ct = ukf_jichu('init', imm.ukf_ct, meas1, meas2);
    imm.ukf_ct.dt = imm.dt;
    imm.ukf_ct.initialized = true;
    imm.ukf_ct.Q_base = imm.ukf_ct.Q;
    % 3in1 模式下 CT 模型使用固定 Q 缩放（CT 模型本身已考虑机动）
    if strcmp(get_imm_adapt_mode(imm), '3in1')
        imm.ukf_ct.Q = imm.ukf_ct.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
    end
    imm.ukf_ct.Q_ema = 1.0;
    imm.ukf_ct.nis_history = [];  % 重起始时清空 NIS 历史

    % 重置模型概率为均匀分布
    imm.mu = [0.5; 0.5];
    imm.imm_adapt_mode = get_imm_adapt_mode(imm);
    imm.capability.adaptive_q = strcmp(imm.imm_adapt_mode, '3in1');
    imm.initialized = true;
    imm.nis_history = [];     % 镜像 CV 的 NIS，供诊断代码读取
    imm.mu_history = zeros(0, 2);  % 模型概率历史 [n_frames × 2]
    % 顶层状态初始化为 CV 模型的状态（初始化后 CV 和 CT 状态相同）
    imm.x = imm.ukf_cv.x;     % 顶层组合状态（供时间对齐/融合）
    imm.P = imm.ukf_cv.P;     % 顶层组合协方差
    imm.Q = imm.ukf_cv.Q;     % 代表性过程噪声（供时间对齐）
end


% =========================================================================
% prepare_imm — IMM 预测步：混合 → 双预测 → 组合
% 返回 7 个输出（与 ukf_jichu.prepare 接口兼容）
% tracker 取用: x_pred(输出1), z_pred(输出4), P_zz(输出6), imm(输出7)
% =========================================================================
% IMM 预测的核心三步曲：
%   1. 模型混合：用 Markov 转移概率计算混合初始状态
%   2. 双模型独立预测：CV 和 CT 各自做 UKF 预测
%   3. 组合输出：按模型概率加权组合
function [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, imm] = prepare_imm(imm)
    M = imm.M;           % 模型数量 = 2
    Pi = imm.Pi;         % Markov 转移矩阵
    mu = imm.mu;         % 当前模型概率
    ukf_cv = imm.ukf_cv;  % CV 模型 UKF 实例
    ukf_ct = imm.ukf_ct;  % CT 模型 UKF 实例

    % ---- Step 1: 模型混合（Mixing） ----
    % 计算混合概率 mu_mix(i,j) = P(model_i | model_j, prior)
    % 这是 IMM 算法的核心：根据 Markov 转移概率和先验模型概率，
    % 计算每个模型在给定当前模型条件下的后验混合概率
    c_bar = Pi' * mu;  % 归一化常数：c_bar(j) = Σ_i Pi(i,j) * mu(i)
    mu_mix = zeros(M, M);
    for i = 1:M
        for j = 1:M
            % mu_mix(i,j) = P(model_i 来自 model_j)
            % 注意：分母用 max(c_bar(j), 1e-12) 防止除零
            mu_mix(i, j) = Pi(i, j) * mu(i) / max(c_bar(j), 1e-12);
        end
    end

    % 计算混合初始状态 x_mix 和 P_mix
    % x_mix_j = Σ_i mu_mix(i,j) * x_i
    % P_mix_j = Σ_i mu_mix(i,j) * (P_i + dx_i * dx_i')
    % 第二个公式包含了模型间的离散项，确保混合协方差不会低估不确定性
    ukf_models = {ukf_cv, ukf_ct};
    x_mix = {zeros(4,1), zeros(4,1)};
    P_mix = {zeros(4,4), zeros(4,4)};
    for j = 1:M
        % 计算混合状态（加权平均）
        for i = 1:M
            x_mix{j} = x_mix{j} + mu_mix(i, j) * ukf_models{i}.x;
        end
        % 计算混合协方差（包含离散项）
        for i = 1:M
            dx = ukf_models{i}.x - x_mix{j};
            P_mix{j} = P_mix{j} + mu_mix(i, j) * (ukf_models{i}.P + dx * dx');
        end
    end
    % 将混合状态赋回各模型 UKF 实例
    imm.ukf_cv.x = x_mix{1};  imm.ukf_cv.P = regularize_cov_imm(P_mix{1});
    imm.ukf_ct.x = x_mix{2};  imm.ukf_ct.P = regularize_cov_imm(P_mix{2});

    % ---- Step 1.5: 自适应 Q 在预测前施加，使 Q 变化影响当前帧的似然度 ----
    % 在预测之前调整 Q，可以让预测阶段的过程噪声也反映自适应结果
    if isfield(imm, 'life_count')
        imm.ukf_cv.life_count = imm.life_count;
        imm.ukf_ct.life_count = imm.life_count;
    end
    if isfield(imm.params, 'use_fuzzy_adaptive') && imm.params.use_fuzzy_adaptive
        adapt_mode = get_imm_adapt_mode(imm);

        if strcmp(adapt_mode, '3in1')
            % 3in1 模式：CV 模型使用瞬态增益自适应，CT 模型使用固定 Q 缩放
            imm.ukf_cv = apply_transient_q_imm(imm.ukf_cv, imm.params);
            imm.ukf_ct.Q = imm.ukf_ct.Q_base * get_param_imm(imm.params, 'imm_ct_fixed_Q_scale', 1.8);
            imm.ukf_ct.Q_ema = 1.0;
        elseif strcmp(adapt_mode, 'fuzzy_only')
            % fuzzy_only 模式：两个模型都使用模糊自适应
            imm.ukf_cv = adapt_q(imm.ukf_cv, imm.params, 'fuzzy_only');
            imm.ukf_ct = adapt_q(imm.ukf_ct, imm.params, 'fuzzy_only');
        end
    end

    % ---- Step 2: 各模型独立预测 ----
    % 将混合后的状态分别输入 CV 和 CT 的 UKF 预测
    [x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, imm.ukf_cv] = ...
        ukf_jichu('prepare', imm.ukf_cv);
    [x_pred_ct, P_pred_ct, X_pred_ct, z_pred_ct, Z_pred_ct, P_zz_ct, imm.ukf_ct] = ...
        ukf_jichu('prepare', imm.ukf_ct);

    % ---- Step 3: 计算组合预测（内部使用） ----
    % 状态组合：按模型概率加权
    x_pred_comb = mu(1) * x_pred_cv + mu(2) * x_pred_ct;
    % 协方差组合：包含模型间离散项，确保不低估不确定性
    P_pred_comb = combine_cov_imm({x_pred_cv, x_pred_ct}, {P_pred_cv, P_pred_ct}, mu, x_pred_comb);
    % 量测预测组合
    z_pred_comb = mu(1) * z_pred_cv + mu(2) * z_pred_ct;
    % 量测协方差组合
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
    % 缓存用于 update 阶段，避免重复计算
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
% IMM 更新的核心流程：
%   1. 对各模型分别计算新息并执行 Kalman 更新
%   2. 计算各模型的似然度（Pd-IPDA）
%   3. 贝叶斯更新模型概率
%   4. 组合输出最终状态
function [lon, lat, imm] = update_imm(imm, innov_w)
    cache = imm.cache;
    nz = imm.ukf_cv.m;  % 量测维度

    % ---- 纯预测帧 ----
    % 没有新息（innov_w 为空），说明本帧没有检测到目标
    if isempty(innov_w)
        % 两个模型均保留预测状态，不做 Kalman 更新
        imm.ukf_cv = keep_prediction(imm.ukf_cv, cache, 'cv');
        imm.ukf_ct = keep_prediction(imm.ukf_ct, cache, 'ct');
        % 似然度使用未检测概率
        L_cv = imm.L_no_det;
        L_ct = imm.L_no_det;
    else
        % ---- 重建加权量测（innov_w 相对于 tracker 使用的组合 z_pred） ----
        % 将相对新息还原为绝对量测值
        z_weighted = innov_w + cache.z_pred_comb;

        % ---- 各模型新息 ----
        % 新息 = 实际量测 - 模型预测量测
        innov_cv = z_weighted - cache.z_pred_cv;
        innov_ct = z_weighted - cache.z_pred_ct;
        % 方位角处理：跨越 180/-180 边界时需要修正（角度环绕问题）
        if abs(innov_cv(2)) > 180
            innov_cv(2) = innov_cv(2) - 360 * round(innov_cv(2) / 360);
        end
        if abs(innov_ct(2)) > 180
            innov_ct(2) = innov_ct(2) - 360 * round(innov_ct(2) / 360);
        end

        % ---- 各模型独立更新 ----
        % 委托 ukf_jichu 的 update 函数执行纯 Kalman 数学
        [~, ~, imm.ukf_cv] = ukf_jichu('update', imm.ukf_cv, innov_cv);
        [~, ~, imm.ukf_ct] = ukf_jichu('update', imm.ukf_ct, innov_ct);

        % ---- 记录各模型 NIS ----
        % NIS = 新息' * P_zz_inv * 新息，服从 chi^2 分布
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
        % 似然度 = Pd * Pg * N(z; z_pred, P_zz) + (1-Pd*Pg)
        % 其中 N 是高斯密度函数，nis 是马氏距离
        % log_norm 是高斯密度的对数归一化常数
        log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
        L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
        log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_ct), 1e-30)));
        L_ct = imm.Pd_Pg * exp(log_norm - 0.5 * nis_ct_val);

        % 3in1 模式保持 IMM 原生似然更新，不用自适应 Q 反向改写模型概率。
    end

    % ---- 贝叶斯模型概率更新 ----
    % mu_new(j) ∝ L_j * Σ_i Pi(i,j) * mu_i
    % 即：新概率 = 似然度 × 混合概率，再归一化
    c_total = L_cv * cache.c_bar(1) + L_ct * cache.c_bar(2);
    if c_total > 1e-30
        mu_new = [L_cv * cache.c_bar(1); L_ct * cache.c_bar(2)] / c_total;
    else
        mu_new = imm.mu;
    end
    % 概率钳位：防止概率坍缩到 0 或 1
    imm.mu = max(imm.mu_min, min(imm.mu_max, mu_new));
    % 归一化：确保概率之和为 1
    imm.mu = imm.mu / sum(imm.mu);

    % ---- 组合状态 ----
    % 按更新后的模型概率加权组合
    x_comb = imm.mu(1) * imm.ukf_cv.x + imm.mu(2) * imm.ukf_ct.x;
    lon = x_comb(1);
    lat = x_comb(3);

    % ---- 更新顶层状态（供时间对齐/融合/诊断使用） ----
    imm.x = x_comb;
    % 组合协方差（包含模型间离散项）
    imm.P = combine_cov_imm({imm.ukf_cv.x, imm.ukf_ct.x}, {imm.ukf_cv.P, imm.ukf_ct.P}, imm.mu, x_comb);
    % 组合过程噪声
    imm.Q = imm.mu(1) * imm.ukf_cv.Q + imm.mu(2) * imm.ukf_ct.Q;
    imm.Q_ema = imm.mu(1) * imm.ukf_cv.Q_ema + imm.mu(2) * imm.ukf_ct.Q_ema;
    imm.mu_history(end+1, :) = imm.mu';  % 记录模型概率历史
end


% =========================================================================
% combine_cov_imm — 组合协方差矩阵
% =========================================================================
% P_comb = Σ mu_i * (P_i + dx_i * dx_i')
% 包含模型间离散项 dx_i * dx_i'，保证组合协方差不会低估不确定性
% 这是 IMM 算法的关键：如果不加离散项，组合协方差会小于任一模型协方差，
% 导致滤波器过度自信而发散
function P_comb = combine_cov_imm(x_models, P_models, mu, x_comb)
    P_comb = zeros(size(P_models{1}));
    for i = 1:length(mu)
        dx = x_models{i} - x_comb;
        P_comb = P_comb + mu(i) * (P_models{i} + dx * dx');
    end
    P_comb = regularize_cov_imm(P_comb);
end


% =========================================================================
% regularize_cov_imm — IMM 专用协方差正则化
% =========================================================================
% 确保协方差矩阵满足：
%   1. 对称性（浮点误差可能导致轻微不对称）
%   2. 正定性（所有特征值 > min_eig）
%   3. 无 NaN/Inf
function P_reg = regularize_cov_imm(P)
    % 强制对称化
    P_reg = (P + P') / 2;
    % NaN/Inf 守卫：如果出现异常值，直接返回单位矩阵的小倍数
    if any(isnan(P_reg(:))) || any(isinf(P_reg(:)))
        P_reg = eye(size(P_reg)) * 1e-6;
        return;
    end
    min_eig = 1e-12;
    % 特征值分解，检查并修正负特征值
    [V, D] = eig(P_reg);
    d = diag(D);
    d(d < min_eig) = min_eig;
    P_reg = V * diag(d) * V';
    % 再次对称化（特征值重构后可能轻微不对称）
    P_reg = (P_reg + P_reg') / 2;
end


% =========================================================================
% combine_meas_cov_imm — 组合量测协方差矩阵
% =========================================================================
% 与 combine_cov_imm 类似，但是针对量测空间（而非状态空间）
% 特殊处理方位角的环绕问题
function P_zz_comb = combine_meas_cov_imm(z_models, P_zz_models, mu, z_comb)
    P_zz_comb = zeros(size(P_zz_models{1}));
    for i = 1:length(mu)
        dz = z_models{i} - z_comb;
        % 方位角（第二个分量）的环绕修正
        if length(dz) >= 2 && abs(dz(2)) > 180
            dz(2) = dz(2) - 360 * round(dz(2) / 360);
        end
        P_zz_comb = P_zz_comb + mu(i) * (P_zz_models{i} + dz * dz');
    end
    P_zz_comb = (P_zz_comb + P_zz_comb') / 2;
end


% =========================================================================
% get_imm_adapt_mode — 获取 IMM 自适应模式
% =========================================================================
% 自适应模式决定 IMM 内部的 Q 调整策略：
%   '3in1'     - 3in1 模式：CV 用瞬态增益，CT 用固定缩放
%   'fuzzy_only' - 纯模糊自适应模式
%   'none'     - 无自适应
function adapt_mode = get_imm_adapt_mode(imm)
    adapt_mode = '3in1';  % 默认模式
    if isfield(imm, 'imm_adapt_mode')
        adapt_mode = imm.imm_adapt_mode;
    elseif isfield(imm, 'params') && isfield(imm.params, 'imm_adapt_mode')
        adapt_mode = imm.params.imm_adapt_mode;
    end
end


% =========================================================================
% apply_transient_q_imm — CV 模型瞬态增益自适应 Q
% =========================================================================
% 基于 NIS 的 EWMA 平滑值，在 NIS 超过阈值时渐进提升 Q。
% 这是 3in1 模式的核心创新：CV 模型在机动瞬态时自动提升过程噪声。
% 与 adapt_q 不同，此函数不依赖生命周期检查，专门在 prepare 阶段调用。
function ukf = apply_transient_q_imm(ukf, params)
    % 确保 Q_base 存在
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end
    % 重置 Q 为基线
    ukf.Q = ukf.Q_base;
    ukf.Q_ema = 1.0;

    % 没有 NIS 历史则无法自适应
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history)
        return;
    end

    nis_now = ukf.nis_history(end);
    % 读取瞬态自适应参数
    nis_start = get_param_imm(params, 'imm_transient_nis_start', 3.0);   % 触发阈值下限
    nis_full = get_param_imm(params, 'imm_transient_nis_full', 12.0);   % 触发阈值上限
    gain_max = get_param_imm(params, 'imm_transient_gain_max', 5.0);    % 最大增益倍数
    ewma_alpha = get_param_imm(params, 'imm_transient_ewma_alpha', 0.65); % EWMA 系数

    % 初始化 EWMA 状态
    if ~isfield(ukf, 'transient_nis_ewma') || isempty(ukf.transient_nis_ewma)
        ukf.transient_nis_ewma = 0.0;
    end

    % 计算超出的 NIS 量（低于 nis_start 的部分不计入）
    nis_excess = max(0.0, nis_now - nis_start);
    % EWMA 平滑：平滑后的超出量
    ukf.transient_nis_ewma = ewma_alpha * nis_excess + (1.0 - ewma_alpha) * ukf.transient_nis_ewma;
    if ukf.transient_nis_ewma <= 0
        return;
    end

    % 计算增益比例：0~1 之间线性插值
    nis_span = max(nis_full - nis_start, 1e-6);
    gain_ratio = min(1.0, ukf.transient_nis_ewma / nis_span);
    % 计算最终增益：1.0（基线）到 gain_max（最大值）
    q_gain = 1.0 + (gain_max - 1.0) * gain_ratio;
    ukf.Q = ukf.Q_base * q_gain;
    ukf.Q_ema = q_gain;
end


% =========================================================================
% get_param_imm — 安全参数读取
% =========================================================================
% 安全地从结构体中读取字段，如果字段不存在则返回默认值
function value = get_param_imm(params, field_name, default_value)
    value = default_value;
    if isfield(params, field_name)
        value = params.(field_name);
    end
end


% =========================================================================
% keep_prediction — 纯预测：保留模型预测状态
% =========================================================================
% 当没有新息时（纯预测帧），各模型保留 prepare 阶段计算的预测状态
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
