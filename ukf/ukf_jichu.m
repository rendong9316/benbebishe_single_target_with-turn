% =========================================================================
% ukf_jichu.m
% =========================================================================
% 功能概要：
%   UKF（无迹卡尔曼滤波）基础模块 — 纯滤波数学，不含任何关联逻辑。
%   采用 action dispatcher 模式，所有函数均为过程式。
%
% 公共 actions：
%   ukf = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
%   ukf = ukf_jichu('init', ukf, meas1, meas2)
%   [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf)
%      预测 + 量测统计（供上层关联模块使用）
%   [lon, lat, ukf] = ukf_jichu('update', ukf, innov_w)
%      纯 Kalman 数学：P_xz → K → x_new → P_new。不含任何检测/关联/门限。
%      innov_w=[] 表示纯预测帧，跳过更新仅保留预测状态。
%      所有中间量从 ukf.cache 内部读取（由 prepare 写入）。
%   [x_pred, P_pred, X_pred, ukf] = ukf_jichu('predict', ukf)
%   z = ukf_jichu('measurement', ukf, x)
% =========================================================================

function varargout = ukf_jichu(action, varargin)
    % 动作分发器：将外部 action 请求转发到对应的内部函数
    switch action
        case 'create'
            varargout{1} = create_ukf(varargin{:});
        case 'init'
            varargout{1} = init_ukf(varargin{:});
        case 'prepare'
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}, ...
             varargout{5}, varargout{6}, varargout{7}] = prepare_ukf(varargin{:});
        case 'update'
            % varargin = {ukf, innov_w} — innov_w=[] 表示纯预测
            [varargout{1}, varargout{2}, varargout{3}] = update_with_innov(varargin{:});
        case 'predict'
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = predict_step_ukf(varargin{:});
        case 'measurement'
            varargout{1} = measurement_ukf(varargin{:});
        otherwise
            error('ukf_jichu: unknown action ''%s''', action);
    end
end


% =========================================================================
% create_ukf — 创建 UKF 滤波器模板
% =========================================================================
% 初始化 UKF 的所有核心参数：
%   - Sigma 点权重（Wm, Wc）
%   - 量测噪声协方差 R
%   - 过程噪声协方差 Q
%   - 初始状态协方差 P
function ukf = create_ukf(params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
    % 第1部分：保存基础参数到结构体
    ukf.params = params;
    ukf.radar_lon = radar_lon;
    ukf.radar_lat = radar_lat;
    ukf.tx_lon = tx_lon;
    ukf.tx_lat = tx_lat;
    ukf.dt = dt;

    % 第2部分：UKF 核心参数计算
    % n = 4：状态维度（经度、经度速度、纬度、纬度速度）
    n = 4;
    % alpha：UKF 缩放参数，控制 Sigma 点的分布宽度（通常 1e-3 ~ 1）
    alpha = params.ukf_alpha;
    % beta：UKF 参数，对高斯分布最优值为 2
    beta  = params.ukf_beta;
    % kappa：UKF 缩放参数，通常设为 0 或 3-n
    kappa = params.ukf_kappa;
    % lambda = alpha^2 * (n + kappa) - n，Sigma 点的总缩放参数
    lam = alpha^2 * (n + kappa) - n;

    ukf.n   = n;   % 状态维度 = 4
    ukf.m   = 3;   % 量测维度 = 3（距离、方位、多普勒）
    ukf.lam = lam;

    % 第3部分：计算 Sigma 点权重
    % Wm：均值权重，Wc：协方差权重
    % 2n+1 个 Sigma 点（1 个中心点 + 2n 个方向点）
    % 默认情况下，每个点的权重 = 1/(2*(n+lam))
    ukf.Wm = ones(2 * n + 1, 1) / (2.0 * (n + lam));
    ukf.Wc = ones(2 * n + 1, 1) / (2.0 * (n + lam));
    % 中心点（索引 1）的权重需要额外加上 (1-alpha^2+beta)，
    % 以便 beta 可以编码关于目标分布高阶信息的先验知识
    % 对于高斯分布，beta=2 是最优的
    ukf.Wm(1) = lam / (n + lam);
    ukf.Wc(1) = lam / (n + lam) + (1.0 - alpha^2 + beta);

    % 第4部分：构建量测噪声协方差矩阵 R (3x3)
    % 对角线元素分别为距离噪声、方位噪声、多普勒速度的方差
    ukf.R = diag([params.ukf_range_std_m^2, ...
                  params.ukf_azimuth_std_deg^2, ...
                  params.ukf_rv_std_ms^2]);

    % 第5部分：构建过程噪声协方差矩阵 Q (4x4)
    % 基础 Q 是对角阵，四个状态分量的噪声强度不同
    % 位置和速度的先验不确定性差异很大（1e-9 vs 1e-13）
    % 最后乘以 ukf_Q_scale 进行整体缩放
    Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13]);
    ukf.Q = Q_base * params.ukf_Q_scale;

    % 第6部分：构建初始状态协方差矩阵 P (4x4)
    % 初始不确定性由参数 ukf_P_pos_std 和 ukf_P_vel_std 决定
    pp = params.ukf_P_pos_std;
    pv = params.ukf_P_vel_std;
    ukf.P = diag([pp^2, pv^2, pp^2, pv^2]);

    % 第7部分：初始化状态向量 x (4x1) 和初始化标志
    ukf.x = zeros(4, 1);
    ukf.initialized = false;

    % 第8部分：运动模型配置（默认CV，IMM场景可覆盖为CT）
    ukf.model_type = 'CV';
    ukf.turn_rate_rad_per_sec = 0.0;

    % 第9部分：地球半径常量（米），用于球面几何计算
    ukf.R_EARTH = 6371000.0;
end


% =========================================================================
% init_ukf — UKF 单点/两点初始化
% =========================================================================
% 使用 1-2 次量测初始化 UKF 状态：
%   - 位置：通过量测反解经纬度
%   - 速度：两点差分法（如果有两次不同帧的量测）
%   - 速度范围钳位：50-500 m/s，超出则回退到 v=0
function ukf = init_ukf(ukf, meas, meas2)
    % 步骤1：用最新可用量测反解初始位置
    % 若两点起始中有meas2，优先用meas2（当前帧，不过时）
    % 若仅有单点，用meas
    if nargin >= 3 && ~isempty(meas2)
        [lon, lat] = meas_to_latlon_ukf(ukf, meas2.range_meas, meas2.azimuth_meas);
    else
        [lon, lat] = meas_to_latlon_ukf(ukf, meas.range_meas, meas.azimuth_meas);
    end

    % 步骤2：尝试用两个实际量测按真实时间差计算初始速度
    % Oracle 起始允许量测跨越较宽窗口，但仍以物理速度范围拒绝异常差分
    lon_dot = 0.0;
    lat_dot = 0.0;

    % 检查是否有有效的第二点量测（不同帧、有时间戳）
    if nargin >= 3 && ~isempty(meas2) && isfield(meas2, 'frameID') ...
       && isfield(meas, 'frameID') && meas2.frameID ~= meas.frameID

        n_frames_apart = abs(meas2.frameID - meas.frameID);
        % 优先使用真实时间差（秒），如果没有则用帧数 * dt
        if isfield(meas, 'time_sec') && isfield(meas2, 'time_sec') && ...
                isfinite(meas.time_sec) && isfinite(meas2.time_sec) && meas2.time_sec ~= meas.time_sec
            dt_sec = abs(meas2.time_sec - meas.time_sec);
        else
            dt_sec = n_frames_apart * ukf.dt;
        end

        % 将两次量测反解为经纬度
        [lon1, lat1] = meas_to_latlon_ukf(ukf, meas.range_meas, meas.azimuth_meas);
        [lon2, lat2] = meas_to_latlon_ukf(ukf, meas2.range_meas, meas2.azimuth_meas);

        % 将球面距离近似为平面距离（中纬度处 1° ≈ 111320m）
        dlat_m = (lat2 - lat1) * 111320.0;
        dlon_m = (lon2 - lon1) * 111320.0 * cosd((lat1 + lat2) / 2);
        speed_ms = sqrt(dlat_m^2 + dlon_m^2) / max(dt_sec, 1.0);

        % 速度范围钳位：50-500 m/s 是合理的目标速度范围
        % 超出此范围说明差分计算可能出错（如量测跳变），回退到 v=0
        if speed_ms >= 50 && speed_ms <= 500
            lon_dot = (lon2 - lon1) / max(dt_sec, 1.0);
            lat_dot = (lat2 - lat1) / max(dt_sec, 1.0);
        end
        % 异常速度仍回退到 v=0，由 UKF 自行收敛
    end

    % 组装初始状态：[经度, 经度速度, 纬度, 纬度速度]
    ukf.x = [lon; lon_dot; lat; lat_dot];

    % 步骤3：构建初始协方差矩阵
    pp = ukf.params.ukf_P_pos_std;
    pv = ukf.params.ukf_P_vel_std;
    ukf.P = diag([pp^2, pv^2, pp^2, pv^2]);

    % 步骤4：标记滤波器已初始化
    ukf.initialized = true;
end


% =========================================================================
% prepare_ukf — 预测 + 量测统计（供上层关联模块使用）
% 一步完成：Sigma 点预测 → x_pred, P_pred → Z_pred → z_pred, P_zz
% =========================================================================
% prepare 是 UKF 预测步的核心，完成以下工作：
%   1. 生成 Sigma 点并通过状态转移函数传播
%   2. 计算预测状态均值和协方差
%   3. 通过量测函数将 Sigma 点投影到量测空间
%   4. 计算预测量测均值和协方差
%   5. 将所有中间结果缓存到 ukf.cache，供 update 使用
function [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = prepare_ukf(ukf)
    % ---- Step 1: UKF 预测步骤 ----
    % 调用 predict_step_ukf 完成 Sigma 点生成、传播、统计量计算
    [x_pred, P_pred, X_pred, ukf] = predict_step_ukf(ukf);

    % ---- Step 2: 量测预测及协方差计算 ----
    % 将预测状态通过量测函数 h(x) 得到预测量测
    z_pred = measurement_ukf(ukf, x_pred);
    % 将所有 Sigma 点通过量测函数，得到量测空间的 Sigma 点
    Z_pred = zeros(ukf.m, 2 * ukf.n + 1);
    for s = 1:(2 * ukf.n + 1)
        Z_pred(:, s) = measurement_ukf(ukf, X_pred(:, s));
    end

    % 计算量测协方差 P_zz = E[(z-z_pred)(z-z_pred)']
    % 初始值为量测噪声 R，加上 Sigma 点的加权散度
    P_zz = ukf.R;
    for s = 1:(2 * ukf.n + 1)
        dz = Z_pred(:, s) - z_pred;
        P_zz = P_zz + ukf.Wc(s) * (dz * dz');
    end
    % NaN 守卫：如果量测函数出现异常，回退到仅使用 R
    if any(isnan(P_zz(:)))
        P_zz = ukf.R;
    end

    % ---- Step 3: 缓存中间结果供 update 使用 ----
    % update 阶段不再重新计算这些量，直接从 cache 读取
    ukf.cache = struct('x_pred', x_pred, 'P_pred', P_pred, 'X_pred', X_pred, ...
                       'z_pred', z_pred, 'Z_pred', Z_pred, 'P_zz', P_zz);
end


% =========================================================================
% update_with_innov — 纯 Kalman 更新数学
% 输入：ukf + innov_w（PDA加权新息，[]=纯预测）
% 所有中间量（x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz）从 ukf.cache 读取
% 输出：更新后的 lon, lat, ukf
% =========================================================================
% 这是标准 Kalman 更新的实现：
%   1. 计算互协方差 P_xz
%   2. 计算卡尔曼增益 K = P_xz * P_zz^{-1}
%   3. 状态更新 x = x_pred + K * innov
%   4. 协方差更新 P = P_pred - K * P_zz * K'
%   5. 正则化确保数值稳定性
function [lon, lat, ukf] = update_with_innov(ukf, innov_w)
    % ---- 读取缓存 ----
    cache = ukf.cache;
    x_pred = cache.x_pred;
    P_pred = cache.P_pred;
    X_pred = cache.X_pred;
    z_pred = cache.z_pred;
    Z_pred = cache.Z_pred;
    P_zz  = cache.P_zz;

    % ---- 纯预测帧：保留预测状态，不更新 ----
    % 当 innov_w 为空时，说明本帧没有检测到目标或与点迹关联失败
    if isempty(innov_w)
        ukf.x = x_pred;
        ukf.P = P_pred;
        lon = ukf.x(1);
        lat = ukf.x(3);
        return;
    end

    % ---- Step 1: 计算互协方差 P_xz ----
    % P_xz = E[(x-x_pred)(z-z_pred)']，描述状态和量测之间的相关性
    % 通过 Sigma 点的加权散度计算
    P_xz = zeros(ukf.n, ukf.m);
    for i = 1:(2 * ukf.n + 1)
        dz = Z_pred(:, i) - z_pred;
        dx = X_pred(:, i) - x_pred;
        P_xz = P_xz + ukf.Wc(i) * (dx * dz');
    end

    % ---- Step 2: 计算卡尔曼增益 K ----
    % K = P_xz * P_zz^{-1}，决定新息对状态更新的贡献权重
    % 使用 / 运算符（MATLAB 的 mldivide），如果奇异则 fallback 到 pinv
    try
        K = P_xz / P_zz;
    catch
        K = P_xz * pinv(P_zz);
    end

    % ---- Step 3: 状态更新（后验均值） ----
    % x_new = x_pred + K * innov_w
    % 将预测状态沿新息方向修正，K 控制修正幅度
    ukf.x = x_pred + K * innov_w;

    % ---- Step 4: NaN 守卫 ----
    % 如果更新后状态出现 NaN 或 Inf，回退到预测状态
    if any(isnan(ukf.x)) || any(isinf(ukf.x))
        ukf.x = x_pred;
        ukf.P = P_pred;
        lon = ukf.x(1);
        lat = ukf.x(3);
        return;
    end

    % ---- Step 5: 协方差更新（后验协方差） ----
    % P_new = P_pred - K * P_zz * K'
    % 这是 Joseph 形式的简化版，协方差总是减小的（因为用到了量测信息）
    ukf.P = P_pred - K * P_zz * K';

    % ---- Step 6: 协方差正则化（数值稳定性维护） ----
    % 确保 P 保持对称正定（防止浮点误差导致发散）
    ukf.P = regularize_cov_ukf(ukf.P);

    % ---- Step 7: 提取滤波输出 ----
    lon = ukf.x(1);
    lat = ukf.x(3);
end


% =========================================================================
% predict_step_ukf — UKF 预测步（时间更新/先验估计）
% =========================================================================
% UKF 预测的核心：
%   1. 生成 2n+1 个 Sigma 点（覆盖当前状态的分布）
%   2. 将每个 Sigma 点通过非线性状态转移函数
%   3. 加权平均得到预测状态均值
%   4. 加权散度得到预测协方差
function [x_pred, P_pred, X_pred, ukf] = predict_step_ukf(ukf)
    % 子步骤1：生成 Sigma 点
    % Sigma 点是通过协方差的 Cholesky 分解构造的，确保覆盖状态的分布
    X = sigma_points_ukf(ukf.x, ukf.P, ukf.n, ukf.lam);

    % 子步骤2：传播每个 Sigma 点通过状态转移函数（根据运动模型分发）
    X_pred = zeros(ukf.n, 2 * ukf.n + 1);
    % 根据模型类型选择不同的状态转移函数
    % CT 模型且转弯率不为 0 时使用协调转弯模型
    if strcmp(ukf.model_type, 'CT') && abs(ukf.turn_rate_rad_per_sec) > 1e-12
        for i = 1:(2 * ukf.n + 1)
            X_pred(:, i) = state_transition_ct_ukf(X(:, i), ukf.dt, ukf.turn_rate_rad_per_sec);
        end
    else
        % 否则使用 CV（匀速直线）模型
        for i = 1:(2 * ukf.n + 1)
            X_pred(:, i) = state_transition_ukf(X(:, i), ukf.dt);
        end
    end

    % 子步骤3：计算预测状态均值 x_pred
    % x_pred = Σ Wm_i * X_pred_i，加权平均
    x_pred = X_pred * ukf.Wm;

    % 子步骤4：计算预测协方差矩阵 P_pred
    % P_pred = Q + Σ Wc_i * (X_pred_i - x_pred)(X_pred_i - x_pred)'
    % Q 是过程噪声协方差，第二项是 Sigma 点的加权散度
    P_pred = ukf.Q;
    for i = 1:(2 * ukf.n + 1)
        dx = X_pred(:, i) - x_pred;
        P_pred = P_pred + ukf.Wc(i) * (dx * dx');
    end

    % 子步骤5：协方差正则化（数值稳定性保障）
    P_pred = regularize_cov_ukf(P_pred);
end


% =========================================================================
% measurement_ukf — 天波双基地量测模型 h(x)
%
% 量测向量 z = [Rg; az; vd]（群距离、方位角、多普勒速度）
%
% 天波传播模型（与仿真端 generate_frame_detections 严格一致）：
%   1. 地心角 σ = Haversine(站点, 目标)
%   2. 地表弦长 D = 2·R_e·sin(σ/2)          ← 替代大圆弧长
%   3. 天波斜距 r = √(D² + (2H)²)            ← 电离层双跳
%   4. 群距离 Rg = r_tx + r_rx
%   5. 多普勒 vd = dRg/dt = dr_tx/dt + dr_rx/dt
% =========================================================================
function z = measurement_ukf(ukf, x)
    % 步骤1：提取状态分量
    lon = x(1);
    lon_rate = x(2);
    lat = x(3);
    lat_rate = x(4);

    % 步骤2：天波群距离 Rg = r_tx + r_rx（弦长+电离层虚高模型）
    % 调用 skywave_geometry 计算发射机-目标-接收机的群距离
    rng = skywave_geometry('group_range', ukf.tx_lon, ukf.tx_lat, ...
        ukf.radar_lon, ukf.radar_lat, lon, lat);

    % 步骤3：方位角（接收站→目标，球面方位角，与仿真端共用）
    az = skywave_geometry('azimuth', ukf.radar_lon, ukf.radar_lat, lon, lat);

    % 步骤4：天波多普勒 vd = dRg/dt = dr_tx/dt + dr_rx/dt
    % 多普勒速度是群距离对时间的导数，由 skywave_geometry 内部计算
    radial_vel = skywave_geometry('doppler', ukf.tx_lon, ukf.tx_lat, ...
        ukf.radar_lon, ukf.radar_lat, lon, lat, lon_rate, lat_rate);

    % 步骤5：组装预测量测向量 [群距离; 方位角; 多普勒速度]
    z = [rng; az; radial_vel];
end


% =========================================================================
% sigma_points_ukf — UKF Sigma 点生成（UT 变换）
% =========================================================================
% UT（Unscented Transform）的核心：用确定性的方式采样 2n+1 个 Sigma 点，
% 使其均值和协方差与原分布匹配。然后将这些点通过非线性函数传播，
% 最后加权重组得到传播后的统计量。
%
% Sigma 点构造方法：
%   X_0 = x（均值点）
%   X_{i}   = x + sqrt((n+lam)*P)_i（第 i 列 Cholesky 分解的列向量方向）
%   X_{n+i} = x - sqrt((n+lam)*P)_i（负方向）
%
% 其中 sqrt((n+lam)*P) 是 (n+lam)*P 的 Cholesky 下三角因子
function X = sigma_points_ukf(x, P, n, lam)
    % 步骤1：Cholesky 分解——计算协方差的平方根矩阵
    % sqrtP 满足 sqrtP * sqrtP' = (n+lam) * P
    % 如果 P 不是正定矩阵（浮点误差可能导致），加一个小扰动 1e-8*I
    try
        sqrtP = chol((n + lam) * P, 'lower');
    catch
        sqrtP = chol((n + lam) * P + 1e-8 * eye(n), 'lower');
    end

    % 步骤2：初始化 Sigma 点矩阵（预分配内存）
    % 2n+1 个点，每个点 n 维
    X = zeros(n, 2 * n + 1);

    % 步骤3：设置中心 Sigma 点 X_0 = x
    X(:, 1) = x;

    % 步骤4：生成正负方向的 Sigma 点
    % 沿每个状态维度的 Cholesky 列向量方向，取正负两个点
    for i = 1:n
        X(:, i + 1) = x + sqrtP(:, i);     % 正方向
        X(:, i + 1 + n) = x - sqrtP(:, i); % 负方向
    end
end


% =========================================================================
% state_transition_ukf — CV 匀速运动模型状态转移
% =========================================================================
% CV 模型的状态转移矩阵 F：
%   [1  dt  0  0]
%   [0  1  0  0]
%   [0  0  1  dt]
%   [0  0  0  1]
%
% 假设经度和纬度方向相互独立，各自用恒速模型
% x_next = F * x，即：
%   lon_next = lon + dt * lon_dot
%   lon_dot_next = lon_dot
%   lat_next = lat + dt * lat_dot
%   lat_dot_next = lat_dot
function x_next = state_transition_ukf(x, dt)
    F = [1.0, dt,  0.0, 0.0; ...
         0.0, 1.0, 0.0, 0.0; ...
         0.0, 0.0, 1.0, dt;  ...
         0.0, 0.0, 0.0, 1.0];
    x_next = F * x;
end

% =========================================================================
% state_transition_ct_ukf — CT 协调转弯模型状态转移（已知转弯率 ω）
%
% 状态 x = [lon, lon_dot, lat, lat_dot]'
% 对右转 ω>0，对左转 ω<0
%
% F_CT(Δt, ω) = [
%   1,  sin(ωΔt)/ω,          0,  -(1-cos(ωΔt))/ω;
%   0,  cos(ωΔt),            0,  -sin(ωΔt);
%   0,  (1-cos(ωΔt))/ω,      1,  sin(ωΔt)/ω;
%   0,  sin(ωΔt),            0,  cos(ωΔt)
% ]
%
% 当 ω→0 时，F_CT 退化为 F_CV（用泰勒展开验证）
% =========================================================================
function x_next = state_transition_ct_ukf(x, dt, omega)
    % 计算 ωΔt（无量纲转角）
    wT = omega * dt;
    cos_wT = cos(wT);
    sin_wT = sin(wT);
    one_m_cos = 1.0 - cos_wT;
    inv_omega = 1.0 / omega;

    % 构建 CT 状态转移矩阵
    F = [1.0, sin_wT * inv_omega,  0.0, -one_m_cos * inv_omega; ...
         0.0, cos_wT,              0.0, -sin_wT; ...
         0.0, one_m_cos * inv_omega, 1.0, sin_wT * inv_omega; ...
         0.0, sin_wT,              0.0, cos_wT];
    x_next = F * x;
end


% =========================================================================
% meas_to_latlon_ukf — 天波双基地极坐标→经纬度反解
% 使用迭代法：先用经典双基地反解得初值，再用天波模型精化
% =========================================================================
% 将天波双基地量测（群距离 Rg、方位角 az）反解为目标经纬度
% 由于天波传播路径复杂（电离层反射），不能直接用简单的三角测量，
% 需要迭代求解：假设一个距离 r1，计算目标位置，再用天波模型验证 Rg
function [lon, lat] = meas_to_latlon_ukf(ukf, rng, az)
    % 第1步：经典双基地反解初值
    % 计算发射机和接收机之间的基线长度
    dlon_b = deg2rad(ukf.tx_lon - ukf.radar_lon);
    dlat_b = deg2rad(ukf.tx_lat - ukf.radar_lat);
    a_b = sin(dlat_b/2)^2 ...
        + cos(deg2rad(ukf.radar_lat)) * cos(deg2rad(ukf.tx_lat)) * sin(dlon_b/2)^2;
    a_b = max(0, min(1, a_b));  % 钳位到 [0,1] 防止 asin 域外
    baseline = ukf.R_EARTH * 2 * atan2(sqrt(a_b), sqrt(1 - a_b));

    % 计算发射机方位角（tx 相对于 radar 的球面方位）
    y_b = sin(dlon_b) * cos(deg2rad(ukf.tx_lat));
    x_b = cos(deg2rad(ukf.radar_lat)) * sin(deg2rad(ukf.tx_lat)) ...
        - sin(deg2rad(ukf.radar_lat)) * cos(deg2rad(ukf.tx_lat)) * cos(dlon_b);
    tx_az = mod(rad2deg(atan2(y_b, x_b)), 360.0);

    % 计算目标相对于基线的方位角
    phi = az - tx_az;
    % 经典双基地距离公式（椭圆方程求解）
    r1 = 0.5 * (rng^2 - baseline^2) / (rng - baseline * cosd(phi));

    % 钳位到合理范围（1km - 5000km）
    r1 = max(1e3, min(r1, 5e6));

    % 第2步：迭代精化——用天波模型修正初值
    % 迭代最多 30 次，每次用天波模型预测 Rg，与实测 Rg 比较修正 r1
    for iter = 1:30
        % 根据当前 r1 和 az 计算目标经纬度
        [tgt_lon, tgt_lat] = sphere_utils_destination_point(ukf.radar_lon, ukf.radar_lat, r1, az);
        % 用天波模型计算该位置的预测群距离
        Rg_pred = skywave_geometry('group_range', ukf.tx_lon, ukf.tx_lat, ...
            ukf.radar_lon, ukf.radar_lat, tgt_lon, tgt_lat);
        err = rng - Rg_pred;  % 残差
        if abs(err) < 1.0
            break;  % 收敛（残差 < 1m）
        end
        % 按比例修正 r1
        r1 = r1 * rng / Rg_pred;
        r1 = max(1e3, min(r1, 5e6));
    end

    % 第3步：球面正算得到最终经纬度
    % 从雷达位置出发，沿方位角 az 方向走弧长 r1/R_EARTH
    arc_len = r1 / ukf.R_EARTH;
    az_rad = deg2rad(az);
    lat1 = deg2rad(ukf.radar_lat);
    lon1 = deg2rad(ukf.radar_lon);

    % 球面三角正算公式
    lat2 = asin(sin(lat1)*cos(arc_len) + cos(lat1)*sin(arc_len)*cos(az_rad));
    lon2 = lon1 + atan2(sin(az_rad)*sin(arc_len)*cos(lat1), ...
                          cos(arc_len) - sin(lat1)*sin(lat2));

    lon = rad2deg(lon2);
    lat = rad2deg(lat2);
end


% =========================================================================
% regularize_cov_ukf — 协方差矩阵正则化
% =========================================================================
% 确保协方差矩阵满足数值稳定性要求：
%   1. 对称化（消除浮点不对称性）
%   2. 特征值裁剪（确保正定性）
%   3. NaN/Inf 守卫
% =========================================================================
function P_reg = regularize_cov_ukf(P, min_eig)
    % 允许自定义最小特征值阈值
    if nargin < 2
        min_eig = 1e-12;
    end

    % NaN 守卫：如果出现 NaN/Inf，直接返回单位矩阵
    if any(isnan(P(:))) || any(isinf(P(:)))
        P_reg = eye(size(P, 1));
        return;
    end

    % 步骤1：强制对称化
    P_sym = (P + P') / 2.0;

    % 步骤2：特征值分解
    [V, D] = eig(P_sym);
    d = diag(D);

    % 步骤3：计算裁剪阈值（双阈值策略）
    % 取 min_eig 和 max_d 的 1e-6 倍中的较大者
    % 这样可以适应不同量级的协方差矩阵
    max_d = max(d);
    min_allowed = max(min_eig, 1e-6 * max_d);

    % 步骤4：裁剪特征值（将过小的特征值提升到 min_allowed）
    d_clip = max(d, min_allowed);

    % 步骤5：用裁剪后的特征值重构协方差矩阵
    P_reg = V * diag(d_clip) * V';
end


% =========================================================================
% haversine_ukf — Haversine 球面大圆距离
% 原函数：ukf_haversine
% =========================================================================
% 计算地球上两点之间的大圆距离（球面几何）
% 使用 Haversine 公式，数值稳定性优于直接球面余弦定理
function d = haversine_ukf(lon1, lat1, lon2, lat2, R)
    dlon = deg2rad(lon2 - lon1);
    dlat = deg2rad(lat2 - lat1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));  % 钳位到 [0,1] 防止浮点误差导致 acos 域外
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end
