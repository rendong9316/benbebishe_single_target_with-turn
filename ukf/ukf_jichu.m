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
%   [lon, lat, ukf] = ukf_jichu('update', ukf, innov, z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz)
%      纯 Kalman 数学：P_xz → K → x_new → P_new。不含任何检测/关联/门限。
%   [x_pred, P_pred, X_pred, ukf] = ukf_jichu('predict', ukf)
%   z = ukf_jichu('measurement', ukf, x)
% =========================================================================

function varargout = ukf_jichu(action, varargin)
    switch action
        case 'create'
            varargout{1} = create_ukf(varargin{:});
        case 'init'
            varargout{1} = init_ukf(varargin{:});
        case 'prepare'
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}, ...
             varargout{5}, varargout{6}, varargout{7}] = prepare_ukf(varargin{:});
        case 'update'
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
function ukf = create_ukf(params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
    % 第1部分：保存基础参数到结构体
    ukf.params = params;
    ukf.radar_lon = radar_lon;
    ukf.radar_lat = radar_lat;
    ukf.tx_lon = tx_lon;
    ukf.tx_lat = tx_lat;
    ukf.dt = dt;

    % 第2部分：UKF 核心参数计算
    n = 4;
    alpha = params.ukf_alpha;
    beta  = params.ukf_beta;
    kappa = params.ukf_kappa;
    lam = alpha^2 * (n + kappa) - n;

    ukf.n   = n;
    ukf.m   = 3;
    ukf.lam = lam;

    % 第3部分：计算 Sigma 点权重
    ukf.Wm = ones(2 * n + 1, 1) / (2.0 * (n + lam));
    ukf.Wc = ones(2 * n + 1, 1) / (2.0 * (n + lam));
    ukf.Wm(1) = lam / (n + lam);
    ukf.Wc(1) = lam / (n + lam) + (1.0 - alpha^2 + beta);

    % 第4部分：构建量测噪声协方差矩阵 R (3x3)
    ukf.R = diag([params.ukf_range_std_m^2, ...
                  params.ukf_azimuth_std_deg^2, ...
                  params.ukf_rv_std_ms^2]);

    % 第5部分：构建过程噪声协方差矩阵 Q (4x4)
    Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13]);
    ukf.Q = Q_base * params.ukf_Q_scale;

    % 第6部分：构建初始状态协方差矩阵 P (4x4)
    pp = params.ukf_P_pos_std;
    pv = params.ukf_P_vel_std;
    ukf.P = diag([pp^2, pv^2, pp^2, pv^2]);

    % 第7部分：初始化状态向量 x (4x1) 和初始化标志
    ukf.x = zeros(4, 1);
    ukf.initialized = false;

    % 第8部分：常量
    ukf.R_EARTH = 6371000.0;
end


% =========================================================================
% init_ukf — UKF 单点/两点初始化
% =========================================================================
function ukf = init_ukf(ukf, meas, meas2)
    % 步骤1：用最新可用量测反解初始位置
    % 若两点起始中有meas2，优先用meas2（当前帧，不过时）
    % 若仅有单点，用meas
    if nargin >= 3 && ~isempty(meas2)
        [lon, lat] = meas_to_latlon_ukf(ukf, meas2.range_meas, meas2.azimuth_meas);
    else
        [lon, lat] = meas_to_latlon_ukf(ukf, meas.range_meas, meas.azimuth_meas);
    end

    % 步骤2：尝试两点差分计算初始速度，带多重合理性检查
    %   - 帧间隔 ≤ 2（时间近，杂波混入概率低）
    %   - 速度大小在 [50, 500] m/s 范围内（亚音速）
    % 任一条件不满足则回退到 v=0，由 UKF 自行收敛
    lon_dot = 0.0;
    lat_dot = 0.0;

    if nargin >= 3 && ~isempty(meas2) && isfield(meas2, 'frameID') ...
       && isfield(meas, 'frameID') && meas2.frameID ~= meas.frameID

        n_frames_apart = abs(meas2.frameID - meas.frameID);

        % 仅当两点时间接近（≤2帧）时才信任差分速度
        % 帧间隔大将大幅增加其中一点为杂波的概率
        if n_frames_apart <= 2
            [lon1, lat1] = meas_to_latlon_ukf(ukf, meas.range_meas, meas.azimuth_meas);
            [lon2, lat2] = meas_to_latlon_ukf(ukf, meas2.range_meas, meas2.azimuth_meas);

            dt_sec = n_frames_apart * ukf.dt;
            dlat_m = (lat2 - lat1) * 111320.0;
            dlon_m = (lon2 - lon1) * 111320.0 * cosd((lat1 + lat2) / 2);
            speed_ms = sqrt(dlat_m^2 + dlon_m^2) / max(dt_sec, 1.0);

            if speed_ms >= 50 && speed_ms <= 500
                lon_dot = (lon2 - lon1) / max(dt_sec, 1.0);
                lat_dot = (lat2 - lat1) / max(dt_sec, 1.0);
            end
        end
        % 否则保持 v=0，由 UKF 自行收敛
    end

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
function [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = prepare_ukf(ukf)
    % ---- Step 1: UKF 预测步骤 ----
    [x_pred, P_pred, X_pred, ukf] = predict_step_ukf(ukf);

    % ---- Step 2: 量测预测及协方差计算 ----
    z_pred = measurement_ukf(ukf, x_pred);
    Z_pred = zeros(ukf.m, 2 * ukf.n + 1);
    for s = 1:(2 * ukf.n + 1)
        Z_pred(:, s) = measurement_ukf(ukf, X_pred(:, s));
    end

    P_zz = ukf.R;
    for s = 1:(2 * ukf.n + 1)
        dz = Z_pred(:, s) - z_pred;
        P_zz = P_zz + ukf.Wc(s) * (dz * dz');
    end
    if any(isnan(P_zz(:)))
        P_zz = ukf.R;
    end
end


% =========================================================================
% update_with_innov — 纯 Kalman 更新数学
% 输入：UKF 结构体 + 外部提供的加权新息及预测统计量
% 输出：更新后的 lon, lat, ukf（ukf.x 和 ukf.P 已更新）
% =========================================================================
function [lon, lat, ukf] = update_with_innov(ukf, innov, z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz)
    % ---- Step 1: 计算互协方差 P_xz ----
    P_xz = zeros(ukf.n, ukf.m);
    for i = 1:(2 * ukf.n + 1)
        dz = Z_pred(:, i) - z_pred;
        dx = X_pred(:, i) - x_pred;
        P_xz = P_xz + ukf.Wc(i) * (dx * dz');
    end

    % ---- Step 2: 计算卡尔曼增益 K ----
    try
        K = P_xz / P_zz;
    catch
        K = P_xz * pinv(P_zz);
    end

    % ---- Step 3: 状态更新（后验均值） ----
    ukf.x = x_pred + K * innov;

    % ---- Step 4: NaN 守卫 ----
    if any(isnan(ukf.x)) || any(isinf(ukf.x))
        ukf.x = x_pred;
        ukf.P = P_pred;
        lon = ukf.x(1);
        lat = ukf.x(3);
        return;
    end

    % ---- Step 5: 协方差更新（后验协方差） ----
    ukf.P = P_pred - K * P_zz * K';

    % ---- Step 6: 协方差正则化（数值稳定性维护） ----
    ukf.P = regularize_cov_ukf(ukf.P);

    % ---- Step 7: 提取滤波输出 ----
    lon = ukf.x(1);
    lat = ukf.x(3);
end


% =========================================================================
% predict_step_ukf — UKF 预测步（时间更新/先验估计）
% =========================================================================
function [x_pred, P_pred, X_pred, ukf] = predict_step_ukf(ukf)
    % 子步骤1：生成 Sigma 点
    X = sigma_points_ukf(ukf.x, ukf.P, ukf.n, ukf.lam);

    % 子步骤2：传播每个 Sigma 点通过状态转移函数
    X_pred = zeros(ukf.n, 2 * ukf.n + 1);
    for i = 1:(2 * ukf.n + 1)
        X_pred(:, i) = state_transition_ukf(X(:, i), ukf.dt);
    end

    % 子步骤3：计算预测状态均值 x_pred
    x_pred = X_pred * ukf.Wm;

    % 子步骤4：计算预测协方差矩阵 P_pred
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
    rng = skywave_geometry('group_range', ukf.tx_lon, ukf.tx_lat, ...
        ukf.radar_lon, ukf.radar_lat, lon, lat);

    % 步骤3：方位角（接收站→目标，球面方位角，与仿真端共用）
    az = skywave_geometry('azimuth', ukf.radar_lon, ukf.radar_lat, lon, lat);

    % 步骤4：天波多普勒 vd = dRg/dt = dr_tx/dt + dr_rx/dt
    radial_vel = skywave_geometry('doppler', ukf.tx_lon, ukf.tx_lat, ...
        ukf.radar_lon, ukf.radar_lat, lon, lat, lon_rate, lat_rate);

    % 步骤5：组装预测量测向量
    z = [rng; az; radial_vel];
end


% =========================================================================
% sigma_points_ukf — UKF Sigma 点生成（UT 变换）
% =========================================================================
function X = sigma_points_ukf(x, P, n, lam)
    % 步骤1：Cholesky 分解——计算协方差的平方根矩阵
    try
        sqrtP = chol((n + lam) * P, 'lower');
    catch
        sqrtP = chol((n + lam) * P + 1e-8 * eye(n), 'lower');
    end

    % 步骤2：初始化 Sigma 点矩阵（预分配内存）
    X = zeros(n, 2 * n + 1);

    % 步骤3：设置中心 Sigma 点 X_0
    X(:, 1) = x;

    % 步骤4：生成正负方向的 Sigma 点
    for i = 1:n
        X(:, i + 1) = x + sqrtP(:, i);
        X(:, i + 1 + n) = x - sqrtP(:, i);
    end
end


% =========================================================================
% state_transition_ukf — CV 匀速运动模型状态转移
% =========================================================================
function x_next = state_transition_ukf(x, dt)
    F = [1.0, dt,  0.0, 0.0; ...
         0.0, 1.0, 0.0, 0.0; ...
         0.0, 0.0, 1.0, dt;  ...
         0.0, 0.0, 0.0, 1.0];
    x_next = F * x;
end


% =========================================================================
% meas_to_latlon_ukf — 天波双基地极坐标→经纬度反解
% 使用迭代法：先用经典双基地反解得初值，再用天波模型精化
% =========================================================================
function [lon, lat] = meas_to_latlon_ukf(ukf, rng, az)
    % 第1步：经典双基地反解初值
    dlon_b = deg2rad(ukf.tx_lon - ukf.radar_lon);
    dlat_b = deg2rad(ukf.tx_lat - ukf.radar_lat);
    a_b = sin(dlat_b/2)^2 ...
        + cos(deg2rad(ukf.radar_lat)) * cos(deg2rad(ukf.tx_lat)) * sin(dlon_b/2)^2;
    a_b = max(0, min(1, a_b));
    baseline = ukf.R_EARTH * 2 * atan2(sqrt(a_b), sqrt(1 - a_b));

    y_b = sin(dlon_b) * cos(deg2rad(ukf.tx_lat));
    x_b = cos(deg2rad(ukf.radar_lat)) * sin(deg2rad(ukf.tx_lat)) ...
        - sin(deg2rad(ukf.radar_lat)) * cos(deg2rad(ukf.tx_lat)) * cos(dlon_b);
    tx_az = mod(rad2deg(atan2(y_b, x_b)), 360.0);

    phi = az - tx_az;
    r1 = 0.5 * (rng^2 - baseline^2) / (rng - baseline * cosd(phi));

    % 钳位到合理范围
    r1 = max(1e3, min(r1, 5e6));

    % 第2步：迭代精化——用天波模型修正初值
    for iter = 1:30
        [tgt_lon, tgt_lat] = sphere_utils_destination_point(ukf.radar_lon, ukf.radar_lat, r1, az);
        Rg_pred = skywave_geometry('group_range', ukf.tx_lon, ukf.tx_lat, ...
            ukf.radar_lon, ukf.radar_lat, tgt_lon, tgt_lat);
        err = rng - Rg_pred;
        if abs(err) < 1.0
            break;
        end
        r1 = r1 * rng / Rg_pred;
        r1 = max(1e3, min(r1, 5e6));
    end

    % 第3步：球面正算得到最终经纬度
    arc_len = r1 / ukf.R_EARTH;
    az_rad = deg2rad(az);
    lat1 = deg2rad(ukf.radar_lat);
    lon1 = deg2rad(ukf.radar_lon);

    lat2 = asin(sin(lat1)*cos(arc_len) + cos(lat1)*sin(arc_len)*cos(az_rad));
    lon2 = lon1 + atan2(sin(az_rad)*sin(arc_len)*cos(lat1), ...
                          cos(arc_len) - sin(lat1)*sin(lat2));

    lon = rad2deg(lon2);
    lat = rad2deg(lat2);
end


% =========================================================================
% regularize_cov_ukf — 协方差矩阵正则化
% =========================================================================
function P_reg = regularize_cov_ukf(P, min_eig)
    if nargin < 2
        min_eig = 1e-12;
    end

    % NaN 守卫
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
    max_d = max(d);
    min_allowed = max(min_eig, 1e-6 * max_d);

    % 步骤4：裁剪特征值
    d_clip = max(d, min_allowed);

    % 步骤5：用裁剪后的特征值重构协方差矩阵
    P_reg = V * diag(d_clip) * V';
end


% =========================================================================
% haversine_ukf — Haversine 球面大圆距离
% 原函数：ukf_haversine
% =========================================================================
function d = haversine_ukf(lon1, lat1, lon2, lat2, R)
    dlon = deg2rad(lon2 - lon1);
    dlat = deg2rad(lat2 - lat1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end
