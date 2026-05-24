% =========================================================================
% ukf_measurement_model.m
% =========================================================================
% 功能说明：
%   本函数实现UKF中的非线性双基地量测模型，即观测方程 z = h(x)。
%   它将状态向量（经纬度坐标+速度）映射到双基地雷达的量测空间：
%   群距离（bistatic range Rg = r0+r1）、方位角（azimuth）和
%   双基地径向速度（radial velocity）。
%
%   量测模型包含三个输出分量：
%     [1] 群距离 — Rg = r0 + r1
%         r0 = Haversine(Tx, 目标) + Haversine(目标, Rx)
%     [2] 方位角 — 从接收站观测目标的方向角（与单基地相同）
%     [3] 双基地径向速度 — 目标速度在 Tx→目标 + 目标→Rx 两方向投影之和
%
%   输入参数：
%     ukf  : UKF滤波器结构体，需要用到：
%            - ukf.tx_lon, ukf.tx_lat   : 照射站位置（度）
%            - ukf.radar_lon, ukf.radar_lat : 接收站位置（度）
%            - ukf.R_EARTH   : 地球半径（米）
%     x    : 状态向量，4×1，分量：[lon; lon_dot; lat; lat_dot]
%
%   输出参数：
%     z    : 预测量测向量，3×1，分量：[range; azimuth; radial_vel]
% =========================================================================

function z = ukf_measurement_model(ukf, x)
    % ======== 步骤1：提取状态分量并转换为弧度制 ========
    lon = x(1);
    lon_rate = x(2);
    lat = x(3);
    lat_rate = x(4);
    lat_rad = deg2rad(lat);

    % ======== 步骤2：双基地群距离 Rg = r0 + r1 ========
    r0 = ukf_haversine(ukf.tx_lon, ukf.tx_lat, lon, lat, ukf.R_EARTH);
    r1 = ukf_haversine(ukf.radar_lon, ukf.radar_lat, lon, lat, ukf.R_EARTH);
    rng = r0 + r1;

    % ======== 步骤3：方位角（接收站→目标） ========
    dlon = deg2rad(lon - ukf.radar_lon);
    dlat = deg2rad(lat - ukf.radar_lat);
    radar_lat_rad = deg2rad(ukf.radar_lat);
    y = sin(dlon) * cos(lat_rad);
    xval = cos(radar_lat_rad) * sin(lat_rad) ...
         - sin(radar_lat_rad) * cos(lat_rad) * cos(dlon);
    az = mod(rad2deg(atan2(y, xval)), 360.0);

    % ======== 步骤4：双基地径向速度 ========
    % 照射站→目标方向的方位角
    dlon_tx = deg2rad(lon - ukf.tx_lon);
    dlat_tx = deg2rad(lat - ukf.tx_lat);
    tx_lat_rad = deg2rad(ukf.tx_lat);
    y_tx = sin(dlon_tx) * cos(lat_rad);
    xval_tx = cos(tx_lat_rad) * sin(lat_rad) ...
            - sin(tx_lat_rad) * cos(lat_rad) * cos(dlon_tx);
    az_tx = mod(rad2deg(atan2(y_tx, xval_tx)), 360.0);

    % 目标地表速度分量
    v_east = lon_rate * pi / 180.0 * ukf.R_EARTH * cos(lat_rad);
    v_north = lat_rate * pi / 180.0 * ukf.R_EARTH;

    % 双基地径向速度 = Tx方向投影 + Rx方向投影
    rv_tx = v_east * sin(deg2rad(az_tx)) + v_north * cos(deg2rad(az_tx));
    rv_rx = v_east * sin(deg2rad(az)) + v_north * cos(deg2rad(az));
    radial_vel = rv_tx + rv_rx;

    % ======== 步骤5：组装预测量测向量 ========
    z = [rng; az; radial_vel];
end

% =========================================================================
% 内部函数：Haversine 球面大圆距离
% =========================================================================
function d = ukf_haversine(lon1, lat1, lon2, lat2, R)
    dlon = deg2rad(lon2 - lon1);
    dlat = deg2rad(lat2 - lat1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));  % 钳制a∈[0,1]，防止浮点误差导致sqrt(1-a)为复数
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end
