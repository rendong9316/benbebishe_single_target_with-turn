% =========================================================================
% ukf_meas_to_latlon.m
% =========================================================================
% 功能说明：
%   本函数实现从雷达极坐标量测（斜距、方位角）到目标WGS-84经纬度的
%   球面反算。给定雷达站位置和极坐标量测，通过球面几何公式计算出
%   目标的经纬度坐标。主要用于滤波器初始化阶段（第一帧量测）将雷达
%   量测转换为滤波器状态空间中的位置坐标。
%
%   UKF算法中的角色：
%     在 ukf_filter_init.m 中被调用，用于根据第一帧雷达量测计算出
%     目标初始经纬度估计，从而初始化滤波器状态向量。
%     这是从量测空间到状态空间的第一次映射，后续滤波过程中通过
%     ukf_measurement_model.m 进行正算（状态→量测），不调用本函数。
%
%   数学原理——球面正向/反向方位角问题（Vincenty直接法）：
%     已知：起点 P1(lon1, lat1)、球面距离 d、方位角 α
%     求：终点 P2(lon2, lat2)
%
%     设弧长角度 σ = d / R（球面角距）
%     则终点纬度：
%       lat2 = arcsin(sin(lat1)*cos(σ) + cos(lat1)*sin(σ)*cos(α))
%     终点经度差：
%       Δlon = atan2(sin(α)*sin(σ)*cos(lat1), cos(σ) - sin(lat1)*sin(lat2))
%     终点经度：
%       lon2 = lon1 + Δlon
%
%     这些公式来自球面三角学中的正弦定理和余弦定理的组合。
%
%   输入参数：
%     ukf  : UKF滤波器结构体，需包含：
%            - ukf.radar_lon : 雷达站经度（度）
%            - ukf.radar_lat : 雷达站纬度（度）
%            - ukf.R_EARTH   : 地球半径（米）
%     rng  : 斜距（米），即雷达量测的斜距
%     az   : 方位角（度），以正北为0度，顺时针旋转
%
%   输出参数：
%     lon  : 计算得到的目标经度（度）
%     lat  : 计算得到的目标纬度（度）
%
%   注意事项：
%     - 此函数假设地球为完美球体（R=6371000米），不进行椭球修正
%     - 对于中近距离应用（雷达探测范围通常<500km），球面近似的
%       误差远小于雷达量测误差，可以接受
%     - 输入方位角的定义域为 [0, 360)，atan2 和三角运算自动处理
%       各类象限情况
% =========================================================================

function [lon, lat] = ukf_meas_to_latlon(ukf, rng, az)
    % ======== 步骤1：双基地反解——求目标到接收站距离 r1 ========
    % 已知群距离 Rg = r0 + r1 = distance(Tx,target) + distance(target,Rx)
    % 基线 d = distance(Tx, Rx)
    % 由双基地椭圆方程解出 r1：
    %   r1 = (Rg² - d²) / (2·(Rg - d·cos(φ)))
    %   其中 φ = az - azimuth(Rx→Tx)

    % 计算基线距离和Tx在Rx坐标系中的方位
    dlon_b = deg2rad(ukf.tx_lon - ukf.radar_lon);
    dlat_b = deg2rad(ukf.tx_lat - ukf.radar_lat);
    a_b = sin(dlat_b/2)^2 ...
        + cos(deg2rad(ukf.radar_lat)) * cos(deg2rad(ukf.tx_lat)) * sin(dlon_b/2)^2;
    a_b = max(0, min(1, a_b));
    baseline = ukf.R_EARTH * 2 * atan2(sqrt(a_b), sqrt(1 - a_b));

    % Tx在Rx坐标系中的方位角
    y_b = sin(dlon_b) * cos(deg2rad(ukf.tx_lat));
    x_b = cos(deg2rad(ukf.radar_lat)) * sin(deg2rad(ukf.tx_lat)) ...
        - sin(deg2rad(ukf.radar_lat)) * cos(deg2rad(ukf.tx_lat)) * cos(dlon_b);
    tx_az = mod(rad2deg(atan2(y_b, x_b)), 360.0);

    % 双基地反解
    phi = az - tx_az;
    r1 = 0.5 * (rng^2 - baseline^2) / (rng - baseline * cosd(phi));

    % ======== 步骤2：球面正算（与单基地相同，但用 r1 而非 rng） ========
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
