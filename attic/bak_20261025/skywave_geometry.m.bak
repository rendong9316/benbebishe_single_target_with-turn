% =========================================================================
% skywave_geometry.m
% =========================================================================
% 功能概要：
%   天波双基地OTH雷达几何模型工具函数（action dispatcher 模式）。
%   提供天波传播模型所需的地心角、弦长、斜距、群距离和多普勒计算。
%   仿真端（generate_frame_detections）和 UKF 端（ukf_jichu）共用此模块，
%   确保两端几何模型严格一致，消除模型失配。
%
% 天波传播模型核心公式：
%   1. 地心角 σ: Haversine 公式（球面两点间球心角）
%   2. 地表弦长 D = 2·R_e·sin(σ/2)       ← 替代大圆弧长 R_e·σ
%   3. 天波斜距 r = √(D² + (2H)²)         ← 电离层双跳（地面→电离层→地面）
%   4. 群距离 Rg = r_tx + r_rx            ← 发射路径 + 接收路径
%   5. 多普勒 v_d = dr_tx/dt + dr_rx/dt   ← 总传播路径变化率
%
% 常量：
%   R_e = 6371×10³ m  — 地球平均半径
%   H   = 300×10³ m   — 电离层等效固定虚高（F层，半小时仿真不做时变）
%
% 公共 actions：
%   [Re, H] = skywave_geometry('constants')
%   sigma   = skywave_geometry('geocentric_angle', lon1, lat1, lon2, lat2)
%   D       = skywave_geometry('chord_length', lon1, lat1, lon2, lat2)
%   r       = skywave_geometry('slant_range', D)
%   Rg      = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat)
%   vd      = skywave_geometry('doppler', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat, lon_rate, lat_rate)
%   az      = skywave_geometry('azimuth', lon_from, lat_from, lon_to, lat_to)
%   [r_tx, r_rx, D_tx, D_rx, sigma_tx, sigma_rx, az_tx, az_rx] = skywave_geometry('full', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat)
% =========================================================================

function varargout = skywave_geometry(action, varargin)
    % 全局常量（地球半径、电离层虚高）
    R_e = 6371000.0;
    H   = 300000.0;

    switch action
        case 'constants'
            varargout{1} = R_e;
            varargout{2} = H;

        case 'geocentric_angle'
            [lon1, lat1, lon2, lat2] = deal(varargin{:});
            varargout{1} = geocentric_angle_impl(lon1, lat1, lon2, lat2);

        case 'chord_length'
            [lon1, lat1, lon2, lat2] = deal(varargin{:});
            sigma = geocentric_angle_impl(lon1, lat1, lon2, lat2);
            varargout{1} = 2.0 * R_e * sin(sigma / 2.0);

        case 'slant_range'
            D = varargin{1};
            varargout{1} = sqrt(D^2 + (2.0 * H)^2);

        case 'group_range'
            [tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat] = deal(varargin{:});
            sigma_tx = geocentric_angle_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
            sigma_rx = geocentric_angle_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
            D_tx = 2.0 * R_e * sin(sigma_tx / 2.0);
            D_rx = 2.0 * R_e * sin(sigma_rx / 2.0);
            r_tx = sqrt(D_tx^2 + (2.0 * H)^2);
            r_rx = sqrt(D_rx^2 + (2.0 * H)^2);
            varargout{1} = r_tx + r_rx;

        case 'doppler'
            [tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat, lon_rate, lat_rate] = deal(varargin{:});
            varargout{1} = doppler_impl(tx_lon, tx_lat, rx_lon, rx_lat, ...
                tgt_lon, tgt_lat, lon_rate, lat_rate, R_e, H);

        case 'azimuth'
            [lon_from, lat_from, lon_to, lat_to] = deal(varargin{:});
            varargout{1} = azimuth_impl(lon_from, lat_from, lon_to, lat_to);

        case 'full'
            [tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat] = deal(varargin{:});
            sigma_tx = geocentric_angle_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
            sigma_rx = geocentric_angle_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
            D_tx = 2.0 * R_e * sin(sigma_tx / 2.0);
            D_rx = 2.0 * R_e * sin(sigma_rx / 2.0);
            r_tx = sqrt(D_tx^2 + (2.0 * H)^2);
            r_rx = sqrt(D_rx^2 + (2.0 * H)^2);
            az_tx = azimuth_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
            az_rx = azimuth_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
            varargout{1} = r_tx;
            varargout{2} = r_rx;
            varargout{3} = D_tx;
            varargout{4} = D_rx;
            varargout{5} = sigma_tx;
            varargout{6} = sigma_rx;
            varargout{7} = az_tx;
            varargout{8} = az_rx;

        otherwise
            error('skywave_geometry: unknown action ''%s''', action);
    end
end


% =========================================================================
% geocentric_angle_impl — Haversine 公式计算两点间地心角 σ（弧度）
% =========================================================================
function sigma = geocentric_angle_impl(lon1, lat1, lon2, lat2)
    dlon = deg2rad(lon2 - lon1);
    dlat = deg2rad(lat2 - lat1);
    lat1_rad = deg2rad(lat1);
    lat2_rad = deg2rad(lat2);
    a = sin(dlat / 2.0)^2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2.0)^2;
    a = max(0.0, min(1.0, a));
    sigma = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
end


% =========================================================================
% azimuth_impl — 球面方位角（大圆初始方位角）
% =========================================================================
function az = azimuth_impl(lon_from, lat_from, lon_to, lat_to)
    dlon = deg2rad(lon_to - lon_from);
    lat_from_rad = deg2rad(lat_from);
    lat_to_rad = deg2rad(lat_to);
    y = sin(dlon) * cos(lat_to_rad);
    x = cos(lat_from_rad) * sin(lat_to_rad) ...
      - sin(lat_from_rad) * cos(lat_to_rad) * cos(dlon);
    az = mod(rad2deg(atan2(y, x)), 360.0);
end


% =========================================================================
% doppler_impl — 天波总路径变化率（多普勒速度）
%
% 物理定义：v_d = dR_g/dt = dr_tx/dt + dr_rx/dt
%
% 推导步骤：
%   1. 经纬度变化率 → 局部 ENU 速度
%        v_E = π·R_e·cos(φ_rad)·λ̇ / 180
%        v_N = π·R_e·φ̇ / 180
%   2. 单段斜距变化率（链式法则）：
%        dr/dt = (D/r) · dD/dt
%        dD/dt = R_e·cos(σ/2) · dσ/dt
%        dσ/dt = (v_E·sin(az) + v_N·cos(az)) / R_e
%      → dr/dt = (D/r) · cos(σ/2) · (v_E·sin(az) + v_N·cos(az))
%   3. 总多普勒 = dr_tx/dt + dr_rx/dt
% =========================================================================
function vd = doppler_impl(tx_lon, tx_lat, rx_lon, rx_lat, ...
        tgt_lon, tgt_lat, lon_rate, lat_rate, R_e, H)

    lat_rad = deg2rad(tgt_lat);

    % 第1步：经纬度变化率 → ENU 速度
    v_east  = lon_rate * pi / 180.0 * R_e * cos(lat_rad);
    v_north = lat_rate * pi / 180.0 * R_e;

    % 第2步：计算各段地心角、弦长、斜距、方位角
    sigma_tx = geocentric_angle_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
    sigma_rx = geocentric_angle_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
    D_tx = 2.0 * R_e * sin(sigma_tx / 2.0);
    D_rx = 2.0 * R_e * sin(sigma_rx / 2.0);
    r_tx = sqrt(D_tx^2 + (2.0 * H)^2);
    r_rx = sqrt(D_rx^2 + (2.0 * H)^2);
    az_tx = azimuth_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
    az_rx = azimuth_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);

    % 第3步：单段路径变化率
    dr_tx_dt = path_rate_impl(D_tx, r_tx, sigma_tx, az_tx, v_east, v_north);
    dr_rx_dt = path_rate_impl(D_rx, r_rx, sigma_rx, az_rx, v_east, v_north);

    % 第4步：合成总多普勒
    vd = dr_tx_dt + dr_rx_dt;
end


% =========================================================================
% path_rate_impl — 单段天波路径变化率 dr/dt
%   dr/dt = (D/r) · cos(σ/2) · (v_E·sin(az) + v_N·cos(az))
% =========================================================================
function dr_dt = path_rate_impl(D, r, sigma, az, v_east, v_north)
    az_rad = deg2rad(az);
    v_along_gc = v_east * sin(az_rad) + v_north * cos(az_rad);
    dr_dt = (D / r) * cos(sigma / 2.0) * v_along_gc;
end
