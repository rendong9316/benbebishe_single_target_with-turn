% ========================================================================
% radar_station_true_polar.m
% ========================================================================
%
% 【功能概述】
% 双基地真实极坐标量测计算函数。根据照射站（Tx）位置、接收站（Rx）位置
% 和目标位置（经纬度），计算目标在双基地雷达极坐标系中的三个真实量：
%   1. 群距离（bistatic range）：Tx→目标 + 目标→Rx 的地表大圆距离之和
%   2. 方位角（azimuth）：从接收站看目标的真北偏东角度
%   3. 双基地径向速度：目标速度在 Tx→目标 和 目标→Rx 两方向投影之和
%
% 【数学原理】
%
% 1. 双基地群距离：
%    Rg = r0 + r1
%    r0 = Haversine(照射站, 目标)  —— 照射站→目标的球面大圆距离
%    r1 = Haversine(目标, 接收站)    —— 目标→接收站的球面大圆距离
%
% 2. 方位角（仅与接收站有关，与单基地相同）：
%    az = sphere_utils_azimuth(接收站, 目标)
%
% 3. 双基地径向速度（双基地多普勒的几何分量）：
%    rv = radial_vel(目标速度, az_tx) + radial_vel(目标速度, az_rx)
%    即目标速度分别投影到 Tx→目标 和 Rx→目标 方向后求和
%
% 【输入参数】
%   radar       - 接收站结构体，包含 .lon 和 .lat 字段
%   tx_lon      - 照射站（发射站）经度（度）
%   tx_lat      - 照射站（发射站）纬度（度）
%   target_lon  - 目标当前经度（度）
%   target_lat  - 目标当前纬度（度）
%   lon_rate    - 目标经度变化率（度/秒）
%   lat_rate    - 目标纬度变化率（度/秒）
%
% 【返回值】
%   rng - 双基地群距离（米）= r0 + r1
%   az  - 方位角（度，0°=正北，顺时针），从接收站观测
%   rv  - 双基地径向速度（m/s）
% ========================================================================

function [rng, az, rv] = radar_station_true_polar(radar, tx_lon, tx_lat, ...
        target_lon, target_lat, lon_rate, lat_rate)
    % ---- 第1步：双基地群距离 ----
    r0 = sphere_utils_haversine_distance(tx_lon, tx_lat, target_lon, target_lat);
    r1 = sphere_utils_haversine_distance(radar.lon, radar.lat, target_lon, target_lat);
    rng = r0 + r1;

    % ---- 第2步：方位角（接收站→目标） ----
    az = sphere_utils_azimuth(radar.lon, radar.lat, target_lon, target_lat);

    % ---- 第3步：双基地径向速度 ----
    az_tx = sphere_utils_azimuth(tx_lon, tx_lat, target_lon, target_lat);
    rv_tx = sphere_utils_radial_velocity(lon_rate, lat_rate, target_lat, az_tx);
    rv_rx = sphere_utils_radial_velocity(lon_rate, lat_rate, target_lat, az);
    rv = rv_tx + rv_rx;
end
% ========================================================================
% 文件结束
% ========================================================================
