% ========================================================================
% radar_station_true_polar.m
% ========================================================================
%
% 【功能概述】
% 双基地真实极坐标量测计算函数。根据照射站（Tx）位置、接收站（Rx）位置
% 和目标位置（经纬度），计算目标在双基地雷达极坐标系中的三个真实量：
%   1. 群距离（bistatic range）：天波模型，地心角→弦长→双跳斜距→群距离
%   2. 方位角（azimuth）：从接收站看目标的真北偏东角度
%   3. 多普勒速度：天波总路径变化率 dRg/dt = dr_tx/dt + dr_rx/dt
%
% 【数学原理】
%
% 1. 天波双基地群距离：
%    σ   = Haversine(站点, 目标)  — 地心角
%    D   = 2·R_e·sin(σ/2)         — 地表弦长（替代大圆弧长）
%    r   = √(D² + (2H)²)          — 天波双跳斜距
%    Rg  = r_tx + r_rx            — 双基地群距离
%
% 2. 方位角（仅与接收站有关，与单基地相同）：
%    az = 球面方位角(接收站, 目标)
%
% 3. 天波多普勒（双基地多普勒的几何分量）：
%    vd = dRg/dt = dr_tx/dt + dr_rx/dt
%    目标速度 ENU 分解 → 各段路径变化率 → 合成总多普勒
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
    % ---- 第1步：天波双基地群距离（弦长+电离层虚高模型） ----
    rng = skywave_geometry('group_range', tx_lon, tx_lat, ...
        radar.lon, radar.lat, target_lon, target_lat);

    % ---- 第2步：方位角（接收站→目标） ----
    az = skywave_geometry('azimuth', radar.lon, radar.lat, target_lon, target_lat);

    % ---- 第3步：天波多普勒 dRg/dt = dr_tx/dt + dr_rx/dt ----
    rv = skywave_geometry('doppler', tx_lon, tx_lat, radar.lon, radar.lat, ...
        target_lon, target_lat, lon_rate, lat_rate);
end
% ========================================================================
% 文件结束
% ========================================================================
