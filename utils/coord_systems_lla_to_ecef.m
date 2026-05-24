% coord_systems_lla_to_ecef.m
% 大地坐标系(经纬度高度)转换为地心地固坐标系(ECEF)
% ================================================
% 输入：
%   lat_deg - 纬度（度）
%   lon_deg - 经度（度）
%   alt_m   - 高度（米）
% 输出：
%   ecef    - ECEF 坐标 [x; y; z]，单位：米
% 算法说明：
%   1. 将经纬度从度转换为弧度
%   2. 计算卯酉曲率半径 N = A / sqrt(1 - E2 * sin^2(lat))
%   3. 根据公式计算 ECEF 三维坐标

function ecef = coord_systems_lla_to_ecef(lat_deg, lon_deg, alt_m)
    % 将角度转换为弧度
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);
    % 计算正弦和余弦
    sin_lat = sin(lat);
    cos_lat = cos(lat);
    % 计算卯酉曲率半径 N
    N = coord_systems_get_A() / sqrt(1.0 - coord_systems_get_E2() * sin_lat^2);
    % 计算 ECEF 坐标分量
    x = (N + alt_m) * cos_lat * cos(lon);
    y = (N + alt_m) * cos_lat * sin(lon);
    z = (N * (1.0 - coord_systems_get_E2()) + alt_m) * sin_lat;
    % 输出列向量形式
    ecef = [x; y; z];
end
