% =========================================================================
% coord_systems_lla_to_ecef.m — LLA 坐标转 ECEF 坐标
% =========================================================================
% 【功能】
%   将大地坐标系（经纬度高度）转换为地心地固坐标系（ECEF: X/Y/Z）。
%   使用 WGS84 椭球模型，卯酉曲率半径 N 计算。
%
% 【输入】
%   lat_deg - 纬度（度）
%   lon_deg - 经度（度）
%   alt_m   - 高度（米，WGS84 椭球面以上）
%
% 【输出】
%   ecef    - ECEF 坐标 [x; y; z]，单位：米
%
% 【算法】
%   1. 计算卯酉曲率半径 N = A / sqrt(1 - E2 * sin^2(lat))
%      其中 A = 6378137.0 (WGS84 长半轴), f = 1/298.257223563
%   2. ECEF 坐标：
%      x = (N + alt) * cos(lat) * cos(lon)
%      y = (N + alt) * cos(lat) * sin(lon)
%      z = (N*(1-E2) + alt) * sin(lat)
% =========================================================================
function ecef = coord_systems_lla_to_ecef(lat_deg, lon_deg, alt_m)
    % 将角度从"度"转换为"弧度"，MATLAB 三角函数需要弧度输入
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);
    % 计算纬度的正弦和余弦值，避免重复计算
    sin_lat = sin(lat);
    cos_lat = cos(lat);
    % 计算卯酉曲率半径 N（子午圈曲率半径）
    % WGS84 椭球参数：长半轴 A=6378137.0 米，扁率 f=1/298.257223563
    % 第一偏心率平方 E2 = 2f - f^2
    f = 1.0 / 298.257223563;
    E2 = 2.0 * f - f^2;
    % N = A / sqrt(1 - E2 * sin^2(lat))，随纬度变化
    % 赤道处 N 最大（约 6378km），极点处 N 最小（约 6357km）
    N = 6378137.0 / sqrt(1.0 - E2 * sin_lat^2);
    % 计算 ECEF 坐标系的 X 分量
    % X = (N + alt) * cos(lat) * cos(lon)，将地理坐标投影到地心地固坐标系
    x = (N + alt_m) * cos_lat * cos(lon);
    % 计算 ECEF 坐标系的 Y 分量
    y = (N + alt_m) * cos_lat * sin(lon);
    % 计算 ECEF 坐标系的 Z 分量
    % Z = (N*(1-E2) + alt) * sin(lat)，注意 N 乘以 (1-E2) 是因为椭球在 Z 方向被压缩
    z = (N * (1.0 - E2) + alt_m) * sin_lat;
    % 输出列向量形式 [x; y; z]，符合 MATLAB 向量惯例
    ecef = [x; y; z];
end
