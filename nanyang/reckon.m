% =========================================================================
% reckon.m — Mapping Toolbox reckon() 的兼容层
% 使用 sphere_utils_destination_point 替代 Mapping Toolbox 的 reckon。
% 接口与原版 reckon 兼容：
%   [latout, lonout] = reckon(lat, lon, arclen, az)
%   [latout, lonout] = reckon(lat, lon, arclen, az, R)
%
% 注意：南阳项目用 R_earth=6371 (km)，arclen 单位是 km。
% sphere_utils_destination_point 接受弧长单位为米，此处做 km→m 转换。
% =========================================================================
function [latout, lonout] = reckon(lat, lon, arclen, az, R)
    lat = double(lat);
    lon = double(lon);
    arclen = double(arclen);
    az = double(az);

    % sphere_utils_destination_point 接受弧长（米），返回 [lon, lat]（度）
    [lonout, latout] = sphere_utils_destination_point(lon, lat, arclen * 1000.0, az);
end
