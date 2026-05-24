% =========================================================================
% bistatic_inverse_solver.m
% 双基地反解：从群距离 Rg 和方位角 az 求目标到接收站的地表距离 r1
% =========================================================================
% 输入:
%   Rg — 群距离 (m), Rg = distance(Tx,target) + distance(target,Rx)
%   az — 接收站观测目标的方位角 (deg)
%   tx_lon, tx_lat — 照射站经纬度 (deg)
%   rx_lon, rx_lat — 接收站经纬度 (deg)
% 输出:
%   r1 — 目标到接收站的地表距离 (m)
%   lat, lon — 目标推算经纬度 (deg)
% =========================================================================

function [r1, lat, lon] = bistatic_inverse_solver(Rg, az, tx_lon, tx_lat, rx_lon, rx_lat)
    baseline = sphere_utils_haversine_distance(tx_lon, tx_lat, rx_lon, rx_lat);
    tx_az = sphere_utils_azimuth(rx_lon, rx_lat, tx_lon, tx_lat);
    phi = az - tx_az;
    r1 = 0.5 * (Rg^2 - baseline^2) / (Rg - baseline * cosd(phi));

    [lon, lat] = sphere_utils_destination_point(rx_lon, rx_lat, r1, az);
end
