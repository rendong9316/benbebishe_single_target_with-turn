% =========================================================================
% bistatic_inverse_solver.m
% 天波双基地反解：从群距离 Rg 和方位角 az 求目标经纬度
% =========================================================================
% 输入:
%   Rg — 天波群距离 (m), Rg = r_tx + r_rx = sqrt(D_tx²+4H²)+sqrt(D_rx²+4H²)
%   az — 接收站观测目标的方位角 (deg)
%   tx_lon, tx_lat — 照射站经纬度 (deg)
%   rx_lon, rx_lat — 接收站经纬度 (deg)
% 输出:
%   r1 — 目标到接收站的地表大圆距离 (m)
%   lat, lon — 目标推算经纬度 (deg)
%
% 算法：
%   1. 用经典双基地反解公式（假设Rg=r0+r1，大圆距离和）得到初值
%   2. 迭代精化：用天波模型预测Rg，按比例修正r1，直到收敛
% =========================================================================

function [r1, lat, lon] = bistatic_inverse_solver(Rg, az, tx_lon, tx_lat, rx_lon, rx_lat)
    % 第1步：经典双基地反解初值（假设Rg=r0+r1为大圆距离和）
    baseline = sphere_utils_haversine_distance(tx_lon, tx_lat, rx_lon, rx_lat);
    tx_az = sphere_utils_azimuth(rx_lon, rx_lat, tx_lon, tx_lat);
    phi = az - tx_az;
    r1 = 0.5 * (Rg^2 - baseline^2) / (Rg - baseline * cosd(phi));

    % 钳位到合理范围（1km ~ 5000km）
    r1 = max(1e3, min(r1, 5e6));

    % 第2步：迭代精化——用天波模型修正
    for iter = 1:30
        [tgt_lon, tgt_lat] = sphere_utils_destination_point(rx_lon, rx_lat, r1, az);
        Rg_pred = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat);
        err = Rg - Rg_pred;
        if abs(err) < 1.0
            break;
        end
        r1 = r1 * Rg / Rg_pred;
        r1 = max(1e3, min(r1, 5e6));
    end

    % 第3步：用最终r1正算经纬度
    [lon, lat] = sphere_utils_destination_point(rx_lon, rx_lat, r1, az);
end
