% =========================================================================
% radar_coverage_check.m
% 判断目标是否在雷达探测范围内（地理威力 + 波束角度）
% =========================================================================
% 输入:
%   rx_lon, rx_lat — 接收站经纬度 (deg)
%   tgt_lon, tgt_lat — 目标经纬度 (deg)
%   params — 仿真参数 (含 beam_center_deg, beam_width_deg, range_min_m, range_max_m)
% 输出:
%   in_coverage — logical
%   r1 — 接收站到目标的地表距离 (m)
%   az — 接收站到目标的方位角 (deg)
% =========================================================================

function [in_coverage, r1, az] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, beam_center, params)
    r1 = sphere_utils_haversine_distance(rx_lon, rx_lat, tgt_lon, tgt_lat);
    az = sphere_utils_azimuth(rx_lon, rx_lat, tgt_lon, tgt_lat);

    half_beam = params.beam_width_deg / 2;
    az_diff = abs(az - beam_center);
    if az_diff > 180
        az_diff = 360 - az_diff;
    end

    in_coverage = (r1 >= params.range_min_m) && ...
                  (r1 <= params.range_max_m) && ...
                  (az_diff <= half_beam);
end
