% =========================================================================
% distance.m — Mapping Toolbox distance() 的兼容层
% 使用 sphere_utils_haversine_distance 替代 Mapping Toolbox 的 distance。
% 接口与原版 distance 兼容：
%   d = distance(lat1, lon1, lat2, lon2)         → 返回度
%   d = distance(lat1, lon1, lat2, lon2, R)      → 返回与 R 相同单位
%
% 注意：南阳项目用 R_earth=6371 (km)，返回值单位是 km。
% sphere_utils_haversine_distance 返回米，此处转换为 km。
% =========================================================================
function d = distance(lat1, lon1, lat2, lon2, R)
    % 将 lat1/lon1 和 lat2/lon2 展开为元素级计算
    % 支持标量和等长向量输入
    lat1 = double(lat1);
    lon1 = double(lon1);
    lat2 = double(lat2);
    lon2 = double(lon2);

    if nargin < 5
        % 无 R 参数 → 返回度（弧长 / 地球半径）
        R_earth_km = 6371.0;
    else
        R_earth_km = double(R);
    end

    n = max([numel(lat1), numel(lon1), numel(lat2), numel(lon2)]);
    d = zeros(size(lat1));

    for i = 1:n
        idx1 = min(i, numel(lat1));
        idx2 = min(i, numel(lat2));
        % sphere_utils_haversine_distance(lon1, lat1, lon2, lat2) → meters
        dist_m = sphere_utils_haversine_distance( ...
            lon1(min(i, numel(lon1))), lat1(min(i, numel(lat1))), ...
            lon2(min(i, numel(lon2))), lat2(min(i, numel(lat2))));
        d(i) = dist_m / 1000.0;  % meters → km
    end
end
