function [s, ok] = extract_state_local(trk)
% 从航迹中提取位置/速度及其协方差子块（度 / 度每秒 / km² 转换前的局部坐标）
% UKF 状态 x = [lon, v_lon, lat, v_lat]；位置协方差 P([1 3],[1 3])，速度 P([2 4],[2 4])
    s = struct('lon', NaN, 'lat', NaN, 'vel', [NaN; NaN], ...
               'Ppos', zeros(2, 2), 'Pvel', zeros(2, 2));
    ok = false;
    if ~isfield(trk, 'ukf') || isempty(trk.ukf), return; end
    if ~isfield(trk.ukf, 'x') || numel(trk.ukf.x) < 4, return; end
    if ~isfield(trk.ukf, 'P') || isempty(trk.ukf.P), return; end
    if size(trk.ukf.P, 1) < 4 || size(trk.ukf.P, 2) < 4, return; end
    x = trk.ukf.x(:);
    P = trk.ukf.P;
    if any(~isfinite(x(1:4))) || any(~isfinite(P(1:4, 1:4)), 'all'), return; end
    s.lon  = x(1);
    s.lat  = x(3);
    s.vel  = [x(2); x(4)];
    s.Ppos = P([1 3], [1 3]);
    s.Pvel = P([2 4], [2 4]);
    ok = true;
end
