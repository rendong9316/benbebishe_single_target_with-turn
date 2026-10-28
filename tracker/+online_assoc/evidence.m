function ev = evidence(local1, local2)
%EVIDENCE 计算两站局部航迹之间的逐帧证据量
%
%   ev = online_assoc.evidence(a, b)
%
%   输出字段：
%     dist_km — Haversine 距离（km）
%     d2      — 位置维马氏距离平方
%     vdiff   — 速度差模 (km/s)
%     vtol    — 速度容差 sqrt(tr(Pv1)+tr(Pv2)) (km/s)
%     ok_maha — d2 是否小于门限
%     ok_vel  — 速度差是否在容差内
%     ok_dist — 距离是否小于 T1

    ev = struct('dist_km', NaN, 'd2', NaN, 'vdiff', NaN, 'vtol', NaN, ...
                'ok_maha', false, 'ok_vel', false, 'ok_dist', false);

    x1 = local1.x;  x2 = local2.x;
    P1 = local1.P;  P2 = local2.P;
    if any(~isfinite(x1(1:4))) || any(~isfinite(x2(1:4))), return; end

    % ---- 距离（km）----
    R = 6371;
    p1 = deg2rad(x1(3));  p2 = deg2rad(x2(3));
    dp = deg2rad(x2(3) - x1(3));  dl = deg2rad(x2(1) - x1(1));
    q = sin(dp/2)^2 + cos(p1)*cos(p2)*sin(dl/2)^2;
    ev.dist_km = 2*R*asin(min(1, sqrt(q)));

    % ---- 位置维马氏距离平方（度→km 局部平面换算）----
    lat_mid = 0.5 * (x1(3) + x2(3));
    S = diag([111.32*cosd(lat_mid), 110.57]);
    dx = S * [x1(1) - x2(1); x1(3) - x2(3)];
    Pp = [P1(1,1) + P2(1,1), P1(1,3) + P2(1,3);
          P1(3,1) + P2(3,1), P1(3,3) + P2(3,3)];
    Pp = S * Pp * S';
    Pp = 0.5 * (Pp + Pp');
    if rcond(Pp) < 1e-12
        Pp = Pp + 1e-6*eye(2);
    end
    ev.d2 = dx' * (Pp \ dx);

    % ---- 速度差与容差（km/s）----
    sx = 111.32*cosd(lat_mid);  sy = 110.57;
    v1x = sx*x1(2);  v1y = sy*x1(4);
    v2x = sx*x2(2);  v2y = sy*x2(4);
    ev.vdiff = sqrt((v1x - v2x).^2 + (v1y - v2y).^2);
    tr1 = sx^2*P1(2,2) + sy^2*P1(4,4);
    tr2 = sx^2*P2(2,2) + sy^2*P2(4,4);
    ev.vtol = sqrt(max(tr1, 0) + max(tr2, 0));
end
