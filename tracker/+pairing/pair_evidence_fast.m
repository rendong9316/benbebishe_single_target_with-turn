function [ds, D2, vdiff, vtol, u] = pair_evidence_fast(I1, I2, id1, id2)
% pair_evidence_fast — 一次性返回候选对在「全部共现帧」上的逐帧证据量
%
% 【与 pair_stats_fast 的区别】
%   pair_stats_fast 只返回标量摘要（均值距离 / 马氏 D² 数组），用于「全区间
%   统计型」判决；本函数额外返回逐帧距离与速度差序列，供「逐帧加权证据累积」
%   使用（EALM）。口径与原 pair_stats 完全一致，只是不做聚合。
%
% 【输出】
%   ds     [1×nf]  逐帧 Haversine 距离 (km)，仅在 u 帧有效，其余 NaN
%   D2     [1×nf]  逐帧马氏距离平方，仅在 u 帧有效
%   vdiff  [1×nf]  逐帧速度差模 (km/s)
%   vtol   [1×nf]  逐帧速度容差 sqrt(tr(Pv1)+tr(Pv2)) (km/s)
%   u      [1×nf]  统一有效掩码：该帧两站「原始位置」与「UKF 状态」均可用
%
%   统一掩码 u = has_pos(两站) & has_state(两站)，保证 ds / D2 / vdiff 同帧可比，
%   避免「距离来自原始量测、马氏来自滤波状态」造成的帧错位。

    nf = I1.n_frames;
    ds    = nan(1, nf);
    D2    = nan(1, nf);
    vdiff = nan(1, nf);
    vtol  = nan(1, nf);
    u     = false(1, nf);

    c1 = pairing.idx_col(I1, id1);
    c2 = pairing.idx_col(I2, id2);
    if c1 == 0 || c2 == 0, return; end

    mp = I1.has_pos(:, c1) & I2.has_pos(:, c2);
    ms = I1.has_state(:, c1) & I2.has_state(:, c2);
    u  = (mp & ms).';
    if ~any(u), return; end

    % ---------- 逐帧距离（原始位置口径）----------
    dp = pairing.hav_vec(I1.lon_raw(u, c1), I1.lat_raw(u, c1), ...
                         I2.lon_raw(u, c2), I2.lat_raw(u, c2));
    ds(u) = dp(:).';

    % ---------- 逐帧马氏 D² + 速度差（UKF 状态口径）----------
    lo1 = I1.lon_ukf(u, c1); la1 = I1.lat_ukf(u, c1);
    lo2 = I2.lon_ukf(u, c2); la2 = I2.lat_ukf(u, c2);
    p11 = I1.P11(u, c1) + I2.P11(u, c2);
    p12 = I1.P12(u, c1) + I2.P12(u, c2);
    p22 = I1.P22(u, c1) + I2.P22(u, c2);

    lat_mid = 0.5 * (la1 + la2);
    sx = 111.32 * cosd(lat_mid);   % 度 -> km（经向）
    sy = 110.57;                   % 度 -> km（纬向）
    dx = sx .* (lo1 - lo2);
    dy = sy .* (la1 - la2);

    q11 = sx.^2 .* p11;
    q12 = sx .* sy .* p12;
    q22 = sy.^2 .* p22;

    det = q11 .* q22 - q12.^2;
    bad = det < 1e-12;
    if any(bad)
        q11(bad) = q11(bad) + 1e-6;
        q22(bad) = q22(bad) + 1e-6;
        det = q11 .* q22 - q12.^2;
    end
    d2v = (q22 .* dx.^2 - 2 * q12 .* dx .* dy + q11 .* dy.^2) ./ det;
    D2(u) = d2v(:).';

    % 速度差与容差
    v1x = sx .* I1.vlon(u, c1);  v1y = sy .* I1.vlat(u, c1);
    v2x = sx .* I2.vlon(u, c2);  v2y = sy .* I2.vlat(u, c2);
    vdiff(u) = sqrt((v1x - v2x).^2 + (v1y - v2y).^2).';

    tr1 = sx.^2 .* I1.Pv11(u, c1) + sy.^2 .* I1.Pv22(u, c1);
    tr2 = sx.^2 .* I2.Pv11(u, c2) + sy.^2 .* I2.Pv22(u, c2);
    vtol(u) = sqrt(max(tr1, 0) + max(tr2, 0)).';
end
