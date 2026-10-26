function [md, nc, D2, valid] = pair_stats_fast(I1, I2, id1, id2)
% pair_stats_fast — pair_stats 的向量化等价实现（配合 build_track_index）
%
% 口径与原 pair_stats 一致：
%   md    = 共存帧（has_pos）平均 Haversine 距离
%   nc    = sum(valid)，valid 由 extract_state_local 成功（has_state）决定
%   D2    = 马氏距离平方，dx 与 P 均换算到 km/km²（局部平面近似 S=diag(sx,sy)）
%   valid = 该帧 D2 计算成功掩码
    nf = I1.n_frames;
    D2 = nan(1, nf);
    valid = false(1, nf);
    md = NaN;
    nc = 0;
    c1 = pairing.idx_col(I1, id1);
    c2 = pairing.idx_col(I2, id2);
    if c1 == 0 || c2 == 0, return; end

    % ---- 平均距离（has_pos 口径）----
    mp = I1.has_pos(:, c1) & I2.has_pos(:, c2);
    if any(mp)
        dp = pairing.hav_vec(I1.lon_raw(mp, c1), I1.lat_raw(mp, c1), ...
                             I2.lon_raw(mp, c2), I2.lat_raw(mp, c2));
        dp = dp(~isnan(dp));
        if ~isempty(dp), md = mean(dp); end
    end

    % ---- 马氏距离平方（has_state 口径）----
    ms = I1.has_state(:, c1) & I2.has_state(:, c2);
    if any(ms)
        lo1 = I1.lon_ukf(ms, c1); la1 = I1.lat_ukf(ms, c1);
        lo2 = I2.lon_ukf(ms, c2); la2 = I2.lat_ukf(ms, c2);
        p11 = I1.P11(ms, c1) + I2.P11(ms, c2);
        p12 = I1.P12(ms, c1) + I2.P12(ms, c2);
        p22 = I1.P22(ms, c1) + I2.P22(ms, c2);

        lat_mid = 0.5 * (la1 + la2);
        sx = 111.32 * cosd(lat_mid);   % 度 -> km（经向）
        sy = 110.57;                   % 度 -> km（纬向）
        dx = sx .* (lo1 - lo2);
        dy = sy .* (la1 - la2);

        % P_km = S * (P1+P2) * S'，S = diag(sx, sy)
        q11 = sx.^2 .* p11;
        q12 = sx .* sy .* p12;
        q22 = sy.^2 .* p22;

        det = q11 .* q22 - q12.^2;
        bad = det < 1e-12;             % 与原 rcond 保护等价的奇异保护
        if any(bad)
            q11(bad) = q11(bad) + 1e-6;
            q22(bad) = q22(bad) + 1e-6;
            det = q11 .* q22 - q12.^2;
        end
        d2v = (q22 .* dx.^2 - 2 * q12 .* dx .* dy + q11 .* dy.^2) ./ det;

        idx = find(ms);
        D2(idx) = d2v(:).';
        valid(idx) = true;
    end
    nc = sum(valid);
end
