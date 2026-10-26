function [md, nc] = mean_dist_fast(I1, I2, id1, id2)
% mean_dist_fast — mean_dist 的向量化等价实现（配合 build_track_index）
% 口径与原 mean_dist 完全一致：共存帧 = 两站该帧 has_pos 均为真；
% 距离用原始经纬度 Haversine 均值。
    md = NaN;
    nc = 0;
    c1 = pairing.idx_col(I1, id1);
    c2 = pairing.idx_col(I2, id2);
    if c1 == 0 || c2 == 0, return; end
    m = I1.has_pos(:, c1) & I2.has_pos(:, c2);
    if ~any(m), return; end
    d = pairing.hav_vec(I1.lon_raw(m, c1), I1.lat_raw(m, c1), ...
                        I2.lon_raw(m, c2), I2.lat_raw(m, c2));
    d = d(~isnan(d));
    nc = numel(d);
    if nc > 0
        md = mean(d);
    end
end
