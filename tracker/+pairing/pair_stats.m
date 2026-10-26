function [md, nc, D2, valid] = pair_stats(R1, R2, id1, id2, n_frames)
% 逐帧计算两航迹：平均位置距离 md、共存帧数 nc、马氏距离平方 D2、有效掩码 valid
% D2(k) = dx' inv(P1_pos+P2_pos) dx，dx 与 P 均换算到 km / km²（局部平面近似）
    D2 = nan(1, n_frames);
    ds = nan(1, n_frames);
    valid = false(1, n_frames);
    for k = 1:n_frames
        t1 = pairing.find_track_by_id(R1{k}, id1);
        t2 = pairing.find_track_by_id(R2{k}, id2);
        if isempty(t1) || isempty(t2), continue; end
        if t1.type == 7 || t2.type == 7, continue; end
        if isnan(t1.lat) || isnan(t2.lat), continue; end
        ds(k) = pairing.haversine(t1.lon, t1.lat, t2.lon, t2.lat);
        [s1, ok1] = pairing.extract_state_local(t1);
        [s2, ok2] = pairing.extract_state_local(t2);
        if ~ok1 || ~ok2, continue; end
        lat_mid = 0.5 * (s1.lat + s2.lat);
        scale = diag([111.32 * cosd(lat_mid), 110.57]);   % 度 -> km
        dx_km = scale * [s1.lon - s2.lon; s1.lat - s2.lat];
        P_pos = scale * (s1.Ppos + s2.Ppos) * scale';
        P_pos = 0.5 * (P_pos + P_pos');
        if rcond(P_pos) < 1e-12
            P_pos = P_pos + 1e-6 * eye(2);
        end
        D2(k) = dx_km' * (P_pos \ dx_km);
        valid(k) = true;
    end
    nv = sum(valid);
    nc = nv;
    if nv > 0
        md = mean(ds(valid));
    else
        md = NaN;
    end
end
