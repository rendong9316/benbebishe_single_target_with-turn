function [md, nc] = mean_dist(R1, R2, id1, id2, n_frames)
% 计算两条航迹在共存帧上的平均 Haversine 距离（km）与共存帧数
    ds = nan(1, n_frames);
    for k = 1:n_frames
        t1 = pairing.find_track_by_id(R1{k}, id1);
        t2 = pairing.find_track_by_id(R2{k}, id2);
        if isempty(t1) || isempty(t2), continue; end
        if t1.type == 7 || t2.type == 7, continue; end
        if isnan(t1.lat) || isnan(t2.lat), continue; end
        ds(k) = pairing.haversine(t1.lon, t1.lat, t2.lon, t2.lat);
    end
    valid = ~isnan(ds);
    nc = sum(valid);
    if nc > 0
        md = mean(ds(valid));
    else
        md = NaN;
    end
end
