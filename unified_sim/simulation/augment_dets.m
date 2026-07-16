% =========================================================================
% augment_dets.m — 点迹数组偏差校正字段扩展
% =========================================================================
% 给点迹数组添加偏差校正字段（drange/daz/raw_lat/raw_lon），统一处理
% 结构体数组字段不一致问题：原始 detRaw 来自 generate_frame_detections_multi，
% 没有 drange/daz/raw_lat/raw_lon 字段，直接 detRaw(d)=dp 会因字段集不同报错。
% 解决：先把整个数组扩展到统一字段集，再逐点赋值。
% =========================================================================

function detList = augment_dets(detList, dr_est, da_est, tx_lon, tx_lat, rx_lon, rx_lat)
    n = length(detList);
    if n == 0, return; end
    % 给整个数组预扩展字段（用 NaN 占位）
    for i = 1:n
        if ~isfield(detList(i), 'drange')
            detList(i).drange = NaN;
            detList(i).daz = NaN;
            detList(i).raw_lat = NaN;
            detList(i).raw_lon = NaN;
        end
    end
    % 逐点偏差校正 + 几何反解
    for d = 1:n
        dp = detList(d);
        Rgc = dp.prange - dr_est;  azc = dp.paz - da_est;
        dp.drange = Rgc;  dp.daz = azc;
        dp.range_meas = Rgc;  dp.azimuth_meas = azc;
        if ~isfield(dp, 'lat') || isnan(dp.lat)
            [~, dp.lat, dp.lon] = bistatic_inverse_solver(Rgc, azc, tx_lon, tx_lat, rx_lon, rx_lat);
        end
        [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, tx_lon, tx_lat, rx_lon, rx_lat);
        detList(d) = dp;
    end
end
