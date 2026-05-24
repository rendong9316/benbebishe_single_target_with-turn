% =========================================================================
% compute_stitched_rmse_at_detections.m
% 计算拼接后航迹的位置 RMSE —— 仅在原始量测有实际检测的时刻比较
% =========================================================================
%
% 拼接航迹由时间对齐+插值得来，在碎片段之间的空洞区域存在外推值。
% 为避免外推值污染 RMSE，本函数仅在有实际检测的时刻比较。
%
% 输入：
%   stitched    — 拼接后航迹 cell 数组（统一时间网格）
%   unified_time— 统一时间网格（秒）
%   meas_old    — 原始量测 cell 数组（旧格式，有检测=struct，漏检=[]）
%   t_orig      — 原始量测的时间网格（秒）
%   true_track  — 真实航迹 (n,5) [lon, lat, lon_rate, lat_rate, time]
%   params      — 仿真参数（用于 dt_sec）
% 输出：
%   rmse_val    — RMSE 值（米）
% =========================================================================

function rmse_val = compute_stitched_rmse_at_detections(stitched, unified_time, ...
        meas_old, t_orig, true_track, params)
    errors = [];
    half_dt = params.dt_sec / 2;

    for i = 1:length(meas_old)
        % 跳过漏检帧
        if isempty(meas_old{i}), continue; end

        t_meas = t_orig(i);

        % 在统一时间网格中找到最接近的时刻
        [~, idx] = min(abs(unified_time - t_meas));
        if abs(unified_time(idx) - t_meas) > half_dt
            continue;  % 时刻不匹配，跳过
        end

        m = stitched{idx};
        if isempty(m) || ~isfield(m, 'lat') || isnan(m.lat)
            continue;
        end

        % 在真实航迹中插值得到该时刻的真实位置
        true_lon = interp1(true_track(:,5), true_track(:,1), t_meas, 'linear', 'extrap');
        true_lat = interp1(true_track(:,5), true_track(:,2), t_meas, 'linear', 'extrap');

        err = sphere_utils_haversine_distance(m.lon, m.lat, true_lon, true_lat);
        errors(end+1) = err;
    end

    if isempty(errors)
        rmse_val = NaN;
    else
        rmse_val = sqrt(mean(errors .^ 2));
    end
end
