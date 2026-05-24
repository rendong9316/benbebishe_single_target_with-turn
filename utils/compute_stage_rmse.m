% compute_stage_rmse.m
% 计算某阶段的位置 RMSE（Haversine 距离，m）
% 对每个有效量测，在真实航迹中内插对应时刻的真实位置，计算球面距离误差
% 输入：
%   meas_cell - 量测/滤波 cell 数组
%   true_track - 真实航迹数组 (n,5) [lon, lat, lon_rate, lat_rate, time]
%   time_array - 时间数组
% 输出：rmse_val - RMSE 值

function rmse_val = compute_stage_rmse(meas_cell, true_track, time_array)
    % 误差列表
    errors = [];
    % 遍历所有帧
    for i = 1:length(meas_cell)
        % 当前帧量测
        m = meas_cell{i};
        % 跳过漏检
        if isempty(m), continue; end
        % 跳过无效经纬度
        if ~isfield(m, 'lat') || isnan(m.lat), continue; end
        % 当前帧时刻
        t = time_array(i);
        % 在真实航迹中插值对应时刻的位置
        true_lon_i = interp1(true_track(:, 5), true_track(:, 1), t, 'linear', 'extrap');
        true_lat_i = interp1(true_track(:, 5), true_track(:, 2), t, 'linear', 'extrap');
        % 计算 Haversine 球面距离误差
        err = sphere_utils_haversine_distance(m.lon, m.lat, true_lon_i, true_lat_i);
        % 收集误差
        errors(end+1) = err;
    end
    % 无有效点
    if isempty(errors)
        rmse_val = NaN;
        return;
    end
    % RMSE = sqrt(mean(err^2))
    rmse_val = sqrt(mean(errors .^ 2));
end