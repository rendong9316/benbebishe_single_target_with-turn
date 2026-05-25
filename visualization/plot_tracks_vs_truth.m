% =========================================================================
% plot_tracks_vs_truth.m
% =========================================================================
%
% 【功能概述】
%   绘制 UKF 滤波航迹与真实航迹的并排对比图。使用 tiledlayout
%   将图窗分为左右两列：左侧为 R1 的 UKF 滤波结果与真值对比，
%   右侧为 R2 的 UKF 滤波结果与真值对比。底图使用 darkwater。
%
% 【数学原理】
%   1. UKF (Unscented Kalman Filter) 滤波：
%      通过 Sigma 点采样近似非线性状态方程和量测方程的传播。
%      对于经纬度空间中的目标跟踪，状态向量通常为 [lon, lat, v_lon, v_lat]，
%      状态转移采用匀速运动模型(CV)或匀加速模型(CA)，量测为校准后的
%      经纬度坐标。
%   2. 滤波航迹提取：从 trackState 序列中提取所有有效帧的 UKF 估计
%      位置 (lat/lon)，跳过状态为空或 lat 为 NaN 的无效帧。
%   3. 定性评估：通过将滤波航迹与真实航迹画在同一张地图上，可以直观
%      判断滤波是否收敛、是否存在系统偏差、以及在目标机动段是否
%      出现跟踪丢失。
%
% 【输入参数】
%   trackState_R1  - R1 跟踪状态元胞数组
%   trackState_R2  - R2 跟踪状态元胞数组
%   true_track     - Nx2 矩阵，真值航迹 [lon, lat]
%   params         - 仿真参数字段结构体
%   out_dir        - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig3_tracks_vs_truth.png  - UKF滤波航迹 vs 真实航迹对比图
%
% 【调用关系】
%   被调用: 主仿真脚本
%   调用:   plot_track_on_map() (本文件内部)
%           geoplot()             (MATLAB 内置)
%
% =========================================================================

function plot_tracks_vs_truth(trackState_R1, trackState_R2, true_track, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);
    % tiledlayout 创建 1x2 网格布局
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- 左侧：R1 UKF 滤波航迹 ----
    ax1 = nexttile(tlo);
    try
        gx1 = geoaxes(ax1);
        gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes(ax1);
    end
    hold(gx1, 'on');
    title(gx1, 'R1 UKF滤波航迹');

    plot_track_on_map(gx1, trackState_R1, true_track, params.radar1_lat, params.radar1_lon);

    % ---- 右侧：R2 UKF 滤波航迹 ----
    ax2 = nexttile(tlo);
    try
        gx2 = geoaxes(ax2);
        gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes(ax2);
    end
    hold(gx2, 'on');
    title(gx2, 'R2 UKF滤波航迹');

    plot_track_on_map(gx2, trackState_R2, true_track, params.radar2_lat, params.radar2_lon);

    sgtitle('UKF滤波航迹 vs 真实航迹');
    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig3_tracks_vs_truth.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig3_tracks_vs_truth.png'));
    end
    fprintf('  图3 已保存: fig3_tracks_vs_truth.png\n');
end

% =========================================================================
% plot_track_on_map - 在指定 geoaxes 上绘制航迹对比
%
% 【说明】在同一张地图上绘制三层内容：
%   1. 真实航迹（亮黄虚线）— 作为参考基准
%   2. UKF 滤波航迹（青色实线）— 从 trackState 提取的有效帧位置
%   3. 接收站标记（红色方块）
%
% 【参数】
%   ax         - geoaxes 句柄
%   stateList  - 跟踪状态元胞数组
%   true_track - Nx2 真值航迹
%   rx_lat     - 接收站纬度
%   rx_lon     - 接收站经度
% =========================================================================
function plot_track_on_map(ax, stateList, true_track, rx_lat, rx_lon)
    % 真实航迹 (亮黄虚线，适配暗底图，对比度高)
    geoplot(ax, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.5, ...
        'DisplayName', '真实航迹');

    % 从跟踪状态中提取有效帧的 UKF 滤波位置
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~isfield(s, 'lat') || isnan(s.lat), continue; end
        lats(end+1) = s.lat;
        lons(end+1) = s.lon;
    end
    if ~isempty(lats)
        % 青色(cyan)实线在暗底图上对比度高
        geoplot(ax, lats, lons, 'c-', 'LineWidth', 1.5, 'DisplayName', 'UKF滤波');
    end

    % 接收站标记
    geoplot(ax, rx_lat, rx_lon, 'rs', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'DisplayName', '接收站');

    legend(ax, 'Location', 'best');
end
