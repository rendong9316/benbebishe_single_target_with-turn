% =========================================================================
% plot_turn_fusion_map.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标的融合航迹地图对比图。在主图上展示真实航迹与
%   两种融合结果（基础 UKF 融合虚线、自适应 UKF 融合实线），
%   右上方附带拐弯区域放大子图，右下方附带信息汇总面板。
%
% 【数学原理】
%   1. 融合航迹对比：在同一地理坐标系下对比基础 UKF 融合航迹
%      （固定 Q）和自适应 UKF 融合航迹（机动检测 + Q 提升），
%      直观展示自适应策略在拐弯区域的精度优势。
%   2. 拐弯区域放大：通过白框标记主图中的拐弯区域，在放大子图中
%      以加粗线宽展示放大后的细节，便于定性评估两种策略的差异。
%   3. Haversine 距离：用于定位距离指定拐点 (128.5E, 33.5N) 最近的
%      帧号，确定拐弯区域的中心位置。
%
% 【输入参数】
%   true_track       - Nx2 矩阵，真值航迹 [lon, lat]
%   fused_base       - 元胞数组，基础 UKF 各融合方法的快照
%   fuse_methods     - 基础 UKF 融合方法名称列表
%   best_m_base      - 基础 UKF 最优融合方法索引
%   fused_ad         - 元胞数组，自适应 UKF 各融合方法的快照
%   fuse_methods_ad  - 自适应 UKF 融合方法名称列表
%   best_m_ad        - 自适应 UKF 最优融合方法索引
%   params           - 仿真参数字段结构体
%   out_dir          - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig5_fusion_map.png  - 融合地图对比图
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   sphere_utils_haversine_distance() (球面距离)
%           extract_fused()                 (本文件内部)
%
% =========================================================================

function plot_turn_fusion_map(true_track, ...
        fused_base, fuse_methods, best_m_base, ...
        fused_ad, fuse_methods_ad, best_m_ad, params, out_dir)

    % 提取最优融合航迹的经纬度
    [lat_fb, lon_fb] = extract_fused(fused_base{best_m_base});
    [lat_fa, lon_fa] = extract_fused(fused_ad{best_m_ad});

    % 定位拐点：找到距离 (128.5E, 33.5N) 最近的帧号
    tf = round(size(true_track,1)/2); md = inf;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), 128.5, 33.5);
        if d < md, md = d; tf = kk; end
    end
    % 拐弯放大区域范围：拐点前后各 20 帧
    zs = max(1, tf-20); ze = min(size(true_track,1), tf+20);

    fig = figure('Position', [50, 50, 1400, 750]);

    % ---- 左: 融合全图（主图） ----
    try
        gx = geoaxes('Units', 'normalized', 'Position', [0.04, 0.08, 0.62, 0.90]);
        gx.Basemap = 'darkwater';
    catch
        gx = geoaxes('Units', 'normalized', 'Position', [0.04, 0.08, 0.62, 0.90]);
    end
    hold(gx, 'on');
    title(gx, sprintf('融合航迹: 基础%s(虚线) vs 自适应%s(实线)', ...
        fuse_methods{best_m_base}, fuse_methods_ad{best_m_ad}), 'FontSize', 12);

    % 三层航迹：真值(黄虚线)、基础融合(青虚线)、自适应融合(深绿实线)
    geoplot(gx, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    geoplot(gx, lat_fb, lon_fb, '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2.2, ...
        'DisplayName', sprintf('基础%s融合', fuse_methods{best_m_base}));
    geoplot(gx, lat_fa, lon_fa, '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 3, ...
        'DisplayName', sprintf('自适应%s融合', fuse_methods_ad{best_m_ad}));

    % 站点和拐点标记
    geoplot(gx, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    geoplot(gx, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    geoplot(gx, true_track(tf,2), true_track(tf,1), 'wo', 'MarkerSize', 8, 'LineWidth', 2);

    % 拐弯区域白框：指示右上角放大子图的范围
    rx = [min(true_track(zs:ze,1)), max(true_track(zs:ze,1))];
    ry = [min(true_track(zs:ze,2)), max(true_track(zs:ze,2))];
    geoplot(gx, [ry(1) ry(1) ry(2) ry(2) ry(1)], [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);

    legend(gx, 'Location', 'southwest', 'FontSize', 9);

    % ---- 右上: 拐弯区域放大 ----
    try
        gz = geoaxes('Units', 'normalized', 'Position', [0.68, 0.55, 0.30, 0.42]);
        gz.Basemap = 'darkwater';
    catch
        gz = geoaxes('Units', 'normalized', 'Position', [0.68, 0.55, 0.30, 0.42]);
    end
    hold(gz, 'on');
    title(gz, '拐弯区域放大', 'FontSize', 9);

    % 拐弯区域的真值
    geoplot(gz, true_track(zs:ze,2), true_track(zs:ze,1), 'y--', 'LineWidth', 2.5);
    % 基础融合（拐弯区域）
    if ~isempty(lat_fb)
        iz = zs:min(ze, length(lat_fb));
        geoplot(gz, lat_fb(iz), lon_fb(iz), '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2);
    end
    % 自适应融合（拐弯区域），线宽最粗
    if ~isempty(lat_fa)
        iz = zs:min(ze, length(lat_fa));
        geoplot(gz, lat_fa(iz), lon_fa(iz), '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 3);
    end
    legend(gz, {'真值', '基础融合', '自适应融合'}, 'Location', 'best', 'FontSize', 7);

    % ---- 右下: 信息汇总面板 ----
    ax_info = axes('Units', 'normalized', 'Position', [0.68, 0.06, 0.30, 0.44]);
    ax_info.Visible = 'off';  % 仅显示文字，隐藏坐标轴
    y = 0.92;
    text(0.05, y, '融合结果汇总', 'Units', 'normalized', 'FontSize', 13, 'FontWeight', 'bold'); y = y - 0.12;
    text(0.05, y, sprintf('基础最优: %s', fuse_methods{best_m_base}), 'Units', 'normalized', 'FontSize', 10); y = y - 0.08;
    text(0.05, y, sprintf('自适应最优: %s', fuse_methods_ad{best_m_ad}), 'Units', 'normalized', 'FontSize', 10); y = y - 0.08;
    text(0.05, y, sprintf('拐角: ~113°'), 'Units', 'normalized', 'FontSize', 10); y = y - 0.08;
    text(0.05, y, sprintf('Pd=%.0f%%  Pfa=%.3f', params.detection_probability*100, params.false_alarm_rate), ...
        'Units', 'normalized', 'FontSize', 10); y = y - 0.12;
    text(0.05, y, '图例:', 'Units', 'normalized', 'FontSize', 10, 'FontWeight', 'bold'); y = y - 0.08;
    text(0.10, y, '黄色虚线 = 真实航迹', 'Units', 'normalized', 'FontSize', 9, 'Color', [0.8 0.7 0]); y = y - 0.07;
    text(0.10, y, '虚线 = 基础UKF融合', 'Units', 'normalized', 'FontSize', 9, 'Color', [0 0.5 0.5]); y = y - 0.07;
    text(0.10, y, '实线 = 自适应UKF融合', 'Units', 'normalized', 'FontSize', 9, 'Color', [0 0.3 0.1]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig5_fusion_map.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig5_fusion_map.png'));
    end
    fprintf('  图5 已保存: fig5_fusion_map.png\n');
end

% extract_fused - 从融合快照中提取单个航迹的经纬度序列
function [lats, lons] = extract_fused(snaps)
    lats = []; lons = [];
    for k = 1:length(snaps)
        s = snaps{k};
        if isempty(s.trackList), continue; end
        t = s.trackList{1};
        if ~isfield(t,'lat') || isnan(t.lat), continue; end
        lats(end+1) = t.lat;
        lons(end+1) = t.lon;
    end
end
