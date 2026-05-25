% =========================================================================
% plot_turn_point_clouds.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标的点云与滤波航迹并排对比图。使用 tiledlayout 将图窗
%   分为左右两列：左侧为 R1 的点迹云 + 基础 UKF（虚线）+ 自适应 UKF
%   （实线），右侧为 R2 的对应内容。真实航迹以亮黄虚线叠加作为基准。
%   底图使用 darkwater。
%
% 【数学原理】
%   1. 点迹云 (Point Cloud)：
%      指所有检测到的目标点迹（不含虚警）在校准后的地理位置上
%      的散布。点迹云反映了两个雷达在不同双基地几何下的量测精度：
%      - 双基地角的差异导致方位角分辨率不同
%      - 发射机-接收机几何约束使得某些区域的定位误差椭圆变形
%   2. 滤波航迹对比：
%      在点迹云背景上叠加两种 UKF 策略的滤波航迹，直观展示：
%      - 基础 UKF（虚线）在非机动段紧跟点迹，但转弯段可能滞后
%      - 自适应 UKF（实线）在转弯段能更快响应，减少滞后
%   3. 配色方案：
%      R1：蓝色系（基础=淡蓝虚线，自适应=深蓝实线）
%      R2：红色系（基础=淡红虚线，自适应=深红实线）
%
% 【输入参数】
%   true_track    - Nx2 矩阵，真值航迹 [lon, lat]
%   detList_R1    - R1 检测结果元胞数组
%   detList_R2    - R2 检测结果元胞数组
%   trackR1_base  - R1 基础 UKF 跟踪快照
%   trackR2_base  - R2 基础 UKF 跟踪快照
%   trackR1_ad    - R1 自适应 UKF 跟踪快照
%   trackR2_ad    - R2 自适应 UKF 跟踪快照
%   params        - 仿真参数字段结构体
%   out_dir       - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig2_point_clouds.png  - 点云 + UKF航迹对比图
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   extract_det_ll()   (本文件内部)
%           extract_track_ll() (本文件内部)
%
% =========================================================================

function plot_turn_point_clouds(true_track, detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, params, out_dir)

    % 提取各图层数据
    [r1_lat, r1_lon] = extract_det_ll(detList_R1);  % R1 校准后点迹
    [r2_lat, r2_lon] = extract_det_ll(detList_R2);  % R2 校准后点迹
    [r1b_la, r1b_lo] = extract_track_ll(trackR1_base);
    [r1a_la, r1a_lo] = extract_track_ll(trackR1_ad);
    [r2b_la, r2b_lo] = extract_track_ll(trackR2_base);
    [r2a_la, r2a_lo] = extract_track_ll(trackR2_ad);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % =====================================================================
    % 左: R1 点云 + 滤波航迹
    % =====================================================================
    nexttile(tlo);
    try
        gx1 = geoaxes; gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes;
    end
    hold(gx1, 'on');
    title(gx1, 'R1: 点云 + 基础UKF(虚线) + 自适应UKF(实线)', 'FontSize', 11);

    % 真实航迹 (亮黄虚线参考线)
    geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    % 校准后点迹 (灰色散点，半透明效果)
    if ~isempty(r1_lat)
        geoplot(gx1, r1_lat, r1_lon, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3, 'DisplayName', '点迹');
    end
    % 基础 UKF (蓝色虚线) — 在转弯段可能偏离真值
    h1 = geoplot(gx1, r1b_la, r1b_lo, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2, 'DisplayName', 'R1基础UKF');
    % 自适应 UKF (深蓝实线) — 在转弯段更贴近真值
    h2 = geoplot(gx1, r1a_la, r1a_lo, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2.5, 'DisplayName', 'R1自适应UKF');

    % R1 站点
    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    legend(gx1, [h1, h2], {'基础UKF(虚线)', '自适应UKF(实线)'}, 'Location', 'southwest', 'FontSize', 8);

    % =====================================================================
    % 右: R2 点云 + 滤波航迹
    % =====================================================================
    nexttile(tlo);
    try
        gx2 = geoaxes; gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes;
    end
    hold(gx2, 'on');
    title(gx2, 'R2: 点云 + 基础UKF(虚线) + 自适应UKF(实线)', 'FontSize', 11);

    geoplot(gx2, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    if ~isempty(r2_lat)
        geoplot(gx2, r2_lat, r2_lon, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3, 'DisplayName', '点迹');
    end
    % 基础 UKF (红色虚线)
    h3 = geoplot(gx2, r2b_la, r2b_lo, '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 2, 'DisplayName', 'R2基础UKF');
    % 自适应 UKF (深红实线)
    h4 = geoplot(gx2, r2a_la, r2a_lo, '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 2.5, 'DisplayName', 'R2自适应UKF');

    geoplot(gx2, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    legend(gx2, [h3, h4], {'基础UKF(虚线)', '自适应UKF(实线)'}, 'Location', 'southwest', 'FontSize', 8);

    sgtitle(sprintf('点云 + UKF航迹对比  Pd=%.0f%%  Pfa=%.3f', ...
        params.detection_probability*100, params.false_alarm_rate));

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig2_point_clouds.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig2_point_clouds.png'));
    end
    fprintf('  图2 已保存: fig2_point_clouds.png\n');
end

% =========================================================================
% extract_det_ll - 从检测列表中提取校准后（非杂波）点迹的经纬度
%
% 【说明】遍历所有帧和所有检测，提取 is_clutter==false 且 lat/lon 非 NaN
%        的点迹。这些是已经过椭球校准、过滤了虚警的目标点迹。
% =========================================================================
function [lats, lons] = extract_det_ll(detList)
    lats = []; lons = [];
    for k = 1:length(detList)
        for d = 1:length(detList{k})
            dp = detList{k}(d);
            if dp.is_clutter, continue; end
            if isfield(dp, 'lat') && ~isnan(dp.lat)
                lats(end+1) = dp.lat;
                lons(end+1) = dp.lon;
            end
        end
    end
end

% =========================================================================
% extract_track_ll - 从跟踪快照中提取单个航迹的经纬度序列
%
% 【说明】仅提取 type!=7（非终止状态）且 lat/lon 非 NaN 的帧。
%         type==7 通常表示航迹已终止/失跟。
% =========================================================================
function [lats, lons] = extract_track_ll(snaps)
    lats = []; lons = [];
    for k = 1:length(snaps)
        s = snaps{k};
        if isempty(s.trackList), continue; end
        t = s.trackList{1};
        if t.type == 7 || ~isfield(t,'lat') || isnan(t.lat), continue; end
        lats(end+1) = t.lat;
        lons(end+1) = t.lon;
    end
end
