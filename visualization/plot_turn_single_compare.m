% =========================================================================
% plot_turn_single_compare.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标单站对比综合图。使用 2x3 的 tiledlayout 布局，
%   六个子图分别展示：
%   (1) R1 全图对比 — 基础 UKF(虚线) vs 自适应 UKF(实线)
%   (2) R2 全图对比
%   (3) 拐弯区域放大 — R1 和 R2 同时叠加对比
%   (4) R1 误差时间线
%   (5) R2 误差时间线
%   (6) RMSE 柱状图对比
%
% 【数学原理】
%   1. 单站精度评估：
%      分别评估 R1 和 R2 在基础 UKF 和自适应 UKF 策略下的跟踪精度。
%      误差计算使用逐帧 Haversine 距离 (km)。
%   2. 拐弯区域定位：
%      通过遍历航迹点，找到距离指定地理拐点 (128.5E, 33.5N)
%      最近的帧号作为拐弯中心，前后各扩展 18 帧作为拐弯区域。
%   3. RMSE 计算公式：
%      RMSE = sqrt(mean(x_valid.^2))，其中 x_valid 为非 NaN 的误差值集合。
%   4. 改善百分比：imp = (1 - RMSE_ad / RMSE_base) * 100%
%
% 【输入参数】
%   true_track    - Nx2 矩阵，真值航迹 [lon, lat]
%   detList_R1    - R1 检测结果元胞数组
%   detList_R2    - R2 检测结果元胞数组
%   trackR1_base  - R1 基础 UKF 跟踪快照
%   trackR2_base  - R2 基础 UKF 跟踪快照
%   trackR1_ad    - R1 自适应 UKF 跟踪快照
%   trackR2_ad    - R2 自适应 UKF 跟踪快照
%   params        - 仿真参数字段结构体，含 .dt_sec
%   out_dir       - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig2_single_compare.png  - 单站对比综合图
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   sphere_utils_haversine_distance() (球面距离)
%           extract_valid_ll()             (本文件内部)
%           err_at_frame()                 (本文件内部)
%           rms()                          (本文件内部)
%
% =========================================================================

function plot_turn_single_compare(true_track, detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, params, out_dir)

    % =====================================================================
    % 提取各航迹经纬度
    % =====================================================================
    [lat_r1b, lon_r1b] = extract_valid_ll(trackR1_base);
    [lat_r2b, lon_r2b] = extract_valid_ll(trackR2_base);
    [lat_r1a, lon_r1a] = extract_valid_ll(trackR1_ad);
    [lat_r2a, lon_r2a] = extract_valid_ll(trackR2_ad);

    % =====================================================================
    % 计算每帧位置误差 (km)
    % =====================================================================
    n_frames = length(trackR1_base);
    t = (0:n_frames-1) * params.dt_sec;  % 时间轴
    err_r1b = nan(1, n_frames);
    err_r1a = nan(1, n_frames);
    err_r2b = nan(1, n_frames);
    err_r2a = nan(1, n_frames);

    for k = 1:min(n_frames, size(true_track,1))
        tl = true_track(k,1); tb = true_track(k,2);
        err_r1b(k) = err_at_frame(trackR1_base{k}, tl, tb);
        err_r1a(k) = err_at_frame(trackR1_ad{k}, tl, tb);
        err_r2b(k) = err_at_frame(trackR2_base{k}, tl, tb);
        err_r2a(k) = err_at_frame(trackR2_ad{k}, tl, tb);
    end

    % =====================================================================
    % 定位拐弯区域：找到距离拐点 (128.5E, 33.5N) 最近的帧
    % =====================================================================
    turn_lon = 128.5; turn_lat = 33.5;
    min_dist = inf; turn_frame = round(n_frames/2);
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), turn_lon, turn_lat);
        if d < min_dist, min_dist = d; turn_frame = kk; end
    end
    zoom_half = 18;  % 拐弯前后各 18 帧
    zoom_start = max(1, turn_frame - zoom_half);
    zoom_end   = min(size(true_track,1), turn_frame + zoom_half);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % =====================================================================
    % 子图 1: R1 全图对比（基础虚线 vs 自适应实线）
    % =====================================================================
    nexttile(tlo, 1);
    try
        gx1 = geoaxes;
        gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes;
    end
    hold(gx1, 'on');
    title(gx1, 'R1: 基础UKF(虚线) vs 自适应UKF(实线)', 'FontSize', 10);

    h_truth = geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.8);
    h_r1b   = geoplot(gx1, lat_r1b, lon_r1b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 1.5);
    h_r1a   = geoplot(gx1, lat_r1a, lon_r1a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2.2);

    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    legend(gx1, [h_truth, h_r1b, h_r1a], {'真值', 'R1基础UKF', 'R1自适应UKF'}, ...
        'Location', 'northeast', 'FontSize', 7);

    % R1 拐弯区域放大白框
    rx1 = [min(lon_r1b(zoom_start:min(zoom_end,length(lon_r1b)))), ...
           max(lon_r1b(zoom_start:min(zoom_end,length(lon_r1b))))];
    ry1 = [min(lat_r1b(zoom_start:min(zoom_end,length(lat_r1b)))), ...
           max(lat_r1b(zoom_start:min(zoom_end,length(lat_r1b))))];
    geoplot(gx1, [ry1(1) ry1(1) ry1(2) ry1(2) ry1(1)], ...
                 [rx1(1) rx1(2) rx1(2) rx1(1) rx1(1)], ...
                 'w-', 'LineWidth', 1.2);

    % =====================================================================
    % 子图 2: R2 全图对比
    % =====================================================================
    nexttile(tlo, 2);
    try
        gx2 = geoaxes;
        gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes;
    end
    hold(gx2, 'on');
    title(gx2, 'R2: 基础UKF(虚线) vs 自适应UKF(实线)', 'FontSize', 10);

    geoplot(gx2, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.8);
    geoplot(gx2, lat_r2b, lon_r2b, '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 1.5);
    h_r2a_p = geoplot(gx2, lat_r2a, lon_r2a, '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 2.2);

    geoplot(gx2, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    legend(gx2, {'真值', 'R2基础UKF', 'R2自适应UKF'}, 'Location', 'northeast', 'FontSize', 7);

    % R2 拐弯区域放大白框
    valid_idx = zoom_start:min(zoom_end, length(lon_r2b));
    if ~isempty(valid_idx) && all(valid_idx <= length(lon_r2b))
        rx2 = [min(lon_r2b(valid_idx)), max(lon_r2b(valid_idx))];
        ry2 = [min(lat_r2b(valid_idx)), max(lat_r2b(valid_idx))];
        geoplot(gx2, [ry2(1) ry2(1) ry2(2) ry2(2) ry2(1)], ...
                     [rx2(1) rx2(2) rx2(2) rx2(1) rx2(1)], ...
                     'w-', 'LineWidth', 1.2);
    end

    % =====================================================================
    % 子图 3: 拐弯区域放大（R1 + R2 同时显示）
    % =====================================================================
    nexttile(tlo, 3);
    try
        gx3 = geoaxes;
        gx3.Basemap = 'darkwater';
    catch
        gx3 = geoaxes;
    end
    hold(gx3, 'on');
    title(gx3, '拐弯区域放大对比', 'FontSize', 10);

    % 真值（拐弯区域），线宽加粗
    idx_zoom = zoom_start:min(zoom_end, size(true_track,1));
    geoplot(gx3, true_track(idx_zoom,2), true_track(idx_zoom,1), 'y--', 'LineWidth', 2.5);

    % R1 航迹（拐弯区域）
    iz1 = zoom_start:min(zoom_end, length(lon_r1b));
    geoplot(gx3, lat_r1b(iz1), lon_r1b(iz1), '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2);
    geoplot(gx3, lat_r1a(iz1), lon_r1a(iz1), '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 3);

    % R2 航迹（拐弯区域）
    iz2 = zoom_start:min(zoom_end, length(lon_r2b));
    geoplot(gx3, lat_r2b(iz2), lon_r2b(iz2), '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 2);
    geoplot(gx3, lat_r2a(iz2), lon_r2a(iz2), '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 3);

    legend(gx3, {'真值','R1基础','R1自适应','R2基础','R2自适应'}, ...
        'Location', 'best', 'FontSize', 6);

    % =====================================================================
    % 子图 4: R1 误差时间线
    % =====================================================================
    ax4 = nexttile(tlo, 4);
    hold(ax4, 'on');
    t_plot = t(1:length(err_r1b));
    plot(ax4, t_plot, err_r1b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 1.5);
    plot(ax4, t_plot, err_r1a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2);
    % 竖线标记拐弯开始位置
    xline(ax4, t(turn_frame), 'k--', 'LineWidth', 0.8);
    xlabel(ax4, '时间 (s)'); ylabel(ax4, '位置误差 (km)');
    title(ax4, sprintf('R1误差: 基础RMSE=%.1fkm  自适应RMSE=%.1fkm', ...
        rms(err_r1b,'omitnan'), rms(err_r1a,'omitnan')), 'FontSize', 10);
    legend(ax4, {'基础UKF', '自适应UKF'}, 'Location', 'best', 'FontSize', 8);
    grid(ax4, 'on');

    % =====================================================================
    % 子图 5: R2 误差时间线
    % =====================================================================
    ax5 = nexttile(tlo, 5);
    hold(ax5, 'on');
    plot(ax5, t_plot, err_r2b, '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 1.5);
    plot(ax5, t_plot, err_r2a, '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 2);
    xline(ax5, t(turn_frame), 'k--', 'LineWidth', 0.8);
    xlabel(ax5, '时间 (s)'); ylabel(ax5, '位置误差 (km)');
    title(ax5, sprintf('R2误差: 基础RMSE=%.1fkm  自适应RMSE=%.1fkm', ...
        rms(err_r2b,'omitnan'), rms(err_r2a,'omitnan')), 'FontSize', 10);
    legend(ax5, {'基础UKF', '自适应UKF'}, 'Location', 'best', 'FontSize', 8);
    grid(ax5, 'on');

    % =====================================================================
    % 子图 6: RMSE 柱状图对比（分组：基础灰 vs 自适应绿）
    % =====================================================================
    ax6 = nexttile(tlo, 6);
    rmse_vals = [rms(err_r1b,'omitnan'), rms(err_r1a,'omitnan'); ...
                 rms(err_r2b,'omitnan'), rms(err_r2a,'omitnan')];
    b = bar(ax6, rmse_vals);
    b(1).FaceColor = [0.5 0.5 0.5]; b(1).DisplayName = '基础UKF';
    b(2).FaceColor = [0.0 0.4 0.0]; b(2).DisplayName = '自适应UKF';
    set(ax6, 'XTickLabel', {'R1', 'R2'});
    ylabel(ax6, 'RMSE (km)');
    title(ax6, 'RMSE对比');
    legend(ax6, 'Location', 'best', 'FontSize', 8);
    grid(ax6, 'on');

    % 在柱顶标注 RMSE 数值和改善百分比
    for i = 1:2
        imp = (1 - rmse_vals(i,2)/rmse_vals(i,1))*100;
        text(ax6, i-0.15, rmse_vals(i,1)+0.3, sprintf('%.1f', rmse_vals(i,1)), ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
        text(ax6, i+0.15, rmse_vals(i,2)+0.3, sprintf('%.1f', rmse_vals(i,2)), ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
        text(ax6, i, max(rmse_vals(i,:))+1.2, sprintf('%+.0f%%', imp), ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8 0 0], ...
            'HorizontalAlignment', 'center');
    end

    sgtitle(sprintf('拐弯目标单站对比: 基础UKF(虚线) vs 机动自适应UKF(实线)   拐角~113° Pd=%.0f%%', ...
        params.detection_probability*100));

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig2_single_compare.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig2_single_compare.png'));
    end
    fprintf('  单站对比图已保存: fig2_single_compare.png\n');
end

% =========================================================================
% extract_valid_ll - 从跟踪快照中提取有效航迹的经纬度
%   跳过 type==7（终止）、lat/lon 为 NaN 的无效帧
% =========================================================================
function [lats, lons] = extract_valid_ll(snapshots)
    lats = []; lons = [];
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        trk = snap.trackList{1};
        if trk.type == 7 || ~isfield(trk, 'lat') || isnan(trk.lat), continue; end
        lats(end+1) = trk.lat;
        lons(end+1) = trk.lon;
    end
end

% =========================================================================
% err_at_frame - 计算单帧 UKF 估计与真值之间的 Haversine 距离 (km)
%   返回 NaN 表示该帧无有效航迹估计
% =========================================================================
function d = err_at_frame(snap, t_lon, t_lat)
    d = NaN;
    if isempty(snap.trackList), return; end
    trk = snap.trackList{1};
    if trk.type == 7 || ~isfield(trk, 'lat') || isnan(trk.lat), return; end
    d = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
end

% =========================================================================
% rms - 计算忽略 NaN 后的均方根值
%   支持 'omitnan' 标志（MATLAB R2020b+ 内置 rms 函数的兼容封装）
% =========================================================================
function v = rms(x, flag)
    if nargin < 2, flag = 'omitnan'; end
    x_valid = x(~isnan(x));
    if isempty(x_valid), v = NaN; return; end
    v = sqrt(mean(x_valid.^2));
end
