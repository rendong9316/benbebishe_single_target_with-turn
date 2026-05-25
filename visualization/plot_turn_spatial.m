% =========================================================================
% plot_turn_spatial.m
% 聚合式绘图调度文件 — 拐弯目标空间/地理可视化
% =========================================================================
%
% 【功能概述】
%   将原来分散在 4 个独立 .m 文件中的空间/地理绘图函数合并到一个
%   文件中，通过 mode 字符串调度对应的子函数。
%
% 【调度模式】
%   'point_clouds'   -> plot_turn_point_clouds(...)
%   'radar_compare'  -> plot_turn_radar_compare(...)
%   'fusion_map'     -> plot_turn_fusion_map(...)
%   'comprehensive'  -> plot_turn_comprehensive(...)
%
% 【内部辅助函数命名约定】
%   为避免多个子函数之间的同名冲突，所有辅助函数均添加父函数缩写后缀。
%   tpc  = turn_point_clouds
%   trc  = turn_radar_compare
%   tfm  = turn_fusion_map
%   tc   = turn_comprehensive
%
% =========================================================================

function plot_turn_spatial(mode, varargin)
    switch mode
        case 'point_clouds'
            plot_turn_point_clouds(varargin{:});
        case 'radar_compare'
            plot_turn_radar_compare(varargin{:});
        case 'fusion_map'
            plot_turn_fusion_map(varargin{:});
        case 'comprehensive'
            plot_turn_comprehensive(varargin{:});
        otherwise
            error('plot_turn_spatial: unknown mode "%s". Valid: point_clouds, radar_compare, fusion_map, comprehensive', mode);
    end
end

% =========================================================================
% 1. plot_turn_point_clouds — 拐弯目标的点云与滤波航迹并排对比图
% 来源: plot_turn_point_clouds.m
% =========================================================================
function plot_turn_point_clouds(true_track, detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, params, out_dir)

    [r1_lat, r1_lon] = extract_det_ll_tpc(detList_R1);
    [r2_lat, r2_lon] = extract_det_ll_tpc(detList_R2);
    [r1b_la, r1b_lo] = extract_track_ll_tpc(trackR1_base);
    [r1a_la, r1a_lo] = extract_track_ll_tpc(trackR1_ad);
    [r2b_la, r2b_lo] = extract_track_ll_tpc(trackR2_base);
    [r2a_la, r2a_lo] = extract_track_ll_tpc(trackR2_ad);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(tlo);
    try
        gx1 = geoaxes; gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes;
    end
    hold(gx1, 'on');
    title(gx1, 'R1: 点云 + 基础UKF(虚线) + 自适应UKF(实线)', 'FontSize', 11);

    geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    if ~isempty(r1_lat)
        geoplot(gx1, r1_lat, r1_lon, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3, 'DisplayName', '点迹');
    end
    h1 = geoplot(gx1, r1b_la, r1b_lo, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2, 'DisplayName', 'R1基础UKF');
    h2 = geoplot(gx1, r1a_la, r1a_lo, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2.5, 'DisplayName', 'R1自适应UKF');

    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    legend(gx1, [h1, h2], {'基础UKF(虚线)', '自适应UKF(实线)'}, 'Location', 'southwest', 'FontSize', 8);

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
    h3 = geoplot(gx2, r2b_la, r2b_lo, '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 2, 'DisplayName', 'R2基础UKF');
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

function [lats, lons] = extract_det_ll_tpc(detList)
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

function [lats, lons] = extract_track_ll_tpc(snaps)
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

% =========================================================================
% 2. plot_turn_radar_compare — 单雷达站拐弯场景基础UKF vs 自适应UKF对比
% 来源: plot_turn_radar_compare.m
% =========================================================================
function plot_turn_radar_compare(true_track, track_base, track_ad, ...
        radar_label, rx_lat, rx_lon, params, out_dir, fig_num)

    [lat_b, lon_b] = extract_ll_trc(track_base);
    [lat_a, lon_a] = extract_ll_trc(track_ad);

    n_frames = length(track_base);
    t_vec = (0:n_frames-1) * params.dt_sec;
    err_b = nan(1, n_frames);
    err_a = nan(1, n_frames);
    for k = 1:min(n_frames, size(true_track,1))
        err_b(k) = err_at_trc(track_base{k}, true_track(k,1), true_track(k,2));
        err_a(k) = err_at_trc(track_ad{k}, true_track(k,1), true_track(k,2));
    end

    tf = round(n_frames/2); md = inf;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), 128.5, 33.5);
        if d < md, md = d; tf = kk; end
    end

    fig = figure('Position', [50, 50, 1400, 750]);

    try
        gx = geoaxes('Units', 'normalized', 'Position', [0.05, 0.42, 0.60, 0.56]);
        gx.Basemap = 'darkwater';
    catch
        gx = geoaxes('Units', 'normalized', 'Position', [0.05, 0.42, 0.60, 0.56]);
    end
    hold(gx, 'on');
    title(gx, sprintf('%s: 基础UKF(虚线) vs 自适应UKF(实线)', radar_label), 'FontSize', 12);

    geoplot(gx, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    hb = geoplot(gx, lat_b, lon_b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2, 'DisplayName', '基础UKF');
    ha = geoplot(gx, lat_a, lon_a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2.5, 'DisplayName', '自适应UKF');

    geoplot(gx, rx_lat, rx_lon, 's', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', radar_label);
    geoplot(gx, true_track(tf,2), true_track(tf,1), 'wo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '拐点');

    zs = max(1, tf-18); ze = min(size(true_track,1), tf+18);
    rx = [min(true_track(zs:ze,1)), max(true_track(zs:ze,1))];
    ry = [min(true_track(zs:ze,2)), max(true_track(zs:ze,2))];
    geoplot(gx, [ry(1) ry(1) ry(2) ry(2) ry(1)], [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);
    legend(gx, {'真值', '基础UKF', '自适应UKF'}, 'Location', 'southwest', 'FontSize', 9);

    try
        gz = geoaxes('Units', 'normalized', 'Position', [0.67, 0.55, 0.30, 0.42]);
        gz.Basemap = 'darkwater';
    catch
        gz = geoaxes('Units', 'normalized', 'Position', [0.67, 0.55, 0.30, 0.42]);
    end
    hold(gz, 'on');
    title(gz, '拐弯区域放大', 'FontSize', 9);

    iz = zs:min(ze, min([length(lat_b), length(lat_a)]));
    geoplot(gz, true_track(zs:ze,2), true_track(zs:ze,1), 'y--', 'LineWidth', 2.5);
    geoplot(gz, lat_b(iz), lon_b(iz), '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2);
    geoplot(gz, lat_a(iz), lon_a(iz), '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 3);
    legend(gz, {'真值', '基础', '自适应'}, 'Location', 'best', 'FontSize', 7);

    ax_err = axes('Units', 'normalized', 'Position', [0.08, 0.06, 0.55, 0.30]);
    hold(ax_err, 'on');
    plot(ax_err, t_vec, err_b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 1.5);
    plot(ax_err, t_vec, err_a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2);
    xline(ax_err, t_vec(tf), 'k--', 'LineWidth', 0.8);
    xlabel(ax_err, '时间 (s)'); ylabel(ax_err, '误差 (km)');

    rmse_b = rms_nan_trc(err_b); rmse_a = rms_nan_trc(err_a);
    imp = (1 - rmse_a/rmse_b)*100;
    title(ax_err, sprintf('基础RMSE=%.1fkm  自适应RMSE=%.1fkm  改善%+.0f%%', rmse_b, rmse_a, imp), 'FontSize', 11);
    legend(ax_err, {'基础UKF', '自适应UKF'}, 'Location', 'best', 'FontSize', 9);
    grid(ax_err, 'on');

    ax_bar = axes('Units', 'normalized', 'Position', [0.68, 0.08, 0.28, 0.28]);
    bar(ax_bar, [1 2], [rmse_b, rmse_a], 'FaceColor', 'flat');
    ax_bar.Children.CData(1,:) = [0.0 0.4 0.0];
    ax_bar.Children.CData(2,:) = [0.6 0.6 0.6];
    set(ax_bar, 'XTickLabel', {'基础UKF', '自适应UKF'}, 'FontSize', 9);
    ylabel(ax_bar, 'RMSE (km)');
    title(ax_bar, sprintf('改善 %+.0f%%', imp), 'FontSize', 10);
    grid(ax_bar, 'on');
    text(ax_bar, 1, rmse_b+0.3, sprintf('%.1f', rmse_b), 'HorizontalAlignment', 'center', 'FontSize', 9);
    text(ax_bar, 2, rmse_a+0.3, sprintf('%.1f', rmse_a), 'HorizontalAlignment', 'center', 'FontSize', 9);

    fname = sprintf('fig%d_%s_compare.png', fig_num, radar_label);
    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, fname), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, fname));
    end
    fprintf('  图%d 已保存: %s\n', fig_num, fname);
end

function [lats, lons] = extract_ll_trc(snaps)
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

function d = err_at_trc(snap, tl, tb)
    d = NaN;
    if isempty(snap.trackList), return; end
    t = snap.trackList{1};
    if t.type == 7 || ~isfield(t,'lat') || isnan(t.lat), return; end
    d = sphere_utils_haversine_distance(t.lon, t.lat, tl, tb) / 1000;
end

function v = rms_nan_trc(x)
    xv = x(~isnan(x));
    if isempty(xv), v = NaN; else, v = sqrt(mean(xv.^2)); end
end

% =========================================================================
% 3. plot_turn_fusion_map — 拐弯目标的融合航迹地图对比图
% 来源: plot_turn_fusion_map.m
% =========================================================================
function plot_turn_fusion_map(true_track, ...
        fused_base, fuse_methods, best_m_base, ...
        fused_ad, fuse_methods_ad, best_m_ad, params, out_dir)

    [lat_fb, lon_fb] = extract_fused_tfm(fused_base{best_m_base});
    [lat_fa, lon_fa] = extract_fused_tfm(fused_ad{best_m_ad});

    tf = round(size(true_track,1)/2); md = inf;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), 128.5, 33.5);
        if d < md, md = d; tf = kk; end
    end
    zs = max(1, tf-20); ze = min(size(true_track,1), tf+20);

    fig = figure('Position', [50, 50, 1400, 750]);

    try
        gx = geoaxes('Units', 'normalized', 'Position', [0.04, 0.08, 0.62, 0.90]);
        gx.Basemap = 'darkwater';
    catch
        gx = geoaxes('Units', 'normalized', 'Position', [0.04, 0.08, 0.62, 0.90]);
    end
    hold(gx, 'on');
    title(gx, sprintf('融合航迹: 基础%s(虚线) vs 自适应%s(实线)', ...
        fuse_methods{best_m_base}, fuse_methods_ad{best_m_ad}), 'FontSize', 12);

    geoplot(gx, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    geoplot(gx, lat_fb, lon_fb, '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2.2, ...
        'DisplayName', sprintf('基础%s融合', fuse_methods{best_m_base}));
    geoplot(gx, lat_fa, lon_fa, '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 3, ...
        'DisplayName', sprintf('自适应%s融合', fuse_methods_ad{best_m_ad}));

    geoplot(gx, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    geoplot(gx, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    geoplot(gx, true_track(tf,2), true_track(tf,1), 'wo', 'MarkerSize', 8, 'LineWidth', 2);

    rx = [min(true_track(zs:ze,1)), max(true_track(zs:ze,1))];
    ry = [min(true_track(zs:ze,2)), max(true_track(zs:ze,2))];
    geoplot(gx, [ry(1) ry(1) ry(2) ry(2) ry(1)], [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);

    legend(gx, 'Location', 'southwest', 'FontSize', 9);

    try
        gz = geoaxes('Units', 'normalized', 'Position', [0.68, 0.55, 0.30, 0.42]);
        gz.Basemap = 'darkwater';
    catch
        gz = geoaxes('Units', 'normalized', 'Position', [0.68, 0.55, 0.30, 0.42]);
    end
    hold(gz, 'on');
    title(gz, '拐弯区域放大', 'FontSize', 9);

    geoplot(gz, true_track(zs:ze,2), true_track(zs:ze,1), 'y--', 'LineWidth', 2.5);
    if ~isempty(lat_fb)
        iz = zs:min(ze, length(lat_fb));
        geoplot(gz, lat_fb(iz), lon_fb(iz), '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2);
    end
    if ~isempty(lat_fa)
        iz = zs:min(ze, length(lat_fa));
        geoplot(gz, lat_fa(iz), lon_fa(iz), '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 3);
    end
    legend(gz, {'真值', '基础融合', '自适应融合'}, 'Location', 'best', 'FontSize', 7);

    ax_info = axes('Units', 'normalized', 'Position', [0.68, 0.06, 0.30, 0.44]);
    ax_info.Visible = 'off';
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

function [lats, lons] = extract_fused_tfm(snaps)
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

% =========================================================================
% 4. plot_turn_comprehensive — 拐弯目标全流程综合对比图
% 来源: plot_turn_comprehensive.m
% =========================================================================
function plot_turn_comprehensive(true_track, ...
        detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, ...
        fused_scc_base, fused_scc_ad, params, out_dir)

    [raw1_la, raw1_lo] = extract_det_ll_tc(detList_R1, 'raw');
    [raw2_la, raw2_lo] = extract_det_ll_tc(detList_R2, 'raw');
    [cal1_la, cal1_lo] = extract_det_ll_tc(detList_R1, 'cal');
    [cal2_la, cal2_lo] = extract_det_ll_tc(detList_R2, 'cal');
    [r1b_la, r1b_lo] = extract_track_ll_tc(trackR1_base);
    [r2b_la, r2b_lo] = extract_track_ll_tc(trackR2_base);
    [r1a_la, r1a_lo] = extract_track_ll_tc(trackR1_ad);
    [r2a_la, r2a_lo] = extract_track_ll_tc(trackR2_ad);
    [fb_la, fb_lo] = extract_fused_ll_tc(fused_scc_base);
    [fa_la, fa_lo] = extract_fused_ll_tc(fused_scc_ad);

    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.03, 0.08, 0.72, 0.90]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.03, 0.08, 0.72, 0.90]);
    end
    hold(ax, 'on');

    h_all = {};
    layer_names = {};

    h = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2.5, 'DisplayName', '真实航迹');
    h_all{end+1} = h; layer_names{end+1} = '真实航迹';

    if ~isempty(raw1_la)
        h = geoplot(ax, raw1_la, raw1_lo, '--.', ...
            'Color', [0.5 0.7 1.0], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'DisplayName', 'R1 原始量测(未校准)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 原始量测(校准前)';
    end

    if ~isempty(raw2_la)
        h = geoplot(ax, raw2_la, raw2_lo, '--.', ...
            'Color', [1.0 0.65 0.65], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'DisplayName', 'R2 原始量测(未校准)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 原始量测(校准前)';
    end

    if ~isempty(cal1_la)
        h = geoplot(ax, cal1_la, cal1_lo, '-o', ...
            'Color', [0.2 0.4 0.9], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.2 0.4 0.9], 'DisplayName', 'R1 校准量测(未滤波)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 校准后量测';
    end

    if ~isempty(cal2_la)
        h = geoplot(ax, cal2_la, cal2_lo, '-o', ...
            'Color', [0.9 0.3 0.3], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.9 0.3 0.3], 'DisplayName', 'R2 校准量测(未滤波)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 校准后量测';
    end

    if ~isempty(r1b_la)
        h = geoplot(ax, r1b_la, r1b_lo, '--', ...
            'Color', [0.2 0.4 0.9], 'LineWidth', 1.8, ...
            'DisplayName', 'R1 基础UKF');
        h_all{end+1} = h; layer_names{end+1} = 'R1 基础UKF';
    end

    if ~isempty(r2b_la)
        h = geoplot(ax, r2b_la, r2b_lo, '--', ...
            'Color', [0.9 0.3 0.3], 'LineWidth', 1.8, ...
            'DisplayName', 'R2 基础UKF');
        h_all{end+1} = h; layer_names{end+1} = 'R2 基础UKF';
    end

    if ~isempty(r1a_la)
        h = geoplot(ax, r1a_la, r1a_lo, '-', ...
            'Color', [0.0 0.1 0.6], 'LineWidth', 2.2, ...
            'DisplayName', 'R1 自适应UKF(机动检测)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 自适应UKF(机动检测)';
    end

    if ~isempty(r2a_la)
        h = geoplot(ax, r2a_la, r2a_lo, '-', ...
            'Color', [0.7 0.0 0.0], 'LineWidth', 2.2, ...
            'DisplayName', 'R2 自适应UKF(机动检测)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 自适应UKF(机动检测)';
    end

    if ~isempty(fb_la)
        h = geoplot(ax, fb_la, fb_lo, '--', ...
            'Color', [0.0 0.7 0.7], 'LineWidth', 2.5, ...
            'DisplayName', '基础UKF SCC融合');
        h_all{end+1} = h; layer_names{end+1} = '基础UKF SCC融合';
    end

    if ~isempty(fa_la)
        h = geoplot(ax, fa_la, fa_lo, '-', ...
            'Color', [0.0 0.45 0.0], 'LineWidth', 3.0, ...
            'DisplayName', '自适应UKF SCC融合');
        h_all{end+1} = h; layer_names{end+1} = '自适应UKF SCC融合';
    end

    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1站');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2站');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Tx2');

    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', '终点');

    title(ax, '双基地雷达拐弯目标全流程对比');
    subtitle(ax, sprintf('原始量测 → 校准 → 基础UKF滤波 → 自适应UKF滤波 → SCC融合 | Pd=%.0f%% Pfa=%.3f 拐角~113°', ...
        params.detection_probability*100, params.false_alarm_rate));

    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);
    row_h = min(0.055, 0.88 / n_layers);

    for i = 1:n_layers
        ypos = 0.93 - (i-1) * row_h;
        if ypos < 0.04, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.77, ypos, 0.21, row_h*0.85], ...
            'FontSize', 7, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_vis_tc(h_all{i}, src.Value));
    end

    btn_y = 0.93 - n_layers * row_h - 0.015;
    if btn_y > 0.01
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.77, btn_y, 0.10, 0.035], ...
            'FontSize', 7, ...
            'Callback', @(src, ~) toggle_all_tc(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.88, btn_y, 0.10, 0.035], ...
            'FontSize', 7, ...
            'Callback', @(~, ~) show_all_tc(cb, h_all));
    end

    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.77, 0.003, 0.21, 0.03], ...
        'String', sprintf('R1点迹:%d R2点迹:%d | Pd=%.0f%% Pfa=%.3f', ...
        length(raw1_la), length(raw2_la), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 7, 'BackgroundColor', [1 1 1]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig7_comprehensive.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig7_comprehensive.png'));
    end
    fprintf('  图7 已保存: fig7_comprehensive.png\n');
end

function [lats, lons] = extract_det_ll_tc(detList, mode)
    lats = []; lons = [];
    for k = 1:length(detList)
        for d = 1:length(detList{k})
            dp = detList{k}(d);
            if dp.is_clutter, continue; end
            if strcmp(mode, 'raw')
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    lats(end+1) = dp.raw_lat;
                    lons(end+1) = dp.raw_lon;
                end
            else
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    lats(end+1) = dp.lat;
                    lons(end+1) = dp.lon;
                end
            end
        end
    end
end

function [lats, lons] = extract_track_ll_tc(snaps)
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

function [lats, lons] = extract_fused_ll_tc(snaps)
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

function try_set_vis_tc(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_tc(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_vis_tc(h_all{i}, new_val);
    end
end

function show_all_tc(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_vis_tc(h_all{i}, 1);
    end
end
