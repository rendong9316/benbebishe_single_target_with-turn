% =========================================================================
% plot_turn_fusion_map.m
% 图5: 融合地图对比 — 基础融合(虚线) + 自适应融合(实线) + 真值
% =========================================================================

function plot_turn_fusion_map(true_track, ...
        fused_base, fuse_methods, best_m_base, ...
        fused_ad, fuse_methods_ad, best_m_ad, params, out_dir)

    [lat_fb, lon_fb] = extract_fused(fused_base{best_m_base});
    [lat_fa, lon_fa] = extract_fused(fused_ad{best_m_ad});

    % 拐点
    tf = round(size(true_track,1)/2); md = inf;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), 128.5, 33.5);
        if d < md, md = d; tf = kk; end
    end
    zs = max(1, tf-20); ze = min(size(true_track,1), tf+20);

    fig = figure('Position', [50, 50, 1400, 750]);

    % ---- 左: 融合全图 ----
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

    % 拐弯区域框
    rx = [min(true_track(zs:ze,1)), max(true_track(zs:ze,1))];
    ry = [min(true_track(zs:ze,2)), max(true_track(zs:ze,2))];
    geoplot(gx, [ry(1) ry(1) ry(2) ry(2) ry(1)], [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);

    legend(gx, 'Location', 'southwest', 'FontSize', 9);

    % ---- 右上: 拐弯放大 ----
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

    % ---- 右下: 信息面板 ----
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
