% =========================================================================
% plot_scene_overview.m
% 图1: 场景总览 — geoplot 显示站点、波束、威力范围、航迹 (darkwater底图)
% =========================================================================

function plot_scene_overview(true_track, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Basemap', 'darkwater');
    catch
        ax = geoaxes();
    end
    hold(ax, 'on');

    % 接收站
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');

    % 发射站
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Tx2');

    % R1 波束扇形 (亮色适配暗底图)
    draw_beam_sector(ax, params.radar1_lat, params.radar1_lon, ...
        params.radar1_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [0.3 0.6 1.0]);

    % R2 波束扇形
    draw_beam_sector(ax, params.radar2_lat, params.radar2_lon, ...
        params.radar2_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [1.0 0.4 0.4]);

    % 真实航迹 (亮色适配暗底图)
    geoplot(ax, true_track(:,2), true_track(:,1), 'y-', 'LineWidth', 2, ...
        'DisplayName', '目标真实航迹');

    geoplot(ax, true_track(1,2), true_track(1,1), 'yo', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'g');
    geoplot(ax, true_track(end,2), true_track(end,1), 'yx', 'MarkerSize', 10, ...
        'LineWidth', 2);

    title(ax, '双基地外辐射源雷达仿真场景');
    subtitle(ax, sprintf('Pd=%.0f%%, Pfa=%.3f, dt=%.0fs, 波束15°, %d-%d km', ...
        params.detection_probability*100, params.false_alarm_rate, ...
        params.dt_sec, params.range_min_km, params.range_max_km));
    legend(ax, 'Location', 'best');

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig1_scene_overview.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig1_scene_overview.png'));
    end
    fprintf('  图1 已保存: fig1_scene_overview.png\n');
end

function draw_beam_sector(ax, rx_lat, rx_lon, center_az, width, r_min, r_max, color)
    az_edges = linspace(center_az - width/2, center_az + width/2, 20);
    lats_inner = zeros(1, length(az_edges));
    lons_inner = zeros(1, length(az_edges));
    lats_outer = zeros(1, length(az_edges));
    lons_outer = zeros(1, length(az_edges));
    for i = 1:length(az_edges)
        [lons_inner(i), lats_inner(i)] = sphere_utils_destination_point(...
            rx_lon, rx_lat, r_min, az_edges(i));
        [lons_outer(i), lats_outer(i)] = sphere_utils_destination_point(...
            rx_lon, rx_lat, r_max, az_edges(i));
    end
    geoplot(ax, lats_inner, lons_inner, '--', 'Color', [color 0.8], 'LineWidth', 1);
    geoplot(ax, lats_outer, lons_outer, '--', 'Color', [color 0.8], 'LineWidth', 1);
    for az_edge = [center_az - width/2, center_az + width/2]
        [lon1, lat1] = sphere_utils_destination_point(rx_lon, rx_lat, r_min, az_edge);
        [lon2, lat2] = sphere_utils_destination_point(rx_lon, rx_lat, r_max, az_edge);
        geoplot(ax, [lat1 lat2], [lon1 lon2], '-', 'Color', [color 0.5], 'LineWidth', 1);
    end
end
