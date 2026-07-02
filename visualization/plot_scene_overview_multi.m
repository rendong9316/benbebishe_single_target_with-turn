% =========================================================================
% plot_scene_overview_multi.m — 多目标场景总览图
% =========================================================================
function plot_scene_overview_multi(true_track_A, true_track_B, true_track_C, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Basemap', 'darkwater');
    catch
        ax = geoaxes();
    end
    hold(ax, 'on');

    % Stations
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1 Rx');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2 Rx');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Tx2');

    % Beam sectors (inline, avoiding draw_beam_sector geoaxes RGBA bug)
    draw_beam_sector_geoax(ax, params.radar1_lat, params.radar1_lon, ...
        params.radar1_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [0.3 0.6 1.0]);
    draw_beam_sector_geoax(ax, params.radar2_lat, params.radar2_lon, ...
        params.radar2_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [1.0 0.4 0.4]);

    % Truth tracks
    track_colors = {'g', 'm', 'c'};
    track_lines  = {'-', '-', '-'};
    track_names  = {'Target A', 'Target B', 'Target C'};
    track_data   = {true_track_A, true_track_B, true_track_C};
    for i = 1:3
        tt = track_data{i};
        geoplot(ax, tt(:,2), tt(:,1), strcat(track_lines{i}), 'Color', track_colors{i}, 'LineWidth', 2, ...
            'DisplayName', track_names{i});
        % Start marker
        geoplot(ax, tt(1,2), tt(1,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        % End marker
        geoplot(ax, tt(end,2), tt(end,1), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'Color', track_colors{i});
    end

    title(ax, '双基地外辐射源雷达多目标仿真场景');
    subtitle(ax, sprintf('Pd=%.0f%%, Pfa=%.3f, dt=%.0fs, %d-%d km', ...
        params.detection_probability*100, params.false_alarm_rate, ...
        params.dt_sec, params.range_min_km, params.range_max_km));
    legend(ax, 'Location', 'best');
    drawnow;
end

% =========================================================================
function draw_beam_sector_geoax(ax, rx_lat, rx_lon, center_az, width, r_min, r_max, color)
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
    % geoaxes doesn't support alpha in color vector — use solid lines
    geoplot(ax, lats_inner, lons_inner, '--', 'Color', color, 'LineWidth', 1);
    geoplot(ax, lats_outer, lons_outer, '--', 'Color', color, 'LineWidth', 1);
    for az_edge = [center_az - width/2, center_az + width/2]
        [lon1, lat1] = sphere_utils_destination_point(rx_lon, rx_lat, r_min, az_edge);
        [lon2, lat2] = sphere_utils_destination_point(rx_lon, rx_lat, r_max, az_edge);
        geoplot(ax, [lat1 lat2], [lon1 lon2], '-', 'Color', color, 'LineWidth', 1);
    end
end
