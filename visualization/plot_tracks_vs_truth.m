% =========================================================================
% plot_tracks_vs_truth.m
% 图3: UKF滤波航迹 vs 真实航迹 (geoplot, darkwater底图)
% =========================================================================

function plot_tracks_vs_truth(trackState_R1, trackState_R2, true_track, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- R1 ----
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

    % ---- R2 ----
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

function plot_track_on_map(ax, stateList, true_track, rx_lat, rx_lon)
    % 真实航迹 (亮黄虚线, 适配暗底图)
    geoplot(ax, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.5, ...
        'DisplayName', '真实航迹');

    % 滤波航迹 (青色)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~isfield(s, 'lat') || isnan(s.lat), continue; end
        lats(end+1) = s.lat;
        lons(end+1) = s.lon;
    end
    if ~isempty(lats)
        geoplot(ax, lats, lons, 'c-', 'LineWidth', 1.5, 'DisplayName', 'UKF滤波');
    end

    % 接收站
    geoplot(ax, rx_lat, rx_lon, 'rs', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'DisplayName', '接收站');

    legend(ax, 'Location', 'best');
end
