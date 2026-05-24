% =========================================================================
% plot_turn_point_clouds.m
% 图2: R1/R2点云 + 基础UKF(虚线) + 自适应UKF(实线) 并排
% =========================================================================

function plot_turn_point_clouds(true_track, detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, params, out_dir)

    [r1_lat, r1_lon] = extract_det_ll(detList_R1);
    [r2_lat, r2_lon] = extract_det_ll(detList_R2);
    [r1b_la, r1b_lo] = extract_track_ll(trackR1_base);
    [r1a_la, r1a_lo] = extract_track_ll(trackR1_ad);
    [r2b_la, r2b_lo] = extract_track_ll(trackR2_base);
    [r2a_la, r2a_lo] = extract_track_ll(trackR2_ad);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- R1 ----
    nexttile(tlo);
    try
        gx1 = geoaxes; gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes;
    end
    hold(gx1, 'on');
    title(gx1, 'R1: 点云 + 基础UKF(虚线) + 自适应UKF(实线)', 'FontSize', 11);

    % 真实航迹
    geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 2, 'DisplayName', '真值');
    % 校准后点迹
    if ~isempty(r1_lat)
        geoplot(gx1, r1_lat, r1_lon, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3, 'DisplayName', '点迹');
    end
    % 基础UKF (虚线)
    h1 = geoplot(gx1, r1b_la, r1b_lo, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2, 'DisplayName', 'R1基础UKF');
    % 自适应UKF (实线)
    h2 = geoplot(gx1, r1a_la, r1a_lo, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2.5, 'DisplayName', 'R1自适应UKF');

    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    legend(gx1, [h1, h2], {'基础UKF(虚线)', '自适应UKF(实线)'}, 'Location', 'southwest', 'FontSize', 8);

    % ---- R2 ----
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
