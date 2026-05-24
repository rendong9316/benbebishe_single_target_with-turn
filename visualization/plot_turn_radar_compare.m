% =========================================================================
% plot_turn_radar_compare.m
% 图3(R1)/图4(R2): 单站对比 — 地图(上) + 误差时间线(下)
% 基础UKF=虚线, 自适应UKF=实线
% =========================================================================

function plot_turn_radar_compare(true_track, track_base, track_ad, ...
        radar_label, rx_lat, rx_lon, params, out_dir, fig_num)

    [lat_b, lon_b] = extract_ll(track_base);
    [lat_a, lon_a] = extract_ll(track_ad);

    % 计算每帧误差
    n_frames = length(track_base);
    t_vec = (0:n_frames-1) * params.dt_sec;
    err_b = nan(1, n_frames);
    err_a = nan(1, n_frames);
    for k = 1:min(n_frames, size(true_track,1))
        err_b(k) = err_at(track_base{k}, true_track(k,1), true_track(k,2));
        err_a(k) = err_at(track_ad{k}, true_track(k,1), true_track(k,2));
    end

    % 拐点帧
    tf = round(n_frames/2); md = inf;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), 128.5, 33.5);
        if d < md, md = d; tf = kk; end
    end

    fig = figure('Position', [50, 50, 1400, 750]);

    % ---- 上半: 地图 ----
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

    % 拐弯区域框
    zs = max(1, tf-18); ze = min(size(true_track,1), tf+18);
    rx = [min(true_track(zs:ze,1)), max(true_track(zs:ze,1))];
    ry = [min(true_track(zs:ze,2)), max(true_track(zs:ze,2))];
    geoplot(gx, [ry(1) ry(1) ry(2) ry(2) ry(1)], [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);
    legend(gx, {'真值', '基础UKF', '自适应UKF'}, 'Location', 'southwest', 'FontSize', 9);

    % ---- 右上: 拐弯区域放大 ----
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

    % ---- 下半: 误差时间线 ----
    ax_err = axes('Units', 'normalized', 'Position', [0.08, 0.06, 0.55, 0.30]);
    hold(ax_err, 'on');
    plot(ax_err, t_vec, err_b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 1.5);
    plot(ax_err, t_vec, err_a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2);
    xline(ax_err, t_vec(tf), 'k--', 'LineWidth', 0.8);
    xlabel(ax_err, '时间 (s)'); ylabel(ax_err, '误差 (km)');
    rmse_b = rms_nan(err_b); rmse_a = rms_nan(err_a);
    imp = (1 - rmse_a/rmse_b)*100;
    title(ax_err, sprintf('基础RMSE=%.1fkm  自适应RMSE=%.1fkm  改善%+.0f%%', rmse_b, rmse_a, imp), 'FontSize', 11);
    legend(ax_err, {'基础UKF', '自适应UKF'}, 'Location', 'best', 'FontSize', 9);
    grid(ax_err, 'on');

    % ---- 右下: RMSE柱状图 ----
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

function [lats, lons] = extract_ll(snaps)
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

function d = err_at(snap, tl, tb)
    d = NaN;
    if isempty(snap.trackList), return; end
    t = snap.trackList{1};
    if t.type == 7 || ~isfield(t,'lat') || isnan(t.lat), return; end
    d = sphere_utils_haversine_distance(t.lon, t.lat, tl, tb) / 1000;
end

function v = rms_nan(x)
    xv = x(~isnan(x));
    if isempty(xv), v = NaN; else, v = sqrt(mean(xv.^2)); end
end
