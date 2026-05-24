% =========================================================================
% plot_turn_comparison.m
% 拐弯航迹对比图: 真实航迹 + 基础UKF + 机动自适应UKF + 融合结果
% 带复选框图层控制 (darkwater底图)
% =========================================================================

function plot_turn_comparison(true_track, ...
        trackR1_base, trackR2_base, fused_base, fuse_methods, best_m_base, ...
        trackR1_ad, trackR2_ad, fused_ad, fuse_methods_ad, best_m_ad, ...
        params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);

    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.03, 0.12, 0.68, 0.86]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.03, 0.12, 0.68, 0.86]);
    end
    hold(ax, 'on');

    h_all = [];
    layer_names = {};

    % ---- 真实航迹 (亮黄虚线) ----
    h = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2.5, 'DisplayName', '真实航迹');
    h_all(end+1) = h;
    layer_names{end+1} = '真实航迹';

    % ---- R1 基础UKF (淡蓝色) ----
    [lat1, lon1] = extract_track_ll(trackR1_base);
    if ~isempty(lat1)
        h = geoplot(ax, lat1, lon1, '-', 'Color', [0.3 0.5 1.0], ...
            'LineWidth', 1.8, 'DisplayName', 'R1 基础UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 基础UKF(模糊Q)';
    end

    % ---- R2 基础UKF (淡红色) ----
    [lat2, lon2] = extract_track_ll(trackR2_base);
    if ~isempty(lat2)
        h = geoplot(ax, lat2, lon2, '-', 'Color', [1.0 0.4 0.4], ...
            'LineWidth', 1.8, 'DisplayName', 'R2 基础UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 基础UKF(模糊Q)';
    end

    % ---- R1 自适应UKF (蓝色实线) ----
    [lat1a, lon1a] = extract_track_ll(trackR1_ad);
    if ~isempty(lat1a)
        h = geoplot(ax, lat1a, lon1a, 'b-', ...
            'LineWidth', 2.2, 'DisplayName', 'R1 自适应UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 自适应UKF(机动检测)';
    end

    % ---- R2 自适应UKF (红色实线) ----
    [lat2a, lon2a] = extract_track_ll(trackR2_ad);
    if ~isempty(lat2a)
        h = geoplot(ax, lat2a, lon2a, 'r-', ...
            'LineWidth', 2.2, 'DisplayName', 'R2 自适应UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 自适应UKF(机动检测)';
    end

    % ---- 基础UKF最优融合 (青色) ----
    [lat_fb, lon_fb] = extract_fused_ll(fused_base{best_m_base});
    if ~isempty(lat_fb)
        h = geoplot(ax, lat_fb, lon_fb, 'c-', ...
            'LineWidth', 2.5, 'DisplayName', '基础UKF融合');
        h_all(end+1) = h;
        layer_names{end+1} = sprintf('基础UKF融合(%s)', fuse_methods{best_m_base});
    end

    % ---- 自适应UKF最优融合 (品红) ----
    [lat_fa, lon_fa] = extract_fused_ll(fused_ad{best_m_ad});
    if ~isempty(lat_fa)
        h = geoplot(ax, lat_fa, lon_fa, 'm-', ...
            'LineWidth', 2.5, 'DisplayName', '自适应UKF融合');
        h_all(end+1) = h;
        layer_names{end+1} = sprintf('自适应UKF融合(%s)', fuse_methods_ad{best_m_ad});
    end

    % ---- 站点标记 ----
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    % 起点/终点
    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', '终点');

    % 标注拐点
    mid_idx = round(size(true_track,1)/2);
    geoplot(ax, true_track(mid_idx,2), true_track(mid_idx,1), 'wo', ...
        'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '拐点(~120°)');

    title(ax, '拐弯目标: 基础UKF vs 机动自适应UKF 对比');
    subtitle(ax, sprintf('120°拐角 Pd=%.0f%% Pfa=%.3f 航速%.0fm/s', ...
        params.detection_probability*100, params.false_alarm_rate, 140));

    % ---- 右侧图层控制面板 ----
    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);

    for i = 1:n_layers
        ypos = 0.93 - (i-1) * 0.048;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.73, ypos, 0.25, 0.042], ...
            'FontSize', 8, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_turn(h_all(i), src.Value));
    end

    % 全部显示/隐藏按钮
    btn_y = 0.93 - n_layers * 0.048 - 0.01;
    if btn_y > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.73, btn_y, 0.12, 0.038], ...
            'FontSize', 8, ...
            'Callback', @(src, ~) toggle_all_turn(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.86, btn_y, 0.12, 0.038], ...
            'FontSize', 8, ...
            'Callback', @(~, ~) show_all_turn(cb, h_all));
    end

    % 统计信息
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.73, 0.005, 0.25, 0.04], ...
        'String', sprintf('基础UKF vs 机动自适应UKF | Pd=%.0f%% Pfa=%.3f', ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig_turn_comparison.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig_turn_comparison.png'));
    end
    fprintf('  拐弯航迹对比图已保存: fig_turn_comparison.png\n');
end

function [lats, lons] = extract_track_ll(snapshots)
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

function [lats, lons] = extract_fused_ll(snapshots)
    lats = []; lons = [];
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        trk = snap.trackList{1};
        if ~isfield(trk, 'lat') || isnan(trk.lat), continue; end
        lats(end+1) = trk.lat;
        lons(end+1) = trk.lon;
    end
end

function try_set_visible_turn(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_turn(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible_turn(h_all(i), new_val);
    end
end

function show_all_turn(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible_turn(h_all(i), 1);
    end
end
