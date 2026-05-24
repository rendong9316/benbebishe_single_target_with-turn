% =========================================================================
% plot_turn_comprehensive.m
% 全图层综合对比图 — 所有航迹在一张地图上，按钮控制显隐
% =========================================================================
% 图层:
%   1. 真实航迹 (亮黄虚线)
%   2. R1 原始量测 (淡蓝点虚线)
%   3. R2 原始量测 (淡红点虚线)
%   4. R1 校准后量测 (蓝色点线)
%   5. R2 校准后量测 (红色点线)
%   6. R1 基础UKF滤波 (蓝色虚线)
%   7. R2 基础UKF滤波 (红色虚线)
%   8. R1 自适应UKF滤波 (深蓝实线)
%   9. R2 自适应UKF滤波 (深红实线)
%  10. 基础UKF SCC融合 (青色虚线)
%  11. 自适应UKF SCC融合 (深绿实线)
% =========================================================================

function plot_turn_comprehensive(true_track, ...
        detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, ...
        fused_scc_base, fused_scc_ad, params, out_dir)

    % ---- 提取各图层数据 ----
    % 原始量测: 取 raw_lat/raw_lon 字段 (校准前)
    [raw1_la, raw1_lo] = extract_det_ll(detList_R1, 'raw');
    [raw2_la, raw2_lo] = extract_det_ll(detList_R2, 'raw');
    % 校准后量测: 取 lat/lon 字段 (校准后, 未滤波)
    [cal1_la, cal1_lo] = extract_det_ll(detList_R1, 'cal');
    [cal2_la, cal2_lo] = extract_det_ll(detList_R2, 'cal');
    [r1b_la, r1b_lo] = extract_track_ll(trackR1_base);
    [r2b_la, r2b_lo] = extract_track_ll(trackR2_base);
    [r1a_la, r1a_lo] = extract_track_ll(trackR1_ad);
    [r2a_la, r2a_lo] = extract_track_ll(trackR2_ad);
    [fb_la, fb_lo] = extract_fused_ll(fused_scc_base);
    [fa_la, fa_lo] = extract_fused_ll(fused_scc_ad);

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

    % ---- 图层1: 真实航迹 (亮黄虚线) ----
    h = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2.5, 'DisplayName', '真实航迹');
    h_all{end+1} = h; layer_names{end+1} = '真实航迹';

    % ---- 图层2: R1 原始量测 (淡蓝点虚线) ----
    if ~isempty(raw1_la)
        h = geoplot(ax, raw1_la, raw1_lo, '--.', ...
            'Color', [0.5 0.7 1.0], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'DisplayName', 'R1 原始量测(未校准)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 原始量测(校准前)';
    end

    % ---- 图层3: R2 原始量测 (淡红点虚线) ----
    if ~isempty(raw2_la)
        h = geoplot(ax, raw2_la, raw2_lo, '--.', ...
            'Color', [1.0 0.65 0.65], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'DisplayName', 'R2 原始量测(未校准)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 原始量测(校准前)';
    end

    % ---- 图层4: R1 校准后量测 (蓝色点线) ----
    if ~isempty(cal1_la)
        h = geoplot(ax, cal1_la, cal1_lo, '-o', ...
            'Color', [0.2 0.4 0.9], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.2 0.4 0.9], 'DisplayName', 'R1 校准量测(未滤波)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 校准后量测';
    end

    % ---- 图层5: R2 校准后量测 (红色点线) ----
    if ~isempty(cal2_la)
        h = geoplot(ax, cal2_la, cal2_lo, '-o', ...
            'Color', [0.9 0.3 0.3], 'LineWidth', 0.8, 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.9 0.3 0.3], 'DisplayName', 'R2 校准量测(未滤波)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 校准后量测';
    end

    % ---- 图层6: R1 基础UKF滤波 (蓝色虚线) ----
    if ~isempty(r1b_la)
        h = geoplot(ax, r1b_la, r1b_lo, '--', ...
            'Color', [0.2 0.4 0.9], 'LineWidth', 1.8, ...
            'DisplayName', 'R1 基础UKF(模糊Q)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 基础UKF(模糊Q)';
    end

    % ---- 图层7: R2 基础UKF滤波 (红色虚线) ----
    if ~isempty(r2b_la)
        h = geoplot(ax, r2b_la, r2b_lo, '--', ...
            'Color', [0.9 0.3 0.3], 'LineWidth', 1.8, ...
            'DisplayName', 'R2 基础UKF(模糊Q)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 基础UKF(模糊Q)';
    end

    % ---- 图层8: R1 自适应UKF滤波 (深蓝实线) ----
    if ~isempty(r1a_la)
        h = geoplot(ax, r1a_la, r1a_lo, '-', ...
            'Color', [0.0 0.1 0.6], 'LineWidth', 2.2, ...
            'DisplayName', 'R1 自适应UKF(机动检测)');
        h_all{end+1} = h; layer_names{end+1} = 'R1 自适应UKF(机动检测)';
    end

    % ---- 图层9: R2 自适应UKF滤波 (深红实线) ----
    if ~isempty(r2a_la)
        h = geoplot(ax, r2a_la, r2a_lo, '-', ...
            'Color', [0.7 0.0 0.0], 'LineWidth', 2.2, ...
            'DisplayName', 'R2 自适应UKF(机动检测)');
        h_all{end+1} = h; layer_names{end+1} = 'R2 自适应UKF(机动检测)';
    end

    % ---- 图层10: 基础UKF SCC融合 (青色虚线) ----
    if ~isempty(fb_la)
        h = geoplot(ax, fb_la, fb_lo, '--', ...
            'Color', [0.0 0.7 0.7], 'LineWidth', 2.5, ...
            'DisplayName', '基础UKF SCC融合');
        h_all{end+1} = h; layer_names{end+1} = '基础UKF SCC融合';
    end

    % ---- 图层11: 自适应UKF SCC融合 (深绿实线) ----
    if ~isempty(fa_la)
        h = geoplot(ax, fa_la, fa_lo, '-', ...
            'Color', [0.0 0.45 0.0], 'LineWidth', 3.0, ...
            'DisplayName', '自适应UKF SCC融合');
        h_all{end+1} = h; layer_names{end+1} = '自适应UKF SCC融合';
    end

    % ---- 站点标记 ----
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1站');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2站');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Tx2');

    % 起点/终点
    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', '终点');

    title(ax, '双基地雷达拐弯目标全流程对比');
    subtitle(ax, sprintf('原始量测 → 校准 → 基础UKF滤波 → 自适应UKF滤波 → SCC融合 | Pd=%.0f%% Pfa=%.3f 拐角~113°', ...
        params.detection_probability*100, params.false_alarm_rate));

    % ---- 右侧图层控制面板 ----
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
            'Callback', @(src, ~) try_set_vis(h_all{i}, src.Value));
    end

    % 全部隐藏/显示按钮
    btn_y = 0.93 - n_layers * row_h - 0.015;
    if btn_y > 0.01
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.77, btn_y, 0.10, 0.035], ...
            'FontSize', 7, ...
            'Callback', @(src, ~) toggle_all(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.88, btn_y, 0.10, 0.035], ...
            'FontSize', 7, ...
            'Callback', @(~, ~) show_all(cb, h_all));
    end

    % 底部统计条
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

% =========================================================================
% 辅助函数
% =========================================================================
function [lats, lons] = extract_det_ll(detList, mode)
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
            else  % 'cal'
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    lats(end+1) = dp.lat;
                    lons(end+1) = dp.lon;
                end
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

function [lats, lons] = extract_fused_ll(snaps)
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

function try_set_vis(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_vis(h_all{i}, new_val);
    end
end

function show_all(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_vis(h_all{i}, 1);
    end
end
