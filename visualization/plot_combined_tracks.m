% =========================================================================
% plot_combined_tracks.m
% 综合航迹图：geoplot 叠加真实航迹、原始点迹、校准后点迹、UKF滤波航迹
% 带复选框切换图层显隐 (darkwater底图)
% =========================================================================

function plot_combined_tracks(true_track, detList_R1, detList_R2, ...
        trackState_R1, trackState_R2, params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);

    % ---- 左侧地理图 ----
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    % 提取各层数据
    [assc1_lat, assc1_lon] = extract_associated_dets(trackState_R1);
    [assc2_lat, assc2_lon] = extract_associated_dets(trackState_R2);
    [raw1_lat, raw1_lon] = extract_raw_associated_dets(trackState_R1);
    [raw2_lat, raw2_lon] = extract_raw_associated_dets(trackState_R2);
    [filt1_lat, filt1_lon] = extract_filtered_track(trackState_R1);
    [filt2_lat, filt2_lon] = extract_filtered_track(trackState_R2);

    % 图层1: 真实航迹 (亮黄虚线, 适配暗底图)
    h1 = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2, 'DisplayName', '真实航迹');

    % 图层2: R1 原始（校准前）关联点迹连线 (淡蓝色虚线)
    h2 = geoplot(ax, raw1_lat, raw1_lon, '--', ...
        'Color', [0.4, 0.6, 1.0], 'LineWidth', 1.2, 'Marker', 'o', ...
        'MarkerSize', 5, 'MarkerFaceColor', [0.4, 0.6, 1.0], ...
        'DisplayName', 'R1 原始点迹');

    % 图层3: R2 原始（校准前）关联点迹连线 (淡红色虚线)
    h3 = geoplot(ax, raw2_lat, raw2_lon, '--', ...
        'Color', [1.0, 0.6, 0.6], 'LineWidth', 1.2, 'Marker', 'o', ...
        'MarkerSize', 5, 'MarkerFaceColor', [1.0, 0.6, 0.6], ...
        'DisplayName', 'R2 原始点迹');

    % 图层4: R1 校准后关联点迹连线 (蓝色实线+圆点)
    h4 = geoplot(ax, assc1_lat, assc1_lon, 'bo-', ...
        'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'b', ...
        'DisplayName', 'R1 校准后点迹');

    % 图层5: R2 校准后关联点迹连线 (红色实线+圆点)
    h5 = geoplot(ax, assc2_lat, assc2_lon, 'ro-', ...
        'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'r', ...
        'DisplayName', 'R2 校准后点迹');

    % 图层6: R1 UKF滤波航迹 (青色粗实线, 暗底图醒目)
    h6 = geoplot(ax, filt1_lat, filt1_lon, 'c-', ...
        'LineWidth', 2.5, 'DisplayName', 'R1 UKF滤波');

    % 图层7: R2 UKF滤波航迹 (品红粗实线)
    h7 = geoplot(ax, filt2_lat, filt2_lon, 'm-', ...
        'LineWidth', 2.5, 'DisplayName', 'R2 UKF滤波');

    % 站点标记
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 8, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 8, 'DisplayName', 'Tx2');

    % 起点/终点
    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '终点');

    title(ax, '双基地雷达航迹综合对比');

    % ---- 右侧图层控制面板 ----
    handles = {h1, h2, h3, h4, h5, h6, h7};
    labels = {'真实航迹', 'R1 原始点迹(校准前)', 'R2 原始点迹(校准前)', ...
              'R1 校准后点迹', 'R2 校准后点迹', 'R1 UKF滤波', 'R2 UKF滤波'};

    for i = 1:7
        ypos = 0.92 - (i-1) * 0.09;
        uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', labels{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.06], ...
            'FontSize', 10, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(handles{i}, src.Value));
    end

    % 统计信息
    n1 = length(assc1_lat); n2 = length(assc2_lat);
    n1c = sum_assc_clutter(trackState_R1);
    n2c = sum_assc_clutter(trackState_R2);
    nr1 = length(raw1_lat); nr2 = length(raw2_lat);
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.01, 0.22, 0.06], ...
        'String', sprintf('R1关联:%d(虚警%d) R2关联:%d(虚警%d)\n原始点迹 R1:%d R2:%d  Pd=%.0f%% Pfa=%.3f', ...
        n1, n1c, n2, n2c, nr1, nr2, params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig3_combined_tracks.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig3_combined_tracks.png'));
    end
    fprintf('  综合航迹图已保存: fig3_combined_tracks.png\n');
end

function try_set_visible(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

% ---- 从trackState提取校准后关联点迹位置 ----
function [lats, lons] = extract_associated_dets(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~s.associated, continue; end
        if isfield(s, 'det_lat') && ~isnan(s.det_lat)
            lats(end+1) = s.det_lat;
            lons(end+1) = s.det_lon;
        end
    end
end

% ---- 从trackState提取原始（校准前）关联点迹位置 ----
function [lats, lons] = extract_raw_associated_dets(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~s.associated, continue; end
        if isfield(s, 'det_raw_lat') && ~isnan(s.det_raw_lat)
            lats(end+1) = s.det_raw_lat;
            lons(end+1) = s.det_raw_lon;
        end
    end
end

function n = sum_assc_clutter(stateList)
    n = 0;
    for k = 1:length(stateList)
        s = stateList{k};
        if ~isempty(s) && s.associated && s.assc_is_clutter
            n = n + 1;
        end
    end
end

function [lats, lons] = extract_filtered_track(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~isfield(s, 'lat') || isnan(s.lat), continue; end
        lats(end+1) = s.lat;
        lons(end+1) = s.lon;
    end
end
