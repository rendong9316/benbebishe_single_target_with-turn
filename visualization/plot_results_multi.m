% =========================================================================
% plot_results_multi.m — 多目标结果可视化调度器
% =========================================================================
%
% 【功能概述】
%   将多目标场景的可视化结果聚合到一个dispatcher中。
%   通过 mode 字符串调度对应的子函数。
%
% 【调度模式】
%   'single_track'  -> plot_multi_track_result(...)
%   'single_fusion' -> plot_multi_fusion_result(...)
%
% 【与 plot_results.m 的区别】
%   - 支持三个真值航迹 (A/B/C)
%   - 地图上用不同颜色区分不同目标的航迹
%   - 融合结果按匹配对分别绘制
%
% =========================================================================

function plot_results_multi(mode, varargin)
    switch mode
        case 'single_track'
            plot_multi_track_result(varargin{:});
        case 'single_fusion'
            plot_multi_fusion_result(varargin{:});
        otherwise
            error('plot_results_multi: unknown mode "%s". Valid: single_track, single_fusion', mode);
    end
end

% =========================================================================
% 1. plot_multi_track_result — 多目标双基地雷达航迹综合对比图
% =========================================================================
function plot_multi_track_result(true_track_A, true_track_B, true_track_C, ...
        detList_R1, detList_R2, trackSnapshots_R1, trackSnapshots_R2, params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    h_all = [];
    layer_names = {};

    % 真值航迹 (A=黄色, B=品红, C=青色)
    h = geoplot(ax, true_track_A(:,2), true_track_A(:,1), '--s', ...
        'Color', [1 1 0], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 1 0], ...
        'DisplayName', '真值A');
    h_all(end+1) = h;  layer_names{end+1} = '真值A';

    h = geoplot(ax, true_track_B(:,2), true_track_B(:,1), '--s', ...
        'Color', [1 0 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 0 1], ...
        'DisplayName', '真值B');
    h_all(end+1) = h;  layer_names{end+1} = '真值B';

    h = geoplot(ax, true_track_C(:,2), true_track_C(:,1), '--s', ...
        'Color', [0 1 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [0 1 1], ...
        'DisplayName', '真值C');
    h_all(end+1) = h;  layer_names{end+1} = '真值C';

    % R1 校准点迹 (蓝色系)
    [r1_cal_lat, r1_cal_lon] = extract_dets_multi(detList_R1, 'cal');
    if ~isempty(r1_cal_lat)
        h = geoplot(ax, r1_cal_lat, r1_cal_lon, '.b', ...
            'MarkerSize', 3, 'DisplayName', 'R1校准点迹');
        h_all(end+1) = h;  layer_names{end+1} = 'R1校准点迹';
    end

    % R1 航迹
    r1_tracks = collect_active_tracks_multi(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R1 UKF#%d', trk.id);
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % R2 校准点迹 (红色系)
    [r2_cal_lat, r2_cal_lon] = extract_dets_multi(detList_R2, 'cal');
    if ~isempty(r2_cal_lat)
        h = geoplot(ax, r2_cal_lat, r2_cal_lon, '.r', ...
            'MarkerSize', 3, 'DisplayName', 'R2校准点迹');
        h_all(end+1) = h;  layer_names{end+1} = 'R2校准点迹';
    end

    % R2 航迹
    r2_tracks = collect_active_tracks_multi(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % 雷达站标记
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, '多目标双基地雷达航迹综合对比 (3目标交叉)');
    legend(ax, 'Location', 'northeastoutside');

    % 图层控制复选框
    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);
    for i = 1:n_layers
        ypos = 0.92 - (i-1) * 0.040;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 8, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_multi(h_all(i), src.Value));
    end

    drawnow;

    % 保存
    out_path = fullfile(out_dir, 'fig3_multi_track.png');
    saveas(fig, out_path, 'png');
    fprintf('多目标跟踪图已保存: %s\n', out_path);
end

% =========================================================================
% 2. plot_multi_fusion_result — 多目标融合结果可视化
% =========================================================================
function plot_multi_fusion_result(true_track_A, true_track_B, true_track_C, ...
        trackSnapshots_R1, trackSnapshots_R2, all_fused_snapshots, ...
        method_names, matched_pairs, fusion_eval, truthTrajs, params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    h_all = [];
    layer_names = {};

    % 真值
    h = geoplot(ax, true_track_A(:,2), true_track_A(:,1), '--s', ...
        'Color', [1 1 0], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 1 0]);
    h_all(end+1) = h;  layer_names{end+1} = '真值A';
    h = geoplot(ax, true_track_B(:,2), true_track_B(:,1), '--s', ...
        'Color', [1 0 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 0 1]);
    h_all(end+1) = h;  layer_names{end+1} = '真值B';
    h = geoplot(ax, true_track_C(:,2), true_track_C(:,1), '--s', ...
        'Color', [0 1 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [0 1 1]);
    h_all(end+1) = h;  layer_names{end+1} = '真值C';

    % R1/R2 航迹
    r1_tracks = collect_active_tracks_multi(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R1 UKF#%d', trk.id);
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 1.0, 'MarkerSize', 3, 'MarkerFaceColor', 'b', ...
                'HandleVisibility', 'off');
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    r2_tracks = collect_active_tracks_multi(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 1.0, 'MarkerSize', 3, 'MarkerFaceColor', 'r', ...
                'HandleVisibility', 'off');
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % 融合航迹 (每种算法不同颜色)
    method_colors = {[0 0.5 0], [0.8 0.4 0], [0 0 0.8], [0.6 0 0.6]};
    for p = 1:length(matched_pairs)
        mp = matched_pairs(p);
        for m = 1:length(method_names)
            snaps_m = all_fused_snapshots{p,m};
            fused_pos = collect_fused_positions_multi(snaps_m);
            for t = 1:length(fused_pos)
                ft = fused_pos{t};
                if length(ft.lat_history) > 1
                    seg_label = sprintf('%s Pair#%d', method_names{m}, p);
                    h = geoplot(ax, ft.lat_history, ft.lon_history, '-d', ...
                        'Color', method_colors{m}, 'LineWidth', 2, ...
                        'MarkerSize', 4, 'MarkerFaceColor', method_colors{m}, ...
                        'DisplayName', seg_label);
                    h_all(end+1) = h;  layer_names{end+1} = seg_label;
                end
            end
        end
    end

    % 雷达站
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', 'MarkerSize', 10);
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', 'MarkerSize', 10);

    title(ax, '多目标融合结果 (3目标交叉)');

    % 图层控制
    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);
    for i = 1:n_layers
        ypos = 0.92 - (i-1) * 0.035;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.035], ...
            'FontSize', 7, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_multi(h_all(i), src.Value));
    end

    drawnow;

    out_path = fullfile(out_dir, 'fig4_multi_fusion.png');
    saveas(fig, out_path, 'png');
    fprintf('多目标融合图已保存: %s\n', out_path);
end

% =========================================================================
% 辅助函数
% =========================================================================

function [lats, lons] = extract_dets_multi(detList, mode)
    lats = []; lons = [];
    for k = 1:length(detList)
        dets = detList{k};
        if isempty(dets), continue; end
        for d = 1:length(dets)
            dp = dets(d);
            if dp.is_clutter, continue; end
            if strcmp(mode, 'cal') && isfield(dp, 'lat') && ~isnan(dp.lat)
                lats(end+1) = dp.lat;
                lons(end+1) = dp.lon;
            end
        end
    end
end

function tracks = collect_active_tracks_multi(snaps)
    tracks = {};
    seg_lats = []; seg_lons = [];
    in_seg = false; tid = 0;
    for k = 1:length(snaps)
        snap = snaps{k};
        valid = false; lat_val = NaN; lon_val = NaN;
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                valid = true;
                lat_val = trk.lat;
                lon_val = trk.lon;
                tid = trk.id;
            end
        end
        if valid
            if ~in_seg
                in_seg = true;
                seg_lats = lat_val;
                seg_lons = lon_val;
            else
                seg_lats(end+1) = lat_val;
                seg_lons(end+1) = lon_val;
            end
        else
            if in_seg
                in_seg = false;
                tracks{end+1} = struct('id', tid, 'lat_history', seg_lats, 'lon_history', seg_lons);
                seg_lats = []; seg_lons = [];
            end
        end
    end
    if in_seg
        tracks{end+1} = struct('id', tid, 'lat_history', seg_lats, 'lon_history', seg_lons);
    end
end

function tracks = collect_fused_positions_multi(snaps)
    tracks = {};
    seg_lats = []; seg_lons = [];
    in_seg = false; seg_id = 0;
    for k = 1:length(snaps)
        snap = snaps{k};
        valid = false; lat_val = NaN; lon_val = NaN;
        if ~isempty(snap.trackList)
            ft = snap.trackList{1};
            if ~isnan(ft.lat)
                valid = true;
                lat_val = ft.lat;
                lon_val = ft.lon;
                seg_id = ft.id;
            end
        end
        if valid
            if ~in_seg
                in_seg = true;
                seg_lats = lat_val;
                seg_lons = lon_val;
            else
                seg_lats(end+1) = lat_val;
                seg_lons(end+1) = lon_val;
            end
        else
            if in_seg
                in_seg = false;
                tracks{end+1} = struct('id', seg_id, 'lat_history', seg_lats, 'lon_history', seg_lons);
                seg_lats = []; seg_lons = [];
            end
        end
    end
    if in_seg
        tracks{end+1} = struct('id', seg_id, 'lat_history', seg_lats, 'lon_history', seg_lons);
    end
end

function try_set_visible_multi(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end
