% =========================================================================
% plot_results.m
% 聚合式绘图调度文件 — 单目标双基地雷达结果可视化
% =========================================================================
%
% 【功能概述】
%   将原来分散在 7 个独立 .m 文件中的绘图函数合并到一个文件中，
%   通过 mode 字符串调度对应的子函数。所有图形/绘图逻辑保持不变。
%
% 【调度模式】
%   'single_track'     -> plot_single_track_result(...)
%   'single_fusion'    -> plot_single_fusion_result(...)
%   'combined_tracks'  -> plot_combined_tracks(...)
%   'tracks_vs_truth'  -> plot_tracks_vs_truth(...)
%   'tracker'          -> plot_tracker_result(...)
%   'error_timeline'   -> plot_error_timeline(...)
%   'error_timeline_turn' -> plot_error_timeline_turn(...)
%
% 【内部辅助函数命名约定】
%   为避免多个子函数之间的同名冲突，所有辅助函数均添加父函数缩写后缀。
%   例如：extract_dets → extract_dets_str (str = single_track_result)
%         collect_positions → collect_positions_sfr (sfr = single_fusion_result)
%         try_set_visible → try_set_visible_ct (ct = combined_tracks)
%         ...
%
% =========================================================================

function plot_results(mode, varargin)
    switch mode
        case 'single_track'
            plot_single_track_result(varargin{:});
        case 'single_fusion'
            plot_single_fusion_result(varargin{:});
        case 'combined_tracks'
            plot_combined_tracks(varargin{:});
        case 'tracks_vs_truth'
            plot_tracks_vs_truth(varargin{:});
        case 'tracker'
            plot_tracker_result(varargin{:});
        case 'error_timeline'
            plot_error_timeline(varargin{:});
        case 'error_timeline_turn'
            plot_error_timeline_turn(varargin{:});
        otherwise
            error('plot_results: unknown mode "%s". Valid: single_track, single_fusion, combined_tracks, tracks_vs_truth, tracker, error_timeline, error_timeline_turn', mode);
    end
end

% =========================================================================
% 1. plot_single_track_result — 单目标双基地雷达航迹综合对比图
% 来源: plot_single_track_result.m
% =========================================================================
function plot_single_track_result(true_track, detList_R1, detList_R2, ...
        trackSnapshots_R1, trackSnapshots_R2, params, out_dir)

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

    h_truth = geoplot(ax, true_track(:,2), true_track(:,1), '--s', ...
        'Color', 'g', 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'g', ...
        'DisplayName', '真值');
    h_all(end+1) = h_truth;
    layer_names{end+1} = '真值航迹';

    [r1_raw_lat, r1_raw_lon] = extract_dets_str(detList_R1, 'raw');
    if ~isempty(r1_raw_lat)
        h = geoplot(ax, r1_raw_lat, r1_raw_lon, '--o', ...
            'Color', [0.4 0.6 1.0], 'LineWidth', 1.0, 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.4 0.6 1.0], 'DisplayName', 'R1 原始点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 原始点迹(校准前)';
    end

    [r1_cal_lat, r1_cal_lon] = extract_dets_str(detList_R1, 'cal');
    if ~isempty(r1_cal_lat)
        h = geoplot(ax, r1_cal_lat, r1_cal_lon, '-o', ...
            'Color', [0.0 0.4 1.0], 'LineWidth', 1.2, 'MarkerSize', 5, ...
            'MarkerFaceColor', 'b', 'DisplayName', 'R1 校准点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 校准后点迹';
    end

    r1_tracks = collect_active_tracks_str(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R1 UKF#%d', trk.id);
            if length(r1_tracks) > 1, seg_label = sprintf('R1 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 2.0, 'MarkerSize', 5, 'MarkerFaceColor', 'b', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;
            layer_names{end+1} = seg_label;
        end
    end

    [r2_raw_lat, r2_raw_lon] = extract_dets_str(detList_R2, 'raw');
    if ~isempty(r2_raw_lat)
        h = geoplot(ax, r2_raw_lat, r2_raw_lon, '--o', ...
            'Color', [1.0 0.6 0.6], 'LineWidth', 1.0, 'MarkerSize', 4, ...
            'MarkerFaceColor', [1.0 0.6 0.6], 'DisplayName', 'R2 原始点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 原始点迹(校准前)';
    end

    [r2_cal_lat, r2_cal_lon] = extract_dets_str(detList_R2, 'cal');
    if ~isempty(r2_cal_lat)
        h = geoplot(ax, r2_cal_lat, r2_cal_lon, '-o', ...
            'Color', [1.0 0.2 0.2], 'LineWidth', 1.2, 'MarkerSize', 5, ...
            'MarkerFaceColor', 'r', 'DisplayName', 'R2 校准点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 校准后点迹';
    end

    r2_tracks = collect_active_tracks_str(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            if length(r2_tracks) > 1, seg_label = sprintf('R2 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 2.0, 'MarkerSize', 5, 'MarkerFaceColor', 'r', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;
            layer_names{end+1} = seg_label;
        end
    end

    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, '单目标双基地雷达航迹综合对比');
    legend(ax, 'Location', 'northeastoutside');

    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);

    for i = 1:n_layers
        ypos = 0.92 - (i-1) * 0.045;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_str(h_all(i), src.Value));
    end

    btn_bottom = 0.92 - n_layers * 0.045 - 0.01;
    if btn_bottom > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.76, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb_str(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.87, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb_str(cb, h_all));
    end

    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.005, 0.22, 0.03], ...
        'String', sprintf('R1:%d航迹 R2:%d航迹 | Pd=%.0f%% Pfa=%.3f', ...
        length(r1_tracks), length(r2_tracks), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
end

function [lats, lons] = extract_dets_str(detList, mode)
    lats = []; lons = [];
    for k = 1:length(detList)
        dets = detList{k};
        for d = 1:length(dets)
            dp = dets(d);
            if dp.is_clutter, continue; end
            if strcmp(mode, 'raw') && isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                lats(end+1) = dp.raw_lat;
                lons(end+1) = dp.raw_lon;
            elseif strcmp(mode, 'cal') && isfield(dp, 'lat') && ~isnan(dp.lat)
                lats(end+1) = dp.lat;
                lons(end+1) = dp.lon;
            end
        end
    end
end

function tracks = collect_active_tracks_str(snapshots)
    % 按连续帧自动分段，每段独立（支持断裂重起后的多段航迹分别绘制）
    tracks = {};
    seg_lats = []; seg_lons = [];
    in_seg = false;
    for k = 1:length(snapshots)
        snap = snapshots{k};
        valid = false; lat_val = NaN; lon_val = NaN; tid = 0;
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

function try_set_visible_str(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_cb_str(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible_str(h_all(i), new_val);
    end
end

function show_all_cb_str(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible_str(h_all(i), 1);
    end
end

% =========================================================================
% 2. plot_single_fusion_result — 单目标双基地雷达航迹融合结果可视化
% 来源: plot_single_fusion_result.m
% =========================================================================
function plot_single_fusion_result(true_track, trackSnapshots_R1, trackSnapshots_R2, ...
        all_fused_snapshots, method_names, best_idx, fusion_eval, truthTraj, params, out_dir)

    n_methods = length(method_names);
    frame_times = (0:length(trackSnapshots_R1)-1) * params.dt_sec;

    %% Figure 1: 地图叠加
    fig1 = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.68, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.68, 0.88]);
    end
    hold(ax, 'on');

    h_all = []; layer_names = {};

    h = geoplot(ax, true_track(:,2), true_track(:,1), '--s', ...
        'Color', 'g', 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'g', ...
        'DisplayName', '真值');
    h_all(end+1) = h; layer_names{end+1} = '真值航迹';

    r1_tracks = collect_positions_sfr(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R1 UKF#%d', trk.id);
            if length(r1_tracks) > 1, seg_label = sprintf('R1 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', seg_label);
            h_all(end+1) = h; layer_names{end+1} = seg_label;
        end
    end

    r2_tracks = collect_positions_sfr(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            if length(r2_tracks) > 1, seg_label = sprintf('R2 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', seg_label);
            h_all(end+1) = h; layer_names{end+1} = seg_label;
        end
    end

    method_colors = {[0 0.5 0], [0.8 0.4 0], [0 0 0.8], [0.6 0 0.6]};
    for m = 1:n_methods
        snaps_m = all_fused_snapshots{m};
        fused_pos = collect_fused_positions_sfr(snaps_m);
        for t = 1:length(fused_pos)
            ft = fused_pos{t};
            if length(ft.lat_history) > 1
                lw = 3.0;
                if m == best_idx, lw = 3.5; end
                seg_label = sprintf('%s 融合', method_names{m});
                if length(fused_pos) > 1, seg_label = sprintf('%s 融合-段%d', method_names{m}, t); end
                h = geoplot(ax, ft.lat_history, ft.lon_history, '-d', ...
                    'Color', method_colors{m}, 'LineWidth', lw, ...
                    'MarkerSize', 5, 'MarkerFaceColor', method_colors{m}, ...
                    'DisplayName', seg_label);
                h_all(end+1) = h;
                layer_names{end+1} = seg_label;
            end
        end
    end

    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, sprintf('单目标双基地雷达航迹融合结果 (%s最优)', method_names{best_idx}));

    n_layers1 = length(layer_names);
    cb1 = gobjects(1, n_layers1);
    for i = 1:n_layers1
        ypos = 0.92 - (i-1) * 0.040;
        if ypos < 0.05, break; end
        cb1(i) = uicontrol('Parent', fig1, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.74, ypos, 0.24, 0.036], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_sfr(h_all(i), src.Value));
    end

    btn_bottom1 = 0.92 - n_layers1 * 0.040 - 0.01;
    if btn_bottom1 > 0.02
        uicontrol('Parent', fig1, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.74, btn_bottom1, 0.11, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb_sfr(src, cb1, h_all));
        uicontrol('Parent', fig1, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.86, btn_bottom1, 0.11, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb_sfr(cb1, h_all));
    end

    best_rmse = fusion_eval.overall(best_idx).s.rms;
    uicontrol('Parent', fig1, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.74, 0.005, 0.24, 0.04], ...
        'String', sprintf('最佳: %s RMSE=%.1fkm', method_names{best_idx}, best_rmse), ...
        'FontSize', 9, 'BackgroundColor', [1 1 1], 'FontWeight', 'bold');

    drawnow;
    %% Figure 2: 误差收敛曲线 + CDF
    fig2 = figure('Position', [50, 50, 1400, 750]);
    win = 10;

    method_ls = {'-', '--', '-.', ':'};
    method_clr = {[0 0 0], [1 0 0], [0 0 1], [0 0.7 0]};

    subplot(1, 2, 1);
    hold on; grid on;
    all_h_lines = []; all_line_names = {};

    for m = 1:n_methods
        fe = build_frame_errors_sfr(all_fused_snapshots{m}, truthTraj, frame_times);
        if length(fe) >= win
            smoothed = movmean(fe, win, 'omitnan');
            h = plot(frame_times, smoothed, 'LineStyle', method_ls{m}, ...
                'Color', method_clr{m}, 'LineWidth', 2);
            all_h_lines(end+1) = h;
            all_line_names{end+1} = method_names{m};
        end
    end

    r1_fe = build_single_frame_errors_sfr(trackSnapshots_R1, truthTraj, frame_times);
    if length(r1_fe) >= win
        h = plot(frame_times, movmean(r1_fe, win, 'omitnan'), ...
            ':', 'Color', [0 0 0.7], 'LineWidth', 1.5);
        all_h_lines(end+1) = h;
        all_line_names{end+1} = 'R1 UKF';
    end

    r2_fe = build_single_frame_errors_sfr(trackSnapshots_R2, truthTraj, frame_times);
    if length(r2_fe) >= win
        h = plot(frame_times, movmean(r2_fe, win, 'omitnan'), ...
            ':', 'Color', [0.7 0 0], 'LineWidth', 1.5);
        all_h_lines(end+1) = h;
        all_line_names{end+1} = 'R2 UKF';
    end

    xlabel('时间 (s)'); ylabel('位置误差 (km)');
    title(sprintf('误差收敛曲线 (滑动平均 %d帧)', win));
    legend(all_line_names, 'FontSize', 8, 'Location', 'best');

    subplot(1, 2, 2);
    hold on; grid on;
    for m = 1:n_methods
        errs = fusion_eval.fusion_errors{m, 1};
        if ~isempty(errs)
            [f, x] = ecdf(errs);
            plot(x, f*100, 'LineStyle', method_ls{m}, ...
                'Color', method_clr{m}, 'LineWidth', 2);
        end
    end
    if ~isempty(fusion_eval.r1_errors{1})
        [f, x] = ecdf(fusion_eval.r1_errors{1});
        plot(x, f*100, ':', 'Color', [0 0 0.7], 'LineWidth', 1.5);
    end
    if ~isempty(fusion_eval.r2_errors{1})
        [f, x] = ecdf(fusion_eval.r2_errors{1});
        plot(x, f*100, ':', 'Color', [0.7 0 0], 'LineWidth', 1.5);
    end
    xlabel('位置误差 (km)'); ylabel('累积概率 (%)');
    title('误差CDF对比');
    legend([method_names, {'R1 UKF', 'R2 UKF'}], 'FontSize', 8, 'Location', 'southeast');

    n2 = length(all_line_names);
    cb2 = gobjects(1, n2);
    for i = 1:n2
        ypos = 0.92 - (i-1) * 0.05;
        if ypos < 0.05, break; end
        cb2(i) = uicontrol('Parent', fig2, 'Style', 'checkbox', ...
            'String', all_line_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.045], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_sfr(all_h_lines(i), src.Value));
    end

    drawnow;
end

function tracks = collect_positions_sfr(snapshots)
    % 按连续帧自动分段（与 collect_active_tracks_str 一致）
    tracks = {};
    seg_lats = []; seg_lons = [];
    in_seg = false;
    for k = 1:length(snapshots)
        snap = snapshots{k};
        valid = false; lat_val = NaN; lon_val = NaN; tid = 0;
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

function tracks = collect_fused_positions_sfr(snapshots)
    % 按连续帧自动分段
    tracks = {};
    seg_lats = []; seg_lons = [];
    in_seg = false; seg_id = 0;
    for k = 1:length(snapshots)
        snap = snapshots{k};
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

function fe = build_frame_errors_sfr(fused_snaps, truth, frame_times)
    n_frames = length(fused_snaps);
    fe = nan(1, n_frames);
    for k = 1:n_frames
        t_lat = interp1(truth.time_sec, truth.lat, frame_times(k), 'linear', 'extrap');
        t_lon = interp1(truth.time_sec, truth.lon, frame_times(k), 'linear', 'extrap');
        if isnan(t_lat), continue; end
        snap = fused_snaps{k};
        if isempty(snap.trackList), continue; end
        best_d = inf;
        for t = 1:length(snap.trackList)
            ft = snap.trackList{t};
            d = haversine_km_sfr(ft.lon, ft.lat, t_lon, t_lat);
            if d < best_d, best_d = d; end
        end
        if best_d < inf, fe(k) = best_d; end
    end
end

function fe = build_single_frame_errors_sfr(snapshots, truth, frame_times)
    n_frames = length(snapshots);
    fe = nan(1, n_frames);
    for k = 1:n_frames
        t_lat = interp1(truth.time_sec, truth.lat, frame_times(k), 'linear', 'extrap');
        t_lon = interp1(truth.time_sec, truth.lon, frame_times(k), 'linear', 'extrap');
        if isnan(t_lat), continue; end
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        best_d = inf;
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end
            d = haversine_km_sfr(trk.lon, trk.lat, t_lon, t_lat);
            if d < best_d, best_d = d; end
        end
        if best_d < inf, fe(k) = best_d; end
    end
end

function d = haversine_km_sfr(lon1, lat1, lon2, lat2)
    R = 6371;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2)^2;
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end

function try_set_visible_sfr(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_cb_sfr(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible_sfr(h_all(i), new_val);
    end
end

function show_all_cb_sfr(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible_sfr(h_all(i), 1);
    end
end

% =========================================================================
% 3. plot_combined_tracks — 综合航迹对比图
% 来源: plot_combined_tracks.m
% =========================================================================
function plot_combined_tracks(true_track, detList_R1, detList_R2, ...
        trackState_R1, trackState_R2, params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);

    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    [assc1_lat, assc1_lon] = extract_associated_dets_ct(trackState_R1);
    [assc2_lat, assc2_lon] = extract_associated_dets_ct(trackState_R2);
    [raw1_lat, raw1_lon] = extract_raw_associated_dets_ct(trackState_R1);
    [raw2_lat, raw2_lon] = extract_raw_associated_dets_ct(trackState_R2);
    [filt1_lat, filt1_lon] = extract_filtered_track_ct(trackState_R1);
    [filt2_lat, filt2_lon] = extract_filtered_track_ct(trackState_R2);

    h1 = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2, 'DisplayName', '真实航迹');

    h2 = geoplot(ax, raw1_lat, raw1_lon, '--', ...
        'Color', [0.4, 0.6, 1.0], 'LineWidth', 1.2, 'Marker', 'o', ...
        'MarkerSize', 5, 'MarkerFaceColor', [0.4, 0.6, 1.0], ...
        'DisplayName', 'R1 原始点迹');

    h3 = geoplot(ax, raw2_lat, raw2_lon, '--', ...
        'Color', [1.0, 0.6, 0.6], 'LineWidth', 1.2, 'Marker', 'o', ...
        'MarkerSize', 5, 'MarkerFaceColor', [1.0, 0.6, 0.6], ...
        'DisplayName', 'R2 原始点迹');

    h4 = geoplot(ax, assc1_lat, assc1_lon, 'bo-', ...
        'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'b', ...
        'DisplayName', 'R1 校准后点迹');

    h5 = geoplot(ax, assc2_lat, assc2_lon, 'ro-', ...
        'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'r', ...
        'DisplayName', 'R2 校准后点迹');

    h6 = geoplot(ax, filt1_lat, filt1_lon, 'c-', ...
        'LineWidth', 2.5, 'DisplayName', 'R1 UKF滤波');

    h7 = geoplot(ax, filt2_lat, filt2_lon, 'm-', ...
        'LineWidth', 2.5, 'DisplayName', 'R2 UKF滤波');

    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 8, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 8, 'DisplayName', 'Tx2');

    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '终点');

    title(ax, '双基地雷达航迹综合对比');

    handles = {h1, h2, h3, h4, h5, h6, h7};
    labels = {'真实航迹', 'R1 原始点迹(校准前)', 'R2 原始点迹(校准前)', ...
              'R1 校准后点迹', 'R2 校准后点迹', 'R1 UKF滤波', 'R2 UKF滤波'};

    for i = 1:7
        ypos = 0.92 - (i-1) * 0.09;
        uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', labels{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.06], ...
            'FontSize', 10, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_ct(handles{i}, src.Value));
    end

    n1 = length(assc1_lat); n2 = length(assc2_lat);
    n1c = sum_assc_clutter_ct(trackState_R1);
    n2c = sum_assc_clutter_ct(trackState_R2);
    nr1 = length(raw1_lat); nr2 = length(raw2_lat);
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.01, 0.22, 0.06], ...
        'String', sprintf('R1关联:%d(虚警%d) R2关联:%d(虚警%d)\n原始点迹 R1:%d R2:%d  Pd=%.0f%% Pfa=%.3f', ...
        n1, n1c, n2, n2c, nr1, nr2, params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
end

function try_set_visible_ct(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function [lats, lons] = extract_associated_dets_ct(stateList)
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

function [lats, lons] = extract_raw_associated_dets_ct(stateList)
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

function n = sum_assc_clutter_ct(stateList)
    n = 0;
    for k = 1:length(stateList)
        s = stateList{k};
        if ~isempty(s) && s.associated && s.assc_is_clutter
            n = n + 1;
        end
    end
end

function [lats, lons] = extract_filtered_track_ct(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~isfield(s, 'lat') || isnan(s.lat), continue; end
        lats(end+1) = s.lat;
        lons(end+1) = s.lon;
    end
end

% =========================================================================
% 4. plot_tracks_vs_truth — UKF 滤波航迹与真实航迹的并排对比图
% 来源: plot_tracks_vs_truth.m
% =========================================================================
function plot_tracks_vs_truth(trackState_R1, trackState_R2, true_track, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tlo);
    try
        gx1 = geoaxes(ax1);
        gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes(ax1);
    end
    hold(gx1, 'on');
    title(gx1, 'R1 UKF滤波航迹');

    plot_track_on_map_tvt(gx1, trackState_R1, true_track, params.radar1_lat, params.radar1_lon);

    ax2 = nexttile(tlo);
    try
        gx2 = geoaxes(ax2);
        gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes(ax2);
    end
    hold(gx2, 'on');
    title(gx2, 'R2 UKF滤波航迹');

    plot_track_on_map_tvt(gx2, trackState_R2, true_track, params.radar2_lat, params.radar2_lon);

    sgtitle('UKF滤波航迹 vs 真实航迹');
    drawnow;
end

function plot_track_on_map_tvt(ax, stateList, true_track, rx_lat, rx_lon)
    geoplot(ax, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.5, ...
        'DisplayName', '真实航迹');

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

    geoplot(ax, rx_lat, rx_lon, 'rs', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'DisplayName', '接收站');

    legend(ax, 'Location', 'best');
end

% =========================================================================
% 5. plot_tracker_result — 航迹碎片化与拼接的交互式可视化图
% 来源: plot_tracker_result.m
% =========================================================================
function fig = plot_tracker_result(true_track, ...
    r1_segments, r2_segments, ...
    r1_segments_filt, r2_segments_filt, ...
    r1_segments_aligned, r2_segments_aligned, ...
    r1_stitched, r2_stitched, ...
    radar1, radar2, params, unified_time, out_dir)

    n_r1_seg = length(r1_segments);
    n_r2_seg = length(r2_segments);
    n1_stitch = sum(~cellfun(@isempty, r1_stitched));
    n2_stitch = sum(~cellfun(@isempty, r2_stitched));

    fig = figure('Position', [50, 50, 1400, 750], ...
                 'Name', '雷达航迹碎片化与拼接 — 交互式', ...
                 'NumberTitle', 'off');
    ax = geoaxes('Basemap', 'darkwater');
    ax.Position = [0.05, 0.13, 0.92, 0.83];
    hold(ax, 'on');

    C_TRUE     = [0.10, 0.10, 0.10];
    C_R1_RAW   = [0.45, 0.65, 0.95];
    C_R2_RAW   = [0.95, 0.50, 0.50];
    C_R1_FILT  = [0.00, 0.25, 0.65];
    C_R2_FILT  = [0.70, 0.08, 0.08];
    C_R1_ALIGN = [0.00, 0.15, 0.50];
    C_R2_ALIGN = [0.50, 0.05, 0.05];
    C_R1_STCH  = [0.00, 0.10, 0.40];
    C_R2_STCH  = [0.40, 0.02, 0.02];

    LW = 1.0;
    MS = 4;

    L = {};

    L{1} = draw_struct_array_segments_tr(ax, r1_segments, C_R1_RAW, LW, MS, '-o');
    L{2} = draw_struct_array_segments_tr(ax, r2_segments, C_R2_RAW, LW, MS, '-s');
    L{3} = draw_cell_segments_tr(ax, r1_segments_filt, C_R1_FILT, LW, MS, '-o');
    L{4} = draw_cell_segments_tr(ax, r2_segments_filt, C_R2_FILT, LW, MS, '-s');
    L{5} = draw_cell_segments_tr(ax, r1_segments_aligned, C_R1_ALIGN, LW, MS, '-o');
    L{6} = draw_cell_segments_tr(ax, r2_segments_aligned, C_R2_ALIGN, LW, MS, '-s');
    L{7} = draw_stitched_track_tr(ax, r1_stitched, C_R1_STCH, LW, MS, '-o');
    L{8} = draw_stitched_track_tr(ax, r2_stitched, C_R2_STCH, LW, MS, '-s');

    h = geoplot(ax, true_track(:,2), true_track(:,1), '--o', ...
                'Color', C_TRUE, 'LineWidth', LW, 'MarkerSize', MS);
    L{9} = h;

    geoscatter(ax, radar1.lat, radar1.lon, 220, '*', ...
               'MarkerEdgeColor', '#B71C1C', 'MarkerFaceColor', '#B71C1C', ...
               'LineWidth', 0.5);
    text(ax, radar1.lon + 0.2, radar1.lat - 0.15, 'R1', ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', '#B71C1C', ...
         'BackgroundColor', [1 1 1 0.6]);
    geoscatter(ax, radar2.lat, radar2.lon, 220, '*', ...
               'MarkerEdgeColor', '#1B5E20', 'MarkerFaceColor', '#1B5E20', ...
               'LineWidth', 0.5);
    text(ax, radar2.lon + 0.2, radar2.lat - 0.15, 'R2', ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', '#1B5E20', ...
         'BackgroundColor', [1 1 1 0.6]);

    all_lats = [true_track(:,2); radar1.lat; radar2.lat];
    all_lons = [true_track(:,1); radar1.lon; radar2.lon];
    lat_pad = max(diff([min(all_lats), max(all_lats)]) * 0.15, 0.5);
    lon_pad = max(diff([min(all_lons), max(all_lons)]) * 0.15, 0.5);
    geolimits(ax, [min(all_lats)-lat_pad, max(all_lats)+lat_pad], ...
                   [min(all_lons)-lon_pad, max(all_lons)+lon_pad]);

    title(ax, sprintf(['短波外辐射源双雷达仿真 — 航迹碎片化与拼接\n' ...
           'M/N=%d/%d  K_{loss}=%d  Pd=%d%%  |  ' ...
           '碎片: R1×%d段 R2×%d段  →  拼接后: R1=%d点 R2=%d点'], ...
           params.tracker_M, params.tracker_N, params.tracker_K_loss, ...
           round(params.detection_probability*100), ...
           n_r1_seg, n_r2_seg, n1_stitch, n2_stitch), ...
           'FontSize', 13, 'FontWeight', 'bold');

    panel = uipanel(fig, 'Position', [0.01, 0.005, 0.98, 0.11], ...
                    'Title', '图层控制（单击按钮切换显示/隐藏）', ...
                    'FontSize', 9, 'FontWeight', 'bold');

    btn_w = 85; btn_h = 24; gap_x = 8;
    row1_y = 36; row2_y = 7;

    btn_labels = {'R1量测','R2量测','R1滤波','R2滤波', ...
                  'R1对齐','R2对齐','R1拼接','R2拼接','真实航迹'};
    all_buttons = gobjects(1, 9);

    x0 = 12;
    for k = 1:9
        all_buttons(k) = uicontrol(panel, 'Style', 'togglebutton', ...
            'String', btn_labels{k}, 'Position', [x0, row1_y, btn_w, btn_h], ...
            'Value', 1, 'FontSize', 8, ...
            'Callback', @(src,~) toggle_layer_tr(src, L{k}));
        x0 = x0 + btn_w + gap_x;
    end

    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '全部显示', 'Position', [12, row2_y, 80, 22], ...
        'FontSize', 8, 'FontWeight', 'bold', ...
        'Callback', @(~,~) set_all_layers_tr(L, all_buttons, 'on'));

    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '全部隐藏', 'Position', [100, row2_y, 80, 22], ...
        'FontSize', 8, ...
        'Callback', @(~,~) set_all_layers_tr(L, all_buttons, 'off'));

    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '仅看拼接+真实', 'Position', [188, row2_y, 110, 22], ...
        'FontSize', 8, ...
        'Callback', @(~,~) show_only_tr(L, all_buttons, [7, 8, 9]));

    drawnow;
end

function handles = draw_struct_array_segments_tr(ax, segments, color, lw, ms, style)
    handles = gobjects(0);
    for s = 1:length(segments)
        seg = segments{s};
        lats = [seg.lat]; lons = [seg.lon];
        if length(lats) > 1
            h = geoplot(ax, lats, lons, style, 'Color', color, ...
                        'LineWidth', lw, 'MarkerSize', ms);
            handles(end+1) = h;
        elseif length(lats) == 1
            h = geoplot(ax, lats, lons, style(2), 'Color', color, ...
                        'MarkerSize', ms);
            handles(end+1) = h;
        end
    end
end

function handles = draw_cell_segments_tr(ax, segments, color, lw, ms, style)
    handles = gobjects(0);
    for s = 1:length(segments)
        fc = segments{s};
        if isempty(fc), continue; end
        lats = []; lons = [];
        for i = 1:length(fc)
            if ~isempty(fc{i}) && isfield(fc{i},'lat') && ~isnan(fc{i}.lat)
                lats(end+1) = fc{i}.lat;
                lons(end+1) = fc{i}.lon;
            end
        end
        if length(lats) > 1
            h = geoplot(ax, lats, lons, style, 'Color', color, ...
                        'LineWidth', lw, 'MarkerSize', ms);
            handles(end+1) = h;
        elseif length(lats) == 1
            h = geoplot(ax, lats, lons, style(2), 'Color', color, ...
                        'MarkerSize', ms);
            handles(end+1) = h;
        end
    end
end

function handles = draw_stitched_track_tr(ax, stitched, color, lw, ms, style)
    handles = gobjects(0);
    n = length(stitched);
    seg_start = 1;
    in_gap = false;

    for i = 1:n
        is_valid = ~isempty(stitched{i}) && isfield(stitched{i},'lat') ...
                   && ~isnan(stitched{i}.lat);
        if is_valid && in_gap
            seg_start = i;
            in_gap = false;
        elseif ~is_valid && ~in_gap
            count = i - seg_start;
            if count >= 1
                lats = zeros(1, count);
                lons = zeros(1, count);
                for j = seg_start:(i-1)
                    lats(j-seg_start+1) = stitched{j}.lat;
                    lons(j-seg_start+1) = stitched{j}.lon;
                end
                h = geoplot(ax, lats, lons, style, 'Color', color, ...
                            'LineWidth', lw, 'MarkerSize', ms);
                handles(end+1) = h;
            end
            in_gap = true;
        end
    end

    if ~in_gap
        count = n - seg_start + 1;
        if count >= 1
            lats = zeros(1, count);
            lons = zeros(1, count);
            for j = seg_start:n
                lats(j-seg_start+1) = stitched{j}.lat;
                lons(j-seg_start+1) = stitched{j}.lon;
            end
            h = geoplot(ax, lats, lons, style, 'Color', color, ...
                        'LineWidth', lw, 'MarkerSize', ms);
            handles(end+1) = h;
        end
    end
end

function toggle_layer_tr(src, handles)
    if src.Value
        state = 'on';
    else
        state = 'off';
    end
    for k = 1:length(handles)
        set(handles(k), 'Visible', state);
    end
end

function set_all_layers_tr(layers, buttons, state)
    for i = 1:length(layers)
        for k = 1:length(layers{i})
            set(layers{i}(k), 'Visible', state);
        end
        if strcmp(state, 'on')
            buttons(i).Value = 1;
        else
            buttons(i).Value = 0;
        end
    end
end

function show_only_tr(layers, buttons, show_idx)
    for i = 1:length(layers)
        for k = 1:length(layers{i})
            set(layers{i}(k), 'Visible', 'off');
        end
        buttons(i).Value = 0;
    end
    for i = show_idx
        for k = 1:length(layers{i})
            set(layers{i}(k), 'Visible', 'on');
        end
        buttons(i).Value = 1;
    end
end

% =========================================================================
% 6. plot_error_timeline — 双基地雷达跟踪的误差时间线图
% 来源: plot_error_timeline.m
% =========================================================================
function plot_error_timeline(trackState_R1, trackState_R2, detList_R1, detList_R2, ...
        true_track, t1_grid, t2_grid, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);

    n_frames = length(trackState_R1);
    err_R1 = nan(n_frames, 1);
    err_R2 = nan(n_frames, 1);
    err_det_R1 = nan(n_frames, 1);
    err_det_R2 = nan(n_frames, 1);

    for k = 1:n_frames
        t = t1_grid(k);
        true_lon = interp1(true_track(:,5), true_track(:,1), t, 'linear', 'extrap');
        true_lat = interp1(true_track(:,5), true_track(:,2), t, 'linear', 'extrap');

        s = trackState_R1{k};
        if ~isempty(s) && isfield(s, 'lat') && ~isnan(s.lat)
            err_R1(k) = sphere_utils_haversine_distance(s.lon, s.lat, true_lon, true_lat);
        end

        dets = detList_R1{k};
        if ~isempty(dets)
            for d = 1:length(dets)
                if ~dets(d).is_clutter && isfield(dets(d), 'lat') && ~isnan(dets(d).lat)
                    err_det_R1(k) = sphere_utils_haversine_distance(...
                        dets(d).lon, dets(d).lat, true_lon, true_lat);
                    break;
                end
            end
        end
    end

    for k = 1:length(t2_grid)
        t = t2_grid(k);
        if k > n_frames, break; end
        true_lon = interp1(true_track(:,5), true_track(:,1), t, 'linear', 'extrap');
        true_lat = interp1(true_track(:,5), true_track(:,2), t, 'linear', 'extrap');

        s = trackState_R2{k};
        if ~isempty(s) && isfield(s, 'lat') && ~isnan(s.lat)
            err_R2(k) = sphere_utils_haversine_distance(s.lon, s.lat, true_lon, true_lat);
        end

        dets = detList_R2{k};
        if ~isempty(dets)
            for d = 1:length(dets)
                if ~dets(d).is_clutter && isfield(dets(d), 'lat') && ~isnan(dets(d).lat)
                    err_det_R2(k) = sphere_utils_haversine_distance(...
                        dets(d).lon, dets(d).lat, true_lon, true_lat);
                    break;
                end
            end
        end
    end

    subplot(2, 1, 1);
    plot(t1_grid(1:n_frames)/60, err_R1/1000, 'b-', 'LineWidth', 1, 'DisplayName', 'R1 UKF滤波');
    hold on;
    plot(t2_grid(1:n_frames)/60, err_R2(1:n_frames)/1000, 'r-', 'LineWidth', 1, 'DisplayName', 'R2 UKF滤波');
    plot(t1_grid(1:n_frames)/60, err_det_R1/1000, 'b.', 'MarkerSize', 3, 'DisplayName', 'R1 点迹');
    plot(t2_grid(1:n_frames)/60, err_det_R2(1:n_frames)/1000, 'r.', 'MarkerSize', 3, 'DisplayName', 'R2 点迹');
    ylabel('位置误差 (km)');
    xlabel('时间 (min)');
    title('位置误差时序');
    legend('Location', 'best');
    grid on;

    subplot(2, 1, 2);
    hold on;
    ylim([0, 5]);

    for k = 1:n_frames
        s = trackState_R1{k};
        if isempty(s), continue; end
        if strcmp(s.status, 'TRACKING') && s.associated && ~s.assc_is_clutter
            plot(k, 4, 'b.', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && s.associated && s.assc_is_clutter
            plot(k, 4, 'rx', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && ~s.associated
            plot(k, 4, 'b.', 'MarkerSize', 4, 'Color', [0.5 0.5 0.5]);
        end
    end

    for k = 1:n_frames
        s = trackState_R2{k};
        if isempty(s), continue; end
        if strcmp(s.status, 'TRACKING') && s.associated && ~s.assc_is_clutter
            plot(k, 3, 'r.', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && s.associated && s.assc_is_clutter
            plot(k, 3, 'rx', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && ~s.associated
            plot(k, 3, 'r.', 'MarkerSize', 4, 'Color', [0.5 0.5 0.5]);
        end
    end

    yticks([3 4]);
    yticklabels({'R2', 'R1'});
    xlabel('帧号');
    title('检测/关联事件 ●=关联目标 ×=关联虚警 ·=漏检');
    grid on;

    sgtitle(sprintf('误差与事件时间线 (nFrames=%d)', n_frames));
    drawnow;
end

% =========================================================================
% 7. plot_error_timeline_turn — 拐弯目标的滤波误差时间线对比图
% 来源: plot_error_timeline_turn.m
% =========================================================================
function plot_error_timeline_turn(true_track, ...
        trackR1_base, trackR2_base, ...
        trackR1_ad, trackR2_ad, params, out_dir)

    n_frames = length(trackR1_base);

    err_r1_base = nan(1, n_frames);
    err_r2_base = nan(1, n_frames);
    err_r1_ad   = nan(1, n_frames);
    err_r2_ad   = nan(1, n_frames);

    for k = 1:n_frames
        if k <= size(true_track, 1)
            t_lon = true_track(k, 1);
            t_lat = true_track(k, 2);

            snap = trackR1_base{k};
            if ~isempty(snap.trackList)
                trk = snap.trackList{1};
                if trk.type == 1 && isfield(trk, 'lat') && ~isnan(trk.lat)
                    err_r1_base(k) = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
                end
            end

            snap2 = trackR2_base{k};
            if ~isempty(snap2.trackList)
                trk2 = snap2.trackList{1};
                if trk2.type == 1 && isfield(trk2, 'lat') && ~isnan(trk2.lat)
                    err_r2_base(k) = sphere_utils_haversine_distance(trk2.lon, trk2.lat, t_lon, t_lat) / 1000;
                end
            end

            snap_a = trackR1_ad{k};
            if ~isempty(snap_a.trackList)
                trk_a = snap_a.trackList{1};
                if trk_a.type == 1 && isfield(trk_a, 'lat') && ~isnan(trk_a.lat)
                    err_r1_ad(k) = sphere_utils_haversine_distance(trk_a.lon, trk_a.lat, t_lon, t_lat) / 1000;
                end
            end

            snap2_a = trackR2_ad{k};
            if ~isempty(snap2_a.trackList)
                trk2_a = snap2_a.trackList{1};
                if trk2_a.type == 1 && isfield(trk2_a, 'lat') && ~isnan(trk2_a.lat)
                    err_r2_ad(k) = sphere_utils_haversine_distance(trk2_a.lon, trk2_a.lat, t_lon, t_lat) / 1000;
                end
            end
        end
    end

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tlo);
    hold(ax1, 'on');
    t = 0:params.dt_sec:(n_frames-1)*params.dt_sec;
    t_plot = t(1:min(length(t), length(err_r1_base)));

    p1 = plot(ax1, t_plot, err_r1_base, '-', 'Color', [0.3 0.5 1.0], 'LineWidth', 1.5, ...
        'DisplayName', 'R1 基础UKF');
    p2 = plot(ax1, t_plot, err_r1_ad, 'b-', 'LineWidth', 1.8, ...
        'DisplayName', 'R1 自适应UKF');

    mid_t = t_plot(round(end/2));
    xline(ax1, mid_t, 'k--', '拐弯区', 'LineWidth', 1, 'Alpha', 0.5);

    xlabel(ax1, '时间 (s)');
    ylabel(ax1, '位置误差 (km)');
    title(ax1, 'R1 滤波误差对比');
    legend(ax1, 'Location', 'best');
    grid(ax1, 'on');

    ax2 = nexttile(tlo);
    hold(ax2, 'on');

    plot(ax2, t_plot, err_r2_base, '-', 'Color', [1.0 0.4 0.4], 'LineWidth', 1.5, ...
        'DisplayName', 'R2 基础UKF');
    plot(ax2, t_plot, err_r2_ad, 'r-', 'LineWidth', 1.8, ...
        'DisplayName', 'R2 自适应UKF');

    xline(ax2, mid_t, 'k--', '拐弯区', 'LineWidth', 1, 'Alpha', 0.5);

    xlabel(ax2, '时间 (s)');
    ylabel(ax2, '位置误差 (km)');
    title(ax2, 'R2 滤波误差对比');
    legend(ax2, 'Location', 'best');
    grid(ax2, 'on');

    sgtitle('拐弯目标: 基础UKF vs 机动自适应UKF (机动检测+Q提升)');

    drawnow;
end
