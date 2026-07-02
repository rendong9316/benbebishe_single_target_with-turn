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
        ypos = 0.92 - (i-1) * 0.045;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_multi(h_all(i), src.Value));
    end

    btn_bottom = 0.92 - n_layers * 0.045 - 0.01;
    if btn_bottom > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.76, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb_multi(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.87, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb_multi(cb, h_all));
    end

    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.005, 0.22, 0.03], ...
        'String', sprintf('R1:%d航迹 R2:%d航迹 | Pd=%.0f%% Pfa=%.3f', ...
        length(r1_tracks), length(r2_tracks), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
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

    % R1 UKF 航迹
    r1_tracks = collect_active_tracks_multi(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R1 UKF#%d', trk.id);
            if length(r1_tracks) > 1, seg_label = sprintf('R1 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 2.0, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % R2 UKF 航迹
    r2_tracks = collect_active_tracks_multi(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            if length(r2_tracks) > 1, seg_label = sprintf('R2 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 2.0, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', seg_label);
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
        ypos = 0.92 - (i-1) * 0.045;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_multi(h_all(i), src.Value));
    end

    btn_bottom = 0.92 - n_layers * 0.045 - 0.01;
    if btn_bottom > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.76, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb_multi(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.87, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb_multi(cb, h_all));
    end

    % 融合算法统计信息
    n_methods = length(method_names);
    info_str = sprintf('%d匹配对 x %d融合算法', length(matched_pairs), n_methods);
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.005, 0.22, 0.03], ...
        'String', info_str, ...
        'FontSize', 8, 'BackgroundColor', [1 1 1], 'FontWeight', 'bold');

    drawnow;

    %% ====== Figure 2: 误差收敛曲线 + CDF ======
    fig2 = figure('Position', [50, 50, 1400, 750]);
    n_frames_total = length(trackSnapshots_R1);
    frame_times = (0:n_frames_total-1) * params.dt_sec;

    method_ls = {'-', '--', '-.', ':'};
    method_clr = {[0 0 0], [1 0 0], [0 0 1], [0 0.7 0]};
    all_h_lines = []; all_line_names = {};

    subplot(1, 2, 1);
    hold on; grid on;

    % 每条匹配对的每种融合算法画误差曲线
    for p = 1:length(matched_pairs)
        mp = matched_pairs{p};
        for m = 1:n_methods
            snaps_m = all_fused_snapshots{p,m};
            errs = build_frame_errors_multi(snaps_m, truthTrajs, frame_times, mp);
            if ~isempty(errs) && sum(~isnan(errs)) > 2
                smoothed = movmean(errs, 10, 'omitnan');
                h = plot(frame_times, smoothed, 'LineStyle', method_ls{m}, ...
                    'Color', method_clr{m}, 'LineWidth', 1.5);
                all_h_lines(end+1) = h;
                all_line_names{end+1} = sprintf('%s Pair#%d', method_names{m}, p);
            end
        end
    end

    % R1/R2 单站误差
    for r = 1:2
        if r == 1
            snaps = trackSnapshots_R1;
            clr = [0 0 0.7];
        else
            snaps = trackSnapshots_R2;
            clr = [0.7 0 0];
        end
        for ac = 1:3
            fe = build_single_frame_errors_multi(snaps, truthTrajs{ac}, frame_times);
            if ~isempty(fe) && sum(~isnan(fe)) > 2
                smoothed = movmean(fe, 10, 'omitnan');
                h = plot(frame_times, smoothed, ':', 'Color', clr, 'LineWidth', 1.5);
                all_h_lines(end+1) = h;
                all_line_names{end+1} = sprintf('R%d UKF Target%c', r, char('A'+ac-1));
            end
        end
    end

    xlabel('时间 (s)'); ylabel('位置误差 (km)');
    title('误差收敛曲线 (滑动平均 10帧)');
    legend(all_line_names, 'FontSize', 7, 'Location', 'best', 'NumColumns', 3);

    subplot(1, 2, 2);
    hold on; grid on;
    for p = 1:length(matched_pairs)
        mp = matched_pairs{p};
        for m = 1:n_methods
            errs = [];
            snaps_m = all_fused_snapshots{p,m};
            for k = 1:length(snaps_m)
                snap = snaps_m{k};
                if isempty(snap.trackList), continue; end
                ft = snap.trackList{1};
                if isnan(ft.lat), continue; end
                best_d = inf;
                for ac = 1:3
                    truth_ac = truthTrajs{ac};
                    t_lon = interp1(truth_ac.time_sec, truth_ac.lon, frame_times(k), 'linear', 'extrap');
                    t_lat = interp1(truth_ac.time_sec, truth_ac.lat, frame_times(k), 'linear', 'extrap');
                    if isnan(t_lat), continue; end
                    d = sphere_utils_haversine_distance(ft.lon, ft.lat, t_lon, t_lat) / 1000;
                    if d < best_d, best_d = d; end
                end
                if best_d < inf, errs(end+1) = best_d; end
            end
            if ~isempty(errs)
                [f, x] = ecdf(errs);
                plot(x, f*100, 'LineStyle', method_ls{m}, ...
                    'Color', method_clr{m}, 'LineWidth', 2);
            end
        end
    end
    xlabel('位置误差 (km)'); ylabel('累积概率 (%)');
    title('融合误差CDF');
    legend(method_names, 'FontSize', 8, 'Location', 'southeast');

    drawnow;
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
    % 按航迹ID分组，每条航迹独立收集历史点
    all_tracks = {};  % {id, lat_history, lon_history}
    for k = 1:length(snaps)
        snap = snaps{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7 || isnan(trk.lat), continue; end
            % 查找是否已有该ID的航迹
            found = false;
            for i = 1:length(all_tracks)
                if all_tracks{i}.id == trk.id
                    all_tracks{i}.lat_history(end+1) = trk.lat;
                    all_tracks{i}.lon_history(end+1) = trk.lon;
                    found = true;
                    break;
                end
            end
            if ~found
                all_tracks{end+1} = struct('id', trk.id, ...
                    'lat_history', trk.lat, 'lon_history', trk.lon);
            end
        end
    end
    tracks = all_tracks;
end

function tracks = collect_fused_positions_multi(snaps)
    all_tracks = {};
    for k = 1:length(snaps)
        snap = snaps{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            ft = snap.trackList{t};
            if isnan(ft.lat) || isnan(ft.lon), continue; end
            found = false;
            for i = 1:length(all_tracks)
                if all_tracks{i}.id == ft.id
                    all_tracks{i}.lat_history(end+1) = ft.lat;
                    all_tracks{i}.lon_history(end+1) = ft.lon;
                    found = true;
                    break;
                end
            end
            if ~found
                all_tracks{end+1} = struct('id', ft.id, ...
                    'lat_history', ft.lat, 'lon_history', ft.lon);
            end
        end
    end
    tracks = all_tracks;
end

function try_set_visible_multi(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_cb_multi(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible_multi(h_all(i), new_val);
    end
end

function show_all_cb_multi(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible_multi(h_all(i), 1);
    end
end

function errs = build_frame_errors_multi(fused_snaps, truthTrajs, frame_times, mp)
    errs = nan(1, length(fused_snaps));
    for k = 1:length(fused_snaps)
        snap = fused_snaps{k};
        if isempty(snap.trackList), continue; end
        ft = snap.trackList{1};
        if isnan(ft.lat), continue; end
        best_d = inf;
        for ac = 1:length(truthTrajs)
            t_lon = interp1(truthTrajs{ac}.time_sec, truthTrajs{ac}.lon, frame_times(k), 'linear', 'extrap');
            t_lat = interp1(truthTrajs{ac}.time_sec, truthTrajs{ac}.lat, frame_times(k), 'linear', 'extrap');
            if isnan(t_lat), continue; end
            d = sphere_utils_haversine_distance(ft.lon, ft.lat, t_lon, t_lat) / 1000;
            if d < best_d, best_d = d; end
        end
        if best_d < inf, errs(k) = best_d; end
    end
end

function errs = build_single_frame_errors_multi(snaps, truth_ac, frame_times)
    errs = nan(1, length(snaps));
    for k = 1:length(snaps)
        snap = snaps{k};
        if isempty(snap.trackList), continue; end
        t_lon = interp1(truth_ac.time_sec, truth_ac.lon, frame_times(k), 'linear', 'extrap');
        t_lat = interp1(truth_ac.time_sec, truth_ac.lat, frame_times(k), 'linear', 'extrap');
        if isnan(t_lat), continue; end
        best_d = inf;
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end
            if isnan(trk.lat), continue; end
            d = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
            if d < best_d, best_d = d; end
        end
        if best_d < inf, errs(k) = best_d; end
    end
end
