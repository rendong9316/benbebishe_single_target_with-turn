% =========================================================================
% plot_single_fusion_result.m
% 单目标融合航迹可视化: 地图叠加 + 误差收敛曲线，带图层复选框
% =========================================================================

function plot_single_fusion_result(true_track, trackSnapshots_R1, trackSnapshots_R2, ...
        all_fused_snapshots, method_names, best_idx, fusion_eval, truthTraj, params, out_dir)

    n_methods = length(method_names);
    frame_times = (0:length(trackSnapshots_R1)-1) * params.dt_sec;

    %% ===== Figure 1: 地图叠加 =====
    fig1 = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.68, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.68, 0.88]);
    end
    hold(ax, 'on');

    h_all = []; layer_names = {};

    % 真值
    h = geoplot(ax, true_track(:,2), true_track(:,1), '--s', ...
        'Color', 'g', 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'g', ...
        'DisplayName', '真值');
    h_all(end+1) = h; layer_names{end+1} = '真值航迹';

    % R1 UKF
    r1_tracks = collect_positions(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 2
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', sprintf('R1 UKF#%d', trk.id));
            h_all(end+1) = h; layer_names{end+1} = sprintf('R1 UKF#%d', trk.id);
        end
    end

    % R2 UKF
    r2_tracks = collect_positions(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 2
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', sprintf('R2 UKF#%d', trk.id));
            h_all(end+1) = h; layer_names{end+1} = sprintf('R2 UKF#%d', trk.id);
        end
    end

    % 四种融合航迹
    method_colors = {[0 0.5 0], [0.8 0.4 0], [0 0 0.8], [0.6 0 0.6]};
    for m = 1:n_methods
        snaps_m = all_fused_snapshots{m};
        fused_pos = collect_fused_positions(snaps_m);
        for t = 1:length(fused_pos)
            ft = fused_pos{t};
            if length(ft.lat_history) > 2
                lw = 3.0; if m == best_idx, lw = 3.5; end
                h = geoplot(ax, ft.lat_history, ft.lon_history, '-d', ...
                    'Color', method_colors{m}, 'LineWidth', lw, ...
                    'MarkerSize', 5, 'MarkerFaceColor', method_colors{m}, ...
                    'DisplayName', sprintf('%s 融合', method_names{m}));
                h_all(end+1) = h;
                layer_names{end+1} = sprintf('%s 融合航迹', method_names{m});
            end
        end
    end

    % 站点
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, sprintf('单目标双基地雷达航迹融合结果 (%s最优)', method_names{best_idx}));

    % ---- 右侧图层控制面板 ----
    n_layers1 = length(layer_names);
    cb1 = gobjects(1, n_layers1);
    for i = 1:n_layers1
        ypos = 0.92 - (i-1) * 0.040;
        if ypos < 0.05, break; end
        cb1(i) = uicontrol('Parent', fig1, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.74, ypos, 0.24, 0.036], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(h_all(i), src.Value));
    end

    btn_bottom1 = 0.92 - n_layers1 * 0.040 - 0.01;
    if btn_bottom1 > 0.02
        uicontrol('Parent', fig1, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.74, btn_bottom1, 0.11, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb(src, cb1, h_all));
        uicontrol('Parent', fig1, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.86, btn_bottom1, 0.11, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb(cb1, h_all));
    end

    % 融合结果标注
    best_rmse = fusion_eval.overall(best_idx).s.rms;
    uicontrol('Parent', fig1, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.74, 0.005, 0.24, 0.04], ...
        'String', sprintf('最佳: %s RMSE=%.1fkm', method_names{best_idx}, best_rmse), ...
        'FontSize', 9, 'BackgroundColor', [1 1 1], 'FontWeight', 'bold');

    drawnow;
    try
        exportgraphics(fig1, fullfile(out_dir, 'fig6_single_fusion_map.png'), 'Resolution', 200);
    catch
        saveas(fig1, fullfile(out_dir, 'fig6_single_fusion_map.png'));
    end
    fprintf('  融合地图已保存: fig6_single_fusion_map.png\n');

    %% ===== Figure 2: 误差收敛曲线 =====
    fig2 = figure('Position', [50, 50, 1400, 750]);
    win = 10;

    method_ls = {'-', '--', '-.', ':'};
    method_clr = {[0 0 0], [1 0 0], [0 0 1], [0 0.7 0]};

    subplot(1, 2, 1);
    hold on; grid on;
    all_h_lines = []; all_line_names = {};

    for m = 1:n_methods
        fe = build_frame_errors(all_fused_snapshots{m}, truthTraj, frame_times);
        if length(fe) >= win
            smoothed = movmean(fe, win, 'omitnan');
            h = plot(frame_times, smoothed, 'LineStyle', method_ls{m}, ...
                'Color', method_clr{m}, 'LineWidth', 2);
            all_h_lines(end+1) = h;
            all_line_names{end+1} = method_names{m};
        end
    end

    % R1
    r1_fe = build_single_frame_errors(trackSnapshots_R1, truthTraj, frame_times);
    if length(r1_fe) >= win
        h = plot(frame_times, movmean(r1_fe, win, 'omitnan'), ...
            ':', 'Color', [0 0 0.7], 'LineWidth', 1.5);
        all_h_lines(end+1) = h;
        all_line_names{end+1} = 'R1 UKF';
    end

    % R2
    r2_fe = build_single_frame_errors(trackSnapshots_R2, truthTraj, frame_times);
    if length(r2_fe) >= win
        h = plot(frame_times, movmean(r2_fe, win, 'omitnan'), ...
            ':', 'Color', [0.7 0 0], 'LineWidth', 1.5);
        all_h_lines(end+1) = h;
        all_line_names{end+1} = 'R2 UKF';
    end

    xlabel('时间 (s)'); ylabel('位置误差 (km)');
    title(sprintf('误差收敛曲线 (滑动平均 %d帧)', win));
    legend(all_line_names, 'FontSize', 8, 'Location', 'best');

    % CDF
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

    % ---- 右侧曲线控制面板 ----
    n2 = length(all_line_names);
    cb2 = gobjects(1, n2);
    for i = 1:n2
        ypos = 0.92 - (i-1) * 0.05;
        if ypos < 0.05, break; end
        cb2(i) = uicontrol('Parent', fig2, 'Style', 'checkbox', ...
            'String', all_line_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.045], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(all_h_lines(i), src.Value));
    end

    drawnow;
    try
        exportgraphics(fig2, fullfile(out_dir, 'fig7_single_fusion_error.png'), 'Resolution', 200);
    catch
        saveas(fig2, fullfile(out_dir, 'fig7_single_fusion_error.png'));
    end
    fprintf('  误差收敛曲线已保存: fig7_single_fusion_error.png\n');
end

% =========================================================================
% 辅助函数
% =========================================================================

function tracks = collect_positions(snapshots)
    track_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end
            tid = trk.id;
            if ~track_map.isKey(tid)
                track_map(tid) = struct('id', tid, 'lat_history', [], 'lon_history', []);
            end
            rec = track_map(tid);
            rec.lat_history(end+1) = trk.lat;
            rec.lon_history(end+1) = trk.lon;
            track_map(tid) = rec;
        end
    end
    tracks = values(track_map);
end

function tracks = collect_fused_positions(snapshots)
    track_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            ft = snap.trackList{t};
            pid = ft.id;
            if ~track_map.isKey(pid)
                track_map(pid) = struct('id', pid, 'lat_history', [], 'lon_history', []);
            end
            rec = track_map(pid);
            rec.lat_history(end+1) = ft.lat;
            rec.lon_history(end+1) = ft.lon;
            track_map(pid) = rec;
        end
    end
    tracks = values(track_map);
end

function fe = build_frame_errors(fused_snaps, truth, frame_times)
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
            d = haversine_km(ft.lon, ft.lat, t_lon, t_lat);
            if d < best_d, best_d = d; end
        end
        if best_d < inf, fe(k) = best_d; end
    end
end

function fe = build_single_frame_errors(snapshots, truth, frame_times)
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
            d = haversine_km(trk.lon, trk.lat, t_lon, t_lat);
            if d < best_d, best_d = d; end
        end
        if best_d < inf, fe(k) = best_d; end
    end
end

function d = haversine_km(lon1, lat1, lon2, lat2)
    R = 6371;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2)^2;
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end

function try_set_visible(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_cb(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible(h_all(i), new_val);
    end
end

function show_all_cb(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible(h_all(i), 1);
    end
end
