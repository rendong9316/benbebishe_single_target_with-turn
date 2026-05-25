% =========================================================================
% plot_turn_stats.m
% 聚合式绘图调度文件 — 拐弯目标统计/对比可视化
% =========================================================================
%
% 【功能概述】
%   将原来分散在 4 个独立 .m 文件中的统计分析绘图函数合并到一个
%   文件中，通过 mode 字符串调度对应的子函数。
%
% 【调度模式】
%   'comparison'     -> plot_turn_comparison(...)
%   'fusion_compare' -> plot_turn_fusion_compare(...)
%   'rmse_bars'      -> plot_turn_rmse_bars(...)
%   'single_compare' -> plot_turn_single_compare(...)
%
% 【内部辅助函数命名约定】
%   为避免多个子函数之间的同名冲突，所有辅助函数均添加父函数缩写后缀。
%   tcomp = turn_comparison
%   tfc   = turn_fusion_compare
%   trb   = turn_rmse_bars
%   tsc   = turn_single_compare
%
% =========================================================================

function plot_turn_stats(mode, varargin)
    switch mode
        case 'comparison'
            plot_turn_comparison(varargin{:});
        case 'fusion_compare'
            plot_turn_fusion_compare(varargin{:});
        case 'rmse_bars'
            plot_turn_rmse_bars(varargin{:});
        case 'single_compare'
            plot_turn_single_compare(varargin{:});
        otherwise
            error('plot_turn_stats: unknown mode "%s". Valid: comparison, fusion_compare, rmse_bars, single_compare', mode);
    end
end

% =========================================================================
% 1. plot_turn_comparison — 拐弯目标航迹对比图
% 来源: plot_turn_comparison.m
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

    h = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2.5, 'DisplayName', '真实航迹');
    h_all(end+1) = h;
    layer_names{end+1} = '真实航迹';

    [lat1, lon1] = extract_track_ll_tcomp(trackR1_base);
    if ~isempty(lat1)
        h = geoplot(ax, lat1, lon1, '-', 'Color', [0.3 0.5 1.0], ...
            'LineWidth', 1.8, 'DisplayName', 'R1 基础UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 基础UKF';
    end

    [lat2, lon2] = extract_track_ll_tcomp(trackR2_base);
    if ~isempty(lat2)
        h = geoplot(ax, lat2, lon2, '-', 'Color', [1.0 0.4 0.4], ...
            'LineWidth', 1.8, 'DisplayName', 'R2 基础UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 基础UKF';
    end

    [lat1a, lon1a] = extract_track_ll_tcomp(trackR1_ad);
    if ~isempty(lat1a)
        h = geoplot(ax, lat1a, lon1a, 'b-', ...
            'LineWidth', 2.2, 'DisplayName', 'R1 自适应UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 自适应UKF(机动检测)';
    end

    [lat2a, lon2a] = extract_track_ll_tcomp(trackR2_ad);
    if ~isempty(lat2a)
        h = geoplot(ax, lat2a, lon2a, 'r-', ...
            'LineWidth', 2.2, 'DisplayName', 'R2 自适应UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 自适应UKF(机动检测)';
    end

    [lat_fb, lon_fb] = extract_fused_ll_tcomp(fused_base{best_m_base});
    if ~isempty(lat_fb)
        h = geoplot(ax, lat_fb, lon_fb, 'c-', ...
            'LineWidth', 2.5, 'DisplayName', '基础UKF融合');
        h_all(end+1) = h;
        layer_names{end+1} = sprintf('基础UKF融合(%s)', fuse_methods{best_m_base});
    end

    [lat_fa, lon_fa] = extract_fused_ll_tcomp(fused_ad{best_m_ad});
    if ~isempty(lat_fa)
        h = geoplot(ax, lat_fa, lon_fa, 'm-', ...
            'LineWidth', 2.5, 'DisplayName', '自适应UKF融合');
        h_all(end+1) = h;
        layer_names{end+1} = sprintf('自适应UKF融合(%s)', fuse_methods_ad{best_m_ad});
    end

    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', '终点');

    mid_idx = round(size(true_track,1)/2);
    geoplot(ax, true_track(mid_idx,2), true_track(mid_idx,1), 'wo', ...
        'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '拐点(~120°)');

    title(ax, '拐弯目标: 基础UKF vs 机动自适应UKF 对比');
    subtitle(ax, sprintf('120°拐角 Pd=%.0f%% Pfa=%.3f 航速%.0fm/s', ...
        params.detection_probability*100, params.false_alarm_rate, 140));

    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);

    for i = 1:n_layers
        ypos = 0.93 - (i-1) * 0.048;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.73, ypos, 0.25, 0.042], ...
            'FontSize', 8, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_turn_tcomp(h_all(i), src.Value));
    end

    btn_y = 0.93 - n_layers * 0.048 - 0.01;
    if btn_y > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.73, btn_y, 0.12, 0.038], ...
            'FontSize', 8, ...
            'Callback', @(src, ~) toggle_all_turn_tcomp(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.86, btn_y, 0.12, 0.038], ...
            'FontSize', 8, ...
            'Callback', @(~, ~) show_all_turn_tcomp(cb, h_all));
    end

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

function [lats, lons] = extract_track_ll_tcomp(snapshots)
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

function [lats, lons] = extract_fused_ll_tcomp(snapshots)
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

function try_set_visible_turn_tcomp(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_turn_tcomp(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible_turn_tcomp(h_all(i), new_val);
    end
end

function show_all_turn_tcomp(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible_turn_tcomp(h_all(i), 1);
    end
end

% =========================================================================
% 2. plot_turn_fusion_compare — 拐弯目标的融合对比综合图
% 来源: plot_turn_fusion_compare.m
% =========================================================================
function plot_turn_fusion_compare(true_track, ...
        fused_base, fuse_methods, best_m_base, ...
        fused_ad, fuse_methods_ad, best_m_ad, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, ...
        fusion_eval_base, fusion_eval_ad, params, out_dir)

    [lat_fb, lon_fb] = extract_fused_ll_tfc(fused_base{best_m_base});
    [lat_fa, lon_fa] = extract_fused_ll_tfc(fused_ad{best_m_ad});

    [lat_r1b, lon_r1b] = extract_track_ll_tfc(trackR1_base);
    [lat_r1a, lon_r1a] = extract_track_ll_tfc(trackR1_ad);
    [lat_r2b, lon_r2b] = extract_track_ll_tfc(trackR2_base);
    [lat_r2a, lon_r2a] = extract_track_ll_tfc(trackR2_ad);

    mid = round(size(true_track,1)/2);
    turn_lon = 128.5; turn_lat = 33.5;
    min_dist = inf; turn_frame = mid;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), turn_lon, turn_lat);
        if d < min_dist, min_dist = d; turn_frame = kk; end
    end
    zoom_range = max(1,turn_frame-20):min(size(true_track,1),turn_frame+20);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % 子图 1: 融合全图
    nexttile(tlo, 1);
    try
        gx1 = geoaxes;
        gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes;
    end
    hold(gx1, 'on');
    title(gx1, sprintf('融合全图: 基础%s(虚线) vs 自适应%s(实线)', ...
        fuse_methods{best_m_base}, fuse_methods_ad{best_m_ad}), 'FontSize', 10);

    geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.8, 'DisplayName', '真值');
    h_fb = geoplot(gx1, lat_fb, lon_fb, '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2, ...
        'DisplayName', sprintf('基础%s融合', fuse_methods{best_m_base}));
    h_fa = geoplot(gx1, lat_fa, lon_fa, '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 2.8, ...
        'DisplayName', sprintf('自适应%s融合', fuse_methods_ad{best_m_ad}));

    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    geoplot(gx1, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    legend(gx1, 'Location', 'northeast', 'FontSize', 7);

    rx = [min(true_track(zoom_range,1)), max(true_track(zoom_range,1))];
    ry = [min(true_track(zoom_range,2)), max(true_track(zoom_range,2))];
    geoplot(gx1, [ry(1) ry(1) ry(2) ry(2) ry(1)], ...
                 [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);

    % 子图 2: 拐弯区域放大
    nexttile(tlo, 2);
    try
        gx2 = geoaxes;
        gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes;
    end
    hold(gx2, 'on');
    title(gx2, '拐弯区域放大', 'FontSize', 10);

    geoplot(gx2, true_track(zoom_range,2), true_track(zoom_range,1), 'y--', 'LineWidth', 2.5);

    if ~isempty(lat_fb)
        iz = get_zoom_idx_tfc(lat_fb, zoom_range);
        geoplot(gx2, lat_fb(iz), lon_fb(iz), '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2.5);
    end
    if ~isempty(lat_fa)
        iz = get_zoom_idx_tfc(lat_fa, zoom_range);
        geoplot(gx2, lat_fa(iz), lon_fa(iz), '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 3);
    end

    if ~isempty(lat_r1b)
        iz = get_zoom_idx_tfc(lat_r1b, zoom_range);
        geoplot(gx2, lat_r1b(iz), lon_r1b(iz), ':', 'Color', [0.4 0.6 1.0 0.5], 'LineWidth', 1);
    end
    if ~isempty(lat_r2b)
        iz = get_zoom_idx_tfc(lat_r2b, zoom_range);
        geoplot(gx2, lat_r2b(iz), lon_r2b(iz), ':', 'Color', [1.0 0.5 0.5 0.5], 'LineWidth', 1);
    end

    legend(gx2, {'真值', '基础融合', '自适应融合', 'R1单站', 'R2单站'}, ...
        'Location', 'best', 'FontSize', 6);

    % 子图 3: RMSE 柱状图
    ax3 = nexttile(tlo, 3);
    methods_all = [fuse_methods, {'R1_only', 'R2_only'}];
    n_m = length(methods_all);
    rmse_base = zeros(1, n_m);
    rmse_ad   = zeros(1, n_m);
    for m = 1:n_m
        rmse_base(m) = fusion_eval_base.overall(m).s.rms;
        rmse_ad(m)   = fusion_eval_ad.overall(m).s.rms;
    end

    hold(ax3, 'on');
    x_pos = 1:n_m;
    w = 0.35;
    b1 = bar(ax3, x_pos - w/2, rmse_base, w, 'FaceColor', [0.6 0.6 0.6]);
    b2 = bar(ax3, x_pos + w/2, rmse_ad, w, 'FaceColor', [0.0 0.5 0.0]);
    set(ax3, 'XTick', x_pos, 'XTickLabel', methods_all, 'FontSize', 7);
    xtickangle(ax3, 30);
    ylabel(ax3, 'RMSE (km)');
    title(ax3, '融合RMSE对比: 基础(灰) vs 自适应(绿)', 'FontSize', 10);
    legend(ax3, [b1, b2], {'基础UKF融合', '自适应UKF融合'}, 'Location', 'best', 'FontSize', 7);
    grid(ax3, 'on');

    for m = 1:n_m
        if rmse_base(m) > 0
            imp = (1 - rmse_ad(m)/rmse_base(m))*100;
            y_pos = max(rmse_base(m), rmse_ad(m)) + 0.5;
            text(ax3, x_pos(m), y_pos, sprintf('%+.0f%%', imp), ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.8 0 0], ...
                'HorizontalAlignment', 'center');
        end
    end

    % 子图 4: 融合误差时间线
    ax4 = nexttile(tlo, 4);
    n_frames = length(fused_base{best_m_base});
    t = (0:n_frames-1) * params.dt_sec;

    err_fb = nan(1, n_frames);
    err_fa = nan(1, n_frames);
    for k = 1:min(n_frames, size(true_track,1))
        err_fb(k) = fused_err_at_frame_tfc(fused_base{best_m_base}{k}, true_track(k,1), true_track(k,2));
        err_fa(k) = fused_err_at_frame_tfc(fused_ad{best_m_ad}{k}, true_track(k,1), true_track(k,2));
    end

    hold(ax4, 'on');
    plot(ax4, t, err_fb, '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 1.5);
    plot(ax4, t, err_fa, '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 2);
    xline(ax4, t(turn_frame), 'k--', 'LineWidth', 0.8);
    xlabel(ax4, '时间 (s)'); ylabel(ax4, '位置误差 (km)');
    title(ax4, sprintf('融合误差: 基础RMSE=%.1f  自适应RMSE=%.1f', ...
        rms_val_tfc(err_fb), rms_val_tfc(err_fa)), 'FontSize', 10);
    legend(ax4, {sprintf('基础%s融合', fuse_methods{best_m_base}), ...
        sprintf('自适应%s融合', fuse_methods_ad{best_m_ad})}, 'Location', 'best', 'FontSize', 8);
    grid(ax4, 'on');

    % 子图 5: 单站 vs 融合误差对比
    ax5 = nexttile(tlo, 5);
    hold(ax5, 'on');

    r1b_rmse = fusion_eval_base.overall(end-1).s.rms;
    r1a_rmse = fusion_eval_ad.overall(end-1).s.rms;
    r2b_rmse = fusion_eval_base.overall(end).s.rms;
    r2a_rmse = fusion_eval_ad.overall(end).s.rms;
    fb_rmse  = fusion_eval_base.overall(best_m_base).s.rms;
    fa_rmse  = fusion_eval_ad.overall(best_m_ad).s.rms;

    methods_short = {'R1单站', 'R2单站', '融合'};
    base_vals = [r1b_rmse, r2b_rmse, fb_rmse];
    ad_vals   = [r1a_rmse, r2a_rmse, fa_rmse];

    xp = 1:3;
    bar(ax5, xp-0.2, base_vals, 0.35, 'FaceColor', [0.6 0.6 0.6], 'DisplayName', '基础UKF');
    bar(ax5, xp+0.2, ad_vals, 0.35, 'FaceColor', [0.0 0.5 0.0], 'DisplayName', '自适应UKF');
    set(ax5, 'XTick', xp, 'XTickLabel', methods_short);
    ylabel(ax5, 'RMSE (km)');
    title(ax5, '单站→融合 精度提升链', 'FontSize', 10);
    legend(ax5, 'Location', 'best', 'FontSize', 8);
    grid(ax5, 'on');

    for i = 1:3
        imp = (1 - ad_vals(i)/base_vals(i))*100;
        text(ax5, xp(i), max(base_vals(i), ad_vals(i))+0.3, sprintf('%+.0f%%', imp), ...
            'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end

    % 子图 6: 数值汇总表
    ax6 = nexttile(tlo, 6);
    ax6.Visible = 'off';
    text(0.05, 0.9, sprintf('=== 拐弯目标融合结果 ==='), ...
        'Units', 'normalized', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.05, 0.80, sprintf('基础UKF最优融合: %s  RMSE=%.1f km', ...
        fuse_methods{best_m_base}, fb_rmse), 'Units', 'normalized', 'FontSize', 9);
    text(0.05, 0.72, sprintf('自适应UKF最优融合: %s  RMSE=%.1f km', ...
        fuse_methods_ad{best_m_ad}, fa_rmse), 'Units', 'normalized', 'FontSize', 9);
    text(0.05, 0.64, sprintf('融合改善: %+.1f%%', (1-fa_rmse/fb_rmse)*100), ...
        'Units', 'normalized', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8 0 0]);
    text(0.05, 0.52, sprintf('R1单站: %.1f -> %.1f km (%+.1f%%)', ...
        r1b_rmse, r1a_rmse, (1-r1a_rmse/r1b_rmse)*100), 'Units', 'normalized', 'FontSize', 9);
    text(0.05, 0.44, sprintf('R2单站: %.1f -> %.1f km (%+.1f%%)', ...
        r2b_rmse, r2a_rmse, (1-r2a_rmse/r2b_rmse)*100), 'Units', 'normalized', 'FontSize', 9);
    text(0.05, 0.32, sprintf('Pd=%.0f%%  Pfa=%.3f', ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'Units', 'normalized', 'FontSize', 9);
    text(0.05, 0.24, sprintf('拐角: ~113°  帧数: %d', n_frames), ...
        'Units', 'normalized', 'FontSize', 9);

    sgtitle(sprintf('拐弯目标融合对比: 基础UKF(灰色/虚线) vs 机动自适应UKF(绿色/实线)'));

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig3_fusion_compare.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig3_fusion_compare.png'));
    end
    fprintf('  融合对比图已保存: fig3_fusion_compare.png\n');
end

function [lats, lons] = extract_fused_ll_tfc(snapshots)
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

function [lats, lons] = extract_track_ll_tfc(snapshots)
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

function d = fused_err_at_frame_tfc(snap, t_lon, t_lat)
    d = NaN;
    if isempty(snap.trackList), return; end
    trk = snap.trackList{1};
    if ~isfield(trk, 'lon') || isnan(trk.lon), return; end
    d = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
end

function iz = get_zoom_idx_tfc(arr, zoom_range)
    iz = zoom_range(zoom_range <= length(arr));
end

function v = rms_val_tfc(x)
    x_valid = x(~isnan(x));
    if isempty(x_valid), v = NaN; return; end
    v = sqrt(mean(x_valid.^2));
end

% =========================================================================
% 3. plot_turn_rmse_bars — 拐弯目标全部融合方法的RMSE对比柱状图
% 来源: plot_turn_rmse_bars.m
% =========================================================================
function plot_turn_rmse_bars(fusion_eval_base, fusion_eval_ad, ...
        fuse_methods, best_m_base, best_m_ad, params, out_dir)

    methods_all = [fuse_methods, {'R1_only', 'R2_only'}];
    n_m = length(methods_all);
    rmse_base = zeros(1, n_m);
    rmse_ad   = zeros(1, n_m);
    for m = 1:n_m
        rmse_base(m) = fusion_eval_base.overall(m).s.rms;
        rmse_ad(m)   = fusion_eval_ad.overall(m).s.rms;
    end

    fig = figure('Position', [50, 50, 1400, 750]);

    ax1 = axes('Units', 'normalized', 'Position', [0.08, 0.10, 0.55, 0.85]);
    hold(ax1, 'on');
    x_pos = 1:n_m;
    w = 0.35;
    b1 = bar(ax1, x_pos - w/2, rmse_base, w, 'FaceColor', [0.65 0.65 0.65], 'EdgeColor', 'none');
    b2 = bar(ax1, x_pos + w/2, rmse_ad, w, 'FaceColor', [0.0 0.45 0.0], 'EdgeColor', 'none');
    set(ax1, 'XTick', x_pos, 'XTickLabel', methods_all, 'FontSize', 11);
    ylabel(ax1, 'RMSE (km)', 'FontSize', 12);
    title(ax1, '融合RMSE对比: 基础UKF(灰) vs 机动自适应UKF(绿)', 'FontSize', 13);
    legend(ax1, [b1, b2], {'基础UKF融合', '自适应UKF融合'}, 'Location', 'northwest', 'FontSize', 11);
    grid(ax1, 'on');

    for m = 1:n_m
        text(ax1, x_pos(m)-0.15, rmse_base(m)+0.3, sprintf('%.1f', rmse_base(m)), ...
            'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', [0.4 0.4 0.4]);
        text(ax1, x_pos(m)+0.15, rmse_ad(m)+0.3, sprintf('%.1f', rmse_ad(m)), ...
            'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', [0 0.3 0]);
        if rmse_base(m) > 0
            imp = (1 - rmse_ad(m)/rmse_base(m))*100;
            c = [0.8 0 0]; if imp < 0, c = [0 0 0.6]; end
            yp = max(rmse_base(m), rmse_ad(m)) + 1.2;
            text(ax1, x_pos(m), yp, sprintf('%+.0f%%', imp), ...
                'FontSize', 11, 'FontWeight', 'bold', 'Color', c, 'HorizontalAlignment', 'center');
        end
    end

    ax2 = axes('Units', 'normalized', 'Position', [0.66, 0.10, 0.32, 0.85]);
    ax2.Visible = 'off';
    y = 0.95;
    text(0.05, y, '=== 拐弯目标仿真结果 ===', 'Units', 'normalized', 'FontSize', 13, 'FontWeight', 'bold');
    y = y - 0.08;
    text(0.05, y, sprintf('基础UKF最优融合: %s  %.1f km', fuse_methods{best_m_base}, ...
        fusion_eval_base.overall(best_m_base).s.rms), 'Units', 'normalized', 'FontSize', 10);
    y = y - 0.06;
    text(0.05, y, sprintf('自适应UKF最优融合: %s  %.1f km', fuse_methods{best_m_ad}, ...
        fusion_eval_ad.overall(best_m_ad).s.rms), 'Units', 'normalized', 'FontSize', 10);
    y = y - 0.06;

    fb_rmse = fusion_eval_base.overall(best_m_base).s.rms;
    fa_rmse = fusion_eval_ad.overall(best_m_ad).s.rms;
    imp_fusion = (1 - fa_rmse/fb_rmse)*100;
    text(0.05, y, sprintf('融合改善: %+.1f%%', imp_fusion), ...
        'Units', 'normalized', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.8 0 0]);
    y = y - 0.10;

    r1b = fusion_eval_base.overall(end-1).s.rms;
    r1a = fusion_eval_ad.overall(end-1).s.rms;
    r2b = fusion_eval_base.overall(end).s.rms;
    r2a = fusion_eval_ad.overall(end).s.rms;

    text(0.05, y, '--- 单站改善 ---', 'Units', 'normalized', 'FontSize', 11, 'FontWeight', 'bold');
    y = y - 0.07;
    text(0.05, y, sprintf('R1: %.1f → %.1f km (%+.0f%%)', r1b, r1a, (1-r1a/r1b)*100), ...
        'Units', 'normalized', 'FontSize', 10);
    y = y - 0.06;
    text(0.05, y, sprintf('R2: %.1f → %.1f km (%+.0f%%)', r2b, r2a, (1-r2a/r2b)*100), ...
        'Units', 'normalized', 'FontSize', 10);
    y = y - 0.10;

    text(0.05, y, '--- 参数 ---', 'Units', 'normalized', 'FontSize', 11, 'FontWeight', 'bold');
    y = y - 0.07;
    text(0.05, y, sprintf('Pd=%.0f%%  Pfa=%.3f', params.detection_probability*100, ...
        params.false_alarm_rate), 'Units', 'normalized', 'FontSize', 10);
    y = y - 0.06;
    text(0.05, y, '拐角: ~113°  帧数: 109', 'Units', 'normalized', 'FontSize', 10);
    y = y - 0.06;
    text(0.05, y, '航速: 140 m/s', 'Units', 'normalized', 'FontSize', 10);
    y = y - 0.10;

    text(0.05, y, '--- 图例 ---', 'Units', 'normalized', 'FontSize', 11, 'FontWeight', 'bold');
    y = y - 0.07;
    text(0.08, y, '灰色柱 = 基础UKF', 'Units', 'normalized', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);
    y = y - 0.05;
    text(0.08, y, '绿色柱 = 机动自适应UKF', 'Units', 'normalized', 'FontSize', 9, 'Color', [0 0.3 0]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig6_rmse_bars.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig6_rmse_bars.png'));
    end
    fprintf('  图6 已保存: fig6_rmse_bars.png\n');
end

% =========================================================================
% 4. plot_turn_single_compare — 拐弯目标单站对比综合图
% 来源: plot_turn_single_compare.m
% =========================================================================
function plot_turn_single_compare(true_track, detList_R1, detList_R2, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, params, out_dir)

    [lat_r1b, lon_r1b] = extract_valid_ll_tsc(trackR1_base);
    [lat_r2b, lon_r2b] = extract_valid_ll_tsc(trackR2_base);
    [lat_r1a, lon_r1a] = extract_valid_ll_tsc(trackR1_ad);
    [lat_r2a, lon_r2a] = extract_valid_ll_tsc(trackR2_ad);

    n_frames = length(trackR1_base);
    t = (0:n_frames-1) * params.dt_sec;
    err_r1b = nan(1, n_frames);
    err_r1a = nan(1, n_frames);
    err_r2b = nan(1, n_frames);
    err_r2a = nan(1, n_frames);

    for k = 1:min(n_frames, size(true_track,1))
        tl = true_track(k,1); tb = true_track(k,2);
        err_r1b(k) = err_at_frame_tsc(trackR1_base{k}, tl, tb);
        err_r1a(k) = err_at_frame_tsc(trackR1_ad{k}, tl, tb);
        err_r2b(k) = err_at_frame_tsc(trackR2_base{k}, tl, tb);
        err_r2a(k) = err_at_frame_tsc(trackR2_ad{k}, tl, tb);
    end

    turn_lon = 128.5; turn_lat = 33.5;
    min_dist = inf; turn_frame = round(n_frames/2);
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), turn_lon, turn_lat);
        if d < min_dist, min_dist = d; turn_frame = kk; end
    end
    zoom_half = 18;
    zoom_start = max(1, turn_frame - zoom_half);
    zoom_end   = min(size(true_track,1), turn_frame + zoom_half);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % 子图 1: R1 全图对比
    nexttile(tlo, 1);
    try
        gx1 = geoaxes;
        gx1.Basemap = 'darkwater';
    catch
        gx1 = geoaxes;
    end
    hold(gx1, 'on');
    title(gx1, 'R1: 基础UKF(虚线) vs 自适应UKF(实线)', 'FontSize', 10);

    h_truth = geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.8);
    h_r1b   = geoplot(gx1, lat_r1b, lon_r1b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 1.5);
    h_r1a   = geoplot(gx1, lat_r1a, lon_r1a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2.2);

    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    legend(gx1, [h_truth, h_r1b, h_r1a], {'真值', 'R1基础UKF', 'R1自适应UKF'}, ...
        'Location', 'northeast', 'FontSize', 7);

    rx1 = [min(lon_r1b(zoom_start:min(zoom_end,length(lon_r1b)))), ...
           max(lon_r1b(zoom_start:min(zoom_end,length(lon_r1b))))];
    ry1 = [min(lat_r1b(zoom_start:min(zoom_end,length(lat_r1b)))), ...
           max(lat_r1b(zoom_start:min(zoom_end,length(lat_r1b))))];
    geoplot(gx1, [ry1(1) ry1(1) ry1(2) ry1(2) ry1(1)], ...
                 [rx1(1) rx1(2) rx1(2) rx1(1) rx1(1)], ...
                 'w-', 'LineWidth', 1.2);

    % 子图 2: R2 全图对比
    nexttile(tlo, 2);
    try
        gx2 = geoaxes;
        gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes;
    end
    hold(gx2, 'on');
    title(gx2, 'R2: 基础UKF(虚线) vs 自适应UKF(实线)', 'FontSize', 10);

    geoplot(gx2, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.8);
    geoplot(gx2, lat_r2b, lon_r2b, '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 1.5);
    h_r2a_p = geoplot(gx2, lat_r2a, lon_r2a, '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 2.2);

    geoplot(gx2, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    legend(gx2, {'真值', 'R2基础UKF', 'R2自适应UKF'}, 'Location', 'northeast', 'FontSize', 7);

    valid_idx = zoom_start:min(zoom_end, length(lon_r2b));
    if ~isempty(valid_idx) && all(valid_idx <= length(lon_r2b))
        rx2 = [min(lon_r2b(valid_idx)), max(lon_r2b(valid_idx))];
        ry2 = [min(lat_r2b(valid_idx)), max(lat_r2b(valid_idx))];
        geoplot(gx2, [ry2(1) ry2(1) ry2(2) ry2(2) ry2(1)], ...
                     [rx2(1) rx2(2) rx2(2) rx2(1) rx2(1)], ...
                     'w-', 'LineWidth', 1.2);
    end

    % 子图 3: 拐弯区域放大
    nexttile(tlo, 3);
    try
        gx3 = geoaxes;
        gx3.Basemap = 'darkwater';
    catch
        gx3 = geoaxes;
    end
    hold(gx3, 'on');
    title(gx3, '拐弯区域放大对比', 'FontSize', 10);

    idx_zoom = zoom_start:min(zoom_end, size(true_track,1));
    geoplot(gx3, true_track(idx_zoom,2), true_track(idx_zoom,1), 'y--', 'LineWidth', 2.5);

    iz1 = zoom_start:min(zoom_end, length(lon_r1b));
    geoplot(gx3, lat_r1b(iz1), lon_r1b(iz1), '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 2);
    geoplot(gx3, lat_r1a(iz1), lon_r1a(iz1), '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 3);

    iz2 = zoom_start:min(zoom_end, length(lon_r2b));
    geoplot(gx3, lat_r2b(iz2), lon_r2b(iz2), '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 2);
    geoplot(gx3, lat_r2a(iz2), lon_r2a(iz2), '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 3);

    legend(gx3, {'真值','R1基础','R1自适应','R2基础','R2自适应'}, ...
        'Location', 'best', 'FontSize', 6);

    % 子图 4: R1 误差时间线
    ax4 = nexttile(tlo, 4);
    hold(ax4, 'on');
    t_plot = t(1:length(err_r1b));
    plot(ax4, t_plot, err_r1b, '--', 'Color', [0.3 0.5 0.9], 'LineWidth', 1.5);
    plot(ax4, t_plot, err_r1a, '-', 'Color', [0.0 0.1 0.6], 'LineWidth', 2);
    xline(ax4, t(turn_frame), 'k--', 'LineWidth', 0.8);
    xlabel(ax4, '时间 (s)'); ylabel(ax4, '位置误差 (km)');
    title(ax4, sprintf('R1误差: 基础RMSE=%.1fkm  自适应RMSE=%.1fkm', ...
        rms_tsc(err_r1b,'omitnan'), rms_tsc(err_r1a,'omitnan')), 'FontSize', 10);
    legend(ax4, {'基础UKF', '自适应UKF'}, 'Location', 'best', 'FontSize', 8);
    grid(ax4, 'on');

    % 子图 5: R2 误差时间线
    ax5 = nexttile(tlo, 5);
    hold(ax5, 'on');
    plot(ax5, t_plot, err_r2b, '--', 'Color', [1.0 0.5 0.4], 'LineWidth', 1.5);
    plot(ax5, t_plot, err_r2a, '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 2);
    xline(ax5, t(turn_frame), 'k--', 'LineWidth', 0.8);
    xlabel(ax5, '时间 (s)'); ylabel(ax5, '位置误差 (km)');
    title(ax5, sprintf('R2误差: 基础RMSE=%.1fkm  自适应RMSE=%.1fkm', ...
        rms_tsc(err_r2b,'omitnan'), rms_tsc(err_r2a,'omitnan')), 'FontSize', 10);
    legend(ax5, {'基础UKF', '自适应UKF'}, 'Location', 'best', 'FontSize', 8);
    grid(ax5, 'on');

    % 子图 6: RMSE 柱状图
    ax6 = nexttile(tlo, 6);
    rmse_vals = [rms_tsc(err_r1b,'omitnan'), rms_tsc(err_r1a,'omitnan'); ...
                 rms_tsc(err_r2b,'omitnan'), rms_tsc(err_r2a,'omitnan')];
    b = bar(ax6, rmse_vals);
    b(1).FaceColor = [0.5 0.5 0.5]; b(1).DisplayName = '基础UKF';
    b(2).FaceColor = [0.0 0.4 0.0]; b(2).DisplayName = '自适应UKF';
    set(ax6, 'XTickLabel', {'R1', 'R2'});
    ylabel(ax6, 'RMSE (km)');
    title(ax6, 'RMSE对比');
    legend(ax6, 'Location', 'best', 'FontSize', 8);
    grid(ax6, 'on');

    for i = 1:2
        imp = (1 - rmse_vals(i,2)/rmse_vals(i,1))*100;
        text(ax6, i-0.15, rmse_vals(i,1)+0.3, sprintf('%.1f', rmse_vals(i,1)), ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
        text(ax6, i+0.15, rmse_vals(i,2)+0.3, sprintf('%.1f', rmse_vals(i,2)), ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
        text(ax6, i, max(rmse_vals(i,:))+1.2, sprintf('%+.0f%%', imp), ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8 0 0], ...
            'HorizontalAlignment', 'center');
    end

    sgtitle(sprintf('拐弯目标单站对比: 基础UKF(虚线) vs 机动自适应UKF(实线)   拐角~113° Pd=%.0f%%', ...
        params.detection_probability*100));

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig2_single_compare.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig2_single_compare.png'));
    end
    fprintf('  单站对比图已保存: fig2_single_compare.png\n');
end

function [lats, lons] = extract_valid_ll_tsc(snapshots)
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

function d = err_at_frame_tsc(snap, t_lon, t_lat)
    d = NaN;
    if isempty(snap.trackList), return; end
    trk = snap.trackList{1};
    if trk.type == 7 || ~isfield(trk, 'lat') || isnan(trk.lat), return; end
    d = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
end

function v = rms_tsc(x, flag)
    if nargin < 2, flag = 'omitnan'; end
    x_valid = x(~isnan(x));
    if isempty(x_valid), v = NaN; return; end
    v = sqrt(mean(x_valid.^2));
end
