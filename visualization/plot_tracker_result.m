% =========================================================================
% plot_tracker_result.m
% 交互式可视化 —— 展示航迹碎片化 → 拼接的完整处理链
% =========================================================================
%
% 【功能概述】
%   在一张图上分层显示：真实航迹、两雷达量测碎片、滤波碎片、时间对齐碎片、
%   拼接后航迹。全部使用圆点+连线，仅以颜色区分。底部提供 toggle 按钮。
%
% 【图层颜色约定】
%   R1 系列 : 蓝色系（量测浅蓝 → 滤波蓝 → 对齐深蓝 → 拼接深蓝）
%   R2 系列 : 红色系（量测浅红 → 滤波红 → 对齐深红 → 拼接深红）
%   真实航迹 : 黑色
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

    % ---- 创建图窗 ----
    fig = figure('Position', [50, 50, 1400, 750], ...
                 'Name', '雷达航迹碎片化与拼接 — 交互式', ...
                 'NumberTitle', 'off');
    ax = geoaxes('Basemap', 'darkwater');
    ax.Position = [0.05, 0.13, 0.92, 0.83];
    hold(ax, 'on');

    % ---- 颜色定义 ----
    C_TRUE     = [0.10, 0.10, 0.10];
    C_R1_RAW   = [0.45, 0.65, 0.95];
    C_R2_RAW   = [0.95, 0.50, 0.50];
    C_R1_FILT  = [0.00, 0.25, 0.65];
    C_R2_FILT  = [0.70, 0.08, 0.08];
    C_R1_ALIGN = [0.00, 0.15, 0.50];
    C_R2_ALIGN = [0.50, 0.05, 0.05];
    C_R1_STCH  = [0.00, 0.10, 0.40];
    C_R2_STCH  = [0.40, 0.02, 0.02];

    % 统一线宽和标记大小
    LW = 1.0;
    MS = 4;

    % ---- 各图层绘图句柄收集器 ----
    L = {};

    % ================================================================
    % 图层 1: R1 量测碎片（浅蓝，圆点+连线）
    % ================================================================
    L{1} = draw_struct_array_segments(ax, r1_segments, C_R1_RAW, LW, MS, '-o');

    % ================================================================
    % 图层 2: R2 量测碎片（浅红，方块+连线）
    % ================================================================
    L{2} = draw_struct_array_segments(ax, r2_segments, C_R2_RAW, LW, MS, '-s');

    % ================================================================
    % 图层 3: R1 滤波碎片（蓝色，圆点+连线）
    % ================================================================
    L{3} = draw_cell_segments(ax, r1_segments_filt, C_R1_FILT, LW, MS, '-o');

    % ================================================================
    % 图层 4: R2 滤波碎片（红色，方块+连线）
    % ================================================================
    L{4} = draw_cell_segments(ax, r2_segments_filt, C_R2_FILT, LW, MS, '-s');

    % ================================================================
    % 图层 5: R1 时间对齐碎片（深蓝，圆点+连线）
    % ================================================================
    L{5} = draw_cell_segments(ax, r1_segments_aligned, C_R1_ALIGN, LW, MS, '-o');

    % ================================================================
    % 图层 6: R2 时间对齐碎片（深红，方块+连线）
    % ================================================================
    L{6} = draw_cell_segments(ax, r2_segments_aligned, C_R2_ALIGN, LW, MS, '-s');

    % ================================================================
    % 图层 7: R1 拼接航迹（深蓝，圆点+连线，遇空洞断线）
    % ================================================================
    L{7} = draw_stitched_track(ax, r1_stitched, C_R1_STCH, LW, MS, '-o');

    % ================================================================
    % 图层 8: R2 拼接航迹（深红，方块+连线，遇空洞断线）
    % ================================================================
    L{8} = draw_stitched_track(ax, r2_stitched, C_R2_STCH, LW, MS, '-s');

    % ================================================================
    % 图层 9: 真实航迹（黑色，圆点+虚线）
    % ================================================================
    h = geoplot(ax, true_track(:,2), true_track(:,1), '--o', ...
                'Color', C_TRUE, 'LineWidth', LW, 'MarkerSize', MS);
    L{9} = h;

    % ================================================================
    % 雷达站标记
    % ================================================================
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

    % ---- 自动缩放 ----
    all_lats = [true_track(:,2); radar1.lat; radar2.lat];
    all_lons = [true_track(:,1); radar1.lon; radar2.lon];
    lat_pad = max(diff([min(all_lats), max(all_lats)]) * 0.15, 0.5);
    lon_pad = max(diff([min(all_lons), max(all_lons)]) * 0.15, 0.5);
    geolimits(ax, [min(all_lats)-lat_pad, max(all_lats)+lat_pad], ...
                   [min(all_lons)-lon_pad, max(all_lons)+lon_pad]);

    % ---- 标题 ----
    title(ax, sprintf(['短波外辐射源双雷达仿真 — 航迹碎片化与拼接\n' ...
           'M/N=%d/%d  K_{loss}=%d  Pd=%d%%  |  ' ...
           '碎片: R1×%d段 R2×%d段  →  拼接后: R1=%d点 R2=%d点'], ...
           params.tracker_M, params.tracker_N, params.tracker_K_loss, ...
           round(params.detection_probability*100), ...
           n_r1_seg, n_r2_seg, n1_stitch, n2_stitch), ...
           'FontSize', 13, 'FontWeight', 'bold');

    % ================================================================
    % 控制面板
    % ================================================================
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
            'Callback', @(src,~) toggle_layer(src, L{k}));
        x0 = x0 + btn_w + gap_x;
    end

    % ---- 第2行按钮 ----
    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '全部显示', 'Position', [12, row2_y, 80, 22], ...
        'FontSize', 8, 'FontWeight', 'bold', ...
        'Callback', @(~,~) set_all_layers(L, all_buttons, 'on'));

    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '全部隐藏', 'Position', [100, row2_y, 80, 22], ...
        'FontSize', 8, ...
        'Callback', @(~,~) set_all_layers(L, all_buttons, 'off'));

    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '仅看拼接+真实', 'Position', [188, row2_y, 110, 22], ...
        'FontSize', 8, ...
        'Callback', @(~,~) show_only(L, all_buttons, [7, 8, 9]));

    % ---- 导出 ----
    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'tracker_fragments_and_stitching.png'), ...
                       'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'tracker_fragments_and_stitching.png'));
    end
    fprintf('  可视化已保存到 %s\n', fullfile(out_dir, 'tracker_fragments_and_stitching.png'));
end

% =========================================================================
% 绘制 struct 数组形式的碎片段（如原始量测 r1_segments）
% =========================================================================
function handles = draw_struct_array_segments(ax, segments, color, lw, ms, style)
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

% =========================================================================
% 绘制 cell 数组形式的碎片段（如滤波/对齐后的片段）
% =========================================================================
function handles = draw_cell_segments(ax, segments, color, lw, ms, style)
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

% =========================================================================
% 绘制拼接航迹（遇空洞断线，标记+连线）
% =========================================================================
function handles = draw_stitched_track(ax, stitched, color, lw, ms, style)
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

% =========================================================================
% 辅助函数: 切换图层
% =========================================================================
function toggle_layer(src, handles)
    if src.Value
        state = 'on';
    else
        state = 'off';
    end
    for k = 1:length(handles)
        set(handles(k), 'Visible', state);
    end
end

% =========================================================================
% 辅助函数: 全部显示/隐藏
% =========================================================================
function set_all_layers(layers, buttons, state)
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

% =========================================================================
% 辅助函数: 仅显示指定图层
% =========================================================================
function show_only(layers, buttons, show_idx)
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
