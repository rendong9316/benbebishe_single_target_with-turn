% =========================================================================
% plot_tracker_result.m
% =========================================================================
%
% 【功能概述】
%   绘制航迹碎片化与拼接的交互式可视化图。在一张 darkwater 底图的
%   地理坐标平面上，按处理阶段分层展示：雷达原始量测碎片、滤波后
%   碎片、时间对齐后碎片、拼接后航迹，以及真实航迹作为参考基准。
%   底部提供 toggle 按钮控制面板，可单独切换每个图层的显隐状态。
%
% 【数学原理】
%   1. 航迹碎片化 (Track Fragmentation)：
%      由于检测概率 Pd < 1、虚警概率 Pfa > 0、以及遮挡等效应，
%      UKF 跟踪器无法在所有帧都获得有效的量测关联。当连续多帧
%      漏检时，跟踪器可能丢失目标并重新起始，产生不连贯的航迹片段
%      （即碎片）。这是 M/N 逻辑跟踪器在 Pd<1 条件下的固有行为。
%   2. M/N 逻辑跟踪器：
%      - 初始化：在连续 M 帧中有 N 帧检测到目标才建立航迹(M ≤ N)
%      - 保持：当连续 K_loss 帧无关联检测时，宣告航迹终止
%      - 参数 (M/N, K_loss) 直接决定了碎片化的程度
%   3. 时间对齐 (Time Alignment)：
%      R1 和 R2 可能在不同时间网格上运行。为便于后续拼接和融合，
%      需要将两个雷达的航迹插值到统一的时间基准上。
%   4. 航迹拼接 (Track Stitching)：
%      对时间"接近"的碎片段进行关联和拼接，恢复目标的完整轨迹。
%   5. 颜色方案：
%      R1 系列：浅蓝 → 蓝色 → 深蓝 → 更深蓝（随处理阶段加深）
%      R2 系列：浅红 → 红色 → 深红 → 更深红
%      真实航迹：黑色
%
% 【输入参数】
%   true_track          - Nx2 矩阵，真值航迹 [lon, lat]
%   r1_segments         - R1 原始量测碎片（struct数组形式）
%   r2_segments         - R2 原始量测碎片（struct数组形式）
%   r1_segments_filt    - R1 滤波后碎片（cell数组形式）
%   r2_segments_filt    - R2 滤波后碎片（cell数组形式）
%   r1_segments_aligned - R1 时间对齐后碎片
%   r2_segments_aligned - R2 时间对齐后碎片
%   r1_stitched         - R1 拼接后航迹（cell数组，空洞为[]）
%   r2_stitched         - R2 拼接后航迹（cell数组，空洞为[]）
%   radar1 / radar2     - 雷达站信息结构体，含 .lat, .lon 字段
%   params              - 仿真参数字段结构体
%   unified_time        - 统一时间向量
%   out_dir             - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       tracker_fragments_and_stitching.png  - 航迹碎片化与拼接可视化
%
% 【调用关系】
%   被调用: 主仿真脚本
%   调用:   draw_struct_array_segments()  (本文件内部)
%           draw_cell_segments()          (本文件内部)
%           draw_stitched_track()         (本文件内部)
%           toggle_layer()               (本文件内部)
%           set_all_layers()             (本文件内部)
%           show_only()                  (本文件内部)
%
% =========================================================================

function fig = plot_tracker_result(true_track, ...
    r1_segments, r2_segments, ...
    r1_segments_filt, r2_segments_filt, ...
    r1_segments_aligned, r2_segments_aligned, ...
    r1_stitched, r2_stitched, ...
    radar1, radar2, params, unified_time, out_dir)

    % 统计碎片和拼接后的有效点数
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

    % ---- 颜色定义：按处理阶段从浅到深 ----
    C_TRUE     = [0.10, 0.10, 0.10];  % 真值：黑色
    C_R1_RAW   = [0.45, 0.65, 0.95];  % R1 量测碎片：浅蓝
    C_R2_RAW   = [0.95, 0.50, 0.50];  % R2 量测碎片：浅红
    C_R1_FILT  = [0.00, 0.25, 0.65];  % R1 滤波碎片：蓝色
    C_R2_FILT  = [0.70, 0.08, 0.08];  % R2 滤波碎片：红色
    C_R1_ALIGN = [0.00, 0.15, 0.50];  % R1 对齐碎片：深蓝
    C_R2_ALIGN = [0.50, 0.05, 0.05];  % R2 对齐碎片：深红
    C_R1_STCH  = [0.00, 0.10, 0.40];  % R1 拼接航迹：最深蓝
    C_R2_STCH  = [0.40, 0.02, 0.02];  % R2 拼接航迹：最深红

    % 统一线宽和标记大小：所有图层使用相同尺寸以便对比
    LW = 1.0;
    MS = 4;

    % ---- 各图层绘图句柄收集器 L (cell数组) ----
    L = {};

    % ================================================================
    % 图层 1: R1 量测碎片（浅蓝，圆点+连线）
    % 表示 UKF 滤波之前，从检测器直接输出的、经校准的点迹片段
    % ================================================================
    L{1} = draw_struct_array_segments(ax, r1_segments, C_R1_RAW, LW, MS, '-o');

    % ================================================================
    % 图层 2: R2 量测碎片（浅红，方块+连线）
    % 使用方块(s)标记区分于 R1 的圆点(o)
    % ================================================================
    L{2} = draw_struct_array_segments(ax, r2_segments, C_R2_RAW, LW, MS, '-s');

    % ================================================================
    % 图层 3: R1 滤波碎片（蓝色，圆点+连线）
    % UKF 滤波平滑后的位置估计，噪声比量测碎片小
    % ================================================================
    L{3} = draw_cell_segments(ax, r1_segments_filt, C_R1_FILT, LW, MS, '-o');

    % ================================================================
    % 图层 4: R2 滤波碎片（红色，方块+连线）
    % ================================================================
    L{4} = draw_cell_segments(ax, r2_segments_filt, C_R2_FILT, LW, MS, '-s');

    % ================================================================
    % 图层 5: R1 时间对齐碎片（深蓝，圆点+连线）
    % 时间对齐将 R1 的碎片插值到统一时间网格上
    % ================================================================
    L{5} = draw_cell_segments(ax, r1_segments_aligned, C_R1_ALIGN, LW, MS, '-o');

    % ================================================================
    % 图层 6: R2 时间对齐碎片（深红，方块+连线）
    % ================================================================
    L{6} = draw_cell_segments(ax, r2_segments_aligned, C_R2_ALIGN, LW, MS, '-s');

    % ================================================================
    % 图层 7: R1 拼接航迹（深蓝，圆点+连线，遇空洞断线）
    % 拼接算法将时间上连续的碎片段合并为完整航迹
    % ================================================================
    L{7} = draw_stitched_track(ax, r1_stitched, C_R1_STCH, LW, MS, '-o');

    % ================================================================
    % 图层 8: R2 拼接航迹（深红，方块+连线，遇空洞断线）
    % ================================================================
    L{8} = draw_stitched_track(ax, r2_stitched, C_R2_STCH, LW, MS, '-s');

    % ================================================================
    % 图层 9: 真实航迹（黑色，圆点+虚线）— 最终参考基准
    % ================================================================
    h = geoplot(ax, true_track(:,2), true_track(:,1), '--o', ...
                'Color', C_TRUE, 'LineWidth', LW, 'MarkerSize', MS);
    L{9} = h;

    % ================================================================
    % 雷达站标记：星形(scatter) + 文字标签
    % ================================================================
    geoscatter(ax, radar1.lat, radar1.lon, 220, '*', ...
               'MarkerEdgeColor', '#B71C1C', 'MarkerFaceColor', '#B71C1C', ...
               'LineWidth', 0.5);
    text(ax, radar1.lon + 0.2, radar1.lat - 0.15, 'R1', ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', '#B71C1C', ...
         'BackgroundColor', [1 1 1 0.6]);  % 半透明白色背景提高可读性
    geoscatter(ax, radar2.lat, radar2.lon, 220, '*', ...
               'MarkerEdgeColor', '#1B5E20', 'MarkerFaceColor', '#1B5E20', ...
               'LineWidth', 0.5);
    text(ax, radar2.lon + 0.2, radar2.lat - 0.15, 'R2', ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', '#1B5E20', ...
         'BackgroundColor', [1 1 1 0.6]);

    % ---- 自动缩放地图范围：覆盖所有航迹点 + 15% 边距 ----
    all_lats = [true_track(:,2); radar1.lat; radar2.lat];
    all_lons = [true_track(:,1); radar1.lon; radar2.lon];
    lat_pad = max(diff([min(all_lats), max(all_lats)]) * 0.15, 0.5);
    lon_pad = max(diff([min(all_lons), max(all_lons)]) * 0.15, 0.5);
    geolimits(ax, [min(all_lats)-lat_pad, max(all_lats)+lat_pad], ...
                   [min(all_lons)-lon_pad, max(all_lons)+lon_pad]);

    % ---- 标题：含跟踪器参数和碎片统计 ----
    title(ax, sprintf(['短波外辐射源双雷达仿真 — 航迹碎片化与拼接\n' ...
           'M/N=%d/%d  K_{loss}=%d  Pd=%d%%  |  ' ...
           '碎片: R1×%d段 R2×%d段  →  拼接后: R1=%d点 R2=%d点'], ...
           params.tracker_M, params.tracker_N, params.tracker_K_loss, ...
           round(params.detection_probability*100), ...
           n_r1_seg, n_r2_seg, n1_stitch, n2_stitch), ...
           'FontSize', 13, 'FontWeight', 'bold');

    % ================================================================
    % 底部控制面板 (uipanel)：9 个 toggle 按钮控制 9 个图层
    % ================================================================
    panel = uipanel(fig, 'Position', [0.01, 0.005, 0.98, 0.11], ...
                    'Title', '图层控制（单击按钮切换显示/隐藏）', ...
                    'FontSize', 9, 'FontWeight', 'bold');

    btn_w = 85; btn_h = 24; gap_x = 8;
    row1_y = 36; row2_y = 7;

    % 图层按钮标签
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

    % ---- 第2行按钮：全部显示、全部隐藏、仅看拼接+真实 ----
    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '全部显示', 'Position', [12, row2_y, 80, 22], ...
        'FontSize', 8, 'FontWeight', 'bold', ...
        'Callback', @(~,~) set_all_layers(L, all_buttons, 'on'));

    uicontrol(panel, 'Style', 'pushbutton', ...
        'String', '全部隐藏', 'Position', [100, row2_y, 80, 22], ...
        'FontSize', 8, ...
        'Callback', @(~,~) set_all_layers(L, all_buttons, 'off'));

    % "仅看拼接+真实"按钮：快速聚焦最终结果
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
% draw_struct_array_segments - 绘制 struct 数组形式的碎片段
%
% 【说明】segments 是一个元胞数组，每个元素 seg 是 struct 数组，
%         每个 struct 含 .lat 和 .lon 字段。逐一取出画为地理线。
%         常用于绘制原始量测碎片（每个碎片是一个struct数组）。
% =========================================================================
function handles = draw_struct_array_segments(ax, segments, color, lw, ms, style)
    handles = gobjects(0);  % 初始化为空的图形对象数组
    for s = 1:length(segments)
        seg = segments{s};
        lats = [seg.lat]; lons = [seg.lon];
        if length(lats) > 1
            % 多个点：画连线+标记
            h = geoplot(ax, lats, lons, style, 'Color', color, ...
                        'LineWidth', lw, 'MarkerSize', ms);
            handles(end+1) = h;
        elseif length(lats) == 1
            % 单个点：仅画标记（style(2)取标记字符，如 '-o' 中的 'o'）
            h = geoplot(ax, lats, lons, style(2), 'Color', color, ...
                        'MarkerSize', ms);
            handles(end+1) = h;
        end
    end
end

% =========================================================================
% draw_cell_segments - 绘制 cell 数组形式的碎片段
%
% 【说明】与 draw_struct_array_segments 的区别在于数据结构：
%         segments 是元胞数组，每个元素 fc 本身是元胞数组，
%         fc{i} 是单个 struct(含 .lat/.lon)，需逐个提取。
%         常用于绘制滤波后和对齐后的碎片。
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
% draw_stitched_track - 绘制拼接航迹（遇空洞自动断线）
%
% 【说明】stitched 是一个 cell 数组，长度为总帧数。有效帧包含 struct
%         （含 .lat/.lon），无效帧为空 []。绘制时每当遇到空洞，
%         就结束当前线段并开始新线段，形成"断线"效果，直观展示
%         拼接算法保留的空洞。
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
            % 从空洞进入有效段：标记新片段起点
            seg_start = i;
            in_gap = false;
        elseif ~is_valid && ~in_gap
            % 从有效段进入空洞：输出当前片段
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

    % 处理末尾的最后一个有效段
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
% toggle_layer - 切换单个图层的显隐状态
%   当 toggle 按钮按下时显示，弹起时隐藏，即 Value=true → 'on'
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
% set_all_layers - 全部显示或隐藏所有图层，并同步按钮状态
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
% show_only - 仅显示指定索引的图层，隐藏其余所有图层
%   用于快速聚焦到拼接航迹和真实航迹的对比
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
