% =========================================================================
% plot_fragment_study_dashboard.m — 碎片研究仪表盘可视化
% =========================================================================
% 【功能概述】
%   为每个目标生成一个 2x2 的四宫格仪表盘，展示：
%     1. 空间轨迹构造图：各雷达段的拼接、间隙、重叠
%     2. 时间线可用性图：各段的时间覆盖和重叠区域
%     3. 片段兼容性图：片段间的重叠/间隙关系图
%     4. 指标汇总：覆盖范围、延长帧数、RMSE 等
%
% 【输入】
%   view   - cell 数组，每个元素包含一个目标的碎片分析数据
%   config - 配置结构体，包含 output_root、figure_visible、save_figures、figure_dpi
%
% 【输出】
%   paths  - 生成的图片文件路径列表
% =========================================================================

function paths = plot_fragment_study_dashboard(view, config)
    % 主入口：遍历每个目标视图，生成 2x2 四宫格仪表盘
    % view 是 cell 数组，每个元素包含一个目标的碎片分析数据
    % config 包含输出目录、DPI、是否保存图等配置
    paths = {};  % 累积生成的图片路径列表
    % 如果输出目录不存在则创建
    if ~exist(config.output_root, 'dir'), mkdir(config.output_root); end
    % 遍历每个目标视图
    for q = 1:numel(view)
        v = view(q);  % 取出第 q 个目标的数据
        % 创建白色背景的图窗
        fig = figure('Color', 'w', 'Visible', config.figure_visible, ...
            'Name', sprintf('Fragment study - target %d', v.truth_idx));
        % 创建 2x2 紧凑布局的子图区
        tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        % 四个子图：空间轨迹、时间线、兼容性图、指标汇总
        plot_space(v);    % 左上：空间轨迹构造图
        plot_timeline(v); % 右上：可用性时间线
        plot_graph(v);    % 左下：片段兼容性图
        plot_metrics(v);  % 右下：指标汇总
        % 设置总标题：显示目标和分组信息
        title_text = sprintf('Target %d / Group %d: three tracklets to one fused track', ...
            v.truth_idx, v.group.group_id);
        sgtitle(title_text, 'Interpreter', 'none', 'FontWeight', 'bold');
        drawnow;  % 强制刷新图形
        % 如果配置了保存图片，则导出到输出目录
        if config.save_figures
            path = fullfile(config.output_root, sprintf('target_%02d_dashboard.png', q));
            exportgraphics(fig, path, 'Resolution', config.figure_dpi);
            paths{end+1} = path; %#ok<AGROW>
        end
    end
end

% =========================================================================
% plot_space — 空间轨迹构造图（左上子图）
% =========================================================================
% 绘制目标在二维平面上的轨迹构造过程：
%   - 灰色虚线：真值航迹
%   - 蓝色：R1 航迹（前后两段）
%   - 橙色：R2 航迹（中间段）
%   - 绿色：融合后的完整航迹
%   - 红色虚线：R1 前后的间隙
%   - 紫色圆点：重叠区域
function plot_space(v)
    nexttile; hold on; grid on; box on;  % 创建第一个子图
    truth = v.truth;  % 真值航迹数据
    % 绘制真值航迹（灰色虚线方块）
    plot(truth.lon, truth.lat, '--', 'Color', [.35 .35 .35], 'LineWidth', 1.1, 'DisplayName', 'Truth');
    % 绘制 R1 航迹的前段（蓝色圆点）
    plot_segment(v.r1_before, [.10 .38 .85], 'o', 'R1 before');
    % 绘制 R1 航迹的后段（蓝色方块）
    plot_segment(v.r1_after, [.05 .18 .55], 's', 'R1 after');
    % 绘制 R2 航迹的中间段（橙色三角）
    plot_segment(v.r2_middle, [.90 .40 .08], '^', 'R2 middle');
    % 绘制融合后的完整航迹（绿色实线）
    plot(v.fused.lon, v.fused.lat, '-', 'Color', [.05 .60 .25], 'LineWidth', 2.4, 'DisplayName', 'Fused group');
    % 绘制 R1 前后的间隙（红色虚线连接）
    plot_gap(v.r1_before, v.r1_after, v.plan);
    % 绘制两个重叠区域的紫色标记
    plot_overlap(v.r1_before, v.r2_middle, v.overlap1, 'M1');
    plot_overlap(v.r1_after, v.r2_middle, v.overlap2, 'M2');
    xlabel('Longitude (deg)'); ylabel('Latitude (deg)');  % 坐标轴标签
    legend('Location', 'best'); title('Spatial track construction');  % 图例和标题
end

% =========================================================================
% plot_segment — 绘制航迹段
% =========================================================================
% 将航迹段的经纬度坐标绘制为带标记的线段
% 为减少视觉混乱，只在等间距的 12 个点上放置标记
function plot_segment(seg, color, marker, name)
    if isempty(seg), return; end  % 空段直接返回
    % 绘制完整线段（隐藏句柄，不参与图例）
    plot(seg.lons, seg.lats, '-', 'Color', color, 'LineWidth', 1.4, 'HandleVisibility', 'off');
    % 在等间距的 12 个位置上放置标记点
    idx = unique(round(linspace(1, numel(seg.frames), min(12, numel(seg.frames)))));
    % 绘制标记点（显示图例名称）
    plot(seg.lons(idx), seg.lats(idx), marker, 'Color', color, 'MarkerSize', 5, ...
        'DisplayName', name);
end

% =========================================================================
% plot_gap — 绘制间隙标记
% =========================================================================
% 在 R1 前后两段之间绘制红色虚线连接，标注间隙帧范围
function plot_gap(before, after, plan)
    if isempty(before) || isempty(after), return; end  % 任一段为空则跳过
    % 绘制连接前后段端点的红色虚线（隐藏句柄）
    plot([before.lons(end), after.lons(1)], [before.lats(end), after.lats(1)], ':', ...
        'Color', [.85 .10 .10], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    % 在 R1 前段末端标注间隙帧范围
    text(before.lons(end), before.lats(end), sprintf('  R1 gap [%d,%d]', ...
        plan.r1_gap_start, plan.r1_gap_end), 'Color', [.75 .05 .05], 'FontSize', 9);
end

% =========================================================================
% plot_overlap — 绘制重叠区域标记
% =========================================================================
% 在重叠帧对应的位置绘制紫色圆点，标注重叠标签
function plot_overlap(a, b, common, label)
    if isempty(common), return; end  % 无重叠则跳过
    % 在航迹 a 中找到重叠帧对应的索引
    ia = arrayfun(@(k) find(a.frames == k, 1), common);
    % 绘制紫色圆点标记重叠区域（隐藏句柄）
    plot(a.lons(ia), a.lats(ia), 'o', 'Color', [.55 .15 .70], ...
        'MarkerSize', 9, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    % 标注重叠标签
    text(a.lons(ia(1)), a.lats(ia(1)), ['  ', label], 'Color', [.45 .10 .60], 'FontWeight', 'bold');
end

% =========================================================================
% plot_timeline — 时间线可用性图（右上子图）
% =========================================================================
% 绘制各航迹段的时间覆盖范围和重叠区域
% Y 轴从下到上：Fused、R1 after、R2 middle、R1 before
function plot_timeline(v)
    nexttile; hold on; box on; grid on;  % 创建第二个子图
    % 计算总帧数范围
    all_frames = 1:max([v.plan.r1_gap_end, v.group.end_frame]);
    % 绘制各航迹段的时间线（不同 Y 层级，不同颜色）
    plot_lane(v.r1_before.frames, 4, [.10 .38 .85]);  % R1 before (Y=4)
    plot_lane(v.r2_middle.frames, 3, [.90 .40 .08]);  % R2 middle (Y=3)
    plot_lane(v.r1_after.frames, 2, [.05 .18 .55]);   % R1 after (Y=2)
    plot_lane(v.fused.frames, 1, [.05 .60 .25]);       % Fused (Y=1)
    % 标注 R1 间隙区域（红色半透明矩形）
    area([v.plan.r1_gap_start, v.plan.r1_gap_end], [5 5], 'FaceColor', [.95 .65 .65], ...
        'FaceAlpha', .25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    % 标注重叠区域（紫色半透明矩形）
    for common = {v.overlap1, v.overlap2}
        if ~isempty(common{1})
            area([common{1}(1), common{1}(end)], [5 5], 'FaceColor', [.75 .60 .90], ...
                'FaceAlpha', .18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
    xlim([min(all_frames), max(all_frames)]); ylim([.5 4.5]);  % 设置坐标轴范围
    % Y 轴标签从下到上：Fused、R1 after、R2 middle、R1 before
    yticks(1:4); yticklabels({'Fused', 'R1 after', 'R2 middle', 'R1 before'});
    xlabel('Frame'); title('Availability and overlap timeline');  % X 轴标签和标题
end

% =========================================================================
% plot_lane — 绘制时间线航迹段
% =========================================================================
% 在指定的 Y 层级上绘制点状时间线
function plot_lane(frames, y, color)
    if isempty(frames), return; end  % 空帧列表跳过
    % 绘制水平点线：每个帧号对应一个点
    plot(frames, y * ones(size(frames)), '.', 'Color', color, 'MarkerSize', 10, 'HandleVisibility', 'off');
end

% =========================================================================
% plot_graph — 片段兼容性图（左下子图）
% =========================================================================
% 绘制片段兼容性图：节点表示航迹段，边表示重叠或间隙关系
% 节点按雷达来源着色（R1=蓝色, R2=橙色）
% 实线边=重叠关系，虚线边=间隙关系
function plot_graph(v)
    nexttile; hold on; axis off;  % 创建第四个子图，关闭坐标轴
    segs = v.group_segments;  % 片段列表
    % 计算每个片段的中心帧号（X 轴坐标）
    xs = arrayfun(@(s) mean([s.start_frame, s.end_frame]), segs);
    ys = [1, 0, 1];  % Y 轴交替排列，避免边交叉
    % 绘制片段之间的兼容性边
    for e = 1:numel(v.group_edges)
        edge = v.group_edges(e);  % 取出第 e 条边
        % 找到边的两个端点对应的片段索引
        ia = find([v.group.segment_indices] == edge.a, 1);
        ib = find([v.group.segment_indices] == edge.b, 1);
        if isempty(ia) || isempty(ib), continue; end  % 找不到片段则跳过
        % 根据边类型设置颜色和样式
        if strcmp(edge.edge_type, 'overlap')
            line_color = [.55 .15 .70]; style = '-'; width = 2.4;  % 重叠：紫色实线
            common = intersect(segs(ia).frames, segs(ib).frames);
            label = sprintf('overlap %d f / %.2f', numel(common), edge.score);
        else
            line_color = [.45 .45 .45]; style = '--'; width = 1.5;  % 间隙：灰色虚线
            label = sprintf('gap %d f / %.2f', edge.gap_frames, edge.score);
        end
        % 绘制边
        plot([xs(ia), xs(ib)], [ys(ia), ys(ib)], style, 'Color', line_color, ...
            'LineWidth', width, 'HandleVisibility', 'off');
        % 在边中点标注信息
        text(mean([xs(ia), xs(ib)]), mean([ys(ia), ys(ib)]) + .08, label, ...
            'Color', line_color, 'FontSize', 8, 'HorizontalAlignment', 'center');
    end
    % 绘制片段节点
    for i = 1:numel(segs)
        if segs(i).radar_id == 1, color = [.10 .38 .85]; else, color = [.90 .40 .08]; end
        % 绘制节点圆点（带黑色边框）
        plot(xs(i), ys(i), 'o', 'MarkerSize', 16, 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', 'k');
        % 标注节点信息
        text(xs(i), ys(i) - .14, sprintf('R%d #%d\n[%d,%d]', segs(i).radar_id, ...
            segs(i).track_id, segs(i).start_frame, segs(i).end_frame), ...
            'HorizontalAlignment', 'center', 'FontSize', 9);
    end
    xlim([min(xs)-10, max(xs)+10]); ylim([-.45, 1.45]);  % 设置坐标轴范围
    title('Segment compatibility graph'); xlabel('Segment midpoint frame');  % 标题和标签
end

% =========================================================================
% plot_metrics — 指标汇总（右下子图）
% =========================================================================
% 显示关键指标：片段数、覆盖范围、RMSE 等
function plot_metrics(v)
    nexttile; axis off;  % 创建第五个子图，关闭坐标轴
    e = v.evaluation;  % 提取评估数据
    % 构建指标列表：每行一个指标
    lines = {sprintf('Target %d / Group %d', v.truth_idx, v.group.group_id), ...  % 目标/分组信息
        sprintf('Segments: %d  ->  1 group', numel(v.group_segments)), ...         % 片段合并
        sprintf('R1 before: %d frames', numel(v.r1_before.frames)), ...            % R1 前段帧数
        sprintf('R2 middle: %d frames', numel(v.r2_middle.frames)), ...            % R2 中段帧数
        sprintf('R1 after:  %d frames', numel(v.r1_after.frames)), ...             % R1 后段帧数
        sprintf('Fused coverage: %d / %d (%.1f%%)', e.coverage_frames, e.truth_frames, 100*e.coverage_ratio), ...  % 覆盖率
        sprintf('Extension: +%d frames', e.extension_frames), ...                   % 延长帧数
        sprintf('RMSE: %.2f km', e.rmse_km), ...                                   % RMSE
        sprintf('Overlap M1/M2: %d / %d frames', numel(v.overlap1), numel(v.overlap2))};  % 重叠帧数
    % 在左上角显示指标列表
    text(.05, .92, lines, 'VerticalAlignment', 'top', 'FontSize', 11, 'Interpreter', 'none');
    title('Metrics');  % 子图标题
end
