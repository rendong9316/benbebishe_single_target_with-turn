function paths = plot_fragment_study_dashboard(view, config)
% PLOT_FRAGMENT_STUDY_DASHBOARD 显示碎片、匹配边和凝聚结果。
paths = {};
if ~exist(config.output_root, 'dir'), mkdir(config.output_root); end
for q = 1:numel(view)
    v = view(q);
    fig = figure('Color', 'w', 'Visible', config.figure_visible, ...
        'Name', sprintf('Fragment study - target %d', v.truth_idx));
    tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    plot_space(v);
    plot_timeline(v);
    plot_graph(v);
    plot_metrics(v);
    title_text = sprintf('Target %d / Group %d: three tracklets to one fused track', ...
        v.truth_idx, v.group.group_id);
    sgtitle(title_text, 'Interpreter', 'none', 'FontWeight', 'bold');
    drawnow;
    if config.save_figures
        path = fullfile(config.output_root, sprintf('target_%02d_dashboard.png', q));
        exportgraphics(fig, path, 'Resolution', config.figure_dpi);
        paths{end+1} = path; %#ok<AGROW>
    end
end
end

function plot_space(v)
nexttile; hold on; grid on; box on;
truth = v.truth;
plot(truth.lon, truth.lat, '--', 'Color', [.35 .35 .35], 'LineWidth', 1.1, 'DisplayName', 'Truth');
plot_segment(v.r1_before, [.10 .38 .85], 'o', 'R1 before');
plot_segment(v.r1_after, [.05 .18 .55], 's', 'R1 after');
plot_segment(v.r2_middle, [.90 .40 .08], '^', 'R2 middle');
plot(v.fused.lon, v.fused.lat, '-', 'Color', [.05 .60 .25], 'LineWidth', 2.4, 'DisplayName', 'Fused group');
plot_gap(v.r1_before, v.r1_after, v.plan);
plot_overlap(v.r1_before, v.r2_middle, v.overlap1, 'M1');
plot_overlap(v.r1_after, v.r2_middle, v.overlap2, 'M2');
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
legend('Location', 'best'); title('Spatial track construction');
end

function plot_segment(seg, color, marker, name)
if isempty(seg), return; end
plot(seg.lons, seg.lats, '-', 'Color', color, 'LineWidth', 1.4, 'HandleVisibility', 'off');
idx = unique(round(linspace(1, numel(seg.frames), min(12, numel(seg.frames)))));
plot(seg.lons(idx), seg.lats(idx), marker, 'Color', color, 'MarkerSize', 5, ...
    'DisplayName', name);
end

function plot_gap(before, after, plan)
if isempty(before) || isempty(after), return; end
plot([before.lons(end), after.lons(1)], [before.lats(end), after.lats(1)], ':', ...
    'Color', [.85 .10 .10], 'LineWidth', 1.8, 'HandleVisibility', 'off');
text(before.lons(end), before.lats(end), sprintf('  R1 gap [%d,%d]', ...
    plan.r1_gap_start, plan.r1_gap_end), 'Color', [.75 .05 .05], 'FontSize', 9);
end

function plot_overlap(a, b, common, label)
if isempty(common), return; end
ia = arrayfun(@(k) find(a.frames == k, 1), common);
plot(a.lons(ia), a.lats(ia), 'o', 'Color', [.55 .15 .70], ...
    'MarkerSize', 9, 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(a.lons(ia(1)), a.lats(ia(1)), ['  ', label], 'Color', [.45 .10 .60], 'FontWeight', 'bold');
end

function plot_timeline(v)
nexttile; hold on; box on; grid on;
all_frames = 1:max([v.plan.r1_gap_end, v.group.end_frame]);
plot_lane(v.r1_before.frames, 4, [.10 .38 .85]);
plot_lane(v.r2_middle.frames, 3, [.90 .40 .08]);
plot_lane(v.r1_after.frames, 2, [.05 .18 .55]);
fused_frames = v.fused.frames;
plot_lane(fused_frames, 1, [.05 .60 .25]);
area([v.plan.r1_gap_start, v.plan.r1_gap_end], [5 5], 'FaceColor', [.95 .65 .65], ...
    'FaceAlpha', .25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
for common = {v.overlap1, v.overlap2}
    if ~isempty(common{1})
        area([common{1}(1), common{1}(end)], [5 5], 'FaceColor', [.75 .60 .90], ...
            'FaceAlpha', .18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
end
xlim([min(all_frames), max(all_frames)]); ylim([.5 4.5]);
yticks(1:4); yticklabels({'Fused', 'R1 after', 'R2 middle', 'R1 before'});
xlabel('Frame'); title('Availability and overlap timeline');
end

function plot_lane(frames, y, color)
if isempty(frames), return; end
plot(frames, y * ones(size(frames)), '.', 'Color', color, 'MarkerSize', 10, 'HandleVisibility', 'off');
end

function plot_graph(v)
nexttile; hold on; axis off;
segs = v.group_segments;
xs = arrayfun(@(s) mean([s.start_frame, s.end_frame]), segs);
ys = [1, 0, 1];
for e = 1:numel(v.group_edges)
    edge = v.group_edges(e);
    ia = find([v.group.segment_indices] == edge.a, 1);
    ib = find([v.group.segment_indices] == edge.b, 1);
    if isempty(ia) || isempty(ib), continue; end
    if strcmp(edge.edge_type, 'overlap')
        line_color = [.55 .15 .70]; style = '-'; width = 2.4;
        common = intersect(segs(ia).frames, segs(ib).frames);
        label = sprintf('overlap %d f / %.2f', numel(common), edge.score);
    else
        line_color = [.45 .45 .45]; style = '--'; width = 1.5;
        label = sprintf('gap %d f / %.2f', edge.gap_frames, edge.score);
    end
    plot([xs(ia), xs(ib)], [ys(ia), ys(ib)], style, 'Color', line_color, ...
        'LineWidth', width, 'HandleVisibility', 'off');
    text(mean([xs(ia), xs(ib)]), mean([ys(ia), ys(ib)]) + .08, label, ...
        'Color', line_color, 'FontSize', 8, 'HorizontalAlignment', 'center');
end
for i = 1:numel(segs)
    if segs(i).radar_id == 1, color = [.10 .38 .85]; else, color = [.90 .40 .08]; end
    plot(xs(i), ys(i), 'o', 'MarkerSize', 16, 'MarkerFaceColor', color, ...
        'MarkerEdgeColor', 'k');
    text(xs(i), ys(i) - .14, sprintf('R%d #%d\n[%d,%d]', segs(i).radar_id, ...
        segs(i).track_id, segs(i).start_frame, segs(i).end_frame), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end
xlim([min(xs)-10, max(xs)+10]); ylim([-.45, 1.45]);
title('Segment compatibility graph'); xlabel('Segment midpoint frame');
end

function plot_metrics(v)
nexttile; axis off;
e = v.evaluation;
lines = {sprintf('Target %d / Group %d', v.truth_idx, v.group.group_id), ...
    sprintf('Segments: %d  ->  1 group', numel(v.group_segments)), ...
    sprintf('R1 before: %d frames', numel(v.r1_before.frames)), ...
    sprintf('R2 middle: %d frames', numel(v.r2_middle.frames)), ...
    sprintf('R1 after:  %d frames', numel(v.r1_after.frames)), ...
    sprintf('Fused coverage: %d / %d (%.1f%%)', e.coverage_frames, e.truth_frames, 100*e.coverage_ratio), ...
    sprintf('Extension: +%d frames', e.extension_frames), ...
    sprintf('RMSE: %.2f km', e.rmse_km), ...
    sprintf('Overlap M1/M2: %d / %d frames', numel(v.overlap1), numel(v.overlap2))};
text(.05, .92, lines, 'VerticalAlignment', 'top', 'FontSize', 11, 'Interpreter', 'none');
title('Metrics');
end
