function figs = plot_online_fr24(result, outdir)
%PLOT_ONLINE_FR24 跨区双站在线配对融合的可视化
%
%   figs = plot_online_fr24(result, outdir)
%
%   输出四张图：
%     1) 布设与覆盖互补示意（站点、波束、真实航迹、系统航迹）
%     2) 代表性目标的覆盖时间轴（仅 R2 / 共视 / 仅 R1 / 系统航迹）
%     3) 逐目标航迹长度比（系统航迹 vs 单站基线）
%     4) 双源帧上的位置误差对比（R1 / R2 / 系统）
%
%   result — run_online 的输出结构体
%   outdir — 输出目录（不存在则创建）

    if nargin < 2 || isempty(outdir), outdir = 'results/online_fr24'; end
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    figs = {};

    p = result.params;
    M = result.metrics;
    n_frames = result.n_frames;
    ctSnap = result.ctSnap;
    % run.m 九阶段重构后的字段名（旧版为 snap1s / snap2s，已不存在）
    snap1s = result.trackSnapshots_R1;
    snap2s = result.aligned_R2;

    % ---------------- 图 1：布设与覆盖 ----------------
    f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 800]);
    hold on;
    % 真实航迹（浅灰）
    for i = 1:numel(result.truth_all)
        tt = result.truth_all{i};
        plot(tt(:,1), tt(:,2), '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.6);
    end
    % 波束扇区
    draw_beam(f1, p.radar1_lon, p.radar1_lat, p.radar1_beam_center_deg, ...
        p.beam_width_deg/2, [0.20 0.55 0.85]);
    draw_beam(f1, p.radar2_lon, p.radar2_lat, p.radar2_beam_center_deg, ...
        p.beam_width_deg/2, [0.90 0.55 0.20]);
    % 站点
    plot(p.radar1_lon, p.radar1_lat, 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.20 0.55 0.85], 'MarkerEdgeColor', 'k');
    plot(p.radar2_lon, p.radar2_lat, 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.90 0.55 0.20], 'MarkerEdgeColor', 'k');
    text(p.radar1_lon, p.radar1_lat+0.35, 'R1 北站', 'FontSize', 11, 'HorizontalAlignment', 'center');
    text(p.radar2_lon, p.radar2_lat-0.55, 'R2 南站', 'FontSize', 11, 'HorizontalAlignment', 'center');
    % 发射站
    plot(p.radar1_tx_lon, p.radar1_tx_lat, '^', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.6 0.8 1.0], 'MarkerEdgeColor', 'k');
    plot(p.radar2_tx_lon, p.radar2_tx_lat, '^', 'MarkerSize', 8, ...
        'MarkerFaceColor', [1.0 0.8 0.6], 'MarkerEdgeColor', 'k');
    % 选中目标的系统航迹
    sel = pick_target(result);
    if ~isempty(sel)
        xs = system_track_xy(ctSnap, sel);
        if ~isempty(xs)
            plot(xs(:,1), xs(:,2), '-', 'Color', [0.15 0.60 0.35], 'LineWidth', 2.4);
        end
    end
    xlabel('经度 (°E)'); ylabel('纬度 (°N)');
    title(sprintf('覆盖互补布设：两站视场 15°，R1 偏北 / R2 偏南（绿线为系统航迹，标签 %s）', ...
        label_of(sel)));
    grid on; axis tight; xlim([100 113]); ylim([21 40]);
    legend({'真实航迹 (ADS-B)', 'R1 覆盖扇区', 'R2 覆盖扇区'}, 'Location', 'southwest');
    set(f1, 'Color', 'w');
    f = fullfile(outdir, 'fig1_deployment_coverage.png');
    print(f1, f, '-dpng', '-r130'); close(f1);
    figs{end+1} = f;

    % ---------------- 图 2：覆盖时间轴 ----------------
    f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 700]);
    if ~isempty(sel)
        f1_frames = frames_of_truth(snap1s, sel);
        f2_frames = frames_of_truth(snap2s, sel);
        ct_frames = system_frames(ctSnap, sel);
        both = intersect(f1_frames, f2_frames);
        only1 = setdiff(f1_frames, f2_frames);
        only2 = setdiff(f2_frames, f1_frames);
        hold on;
        y = 4; w = 0.55;
        bar_spans(only2, y,   w, [0.90 0.55 0.20], '仅 R2 覆盖');
        bar_spans(both,   y-w, w, [0.25 0.70 0.45], '两站共视');
        bar_spans(only1, y-2*w, w, [0.20 0.55 0.85], '仅 R1 覆盖');
        bar_spans(ct_frames, 1, 0.5, [0.35 0.35 0.40], '系统航迹（有源帧）');
        set(gca, 'YTick', [1, y-2*w, y-w, y], ...
            'YTickLabel', {'系统航迹', '仅 R1', '共视', '仅 R2'});
        xlabel('帧号（30 s/帧）'); ylim([0.5 4.9]);
        title(sprintf('目标 %s 的覆盖接力与系统航迹长度（单站 %d 帧 → 系统 %d 帧）', ...
            label_of(sel), max(numel(f1_frames), numel(f2_frames)), numel(ct_frames)));
        grid on; axis tight;
        legend('Location', 'eastoutside');
    end
    set(f2, 'Color', 'w');
    f = fullfile(outdir, 'fig2_coverage_timeline.png');
    print(f2, f, '-dpng', '-r130'); close(f2);
    figs{end+1} = f;

    % ---------------- 图 3：逐目标长度比 ----------------
    f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 620]);
    T = result.lengthTable;
    if ~isempty(T)
        rt = [T.ratio];
        ct = [T.ct_len]; sg = [T.single];
        ok = sg > 0 & isfinite(rt);
        bar(1:sum(ok), rt(ok), 0.6, 'FaceColor', [0.25 0.70 0.45]);
        hold on;
        plot([0.5, sum(ok)+0.5], [1 1], 'k--', 'LineWidth', 1.2);
        ylabel('系统航迹长度 / 单站基线长度');
        xlabel('目标序号');
        title(sprintf('逐目标航迹长度提升（均值 %.2f，中位 %.2f；共 %d 条真值）', ...
            mean(rt(ok)), median(rt(ok)), sum(ok)));
        grid on; xlim([0.5, sum(ok)+0.5]);
        ylim([0, max(1.6, max(rt(ok))*1.15)]);
    end
    set(f3, 'Color', 'w');
    f = fullfile(outdir, 'fig3_length_ratio.png');
    print(f3, f, '-dpng', '-r130'); close(f3);
    figs{end+1} = f;

    % ---------------- 图 4：融合精度 ----------------
    f4 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 620]);
    cf = M.common_frame;
    v = [cf.R1, cf.R2, cf.fused];
    names = {'R1 单站', 'R2 单站', '系统航迹(融合)'};
    bar(v, 0.5); set(gca, 'XTickLabel', names);
    for i = 1:3
        text(i, v(i), sprintf(' %.2f km', v(i)), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    ylabel('位置误差 RMSE (km)');
    gain = 100*(min(cf.R1, cf.R2)-cf.fused)/min(cf.R1, cf.R2);
    title(sprintf('共同帧上的位置误差：系统 %.2f km vs 最优单站 %.2f km（增益 %+.1f%%，n=%d 帧）', ...
        cf.fused, min(cf.R1, cf.R2), gain, cf.n));
    grid on; ylim([0, max(v)*1.25]);
    set(f4, 'Color', 'w');
    f = fullfile(outdir, 'fig4_fusion_accuracy.png');
    print(f4, f, '-dpng', '-r130'); close(f4);
    figs{end+1} = f;
end

% =====================================================================
function draw_beam(fig, lon0, lat0, azc, half, col)
    rmax = 2000e3;
    for s = [-half, half]
        [lon1, lat1] = sphere_utils_destination_point(lon0, lat0, rmax, azc + s);
        plot([lon0 lon1], [lat0 lat1], '--', 'Color', col, 'LineWidth', 1.2);
    end
    % 中心线
    [lonm, latm] = sphere_utils_destination_point(lon0, lat0, rmax, azc);
    plot([lon0 lonm], [lat0 latm], '-', 'Color', col, 'LineWidth', 1.0);
    plot(lon0, lat0, 's', 'MarkerSize', 6, 'MarkerFaceColor', col, 'MarkerEdgeColor', 'k');
end

function bar_spans(frames, y, h, col, name)
    if isempty(frames), return; end
    brk = [0; find(diff(frames) > 1); numel(frames)];
    for i = 1:numel(brk)-1
        a = frames(brk(i)+1); b = frames(brk(i+1));
        patch([a b+1 b+1 a], [y y y+h y+h], col, 'FaceAlpha', 0.85, ...
            'EdgeColor', 'none', 'DisplayName', name);
    end
end

function t = pick_target(result)
%PICK_TARGET 选长度比最大的真值目标，用于图 1/图 2 的代表性展示
%   （原来误用外层变量 r，子函数作用域里取不到，会报"未定义变量 r"）
    T = result.lengthTable;
    t = [];
    if isempty(T), return; end
    rt = [T.ratio]; sg = [T.single];
    ok = sg > 0 & isfinite(rt);
    if ~any(ok), return; end
    idx = find(ok);
    [~, j] = max(rt(idx));
    t = T(idx(j)).truth;
end

function xy = system_track_xy(ctSnap, tid)
% 从逐帧系统航迹里抽出某条真值的系统航迹轨迹
    xy = [];
    for k = 1:numel(ctSnap)
        CTk = ctSnap{k};
        if isempty(CTk), continue; end
        for i = 1:numel(CTk)
            if ~isfinite(CTk(i).truth_idx) || CTk(i).truth_idx ~= tid, continue; end
            if CTk(i).n_src == 0, continue; end
            x = CTk(i).x;
            if isempty(x), continue; end
            xy(end+1, :) = [x(1), x(3)]; %#ok<AGROW>
        end
    end
end

function fr = frames_of_truth(snaps, tid)
    fr = [];
    for k = 1:numel(snaps)
        s = snaps{k};
        if isempty(s) || ~isfield(s, 'trackList'), continue; end
        for i = 1:numel(s.trackList)
            trk = s.trackList{i};
            if ~isfield(trk, 'truth_idx') || ~isfinite(double(trk.truth_idx)), continue; end
            if double(trk.truth_idx) == tid
                fr(end+1) = k; %#ok<AGROW>
                break;
            end
        end
    end
    fr = unique(fr);
end

function fr = system_frames(ctSnap, tid)
% 某条真值的系统航迹"有源帧"列表
    fr = [];
    for k = 1:numel(ctSnap)
        CTk = ctSnap{k};
        if isempty(CTk), continue; end
        for i = 1:numel(CTk)
            if ~isfinite(CTk(i).truth_idx) || CTk(i).truth_idx ~= tid, continue; end
            if CTk(i).n_src > 0, fr(end+1) = k; end %#ok<AGROW>
        end
    end
    fr = unique(fr);
end

function s = label_of(tid)
    s = sprintf('#%d', tid);
end
