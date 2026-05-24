% =========================================================================
% plot_turn_rmse_bars.m
% 图6: RMSE柱状图 — 全部方法对比 (基础灰 vs 自适应绿)
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

    % ---- 左侧: 柱状图 ----
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

    % 标注数值和改善
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

    % ---- 右侧: 文字汇总 ----
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
