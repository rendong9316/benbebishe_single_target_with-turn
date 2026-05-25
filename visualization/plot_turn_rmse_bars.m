% =========================================================================
% plot_turn_rmse_bars.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标全部融合方法及单站滤波的 RMSE 对比柱状图。左侧为
%   柱状图区域：灰色柱 = 基础 UKF，绿色柱 = 机动自适应 UKF，
%   柱顶标注改善百分比。右侧为文字汇总面板，列出最优融合方法、
%   单站改善、仿真参数和图例信息。
%
% 【数学原理】
%   1. RMSE (均方根误差)：
%      RMSE = sqrt( (1/N) * sum_i (err_i)^2 )
%      其中 err_i = haversine_distance(lon_est_i, lat_est_i, lon_truth_i, lat_truth_i)
%      是每帧的融合/滤波位置估计与真值之间的 km 级误差。
%   2. 改善百分比：
%      对于每种方法和策略，计算 imp = (1 - RMSE_ad / RMSE_base) * 100%
%      正值(红色) = 自适应 UKF 精度提高
%      负值(蓝色) = 自适应 UKF 精度下降（退化）
%   3. 多种融合算法对比：
%      融合算法通常包括 CI (Covariance Intersection)、SCC (Simple Convex
%      Combination)、IF (Information Filter) 等。不同算法对传感器协方差
%      信息的利用方式不同，在拐弯场景下表现各异。
%   4. 单站 only 代理：
%      在 methods_all 末尾添加 'R1_only' 和 'R2_only'，其 RMSE 值来自
%      fusion_eval 结构体中对应单站误差的评估。
%
% 【输入参数】
%   fusion_eval_base - 基础 UKF 融合评估结果结构体，
%                      含 .overall(m).s.rms 字段
%   fusion_eval_ad   - 自适应 UKF 融合评估结果结构体
%   fuse_methods     - 融合方法名称元胞数组
%   best_m_base      - 基础 UKF 最优融合方法索引
%   best_m_ad        - 自适应 UKF 最优融合方法索引
%   params           - 仿真参数字段结构体
%   out_dir          - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig6_rmse_bars.png  - RMSE 柱状图对比
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   无外部依赖（MATLAB 内置 bar, text, axes 等）
%
% =========================================================================

function plot_turn_rmse_bars(fusion_eval_base, fusion_eval_ad, ...
        fuse_methods, best_m_base, best_m_ad, params, out_dir)

    % 构建完整方法列表：融合方法 + 单站
    methods_all = [fuse_methods, {'R1_only', 'R2_only'}];
    n_m = length(methods_all);
    rmse_base = zeros(1, n_m);
    rmse_ad   = zeros(1, n_m);
    for m = 1:n_m
        rmse_base(m) = fusion_eval_base.overall(m).s.rms;
        rmse_ad(m)   = fusion_eval_ad.overall(m).s.rms;
    end

    fig = figure('Position', [50, 50, 1400, 750]);

    % =====================================================================
    % 左侧: 柱状图（灰色 vs 绿色分组柱状图）
    % =====================================================================
    ax1 = axes('Units', 'normalized', 'Position', [0.08, 0.10, 0.55, 0.85]);
    hold(ax1, 'on');
    x_pos = 1:n_m;
    w = 0.35;  % 每组两柱总宽约 0.7
    b1 = bar(ax1, x_pos - w/2, rmse_base, w, 'FaceColor', [0.65 0.65 0.65], 'EdgeColor', 'none');
    b2 = bar(ax1, x_pos + w/2, rmse_ad, w, 'FaceColor', [0.0 0.45 0.0], 'EdgeColor', 'none');
    set(ax1, 'XTick', x_pos, 'XTickLabel', methods_all, 'FontSize', 11);
    ylabel(ax1, 'RMSE (km)', 'FontSize', 12);
    title(ax1, '融合RMSE对比: 基础UKF(灰) vs 机动自适应UKF(绿)', 'FontSize', 13);
    legend(ax1, [b1, b2], {'基础UKF融合', '自适应UKF融合'}, 'Location', 'northwest', 'FontSize', 11);
    grid(ax1, 'on');

    % 逐方法标注数值和改善百分比
    for m = 1:n_m
        % 柱顶标注 RMSE 数值（灰色柱上灰字，绿色柱上绿字）
        text(ax1, x_pos(m)-0.15, rmse_base(m)+0.3, sprintf('%.1f', rmse_base(m)), ...
            'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', [0.4 0.4 0.4]);
        text(ax1, x_pos(m)+0.15, rmse_ad(m)+0.3, sprintf('%.1f', rmse_ad(m)), ...
            'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', [0 0.3 0]);
        if rmse_base(m) > 0
            imp = (1 - rmse_ad(m)/rmse_base(m))*100;
            % 红色=改善，蓝色=退化
            c = [0.8 0 0]; if imp < 0, c = [0 0 0.6]; end
            yp = max(rmse_base(m), rmse_ad(m)) + 1.2;
            text(ax1, x_pos(m), yp, sprintf('%+.0f%%', imp), ...
                'FontSize', 11, 'FontWeight', 'bold', 'Color', c, 'HorizontalAlignment', 'center');
        end
    end

    % =====================================================================
    % 右侧: 文字汇总面板
    % =====================================================================
    ax2 = axes('Units', 'normalized', 'Position', [0.66, 0.10, 0.32, 0.85]);
    ax2.Visible = 'off';  % 隐藏坐标轴，仅显示文字
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
