% =========================================================================
% plot_turn_fusion_compare.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标的融合对比综合图。使用 2x3 的 tiledlayout 布局，
%   包含六个子图：
%   (1) 融合全图 — 地图上对比基础融合（虚线）和自适应融合（实线）
%   (2) 拐弯区域放大 — 放大拐弯区域的细节对比
%   (3) RMSE 柱状图 — 所有融合方法和单站的 RMSE 对比
%   (4) 融合误差时间线 — 逐帧误差随时间的变化
%   (5) 单站→融合精度提升链 — 柱状图展示 R1→R2→融合 的精度提升
%   (6) 数值汇总表 — 文字形式列出关键数值结果
%
% 【数学原理】
%   1. 融合误差计算：逐帧取融合航迹与真值的 Haversine 距离。
%   2. RMSE 定义：RMSE = sqrt(mean(非NaN误差值的平方))
%      用于评估各融合方法在整条航迹上的平均跟踪精度。
%   3. 改善百分比：imp = (1 - RMSE_ad / RMSE_base) * 100%
%      正值表示自适应策略提升了精度，负值表示退化。
%   4. 累积分布函数(CDF)的角度：RMSE 柱状图从均值角度评估，
%      误差时间线从时变角度评估，两者互补。
%
% 【输入参数】
%   true_track       - Nx2 矩阵，真值航迹 [lon, lat]
%   fused_base       - 元胞数组，基础 UKF 各融合方法的快照
%   fuse_methods     - 基础 UKF 融合方法名称列表
%   best_m_base      - 基础 UKF 最优融合方法索引
%   fused_ad         - 元胞数组，自适应 UKF 各融合方法的快照
%   fuse_methods_ad  - 自适应 UKF 融合方法名称列表
%   best_m_ad        - 自适应 UKF 最优融合方法索引
%   trackR1_base     - R1 基础 UKF 跟踪快照
%   trackR2_base     - R2 基础 UKF 跟踪快照
%   trackR1_ad       - R1 自适应 UKF 跟踪快照
%   trackR2_ad       - R2 自适应 UKF 跟踪快照
%   fusion_eval_base - 基础 UKF 融合评估结果结构体
%   fusion_eval_ad   - 自适应 UKF 融合评估结果结构体
%   params           - 仿真参数字段结构体
%   out_dir          - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig3_fusion_compare.png  - 融合对比综合图
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   sphere_utils_haversine_distance() (球面距离)
%           extract_fused_ll()             (本文件内部)
%           extract_track_ll()             (本文件内部)
%           fused_err_at_frame()           (本文件内部)
%           get_zoom_idx()                 (本文件内部)
%           rms_val()                      (本文件内部)
%
% =========================================================================

function plot_turn_fusion_compare(true_track, ...
        fused_base, fuse_methods, best_m_base, ...
        fused_ad, fuse_methods_ad, best_m_ad, ...
        trackR1_base, trackR2_base, trackR1_ad, trackR2_ad, ...
        fusion_eval_base, fusion_eval_ad, params, out_dir)

    % 提取最优融合航迹的经纬度
    [lat_fb, lon_fb] = extract_fused_ll(fused_base{best_m_base});
    [lat_fa, lon_fa] = extract_fused_ll(fused_ad{best_m_ad});

    % 提取单站航迹经纬度（用于地图叠加和拐弯区域放大）
    [lat_r1b, lon_r1b] = extract_track_ll(trackR1_base);
    [lat_r1a, lon_r1a] = extract_track_ll(trackR1_ad);
    [lat_r2b, lon_r2b] = extract_track_ll(trackR2_base);
    [lat_r2a, lon_r2a] = extract_track_ll(trackR2_ad);

    % =====================================================================
    % 定位拐弯区域：找到距离指定拐点 (128.5E, 33.5N) 最近的帧
    % =====================================================================
    mid = round(size(true_track,1)/2);
    turn_lon = 128.5; turn_lat = 33.5;
    min_dist = inf; turn_frame = mid;
    for kk = 1:size(true_track,1)
        d = sphere_utils_haversine_distance(true_track(kk,1), true_track(kk,2), turn_lon, turn_lat);
        if d < min_dist, min_dist = d; turn_frame = kk; end
    end
    % 拐弯区域范围：拐点前后各 20 帧
    zoom_range = max(1,turn_frame-20):min(size(true_track,1),turn_frame+20);

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % =====================================================================
    % 子图 1: 融合全图（基础融合虚线 vs 自适应融合实线）
    % =====================================================================
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

    % 真值、基础融合、自适应融合
    geoplot(gx1, true_track(:,2), true_track(:,1), 'y--', 'LineWidth', 1.8, 'DisplayName', '真值');
    h_fb = geoplot(gx1, lat_fb, lon_fb, '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2, ...
        'DisplayName', sprintf('基础%s融合', fuse_methods{best_m_base}));
    h_fa = geoplot(gx1, lat_fa, lon_fa, '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 2.8, ...
        'DisplayName', sprintf('自适应%s融合', fuse_methods_ad{best_m_ad}));

    % 站点标记
    geoplot(gx1, params.radar1_lat, params.radar1_lon, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    geoplot(gx1, params.radar2_lat, params.radar2_lon, 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    legend(gx1, 'Location', 'northeast', 'FontSize', 7);

    % 拐弯区域白框：指示放大子图的区域
    rx = [min(true_track(zoom_range,1)), max(true_track(zoom_range,1))];
    ry = [min(true_track(zoom_range,2)), max(true_track(zoom_range,2))];
    geoplot(gx1, [ry(1) ry(1) ry(2) ry(2) ry(1)], ...
                 [rx(1) rx(2) rx(2) rx(1) rx(1)], 'w-', 'LineWidth', 1.2);

    % =====================================================================
    % 子图 2: 拐弯区域放大
    % =====================================================================
    nexttile(tlo, 2);
    try
        gx2 = geoaxes;
        gx2.Basemap = 'darkwater';
    catch
        gx2 = geoaxes;
    end
    hold(gx2, 'on');
    title(gx2, '拐弯区域放大', 'FontSize', 10);

    % 真值（拐弯区域），线宽加粗
    geoplot(gx2, true_track(zoom_range,2), true_track(zoom_range,1), 'y--', 'LineWidth', 2.5);

    % 基础融合（拐弯区域）
    if ~isempty(lat_fb)
        iz = get_zoom_idx(lat_fb, zoom_range);
        geoplot(gx2, lat_fb(iz), lon_fb(iz), '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 2.5);
    end
    % 自适应融合（拐弯区域），线宽最粗
    if ~isempty(lat_fa)
        iz = get_zoom_idx(lat_fa, zoom_range);
        geoplot(gx2, lat_fa(iz), lon_fa(iz), '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 3);
    end

    % 单站航迹叠加（半透明，用于展示融合对精度的提升）
    if ~isempty(lat_r1b)
        iz = get_zoom_idx(lat_r1b, zoom_range);
        geoplot(gx2, lat_r1b(iz), lon_r1b(iz), ':', 'Color', [0.4 0.6 1.0 0.5], 'LineWidth', 1);
    end
    if ~isempty(lat_r2b)
        iz = get_zoom_idx(lat_r2b, zoom_range);
        geoplot(gx2, lat_r2b(iz), lon_r2b(iz), ':', 'Color', [1.0 0.5 0.5 0.5], 'LineWidth', 1);
    end

    legend(gx2, {'真值', '基础融合', '自适应融合', 'R1单站', 'R2单站'}, ...
        'Location', 'best', 'FontSize', 6);

    % =====================================================================
    % 子图 3: RMSE 柱状图 — 所有融合方法 + 单站的 RMSE 对比
    % =====================================================================
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
    w = 0.35;  % 柱宽
    % 灰色柱 = 基础 UKF，绿色柱 = 自适应 UKF
    b1 = bar(ax3, x_pos - w/2, rmse_base, w, 'FaceColor', [0.6 0.6 0.6]);
    b2 = bar(ax3, x_pos + w/2, rmse_ad, w, 'FaceColor', [0.0 0.5 0.0]);
    set(ax3, 'XTick', x_pos, 'XTickLabel', methods_all, 'FontSize', 7);
    xtickangle(ax3, 30);  % 标签倾斜 30° 避免重叠
    ylabel(ax3, 'RMSE (km)');
    title(ax3, '融合RMSE对比: 基础(灰) vs 自适应(绿)', 'FontSize', 10);
    legend(ax3, [b1, b2], {'基础UKF融合', '自适应UKF融合'}, 'Location', 'best', 'FontSize', 7);
    grid(ax3, 'on');

    % 在柱顶标注改善百分比
    for m = 1:n_m
        if rmse_base(m) > 0
            imp = (1 - rmse_ad(m)/rmse_base(m))*100;
            y_pos = max(rmse_base(m), rmse_ad(m)) + 0.5;
            text(ax3, x_pos(m), y_pos, sprintf('%+.0f%%', imp), ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.8 0 0], ...
                'HorizontalAlignment', 'center');
        end
    end

    % =====================================================================
    % 子图 4: 融合误差时间线
    % =====================================================================
    ax4 = nexttile(tlo, 4);
    n_frames = length(fused_base{best_m_base});
    t = (0:n_frames-1) * params.dt_sec;

    % 逐帧计算基础融合和自适应融合的位置误差 (km)
    err_fb = nan(1, n_frames);
    err_fa = nan(1, n_frames);
    for k = 1:min(n_frames, size(true_track,1))
        err_fb(k) = fused_err_at_frame(fused_base{best_m_base}{k}, true_track(k,1), true_track(k,2));
        err_fa(k) = fused_err_at_frame(fused_ad{best_m_ad}{k}, true_track(k,1), true_track(k,2));
    end

    hold(ax4, 'on');
    plot(ax4, t, err_fb, '--', 'Color', [0.0 0.7 0.7], 'LineWidth', 1.5);
    plot(ax4, t, err_fa, '-', 'Color', [0.0 0.4 0.1], 'LineWidth', 2);
    % 竖线标记拐弯开始时刻
    xline(ax4, t(turn_frame), 'k--', 'LineWidth', 0.8);
    xlabel(ax4, '时间 (s)'); ylabel(ax4, '位置误差 (km)');
    title(ax4, sprintf('融合误差: 基础RMSE=%.1f  自适应RMSE=%.1f', ...
        rms_val(err_fb), rms_val(err_fa)), 'FontSize', 10);
    legend(ax4, {sprintf('基础%s融合', fuse_methods{best_m_base}), ...
        sprintf('自适应%s融合', fuse_methods_ad{best_m_ad})}, 'Location', 'best', 'FontSize', 8);
    grid(ax4, 'on');

    % =====================================================================
    % 子图 5: 单站 vs 融合误差对比（精度提升链）
    % =====================================================================
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

    % 标注改善百分比
    for i = 1:3
        imp = (1 - ad_vals(i)/base_vals(i))*100;
        text(ax5, xp(i), max(base_vals(i), ad_vals(i))+0.3, sprintf('%+.0f%%', imp), ...
            'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end

    % =====================================================================
    % 子图 6: 数值汇总表（文字形式）
    % =====================================================================
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

% =========================================================================
% 辅助函数
% =========================================================================

% extract_fused_ll - 从融合快照中提取单个航迹的经纬度序列
function [lats, lons] = extract_fused_ll(snapshots)
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

% extract_track_ll - 从跟踪快照中提取单个航迹的经纬度
function [lats, lons] = extract_track_ll(snapshots)
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

% fused_err_at_frame - 计算单帧融合航迹与真值的位置误差 (km)
function d = fused_err_at_frame(snap, t_lon, t_lat)
    d = NaN;
    if isempty(snap.trackList), return; end
    trk = snap.trackList{1};
    if ~isfield(trk, 'lon') || isnan(trk.lon), return; end
    d = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
end

% get_zoom_idx - 获取拐弯区域放大子图的索引范围
%   确保索引不超出数组长度
function iz = get_zoom_idx(arr, zoom_range)
    iz = zoom_range(zoom_range <= length(arr));
end

% rms_val - 计算忽略 NaN 后的均方根值 (RMS)
function v = rms_val(x)
    x_valid = x(~isnan(x));
    if isempty(x_valid), v = NaN; return; end
    v = sqrt(mean(x_valid.^2));
end
