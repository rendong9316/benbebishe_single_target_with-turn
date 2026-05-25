% =========================================================================
% plot_turn_comparison.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标航迹对比图。在 darkwater 暗底图上并排展示六类航迹：
%   真实航迹、R1/R2 基础 UKF 滤波、R1/R2 机动自适应 UKF 滤波、
%   以及两种 UKF 策略下的最优融合结果。右侧提供复选框图层控制器，
%   支持交互式切换各图层的显隐。
%
% 【数学原理】
%   1. 拐弯机动对滤波器的影响：
%      目标在转弯时，速度方向发生快速变化，匀速(CV)或匀加速(CA)
%      运动模型不再适用。基础 UKF 使用恒定的过程噪声 Q 无法适应
%      机动，而自适应 UKF 在检测到机动后增大 Q，使滤波器能更快
%      地跟踪机动。
%   2. 融合航迹 (Fused Track)：
%      对 R1 和 R2 独立跟踪得到的 UKF 估计进行多传感器融合。
%      自适应 UKF 由于在机动段估计精度更高，融合后的航迹精度也
%      相应提升。图中用菱形(-d)标记融合航迹。
%   3. 最优融合方法选择：
%      遍历 CI, SCC, IF 等多种融合算法，选取 RMSE 最小的作为最优
%      融合方法(best_m_base 和 best_m_ad 索引)。
%
% 【输入参数】
%   true_track     - Nx2 矩阵，真值航迹 [lon, lat]
%   trackR1_base   - R1 基础 UKF 跟踪快照
%   trackR2_base   - R2 基础 UKF 跟踪快照
%   fused_base     - 元胞数组，基础 UKF 下各融合方法的快照
%   fuse_methods   - 融合方法名称元胞数组
%   best_m_base    - 基础 UKF 最优融合方法索引
%   trackR1_ad     - R1 自适应 UKF 跟踪快照
%   trackR2_ad     - R2 自适应 UKF 跟踪快照
%   fused_ad       - 元胞数组，自适应 UKF 下各融合方法的快照
%   fuse_methods_ad - 自适应 UKF 融合方法名称元胞数组
%   best_m_ad      - 自适应 UKF 最优融合方法索引
%   params         - 仿真参数字段结构体
%   out_dir        - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig_turn_comparison.png  - 拐弯航迹对比图
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   extract_track_ll()      (本文件内部)
%           extract_fused_ll()      (本文件内部)
%           try_set_visible_turn()  (本文件内部)
%
% =========================================================================

function plot_turn_comparison(true_track, ...
        trackR1_base, trackR2_base, fused_base, fuse_methods, best_m_base, ...
        trackR1_ad, trackR2_ad, fused_ad, fuse_methods_ad, best_m_ad, ...
        params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);

    % 地理坐标轴占左侧 68% 宽度
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.03, 0.12, 0.68, 0.86]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.03, 0.12, 0.68, 0.86]);
    end
    hold(ax, 'on');

    h_all = [];
    layer_names = {};

    % =====================================================================
    % 图层 1: 真实航迹 (亮黄虚线)
    % =====================================================================
    h = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2.5, 'DisplayName', '真实航迹');
    h_all(end+1) = h;
    layer_names{end+1} = '真实航迹';

    % =====================================================================
    % 图层 2: R1 基础 UKF (淡蓝色)
    % 基础 UKF 使用固定 Q，转弯时可能出现滞后或过冲
    % =====================================================================
    [lat1, lon1] = extract_track_ll(trackR1_base);
    if ~isempty(lat1)
        h = geoplot(ax, lat1, lon1, '-', 'Color', [0.3 0.5 1.0], ...
            'LineWidth', 1.8, 'DisplayName', 'R1 基础UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 基础UKF(模糊Q)';
    end

    % =====================================================================
    % 图层 3: R2 基础 UKF (淡红色)
    % =====================================================================
    [lat2, lon2] = extract_track_ll(trackR2_base);
    if ~isempty(lat2)
        h = geoplot(ax, lat2, lon2, '-', 'Color', [1.0 0.4 0.4], ...
            'LineWidth', 1.8, 'DisplayName', 'R2 基础UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 基础UKF(模糊Q)';
    end

    % =====================================================================
    % 图层 4: R1 自适应 UKF (深蓝实线)
    % 自适应 UKF 在转弯段动态增大 Q，能更紧密地跟踪真实航迹
    % =====================================================================
    [lat1a, lon1a] = extract_track_ll(trackR1_ad);
    if ~isempty(lat1a)
        h = geoplot(ax, lat1a, lon1a, 'b-', ...
            'LineWidth', 2.2, 'DisplayName', 'R1 自适应UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 自适应UKF(机动检测)';
    end

    % =====================================================================
    % 图层 5: R2 自适应 UKF (深红实线)
    % =====================================================================
    [lat2a, lon2a] = extract_track_ll(trackR2_ad);
    if ~isempty(lat2a)
        h = geoplot(ax, lat2a, lon2a, 'r-', ...
            'LineWidth', 2.2, 'DisplayName', 'R2 自适应UKF');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 自适应UKF(机动检测)';
    end

    % =====================================================================
    % 图层 6: 基础 UKF 最优融合 (青色)
    % =====================================================================
    [lat_fb, lon_fb] = extract_fused_ll(fused_base{best_m_base});
    if ~isempty(lat_fb)
        h = geoplot(ax, lat_fb, lon_fb, 'c-', ...
            'LineWidth', 2.5, 'DisplayName', '基础UKF融合');
        h_all(end+1) = h;
        layer_names{end+1} = sprintf('基础UKF融合(%s)', fuse_methods{best_m_base});
    end

    % =====================================================================
    % 图层 7: 自适应 UKF 最优融合 (品红)
    % 自适应 UKF 的融合航迹应在转弯处显著优于基础 UKF 融合
    % =====================================================================
    [lat_fa, lon_fa] = extract_fused_ll(fused_ad{best_m_ad});
    if ~isempty(lat_fa)
        h = geoplot(ax, lat_fa, lon_fa, 'm-', ...
            'LineWidth', 2.5, 'DisplayName', '自适应UKF融合');
        h_all(end+1) = h;
        layer_names{end+1} = sprintf('自适应UKF融合(%s)', fuse_methods_ad{best_m_ad});
    end

    % ---- 站点标记 ----
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    % 起点/终点标记
    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', '终点');

    % 标注拐点：白色空心圆标记航迹中段（约 120° 拐角处）
    mid_idx = round(size(true_track,1)/2);
    geoplot(ax, true_track(mid_idx,2), true_track(mid_idx,1), 'wo', ...
        'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '拐点(~120°)');

    title(ax, '拐弯目标: 基础UKF vs 机动自适应UKF 对比');
    subtitle(ax, sprintf('120°拐角 Pd=%.0f%% Pfa=%.3f 航速%.0fm/s', ...
        params.detection_probability*100, params.false_alarm_rate, 140));

    % ---- 右侧图层控制面板 ----
    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);

    for i = 1:n_layers
        ypos = 0.93 - (i-1) * 0.048;
        if ypos < 0.05, break; end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.73, ypos, 0.25, 0.042], ...
            'FontSize', 8, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_turn(h_all(i), src.Value));
    end

    % 全部显示/隐藏按钮
    btn_y = 0.93 - n_layers * 0.048 - 0.01;
    if btn_y > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.73, btn_y, 0.12, 0.038], ...
            'FontSize', 8, ...
            'Callback', @(src, ~) toggle_all_turn(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.86, btn_y, 0.12, 0.038], ...
            'FontSize', 8, ...
            'Callback', @(~, ~) show_all_turn(cb, h_all));
    end

    % 底部统计信息条
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

% =========================================================================
% extract_track_ll - 从跟踪快照中提取单个航迹的经纬度序列
%   仅提取 type!=7 且 lat/lon 有效的帧
% =========================================================================
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

% =========================================================================
% extract_fused_ll - 从融合快照中提取融合航迹的经纬度序列
% =========================================================================
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

function try_set_visible_turn(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_turn(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible_turn(h_all(i), new_val);
    end
end

function show_all_turn(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible_turn(h_all(i), 1);
    end
end
