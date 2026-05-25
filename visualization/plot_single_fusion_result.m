% =========================================================================
% plot_single_fusion_result.m
% =========================================================================
%
% 【功能概述】
%   绘制单目标双基地雷达航迹融合结果的综合可视化。分两张图：
%   (1) 地图叠加图 — 显示真值、R1/R2 单站 UKF 滤波航迹、以及
%       四种不同的融合航迹，右侧提供复选框图层控制；
%   (2) 误差分析图 — 左右分栏显示滑动平均误差收敛曲线和累积分布
%       函数(CDF)，右侧提供曲线复选框控制。
%
% 【数学原理】
%   1. 多传感器航迹融合：将两个雷达独立跟踪产生的航迹通过融合算法
%      （如协方差交叉CI、简单凸组合SCC、信息滤波器IF等）合并为
%      一条精度更高的融合航迹。融合利用了两个雷达不同的观测几何
%      （双基地角不同），可以互补各自的定位盲区。
%   2. Haversine 距离公式：
%      d = 2R * atan2(sqrt(a), sqrt(1-a))
%      其中 a = sin²(Δlat/2) + cos(lat1)*cos(lat2)*sin²(Δlon/2)
%      R = 6371 km (地球平均半径)，用于计算两点间的大圆距离。
%   3. 误差评估指标：
%      - RMSE (Root Mean Square Error)：逐帧计算融合航迹与真值之间
%        的 Haversine 距离，取有效帧的均方根
%      - CDF (累积分布函数)：误差的统计分布，反映不同置信水平下
%        的误差上界
%      - 滑动平均：对帧误差序列进行 window=10 帧的 movmean 平滑，
%        滤除高频抖动，展示误差的收敛趋势
%   4. 最优融合选择：遍历所有融合方法，选取 RMSE 最小的作为最优方法
%
% 【输入参数】
%   true_track            - Nx2 矩阵，真值航迹 [lon, lat]
%   trackSnapshots_R1     - R1 UKF 跟踪快照
%   trackSnapshots_R2     - R2 UKF 跟踪快照
%   all_fused_snapshots   - 元胞数组，all_fused_snapshots{m} 为第 m 种
%                           融合方法的快照序列
%   method_names          - 字符串元胞数组，各融合方法的名称
%   best_idx              - 最优融合方法的索引（RMSE 最小者）
%   fusion_eval           - 融合评估结果结构体，含 fusion_errors,
%                           r1_errors, r2_errors 和 overall 数组
%   truthTraj             - 真值轨迹结构体，含 time_sec, lat, lon 字段
%   params                - 仿真参数字段结构体
%   out_dir               - 输出图片目录路径
%
% 【输出】
%   生成两个 PNG 文件：
%       fig6_single_fusion_map.png   - 融合地图叠加图
%       fig7_single_fusion_error.png - 误差收敛曲线 + CDF 图
%
% 【调用关系】
%   被调用: 主仿真脚本
%   调用:   collect_positions()         (本文件内部)
%           collect_fused_positions()   (本文件内部)
%           build_frame_errors()        (本文件内部)
%           build_single_frame_errors() (本文件内部)
%           haversine_km()              (本文件内部)
%
% =========================================================================

function plot_single_fusion_result(true_track, trackSnapshots_R1, trackSnapshots_R2, ...
        all_fused_snapshots, method_names, best_idx, fusion_eval, truthTraj, params, out_dir)

    n_methods = length(method_names);
    % 构建时间轴：每帧间隔 params.dt_sec 秒
    frame_times = (0:length(trackSnapshots_R1)-1) * params.dt_sec;

    %% ===== Figure 1: 地图叠加 =====
    % 目的：在同一张地理地图上对比真值、单站滤波航迹和各种融合航迹
    fig1 = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.68, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.68, 0.88]);
    end
    hold(ax, 'on');

    h_all = []; layer_names = {};

    % 真值航迹 (绿色虚线+方块)
    h = geoplot(ax, true_track(:,2), true_track(:,1), '--s', ...
        'Color', 'g', 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'g', ...
        'DisplayName', '真值');
    h_all(end+1) = h; layer_names{end+1} = '真值航迹';

    % R1 UKF 滤波航迹 (蓝色圆点)
    r1_tracks = collect_positions(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 2
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', sprintf('R1 UKF#%d', trk.id));
            h_all(end+1) = h; layer_names{end+1} = sprintf('R1 UKF#%d', trk.id);
        end
    end

    % R2 UKF 滤波航迹 (红色上三角)
    r2_tracks = collect_positions(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 2
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', sprintf('R2 UKF#%d', trk.id));
            h_all(end+1) = h; layer_names{end+1} = sprintf('R2 UKF#%d', trk.id);
        end
    end

    % 四种融合航迹（菱形标记，最优方法用更粗的线宽突出）
    % 颜色方案：深绿、橙色、深蓝、紫色
    method_colors = {[0 0.5 0], [0.8 0.4 0], [0 0 0.8], [0.6 0 0.6]};
    for m = 1:n_methods
        snaps_m = all_fused_snapshots{m};
        fused_pos = collect_fused_positions(snaps_m);
        for t = 1:length(fused_pos)
            ft = fused_pos{t};
            if length(ft.lat_history) > 2
                lw = 3.0;
                if m == best_idx, lw = 3.5; end  % 最优方法线宽加粗
                h = geoplot(ax, ft.lat_history, ft.lon_history, '-d', ...
                    'Color', method_colors{m}, 'LineWidth', lw, ...
                    'MarkerSize', 5, 'MarkerFaceColor', method_colors{m}, ...
                    'DisplayName', sprintf('%s 融合', method_names{m}));
                h_all(end+1) = h;
                layer_names{end+1} = sprintf('%s 融合航迹', method_names{m});
            end
        end
    end

    % 站点标记
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, sprintf('单目标双基地雷达航迹融合结果 (%s最优)', method_names{best_idx}));

    % ---- 右侧图层控制面板 ----
    n_layers1 = length(layer_names);
    cb1 = gobjects(1, n_layers1);
    for i = 1:n_layers1
        ypos = 0.92 - (i-1) * 0.040;
        if ypos < 0.05, break; end
        cb1(i) = uicontrol('Parent', fig1, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.74, ypos, 0.24, 0.036], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(h_all(i), src.Value));
    end

    btn_bottom1 = 0.92 - n_layers1 * 0.040 - 0.01;
    if btn_bottom1 > 0.02
        uicontrol('Parent', fig1, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.74, btn_bottom1, 0.11, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb(src, cb1, h_all));
        uicontrol('Parent', fig1, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.86, btn_bottom1, 0.11, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb(cb1, h_all));
    end

    % 底部融合结果标注
    best_rmse = fusion_eval.overall(best_idx).s.rms;
    uicontrol('Parent', fig1, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.74, 0.005, 0.24, 0.04], ...
        'String', sprintf('最佳: %s RMSE=%.1fkm', method_names{best_idx}, best_rmse), ...
        'FontSize', 9, 'BackgroundColor', [1 1 1], 'FontWeight', 'bold');

    drawnow;
    try
        exportgraphics(fig1, fullfile(out_dir, 'fig6_single_fusion_map.png'), 'Resolution', 200);
    catch
        saveas(fig1, fullfile(out_dir, 'fig6_single_fusion_map.png'));
    end
    fprintf('  融合地图已保存: fig6_single_fusion_map.png\n');

    %% ===== Figure 2: 误差收敛曲线 + CDF =====
    fig2 = figure('Position', [50, 50, 1400, 750]);
    win = 10;  % 滑动平均窗口大小（帧数）

    % 线型方案：实线、虚线、点划线、点线
    method_ls = {'-', '--', '-.', ':'};
    method_clr = {[0 0 0], [1 0 0], [0 0 1], [0 0.7 0]};

    % ---- 左子图：误差收敛曲线（滑动平均） ----
    subplot(1, 2, 1);
    hold on; grid on;
    all_h_lines = []; all_line_names = {};

    for m = 1:n_methods
        % 逐帧构建融合航迹相对于真值的位置误差
        fe = build_frame_errors(all_fused_snapshots{m}, truthTraj, frame_times);
        if length(fe) >= win
            % movmean: 滑动平均平滑，处理 NaN (omitnan)
            smoothed = movmean(fe, win, 'omitnan');
            h = plot(frame_times, smoothed, 'LineStyle', method_ls{m}, ...
                'Color', method_clr{m}, 'LineWidth', 2);
            all_h_lines(end+1) = h;
            all_line_names{end+1} = method_names{m};
        end
    end

    % R1 单站 UKF 误差曲线（蓝色点线）
    r1_fe = build_single_frame_errors(trackSnapshots_R1, truthTraj, frame_times);
    if length(r1_fe) >= win
        h = plot(frame_times, movmean(r1_fe, win, 'omitnan'), ...
            ':', 'Color', [0 0 0.7], 'LineWidth', 1.5);
        all_h_lines(end+1) = h;
        all_line_names{end+1} = 'R1 UKF';
    end

    % R2 单站 UKF 误差曲线（红色点线）
    r2_fe = build_single_frame_errors(trackSnapshots_R2, truthTraj, frame_times);
    if length(r2_fe) >= win
        h = plot(frame_times, movmean(r2_fe, win, 'omitnan'), ...
            ':', 'Color', [0.7 0 0], 'LineWidth', 1.5);
        all_h_lines(end+1) = h;
        all_line_names{end+1} = 'R2 UKF';
    end

    xlabel('时间 (s)'); ylabel('位置误差 (km)');
    title(sprintf('误差收敛曲线 (滑动平均 %d帧)', win));
    legend(all_line_names, 'FontSize', 8, 'Location', 'best');

    % ---- 右子图：误差的累积分布函数 (CDF) ----
    % CDF 横轴为误差值(km)，纵轴为累积概率(%)，反映误差的统计分布
    subplot(1, 2, 2);
    hold on; grid on;
    for m = 1:n_methods
        errs = fusion_eval.fusion_errors{m, 1};
        if ~isempty(errs)
            % ecdf: 经验累积分布函数
            [f, x] = ecdf(errs);
            plot(x, f*100, 'LineStyle', method_ls{m}, ...
                'Color', method_clr{m}, 'LineWidth', 2);
        end
    end
    % 叠加单站 UKF 的 CDF 作为基准对比
    if ~isempty(fusion_eval.r1_errors{1})
        [f, x] = ecdf(fusion_eval.r1_errors{1});
        plot(x, f*100, ':', 'Color', [0 0 0.7], 'LineWidth', 1.5);
    end
    if ~isempty(fusion_eval.r2_errors{1})
        [f, x] = ecdf(fusion_eval.r2_errors{1});
        plot(x, f*100, ':', 'Color', [0.7 0 0], 'LineWidth', 1.5);
    end
    xlabel('位置误差 (km)'); ylabel('累积概率 (%)');
    title('误差CDF对比');
    legend([method_names, {'R1 UKF', 'R2 UKF'}], 'FontSize', 8, 'Location', 'southeast');

    % ---- 右侧曲线控制面板 ----
    n2 = length(all_line_names);
    cb2 = gobjects(1, n2);
    for i = 1:n2
        ypos = 0.92 - (i-1) * 0.05;
        if ypos < 0.05, break; end
        cb2(i) = uicontrol('Parent', fig2, 'Style', 'checkbox', ...
            'String', all_line_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.045], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(all_h_lines(i), src.Value));
    end

    drawnow;
    try
        exportgraphics(fig2, fullfile(out_dir, 'fig7_single_fusion_error.png'), 'Resolution', 200);
    catch
        saveas(fig2, fullfile(out_dir, 'fig7_single_fusion_error.png'));
    end
    fprintf('  误差收敛曲线已保存: fig7_single_fusion_error.png\n');
end

% =========================================================================
% 辅助函数
% =========================================================================

% collect_positions - 从 UKF 跟踪快照中搜集各航迹 ID 的位置历史
% 使用 containers.Map 按航迹 ID 聚合，跳过 type==7 的无效状态
function tracks = collect_positions(snapshots)
    track_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end
            tid = trk.id;
            if ~track_map.isKey(tid)
                track_map(tid) = struct('id', tid, 'lat_history', [], 'lon_history', []);
            end
            rec = track_map(tid);
            rec.lat_history(end+1) = trk.lat;
            rec.lon_history(end+1) = trk.lon;
            track_map(tid) = rec;
        end
    end
    tracks = values(track_map);
end

% collect_fused_positions - 从融合快照中搜集融合航迹的位置历史
% 与 collect_positions 类似，但不跳过 type==7(融合航迹通常无此字段)
function tracks = collect_fused_positions(snapshots)
    track_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            ft = snap.trackList{t};
            pid = ft.id;
            if ~track_map.isKey(pid)
                track_map(pid) = struct('id', pid, 'lat_history', [], 'lon_history', []);
            end
            rec = track_map(pid);
            rec.lat_history(end+1) = ft.lat;
            rec.lon_history(end+1) = ft.lon;
            track_map(pid) = rec;
        end
    end
    tracks = values(track_map);
end

% build_frame_errors - 逐帧计算融合航迹与真值之间的最小位置误差 (km)
% 在每帧中找与真值最近的融合航迹，计算 Haversine 距离
function fe = build_frame_errors(fused_snaps, truth, frame_times)
    n_frames = length(fused_snaps);
    fe = nan(1, n_frames);
    for k = 1:n_frames
        % 通过线性插值获取该帧时刻的真值位置
        t_lat = interp1(truth.time_sec, truth.lat, frame_times(k), 'linear', 'extrap');
        t_lon = interp1(truth.time_sec, truth.lon, frame_times(k), 'linear', 'extrap');
        if isnan(t_lat), continue; end
        snap = fused_snaps{k};
        if isempty(snap.trackList), continue; end
        best_d = inf;
        for t = 1:length(snap.trackList)
            ft = snap.trackList{t};
            d = haversine_km(ft.lon, ft.lat, t_lon, t_lat);
            if d < best_d, best_d = d; end
        end
        if best_d < inf, fe(k) = best_d; end
    end
end

% build_single_frame_errors - 逐帧计算单站 UKF 与真值之间的最小位置误差
% 与 build_frame_errors 类似，但使用单站跟踪快照
function fe = build_single_frame_errors(snapshots, truth, frame_times)
    n_frames = length(snapshots);
    fe = nan(1, n_frames);
    for k = 1:n_frames
        t_lat = interp1(truth.time_sec, truth.lat, frame_times(k), 'linear', 'extrap');
        t_lon = interp1(truth.time_sec, truth.lon, frame_times(k), 'linear', 'extrap');
        if isnan(t_lat), continue; end
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        best_d = inf;
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end
            d = haversine_km(trk.lon, trk.lat, t_lon, t_lat);
            if d < best_d, best_d = d; end
        end
        if best_d < inf, fe(k) = best_d; end
    end
end

% haversine_km - Haversine 公式计算两经纬度点的大圆距离 (km)
% 公式：d = 2R * atan2(sqrt(a), sqrt(1-a))
%       其中 a = sin²(Δφ/2) + cos(φ1)*cos(φ2)*sin²(Δλ/2)
% R = 6371 km (地球平均半径)
function d = haversine_km(lon1, lat1, lon2, lat2)
    R = 6371;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2)^2;
    a = max(0, min(1, a));  % 数值稳定：限制在 [0,1]
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end

function try_set_visible(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

function toggle_all_cb(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')
        new_val = 0; btn.String = '全部显示';
    else
        new_val = 1; btn.String = '全部隐藏';
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end
        try_set_visible(h_all(i), new_val);
    end
end

function show_all_cb(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible(h_all(i), 1);
    end
end
