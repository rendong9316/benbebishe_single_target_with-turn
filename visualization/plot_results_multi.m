% =========================================================================
% plot_results_multi.m — 多目标结果可视化调度器
% =========================================================================
%
% 【功能概述】
%   将多目标场景的可视化结果聚合到一个 dispatcher 中。
%   通过 mode 字符串调度对应的子函数。
%
% 【调度模式】
%   'single_track'  -> plot_multi_track_result(...)
%   'single_fusion' -> plot_multi_fusion_result(...)
%
% 【与 plot_results.m 的区别】
%   - 支持三个真值航迹 (A/B/C)
%   - 地图上用不同颜色区分不同目标的航迹
%   - 融合结果按匹配对分别绘制
%
% =========================================================================

% =========================================================================
% plot_results_multi — 可视化调度入口
% =========================================================================
% 根据 mode 参数分发到不同的子函数
% varargin 传递其余参数，{:} 展开为逗号分隔的参数列表
function plot_results_multi(mode, varargin)
    switch mode  % 根据 mode 字符串选择分支
        case 'single_track'
            % 调用多目标航迹结果绘制函数
            plot_multi_track_result(varargin{:});
        case 'single_fusion'
            % 调用多目标融合结果绘制函数
            plot_multi_fusion_result(varargin{:});
        otherwise
            % 无效的 mode 抛出错误
            error('plot_results_multi: unknown mode "%s". Valid: single_track, single_fusion', mode);
    end
end

% =========================================================================
% 1. plot_multi_track_result — 多目标双基地雷达航迹综合对比图
% =========================================================================
% 在一张地理坐标地图上同时绘制：
%   - 三架飞机的真值航迹（黄/品红/青）
%   - R1/R2 的校准点迹（蓝色系/红色系小圆点）
%   - R1/R2 的 UKF 航迹（蓝色实线+圆圈 / 红色实线+三角）
%   - 雷达站标记（接收站方块 + 发射站三角）
%   - 右侧图层控制复选框（可独立切换各图层可见性）
function plot_multi_track_result(true_track_A, true_track_B, true_track_C, ...
        detList_R1, detList_R2, trackSnapshots_R1, trackSnapshots_R2, params, out_dir)

    % 创建图窗
    fig = figure('Position', [50, 50, 1400, 750]);
    try
        % 尝试使用 darkwater 暗色底图，地图区域占左侧 70%
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        % 降级为默认底图
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');  % 保持坐标系，允许多层叠加

    h_all = [];       % 存储所有绘图句柄，用于图层控制
    layer_names = {}; % 存储图层名称，用于复选框显示

    % 真值航迹 (A=黄色, B=品红, C=青色)
    % '--s' 表示虚线+方块标记，颜色用 RGB 三元组 [1 1 0]=黄色
    h = geoplot(ax, true_track_A(:,2), true_track_A(:,1), '--s', ...
        'Color', [1 1 0], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 1 0], ...
        'DisplayName', '真值A');
    h_all(end+1) = h;  layer_names{end+1} = '真值A';  % 累积句柄和名称

    h = geoplot(ax, true_track_B(:,2), true_track_B(:,1), '--s', ...
        'Color', [1 0 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 0 1], ...
        'DisplayName', '真值B');
    h_all(end+1) = h;  layer_names{end+1} = '真值B';

    h = geoplot(ax, true_track_C(:,2), true_track_C(:,1), '--s', ...
        'Color', [0 1 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [0 1 1], ...
        'DisplayName', '真值C');
    h_all(end+1) = h;  layer_names{end+1} = '真值C';

    % R1 校准点迹 (蓝色系)
    % extract_dets_multi 从 detList_R1 中提取所有非杂波校准点迹的经纬度
    [r1_cal_lat, r1_cal_lon] = extract_dets_multi(detList_R1, 'cal');
    if ~isempty(r1_cal_lat)  % 如果有校准点迹数据
        % '.' 表示单独的小圆点，不连线
        h = geoplot(ax, r1_cal_lat, r1_cal_lon, '.b', ...
            'MarkerSize', 3, 'DisplayName', 'R1校准点迹');
        h_all(end+1) = h;  layer_names{end+1} = 'R1校准点迹';
    end

    % R1 航迹
    % collect_active_tracks_multi 从 trackSnapshots_R1 中收集所有活跃航迹
    % 返回 {id, lat_history, lon_history} 结构的 cell 数组
    r1_tracks = collect_active_tracks_multi(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};  % 取出第 t 条航迹
        if length(trk.lat_history) > 1  % 至少需要 2 个点才能画线
            seg_label = sprintf('R1 UKF#%d', trk.id);  % 格式化标签
            % 'b-o' 表示蓝色实线+圆形标记
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % R2 校准点迹 (红色系)
    [r2_cal_lat, r2_cal_lon] = extract_dets_multi(detList_R2, 'cal');
    if ~isempty(r2_cal_lat)
        h = geoplot(ax, r2_cal_lat, r2_cal_lon, '.r', ...
            'MarkerSize', 3, 'DisplayName', 'R2校准点迹');
        h_all(end+1) = h;  layer_names{end+1} = 'R2校准点迹';
    end

    % R2 航迹
    r2_tracks = collect_active_tracks_multi(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            % 'r-^' 表示红色实线+三角标记
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % 雷达站标记
    % 接收站：bs/rs 填充方块，发射站：b^/r^ 三角
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, '多目标双基地雷达航迹综合对比 (3目标交叉)');
    legend(ax, 'Location', 'northeastoutside');  % 图例放在地图区域外右上角

    % 图层控制复选框
    % 在地图右侧（x=0.76~0.98）从上到下排列复选框
    n_layers = length(layer_names);  % 图层总数
    cb = gobjects(1, n_layers);      % 预分配复选框句柄数组
    n_cb = 0;                        % 实际创建的复选框计数器
    for i = 1:n_layers
        ypos = 0.92 - n_cb * 0.045;  % 从上到下排列，每个间距 0.045
        if ypos < 0.05, break; end   % 超出底部边界时停止
        n_cb = n_cb + 1;
        % 创建复选框控件
        % Callback 使用匿名函数捕获 i 索引，src.Value 是复选框状态
        cb(n_cb) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_multi(h_all(i), src.Value));
    end

    % 全部隐藏/全部显示按钮
    % btn_bottom 计算按钮的 y 坐标，位于复选框下方
    btn_bottom = 0.92 - n_cb * 0.045 - 0.01;
    if btn_bottom > 0.02  % 按钮不能超出图窗底部
        % 全部隐藏按钮：toggle_all_cb_multi 将所有复选框设为 0
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.76, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb_multi(src, cb(1:n_cb), h_all(1:n_cb)));
        % 全部显示按钮：show_all_cb_multi 将所有复选框设为 1
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.87, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb_multi(cb(1:n_cb), h_all(1:n_cb)));
    end

    % 底部状态栏文本：显示 R1/R2 航迹数量和仿真参数
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.005, 0.22, 0.03], ...
        'String', sprintf('R1:%d航迹 R2:%d航迹 | Pd=%.0f%% Pfa=%.3f', ...
        length(r1_tracks), length(r2_tracks), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;  % 强制刷新图形显示
end

% ========================================================================= — 多目标融合结果可视化
% =========================================================================
% 绘制融合后的航迹结果，包含：
%   - 三架飞机的真值航迹
%   - R1/R2 的 UKF 航迹
%   - 最优融合算法的融合航迹（从 fusion_eval 中选出 RMSE 最小的算法）
%   - 误差收敛曲线图（Figure 2）
%   - 融合误差 CDF 图（Figure 2 右子图）
function plot_multi_fusion_result(true_track_A, true_track_B, true_track_C, ...
        trackSnapshots_R1, trackSnapshots_R2, all_fused_snapshots, ...
        method_names, matched_pairs, fusion_eval, truthTrajs, params, out_dir)

    % 创建图窗
    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    h_all = [];       % 存储所有绘图句柄
    layer_names = {}; % 存储图层名称

    % 真值航迹 (A=黄色, B=品红, C=青色)
    h = geoplot(ax, true_track_A(:,2), true_track_A(:,1), '--s', ...
        'Color', [1 1 0], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 1 0]);
    h_all(end+1) = h;  layer_names{end+1} = '真值A';
    h = geoplot(ax, true_track_B(:,2), true_track_B(:,1), '--s', ...
        'Color', [1 0 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 0 1]);
    h_all(end+1) = h;  layer_names{end+1} = '真值B';
    h = geoplot(ax, true_track_C(:,2), true_track_C(:,1), '--s', ...
        'Color', [0 1 1], 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', [0 1 1]);
    h_all(end+1) = h;  layer_names{end+1} = '真值C';

    % R1 UKF 航迹
    % collect_active_tracks_multi 从快照中收集活跃航迹
    r1_tracks = collect_active_tracks_multi(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R1 UKF#%d', trk.id);
            % 多条航迹时添加段号后缀
            if length(r1_tracks) > 1, seg_label = sprintf('R1 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 2.0, 'MarkerSize', 4, 'MarkerFaceColor', 'b', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % R2 UKF 航迹
    r2_tracks = collect_active_tracks_multi(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 1
            seg_label = sprintf('R2 UKF#%d', trk.id);
            if length(r2_tracks) > 1, seg_label = sprintf('R2 UKF#%d-段%d', trk.id, t); end
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 2.0, 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
                'DisplayName', seg_label);
            h_all(end+1) = h;  layer_names{end+1} = seg_label;
        end
    end

    % 融合航迹 — 仅绘制最优算法
    % 从 fusion_eval.overall 中选 RMSE 最小的融合算法
    % 遍历所有融合方法，比较 overall RMS 指标
    best_method_idx = 1;  % 默认选第一个方法
    best_rmse = inf;      % 初始最小 RMSE 设为无穷大
    for m = 1:length(method_names)
        if ~isempty(fusion_eval) && isfield(fusion_eval, 'overall')
            rms_m = fusion_eval.overall(m).s.rms;  % 取出第 m 个方法的 RMS 误差
            if rms_m < best_rmse  % 如果更小则更新最优
                best_rmse = rms_m;
                best_method_idx = m;
            end
        end
    end
    best_method = method_names{best_method_idx};  % 获取最优算法名称
    % 融合航迹颜色映射表：绿/橙/深蓝/紫
    method_colors = {[0 0.5 0], [0.8 0.4 0], [0 0 0.8], [0.6 0 0.6]};
    % 遍历每个匹配对（每个匹配对对应一架飞机）
    for p = 1:length(matched_pairs)
        snaps_m = all_fused_snapshots{p, best_method_idx};  % 取出最优算法在该匹配对的融合快照
        fused_pos = collect_fused_positions_multi(snaps_m);  % 收集融合位置历史
        for t = 1:length(fused_pos)
            ft = fused_pos{t};  % 第 t 条融合航迹
            if length(ft.lat_history) > 1
                seg_label = sprintf('%s Pair#%d', best_method, p);  % 标签：算法名+配对号
                % '-d' 表示菱形标记的实线
                h = geoplot(ax, ft.lat_history, ft.lon_history, '-d', ...
                    'Color', method_colors{best_method_idx}, 'LineWidth', 2, ...
                    'MarkerSize', 4, 'MarkerFaceColor', method_colors{best_method_idx}, ...
                    'DisplayName', seg_label);
                h_all(end+1) = h;  layer_names{end+1} = seg_label;
            end
        end
    end

    % 雷达站标记（不显示图例，仅作为参考）
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', 'MarkerSize', 10);
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', 'MarkerSize', 10);

    title(ax, sprintf('多目标融合结果 — 最优算法: %s (RMSE=%.1fkm)', best_method, best_rmse));

    % 图层控制复选框（与 plot_multi_track_result 相同的布局逻辑）
    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);
    n_cb = 0;
    for i = 1:n_layers
        ypos = 0.92 - n_cb * 0.045;
        if ypos < 0.05, break; end
        n_cb = n_cb + 1;
        cb(n_cb) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible_multi(h_all(i), src.Value));
    end

    % 全部隐藏/全部显示按钮
    btn_bottom = 0.92 - n_cb * 0.045 - 0.01;
    if btn_bottom > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.76, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb_multi(src, cb(1:n_cb), h_all(1:n_cb)));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.87, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb_multi(cb(1:n_cb), h_all(1:n_cb)));
    end

    % 融合算法统计信息文本（底部状态栏）
    info_str = sprintf('最优算法: %s | RMSE=%.1fkm | %d匹配对', best_method, best_rmse, length(matched_pairs));
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.005, 0.22, 0.03], ...
        'String', info_str, ...
        'FontSize', 8, 'BackgroundColor', [1 1 1], 'FontWeight', 'bold');

    drawnow;

    %% ====== Figure 2: 误差收敛曲线 + CDF ======
    % 创建第二个图窗，包含两个子图：
    %   左子图：误差收敛曲线（滑动平均 10 帧）
    %   右子图：融合误差的累积分布函数 (CDF)
    fig2 = figure('Position', [50, 50, 1400, 750]);
    n_frames_total = length(trackSnapshots_R1);  % 总帧数
    frame_times = (0:n_frames_total-1) * params.dt_sec;  % 计算每帧的时间戳（秒）

    % 误差曲线的线型和颜色预设
    method_ls = {'-', '--', '-.', ':'};  % 线型：实线/虚线/点划线/点线
    method_clr = {[0 0 0], [1 0 0], [0 0 1], [0 0.7 0]};  % 颜色：黑/红/蓝/绿
    all_h_lines = []; all_line_names = {};  % 累积所有曲线句柄和名称

    subplot(1, 2, 1);  % 左子图：误差收敛曲线
    hold on; grid on;  % 保持坐标系，开启网格

    % 每条匹配对的最优融合算法画误差曲线
    for p = 1:length(matched_pairs)
        snaps_m = all_fused_snapshots{p, best_method_idx};  % 取出最优算法的融合快照
        % build_frame_errors_multi 计算逐帧误差序列
        errs = build_frame_errors_multi(snaps_m, truthTrajs, frame_times, []);
        % 只有有效点数 > 2 才绘制
        if ~isempty(errs) && sum(~isnan(errs)) > 2
            % movmean 计算滑动平均（窗口大小 10），平滑误差曲线
            smoothed = movmean(errs, 10, 'omitnan');
            h = plot(frame_times, smoothed, 'Color', method_clr{best_method_idx}, 'LineWidth', 1.5);
            all_h_lines(end+1) = h;  % 累积句柄
            all_line_names{end+1} = sprintf('%s Pair#%d', best_method, p);  % 累积名称
        end
    end

    % R1/R2 单站误差曲线
    % 遍历两个雷达站和三架飞机
    for r = 1:2
        if r == 1
            snaps = trackSnapshots_R1;  % R1 快照
            clr = [0 0 0.7];            % 深蓝色
        else
            snaps = trackSnapshots_R2;  % R2 快照
            clr = [0.7 0 0];            % 深红色
        end
        for ac = 1:length(truthTrajs)  % 遍历每架飞机
            % build_single_frame_errors_multi 计算单站对单个目标的逐帧误差
            fe = build_single_frame_errors_multi(snaps, truthTrajs{ac}, frame_times);
            if ~isempty(fe) && sum(~isnan(fe)) > 2
                smoothed = movmean(fe, 10, 'omitnan');  % 滑动平均平滑
                h = plot(frame_times, smoothed, ':', 'Color', clr, 'LineWidth', 1.5);
                all_h_lines(end+1) = h;
                all_line_names{end+1} = sprintf('R%d UKF Target%c', r, char('A'+ac-1));
            end
        end
    end

    % 左子图标注
    xlabel('时间 (s)'); ylabel('位置误差 (km)');  % x/y 轴标签
    title('误差收敛曲线 (滑动平均 10帧)');  % 子图标题
    legend(all_line_names, 'FontSize', 7, 'Location', 'best', 'NumColumns', 2);  % 图例，2列布局

    subplot(1, 2, 2);  % 右子图：CDF 累积分布
    hold on; grid on;
    % 遍历每个匹配对绘制 CDF
    for p = 1:length(matched_pairs)
        snaps_m = all_fused_snapshots{p, best_method_idx};
        errs = [];  % 初始化误差数组
        % 遍历融合快照的每一帧
        for k = 1:length(snaps_m)
            snap = snaps_m{k};  % 第 k 帧融合快照
            if isempty(snap.trackList), continue; end  % 无航迹则跳过
            ft = snap.trackList{1};  % 取第一条融合航迹
            if isnan(ft.lat), continue; end  % 无效航迹跳过
            best_d = inf;  % 初始化最小距离
            % 与所有真值目标比较，取最近的作为匹配
            for ac = 1:length(truthTrajs)
                truth_ac = truthTrajs{ac};  % 第 ac 架真值
                % interp1 插值得到该时刻的真值经度
                t_lon = interp1(truth_ac.time_sec, truth_ac.lon, frame_times(k), 'linear', 'extrap');
                % interp1 插值得到该时刻的真值纬度
                t_lat = interp1(truth_ac.time_sec, truth_ac.lat, frame_times(k), 'linear', 'extrap');
                if isnan(t_lat), continue; end  % 插值无效则跳过
                % 计算融合点与真值的大圆距离（km）
                d = sphere_utils_haversine_distance(ft.lon, ft.lat, t_lon, t_lat) / 1000;
                if d < best_d, best_d = d; end  % 取最小距离
            end
            if best_d < inf, errs(end+1) = best_d; end  % 累积有效误差
        end
        % 使用 ecdf 计算经验累积分布函数
        if ~isempty(errs)
            [f, x] = ecdf(errs);  % f=累积概率, x=误差值
            plot(x, f*100, 'Color', method_clr{best_method_idx}, 'LineWidth', 2);  % y 轴转为百分比
        end
    end
    xlabel('位置误差 (km)'); ylabel('累积概率 (%)');  % 右子图标注
    title('融合误差CDF');  % CDF 标题
    legend(best_method, 'FontSize', 8, 'Location', 'southeast');  % 显示最优算法名

    drawnow;  % 强制刷新图形
end

% =========================================================================
% 辅助函数
% =========================================================================

% =========================================================================
% extract_dets_multi — 从检测列表中按 aircraft_id 提取校准点迹坐标
% =========================================================================
% 遍历 detList 的所有帧，提取非杂波且有有效 lat/lon 的检测点
% mode='cal' 表示提取校准点迹（经过位置校正的检测）
function [lats, lons] = extract_dets_multi(detList, mode)
    lats = []; lons = [];  % 初始化输出数组
    for k = 1:length(detList)
        dets = detList{k};  % 第 k 帧的检测
        if isempty(dets), continue; end  % 空帧跳过
        for d = 1:length(dets)
            dp = dets(d);  % 第 d 个检测点
            if dp.is_clutter, continue; end  % 杂波跳过
            % 如果是 cal 模式且 lat 字段存在且有效，则累加坐标
            if strcmp(mode, 'cal') && isfield(dp, 'lat') && ~isnan(dp.lat)
                lats(end+1) = dp.lat;  % 累加纬度
                lons(end+1) = dp.lon;  % 累加经度
            end
        end
    end
end

% =========================================================================
% collect_active_tracks_multi — 按航迹ID分组收集历史点
% =========================================================================
% 遍历所有帧的快照，将同一航迹ID的各帧位置收集为连续序列。
% 每条航迹返回一个 struct：{id, lat_history, lon_history}
% trk.type==7 表示该航迹已被删除，跳过
function tracks = collect_active_tracks_multi(snaps)
    all_tracks = {};  % {id, lat_history, lon_history} 结构体 cell 数组
    for k = 1:length(snaps)
        snap = snaps{k};  % 第 k 帧快照
        if isempty(snap.trackList), continue; end  % 无航迹则跳过
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};  % 第 t 条航迹
            % type==7 表示航迹已终止，跳过；lat==NaN 表示无效航迹
            if trk.type == 7 || isnan(trk.lat), continue; end
            % 查找是否已有该ID的航迹
            found = false;  % 标记是否已找到
            for i = 1:length(all_tracks)
                if all_tracks{i}.id == trk.id  % ID 匹配
                    % 将当前位置追加到该航迹的历史序列末尾
                    all_tracks{i}.lat_history(end+1) = trk.lat;
                    all_tracks{i}.lon_history(end+1) = trk.lon;
                    found = true;
                    break;  % 找到后跳出内层循环
                end
            end
            % 如果没找到该ID，创建新的航迹记录
            if ~found
                all_tracks{end+1} = struct('id', trk.id, ...
                    'lat_history', trk.lat, 'lon_history', trk.lon);
            end
        end
    end
    tracks = all_tracks;  % 返回航迹列表
end

% =========================================================================
% collect_fused_positions_multi — 收集融合航迹位置历史
% =========================================================================
% 与 collect_active_tracks_multi 类似，但处理的是融合后的航迹数据
% 不需要过滤 type==7，因为融合数据中没有该字段
function tracks = collect_fused_positions_multi(snaps)
    all_tracks = {};  % 初始化航迹列表
    for k = 1:length(snaps)
        snap = snaps{k};  % 第 k 帧融合快照
        if isempty(snap.trackList), continue; end  % 无航迹跳过
        for t = 1:length(snap.trackList)
            ft = snap.trackList{t};  % 第 t 条融合航迹
            % 跳过经纬度无效的航迹
            if isnan(ft.lat) || isnan(ft.lon), continue; end
            found = false;  % 查找标记
            for i = 1:length(all_tracks)
                if all_tracks{i}.id == ft.id  % ID 匹配
                    all_tracks{i}.lat_history(end+1) = ft.lat;  % 累加纬度
                    all_tracks{i}.lon_history(end+1) = ft.lon;   % 累加经度
                    found = true;
                    break;
                end
            end
            if ~found
                all_tracks{end+1} = struct('id', ft.id, ...
                    'lat_history', ft.lat, 'lon_history', ft.lon);
            end
        end
    end
    tracks = all_tracks;
end

% =========================================================================
% try_set_visible_multi — 安全设置图层可见性
% =========================================================================
% 封装 set 调用，防止因句柄无效导致崩溃
function try_set_visible_multi(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end  % 逻辑值转字符串
        set(h, 'Visible', v);  % 设置可见性
    catch
        % 静默忽略错误（句柄已失效等情况）
    end
end

% =========================================================================
% toggle_all_cb_multi — 切换所有复选框状态
% =========================================================================
% 实现"全部隐藏"/"全部显示"按钮的切换逻辑
% 根据按钮当前文本判断要切换的方向，并同步更新所有复选框和图层
function toggle_all_cb_multi(btn, cb, h_all)
    if strcmp(btn.String, '全部隐藏')  % 当前是全隐藏状态
        new_val = 0; btn.String = '全部显示';  % 切换到全部显示
    else
        new_val = 1; btn.String = '全部隐藏';  % 切换到全部隐藏
    end
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', new_val); end  % 更新复选框状态
        try_set_visible_multi(h_all(i), new_val);  % 更新图层可见性
    end
end

% =========================================================================
% show_all_cb_multi — 显示所有图层
% =========================================================================
% 将所有复选框设为选中，所有图层设为可见
function show_all_cb_multi(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end  % 选中复选框
        try_set_visible_multi(h_all(i), 1);  % 显示图层
    end
end

% =========================================================================
% get_match_pair_multi — 兼容 cell 和 struct 数组的配对访问
% =========================================================================
% matched_pairs 可能是 cell 数组或 struct 数组，此函数统一访问接口
function mp = get_match_pair_multi(matched_pairs, p)
    if iscell(matched_pairs)
        mp = matched_pairs{p};  % cell 数组用 {} 访问
    else
        mp = matched_pairs(p);  % struct 数组用 () 访问
    end
end

% =========================================================================
% build_frame_errors_multi — 构建逐帧误差序列
% =========================================================================
% 对于每个融合快照帧，找到最近的真值目标，计算大圆距离作为误差
% 返回长度为 n_frames 的数组，无效帧为 NaN
function errs = build_frame_errors_multi(fused_snaps, truthTrajs, frame_times, mp)
    errs = nan(1, length(fused_snaps));  % 初始化为 NaN
    for k = 1:length(fused_snaps)
        snap = fused_snaps{k};  % 第 k 帧融合快照
        if isempty(snap.trackList), continue; end  % 无航迹跳过
        ft = snap.trackList{1};  % 取第一条融合航迹
        if isnan(ft.lat), continue; end  % 无效航迹跳过
        best_d = inf;  % 初始化最小距离
        % 与所有真值目标比较，取最近的作为匹配
        for ac = 1:length(truthTrajs)
            % interp1 线性插值得到该时刻的真值经纬度
            t_lon = interp1(truthTrajs{ac}.time_sec, truthTrajs{ac}.lon, frame_times(k), 'linear', 'extrap');
            t_lat = interp1(truthTrajs{ac}.time_sec, truthTrajs{ac}.lat, frame_times(k), 'linear', 'extrap');
            if isnan(t_lat), continue; end  % 插值无效跳过
            % 计算大圆距离（km）
            d = sphere_utils_haversine_distance(ft.lon, ft.lat, t_lon, t_lat) / 1000;
            if d < best_d, best_d = d; end  % 取最小
        end
        if best_d < inf, errs(k) = best_d; end  % 记录有效误差
    end
end

% =========================================================================
% build_single_frame_errors_multi — 构建单站逐帧误差序列
% =========================================================================
% 与 build_frame_errors_multi 类似，但处理的是单站 UKF 航迹
% 需要遍历该帧中的所有航迹，取最近的作为匹配
function errs = build_single_frame_errors_multi(snaps, truth_ac, frame_times)
    errs = nan(1, length(snaps));  % 初始化为 NaN
    for k = 1:length(snaps)
        snap = snaps{k};  % 第 k 帧快照
        if isempty(snap.trackList), continue; end  % 无航迹跳过
        % 插值得到该时刻的真值经纬度
        t_lon = interp1(truth_ac.time_sec, truth_ac.lon, frame_times(k), 'linear', 'extrap');
        t_lat = interp1(truth_ac.time_sec, truth_ac.lat, frame_times(k), 'linear', 'extrap');
        if isnan(t_lat), continue; end  % 插值无效跳过
        best_d = inf;  % 初始化最小距离
        % 遍历该帧中的所有航迹，取最近的
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end  % 已终止航迹跳过
            if isnan(trk.lat), continue; end  % 无效航迹跳过
            d = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
            if d < best_d, best_d = d; end
        end
        if best_d < inf, errs(k) = best_d; end  % 记录有效误差
    end
end
