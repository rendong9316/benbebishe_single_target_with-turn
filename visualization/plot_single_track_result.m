% =========================================================================
% plot_single_track_result.m
% =========================================================================
%
% 【功能概述】
%   绘制单目标双基地雷达航迹综合对比图。在 darkwater 暗色底图的
%   地理坐标系上叠加以下内容：目标真实航迹、R1/R2 的原始（校准前）
%   点迹和校准后点迹、以及每个雷达独立 UKF 滤波后的航迹。
%   右侧提供复选框图层控制面板，支持交互式切换各图层的显示/隐藏。
%
% 【数学原理】
%   1. 外辐射源雷达点迹校准：
%      原始量测在群距离/方位角坐标系(Rg, Az)下，需通过椭球相交
%      计算转换为地理经纬度坐标。校准后的点迹 lat/lon 已经与真实
%      航迹在同一坐标系下。
%   2. UKF（无迹卡尔曼滤波）：
%      对校准后的点迹序列进行滤波，估计目标在经纬度空间中的状态
%      （位置和速度）。UKF 通过确定性采样点（Sigma 点）来近似
%      非线性状态转移和量测方程，避免了 EKF 的线性化近似误差。
%   3. 航迹提取：从 trackSnapshots 快照结构中提取每个航迹 ID 的
%      历史经纬度序列，形成完整的航迹线。
%
% 【输入参数】
%   true_track        - Nx2 矩阵，真实航迹 [lon, lat] 序列
%   detList_R1        - R1 检测结果元胞数组，每帧一组检测结构体
%   detList_R2        - R2 检测结果元胞数组
%   trackSnapshots_R1 - R1 跟踪快照元胞数组，含 trackList 等字段
%   trackSnapshots_R2 - R2 跟踪快照元胞数组
%   params            - 仿真参数字段结构体
%   out_dir           - 输出目录路径字符串
%
% 【输出】
%   屏幕打印保存信息，生成文件：
%       fig4_single_track_result.png  - 单目标跟踪综合图
%
% 【调用关系】
%   被调用: 主仿真脚本
%   调用:   extract_dets()          (本文件内部辅助函数)
%           collect_active_tracks() (本文件内部辅助函数)
%           geoplot()               (MATLAB 内置)
%
% =========================================================================

function plot_single_track_result(true_track, detList_R1, detList_R2, ...
        trackSnapshots_R1, trackSnapshots_R2, params, out_dir)

    % ---- 创建图窗和地理坐标轴 ----
    % 地理坐标轴(geoaxes)放置在左侧 70% 区域，右侧留给图层控件
    fig = figure('Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    % 初始化图层句柄和名称容器
    h_all = [];        % 图形句柄数组
    layer_names = {};  % 图层中文名称（用于复选框标签）

    % =====================================================================
    % 图层 1: 真值航迹 (绿色虚线+方块标记)
    % '--s' = 虚线 + 方块标记，绿色醒目
    % =====================================================================
    h_truth = geoplot(ax, true_track(:,2), true_track(:,1), '--s', ...
        'Color', 'g', 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'g', ...
        'DisplayName', '真值');
    h_all(end+1) = h_truth;
    layer_names{end+1} = '真值航迹';

    % =====================================================================
    % 图层 2: R1 原始点迹（校准前的量测，淡蓝色虚线和空心圆标记）
    % 原始点迹是在 (Rg, Az) 空间中的量测，未经椭球校准
    % =====================================================================
    [r1_raw_lat, r1_raw_lon] = extract_dets(detList_R1, 'raw');
    if ~isempty(r1_raw_lat)
        h = geoplot(ax, r1_raw_lat, r1_raw_lon, '--o', ...
            'Color', [0.4 0.6 1.0], 'LineWidth', 1.0, 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.4 0.6 1.0], 'DisplayName', 'R1 原始点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 原始点迹(校准前)';
    end

    % =====================================================================
    % 图层 3: R1 校准后点迹（蓝色实线和实心圆标记）
    % 校准后的点迹已经转换为地理经纬度坐标
    % =====================================================================
    [r1_cal_lat, r1_cal_lon] = extract_dets(detList_R1, 'cal');
    if ~isempty(r1_cal_lat)
        h = geoplot(ax, r1_cal_lat, r1_cal_lon, '-o', ...
            'Color', [0.0 0.4 1.0], 'LineWidth', 1.2, 'MarkerSize', 5, ...
            'MarkerFaceColor', 'b', 'DisplayName', 'R1 校准点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R1 校准后点迹';
    end

    % =====================================================================
    % 图层 4: R1 UKF 滤波航迹（深蓝色实线+圆点标记）
    % 从快照中搜集各个航迹 ID 的历史位置，每个航迹 ID 画一条线
    % =====================================================================
    r1_tracks = collect_active_tracks(trackSnapshots_R1);
    for t = 1:length(r1_tracks)
        trk = r1_tracks{t};
        % 至少需要 3 个点才能形成有意义的航迹线
        if length(trk.lat_history) > 2
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'b-o', ...
                'LineWidth', 2.0, 'MarkerSize', 5, 'MarkerFaceColor', 'b', ...
                'DisplayName', sprintf('R1 UKF#%d', trk.id));
            h_all(end+1) = h;
            layer_names{end+1} = sprintf('R1 UKF航迹#%d', trk.id);
        end
    end

    % =====================================================================
    % 图层 5: R2 原始点迹（淡红色虚线和空心圆标记）
    % =====================================================================
    [r2_raw_lat, r2_raw_lon] = extract_dets(detList_R2, 'raw');
    if ~isempty(r2_raw_lat)
        h = geoplot(ax, r2_raw_lat, r2_raw_lon, '--o', ...
            'Color', [1.0 0.6 0.6], 'LineWidth', 1.0, 'MarkerSize', 4, ...
            'MarkerFaceColor', [1.0 0.6 0.6], 'DisplayName', 'R2 原始点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 原始点迹(校准前)';
    end

    % =====================================================================
    % 图层 6: R2 校准后点迹（红色实线和实心圆标记）
    % =====================================================================
    [r2_cal_lat, r2_cal_lon] = extract_dets(detList_R2, 'cal');
    if ~isempty(r2_cal_lat)
        h = geoplot(ax, r2_cal_lat, r2_cal_lon, '-o', ...
            'Color', [1.0 0.2 0.2], 'LineWidth', 1.2, 'MarkerSize', 5, ...
            'MarkerFaceColor', 'r', 'DisplayName', 'R2 校准点迹');
        h_all(end+1) = h;
        layer_names{end+1} = 'R2 校准后点迹';
    end

    % =====================================================================
    % 图层 7: R2 UKF 滤波航迹（红色实线+上三角标记）
    % =====================================================================
    r2_tracks = collect_active_tracks(trackSnapshots_R2);
    for t = 1:length(r2_tracks)
        trk = r2_tracks{t};
        if length(trk.lat_history) > 2
            h = geoplot(ax, trk.lat_history, trk.lon_history, 'r-^', ...
                'LineWidth', 2.0, 'MarkerSize', 5, 'MarkerFaceColor', 'r', ...
                'DisplayName', sprintf('R2 UKF#%d', trk.id));
            h_all(end+1) = h;
            layer_names{end+1} = sprintf('R2 UKF航迹#%d', trk.id);
        end
    end

    % =====================================================================
    % 站点标记：接收站(方块)、发射站(上三角)
    % =====================================================================
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'Tx2');

    title(ax, '单目标双基地雷达航迹综合对比');
    legend(ax, 'Location', 'northeastoutside');

    % =====================================================================
    % 右侧图层控制面板：复选框 + 全部显示/隐藏按钮
    % 每个复选框绑定一个图层的可见性回调
    % =====================================================================
    n_layers = length(layer_names);
    cb = gobjects(1, n_layers);  % 复选框句柄

    for i = 1:n_layers
        % 从上到下排列复选框，每个高 0.045 (归一化单位)
        ypos = 0.92 - (i-1) * 0.045;
        if ypos < 0.05, break; end  % 超出可视区域则停止
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', layer_names{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(h_all(i), src.Value));
    end

    % 全部显示/隐藏切换按钮
    btn_bottom = 0.92 - n_layers * 0.045 - 0.01;
    if btn_bottom > 0.02
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部隐藏', ...
            'Units', 'normalized', 'Position', [0.76, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(src, ~) toggle_all_cb(src, cb, h_all));
        uicontrol('Parent', fig, 'Style', 'pushbutton', ...
            'String', '全部显示', ...
            'Units', 'normalized', 'Position', [0.87, btn_bottom, 0.10, 0.04], ...
            'FontSize', 9, ...
            'Callback', @(~, ~) show_all_cb(cb, h_all));
    end

    % 底部统计信息条：航迹数量、检测概率、虚警概率
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.005, 0.22, 0.03], ...
        'String', sprintf('R1:%d航迹 R2:%d航迹 | Pd=%.0f%% Pfa=%.3f', ...
        length(r1_tracks), length(r2_tracks), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig4_single_track_result.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig4_single_track_result.png'));
    end
    fprintf('  单目标跟踪综合图已保存: fig4_single_track_result.png\n');
end

% =========================================================================
% extract_dets - 从检测列表中提取指定模式下的经纬度坐标
%
% 【参数】
%   detList - 检测结果元胞数组
%   mode    - 'raw' 提取 raw_lat/raw_lon（校准前），
%             'cal' 提取 lat/lon（校准后）
%
% 【说明】
%   跳过 is_clutter=true 的虚警点，仅提取真实目标的检测。
%   原始点迹(raw)对应的经纬度可能包含系统性偏差（未做椭球校正），
%   校准后(cal)的点迹则已校正到地理坐标系。
% =========================================================================
function [lats, lons] = extract_dets(detList, mode)
    lats = []; lons = [];
    for k = 1:length(detList)
        dets = detList{k};
        for d = 1:length(dets)
            dp = dets(d);
            if dp.is_clutter, continue; end  % 过滤杂波
            if strcmp(mode, 'raw') && isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                lats(end+1) = dp.raw_lat;
                lons(end+1) = dp.raw_lon;
            elseif strcmp(mode, 'cal') && isfield(dp, 'lat') && ~isnan(dp.lat)
                lats(end+1) = dp.lat;
                lons(end+1) = dp.lon;
            end
        end
    end
end

% =========================================================================
% collect_active_tracks - 从快照中搜集每个航迹 ID 的全部历史位置
%
% 【说明】
%   使用 containers.Map 按航迹 ID 聚合所有帧的位置数据。
%   跳过 type==7 的无效航迹状态。
%   最终按航迹 ID 组织成一个结构体数组，每个结构体包含该 ID 的
%   完整经纬度历史序列。
% =========================================================================
function tracks = collect_active_tracks(snapshots)
    track_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap.trackList), continue; end
        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end  % type=7 表示无效/终止状态
            tid = trk.id;
            if ~track_map.isKey(tid)
                % 首次遇到该航迹 ID，初始化记录
                track_map(tid) = struct('id', tid, 'lat_history', [], 'lon_history', []);
            end
            rec = track_map(tid);
            rec.lat_history(end+1) = trk.lat;
            rec.lon_history(end+1) = trk.lon;
            track_map(tid) = rec;
        end
    end
    tracks = values(track_map);  % 返回所有航迹记录
end

% =========================================================================
% try_set_visible - 安全地设置图形句柄的可见性
%   使用 try-catch 包裹以避免无效句柄导致的运行错误
% =========================================================================
function try_set_visible(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

% =========================================================================
% toggle_all_cb - "全部隐藏/全部显示"切换按钮的回调
%   切换按钮文字并同步更新所有复选框和图层可见性
% =========================================================================
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

% =========================================================================
% show_all_cb - "全部显示"按钮的回调
% =========================================================================
function show_all_cb(cb, h_all)
    for i = 1:length(cb)
        if cb(i) ~= 0, set(cb(i), 'Value', 1); end
        try_set_visible(h_all(i), 1);
    end
end
