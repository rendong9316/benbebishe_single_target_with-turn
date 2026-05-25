% =========================================================================
% plot_combined_tracks.m
% =========================================================================
%
% 【功能概述】
%   绘制综合航迹对比图：在 darkwater 暗底图地理坐标系上分层叠加
%   目标真实航迹、R1/R2 原始（校准前）关联点迹、R1/R2 校准后关联点迹、
%   以及 R1/R2 UKF 滤波航迹。右侧提供复选框图层控制器，支持交互式
%   切换各图层的显隐状态。
%
% 【数学原理】
%   1. 关联点迹 (Associated Detections)：指通过数据关联算法（如 NN、
%      PDA、JPDA 等）与跟踪器当前状态关联上的量测点迹。并非所有
%      检测点迹都会被关联——只有落在跟踪门内的点迹才可能被关联。
%   2. 跟踪状态结构体 (trackState)：包含每帧的跟踪状态，关联点迹的
%      原始位置(raw_lat/raw_lon)和校准后位置(det_lat/det_lon)，
%      以及 UKF 滤波后的估计位置(lat/lon)。
%   3. 处理链条：原始检测 → 关联判断 → 校准转换 → UKF 滤波，
%      本图展示了处理链条上各阶段的位置信息在地图上的对比。
%
% 【输入参数】
%   true_track     - Nx2 矩阵，真值航迹 [lon, lat]
%   detList_R1     - R1 检测结果元胞数组
%   detList_R2     - R2 检测结果元胞数组
%   trackState_R1  - R1 跟踪状态元胞数组，每帧一个结构体
%   trackState_R2  - R2 跟踪状态元胞数组
%   params         - 仿真参数字段结构体
%   out_dir        - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig3_combined_tracks.png  - 综合航迹对比图
%
% 【调用关系】
%   被调用: 主仿真脚本
%   调用:   extract_associated_dets()     (本文件内部)
%           extract_raw_associated_dets() (本文件内部)
%           extract_filtered_track()       (本文件内部)
%           sum_assc_clutter()             (本文件内部)
%
% =========================================================================

function plot_combined_tracks(true_track, detList_R1, detList_R2, ...
        trackState_R1, trackState_R2, params, out_dir)

    fig = figure('Position', [50, 50, 1400, 750]);

    % ---- 左侧地理图：geoaxes 占 70% 宽度 ----
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    % 从跟踪状态中提取各层数据：关联校准点迹、关联原始点迹、滤波航迹
    [assc1_lat, assc1_lon] = extract_associated_dets(trackState_R1);
    [assc2_lat, assc2_lon] = extract_associated_dets(trackState_R2);
    [raw1_lat, raw1_lon] = extract_raw_associated_dets(trackState_R1);
    [raw2_lat, raw2_lon] = extract_raw_associated_dets(trackState_R2);
    [filt1_lat, filt1_lon] = extract_filtered_track(trackState_R1);
    [filt2_lat, filt2_lon] = extract_filtered_track(trackState_R2);

    % ---- 图层 1: 真实航迹 (亮黄虚线，在暗底图上对比度高) ----
    h1 = geoplot(ax, true_track(:,2), true_track(:,1), 'y--', ...
        'LineWidth', 2, 'DisplayName', '真实航迹');

    % ---- 图层 2: R1 原始（校准前）关联点迹连线 (淡蓝色虚线+圆点) ----
    % 原始点迹尚未经过椭球校准，可能存在系统性偏差
    h2 = geoplot(ax, raw1_lat, raw1_lon, '--', ...
        'Color', [0.4, 0.6, 1.0], 'LineWidth', 1.2, 'Marker', 'o', ...
        'MarkerSize', 5, 'MarkerFaceColor', [0.4, 0.6, 1.0], ...
        'DisplayName', 'R1 原始点迹');

    % ---- 图层 3: R2 原始（校准前）关联点迹连线 (淡红色虚线+圆点) ----
    h3 = geoplot(ax, raw2_lat, raw2_lon, '--', ...
        'Color', [1.0, 0.6, 0.6], 'LineWidth', 1.2, 'Marker', 'o', ...
        'MarkerSize', 5, 'MarkerFaceColor', [1.0, 0.6, 0.6], ...
        'DisplayName', 'R2 原始点迹');

    % ---- 图层 4: R1 校准后关联点迹连线 (蓝色实线+填充圆点) ----
    % 校准后已从 (Rg, Az) 转为地理坐标 (lat, lon)，消除了椭球偏差
    h4 = geoplot(ax, assc1_lat, assc1_lon, 'bo-', ...
        'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'b', ...
        'DisplayName', 'R1 校准后点迹');

    % ---- 图层 5: R2 校准后关联点迹连线 (红色实线+填充圆点) ----
    h5 = geoplot(ax, assc2_lat, assc2_lon, 'ro-', ...
        'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'r', ...
        'DisplayName', 'R2 校准后点迹');

    % ---- 图层 6: R1 UKF滤波航迹 (青色粗实线) ----
    % 青色(cyan)在暗底图上醒目，且明显区别于蓝色的校准点迹
    h6 = geoplot(ax, filt1_lat, filt1_lon, 'c-', ...
        'LineWidth', 2.5, 'DisplayName', 'R1 UKF滤波');

    % ---- 图层 7: R2 UKF滤波航迹 (品红粗实线) ----
    % 品红(magenta)在暗底图上醒目，区别于红色的校准点迹
    h7 = geoplot(ax, filt2_lat, filt2_lon, 'm-', ...
        'LineWidth', 2.5, 'DisplayName', 'R2 UKF滤波');

    % ---- 站点标记 ----
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 8, 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 8, 'DisplayName', 'Tx2');

    % ---- 起点/终点标记 ----
    geoplot(ax, true_track(1,2), true_track(1,1), 'go', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    geoplot(ax, true_track(end,2), true_track(end,1), 'gx', ...
        'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '终点');

    title(ax, '双基地雷达航迹综合对比');

    % ---- 右侧图层控制面板：7 个复选框对应 7 个图层 ----
    handles = {h1, h2, h3, h4, h5, h6, h7};
    labels = {'真实航迹', 'R1 原始点迹(校准前)', 'R2 原始点迹(校准前)', ...
              'R1 校准后点迹', 'R2 校准后点迹', 'R1 UKF滤波', 'R2 UKF滤波'};

    for i = 1:7
        ypos = 0.92 - (i-1) * 0.09;
        uicontrol('Parent', fig, 'Style', 'checkbox', ...
            'String', labels{i}, 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.06], ...
            'FontSize', 10, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) try_set_visible(handles{i}, src.Value));
    end

    % 底部统计信息：关联点迹数量、虚警数量、原始点迹总数、仿真参数
    n1 = length(assc1_lat); n2 = length(assc2_lat);
    n1c = sum_assc_clutter(trackState_R1);
    n2c = sum_assc_clutter(trackState_R2);
    nr1 = length(raw1_lat); nr2 = length(raw2_lat);
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.76, 0.01, 0.22, 0.06], ...
        'String', sprintf('R1关联:%d(虚警%d) R2关联:%d(虚警%d)\n原始点迹 R1:%d R2:%d  Pd=%.0f%% Pfa=%.3f', ...
        n1, n1c, n2, n2c, nr1, nr2, params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig3_combined_tracks.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig3_combined_tracks.png'));
    end
    fprintf('  综合航迹图已保存: fig3_combined_tracks.png\n');
end

% =========================================================================
% try_set_visible - 安全地设置图形句柄的可见性
% =========================================================================
function try_set_visible(h, val)
    try
        if val, v = 'on'; else, v = 'off'; end
        set(h, 'Visible', v);
    catch
    end
end

% =========================================================================
% extract_associated_dets - 从 trackState 中提取校准后的关联点迹位置
%
% 【说明】遍历所有帧，筛选出 associated==true 且 det_lat/det_lon 非 NaN
%        的状态，提取校准后的关联检测点经纬度。这些点迹是已经通过椭球
%        校准从 (Rg,Az) 转为地理坐标的量测值。
% =========================================================================
function [lats, lons] = extract_associated_dets(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~s.associated, continue; end
        if isfield(s, 'det_lat') && ~isnan(s.det_lat)
            lats(end+1) = s.det_lat;
            lons(end+1) = s.det_lon;
        end
    end
end

% =========================================================================
% extract_raw_associated_dets - 从 trackState 中提取原始（校准前）的关联点迹位置
%
% 【说明】提取 det_raw_lat/det_raw_lon 字段，这些是未经椭球校准的原始
%        经纬度。与校准后的点迹对比可以观察校准过程对定位精度的改善。
% =========================================================================
function [lats, lons] = extract_raw_associated_dets(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~s.associated, continue; end
        if isfield(s, 'det_raw_lat') && ~isnan(s.det_raw_lat)
            lats(end+1) = s.det_raw_lat;
            lons(end+1) = s.det_raw_lon;
        end
    end
end

% =========================================================================
% sum_assc_clutter - 统计所有帧中关联到的虚警（杂波）点迹数量
%
% 【说明】虽然关联算法主要匹配真实目标，但由于杂波可能误入跟踪门，
%        仍有部分杂波被关联。统计这个数量可以评估关联算法的抗杂波能力。
% =========================================================================
function n = sum_assc_clutter(stateList)
    n = 0;
    for k = 1:length(stateList)
        s = stateList{k};
        if ~isempty(s) && s.associated && s.assc_is_clutter
            n = n + 1;
        end
    end
end

% =========================================================================
% extract_filtered_track - 从 trackState 中提取 UKF 滤波后的航迹点
%
% 【说明】提取每帧跟踪状态中 UKF 估计的位置(lat/lon)，这些是经过
%        卡尔曼滤波平滑处理后的目标位置估计，精度应高于原始点迹。
%        仅提取 lat/lon 非 NaN 的有效帧。
% =========================================================================
function [lats, lons] = extract_filtered_track(stateList)
    lats = []; lons = [];
    for k = 1:length(stateList)
        s = stateList{k};
        if isempty(s) || ~isfield(s, 'lat') || isnan(s.lat), continue; end
        lats(end+1) = s.lat;
        lons(end+1) = s.lon;
    end
end
