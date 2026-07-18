% =========================================================================
% plot_scene_overview_multi.m — 多目标场景总览图
% =========================================================================
% 【功能】
%   在地理坐标系地图上绘制多目标仿真场景的总览图，包含：
%     1. 两个雷达接收站（R1/R2）和两个发射站（Tx1/Tx2）的标记
%     2. 两部雷达的波束扇形覆盖范围（彩色虚线框）
%     3. 所有真值航迹（A/B/C，不同颜色）
%     4. 每架飞机的起点（绿色圆点）和终点（红色叉号）
%     5. 仿真参数标注（Pd、Pfa、dt、威力范围）
%
%   底图使用 darkwater 暗色主题以增强可视对比度。
%
% 【输入】
%   true_track_A/B/C — 真值航迹 [lon, lat] 矩阵（N×2）
%   params           — 仿真参数结构体
%   out_dir          — 输出目录路径
% =========================================================================
function plot_scene_overview_multi(true_track_A, true_track_B, true_track_C, params, out_dir)
    % 创建地理坐标图窗，尺寸 1400x750 像素
    fig = figure('Position', [50, 50, 1400, 750]);
    try
        % 尝试使用 darkwater 暗色底图，与浅色航迹对比度高
        ax = geoaxes('Basemap', 'darkwater');
    catch
        % 如果地理信息工具箱不支持 darkwater，降级为默认底图
        ax = geoaxes();
    end
    hold(ax, 'on');  % 保持坐标系，允许后续多层叠加绘图

    % ---- 雷达站标记 ----
    % 接收站：方形标记，R1 蓝色，R2 红色
    % bs 表示蓝色填充方块，MarkerFaceColor 控制填充色
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1 Rx');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2 Rx');
    % 发射站：三角标记，R1 蓝色，R2 红色
    % b^ 表示蓝色上三角，与接收站的方块区分功能角色
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Tx2');

    % ---- 波束扇形覆盖区域 ----
    % draw_beam_sector_geoax 在地理坐标轴上绘制扇形边界线
    % R1 淡蓝色 [0.3 0.6 1.0]，R2 淡红色 [1.0 0.4 0.4]
    % 半宽 15°，距离范围 1000-2000km
    draw_beam_sector_geoax(ax, params.radar1_lat, params.radar1_lon, ...
        params.radar1_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [0.3 0.6 1.0]);
    draw_beam_sector_geoax(ax, params.radar2_lat, params.radar2_lon, ...
        params.radar2_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [1.0 0.4 0.4]);

    % ---- 真值航迹绘制 ----
    % 三架飞机用不同颜色：A=绿色(g), B=品红(m), C=青色(c)
    % 三种颜色在暗色底图上对比度高且互不混淆
    track_colors = {'g', 'm', 'c'};
    track_lines  = {'-', '-', '-'};   % 全部使用实线
    track_names  = {'Target A', 'Target B', 'Target C'};
    track_data   = {true_track_A, true_track_B, true_track_C};
    for i = 1:3
        tt = track_data{i};  % 取出第 i 架飞机的真值航迹 [lon, lat]
        % geoplot 参数顺序为 (lat, lon)，注意与矩阵列顺序相反
        % tt(:,2)=lat 作为 y 轴，tt(:,1)=lon 作为 x 轴
        geoplot(ax, tt(:,2), tt(:,1), strcat(track_lines{i}), 'Color', track_colors{i}, 'LineWidth', 2, ...
            'DisplayName', track_names{i});
        % 起点标记：绿色圆点 (go)，标识每架飞机的起始位置
        geoplot(ax, tt(1,2), tt(1,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        % 终点标记：红色叉号 (rx)，颜色与航迹一致便于关联
        geoplot(ax, tt(end,2), tt(end,1), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'Color', track_colors{i});
    end

    % ---- 标题和参数标注 ----
    % 主标题说明场景类型，副标题显示关键仿真参数
    title(ax, '双基地外辐射源雷达多目标仿真场景');
    subtitle(ax, sprintf('Pd=%.0f%%, Pfa=%.3f, dt=%.0fs, %d-%d km', ...
        params.detection_probability*100, params.false_alarm_rate, ...
        params.dt_sec, params.range_min_km, params.range_max_km));
    legend(ax, 'Location', 'best');  % 自动选择最不遮挡数据的图例位置
    drawnow;  % 强制刷新图形显示
end

% =========================================================================
% draw_beam_sector_geoax — 在 geoaxes 上绘制波束扇形边界
% =========================================================================
% 【功能】
%   从接收站出发，在指定的方位角范围和距离范围内绘制扇形边界线。
%   由于 geoaxes 不支持 RGBA 透明度，这里使用纯色虚线绘制边界。
%
% 【输入】
%   ax        — geoaxes 坐标轴
%   rx_lat/lon — 接收站经纬度
%   center_az — 波束中心方位角（度）
%   width     — 波束全宽度（度）
%   r_min/max — 距离边界（米）
%   color     — [R G B] 颜色向量
% =========================================================================
function draw_beam_sector_geoax(ax, rx_lat, rx_lon, center_az, width, r_min, r_max, color)
    % 在波束宽度范围内均匀采样 20 个方位角点
    % linspace 从 center_az-width/2 到 center_az+width/2 线性插值
    az_edges = linspace(center_az - width/2, center_az + width/2, 20);
    % 预分配内存：内弧和外弧的经纬度数组
    % 提前分配 zeros 避免循环中动态扩展数组，提升性能
    lats_inner = zeros(1, length(az_edges));
    lons_inner = zeros(1, length(az_edges));
    lats_outer = zeros(1, length(az_edges));
    lons_outer = zeros(1, length(az_edges));
    % 逐个方位角计算内弧和外弧的目标点经纬度
    % sphere_utils_destination_point 使用球面大地线公式，从接收站沿给定方位角
    % 前进 r_min(内弧) 或 r_max(外弧) 距离，得到目标点经纬度
    for i = 1:length(az_edges)
        [lons_inner(i), lats_inner(i)] = sphere_utils_destination_point(...
            rx_lon, rx_lat, r_min, az_edges(i));
        [lons_outer(i), lats_outer(i)] = sphere_utils_destination_point(...
            rx_lon, rx_lat, r_max, az_edges(i));
    end
    % 绘制内弧和外弧（虚线）
    % '--' 表示虚线样式，Color 指定扇形颜色，LineWidth=1 细线
    % 注意 geoplot 参数顺序为 (lat, lon)
    geoplot(ax, lats_inner, lons_inner, '--', 'Color', color, 'LineWidth', 1);
    geoplot(ax, lats_outer, lons_outer, '--', 'Color', color, 'LineWidth', 1);
    % 绘制两条径向边界线：内弧 → 外弧在波束起始和结束方位角处
    % 这两条线封闭了扇形的左右两侧，形成完整的扇形轮廓
    for az_edge = [center_az - width/2, center_az + width/2]
        [lon1, lat1] = sphere_utils_destination_point(rx_lon, rx_lat, r_min, az_edge);
        [lon2, lat2] = sphere_utils_destination_point(rx_lon, rx_lat, r_max, az_edge);
        % 用实线连接内外弧端点
        geoplot(ax, [lat1 lat2], [lon1 lon2], '-', 'Color', color, 'LineWidth', 1);
    end
end
