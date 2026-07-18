% =========================================================================
% plot_scene_overview.m
% =========================================================================
%
% 【功能概述】
%   绘制双基地外辐射源雷达仿真场景的总览图。使用地理坐标系(geoplot)在
%   地图上显示：两个接收站(R1/R2)及其关联发射站(Tx1/Tx2)的位置、各站点
%   对应的波束扇形覆盖范围、以及目标的真实航迹。底图使用 darkwater 暗色
%   主题以增强可视对比度。
%
% 【数学原理】
%   1. 波束扇形(Beam Sector)：以接收站为中心，在给定的方位角范围
%      [center_az - width/2, center_az + width/2] 内，按距离边界
%      [r_min, r_max] 绘制扇形区域。扇形边界点通过球面大地线
%      (Geodesic) 计算得到，基于 WGS-84 椭球模型或球面近似。
%   2. 大圆距离计算：使用 sphere_utils_destination_point 函数，根据
%      起点经纬度、方位角和距离，计算目标点的经纬度。
%
% 【输入参数】
%   true_track       - Nx2 矩阵，真实航迹的经纬度序列 [lon, lat]
%   params           - 仿真参数字段结构体，至少包含以下字段：
%       .radar1_lat / .radar1_lon        - R1 接收站经纬度
%       .radar2_lat / .radar2_lon        - R2 接收站经纬度
%       .radar1_tx_lat / .radar1_tx_lon  - R1 关联发射站经纬度
%       .radar2_tx_lat / .radar2_tx_lon  - R2 关联发射站经纬度
%       .radar1_beam_center_deg          - R1 波束中心方位角 (deg)
%       .radar2_beam_center_deg          - R2 波束中心方位角 (deg)
%       .beam_width_deg                  - 波束宽度 (deg)
%       .range_min_m / .range_max_m      - 威力范围边界 (m)
%       .range_min_km / .range_max_km    - 威力范围边界 (km，仅用于标注)
%       .detection_probability           - 检测概率 Pd
%       .false_alarm_rate                - 虚警概率 Pfa
%       .dt_sec                          - 帧间隔时间 (s)
%   out_dir          - 字符串，输出图片的目录路径
%
% 【输出】
%   屏幕打印保存信息，并在 out_dir 中生成文件：
%       fig1_scene_overview.png   - 场景总览图（分辨率 200 DPI）
%
% 【调用关系】
%   被调用: 主仿真脚本 main.m 或 run_simulation.m
%   调用:   sphere_utils_destination_point()  (球面目标点计算)
%
% =========================================================================

function plot_scene_overview(true_track, params, out_dir)
    % 创建图窗，指定位置和大小：左下角(50,50)，宽1400px，高750px
    % Position 格式为 [left, bottom, width, height]，单位像素
    fig = figure('Position', [50, 50, 1400, 750]);

    % 尝试使用 darkwater 暗色主题底图；若不可用则使用默认底图
    % darkwater 底图为深色海洋地图，与浅色航迹线条对比度高
    try
        ax = geoaxes('Basemap', 'darkwater');
    catch
        ax = geoaxes();
    end
    hold(ax, 'on');  % 保持坐标系，允许后续多层叠加绘图

    % ---- 接收站标记 (蓝色/红色方块，填充) ----
    % R1 接收站：蓝色方块(bs)，MarkerSize=12 控制大小，MarkerFaceColor 控制填充
    % bs 表示 square marker with solid fill，b 表示蓝色
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'R1');
    % R2 接收站：红色方块(rs)，rs 表示红色填充方块
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'R2');

    % ---- 发射站标记 (蓝色/红色上三角，不填充) ----
    % 发射站用三角(^)区分于接收站的方块(s)，表明功能角色的不同
    % b^ 表示蓝色上三角，MarkerFaceColor 控制填充色
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Tx1');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Tx2');

    % ---- R1 波束扇形覆盖区域 (淡蓝色半透明) ----
    % 颜色 [0.3 0.6 1.0] 在暗底图上对比度好，透明度 0.8
    % draw_beam_sector 在接收站位置绘制扇形边界线
    draw_beam_sector(ax, params.radar1_lat, params.radar1_lon, ...
        params.radar1_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [0.3 0.6 1.0]);

    % ---- R2 波束扇形覆盖区域 (淡红色半透明) ----
    % 使用暖色调与 R1 的冷色调形成对比，便于区分两个雷达的覆盖范围
    draw_beam_sector(ax, params.radar2_lat, params.radar2_lon, ...
        params.radar2_beam_center_deg, params.beam_width_deg, ...
        params.range_min_m, params.range_max_m, [1.0 0.4 0.4]);

    % ---- 真实航迹绘制 (亮黄色实线连线) ----
    % true_track 的列顺序为 [lon, lat]，geoplot 参数为 (lat, lon)
    % 因此需要交换列序：tt(:,2)=lat 作为 y 轴，tt(:,1)=lon 作为 x 轴
    geoplot(ax, true_track(:,2), true_track(:,1), 'y-', 'LineWidth', 2, ...
        'DisplayName', '目标真实航迹');

    % 起点标记：绿色圆点(go)，填充绿色
    % true_track(1,:) 是第一帧的 [lon, lat]
    geoplot(ax, true_track(1,2), true_track(1,1), 'yo', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'g');
    % 终点标记：黄色叉号(yx)，线宽加大以便识别
    % true_track(end,:) 是最后一帧的 [lon, lat]
    geoplot(ax, true_track(end,2), true_track(end,1), 'yx', 'MarkerSize', 10, ...
        'LineWidth', 2);

    % ---- 标题与副标题：含关键仿真参数 ----
    % title 设置主标题，subtitle 显示检测概率、虚警率等关键参数
    title(ax, '双基地外辐射源雷达仿真场景');
    subtitle(ax, sprintf('Pd=%.0f%%, Pfa=%.3f, dt=%.0fs, 波束15°, %d-%d km', ...
        params.detection_probability*100, params.false_alarm_rate, ...
        params.dt_sec, params.range_min_km, params.range_max_km));
    legend(ax, 'Location', 'best');  % 自动选择最不遮挡数据的图例位置

    % 强制刷新图形显示
    drawnow;

    % 图窗已弹出，不再保存为PNG文件
end

% =========================================================================
% draw_beam_sector - 绘制波束扇形覆盖区域
%
% 【功能】在给定接收站位置，沿指定方位角范围和距离范围，绘制扇形区域轮廓。
%        由于外辐射源雷达使用第三方发射机，波束形状反映了接收天线的方向性
%        和威力范围的几何约束。
%
% 【参数】
%   ax       - geoaxes 坐标轴句柄
%   rx_lat   - 接收站纬度 (deg)
%   rx_lon   - 接收站经度 (deg)
%   center_az - 波束中心方位角 (deg)，0=北，90=东
%   width    - 波束宽度 (deg)，全角不是半角
%   r_min    - 近距离边界 (m)，通常为盲区距离
%   r_max    - 远距离边界 (m)，通常为最大探测距离
%   color    - [R G B] 颜色向量，范围为 [0,1]
%
% 【绘制内容】
%   1. 内弧：距离 r_min 处的圆弧 (虚线)
%   2. 外弧：距离 r_max 处的圆弧 (虚线)
%   3. 两条径向边：连接内弧和外弧两端的直线
% =========================================================================
function draw_beam_sector(ax, rx_lat, rx_lon, center_az, width, r_min, r_max, color)
    % 在波束宽度范围内均匀采样 20 个方位角点
    % linspace 从 center_az-width/2 到 center_az+width/2 线性插值
    % 20 个点保证扇形边界光滑，同时计算量可控
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
        % 内弧点：距离接收站 r_min 处的经纬度
        [lons_inner(i), lats_inner(i)] = sphere_utils_destination_point(...
            rx_lon, rx_lat, r_min, az_edges(i));
        % 外弧点：距离接收站 r_max 处的经纬度
        [lons_outer(i), lats_outer(i)] = sphere_utils_destination_point(...
            rx_lon, rx_lat, r_max, az_edges(i));
    end

    % 绘制内弧 (虚线)
    % '--' 表示虚线样式，Color 指定扇形颜色，LineWidth=1 细线
    geoplot(ax, lats_inner, lons_inner, '--', 'Color', color, 'LineWidth', 1);
    % 绘制外弧 (虚线)
    geoplot(ax, lats_outer, lons_outer, '--', 'Color', color, 'LineWidth', 1);

    % 绘制两条径向边界线：内弧 → 外弧在波束起始和结束方位角处的连接线
    % 这两条线封闭了扇形的左右两侧，形成完整的扇形轮廓
    for az_edge = [center_az - width/2, center_az + width/2]
        % 计算起始方位角处的内外弧端点
        [lon1, lat1] = sphere_utils_destination_point(rx_lon, rx_lat, r_min, az_edge);
        [lon2, lat2] = sphere_utils_destination_point(rx_lon, rx_lat, r_max, az_edge);
        % 用实线连接内外弧端点
        % [lat1 lat2] 表示 y 轴坐标序列，[lon1 lon2] 表示 x 轴坐标序列
        geoplot(ax, [lat1 lat2], [lon1 lon2], '-', 'Color', color, 'LineWidth', 1);
    end
end
