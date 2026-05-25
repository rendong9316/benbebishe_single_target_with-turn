% =========================================================================
% plot_point_cloud_3d.m
% =========================================================================
%
% 【功能概述】
%   绘制点迹检测结果的三维分布图。使用 MATLAB 三维坐标系 (plot3) 在
%   Range-Azimuth-Frame (群距离-方位角-帧号) 空间中展示检测结果，
%   其中真实目标检出点用蓝色圆点标记，虚警杂波点用红色叉号标记。
%   通过三维旋转视角直观展示点迹在时间维度上的积累情况以及杂波
%   在量测空间中的分布规律。
%
% 【数学原理】
%   1. 群距离 (Pseudo-range / Group Range)：
%      外辐射源雷达中，由于发射机-目标-接收机的双程路径延迟，
%      量测得到的"距离"不是真实斜距，而是群距离（发射机到目标再
%      到接收机的总路径长度）。通常以 km 为单位。
%   2. 方位角 (Azimuth)：以接收站为原点的到达角 (DOA)，单位为度。
%   3. 三维点迹空间：将每一帧的量测点按 (Rg, Az, Frame) 三个维度
%      可视化，可以观察到：
%      - 目标点迹沿时间维的连续性（真实目标形成可辨识的轨迹）
%      - 杂波点在空间中的随机散布（虚警的随机性）
%
% 【输入参数】
%   detList   - 元胞数组，detList{k} 为第 k 帧的检测结构体数组，
%               每个检测结构体至少包含以下字段：
%       .prange     - 群距离 (m)，将转换为 km 以便显示
%       .paz        - 方位角 (deg)
%       .is_clutter - 逻辑值，true=虚警杂波, false=真实目标检出
%   title_str - 字符串，图的标题前缀
%   out_path  - 字符串，输出图片的完整路径（含文件名）
%
% 【输出】
%   屏幕打印保存信息，并在 out_path 位置生成 PNG 图片（分辨率 200 DPI）
%
% 【调用关系】
%   被调用: plot_single_track_result.m 或其他需要展示点迹分布的函数
%   调用:   无外部依赖（MATLAB 内置函数 plot3, xlabel, ylabel, zlabel 等）
%
% =========================================================================

function plot_point_cloud_3d(detList, title_str, out_path)
    % 创建图窗
    fig = figure('Position', [50, 50, 1400, 750]);
    hold on;

    % 初始化目标点迹和杂波点迹的存储数组
    range_tgt = []; az_tgt = []; frame_tgt = [];
    range_clt = []; az_clt = []; frame_clt = [];

    % 遍历所有帧，将检测结果按目标/杂波分类存储
    for k = 1:length(detList)
        dets = detList{k};
        if isempty(dets), continue; end
        for d = 1:length(dets)
            % 根据 is_clutter 标记分类：杂波 vs 目标
            if dets(d).is_clutter
                range_clt(end+1) = dets(d).prange / 1000;  % m → km
                az_clt(end+1) = dets(d).paz;
                frame_clt(end+1) = k;
            else
                range_tgt(end+1) = dets(d).prange / 1000;  % m → km
                az_tgt(end+1) = dets(d).paz;
                frame_tgt(end+1) = k;
            end
        end
    end

    % ---- 绘制真实目标点迹：蓝色圆点(bo)，MarkerFaceColor 填充蓝色 ----
    % 使用 plot3 在三维(Rg, Az, Frame)空间中标记目标检出
    if ~isempty(range_tgt)
        plot3(range_tgt, az_tgt, frame_tgt, 'bo', ...
            'MarkerSize', 4, 'MarkerFaceColor', 'b', 'DisplayName', '目标检出');
    end

    % ---- 绘制虚警杂波：红色叉号(rx)，用于区分真实目标 ----
    % 杂波点通常呈随机分布，在三维视角中可观察其空间分布特性
    if ~isempty(range_clt)
        plot3(range_clt, az_clt, frame_clt, 'rx', ...
            'MarkerSize', 3, 'DisplayName', '虚警杂波');
    end

    % 坐标轴标签
    xlabel('群距离 Rg (km)');     % 群距离，外辐射源雷达的关键量测量
    ylabel('方位角 az (deg)');     % 到达方位角
    zlabel('帧号 k');              % 时间维度（每帧对应一个采样时刻）
    title(sprintf('%s — 点迹分布 (R-A-Frame)', title_str));
    legend('Location', 'best');
    grid on;

    % 设置三维视角：方位角 45°，仰角 30°
    % 这个角度可以同时观察 R-A 平面的分布和随帧号的变化
    view(45, 30);

    % 启用三维旋转交互，用户可手动拖动视角
    rotate3d on;
    drawnow;

    % 导出图片
    try
        exportgraphics(fig, out_path, 'Resolution', 200);
    catch
        saveas(fig, out_path);
    end
    fprintf('  点迹图已保存: %s\n', out_path);
end
