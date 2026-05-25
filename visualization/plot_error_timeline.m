% =========================================================================
% plot_error_timeline.m
% =========================================================================
%
% 【功能概述】
%   绘制双基地雷达跟踪的误差时间线图。分上下两个子图：
%   (1) 位置误差时序 — 展示 R1 和 R2 UKF 滤波后位置误差以及
%       点迹级别误差随时间（帧号）的变化，单位为 km；
%   (2) 检测/关联事件标记 — 用不同符号标记每帧是否发生目标检测
%       关联、虚警误关联或漏检事件。
%
% 【数学原理】
%   1. 位置误差计算：逐帧比较 UKF 滤波估计的位置（或校准后点迹位置）
%      与真实航迹之间的 Haversine 大圆距离。
%      误差 = sphere_utils_haversine_distance(lon_est, lat_est, lon_truth, lat_truth)
%      以米为单位计算，除以 1000 转换为 km。
%   2. 真值插值：由于仿真时间网格 (t1_grid, t2_grid) 和真实航迹的采样
%      时刻可能不同步，使用 interp1 线性插值获取对应时刻的真值经纬度。
%   3. 事件分类：
%      - 实心圆点 (.) = 成功关联目标检测
%      - 红色叉号 (x) = 误关联杂波（虚警被关联）
%      - 灰色小点 (.) = 漏检（跟踪状态存在但无关联）
%
% 【输入参数】
%   trackState_R1  - R1 跟踪状态元胞数组
%   trackState_R2  - R2 跟踪状态元胞数组
%   detList_R1     - R1 检测结果元胞数组
%   detList_R2     - R2 检测结果元胞数组
%   true_track     - Nx5 矩阵，列：真实经度、纬度、高度、速度、时间
%   t1_grid        - R1 仿真时间网格 (s)
%   t2_grid        - R2 仿真时间网格 (s)
%   params         - 仿真参数字段结构体
%   out_dir        - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig4_error_timeline.png  - 误差与事件时间线图
%
% 【调用关系】
%   被调用: 主仿真脚本
%   调用:   sphere_utils_haversine_distance() (球面距离计算)
%
% =========================================================================

function plot_error_timeline(trackState_R1, trackState_R2, detList_R1, detList_R2, ...
        true_track, t1_grid, t2_grid, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);

    n_frames = length(trackState_R1);
    % 预分配 NaN 数组：NaN 表示该帧无有效数据
    err_R1 = nan(n_frames, 1);     % R1 UKF滤波位置误差 (m)
    err_R2 = nan(n_frames, 1);     % R2 UKF滤波位置误差 (m)
    err_det_R1 = nan(n_frames, 1); % R1 点迹级别误差 (m)
    err_det_R2 = nan(n_frames, 1); % R2 点迹级别误差 (m)

    % =====================================================================
    % 计算 R1 每帧误差
    % =====================================================================
    for k = 1:n_frames
        t = t1_grid(k);
        % 通过线性插值获取该时刻的真值经纬度
        true_lon = interp1(true_track(:,5), true_track(:,1), t, 'linear', 'extrap');
        true_lat = interp1(true_track(:,5), true_track(:,2), t, 'linear', 'extrap');

        % UKF滤波位置误差
        s = trackState_R1{k};
        if ~isempty(s) && isfield(s, 'lat') && ~isnan(s.lat)
            err_R1(k) = sphere_utils_haversine_distance(s.lon, s.lat, true_lon, true_lat);
        end

        % 取第一帧真实目标检测点迹的误差
        dets = detList_R1{k};
        if ~isempty(dets)
            for d = 1:length(dets)
                if ~dets(d).is_clutter && isfield(dets(d), 'lat') && ~isnan(dets(d).lat)
                    err_det_R1(k) = sphere_utils_haversine_distance(...
                        dets(d).lon, dets(d).lat, true_lon, true_lat);
                    break;  % 取第一个非杂波的检测点
                end
            end
        end
    end

    % =====================================================================
    % 计算 R2 每帧误差（逻辑与 R1 相同）
    % =====================================================================
    for k = 1:length(t2_grid)
        t = t2_grid(k);
        if k > n_frames, break; end
        true_lon = interp1(true_track(:,5), true_track(:,1), t, 'linear', 'extrap');
        true_lat = interp1(true_track(:,5), true_track(:,2), t, 'linear', 'extrap');

        s = trackState_R2{k};
        if ~isempty(s) && isfield(s, 'lat') && ~isnan(s.lat)
            err_R2(k) = sphere_utils_haversine_distance(s.lon, s.lat, true_lon, true_lat);
        end

        dets = detList_R2{k};
        if ~isempty(dets)
            for d = 1:length(dets)
                if ~dets(d).is_clutter && isfield(dets(d), 'lat') && ~isnan(dets(d).lat)
                    err_det_R2(k) = sphere_utils_haversine_distance(...
                        dets(d).lon, dets(d).lat, true_lon, true_lat);
                    break;
                end
            end
        end
    end

    % =====================================================================
    % 上子图：位置 RMSE 随时间变化（min 单位）
    % =====================================================================
    subplot(2, 1, 1);
    % 横轴为时间(分钟)，纵轴为误差(km)
    plot(t1_grid(1:n_frames)/60, err_R1/1000, 'b-', 'LineWidth', 1, 'DisplayName', 'R1 UKF滤波');
    hold on;
    plot(t2_grid(1:n_frames)/60, err_R2(1:n_frames)/1000, 'r-', 'LineWidth', 1, 'DisplayName', 'R2 UKF滤波');
    plot(t1_grid(1:n_frames)/60, err_det_R1/1000, 'b.', 'MarkerSize', 3, 'DisplayName', 'R1 点迹');
    plot(t2_grid(1:n_frames)/60, err_det_R2(1:n_frames)/1000, 'r.', 'MarkerSize', 3, 'DisplayName', 'R2 点迹');
    ylabel('位置误差 (km)');
    xlabel('时间 (min)');
    title('位置误差时序');
    legend('Location', 'best');
    grid on;

    % =====================================================================
    % 下子图：事件标记（二值/三值离散标记）
    % =====================================================================
    subplot(2, 1, 2);
    hold on;
    ylim([0, 5]);

    % R1 事件标记：在 y=4 行上用不同符号标记
    for k = 1:n_frames
        s = trackState_R1{k};
        if isempty(s), continue; end
        if strcmp(s.status, 'TRACKING') && s.associated && ~s.assc_is_clutter
            % 成功关联目标：蓝色实心圆
            plot(k, 4, 'b.', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && s.associated && s.assc_is_clutter
            % 误关联杂波：红色叉号
            plot(k, 4, 'rx', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && ~s.associated
            % 漏检（跟踪态但未关联到任何点迹）：灰色小点
            plot(k, 4, 'b.', 'MarkerSize', 4, 'Color', [0.5 0.5 0.5]);
        end
    end

    % R2 事件标记：在 y=3 行上用不同符号标记
    for k = 1:n_frames
        s = trackState_R2{k};
        if isempty(s), continue; end
        if strcmp(s.status, 'TRACKING') && s.associated && ~s.assc_is_clutter
            plot(k, 3, 'r.', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && s.associated && s.assc_is_clutter
            plot(k, 3, 'rx', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && ~s.associated
            plot(k, 3, 'r.', 'MarkerSize', 4, 'Color', [0.5 0.5 0.5]);
        end
    end

    yticks([3 4]);
    yticklabels({'R2', 'R1'});
    xlabel('帧号');
    title('检测/关联事件 ●=关联目标 ×=关联虚警 ·=漏检');
    grid on;

    sgtitle(sprintf('误差与事件时间线 (nFrames=%d)', n_frames));
    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig4_error_timeline.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig4_error_timeline.png'));
    end
    fprintf('  图4 已保存: fig4_error_timeline.png\n');
end
