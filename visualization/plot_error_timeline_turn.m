% =========================================================================
% plot_error_timeline_turn.m
% =========================================================================
%
% 【功能概述】
%   绘制拐弯目标的滤波误差时间线对比图。并排展示 R1 和 R2 在基础 UKF
%   （使用固定模糊过程噪声 Q）与机动自适应 UKF（检测到机动时自动提升 Q）
%   两种策略下的位置误差随时间变化曲线。用竖线标注拐弯区位置，直观对比
%   两种滤波器在目标机动段的跟踪性能差异。
%
% 【数学原理】
%   1. 基础 UKF（模糊 Q）：
%      使用固定的过程噪声协方差阵 Q，通常设置为覆盖最大预期机动水平
%      的较大值。优点是简单，缺点是非机动段的估计精度被牺牲（Q 过大会
%      导致滤波器对量测噪声敏感）。
%   2. 机动自适应 UKF（机动检测 + Q 提升）：
%      常态下使用较小的 Q（保证高精度），当检测到目标发生机动时
%      （通常基于残差卡方检验或新息序列的统计特性），临时增大 Q
%      以快速响应机动。机动结束后恢复小 Q。
%      数学上：Q_adapted = Q_nominal + alpha * Q_boost * I_maneuver
%      其中 I_maneuver 为机动指示函数 (0 或 1)。
%   3. 误差计算：
%      逐帧取 UKF 估计的经纬度，与真值计算 Haversine 距离 (km)。
%      误差 = haversine_distance(lon_kf, lat_kf, lon_truth, lat_truth) / 1000
%   4. 拐弯区标注：
%      通过竖线 (xline) 标记目标开始转弯的时刻。在拐弯区附近，
%      自适应 UKF 的误差应显著低于基础 UKF。
%
% 【输入参数】
%   true_track    - Nx2 矩阵，真值航迹 [lon, lat]
%   trackR1_base  - R1 基础 UKF 跟踪快照元胞数组
%   trackR2_base  - R2 基础 UKF 跟踪快照元胞数组
%   trackR1_ad    - R1 机动自适应 UKF 跟踪快照元胞数组
%   trackR2_ad    - R2 机动自适应 UKF 跟踪快照元胞数组
%   params        - 仿真参数字段结构体，含 .dt_sec
%   out_dir       - 输出图片目录路径
%
% 【输出】
%   生成文件：
%       fig_turn_error_timeline.png  - 拐弯目标误差时间线对比图
%
% 【调用关系】
%   被调用: 主仿真脚本（拐弯场景）
%   调用:   sphere_utils_haversine_distance() (球面距离计算)
%
% =========================================================================

function plot_error_timeline_turn(true_track, ...
        trackR1_base, trackR2_base, ...
        trackR1_ad, trackR2_ad, params, out_dir)

    n_frames = length(trackR1_base);

    % 预分配每帧误差数组（NaN 表示该帧无有效数据）
    err_r1_base = nan(1, n_frames);
    err_r2_base = nan(1, n_frames);
    err_r1_ad   = nan(1, n_frames);
    err_r2_ad   = nan(1, n_frames);

    % =====================================================================
    % 逐帧计算四种滤波器配置下的位置误差
    % =====================================================================
    for k = 1:n_frames
        if k <= size(true_track, 1)
            t_lon = true_track(k, 1);
            t_lat = true_track(k, 2);

            % ---- R1 基础 UKF ----
            snap = trackR1_base{k};
            if ~isempty(snap.trackList)
                trk = snap.trackList{1};
                % type==1 表示活跃航迹
                if trk.type == 1 && isfield(trk, 'lat') && ~isnan(trk.lat)
                    err_r1_base(k) = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
                end
            end

            % ---- R2 基础 UKF（使用原始 R2 时间网格） ----
            snap2 = trackR2_base{k};
            if ~isempty(snap2.trackList)
                trk2 = snap2.trackList{1};
                if trk2.type == 1 && isfield(trk2, 'lat') && ~isnan(trk2.lat)
                    err_r2_base(k) = sphere_utils_haversine_distance(trk2.lon, trk2.lat, t_lon, t_lat) / 1000;
                end
            end

            % ---- R1 机动自适应 UKF ----
            snap_a = trackR1_ad{k};
            if ~isempty(snap_a.trackList)
                trk_a = snap_a.trackList{1};
                if trk_a.type == 1 && isfield(trk_a, 'lat') && ~isnan(trk_a.lat)
                    err_r1_ad(k) = sphere_utils_haversine_distance(trk_a.lon, trk_a.lat, t_lon, t_lat) / 1000;
                end
            end

            % ---- R2 机动自适应 UKF ----
            snap2_a = trackR2_ad{k};
            if ~isempty(snap2_a.trackList)
                trk2_a = snap2_a.trackList{1};
                if trk2_a.type == 1 && isfield(trk2_a, 'lat') && ~isnan(trk2_a.lat)
                    err_r2_ad(k) = sphere_utils_haversine_distance(trk2_a.lon, trk2_a.lat, t_lon, t_lat) / 1000;
                end
            end
        end
    end

    fig = figure('Position', [50, 50, 1400, 750]);
    tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- R1 对比 (左) ----
    ax1 = nexttile(tlo);
    hold(ax1, 'on');
    t = 0:params.dt_sec:(n_frames-1)*params.dt_sec;
    t_plot = t(1:min(length(t), length(err_r1_base)));

    % 基础 UKF：淡蓝色
    p1 = plot(ax1, t_plot, err_r1_base, '-', 'Color', [0.3 0.5 1.0], 'LineWidth', 1.5, ...
        'DisplayName', 'R1 基础UKF');
    % 自适应 UKF：深蓝色实线（更粗），预期在拐弯区域误差更小
    p2 = plot(ax1, t_plot, err_r1_ad, 'b-', 'LineWidth', 1.8, ...
        'DisplayName', 'R1 自适应UKF');

    % 标注拐弯时段：竖线标记拐弯开始区域
    mid_t = t_plot(round(end/2));
    xline(ax1, mid_t, 'k--', '拐弯区', 'LineWidth', 1, 'Alpha', 0.5);

    xlabel(ax1, '时间 (s)');
    ylabel(ax1, '位置误差 (km)');
    title(ax1, 'R1 滤波误差对比');
    legend(ax1, 'Location', 'best');
    grid(ax1, 'on');

    % ---- R2 对比 (右) ----
    ax2 = nexttile(tlo);
    hold(ax2, 'on');

    % 基础 UKF：淡红色
    plot(ax2, t_plot, err_r2_base, '-', 'Color', [1.0 0.4 0.4], 'LineWidth', 1.5, ...
        'DisplayName', 'R2 基础UKF');
    % 自适应 UKF：深红色实线（更粗）
    plot(ax2, t_plot, err_r2_ad, 'r-', 'LineWidth', 1.8, ...
        'DisplayName', 'R2 自适应UKF');

    xline(ax2, mid_t, 'k--', '拐弯区', 'LineWidth', 1, 'Alpha', 0.5);

    xlabel(ax2, '时间 (s)');
    ylabel(ax2, '位置误差 (km)');
    title(ax2, 'R2 滤波误差对比');
    legend(ax2, 'Location', 'best');
    grid(ax2, 'on');

    % 总标题：说明两种滤波策略的差异机制
    sgtitle('拐弯目标: 基础UKF (模糊Q) vs 机动自适应UKF (机动检测+Q提升)');

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig_turn_error_timeline.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig_turn_error_timeline.png'));
    end
    fprintf('  误差时间线对比图已保存: fig_turn_error_timeline.png\n');
end
