% =========================================================================
% plot_error_timeline_turn.m
% 拐弯目标误差时间线对比: 基础UKF vs 机动自适应UKF
% =========================================================================

function plot_error_timeline_turn(true_track, ...
        trackR1_base, trackR2_base, ...
        trackR1_ad, trackR2_ad, params, out_dir)

    n_frames = length(trackR1_base);

    % 计算每帧误差
    err_r1_base = nan(1, n_frames);
    err_r2_base = nan(1, n_frames);
    err_r1_ad   = nan(1, n_frames);
    err_r2_ad   = nan(1, n_frames);

    for k = 1:n_frames
        if k <= size(true_track, 1)
            t_lon = true_track(k, 1);
            t_lat = true_track(k, 2);

            % R1基础
            snap = trackR1_base{k};
            if ~isempty(snap.trackList)
                trk = snap.trackList{1};
                if trk.type == 1 && isfield(trk, 'lat') && ~isnan(trk.lat)
                    err_r1_base(k) = sphere_utils_haversine_distance(trk.lon, trk.lat, t_lon, t_lat) / 1000;
                end
            end

            % R2基础 (未对齐, 用原始R2时间网格)
            snap2 = trackR2_base{k};
            if ~isempty(snap2.trackList)
                trk2 = snap2.trackList{1};
                if trk2.type == 1 && isfield(trk2, 'lat') && ~isnan(trk2.lat)
                    err_r2_base(k) = sphere_utils_haversine_distance(trk2.lon, trk2.lat, t_lon, t_lat) / 1000;
                end
            end

            % R1自适应
            snap_a = trackR1_ad{k};
            if ~isempty(snap_a.trackList)
                trk_a = snap_a.trackList{1};
                if trk_a.type == 1 && isfield(trk_a, 'lat') && ~isnan(trk_a.lat)
                    err_r1_ad(k) = sphere_utils_haversine_distance(trk_a.lon, trk_a.lat, t_lon, t_lat) / 1000;
                end
            end

            % R2自适应
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

    % ---- R1 对比 ----
    ax1 = nexttile(tlo);
    hold(ax1, 'on');
    t = 0:params.dt_sec:(n_frames-1)*params.dt_sec;
    t_plot = t(1:min(length(t), length(err_r1_base)));

    p1 = plot(ax1, t_plot, err_r1_base, '-', 'Color', [0.3 0.5 1.0], 'LineWidth', 1.5, ...
        'DisplayName', 'R1 基础UKF');
    p2 = plot(ax1, t_plot, err_r1_ad, 'b-', 'LineWidth', 1.8, ...
        'DisplayName', 'R1 自适应UKF');

    % 标注转弯时段
    mid_t = t_plot(round(end/2));
    xline(ax1, mid_t, 'k--', '拐弯区', 'LineWidth', 1, 'Alpha', 0.5);

    xlabel(ax1, '时间 (s)');
    ylabel(ax1, '位置误差 (km)');
    title(ax1, 'R1 滤波误差对比');
    legend(ax1, 'Location', 'best');
    grid(ax1, 'on');

    % ---- R2 对比 ----
    ax2 = nexttile(tlo);
    hold(ax2, 'on');

    plot(ax2, t_plot, err_r2_base, '-', 'Color', [1.0 0.4 0.4], 'LineWidth', 1.5, ...
        'DisplayName', 'R2 基础UKF');
    plot(ax2, t_plot, err_r2_ad, 'r-', 'LineWidth', 1.8, ...
        'DisplayName', 'R2 自适应UKF');

    xline(ax2, mid_t, 'k--', '拐弯区', 'LineWidth', 1, 'Alpha', 0.5);

    xlabel(ax2, '时间 (s)');
    ylabel(ax2, '位置误差 (km)');
    title(ax2, 'R2 滤波误差对比');
    legend(ax2, 'Location', 'best');
    grid(ax2, 'on');

    sgtitle('拐弯目标: 基础UKF (模糊Q) vs 机动自适应UKF (机动检测+Q提升)');

    drawnow;
    try
        exportgraphics(fig, fullfile(out_dir, 'fig_turn_error_timeline.png'), 'Resolution', 200);
    catch
        saveas(fig, fullfile(out_dir, 'fig_turn_error_timeline.png'));
    end
    fprintf('  误差时间线对比图已保存: fig_turn_error_timeline.png\n');
end
