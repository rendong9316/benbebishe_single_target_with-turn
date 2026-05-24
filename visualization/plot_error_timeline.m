% =========================================================================
% plot_error_timeline.m
% 图4: 位置误差时间线 + 检测/关联事件标记
% =========================================================================

function plot_error_timeline(trackState_R1, trackState_R2, detList_R1, detList_R2, ...
        true_track, t1_grid, t2_grid, params, out_dir)
    fig = figure('Position', [50, 50, 1400, 750]);

    n_frames = length(trackState_R1);
    err_R1 = nan(n_frames, 1);
    err_R2 = nan(n_frames, 1);
    err_det_R1 = nan(n_frames, 1);
    err_det_R2 = nan(n_frames, 1);

    % R1误差
    for k = 1:n_frames
        t = t1_grid(k);
        true_lon = interp1(true_track(:,5), true_track(:,1), t, 'linear', 'extrap');
        true_lat = interp1(true_track(:,5), true_track(:,2), t, 'linear', 'extrap');

        s = trackState_R1{k};
        if ~isempty(s) && isfield(s, 'lat') && ~isnan(s.lat)
            err_R1(k) = sphere_utils_haversine_distance(s.lon, s.lat, true_lon, true_lat);
        end

        dets = detList_R1{k};
        if ~isempty(dets)
            for d = 1:length(dets)
                if ~dets(d).is_clutter && isfield(dets(d), 'lat') && ~isnan(dets(d).lat)
                    err_det_R1(k) = sphere_utils_haversine_distance(...
                        dets(d).lon, dets(d).lat, true_lon, true_lat);
                    break;
                end
            end
        end
    end

    % R2误差
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

    % ---- 上子图: 位置RMSE ----
    subplot(2, 1, 1);
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

    % ---- 下子图: 事件标记 ----
    subplot(2, 1, 2);
    hold on;
    ylim([0, 5]);

    % R1 事件
    for k = 1:n_frames
        s = trackState_R1{k};
        if isempty(s), continue; end
        if strcmp(s.status, 'TRACKING') && s.associated && ~s.assc_is_clutter
            plot(k, 4, 'b.', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && s.associated && s.assc_is_clutter
            plot(k, 4, 'rx', 'MarkerSize', 8);
        elseif strcmp(s.status, 'TRACKING') && ~s.associated
            plot(k, 4, 'b.', 'MarkerSize', 4, 'Color', [0.5 0.5 0.5]);
        end
    end

    % R2 事件
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
