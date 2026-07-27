function visualize_rate_scenarios()
    % 可视化所有 rate scenario 的真值航迹
    % 验证整改后是否为真正的 180° 大转弯

    close all;
    addpath(genpath('.'));

    scenarios = { ...
        'right_rate_0p7', '右转 180° 半圆 (0.7°/s, ≈257s)'; ...
        'left_rate_0p7', '左转 180° 半圆 (0.7°/s, ≈257s)'; ...
        'right_rate_1p3', '右转 180° 半圆 (1.3°/s, ≈138s)'; ...
        'left_rate_1p3', '左转 180° 半圆 (1.3°/s, ≈138s)'; ...
        'single_uturn', '左转 uturn 180° (默认 1°/s, 180s)'; ...
        'single_straight', '直线 (无转弯)' ...
    };

    fprintf('=== 航迹几何验证 ===\n');
    fprintf('%-22s | %-8s | %-10s | %-12s | %-10s | %-12s\n', ...
        '场景', '点数', '总时长(s)', '航向变化(°)', '转弯持续', '判定');
    fprintf('%s\n', repmat('-', 1, 100));

    fig = figure('Position', [50, 50, 1600, 1200], 'Color', 'w', 'Visible', 'off');

    for si = 1:size(scenarios, 1)
        sname = scenarios{si, 1};
        sdesc = scenarios{si, 2};

        params = simulation_params_oracle();
        if contains(sname, 'rate_0p7')
            params.truth_turn_rate_deg_per_sec = 0.7;
        elseif contains(sname, 'rate_1p3')
            params.truth_turn_rate_deg_per_sec = 1.3;
        end

        sc = build_truth_scenario(sname, params);
        truth = sc.truthTrajs{1};

        % 计算逐点航向（度），用 lon_rate/lat_rate 推算
        % heading = atan2(lon_rate*cos(lat), lat_rate) in degrees
        lon_rate = truth.lon_rate;
        lat_rate = truth.lat_rate;
        headings = atan2d(lon_rate .* cosd(truth.lat), lat_rate);
        headings = mod(headings, 360);

        % 航向变化总量（累计绝对变化，处理 360° 跨越）
        dhead = diff(headings);
        dhead(dhead > 180) = dhead(dhead > 180) - 360;
        dhead(dhead < -180) = dhead(dhead < -180) + 360;
        total_turn = sum(dhead);
        total_abs_turn = sum(abs(dhead));

        % 转弯段时长：航向变化率超过阈值的帧数 × dt
        turn_rate_per_frame = abs(dhead);
        turning_mask = turn_rate_per_frame > 0.1;  % 每帧变化 >0.1°/dt
        n_turning_frames = sum(turning_mask);
        dt = params.dt_sec;
        turn_duration = n_turning_frames * dt;

        % 判定
        if contains(sname, 'straight')
            verdict = 'OK (直线)';
            expected = '0°';
        else
            expected = '~180°';
            if total_abs_turn > 150
                verdict = '✓ 大转弯';
            elseif total_abs_turn > 60
                verdict = '⚠ 中等转弯';
            else
                verdict = '✗ 小转弯';
            end
        end

        fprintf('%-22s | %-8d | %-10.0f | %-12.1f | %-10.0f | %-12s\n', ...
            sname, length(truth.lon), truth.time_sec(end)-truth.time_sec(1), ...
            total_abs_turn, turn_duration, verdict);

        % 画图
        subplot(3, 2, si);
        plot(truth.lon, truth.lat, 'b-', 'LineWidth', 2);
        hold on; grid on;
        plot(params.radar1_lon, params.radar1_lat, 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        text(params.radar1_lon+0.3, params.radar1_lat, 'R1', 'Color', 'r');
        plot(params.radar2_lon, params.radar2_lat, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
        text(params.radar2_lon+0.3, params.radar2_lat, 'R2', 'Color', 'k');
        plot(truth.lon(1), truth.lat(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        text(truth.lon(1)+0.2, truth.lat(1), 'Start', 'Color', 'g');
        plot(truth.lon(end), truth.lat(end), 'mx', 'MarkerSize', 12, 'LineWidth', 2);
        text(truth.lon(end)+0.2, truth.lat(end), 'End', 'Color', 'm');

        title(sprintf('%s\n[%s] 航向变化=%.0f° 转弯=%.0fs', ...
            sdesc, sname, total_abs_turn, turn_duration), 'FontSize', 10);
        xlabel('经度 (°)'); ylabel('纬度 (°)');
        axis equal;
    end

    sgtitle('整改后：rate 场景真值航迹（应为 180° 半圆）', 'FontSize', 14, 'FontWeight', 'bold');

    % 保存
    if ~exist('results', 'dir'), mkdir('results'); end
    saveas(fig, 'results/rate_scenarios_overview.png');
    fprintf('\n图片已保存: results/rate_scenarios_overview.png\n');
end
