% =========================================================================
% plot_point_cloud_3d.m
% 图2: 三维点迹图 Range-Azimuth-Frame
% =========================================================================

function plot_point_cloud_3d(detList, title_str, out_path)
    fig = figure('Position', [50, 50, 1400, 750]);
    hold on;

    range_tgt = []; az_tgt = []; frame_tgt = [];
    range_clt = []; az_clt = []; frame_clt = [];

    for k = 1:length(detList)
        dets = detList{k};
        if isempty(dets), continue; end
        for d = 1:length(dets)
            if dets(d).is_clutter
                range_clt(end+1) = dets(d).prange / 1000;
                az_clt(end+1) = dets(d).paz;
                frame_clt(end+1) = k;
            else
                range_tgt(end+1) = dets(d).prange / 1000;
                az_tgt(end+1) = dets(d).paz;
                frame_tgt(end+1) = k;
            end
        end
    end

    % 真实目标点迹: 蓝色○
    if ~isempty(range_tgt)
        plot3(range_tgt, az_tgt, frame_tgt, 'bo', ...
            'MarkerSize', 4, 'MarkerFaceColor', 'b', 'DisplayName', '目标检出');
    end

    % 虚警杂波: 红色×
    if ~isempty(range_clt)
        plot3(range_clt, az_clt, frame_clt, 'rx', ...
            'MarkerSize', 3, 'DisplayName', '虚警杂波');
    end

    xlabel('群距离 Rg (km)');
    ylabel('方位角 az (deg)');
    zlabel('帧号 k');
    title(sprintf('%s — 点迹分布 (R-A-Frame)', title_str));
    legend('Location', 'best');
    grid on;
    view(45, 30);

    rotate3d on;
    drawnow;
    try
        exportgraphics(fig, out_path, 'Resolution', 200);
    catch
        saveas(fig, out_path);
    end
    fprintf('  点迹图已保存: %s\n', out_path);
end
