% 验证 load_adsb_scenario 能否正确构造场景结构
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));

params = simulation_params_oracle();

for sid = {'adsb_0002', 'adsb_0608'}
    id = sid{1};
    fprintf('\n========== %s ==========\n', id);
    sc = load_adsb_scenario(id, params);
    fprintf('目标数=%d  帧数=%d  t1=[%g,%g]  t2=[%g,%g]\n', sc.n_targets, sc.n_frames, ...
        sc.t1_grid(1), sc.t1_grid(end), sc.t2_grid(1), sc.t2_grid(end));

    % 每帧活跃数
    act = zeros(1, sc.n_frames);
    for ac = 1:sc.n_targets
        t = sc.truthTrajs{ac}.time_sec;
        idx = round((t - params.time_offset_radar1_sec)/params.dt_sec) + 1;
        idx = idx(idx >= 1 & idx <= sc.n_frames);
        act(idx) = act(idx) + 1;
    end
    fprintf('每帧活跃: 均值=%.2f 最大=%d 为零的帧数=%d\n', mean(act), max(act), sum(act==0));

    % 检查 truth_all 连续性与速度合理性
    bad = 0; spd = [];
    for ac = 1:sc.n_targets
        tt = sc.truth_all{ac};
        if any(diff(tt(:,5)) <= 0), bad = bad + 1; end
        v = sqrt((tt(:,3)*111.32*cosd(mean(tt(:,2)))).^2 + (tt(:,4)*110.57).^2);
        spd(end+1) = mean(v);
    end
    spd = spd * 1000;   % km/s -> m/s
    fprintf('非单调航迹数=%d  速度: 中位=%.0f m/s  p5=%.0f  p95=%.0f\n', ...
        bad, median(spd), prctile(spd,5), prctile(spd,95));

    % 检查 truth_all 第5列（build_target_states_at_time 要求 [lon lat lon_rate lat_rate t]）
    fprintf('truth_all{1} 尺寸=%dx%d (应为 N x 5)\n', size(sc.truth_all{1},1), size(sc.truth_all{1},2));

    % 空间范围
    alllon = []; alllat = [];
    for ac = 1:sc.n_targets
        alllon = [alllon; sc.truthTrajs{ac}.lon]; %#ok<AGROW>
        alllat = [alllat; sc.truthTrajs{ac}.lat]; %#ok<AGROW>
    end
    fprintf('空间范围: lon %.2f-%.2f  lat %.2f-%.2f\n', min(alllon), max(alllon), min(alllat), max(alllat));

    % 覆盖判定：两站距离
    d1 = hav(params.radar1_lon, params.radar1_lat, alllon, alllat);
    d2 = hav(params.radar2_lon, params.radar2_lat, alllon, alllat);
    fprintf('距R1: %.0f-%.0f km   距R2: %.0f-%.0f km\n', min(d1), max(d1), min(d2), max(d2));
    fprintf('超出[1000,2000]km的点: R1 %d/%d, R2 %d/%d\n', ...
        sum(d1<1000|d1>2000), numel(d1), sum(d2<1000|d2>2000), numel(d2));
end

function d = hav(lon1, lat1, lon2, lat2)
R = 6371.0;
p1 = deg2rad(lat1); p2 = deg2rad(lat2);
dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
d = 2*R*asin(sqrt(a));
end
