% probe_span_scene.m — 验证距离分层场景能否被 MATLAB 正确读取
% 只读：不改动任何源码
r0 = fileparts(pwd);            % _audit 的上级即仓库根
addpath(r0);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(r0, d{1}));
end

out = 'span_scene.txt';
fid = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

p = simulation_params_oracle();
pr('=== 距离分层场景验证 ===\n');
pr('h_true=%.0f km  h_assumed=%.0f km\n\n', ...
    iono_h(p,'true')/1000, iono_h(p,'assumed')/1000);

for sid = {'adsb_0002','adsb_span','adsb_span_d'}
    s = char(sid{1});
    try
        sc = load_adsb_scenario(s, p);
    catch ME
        pr('[%s] 读取失败: %s\n', s, ME.message);
        continue;
    end
    nT = numel(sc.truth_all);
    nF = sc.n_frames;
    d2 = []; d1 = []; nf = zeros(nT,1);
    % truth_all{k} 为 N×5 矩阵：[lon, lat, lon_rate, lat_rate, t]
    for k = 1:nT
        tr = sc.truth_all{k};
        nf(k) = size(tr, 1);
        lon = tr(:,1); lat = tr(:,2);
        d2 = [d2; arrayfun(@(a,b) sphere_utils_haversine_distance( ...
            p.radar2_lon, p.radar2_lat, a, b)/1000, lon, lat)];
        d1 = [d1; arrayfun(@(a,b) sphere_utils_haversine_distance( ...
            p.radar1_lon, p.radar1_lat, a, b)/1000, lon, lat)];
    end
    pr('[%s] 目标 %d, 帧 %d, 总点数 %d\n', s, nT, nF, sum(nf));
    pr('  帧数: 中位 %d, min %d, max %d\n', median(nf), min(nf), max(nf));
    pr('  R2 距离: %.0f - %.0f km (p5 %.0f, p95 %.0f), 跨度 %.0f\n', ...
        min(d2), max(d2), prctile(d2,5), prctile(d2,95), prctile(d2,95)-prctile(d2,5));
    pr('  R1 距离: %.0f - %.0f km\n', min(d1), max(d1));
    % 分带帧数
    edges = 1000:100:1600;
    cnt = histcounts(d2, edges);
    pr('  分带帧数:');
    for i = 1:numel(cnt)
        pr(' %d-%d:%d', edges(i), edges(i+1), cnt(i));
    end
    pr('\n\n');
end
fclose(fid);
fprintf('WROTE %s\n', out);
