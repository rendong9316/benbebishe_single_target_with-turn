% probe_reg_band.m — 按目标距离分带统计位置误差
% 核心假设：两参数均值标定的误差随距离呈系统性趋势（近距/远距符号相反），
%           三参数 MLE 应近似平坦。若成立，则三参数的价值不是"平均更准"，
%           而是"误差不随距离漂移"，这对宽域探测是本质性的。
% 只读：不改动源码
r0 = fileparts(pwd);
addpath(r0);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(r0, d{1}));
end

out = 'reg_band.txt';
fid = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

scenes = {'adsb_0002','adsb_span'};
modes  = {'mean','mle3'};
edges  = 1000:100:1600;                 % R2 距离分带边界

pr('=== 按目标到 R2 的距离分带统计位置 RMSE ===\n');
pr('（每带汇总该站所有航迹所有帧；偏差=沿 R2 视线的径向投影均值，正=偏远）\n\n');

for si = 1:numel(scenes)
    sc = scenes{si};
    for mi = 1:numel(modes)
        md = modes{mi};
        p = simulation_params_oracle();
        p.registration_mode = md;
        p.random_seed = 1;
        try
            evalc('r = run(sc, p);');
        catch ME
            pr('[%s %s] FAILED: %s\n', sc, md, ME.message);
            continue;
        end
        truth = r.truthTrajs;
        for st = 1:2
            if st == 1
                snaps = r.trackSnapshots_R1;  tg = r.scenario.t1_grid;  nm = 'R1';
            else
                snaps = r.trackSnapshots_R2;  tg = r.scenario.t2_grid;  nm = 'R2';
            end
            dist = []; err = []; sgn = [];
            for k = 1:numel(snaps)
                tnow = tg(k);
                tl = snaps{k}.trackList;
                for j = 1:numel(tl)
                    tr = tl{j};
                    if tr.type == 7, continue; end
                    if ~isfield(tr,'truth_idx') || isempty(tr.truth_idx), continue; end
                    ti = tr.truth_idx;
                    if ~isfinite(ti) || ti <= 0 || ti > numel(truth), continue; end
                    T = truth{ti};
                    if tnow < T.time_sec(1) || tnow > T.time_sec(end), continue; end
                    tlon = interp1(T.time_sec, T.lon, tnow, 'linear');
                    tlat = interp1(T.time_sec, T.lat, tnow, 'linear');
                    if isnan(tlon) || isnan(tlat), continue; end
                    dy = (tr.lat - tlat) * 110.57;
                    dx = (tr.lon - tlon) * 111.32 * cosd(tlat);
                    e  = hypot(dx, dy);
                    dd = sphere_utils_haversine_distance(p.radar2_lon, p.radar2_lat, ...
                        tlon, tlat) / 1000;
                    dist(end+1,1) = dd;   %#ok<AGROW>
                    err(end+1,1)  = e;    %#ok<AGROW>
                    azr = sphere_utils_azimuth(p.radar2_lon, p.radar2_lat, tlon, tlat);
                    sgn(end+1,1)  = dx*sind(azr) + dy*cosd(azr);  %#ok<AGROW>  % 径向投影
                end
            end
            if isempty(err), continue; end
            pr('[%s / %s / %s]  n=%d, 距离 %.0f-%.0f km\n', sc, md, nm, ...
                numel(err), min(dist), max(dist));
            pr('  %-12s %7s %10s %10s\n', '距离带','帧数','RMSE','偏差');
            for b = 1:numel(edges)-1
                sel = dist >= edges(b) & dist < edges(b+1);
                if sum(sel) < 20, continue; end
                pr('  %-12s %7d %10.3f %10.3f\n', ...
                    sprintf('%d-%d', edges(b), edges(b+1)), sum(sel), ...
                    sqrt(mean(err(sel).^2)), mean(sgn(sel)));
            end
            pr('  %-12s %7d %10.3f %10.3f\n\n', '全部', numel(err), ...
                sqrt(mean(err.^2)), mean(sgn));
        end
    end
end
fclose(fid);
fprintf('WROTE %s\n', out);
