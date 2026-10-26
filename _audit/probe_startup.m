% 关键验证：起始段 vs 稳态段的误差分解
% 假设：两点起始的速度误差 ~ sigma_pos*1.414/dt = 471 m/s，远大于真实速度 230 m/s
% 若成立，则"优化起始"比"优化 IMM"更能降低 RMSE
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
S = load(fullfile(root,'_audit','mc_current_geom.mat'));

snaps = S.trackSnapshots_R1; nf = numel(snaps);
truths = S.truthTrajs;

% 对每个目标，按 life（航迹年龄）分段统计位置误差
fprintf('\n### R1 按航迹年龄分段的位置误差 ###\n');
fprintf('目标  年龄段       n     mean(km)  rms(km)   max(km)\n');
allSeg = struct('lo',{},'hi',{},'e',{});

for ac = 1:numel(truths)
    tt = truths{ac};
    rec = [];  % [life, err]
    for k = 1:nf
        tnow = (k-1)*S.params.dt_sec;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx == ac && s.type ~= 7 && ~isnan(s.lat)
                d = haversine(s.lon, s.lat, tlon, tlat);
                life = 1; if isfield(s,'life'), life = s.life; end
                rec(end+1,:) = [life, d];
                break;
            end
        end
    end
    segs = {[1 2],[3 5],[6 10],[11 20],[21 999]};
    lbl  = {'1-2','3-5','6-10','11-20','>20'};
    for q = 1:numel(segs)
        m = rec(:,1) >= segs{q}(1) & rec(:,1) <= segs{q}(2);
        if sum(m) == 0, continue; end
        e = rec(m,2);
        fprintf('%3d   %-8s  %3d   %7.2f  %7.2f  %7.2f\n', ac, lbl{q}, numel(e), mean(e), sqrt(mean(e.^2)), max(e));
    end
    e_all = rec(:,2);
    fprintf('%3d   %-8s  %3d   %7.2f  %7.2f  %7.2f\n\n', ac, 'ALL', numel(e_all), mean(e_all), sqrt(mean(e_all.^2)), max(e_all));
end

% 汇总：起始段(<=5) vs 稳态(>5) 对总 MSE 的贡献
fprintf('\n### 起始段 vs 稳态：对总 MSE 的贡献 ###\n');
E_start = []; E_steady = [];
for ac = 1:numel(truths)
    tt = truths{ac};
    for k = 1:nf
        tnow = (k-1)*S.params.dt_sec;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx == ac && s.type ~= 7 && ~isnan(s.lat)
                d = haversine(s.lon, s.lat, tlon, tlat);
                life = 1; if isfield(s,'life'), life = s.life; end
                if life <= 5, E_start(end+1) = d; else, E_steady(end+1) = d; end
                break;
            end
        end
    end
end
ns = numel(E_start); ny = numel(E_steady);
mse_s = mean(E_start.^2); mse_y = mean(E_steady.^2);
mse_all = (sum(E_start.^2) + sum(E_steady.^2)) / (ns + ny);
fprintf('起始段(<=5帧): n=%d  rms=%.2f km  贡献MSE=%.1f%%\n', ns, sqrt(mse_s), 100*ns*mse_s/((ns+ny)*mse_all));
fprintf('稳态段(>5帧):  n=%d  rms=%.2f km  贡献MSE=%.1f%%\n', ny, sqrt(mse_y), 100*ny*mse_y/((ns+ny)*mse_all));
fprintf('整体: rms=%.2f km\n', sqrt(mse_all));
fprintf('若起始段完全消除(等于稳态): rms=%.2f km (改善 %.1f%%)\n', ...
    sqrt(mse_y), 100*(1-sqrt(mse_y)/sqrt(mse_all)));

function d = haversine(lon1, lat1, lon2, lat2)
R = 6371.0;
p1 = deg2rad(lat1); p2 = deg2rad(lat2);
dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
d = 2*R*asin(sqrt(a));
end
