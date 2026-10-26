% 速度收敛验证：两点起始的速度误差如何随帧演化，以及它如何影响纯预测帧
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
S = load(fullfile(root,'_audit','mc_current_geom.mat'));
snaps = S.trackSnapshots_R1; nf = numel(snaps); dt = S.params.dt_sec;

fprintf('\n### 速度误差 vs 航迹年龄（R1，km/s）###\n');
fprintf('目标 年龄  n   速度误差均值  位置误差均值  漏检率\n');
for ac = 1:numel(S.truthTrajs)
    tt = S.truthTrajs{ac};
    rec = [];  % [life, dv, dp, updated]
    for k = 1:nf
        tnow = (k-1)*dt;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        % 真值速度（数值差分，km/s）
        k2 = min(k+1, nf); k1 = max(k-1, 1);
        t2 = (k2-1)*dt; t1 = (k1-1)*dt;
        if t2 == t1, continue; end
        la2 = interp1(tt.time_sec, tt.lat, t2, 'linear');
        lo2 = interp1(tt.time_sec, tt.lon, t2, 'linear');
        la1 = interp1(tt.time_sec, tt.lat, t1, 'linear');
        lo1 = interp1(tt.time_sec, tt.lon, t1, 'linear');
        [vn_true, ve_true] = vel_kmps(lo1, la1, lo2, la2, t2-t1);
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx == ac && s.type ~= 7 && ~isnan(s.lat)
                if isfield(s,'ukf') && isfield(s.ukf,'x')
                    vlon = s.ukf.x(2); vlat = s.ukf.x(4);
                    [vn_e, ve_e] = vel_kmps(0,0, vlon*cosd(tlat), vlat, 1);  % 度/秒 → km/s 近似
                    dv = sqrt((vn_e-vn_true)^2 + (ve_e-ve_true)^2);
                else
                    dv = NaN;
                end
                d = haversine(s.lon, s.lat, tlon, tlat);
                upd = 1; if isfield(s,'updated'), upd = double(s.updated); end
                life = 1; if isfield(s,'life'), life = s.life; end
                rec(end+1,:) = [life, dv, d, upd];
                break;
            end
        end
    end
    segs = {[1 5],[6 10],[11 20],[21 40],[41 999]};
    lbl  = {'1-5','6-10','11-20','21-40','>40'};
    for q = 1:numel(segs)
        m = rec(:,1) >= segs{q}(1) & rec(:,1) <= segs{q}(2);
        if sum(m)==0, continue; end
        fprintf('%3d  %-6s %3d   %8.4f     %8.2f     %.2f\n', ac, lbl{q}, sum(m), ...
            mean(rec(m,2),'omitnan'), mean(rec(m,3)), 1-mean(rec(m,4)));
    end
    fprintf('\n');
end

fprintf('\n### 纯预测帧 vs 更新帧的位置误差 ###\n');
Ep = []; Eu = [];
for ac = 1:numel(S.truthTrajs)
    tt = S.truthTrajs{ac};
    for k = 1:nf
        tnow = (k-1)*dt;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx == ac && s.type ~= 7 && ~isnan(s.lat)
                d = haversine(s.lon, s.lat, tlon, tlat);
                upd = 1; if isfield(s,'updated'), upd = double(s.updated); end
                if upd >= 1, Eu(end+1) = d; else, Ep(end+1) = d; end
                break;
            end
        end
    end
end
fprintf('更新帧: n=%d rms=%.2f km\n', numel(Eu), sqrt(mean(Eu.^2)));
fprintf('纯预测帧: n=%d rms=%.2f km\n', numel(Ep), sqrt(mean(Ep.^2)));
fprintf('纯预测帧占比=%.1f%%\n', 100*numel(Ep)/(numel(Ep)+numel(Eu)));

function [vn, ve] = vel_kmps(lon1, lat1, lon2, lat2, dt)
R = 6371.0;
dn = deg2rad(lat2-lat1) * R;
de = deg2rad(lon2-lon1) * R * cos(deg2rad((lat1+lat2)/2));
vn = dn/dt; ve = de/dt;
end

function d = haversine(lon1, lat1, lon2, lat2)
R = 6371.0;
p1 = deg2rad(lat1); p2 = deg2rad(lat2);
dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
d = 2*R*asin(sqrt(a));
end
