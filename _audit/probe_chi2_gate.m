function probe_chi2_gate()
% 用"两站估计一致性"chi2 作为融合门控判据
% chi2 = (x1-x2)' inv(P1+P2) (x1-x2)，2 自由度，一致时 ~2
% 假设：chi2 小 → 两站都可信且一致 → 融合有效；chi2 大 → 融合无效
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','chi2gate.txt'),'w');

r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
dt = r.params.dt_sec; nf = r.scenario.n_frames;
fs = r.all_fused_snapshots;
mi = find(strcmp(r.method_names,'SCC'),1); if isempty(mi), mi=1; end
p2a = r.fusion_eval.pair_to_aircraft;

rec = [];  % [chi2, e1, e2, ef, d1]
for p = 1:numel(r.matched_pairs)
    if p > numel(p2a), break; end
    ac = p2a(p);
    if ac < 1 || ac > numel(r.truthTrajs), continue; end
    tt = r.truthTrajs{ac};
    id1 = r.matched_pairs(p).R1_track_id; id2 = r.matched_pairs(p).R2_track_id;
    for k = 1:nf
        tnow=(k-1)*dt;
        if tnow<tt.time_sec(1)||tnow>tt.time_sec(end), continue; end
        tlat=interp1(tt.time_sec,tt.lat,tnow,'linear');
        tlon=interp1(tt.time_sec,tt.lon,tnow,'linear');
        s1=find_trk(r.trackSnapshots_R1{k},id1);
        s2=find_trk(r.aligned_R2{k},id2);
        sf=[];
        if ~isempty(fs{p,mi})
            tl=fs{p,mi}{k}.trackList;
            if ~isempty(tl), sf=tl{1}; end
        end
        if isempty(s1)||isempty(s2)||isempty(sf), continue; end
        if isfield(sf,'source')&&~strcmp(sf.source,'both'), continue; end
        % 两站差（km）
        [dx,dy] = err_vec(s1.lon,s1.lat,s2.lon,s2.lat);
        P1 = pos_cov_km(s1,tlat); P2 = pos_cov_km(s2,tlat);
        S = P1 + P2;
        chi2 = [dx;dy]' * (S \ [dx;dy]);
        if ~isfinite(chi2) || chi2 < 0, continue; end
        d1 = hav(r.params.radar1_lon,r.params.radar1_lat,s1.lon,s1.lat);
        rec(end+1,:) = [chi2, hav(s1.lon,s1.lat,tlon,tlat), hav(s2.lon,s2.lat,tlon,tlat), hav(sf.lon,sf.lat,tlon,tlat), d1];
    end
end
n = size(rec,1);
r1=sqrt(mean(rec(:,2).^2)); r2=sqrt(mean(rec(:,3).^2)); rf=sqrt(mean(rec(:,4).^2));
fprintf(fid,'n=%d  R1=%.2f R2=%.2f SCC=%.2f (gain %.1f%%)\n\n', n, r1,r2,rf, 100*(min(r1,r2)-rf)/min(r1,r2));

% ---- 按 chi2 分段 ----
edges = [0 2 5 10 20 50 1e9];
fprintf(fid,'=== gain by chi2 bin ===\n');
fprintf(fid,'%-10s %5s %7s %7s %7s %8s\n','chi2','n','R1','R2','SCC','gain%%');
for i=1:numel(edges)-1
    m = rec(:,1)>=edges(i) & rec(:,1)<edges(i+1);
    if sum(m)<15, continue; end
    s = rec(m,:);
    a=sqrt(mean(s(:,2).^2)); b=sqrt(mean(s(:,3).^2)); c=sqrt(mean(s(:,4).^2));
    fprintf(fid,'%-10s %5d %7.2f %7.2f %7.2f %8.1f\n', ...
        sprintf('%.0f-%.0f',edges(i),min(edges(i+1),999)), sum(m), a,b,c, 100*(min(a,b)-c)/min(a,b));
end

% ---- chi2 门控 ----
fprintf(fid,'\n=== gated: fuse if chi2 < thr, else use R2 ===\n');
fprintf(fid,'%-8s %6s %7s %7s %8s\n','thr','n_fuse','R2','gated','gain%%');
for thr = [2 3 5 8 12 20 50 1e9]
    mf = rec(:,1) < thr;
    e = rec(:,4); e(~mf) = rec(~mf,3);
    rg = sqrt(mean(e.^2));
    fprintf(fid,'%-8.0f %6d %7.2f %7.2f %8.1f\n', thr, sum(mf), r2, rg, 100*(min(r1,r2)-rg)/min(r1,r2));
end

% ---- chi2 与"融合是否有效"的相关性 ----
ok = (rec(:,4) < rec(:,2)) & (rec(:,4) < rec(:,3));   % 融合优于两站
fprintf(fid,'\n=== chi2 vs fusion-success ===\n');
fprintf(fid,'  fusion-success frames: %d (%.1f%%)\n', sum(ok), 100*sum(ok)/n);
fprintf(fid,'  mean chi2 | success = %.2f\n', mean(rec(ok,1)));
fprintf(fid,'  mean chi2 | failure = %.2f\n', mean(rec(~ok,1)));
fprintf(fid,'  median chi2 | success = %.2f\n', median(rec(ok,1)));
fprintf(fid,'  median chi2 | failure = %.2f\n', median(rec(~ok,1)));

fclose(fid);
disp(fileread(fullfile(root,'_audit','chi2gate.txt')));
end

function C = pos_cov_km(s, lat)
P = s.ukf.P;
sx = 111.32*cosd(lat); sy = 110.57;
J = diag([sx, sy]);
C = J * P([1 3],[1 3]) * J';
C = (C+C')/2;
if rcond(C) < 1e-12, C = C + 1e-6*eye(2); end
end

function [ex,ey] = err_vec(lon,lat,tlon,tlat)
R=6371; ey=deg2rad(lat-tlat)*R; ex=deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end

function s = find_trk(snap,id)
s=[]; tl=snap.trackList;
for t=1:numel(tl)
    if tl{t}.id==id && tl{t}.type~=7 && ~isnan(tl{t}.lat), s=tl{t}; return; end
end
end

function d = hav(lon1,lat1,lon2,lat2)
R=6371; p1=deg2rad(lat1); p2=deg2rad(lat2);
dp=deg2rad(lat2-lat1); dl=deg2rad(lon2-lon1);
a=sin(dp/2).^2+cos(p1).*cos(p2).*sin(dl/2).^2;
d=2*R*asin(sqrt(a));
end
