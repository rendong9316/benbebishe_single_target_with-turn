function probe_gated()
% 验证"融合门控"：近距段用 SCC，远距段直接用 R2（先验更近）
% 对比：全段 SCC / 全段 R2 / 门控融合
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','gated.txt'),'w');

r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
dt = r.params.dt_sec; nf = r.scenario.n_frames;
fs = r.all_fused_snapshots;
mi = find(strcmp(r.method_names,'SCC'),1); if isempty(mi), mi=1; end
p2a = r.fusion_eval.pair_to_aircraft;

rec = [];  % [d1, d2, e1, e2, ef]
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
        d1 = hav(r.params.radar1_lon,r.params.radar1_lat,s1.lon,s1.lat);
        d2 = hav(r.params.radar2_lon,r.params.radar2_lat,s2.lon,s2.lat);
        rec(end+1,:) = [d1,d2, hav(s1.lon,s1.lat,tlon,tlat), hav(s2.lon,s2.lat,tlon,tlat), hav(sf.lon,sf.lat,tlon,tlat)];
    end
end
n = size(rec,1);
r1 = sqrt(mean(rec(:,3).^2)); r2 = sqrt(mean(rec(:,4).^2)); rf = sqrt(mean(rec(:,5).^2));

fprintf(fid,'n=%d\n', n);
fprintf(fid,'ALL   R1=%.3f  R2=%.3f  SCC=%.3f  (SCC gain %.1f%% vs best single)\n', ...
    r1, r2, rf, 100*(min(r1,r2)-rf)/min(r1,r2));

% ---- 门控：按 R1 距离阈值 ----
fprintf(fid,'\n=== gated by R1 distance (fuse if d1 < thr, else use R2) ===\n');
fprintf(fid,'%-8s %6s %7s %7s %7s %8s\n','thr','n_fuse','R1','R2','gated','gain%%');
for thr = [1300 1350 1400 1450 1500 1700]
    mf = rec(:,1) < thr;
    e = rec(:,5);
    e(~mf) = rec(~mf,4);      % 未门控的用 R2
    rg = sqrt(mean(e.^2));
    fprintf(fid,'%-8d %6d %7.2f %7.2f %7.2f %8.1f\n', thr, sum(mf), r1, r2, rg, 100*(min(r1,r2)-rg)/min(r1,r2));
end

% ---- 门控：按两站距离差 ----
fprintf(fid,'\n=== gated by |d1-d2| (fuse if close, else use nearer station) ===\n');
dd = abs(rec(:,1)-rec(:,2));
fprintf(fid,'%-8s %6s %7s %7s %7s %8s\n','thr','n_fuse','R1','R2','gated','gain%%');
for thr = [100 150 200 250 400]
    mf = dd < thr;
    e = rec(:,5);
    near = rec(:,4);                 % R2 通常更近
    e(~mf) = near(~mf);
    rg = sqrt(mean(e.^2));
    fprintf(fid,'%-8d %6d %7.2f %7.2f %7.2f %8.1f\n', thr, sum(mf), r1, r2, rg, 100*(min(r1,r2)-rg)/min(r1,r2));
end

% ---- 门控：按 R1 距离 + 精度比（事后上界，说明潜力）----
fprintf(fid,'\n=== oracle gate: fuse only when SCC actually beats both singles ===\n');
mok = (rec(:,5) < rec(:,3)) & (rec(:,5) < rec(:,4));
e = rec(:,5);
e(~mok) = min(rec(~mok,3), rec(~mok,4));
rg = sqrt(mean(e.^2));
fprintf(fid,'fuse frames=%d (%.1f%%)  gated=%.3f km  gain=%.1f%%\n', ...
    sum(mok), 100*sum(mok)/n, rg, 100*(min(r1,r2)-rg)/min(r1,r2));

fclose(fid);
disp(fileread(fullfile(root,'_audit','gated.txt')));
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
