function probe_gain_by_dist()
% 按目标距离分段，看融合增益是否随几何变化
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','gain_by_dist.txt'),'w');

r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
dt = r.params.dt_sec; nf = r.scenario.n_frames;
fs = r.all_fused_snapshots;
mi = find(strcmp(r.method_names,'SCC'),1); if isempty(mi), mi=1; end
p2a = r.fusion_eval.pair_to_aircraft;

% 收集：[dist_R1, dist_R2, e1, e2, ef]
rec = [];
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
        e1 = hav(s1.lon,s1.lat,tlon,tlat);
        e2 = hav(s2.lon,s2.lat,tlon,tlat);
        ef = hav(sf.lon,sf.lat,tlon,tlat);
        rec(end+1,:) = [d1, d2, e1, e2, ef];
    end
end
n = size(rec,1);
fprintf(fid,'total n=%d\n\n', n);

% 按 R1 距离分段
edges = [1200 1300 1400 1500 1700];
fprintf(fid,'=== by R1 distance ===\n');
fprintf(fid,'%-12s %5s %7s %7s %7s %8s\n','range','n','R1','R2','SCC','gain%%');
for i=1:numel(edges)-1
    m = rec(:,1)>=edges(i) & rec(:,1)<edges(i+1);
    if sum(m)<20, continue; end
    sub = rec(m,:);
    r1=sqrt(mean(sub(:,3).^2)); r2=sqrt(mean(sub(:,4).^2)); rf=sqrt(mean(sub(:,5).^2));
    fprintf(fid,'%-12s %5d %7.2f %7.2f %7.2f %8.1f\n', ...
        sprintf('%d-%d',edges(i),edges(i+1)), sum(m), r1, r2, rf, 100*(min(r1,r2)-rf)/min(r1,r2));
end

% 按两站平均距离分段（几何综合）
davg = (rec(:,1)+rec(:,2))/2;
edges2 = [1100 1250 1400 1550];
fprintf(fid,'\n=== by mean distance (R1+R2)/2 ===\n');
fprintf(fid,'%-12s %5s %7s %7s %7s %8s\n','range','n','R1','R2','SCC','gain%%');
for i=1:numel(edges2)-1
    m = davg>=edges2(i) & davg<edges2(i+1);
    if sum(m)<20, continue; end
    sub = rec(m,:);
    r1=sqrt(mean(sub(:,3).^2)); r2=sqrt(mean(sub(:,4).^2)); rf=sqrt(mean(sub(:,5).^2));
    fprintf(fid,'%-12s %5d %7.2f %7.2f %7.2f %8.1f\n', ...
        sprintf('%d-%d',edges2(i),edges2(i+1)), sum(m), r1, r2, rf, 100*(min(r1,r2)-rf)/min(r1,r2));
end

% 按 R1/R2 精度比分段
ratio = rec(:,3)./max(rec(:,4),1e-6);
edges3 = [0 0.8 1.0 1.3 3];
fprintf(fid,'\n=== by error ratio e1/e2 ===\n');
fprintf(fid,'%-12s %5s %7s %7s %7s %8s\n','ratio','n','R1','R2','SCC','gain%%');
for i=1:numel(edges3)-1
    m = ratio>=edges3(i) & ratio<edges3(i+1);
    if sum(m)<20, continue; end
    sub = rec(m,:);
    r1=sqrt(mean(sub(:,3).^2)); r2=sqrt(mean(sub(:,4).^2)); rf=sqrt(mean(sub(:,5).^2));
    fprintf(fid,'%-12s %5d %7.2f %7.2f %7.2f %8.1f\n', ...
        sprintf('%.1f-%.1f',edges3(i),edges3(i+1)), sum(m), r1, r2, rf, 100*(min(r1,r2)-rf)/min(r1,r2));
end

fclose(fid);
disp(fileread(fullfile(root,'_audit','gain_by_dist.txt')));
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
