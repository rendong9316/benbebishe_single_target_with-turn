function probe_adsb_diag()
% ADS-B 场景误差来源诊断：航迹长度 / 转弯率 / 共同帧口径 / 融合覆盖率
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','adsb_diag.txt'),'w');

r = run('adsb_0002');
save(fullfile(root,'_audit','adsb_0002_cache.mat'), '-struct','r','-v7.3');
dt = r.params.dt_sec;
snaps = r.trackSnapshots_R1; nf = numel(snaps);

% ---- 每条航迹的误差序列 ----
trk = struct('id',{},'len',{},'err',{},'turnrate',{},'dist',{});
for ac = 1:numel(r.truthTrajs)
    tt = r.truthTrajs{ac};
    ids = []; E = []; TR = []; D = [];
    for k = 1:nf
        tnow = (k-1)*dt;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        % 真值转弯率（用航向差分）
        k2=min(k+1,nf); k1=max(k-1,1);
        if k2>k1
            [vn,ve] = vel(tt,(k1-1)*dt,(k2-1)*dt);
            hdg = atan2(ve,vn)*180/pi;
        else
            hdg = NaN;
        end
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx==ac && s.type~=7 && ~isnan(s.lat)
                ids(end+1) = s.id;
                E(end+1) = hav(s.lon,s.lat,tlon,tlat);
                TR(end+1) = hdg;
                D(end+1) = hav(r.params.radar1_lon, r.params.radar1_lat, s.lon, s.lat);
                break;
            end
        end
    end
    if isempty(E), continue; end
    trk(end+1) = struct('id',ac,'len',numel(E),'err',E,'turnrate',TR,'dist',D);
end

fprintf(fid,'\n===== 按航迹长度分段（R1）=====\n');
segs = {[10 20],[21 30],[31 40],[41 60]};
lbl = {'10-20','21-30','31-40','41-60'};
for q = 1:numel(segs)
    E = [];
    for i = 1:numel(trk)
        if trk(i).len >= segs{q}(1) && trk(i).len <= segs{q}(2)
            E = [E, trk(i).err];
        end
    end
    if isempty(E), continue; end
    fprintf(fid,'  长度 %-6s 航迹数=%2d n=%4d  rms=%6.2f km  mean=%6.2f\n', ...
        lbl{q}, sum([trk.len]>=segs{q}(1) & [trk.len]<=segs{q}(2)), numel(E), sqrt(mean(E.^2)), mean(E));
end

% ---- 按转弯率分段 ----
fprintf(fid,'\n===== 按真值转弯率分段（R1）=====\n');
TRs = {[0 0.05],[0.05 0.15],[0.15 0.4],[0.4 5]};
lbl2 = {'<0.05','0.05-0.15','0.15-0.4','>0.4'};
% 需要算航向变化率：用相邻帧航向差
for q = 1:numel(TRs)
    E = [];
    for i = 1:numel(trk)
        hd = trk(i).turnrate;
        if numel(hd) < 2, continue; end
        rate = abs([0, diff(unwrap(hd*pi/180))])*180/pi/dt;
        for j = 1:numel(rate)
            if rate(j) >= TRs{q}(1) && rate(j) < TRs{q}(2)
                E(end+1) = trk(i).err(j);
            end
        end
    end
    if isempty(E), continue; end
    fprintf(fid,'  转弯率 %-10s n=%4d  rms=%6.2f km  mean=%6.2f\n', lbl2{q}, numel(E), sqrt(mean(E.^2)), mean(E));
end

% ---- 按距离分段 ----
fprintf(fid,'\n===== 按目标距离R1分段（R1）=====\n');
Ds = {[1200 1350],[1350 1450],[1450 1600]};
lbl3 = {'1200-1350','1350-1450','1450-1600'};
for q = 1:numel(Ds)
    E = [];
    for i = 1:numel(trk)
        for j = 1:numel(trk(i).dist)
            if trk(i).dist(j) >= Ds{q}(1) && trk(i).dist(j) < Ds{q}(2)
                E(end+1) = trk(i).err(j);
            end
        end
    end
    if isempty(E), continue; end
    fprintf(fid,'  距离 %-10s n=%4d  rms=%6.2f km\n', lbl3{q}, numel(E), sqrt(mean(E.^2)));
end

% ---- 共同帧口径 ----
fprintf(fid,'\n===== 共同帧 vs 全帧 =====\n');
% 对每个匹配对，找两站都有该目标航迹的帧
mi_scc = find(strcmp(r.method_names,'SCC'),1);
fs = r.all_fused_snapshots;
E1 = []; E2 = []; EF = [];
for p = 1:numel(r.matched_pairs)
    ac = r.fusion_eval.pair_to_aircraft(p);
    if ac < 1 || ac > numel(r.truthTrajs), continue; end
    tt = r.truthTrajs{ac};
    id1 = r.matched_pairs(p).R1_track_id; id2 = r.matched_pairs(p).R2_track_id;
    for k = 1:nf
        tnow=(k-1)*dt;
        if tnow<tt.time_sec(1) || tnow>tt.time_sec(end), continue; end
        tlat=interp1(tt.time_sec,tt.lat,tnow,'linear'); tlon=interp1(tt.time_sec,tt.lon,tnow,'linear');
        s1 = find_trk(snaps{k}, id1);
        s2 = find_trk(r.aligned_R2{k}, id2);
        sf = [];
        if ~isempty(fs{p,mi_scc})
            tl = fs{p,mi_scc}{k}.trackList;
            if ~isempty(tl), sf = tl{1}; end
        end
        if isempty(s1)||isempty(s2)||isempty(sf), continue; end
        if strcmp(sf.source,'both')==0, continue; end
        E1(end+1)=hav(s1.lon,s1.lat,tlon,tlat);
        E2(end+1)=hav(s2.lon,s2.lat,tlon,tlat);
        EF(end+1)=hav(sf.lon,sf.lat,tlon,tlat);
    end
end
fprintf(fid,'  共同帧 n=%d  R1=%.2f  R2=%.2f  SCC=%.2f\n', numel(E1), sqrt(mean(E1.^2)), sqrt(mean(E2.^2)), sqrt(mean(EF.^2)));

% ---- 融合覆盖率 ----
fprintf(fid,'\n===== 融合源分布（SCC）=====\n');
nb=0; n1=0; n2=0;
for p = 1:size(fs,1)
    sn = fs{p,mi_scc};
    for k = 1:numel(sn)
        tl = sn{k}.trackList;
        if isempty(tl), continue; end
        s = tl{1}.source;
        if strcmp(s,'both'), nb=nb+1; elseif strcmp(s,'R1_only'), n1=n1+1; else, n2=n2+1; end
    end
end
fprintf(fid,'  both=%d R1_only=%d R2_only=%d 覆盖率=%.1f%%\n', nb,n1,n2,100*nb/(nb+n1+n2));

fclose(fid);
disp(fileread(fullfile(root,'_audit','adsb_diag.txt')));
end

function s = find_trk(snap, id)
s = [];
tl = snap.trackList;
for t = 1:numel(tl)
    if tl{t}.id == id && tl{t}.type ~= 7 && ~isnan(tl{t}.lat)
        s = tl{t}; return;
    end
end
end

function [vn,ve] = vel(tt,t1,t2)
la1=interp1(tt.time_sec,tt.lat,t1); lo1=interp1(tt.time_sec,tt.lon,t1);
la2=interp1(tt.time_sec,tt.lat,t2); lo2=interp1(tt.time_sec,tt.lon,t2);
R=6371;
vn=deg2rad(la2-la1)*R/(t2-t1);
ve=deg2rad(lo2-lo1)*R*cosd((la1+la2)/2)/(t2-t1);
end

function d = hav(lon1,lat1,lon2,lat2)
R=6371; p1=deg2rad(lat1); p2=deg2rad(lat2);
dp=deg2rad(lat2-lat1); dl=deg2rad(lon2-lon1);
a=sin(dp/2).^2+cos(p1).*cos(p2).*sin(dl/2).^2;
d=2*R*asin(sqrt(a));
end
