function probe_corr()
% 量化两站估计误差的互相关：解释融合增益为何从 21.9% 掉到 0.5%
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','corr_out.txt'),'w');

% ---- adsb_0002（用缓存）----
fprintf(fid,'===== adsb_0002 =====\n');
try
    r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
    [rho, rms1, rms2, rmsf, n, rho_indep] = corr_of(r);
    fprintf(fid,'n=%d  R1=%.2f  R2=%.2f  fused=%.2f\n', n, rms1, rms2, rmsf);
    fprintf(fid,'measured rho=%.4f\n', rho);
    fprintf(fid,'independent-hypothesis fused=%.2f (gain %.1f%%)\n', rho_indep, 100*(min(rms1,rms2)-rho_indep)/min(rms1,rms2));
    fprintf(fid,'actual gain=%.1f%%\n', 100*(min(rms1,rms2)-rmsf)/min(rms1,rms2));
catch ME
    fprintf(fid,'FAILED %s\n', ME.message);
end

% ---- multi_cross ----
fprintf(fid,'\n===== multi_cross =====\n');
r2 = run('multi_cross');
[rho, rms1, rms2, rmsf, n, rho_indep] = corr_of(r2);
fprintf(fid,'n=%d  R1=%.2f  R2=%.2f  fused=%.2f\n', n, rms1, rms2, rmsf);
fprintf(fid,'measured rho=%.4f\n', rho);
fprintf(fid,'independent-hypothesis fused=%.2f\n', rho_indep);
fprintf(fid,'actual gain=%.1f%%\n', 100*(min(rms1,rms2)-rmsf)/min(rms1,rms2));

fclose(fid);
disp(fileread(fullfile(root,'_audit','corr_out.txt')));
end

function [rho, rms1, rms2, rmsf, n, f_indep] = corr_of(r)
dt = r.params.dt_sec; nf = r.scenario.n_frames;
fs = r.all_fused_snapshots;
mi = find(strcmp(r.method_names,'SCC'),1);
if isempty(mi), mi=1; end
E1=[]; E2=[]; EF=[];
p2a = r.fusion_eval.pair_to_aircraft;
for p = 1:numel(r.matched_pairs)
    if p>numel(p2a), break; end
    ac = p2a(p);
    if ac<1 || ac>numel(r.truthTrajs), continue; end
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
        % 误差向量（km，局部平面）
        [e1x,e1y]=err_vec(s1.lon,s1.lat,tlon,tlat);
        [e2x,e2y]=err_vec(s2.lon,s2.lat,tlon,tlat);
        [efx,efy]=err_vec(sf.lon,sf.lat,tlon,tlat);
        E1=[E1;e1x,e1y]; E2=[E2;e2x,e2y];
        EF=[EF; sqrt(efx^2+efy^2)];
    end
end
n = size(E1,1);
rms1 = sqrt(mean(sum(E1.^2,2)));
rms2 = sqrt(mean(sum(E2.^2,2)));
rmsf = sqrt(mean(EF.^2));
% 标量相关：trace(C12)/sqrt(trace(C11)*trace(C22))
C12 = (E1'*E2)/max(n-1,1);
C11 = (E1'*E1)/max(n-1,1);
C22 = (E2'*E2)/max(n-1,1);
rho = trace(C12)/sqrt(trace(C11)*trace(C22));
% 独立假设下的理论融合误差
f_indep = 1/sqrt(1/rms1^2 + 1/rms2^2);
% 考虑相关的最优线性融合误差
den = rms1^2 + rms2^2 - 2*rho*rms1*rms2;
if den > 0
    w = (rms2^2 - rho*rms1*rms2)/den;
    Eopt = w*E1 + (1-w)*E2;
    f_opt = sqrt(mean(sum(Eopt.^2,2)));
else
    w = NaN; f_opt = NaN;
end
fprintf('  optimal weight w=%.3f  optimal fused=%.2f km (gain %.1f%%)\n', ...
    w, f_opt, 100*(min(rms1,rms2)-f_opt)/min(rms1,rms2));
end

function [ex,ey] = err_vec(lon,lat,tlon,tlat)
R=6371;
ey = deg2rad(lat-tlat)*R;
ex = deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end

function s = find_trk(snap,id)
s=[]; tl=snap.trackList;
for t=1:numel(tl)
    if tl{t}.id==id && tl{t}.type~=7 && ~isnan(tl{t}.lat), s=tl{t}; return; end
end
end
