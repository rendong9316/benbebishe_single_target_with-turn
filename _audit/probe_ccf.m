function probe_ccf()
% CCF 离线验证：用 NIS/NEES 校准 P 后，融合增益能恢复多少
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','ccf.txt'),'w');

fprintf(fid,'===== adsb_0002 =====\n');
r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
run_case(r, fid);

fprintf(fid,'\n===== multi_cross (seed42) =====\n');
r2 = run('multi_cross');
run_case(r2, fid);

fclose(fid);
disp(fileread(fullfile(root,'_audit','ccf.txt')));
end

function run_case(r, fid)
dt = r.params.dt_sec; nf = r.scenario.n_frames;
fs = r.all_fused_snapshots;
mi = find(strcmp(r.method_names,'SCC'),1); if isempty(mi), mi=1; end
p2a = r.fusion_eval.pair_to_aircraft;

% 收集逐帧数据
E1=[]; E2=[]; EF=[]; P1c={}; P2c={}; NIS=[];
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
        [e1x,e1y]=err_vec(s1.lon,s1.lat,tlon,tlat);
        [e2x,e2y]=err_vec(s2.lon,s2.lat,tlon,tlat);
        [efx,efy]=err_vec(sf.lon,sf.lat,tlon,tlat);
        E1=[E1;e1x,e1y]; E2=[E2;e2x,e2y];
        EF=[EF; sqrt(efx^2+efy^2)];
        P1c{end+1}=pos_cov_km(s1,tlat);
        P2c{end+1}=pos_cov_km(s2,tlat);
        if isfield(s1,'combined_nis')&&isfinite(s1.combined_nis)&&s1.combined_nis>0
            NIS(end+1)=s1.combined_nis;
        end
    end
end
n = size(E1,1);
if n < 10, fprintf(fid,'  samples insufficient\n'); return; end

rms1=sqrt(mean(sum(E1.^2,2))); rms2=sqrt(mean(sum(E2.^2,2))); rmsf=sqrt(mean(EF.^2));
C12=(E1'*E2)/(n-1); C11=(E1'*E1)/(n-1); C22=(E2'*E2)/(n-1);
rho = trace(C12)/sqrt(trace(C11)*trace(C22));

% ---- 校准因子 ----
% NEES 均值 = 2*kappa (2 自由度)，故 kappa = mean(NEES)/2
NEES = zeros(n,1);
for i=1:n
    e = E1(i,:)';
    P = P1c{i};
    NEES(i) = e'*(P\e);
end
kappa_nees = mean(NEES)/2;
if isempty(NIS), kappa_nis = NaN; else, kappa_nis = mean(NIS)/3; end

fprintf(fid,'  n=%d  R1=%.2f  R2=%.2f  SCC=%.2f  rho=%+.3f\n', n, rms1, rms2, rmsf, rho);
fprintf(fid,'  kappa(NEES)=%.2f  kappa(NIS)=%.2f  (1=consistent)\n', kappa_nees, kappa_nis);
fprintf(fid,'  report P1=%.1f km^2  actual var1=%.1f km^2\n', ...
    mean(cellfun(@(x) trace(x)/2, P1c)), trace(C11)/2);

% ---- 方案 A：SCC（原始 P）----
% 已由实测 rmsf 给出

% ---- 方案 B：CCF-SCC（校准 P 后按信息矩阵融合）----
E_ccf = zeros(n,2);
for i=1:n
    Pa = kappa_nees * P1c{i};
    Pb = kappa_nees * P2c{i};
    % 信息融合（等价 SCC 但用校准后 P）
    Wa = inv(Pa); Wb = inv(Pb);
    Wf = Wa + Wb; Pf = inv(Wf);
    % 用误差表示：x_f = Pf*(Wa*x1 + Wb*x2) => e_f = Pf*(Wa*e1 + Wb*e2)
    E_ccf(i,:) = (Pf*(Wa*E1(i,:)' + Wb*E2(i,:)'))';
end
rms_ccf = sqrt(mean(sum(E_ccf.^2,2)));

% ---- 方案 C：最优权重（真值 rho + 真实方差，上界）----
v1 = trace(C11)/2; v2 = trace(C22)/2;
s1=sqrt(v1); s2=sqrt(v2);
den = v1+v2-2*rho*s1*s2;
w = (v2-rho*s1*s2)/den;
E_opt = w*E1 + (1-w)*E2;
rms_opt = sqrt(mean(sum(E_opt.^2,2)));

fprintf(fid,'  SCC(orig P)  = %.2f km  (gain %.1f%%)\n', rmsf, 100*(min(rms1,rms2)-rmsf)/min(rms1,rms2));
fprintf(fid,'  CCF-SCC      = %.2f km  (gain %.1f%%)\n', rms_ccf, 100*(min(rms1,rms2)-rms_ccf)/min(rms1,rms2));
fprintf(fid,'  OPT(w=%.2f,true rho) = %.2f km  (gain %.1f%%)\n', w, rms_opt, 100*(min(rms1,rms2)-rms_opt)/min(rms1,rms2));
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
R = 6371;
ey = deg2rad(lat-tlat)*R;
ex = deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end

function s = find_trk(snap,id)
s=[]; tl=snap.trackList;
for t=1:numel(tl)
    if tl{t}.id==id && tl{t}.type~=7 && ~isnan(tl{t}.lat), s=tl{t}; return; end
end
end
