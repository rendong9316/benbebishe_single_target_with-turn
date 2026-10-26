function probe_rho_estimate()
% 验证：能否只用两站估计序列（不用真值）估出互相关 rho
% 原理：两站位置差 d = x1 - x2，若独立 Var(d)=P1+P2；若相关 Var(d)=P1+P2-2P12
%       故 P12 = (P1+P2-Var(d))/2
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','rho_est.txt'),'w');

% ---- adsb_0002（缓存）----
fprintf(fid,'===== adsb_0002 =====\n');
r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
check_scene(r, fid);

% ---- multi_cross s42 ----
fprintf(fid,'\n===== multi_cross (seed42) =====\n');
r2 = run('multi_cross');
check_scene(r2, fid);

fclose(fid);
disp(fileread(fullfile(root,'_audit','rho_est.txt')));
end

function check_scene(r, fid)
dt = r.params.dt_sec; nf = r.scenario.n_frames;
p2a = r.fusion_eval.pair_to_aircraft;

D_all = []; E1_all = []; E2_all = [];
P1_all = []; P2_all = [];
for p = 1:numel(r.matched_pairs)
    if p > numel(p2a), break; end
    ac = p2a(p);
    if ac < 1 || ac > numel(r.truthTrajs), continue; end
    tt = r.truthTrajs{ac};
    id1 = r.matched_pairs(p).R1_track_id; id2 = r.matched_pairs(p).R2_track_id;
    for k = 1:nf
        tnow = (k-1)*dt;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        s1 = find_trk(r.trackSnapshots_R1{k}, id1);
        s2 = find_trk(r.aligned_R2{k}, id2);
        if isempty(s1) || isempty(s2), continue; end
        % 真值误差（km 局部平面）
        [e1x,e1y] = err_vec(s1.lon,s1.lat,tlon,tlat);
        [e2x,e2y] = err_vec(s2.lon,s2.lat,tlon,tlat);
        % 两站差（可观测）
        [dx,dy] = err_vec(s1.lon,s1.lat,s2.lon,s2.lat);
        E1_all = [E1_all; e1x,e1y];
        E2_all = [E2_all; e2x,e2y];
        D_all  = [D_all;  dx, dy];
        % 位置协方差（度 -> km，近似）
        P1_all = [P1_all; pos_cov_km(s1, tlat)];
        P2_all = [P2_all; pos_cov_km(s2, tlat)];
    end
end
n = size(E1_all,1);
if n < 10, fprintf(fid,'  样本不足\n'); return; end

% ---- 方法 A：真值误差（上界参考，实际不可用）----
C12 = (E1_all'*E2_all)/(n-1);
C11 = (E1_all'*E1_all)/(n-1);
C22 = (E2_all'*E2_all)/(n-1);
rho_truth = trace(C12)/sqrt(trace(C11)*trace(C22));

% ---- 方法 B：位置差（可观测，在线可用）----
Cd = (D_all'*D_all)/(n-1);
P1m = mean(reshape(P1_all,2,2,[]),3);
P2m = mean(reshape(P2_all,2,2,[]),3);
P12_est = (P1m + P2m - Cd)/2;
rho_est = trace(P12_est)/sqrt(trace(P1m)*trace(P2m));

% ---- 方法 C：位置差的标量形式（更稳健）----
var_d = trace(Cd)/2;              % 每轴平均方差
var_1 = trace(P1m)/2;
var_2 = trace(P2m)/2;
rho_scalar = (var_1 + var_2 - var_d)/(2*sqrt(var_1*var_2));

fprintf(fid,'  n=%d\n', n);
fprintf(fid,'  rho_truth (needs truth)      = %+.4f\n', rho_truth);
fprintf(fid,'  rho_est   (position diff,2x2)= %+.4f\n', rho_est);
fprintf(fid,'  rho_scalar(position diff)    = %+.4f\n', rho_scalar);
fprintf(fid,'  var_1=%.2f var_2=%.2f var_d=%.2f\n', var_1, var_2, var_d);
end

function C = pos_cov_km(s, lat)
% 从 ukf.P 取 2x2 位置协方差，度^2 -> km^2
P = s.ukf.P;
sx = 111.32*cosd(lat);   % km per deg lon
sy = 110.57;             % km per deg lat
J = diag([sx, sy]);
C = J * P([1 3],[1 3]) * J';
end

function [ex,ey] = err_vec(lon,lat,tlon,tlat)
R = 6371;
ey = deg2rad(lat-tlat)*R;
ex = deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end

function s = find_trk(snap,id)
s = []; tl = snap.trackList;
for t = 1:numel(tl)
    if tl{t}.id == id && tl{t}.type ~= 7 && ~isnan(tl{t}.lat), s = tl{t}; return; end
end
end
