function probe_nees()
% NEES 一致性检验：滤波器报告的 P 是否可信
% NEES(k) = e' * inv(P) * e, e 为真值误差(km)，P 为报告位置协方差(km^2)
% 2 自由度下，一致时 E[NEES]=2
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','nees.txt'),'w');

fprintf(fid,'===== adsb_0002 =====\n');
r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
nees_report(r, fid, 'R1', r.trackSnapshots_R1);
nees_report(r, fid, 'R2', r.aligned_R2);

fprintf(fid,'\n===== multi_cross (seed42) =====\n');
r2 = run('multi_cross');
nees_report(r2, fid, 'R1', r2.trackSnapshots_R1);
nees_report(r2, fid, 'R2', r2.aligned_R2);

fclose(fid);
disp(fileread(fullfile(root,'_audit','nees.txt')));
end

function nees_report(r, fid, label, snaps)
dt = r.params.dt_sec; nf = r.scenario.n_frames;
N = []; NIS = [];
for ac = 1:numel(r.truthTrajs)
    tt = r.truthTrajs{ac};
    for k = 1:nf
        tnow = (k-1)*dt;
        if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
        tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
        tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx==ac && s.type~=7 && ~isnan(s.lat) && isfield(s,'ukf')
                [ex,ey] = err_vec(s.lon,s.lat,tlon,tlat);
                P = pos_cov_km(s, tlat);
                e = [ex;ey];
                v = e' * (P \ e);
                if isfinite(v) && v >= 0, N(end+1) = v; end
                if isfield(s,'combined_nis') && isfinite(s.combined_nis) && s.combined_nis>0
                    NIS(end+1) = s.combined_nis;
                end
                break;
            end
        end
    end
end
if isempty(N), fprintf(fid,'  %s: no samples\n', label); return; end
N = sort(N);
fprintf(fid,'  %s: n=%d  NEES mean=%.2f  median=%.2f  p95=%.1f  (consistent if ~2)\n', ...
    label, numel(N), mean(N), median(N), N(min(end,round(0.95*numel(N)))));
fprintf(fid,'      NEES<5.99 占比=%.1f%%  (should be ~95%% if consistent)\n', 100*sum(N<5.99)/numel(N));
if ~isempty(NIS)
    fprintf(fid,'      NIS  mean=%.2f  (measurement dim=3, consistent if ~3)\n', mean(NIS));
end
end

function C = pos_cov_km(s, lat)
P = s.ukf.P;
sx = 111.32*cosd(lat); sy = 110.57;
J = diag([sx, sy]);
C = J * P([1 3],[1 3]) * J';
% 正则化，防止奇异
C = (C+C')/2;
if rcond(C) < 1e-12, C = C + 1e-6*eye(2); end
end

function [ex,ey] = err_vec(lon,lat,tlon,tlat)
R = 6371;
ey = deg2rad(lat-tlat)*R;
ex = deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end
