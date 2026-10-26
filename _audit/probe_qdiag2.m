% English-output diagnostic: does Q scheduling actually speed up velocity convergence?
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
out = fullfile(root,'_audit','qdiag.txt');
fid = fopen(out,'w');

base = simulation_params_oracle();
p0 = base; p0.imm_q_schedule_enable = false; p0.use_fuzzy_adaptive = false; p0.imm_adapt_mode='none';
p1 = base; p1.imm_q_schedule_enable = true;  p1.use_fuzzy_adaptive = false; p1.imm_adapt_mode='none';
P = {p0, p1};
tag = {'A0-clean','A1-Qsched'};
dt = p0.dt_sec;

for c = 1:2
    r = run('multi_cross', P{c});
    snaps = r.trackSnapshots_R1; nf = numel(snaps);
    fprintf(fid,'\n===== %s  R1 RMSE=%.3f =====\n', tag{c}, r.errorStats_R1.overall.ukf.rms);
    segs = {[1 3],[4 5],[6 10],[11 15],[16 25],[26 40],[41 999]};
    lbl  = {'1-3','4-5','6-10','11-15','16-25','26-40','>40'};
    for q = 1:numel(segs)
        dv = []; dp = [];
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
                    if isfield(s,'truth_idx') && s.truth_idx==ac && s.type~=7 && ~isnan(s.lat) && isfield(s.ukf,'x')
                        age = 1; if isfield(s,'life'), age = s.life; end
                        if age < segs{q}(1) || age > segs{q}(2), break; end
                        k2=min(k+1,nf); k1=max(k-1,1);
                        if k2==k1, break; end
                        [vn,ve] = vel(tt, (k1-1)*dt, (k2-1)*dt);
                        vlon = s.ukf.x(2)*111.32*cosd(tlat); vlat = s.ukf.x(4)*110.57;
                        dv(end+1) = sqrt((vlon-ve)^2 + (vlat-vn)^2)*1000;  % m/s
                        dp(end+1) = hav(s.lon, s.lat, tlon, tlat);
                        break;
                    end
                end
            end
        end
        if isempty(dv), continue; end
        fprintf(fid,'  age %-6s n=%3d  velErr=%6.1f m/s  posRMS=%5.2f km\n', lbl{q}, numel(dv), mean(dv), sqrt(mean(dp.^2)));
    end
    % q trace
    fprintf(fid,'  q trace (track1, frames 1-20): ');
    cnt = 0;
    for k = 1:25
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if s.type ~= 7 && isfield(s.ukf,'q_schedule_current')
                fprintf(fid,'%g ', s.ukf.q_schedule_current); cnt = cnt+1; break;
            end
        end
        if cnt>=20, break; end
    end
    fprintf(fid,'\n');
end
fclose(fid);
type(out);
