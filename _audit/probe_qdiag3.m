function probe_qdiag3()
% English-output diagnostic (function form, avoids script local-function issues)
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
out = fullfile(root,'_audit','qdiag3.txt');
fid = fopen(out,'w');

base = simulation_params_oracle();
p0 = base; p0.imm_q_schedule_enable = false; p0.use_fuzzy_adaptive = false; p0.imm_adapt_mode='none';
p1 = base; p1.imm_q_schedule_enable = true;  p1.use_fuzzy_adaptive = false; p1.imm_adapt_mode='none';
P = {p0, p1}; tag = {'A0-clean','A1-Qsched'}; dt = p0.dt_sec;

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
                        t1v=(k1-1)*dt; t2v=(k2-1)*dt;
                        la1=interp1(tt.time_sec,tt.lat,t1v); lo1=interp1(tt.time_sec,tt.lon,t1v);
                        la2=interp1(tt.time_sec,tt.lat,t2v); lo2=interp1(tt.time_sec,tt.lon,t2v);
                        vn = deg2rad(la2-la1)*6371/(t2v-t1v);
                        ve = deg2rad(lo2-lo1)*6371*cosd((la1+la2)/2)/(t2v-t1v);
                        vlon = s.ukf.x(2)*111.32*cosd(tlat); vlat = s.ukf.x(4)*110.57;
                        dv(end+1) = sqrt((vlon-ve)^2 + (vlat-vn)^2)*1000;
                        dp(end+1) = hav_km(s.lon, s.lat, tlon, tlat);
                        break;
                    end
                end
            end
        end
        if isempty(dv), continue; end
        fprintf(fid,'  age %-6s n=%3d  velErr=%6.1f m/s  posRMS=%5.2f km\n', ...
            lbl{q}, numel(dv), mean(dv), sqrt(mean(dp.^2)));
    end
    fprintf(fid,'  q-trace: ');
    cnt=0;
    for k = 1:25
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if s.type ~= 7 && isfield(s.ukf,'q_schedule_current')
                fprintf(fid,'%g ', s.ukf.q_schedule_current); cnt=cnt+1; break;
            end
        end
        if cnt>=20, break; end
    end
    fprintf(fid,'\n');
end
fclose(fid);
disp(fileread(out));
end

function d = hav_km(lon1, lat1, lon2, lat2)
R = 6371.0;
p1 = deg2rad(lat1); p2 = deg2rad(lat2);
dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
d = 2*R*asin(sqrt(a));
end
