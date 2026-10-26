% 诊断：Q 调度是否真的加快了速度收敛？代价是什么？
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));

base = simulation_params_oracle();
p0 = base; p0.imm_q_schedule_enable = false; p0.use_fuzzy_adaptive = false; p0.imm_adapt_mode='none';
p1 = base; p1.imm_q_schedule_enable = true;  p1.use_fuzzy_adaptive = false; p1.imm_adapt_mode='none';

R = cell(1,2); P = {p0, p1};
for c = 1:2
    R{c} = run('multi_cross', P{c});
end

dt = p0.dt_sec;
for c = 1:2
    r = R{c};
    snaps = r.trackSnapshots_R1; nf = numel(snaps);
    fprintf('\n========== 配置 %d (%s) ==========\n', c, char("A0 A1"));
    % 汇总：按 age 分段的速度/位置误差
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
                        d = hav(s.lon, s.lat, tlon, tlat);
                        % 真值速度（数值差分）
                        k2=min(k+1,nf); k1=max(k-1,1);
                        if k2==k1, break; end
                        [vn,ve] = vel(tt, (k1-1)*dt, (k2-1)*dt);
                        % 估计速度（度/秒 -> km/s）
                        vlon = s.ukf.x(2)*111.32*cosd(tlat); vlat = s.ukf.x(4)*110.57;
                        dv(end+1) = sqrt((vlon-ve)^2 + (vlat-vn)^2);
                        dp(end+1) = d;
                        break;
                    end
                end
            end
        end
        if isempty(dv), continue; end
        fprintf('  age %-6s n=%3d  速度误差=%6.1f m/s   位置RMS=%5.2f km\n', ...
            lbl{q}, numel(dv), mean(dv)*1000, sqrt(mean(dp.^2)));
    end
    fprintf('  总体 R1 RMSE = %.3f km\n', r.errorStats_R1.overall.ukf.rms);
end

% 检查 q_schedule 是否真的生效：打印第 1 条航迹前 20 帧的 q
fprintf('\n=== 配置 2 的 Q 调度轨迹（第1条航迹前 25 帧）===\n');
snaps = R{2}.trackSnapshots_R1;
for k = 1:25
    tl = snaps{k}.trackList;
    for t = 1:numel(tl)
        s = tl{t};
        if s.type ~= 7 && isfield(s.ukf,'q_schedule_current')
            fprintf('  frame %2d age=%2d q=%.1f\n', k-1, s.life, s.ukf.q_schedule_current);
            break;
        end
    end
end

function [vn, ve] = vel(tt, t1, t2)
la1 = interp1(tt.time_sec, tt.lat, t1); lo1 = interp1(tt.time_sec, tt.lon, t1);
la2 = interp1(tt.time_sec, tt.lat, t2); lo2 = interp1(tt.time_sec, tt.lon, t2);
R = 6371;
vn = deg2rad(la2-la1)*R/(t2-t1);
ve = deg2rad(lo2-lo1)*R*cosd((la1+la2)/2)/(t2-t1);
end

function d = hav(lon1, lat1, lon2, lat2)
R = 6371.0;
p1 = deg2rad(lat1); p2 = deg2rad(lat2);
dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
d = 2*R*asin(sqrt(a));
end
