function metrics = evaluate_paper_metrics_online(r)
%EVALUATE_PAPER_METRICS_ONLINE 论文六项考核指标（在线主链路口径）
%
%   metrics = evaluate_paper_metrics_online(result)
%
%   输入 result 为 run() 的返回值（在线主链路）。
%   输出 6 项指标：
%     1) RMSE（单站 + 各融合算法）
%     2) 航迹平均长度（真实/虚假分开 + 系统航迹 vs 单站基线）
%     3) 正确匹配概率（跨站配对）
%     4) 虚假航迹率
%     5) 融合覆盖率（双源帧 / 全部融合帧）
%     6) 共同帧 RMSE（公平对比口径）
%   外加一致性诊断（NEES / ρ / 理论界）与长度比。

    metrics = struct();
    metrics.scenario = r.scenario.name;
    metrics.n_targets = r.scenario.n_targets;
    metrics.n_frames = r.n_frames;
    dt = r.params.dt_sec;

    %% ---------- 指标 1：RMSE ----------
    m1 = struct();
    m1.R1_rms = r.errorStats_R1.overall.ukf.rms;
    m1.R1_n   = r.errorStats_R1.overall.ukf.n;
    m1.R2_rms = r.errorStats_R2.overall.ukf.rms;
    m1.R2_n   = r.errorStats_R2.overall.ukf.n;
    fo = r.fusion_eval.overall;
    m1.fusion = struct('method', {}, 'rms', {}, 'n', {}, 'median', {});
    for i = 1:numel(fo)
        m1.fusion(i).method = fo(i).method;
        m1.fusion(i).rms    = fo(i).s.rms;
        m1.fusion(i).n      = fo(i).s.n;
        m1.fusion(i).median = fo(i).s.median;
    end
    metrics.rmse = m1;

    %% ---------- 指标 2 & 4：航迹长度 / 虚假航迹率 ----------
    m2 = track_length_stats(r.trackSnapshots_R1, 'R1', r.n_frames);
    m2_R2 = track_length_stats(r.aligned_R2, 'R2', r.n_frames);
    metrics.track_length = m2;
    metrics.track_length_R2 = m2_R2;

    %% ---------- 指标 3：正确匹配概率 ----------
    m3 = struct('n_pairs', 0, 'n_correct', 0, 'n_wrong', 0, 'n_missed', 0, ...
                'match_rate', NaN, 'wrong_rate', NaN, 'precision', NaN, 'recall', NaN);
    pairs = r.matched_pairs;
    if ~isempty(pairs)
        n_correct = 0; n_wrong = 0;
        covered = false(1, r.scenario.n_targets);
        for p = 1:numel(pairs)
            t1 = pairs(p).truth1;  t2 = pairs(p).truth2;
            if isfinite(t1) && isfinite(t2) && t1 == t2 && t1 > 0
                n_correct = n_correct + 1;
                if t1 <= numel(covered), covered(t1) = true; end
            else
                n_wrong = n_wrong + 1;
            end
        end
        m3.n_pairs   = numel(pairs);
        m3.n_correct = n_correct;
        m3.n_wrong   = n_wrong;
        m3.n_missed  = sum(~covered);
        m3.match_rate = n_correct / max(r.scenario.n_targets, 1);
        m3.wrong_rate = n_wrong / max(numel(pairs), 1);
        m3.precision = n_correct / max(numel(pairs), 1);
        % 查全率的分母必须是"应配对的目标数"。原先写成 sum(covered)+n_wrong，
        % 而 covered 就是被正确配对覆盖的目标，无误配时分母恒等于分子，recall 恒为 100%，
        % 会把"37 个目标只配上 20 个"报成"查全率 100%"。
        m3.recall    = n_correct / max(r.scenario.n_targets, 1);
    end
    metrics.matching = m3;

    %% ---------- 指标 5：融合覆盖率 ----------
    m5 = struct('both', 0, 'r1_only', 0, 'r2_only', 0, 'coverage', NaN);
    for k = 1:r.n_frames
        CTk = r.ctSnap{k};
        if isempty(CTk), continue; end
        for i = 1:numel(CTk)
            if CTk(i).n_src == 2
                m5.both = m5.both + 1;
            elseif CTk(i).src1_id > 0
                m5.r1_only = m5.r1_only + 1;
            elseif CTk(i).src2_id > 0
                m5.r2_only = m5.r2_only + 1;
            end
        end
    end
    m5.coverage = m5.both / max(m5.both + m5.r1_only + m5.r2_only, 1);
    metrics.coverage = m5;

    %% ---------- 指标 6：共同帧 RMSE（公平对比）----------
    [m6, E1, E2] = common_frame_errors(r, dt);
    metrics.common_frame = m6;

    %% ---------- 航迹长度对比（开题"长度提升"指标）----------
    metrics.length = length_metrics(r.lengthTable);

    %% ---------- 一致性诊断 ----------
    metrics.consistency = consistency_analysis(r, E1, E2, m6);

    %% ---------- 打印 ----------
    fprintf('\n========== PAPER METRICS (online): %s ==========\n', metrics.scenario);
    fprintf('targets=%d  frames=%d\n', metrics.n_targets, metrics.n_frames);
    fprintf('\n[1] RMSE (km)\n');
    fprintf('  R1_only   n=%5d  rms=%7.3f\n', m1.R1_n, m1.R1_rms);
    fprintf('  R2_only   n=%5d  rms=%7.3f\n', m1.R2_n, m1.R2_rms);
    for i = 1:numel(m1.fusion)
        fprintf('  %-9s n=%5d  rms=%7.3f\n', m1.fusion(i).method, m1.fusion(i).n, m1.fusion(i).rms);
    end
    fprintf('\n[2] TRACK LENGTH (frames)\n');
    fprintf('  R1: total=%d  real=%d (mean=%.1f)  false=%d\n', ...
        m2.n_total, m2.n_real, m2.real_mean, m2.n_false);
    fprintf('  R2: total=%d  real=%d (mean=%.1f)  false=%d\n', ...
        m2_R2.n_total, m2_R2.n_real, m2_R2.real_mean, m2_R2.n_false);
    fprintf('  系统航迹: 目标 %d，比值均值 %.3f 中位 %.3f，延长 %d 缩短 %d\n', ...
        metrics.length.n_targets, metrics.length.ratio_mean, metrics.length.ratio_median, ...
        metrics.length.n_longer, metrics.length.n_shorter);
    fprintf('\n[3] MATCHING\n');
    fprintf('  pairs=%d  correct=%d  wrong=%d  missed=%d  precision=%.1f%%  recall=%.1f%%\n', ...
        m3.n_pairs, m3.n_correct, m3.n_wrong, m3.n_missed, ...
        100*m3.precision, 100*m3.recall);
    fprintf('\n[4] FALSE TRACK RATE\n');
    fprintf('  R1=%.1f%%  R2=%.1f%%\n', 100*m2.false_rate, 100*m2_R2.false_rate);
    fprintf('\n[5] FUSION COVERAGE\n');
    fprintf('  both=%d  R1_only=%d  R2_only=%d  coverage=%.1f%%\n', ...
        m5.both, m5.r1_only, m5.r2_only, 100*m5.coverage);
    fprintf('\n[6] COMMON-FRAME RMSE (fair comparison)\n');
    fprintf('  n=%d  R1=%.3f  R2=%.3f  fused=%.3f  gain_vs_best_single=%.1f%%\n', ...
        m6.n, m6.R1, m6.R2, m6.fused, 100*(min(m6.R1,m6.R2)-m6.fused)/min(m6.R1,m6.R2));
    c = metrics.consistency;
    fprintf('\n[7] CONSISTENCY / LIMITS\n');
    fprintf('  NEES: R1=%.2f  R2=%.2f   [consistent ~2]\n', c.nees_R1, c.nees_R2);
    fprintf('  rho(R1,R2)=%+.3f\n', c.rho);
    fprintf('  theoretical: indep=%.3f  optimal(rho)=%.3f  actual=%.3f\n', ...
        c.indep_rms, c.opt_rms, m6.fused);
    fprintf('====================================================\n');
end

% =====================================================================
function L = length_metrics(T)
    L = struct('n_targets', 0, 'ratio_mean', NaN, 'ratio_median', NaN, ...
                'ratio_max', NaN, 'ratio_min', NaN, 'n_longer', 0, 'n_shorter', 0, ...
                'single_mean', NaN, 'system_mean', NaN);
    if isempty(T), return; end
    ct = [T.ct_len]; sg = [T.single]; rt = [T.ratio];
    ok = sg > 0 & ct > 0 & isfinite(rt);
    L.n_targets = numel(T);
    if ~any(ok), return; end
    L.ratio_mean = mean(rt(ok));
    L.ratio_median = median(rt(ok));
    L.ratio_max = max(rt(ok));
    L.ratio_min = min(rt(ok));
    L.n_longer = sum(rt(ok) > 1);
    L.n_shorter = sum(rt(ok) < 1);
    L.single_mean = mean(sg(ok));
    L.system_mean = mean(ct(ok));
end

% =====================================================================
function [m6, E1, E2] = common_frame_errors(r, dt)
%COMMON_FRAME_ERRORS 共同帧上两站与融合的误差向量
    m6 = struct('n', 0, 'R1', NaN, 'R2', NaN, 'fused', NaN);
    E1 = zeros(0,2); E2 = zeros(0,2);
    pairs = r.matched_pairs;
    if isempty(pairs), return; end

    e1 = []; e2 = []; ef = [];
    for p = 1:numel(pairs)
        tid = pairs(p).truth1;
        if ~isfinite(tid) || tid ~= pairs(p).truth2, continue; end
        if tid < 1 || tid > numel(r.truthTrajs), continue; end
        tt = r.truthTrajs{tid};
        id1 = pairs(p).R1_track_id; id2 = pairs(p).R2_track_id;
        for k = 1:r.n_frames
            tnow = (k-1)*dt;
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
            tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            s1 = find_trk(r.trackSnapshots_R1{k}, id1);
            s2 = find_trk(r.aligned_R2{k}, id2);
            if isempty(s1) || isempty(s2), continue; end
            % 只取系统航迹确为双源的帧（保证公平比较）
            CTk = r.ctSnap{k};
            sf = [];
            for i = 1:numel(CTk)
                if CTk(i).n_src == 2 && CTk(i).src1_id == id1 && CTk(i).src2_id == id2
                    sf = CTk(i);  break;
                end
            end
            if isempty(sf), continue; end
            e1(end+1) = hav(s1.lon, s1.lat, tlon, tlat); %#ok<AGROW>
            e2(end+1) = hav(s2.lon, s2.lat, tlon, tlat); %#ok<AGROW>
            ef(end+1) = hav(sf.x(1), sf.x(3), tlon, tlat); %#ok<AGROW>
        end
    end
    if isempty(e1), return; end
    m6.n = numel(e1);
    m6.R1 = sqrt(mean(e1.^2));
    m6.R2 = sqrt(mean(e2.^2));
    m6.fused = sqrt(mean(ef.^2));
    % 二维误差向量（经/纬向，km），用于相关性与 NEES
    E1 = zeros(numel(e1), 2); E2 = zeros(numel(e2), 2);
    idx = 1;
    for p = 1:numel(pairs)
        tid = pairs(p).truth1;
        if ~isfinite(tid) || tid ~= pairs(p).truth2, continue; end
        if tid < 1 || tid > numel(r.truthTrajs), continue; end
        tt = r.truthTrajs{tid};
        id1 = pairs(p).R1_track_id; id2 = pairs(p).R2_track_id;
        for k = 1:r.n_frames
            tnow = (k-1)*dt;
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
            tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            s1 = find_trk(r.trackSnapshots_R1{k}, id1);
            s2 = find_trk(r.aligned_R2{k}, id2);
            if isempty(s1) || isempty(s2), continue; end
            CTk = r.ctSnap{k};
            sf = [];
            for i = 1:numel(CTk)
                if CTk(i).n_src == 2 && CTk(i).src1_id == id1 && CTk(i).src2_id == id2
                    sf = CTk(i);  break;
                end
            end
            if isempty(sf), continue; end
            [ax, ay] = err_km(s1.lon, s1.lat, tlon, tlat);
            [bx, by] = err_km(s2.lon, s2.lat, tlon, tlat);
            E1(idx,:) = [ax, ay];
            E2(idx,:) = [bx, by];
            idx = idx + 1;
        end
    end
end

% =====================================================================
function c = consistency_analysis(r, E1, E2, m6)
%CONSISTENCY_ANALYSIS NEES / 互相关 / 融合理论界
    c = struct('nees_R1', NaN, 'nees_R2', NaN, 'rho', NaN, ...
               'indep_rms', NaN, 'opt_rms', NaN, 'n_common', 0);
    c.nees_R1 = nees_of(r.trackSnapshots_R1, r.truthTrajs, r.n_frames, r.params.dt_sec);
    c.nees_R2 = nees_of(r.aligned_R2, r.truthTrajs, r.n_frames, r.params.dt_sec);
    n = size(E1, 1);
    if n < 10, return; end
    c.n_common = n;
    C11 = (E1'*E1)/(n-1);
    C22 = (E2'*E2)/(n-1);
    C12 = (E1'*E2)/(n-1);
    c.rho = trace(C12)/sqrt(trace(C11)*trace(C22));
    rms1 = sqrt(mean(sum(E1.^2,2)));
    rms2 = sqrt(mean(sum(E2.^2,2)));
    c.indep_rms = 1/sqrt(1/rms1^2 + 1/rms2^2);
    v1 = trace(C11)/2; v2 = trace(C22)/2;
    s1 = sqrt(v1); s2 = sqrt(v2);
    den = v1 + v2 - 2*c.rho*s1*s2;
    if den > 0
        w = (v2 - c.rho*s1*s2)/den;
        Eo = w*E1 + (1-w)*E2;
        c.opt_rms = sqrt(mean(sum(Eo.^2,2)));
    end
end

function nees = nees_of(snaps, truthTrajs, n_frames, dt)
    vals = [];
    for ac = 1:numel(truthTrajs)
        tt = truthTrajs{ac};
        for k = 1:n_frames
            tnow = (k-1)*dt;
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
            tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            tl = snaps{k}.trackList;
            for t = 1:numel(tl)
                s = tl{t};
                if isfield(s,'truth_idx') && s.truth_idx==ac && s.type~=7 ...
                   && ~isnan(s.lat) && isfield(s,'ukf')
                    [ex,ey] = err_km(s.lon,s.lat,tlon,tlat);
                    P = pos_cov_km(s, tlat);
                    v = [ex;ey]' * (P \ [ex;ey]);
                    if isfinite(v) && v >= 0, vals(end+1) = v; end %#ok<AGROW>
                    break;
                end
            end
        end
    end
    if isempty(vals), nees = NaN; else, nees = mean(vals); end
end

function C = pos_cov_km(s, lat)
    P = s.ukf.P;
    sx = 111.32*cosd(lat); sy = 110.57;
    J = diag([sx, sy]);
    C = J * P([1 3],[1 3]) * J';
    C = (C+C')/2;
    if rcond(C) < 1e-12, C = C + 1e-6*eye(2); end
end

function [ex,ey] = err_km(lon,lat,tlon,tlat)
    R = 6371;
    ey = deg2rad(lat-tlat)*R;
    ex = deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end

function s = track_length_stats(snaps, label, nf)
    ids = []; is_real = [];
    life_cnt = containers.Map('KeyType','double','ValueType','double');
    real_flag = containers.Map('KeyType','double','ValueType','logical');
    for k = 1:nf
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            tr = tl{t};
            if tr.type == 7, continue; end
            id = tr.id;
            if isKey(life_cnt, id)
                life_cnt(id) = life_cnt(id) + 1;
            else
                life_cnt(id) = 1;
                ids(end+1) = id; %#ok<AGROW>
                ti = NaN;
                if isfield(tr, 'truth_idx'), ti = tr.truth_idx; end
                real_flag(id) = ~isnan(ti) && ti > 0;
            end
        end
    end
    len = zeros(1, numel(ids));
    rf  = false(1, numel(ids));
    for i = 1:numel(ids)
        len(i) = life_cnt(ids(i));
        rf(i)  = real_flag(ids(i));
    end
    s = struct();
    s.label = label;
    s.n_total = numel(ids);
    s.n_real  = sum(rf);
    s.n_false = sum(~rf);
    if s.n_real > 0
        s.real_mean   = mean(len(rf));
        s.real_median = median(len(rf));
        s.real_p10    = prctile(len(rf), 10);
        s.real_p90    = prctile(len(rf), 90);
    else
        s.real_mean = NaN; s.real_median = NaN; s.real_p10 = NaN; s.real_p90 = NaN;
    end
    if s.n_false > 0
        s.false_mean = mean(len(~rf));
    else
        s.false_mean = NaN;
    end
    s.false_rate = s.n_false / max(s.n_total, 1);
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

function d = hav(lon1, lat1, lon2, lat2)
    R = 6371;
    p1 = deg2rad(lat1); p2 = deg2rad(lat2);
    dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
    a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
    d = 2*R*asin(min(1, sqrt(a)));
end
