function metrics = evaluate_paper_metrics(result)
%EVALUATE_PAPER_METRICS 论文六项考核指标一次输出
%
%   metrics = evaluate_paper_metrics(result)
%
%   输入 result 为 run(scenario) 的返回值。
%   输出 6 项指标：
%     1. RMSE（单站 + 各融合方法）
%     2. 航迹平均长度（真实 / 虚假分开）
%     3. 正确匹配概率（跨站配对）
%     4. 虚假航迹率
%     5. 融合覆盖率（both / 全部融合帧）
%     6. 共同帧 RMSE（公平对比口径：两站与融合都有估计的帧）
%
%   设计说明：
%     - 真实航迹：truth_idx 有效且 > 0；虚假航迹：无有效 truth_idx 且 type~=7
%     - type==7 为 HISTORY（已终止）航迹，所有统计均排除
%     - 共同帧口径用于排除"融合包含单站透传帧"造成的不公平比较
%
%   2026-10-25

    metrics = struct();
    metrics.scenario = result.scenario.name;
    metrics.n_targets = result.scenario.n_targets;
    metrics.n_frames = result.scenario.n_frames;

    dt = result.params.dt_sec;
    nf = result.scenario.n_frames;

    %% ---------- 指标 1：RMSE ----------
    m1 = struct();
    m1.R1_rms = result.errorStats_R1.overall.ukf.rms;
    m1.R1_n   = result.errorStats_R1.overall.ukf.n;
    m1.R2_rms = result.errorStats_R2.overall.ukf.rms;
    m1.R2_n   = result.errorStats_R2.overall.ukf.n;
    fo = result.fusion_eval.overall;
    m1.fusion = struct('method', {}, 'rms', {}, 'n', {}, 'median', {});
    for i = 1:numel(fo)
        m1.fusion(i).method = fo(i).method;
        m1.fusion(i).rms    = fo(i).s.rms;
        m1.fusion(i).n      = fo(i).s.n;
        m1.fusion(i).median = fo(i).s.median;
    end
    metrics.rmse = m1;

    %% ---------- 指标 2 & 4：航迹长度 / 虚假航迹率 ----------
    m2 = track_length_stats(result.trackSnapshots_R1, 'R1', nf);
    m2_R2 = track_length_stats(result.aligned_R2, 'R2', nf);
    metrics.track_length = m2;
    metrics.track_length_R2 = m2_R2;

    %% ---------- 指标 3：正确匹配概率 ----------
    m3 = struct('n_pairs', 0, 'n_correct', 0, 'n_wrong', 0, 'n_missed', 0, ...
                'match_rate', NaN, 'wrong_rate', NaN);
    pairs = result.matched_pairs;
    if ~isempty(pairs)
        n_correct = 0; n_wrong = 0;
        covered = false(1, result.scenario.n_targets);
        for p = 1:numel(pairs)
            t1 = truth_idx_of(result.trackSnapshots_R1, pairs(p).R1_track_id);
            t2 = truth_idx_of(result.aligned_R2, pairs(p).R2_track_id);
            if ~isnan(t1) && ~isnan(t2) && t1 == t2 && t1 > 0
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
        m3.match_rate = n_correct / max(result.scenario.n_targets, 1);
        m3.wrong_rate = n_wrong / max(numel(pairs), 1);
    end
    metrics.matching = m3;

    %% ---------- 指标 5 & 6：融合覆盖率 + 共同帧 RMSE ----------
    [m5, m6] = fusion_coverage_and_common_frames(result, dt, nf);
    metrics.coverage = m5;
    metrics.common_frame = m6;

    %% ---------- 指标 7：一致性诊断（NEES / 互相关 / 理论界）----------
    % 这一组是"性能极限"分析的原料：
    %   NEES      — 报告协方差是否可信（2 自由度，一致时 ≈2）
    %   rho       — 两站估计误差的相关系数（决定融合增益上界）
    %   indep_rms — 独立假设下的理论融合 RMSE
    %   opt_rms   — 给定 rho 与真实方差的最优线性融合 RMSE（可达上界）
    metrics.consistency = consistency_analysis(result, dt, nf);

    %% ---------- 打印 ----------
    fprintf('\n========== PAPER METRICS: %s ==========\n', metrics.scenario);
    fprintf('targets=%d  frames=%d\n', metrics.n_targets, metrics.n_frames);
    fprintf('\n[1] RMSE (km)\n');
    fprintf('  R1_only   n=%5d  rms=%7.3f\n', m1.R1_n, m1.R1_rms);
    fprintf('  R2_only   n=%5d  rms=%7.3f\n', m1.R2_n, m1.R2_rms);
    for i = 1:numel(m1.fusion)
        fprintf('  %-9s n=%5d  rms=%7.3f\n', m1.fusion(i).method, m1.fusion(i).n, m1.fusion(i).rms);
    end
    fprintf('\n[2] TRACK LENGTH (frames)\n');
    fprintf('  R1: total=%d  real=%d (mean=%.1f med=%.1f)  false=%d (mean=%.1f)\n', ...
        m2.n_total, m2.n_real, m2.real_mean, m2.real_median, m2.n_false, m2.false_mean);
    fprintf('  R2: total=%d  real=%d (mean=%.1f med=%.1f)  false=%d (mean=%.1f)\n', ...
        m2_R2.n_total, m2_R2.n_real, m2_R2.real_mean, m2_R2.real_median, m2_R2.n_false, m2_R2.false_mean);
    fprintf('\n[3] MATCHING\n');
    fprintf('  pairs=%d  correct=%d  wrong=%d  missed=%d  match_rate=%.1f%%  wrong_rate=%.1f%%\n', ...
        m3.n_pairs, m3.n_correct, m3.n_wrong, m3.n_missed, 100*m3.match_rate, 100*m3.wrong_rate);
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
    fprintf('  NEES: R1=%.2f (n=%d)  R2=%.2f (n=%d)   [consistent ~2]\n', ...
        c.nees_R1, c.nees_R1_n, c.nees_R2, c.nees_R2_n);
    fprintf('  rho(R1,R2)=%+.3f\n', c.rho);
    fprintf('  theoretical: indep=%.3f  optimal(rho)=%.3f  actual=%.3f\n', ...
        c.indep_rms, c.opt_rms, m6.fused);
    fprintf('  gap to optimal: %.1f%%\n', 100*(m6.fused-c.opt_rms)/c.opt_rms);
    fprintf('====================================================\n');
end

% =====================================================================
function c = consistency_analysis(result, dt, nf)
    % 计算 NEES、两站互相关、理论融合界
    c = struct('nees_R1',NaN,'nees_R1_n',0,'nees_R2',NaN,'nees_R2_n',0, ...
               'rho',NaN,'indep_rms',NaN,'opt_rms',NaN,'n_common',0);

    % ---- NEES（单站，对真值）----
    [c.nees_R1, c.nees_R1_n] = nees_of(result, result.trackSnapshots_R1, dt, nf);
    [c.nees_R2, c.nees_R2_n] = nees_of(result, result.aligned_R2, dt, nf);

    % ---- 共同帧误差向量 ----
    [E1, E2, ~] = common_frame_errors(result, dt, nf);
    n = size(E1,1);
    if n < 10, return; end
    c.n_common = n;

    C11 = (E1'*E1)/(n-1);
    C22 = (E2'*E2)/(n-1);
    C12 = (E1'*E2)/(n-1);
    c.rho = trace(C12)/sqrt(trace(C11)*trace(C22));

    rms1 = sqrt(trace(C11));   % 注意：trace/2 是每轴方差，rms = sqrt(2*v) 见下
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

% =====================================================================
function [nees, n] = nees_of(result, snaps, dt, nf)
    vals = [];
    for ac = 1:numel(result.truthTrajs)
        tt = result.truthTrajs{ac};
        for k = 1:nf
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
    n = numel(vals);
    if n == 0, nees = NaN; else, nees = mean(vals); end
end

% =====================================================================
function [E1, E2, EF] = common_frame_errors(result, dt, nf)
    E1 = zeros(0,2); E2 = zeros(0,2); EF = zeros(0,1);
    fs = result.all_fused_snapshots;
    if isempty(fs), return; end
    mi = find(strcmp(result.method_names,'SCC'),1); if isempty(mi), mi=1; end
    if ~isfield(result.fusion_eval,'pair_to_aircraft'), return; end
    p2a = result.fusion_eval.pair_to_aircraft;
    for p = 1:numel(result.matched_pairs)
        if p > numel(p2a), break; end
        ac = p2a(p);
        if ac < 1 || ac > numel(result.truthTrajs), continue; end
        tt = result.truthTrajs{ac};
        id1 = result.matched_pairs(p).R1_track_id;
        id2 = result.matched_pairs(p).R2_track_id;
        for k = 1:nf
            tnow = (k-1)*dt;
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
            tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            s1 = find_trk(result.trackSnapshots_R1{k}, id1);
            s2 = find_trk(result.aligned_R2{k}, id2);
            sf = [];
            if ~isempty(fs{p,mi})
                tl = fs{p,mi}{k}.trackList;
                if ~isempty(tl), sf = tl{1}; end
            end
            if isempty(s1)||isempty(s2)||isempty(sf), continue; end
            if isfield(sf,'source') && ~strcmp(sf.source,'both'), continue; end
            [a,b] = err_km(s1.lon,s1.lat,tlon,tlat);
            [c_,d] = err_km(s2.lon,s2.lat,tlon,tlat);
            [e_,f_] = err_km(sf.lon,sf.lat,tlon,tlat);
            E1(end+1,:) = [a,b]; %#ok<AGROW>
            E2(end+1,:) = [c_,d]; %#ok<AGROW>
            EF(end+1,1) = sqrt(e_^2+f_^2); %#ok<AGROW>
        end
    end
end

% =====================================================================
function C = pos_cov_km(s, lat)
    P = s.ukf.P;
    sx = 111.32*cosd(lat); sy = 110.57;
    J = diag([sx, sy]);
    C = J * P([1 3],[1 3]) * J';
    C = (C+C')/2;
    if rcond(C) < 1e-12, C = C + 1e-6*eye(2); end
end

% =====================================================================
function [ex,ey] = err_km(lon,lat,tlon,tlat)
    R = 6371;
    ey = deg2rad(lat-tlat)*R;
    ex = deg2rad(lon-tlon)*R*cosd((lat+tlat)/2);
end

% =====================================================================
function s = track_length_stats(snaps, label, nf)
    % 统计每条航迹在快照中出现的帧数（排除 type==7）
    ids = []; is_real = []; life_cnt = containers.Map('KeyType','double','ValueType','double');
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
                if isfield(tr,'truth_idx'), ti = tr.truth_idx; end
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

% =====================================================================
function ti = truth_idx_of(snaps, id)
    ti = NaN;
    for k = 1:numel(snaps)
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            if tl{t}.id == id
                if isfield(tl{t},'truth_idx'), ti = tl{t}.truth_idx; end
                return;
            end
        end
    end
end

% =====================================================================
function [m5, m6] = fusion_coverage_and_common_frames(result, dt, nf)
    m5 = struct('both',0,'r1_only',0,'r2_only',0,'coverage',NaN);
    m6 = struct('n',0,'R1',NaN,'R2',NaN,'fused',NaN);

    fs = result.all_fused_snapshots;
    if isempty(fs), return; end
    mi = find(strcmp(result.method_names, 'SCC'), 1);
    if isempty(mi), mi = 1; end

    % ---- 覆盖率 ----
    nb=0; n1=0; n2=0;
    for p = 1:size(fs,1)
        sn = fs{p, mi};
        for k = 1:numel(sn)
            tl = sn{k}.trackList;
            if isempty(tl), continue; end
            src = 'both';
            if isfield(tl{1},'source'), src = tl{1}.source; end
            if strcmp(src,'both'), nb=nb+1;
            elseif strcmp(src,'R1_only'), n1=n1+1;
            else, n2=n2+1; end
        end
    end
    m5.both=nb; m5.r1_only=n1; m5.r2_only=n2;
    m5.coverage = nb / max(nb+n1+n2, 1);

    % ---- 共同帧 RMSE ----
    pairs = result.matched_pairs;
    if isempty(pairs) || ~isfield(result.fusion_eval,'pair_to_aircraft'), return; end
    p2a = result.fusion_eval.pair_to_aircraft;

    E1=[]; E2=[]; EF=[];
    for p = 1:numel(pairs)
        if p > numel(p2a), break; end
        ac = p2a(p);
        if ac < 1 || ac > numel(result.truthTrajs), continue; end
        tt = result.truthTrajs{ac};
        id1 = pairs(p).R1_track_id; id2 = pairs(p).R2_track_id;
        for k = 1:nf
            tnow = (k-1)*dt;
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), continue; end
            tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            s1 = find_trk(result.trackSnapshots_R1{k}, id1);
            s2 = find_trk(result.aligned_R2{k}, id2);
            sf = [];
            if ~isempty(fs{p,mi})
                tl = fs{p,mi}{k}.trackList;
                if ~isempty(tl), sf = tl{1}; end
            end
            if isempty(s1) || isempty(s2) || isempty(sf), continue; end
            if isfield(sf,'source') && ~strcmp(sf.source,'both'), continue; end
            E1(end+1) = hav(s1.lon,s1.lat,tlon,tlat); %#ok<AGROW>
            E2(end+1) = hav(s2.lon,s2.lat,tlon,tlat); %#ok<AGROW>
            EF(end+1) = hav(sf.lon,sf.lat,tlon,tlat); %#ok<AGROW>
        end
    end
    if ~isempty(E1)
        m6.n = numel(E1);
        m6.R1 = sqrt(mean(E1.^2));
        m6.R2 = sqrt(mean(E2.^2));
        m6.fused = sqrt(mean(EF.^2));
    end
end

% =====================================================================
function s = find_trk(snap, id)
    s = [];
    tl = snap.trackList;
    for t = 1:numel(tl)
        if tl{t}.id == id && tl{t}.type ~= 7 && ~isnan(tl{t}.lat)
            s = tl{t}; return;
        end
    end
end

% =====================================================================
function d = hav(lon1, lat1, lon2, lat2)
    R = 6371;
    p1 = deg2rad(lat1); p2 = deg2rad(lat2);
    dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
    a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
    d = 2*R*asin(sqrt(a));
end
