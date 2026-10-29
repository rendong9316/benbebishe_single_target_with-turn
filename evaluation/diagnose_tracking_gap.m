function D = diagnose_tracking_gap(varargin)
%DIAGNOSE_TRACKING_GAP 模型集失配诊断：只改 IMM 转弯率，其余全不动
%
%   D = diagnose_tracking_gap()
%
%   对同一场景、同一 seed、同一 Pd 跑两组模型集，对比：
%     单站跟踪 RMSE、NEES（一致性）、IMM 各模型平均权重、共同帧融合增益
%   目的是回答"为什么跟踪比量测还差"，而不是调参。

    cfg = struct('scenario', 'adsb_ns_fr24', 'sets', []);
    if ~isempty(varargin) && isstruct(varargin{1})
        f = fieldnames(varargin{1});
        for i = 1:numel(f), cfg.(f{i}) = varargin{1}.(f{i}); end
    end
    if isempty(cfg.sets)
        cfg.sets = [0.7 1.5; 0.2 0.7; 0.05 0.2];
    end

    labels = {'CV', 'CT慢左', 'CT快左', 'CT快右', 'CT慢右'};
    D = struct('slow', {}, 'fast', {}, 'r1_rms', {}, 'r2_rms', {}, ...
               'r1_nees', {}, 'r2_nees', {}, 'mu', {}, 'fused_rms', {}, ...
               'gain_pct', {}, 'n_upd', {}, 'r1_rms_upd', {}, 'r1_rms_prd', {});

    for c = 1:size(cfg.sets, 1)
        p = params_coverage_online();
        p.imm_ct_omega_slow_deg = cfg.sets(c, 1);
        p.imm_ct_omega_fast_deg = cfg.sets(c, 2);
        fprintf('\n===== 模型集 %.2f / %.2f deg/s =====\n', p.imm_ct_omega_slow_deg, p.imm_ct_omega_fast_deg);
        r = run(cfg.scenario, p, struct('verbose', false));

        e1 = r.errorStats_R1.overall.ukf;
        e2 = r.errorStats_R2.overall.ukf;
        cs = r.metrics.consistency;
        cf = r.metrics.common_frame;

        % 有量测帧 / 纯外推帧拆分（只做 R1，够说明问题）
        sp = split_r1(r);

        % IMM 平均权重
        mu = mean_model_weights(r.trackSnapshots_R1, 5);

        fprintf('  R1 跟踪 RMSE %.3f km (n=%d) | 有量测 %.3f | 纯外推 %.3f\n', ...
            e1.rms, e1.n, sp.rms_upd, sp.rms_prd);
        fprintf('  R2 跟踪 RMSE %.3f km (n=%d)\n', e2.rms, e2.n);
        fprintf('  NEES  R1=%.2f  R2=%.2f  [应≈2]\n', cs.nees_R1, cs.nees_R2);
        fprintf('  平均权重: ');
        for m = 1:numel(labels), fprintf('%s=%.2f ', labels{m}, mu(m)); end
        fprintf('\n');
        fprintf('  共同帧 n=%d: R1 %.2f, R2 %.2f, 融合 %.2f, 增益 %.1f%%\n', ...
            cf.n, cf.R1, cf.R2, cf.fused, 100 * (1 - cf.fused / min(cf.R1, cf.R2)));

        D(end+1) = struct('slow', p.imm_ct_omega_slow_deg, 'fast', p.imm_ct_omega_fast_deg, ...
            'r1_rms', e1.rms, 'r2_rms', e2.rms, 'r1_nees', cs.nees_R1, 'r2_nees', cs.nees_R2, ...
            'mu', mu, 'fused_rms', cf.fused, 'gain_pct', 100 * (1 - cf.fused / min(cf.R1, cf.R2)), ...
            'n_upd', sp.n_upd, 'r1_rms_upd', sp.rms_upd, 'r1_rms_prd', sp.rms_prd); %#ok<AGROW>
    end

    fprintf('\n%-12s %8s %8s %8s %8s %8s %8s\n', '模型集', 'R1_all', 'R1_量测', 'R1_外推', 'R2_all', 'NEES_R1', '增益%');
    for i = 1:numel(D)
        fprintf('%.2f/%-6.2f %8.2f %8.2f %8.2f %8.2f %8.2f %7.1f\n', D(i).slow, D(i).fast, ...
            D(i).r1_rms, D(i).r1_rms_upd, D(i).r1_rms_prd, D(i).r2_rms, D(i).r1_nees, D(i).gain_pct);
    end
end

% ---------------------------------------------------------------------
function mu = mean_model_weights(snaps, M)
    acc = zeros(M, 1);  cnt = 0;
    for k = 1:numel(snaps)
        s = snaps{k};
        if ~isstruct(s) || ~isfield(s, 'trackList'), continue; end
        for i = 1:numel(s.trackList)
            tr = s.trackList{i};
            if ~isfield(tr, 'ukf') || ~isfield(tr.ukf, 'mu'), continue; end
            v = double(tr.ukf.mu(:));
            if numel(v) ~= M || any(~isfinite(v)), continue; end
            acc = acc + v;  cnt = cnt + 1;
        end
    end
    if cnt == 0, mu = nan(M, 1); else, mu = acc / cnt; end
end

% ---------------------------------------------------------------------
function s = split_r1(r)
    e_upd = [];  e_prd = [];
    for k = 1:r.n_frames
        sk = r.trackSnapshots_R1{k};
        if ~isstruct(sk) || ~isfield(sk, 'trackList'), continue; end
        tk = r.t1_grid(k);
        for i = 1:numel(sk.trackList)
            tr = sk.trackList{i};
            t = double(tr.truth_idx);
            if ~isfinite(t) || t < 1 || t > numel(r.truthTrajs), continue; end
            tt = r.truthTrajs{t};
            if tk < tt.time_sec(1) || tk > tt.time_sec(end), continue; end
            la = interp1(tt.time_sec, tt.lat, tk, 'linear');
            lo = interp1(tt.time_sec, tt.lon, tk, 'linear');
            if isnan(la) || isnan(lo), continue; end
            d = hav(tr.lon, tr.lat, lo, la);
            if has_det(r.detList_R1{k}, t), e_upd(end+1) = d; else, e_prd(end+1) = d; end %#ok<AGROW>
        end
    end
    s = struct('n_upd', numel(e_upd), 'rms_upd', rms_of(e_upd), 'rms_prd', rms_of(e_prd));
end

function ok = has_det(d, t)
    ok = false;
    if isempty(d), return; end
    for i = 1:numel(d)
        if ~d(i).is_clutter && double(d(i).aircraft_id) == t, ok = true; return; end
    end
end

function v = rms_of(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function d = hav(lon1, lat1, lon2, lat2)
    R = 6371.0;
    p1 = lat1 * pi / 180;  p2 = lat2 * pi / 180;
    dp = (lat2 - lat1) * pi / 180;  dl = (lon2 - lon1) * pi / 180;
    a = sin(dp/2).^2 + cos(p1) .* cos(p2) .* sin(dl/2).^2;
    d = 2 * R * asin(sqrt(min(max(a, 0), 1)));
end
