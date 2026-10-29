function T = report_stage_rmse(varargin)
%REPORT_STAGE_RMSE 逐阶段误差汇总：原始量测 → 单站跟踪 → 融合
%
%   T = report_stage_rmse()
%   T = report_stage_rmse('scenario','adsb_ns_fr24','outdir',p)
%
%   口径全部复用既有评估模块，不另起炉灶：
%     原始量测 / 校准量测 / 单站跟踪  → evaluate_all_multi('tracking_errors')
%     融合（多算法）                  → evaluate_online_fusion
%     共同帧公平对比                  → evaluate_paper_metrics_online 的指标 6
%
%   误差 = 估计位置与真值位置的球面距离（km），RMSE 按帧-目标样本汇总。

    opts = struct('scenario', 'adsb_ns_fr24', 'params', [], ...
        'outdir', fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'radarview_export'));
    if ~isempty(varargin) && isstruct(varargin{1}) && isfield(varargin{1}, 'scenario')
        f = fieldnames(varargin{1});
        for i = 1:numel(f), opts.(f{i}) = varargin{1}.(f{i}); end
    end

    result = run(opts.scenario, opts.params, struct('verbose', false));

    e1 = result.errorStats_R1.overall;
    e2 = result.errorStats_R2.overall;

    add = @(nm, s) struct('name', nm, 'n', s.n, 'rms', s.rms, ...
                          'median', s.median, 'p95', s.pct95);

    rows = { ...
        add('R1 原始量测（未跟踪）', e1.raw), ...
        add('R1 校准后量测', e1.det), ...
        add('R1 单站跟踪（IMM-UKF）', e1.ukf), ...
        add('R2 原始量测（未跟踪）', e2.raw), ...
        add('R2 校准后量测', e2.det), ...
        add('R2 单站跟踪（IMM-UKF）', e2.ukf)};

    fo = result.fusion_eval.overall;
    for i = 1:numel(fo)
        rows{end+1} = add(sprintf('融合 %s（系统航迹）', fo(i).method), fo(i).s); %#ok<AGROW>
    end

    fprintf('\n%-28s %6s %10s %10s %10s\n', '阶段', 'n', 'RMSE/km', '中位/km', 'P95/km');
    fprintf('%s\n', repmat('-', 1, 68));
    T = struct('stage', {}, 'n', {}, 'rms_km', {}, 'median_km', {}, 'p95_km', {});
    for i = 1:numel(rows)
        r = rows{i};
        fprintf('%-28s %6d %10.3f %10.3f %10.3f\n', r.name, r.n, r.rms, r.median, r.p95);
        T(end+1) = struct('stage', r.name, 'n', r.n, 'rms_km', r.rms, ...
                          'median_km', r.median, 'p95_km', r.p95); %#ok<AGROW>
    end

    % ---- 单站跟踪误差拆分：有量测帧 vs 纯外推帧 ----
    % 原始量测只存在于"检测到"的帧（Pd=0.6，约六成），跟踪输出却覆盖航迹存活的
    % 全部帧，其中四成是纯预测。两者分母不同，直接比会把"跟踪"看成比"量测"还差。
    sp1 = split_track_errors(result.trackSnapshots_R1, result.detList_R1, ...
                             result.truthTrajs, result.t1_grid);
    sp2 = split_track_errors(result.trackSnapshots_R2, result.detList_R2, ...
                             result.truthTrajs, result.t2_grid);
    fprintf('\n[单站跟踪误差拆分]\n');
    fprintf('  R1  有量测帧 n=%4d RMSE=%6.3f（同帧量测本身 %6.3f）| 纯外推帧 n=%4d RMSE=%6.3f\n', ...
        sp1.n_upd, sp1.rms_upd, sp1.rms_pair_det, sp1.n_prd, sp1.rms_prd);
    fprintf('  R2  有量测帧 n=%4d RMSE=%6.3f（同帧量测本身 %6.3f）| 纯外推帧 n=%4d RMSE=%6.3f\n', ...
        sp2.n_upd, sp2.rms_upd, sp2.rms_pair_det, sp2.n_prd, sp2.rms_prd);

    % ---- 共同帧公平对比（三者在同一批帧上比）----
    cf = result.metrics.common_frame;
    fprintf('\n[共同帧公平对比：三者都在同一批帧上]\n');
    fn = fieldnames(cf);
    for i = 1:numel(fn)
        v = cf.(fn{i});
        if isscalar(v)
            fprintf('  %-12s %.3f\n', fn{i}, v);
        end
    end

    % ---- 落盘 ----
    if ~exist(opts.outdir, 'dir'), mkdir(opts.outdir); end
    fid = fopen(fullfile(opts.outdir, 'stage_rmse.csv'), 'w');
    fprintf(fid, 'stage,n,rms_km,median_km,p95_km\n');
    for i = 1:numel(T)
        fprintf(fid, '%s,%d,%.4f,%.4f,%.4f\n', T(i).stage, T(i).n, ...
            T(i).rms_km, T(i).median_km, T(i).p95_km);
    end
    fprintf(fid, 'R1 跟踪(仅有量测帧),%d,%.4f,,\n', sp1.n_upd, sp1.rms_upd);
    fprintf(fid, 'R1 跟踪(纯外推帧),%d,%.4f,,\n', sp1.n_prd, sp1.rms_prd);
    fprintf(fid, 'R2 跟踪(仅有量测帧),%d,%.4f,,\n', sp2.n_upd, sp2.rms_upd);
    fprintf(fid, 'R2 跟踪(纯外推帧),%d,%.4f,,\n', sp2.n_prd, sp2.rms_prd);
    fprintf(fid, '共同帧 R1,%d,%.4f,,\n', cf.n, cf.R1);
    fprintf(fid, '共同帧 R2,%d,%.4f,,\n', cf.n, cf.R2);
    fprintf(fid, '共同帧 融合,%d,%.4f,,\n', cf.n, cf.fused);
    fclose(fid);
end

% =====================================================================
function s = split_track_errors(snaps, dets, truthTrajs, times)
%SPLIT_TRACK_ERRORS 把单站跟踪误差按"本帧有没有量测"拆成两组
%   同一帧同一真值若有多条航迹，取误差最小的那条（与既有评估口径一致）。
    e_upd = [];  e_prd = [];  e_det = [];
    for k = 1:numel(snaps)
        sk = snaps{k};
        if ~isstruct(sk) || ~isfield(sk, 'trackList'), continue; end
        tk = times(k);
        tids = [];  errs = [];  tlo = [];  tla = [];
        for i = 1:numel(sk.trackList)
            tr = sk.trackList{i};
            t = double(tr.truth_idx);
            if ~isfinite(t) || t < 1 || t > numel(truthTrajs), continue; end
            tt = truthTrajs{t};
            if tk < tt.time_sec(1) || tk > tt.time_sec(end), continue; end
            la = interp1(tt.time_sec, tt.lat, tk, 'linear');
            lo = interp1(tt.time_sec, tt.lon, tk, 'linear');
            if isnan(la) || isnan(lo), continue; end
            d = haversine_km(tr.lon, tr.lat, lo, la);
            j = find(tids == t, 1);
            if isempty(j)
                tids(end+1) = t;  errs(end+1) = d; %#ok<AGROW>
                tlo(end+1) = lo;  tla(end+1) = la; %#ok<AGROW>
            elseif d < errs(j)
                errs(j) = d;
            end
        end
        dk = dets{k};
        for j = 1:numel(tids)
            [di, has] = find_detection(dk, tids(j));
            if has
                e_upd(end+1) = errs(j); %#ok<AGROW>
                % 同帧、同目标：量测自身的误差，用来判断滤波器是否把量测吃进去了
                e_det(end+1) = haversine_km(dk(di).lon, dk(di).lat, tlo(j), tla(j)); %#ok<AGROW>
            else
                e_prd(end+1) = errs(j); %#ok<AGROW>
            end
        end
    end
    s = struct('n_upd', numel(e_upd), 'rms_upd', rms_of(e_upd), ...
               'n_prd', numel(e_prd), 'rms_prd', rms_of(e_prd), ...
               'n_pair', numel(e_det), 'rms_pair_det', rms_of(e_det));
end

function [idx, ok] = find_detection(d, t)
    idx = 0;  ok = false;
    if isempty(d), return; end
    for i = 1:numel(d)
        if ~d(i).is_clutter && double(d(i).aircraft_id) == t
            idx = i;  ok = true;  return;
        end
    end
end

function v = rms_of(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function d = haversine_km(lon1, lat1, lon2, lat2)
    R = 6371.0;
    p1 = lat1 * pi / 180;  p2 = lat2 * pi / 180;
    dp = (lat2 - lat1) * pi / 180;  dl = (lon2 - lon1) * pi / 180;
    a = sin(dp/2).^2 + cos(p1) .* cos(p2) .* sin(dl/2).^2;
    d = 2 * R * asin(sqrt(min(max(a, 0), 1)));
end
