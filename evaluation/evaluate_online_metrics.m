function m = evaluate_online_metrics(ctSnap, snap1s, snap2s, truthAll, labels, t1_grid, params)
%EVALUATE_ONLINE_METRICS 在线配对与融合的指标评估
%
%   m = evaluate_online_metrics(ctSnap, snap1s, snap2s, truthAll, labels, t1_grid, params)
%
%   给出三类指标：
%     1) 航迹长度：系统航迹覆盖帧数 vs 单站基线（取两站较长者）
%        —— 覆盖互补布设下，系统航迹应当是"两站并集"，比值 > 1
%     2) 融合精度：在"双源帧"（真正发生融合的帧）上比较
%        系统航迹 / R1 单站 / R2 单站 的位置误差
%     3) 配对正确率：确认的 (R1 航迹, R2 航迹) 对中，真值一致的占比
%
%   长度口径说明：
%     单站基线 = 该真值在两站中"存活帧数"的较大者（帧集合去重）
%     系统航迹 = 该真值对应的系统航迹（合并后编号）"有源帧"的并集
%     两者都是"帧集合"口径，可直接相除。

    n_frames = numel(ctSnap);
    if nargin < 6 || isempty(labels), labels = {}; end

    % ================= 1. 合并别名（两条编号 -> 一条系统航迹）=================
    alias = [];
    for k = 1:n_frames
        CTk = ctSnap{k};
        for i = 1:numel(CTk)
            if isfield(CTk, 'merge_alias') && ~isempty(CTk(i).merge_alias)
                for a = 1:numel(CTk(i).merge_alias)
                    alias(end+1, :) = [CTk(i).merge_alias(a), CTk(i).id]; %#ok<AGROW>
                end
            end
        end
    end

    % ================= 2. 逐帧收集系统航迹与单站航迹 =================
    sys_frames = {};    % key: canonical id -> 帧号数组
    sys_truth  = [];
    sys_x      = {};    % key: canonical id -> {frame, lon, lat}
    % 配对表用并行数组维护（本机 MATLAB 对 containers.Map 大键集不稳定）
    p_src1 = []; p_src2 = []; p_t1 = []; p_t2 = []; p_ok = false(0,1); p_n = zeros(0,1);
    n_dual = 0;
    err_fused = [];  err_r1 = [];  err_r2 = [];
    d2_dual = [];    dis_dual = [];
    % 逐真值收集两站误差向量，用于在线估计互相关 rho
    e1_by_truth = {};  e2_by_truth = {};
    err_sys_all = [];  err_r1_all = [];  err_r2_all = [];

    for k = 1:n_frames
        CTk = ctSnap{k};
        for i = 1:numel(CTk)
            cid = canon_id(CTk(i).id, alias);
            if CTk(i).n_src > 0
                key = sprintf('c%d', cid);
                if ~isfield(sys_frames, key) || isempty(sys_frames.(key))
                    sys_frames.(key) = [];
                    sys_x.(key) = struct('frame', {}, 'lon', {}, 'lat', {});
                    sys_truth(cid) = CTk(i).truth_idx;
                end
                sys_frames.(key)(end+1) = k;
                x = CTk(i).x;
                sys_x.(key)(end+1) = struct('frame', k, 'lon', x(1), 'lat', x(3));
            end

            % 配对事件
            if isfield(CTk, 'pair_log') && ~isempty(CTk(i).pair_log)
                for p = 1:numel(CTk(i).pair_log)
                    ev = CTk(i).pair_log{p};
                    hit = find(p_src1 == ev.src1_id & p_src2 == ev.src2_id, 1);
                    if isempty(hit)
                        p_src1(end+1) = ev.src1_id;  p_src2(end+1) = ev.src2_id; %#ok<AGROW>
                        p_t1(end+1) = ev.truth1;      p_t2(end+1) = ev.truth2; %#ok<AGROW>
                        p_ok(end+1) = ev.ok;          p_n(end+1) = 0; %#ok<AGROW>
                        hit = numel(p_src1);
                    end
                    p_n(hit) = p_n(hit) + 1;
                end
            end
        end

        % 精度：双源帧上比较三方的位置误差
        for i = 1:numel(CTk)
            if CTk(i).n_src ~= 2, continue; end
            tid = CTk(i).truth_idx;
            if ~isfinite(tid), continue; end
            t1 = find_by_id(snap1s{k}, CTk(i).src1_id);
            t2 = find_by_id(snap2s{k}, CTk(i).src2_id);
            if isempty(t1) || isempty(t2), continue; end
            [tl, ta] = truth_at(truthAll, tid, t1_grid(k));
            if isnan(tl), continue; end
            x = CTk(i).x;
            err_fused(end+1) = hav(x(1), x(3), tl, ta); %#ok<AGROW>
            err_r1(end+1)    = hav(t1.ukf.x(1), t1.ukf.x(3), tl, ta); %#ok<AGROW>
            err_r2(end+1)    = hav(t2.ukf.x(1), t2.ukf.x(3), tl, ta); %#ok<AGROW>
            % 两站估计的一致性诊断：马氏距离平方（同源时应服从 chi2(2)）
            [d2v, disv] = pair_disagreement(t1, t2);
            if isfinite(d2v), d2_dual(end+1) = d2v; end %#ok<AGROW>
            if isfinite(disv), dis_dual(end+1) = disv; end %#ok<AGROW>
            % 两站误差向量（km），用于估计 rho
            lat_mid = 0.5 * (t1.ukf.x(3) + t2.ukf.x(3));
            S = diag([111.32 * cosd(lat_mid), 110.57]);
            ev1 = S * [t1.ukf.x(1) - tl; t1.ukf.x(3) - ta];
            ev2 = S * [t2.ukf.x(1) - tl; t2.ukf.x(3) - ta];
            if ~isfield(e1_by_truth, sprintf('t%d', tid)), e1_by_truth.(sprintf('t%d', tid)) = []; end
            if ~isfield(e2_by_truth, sprintf('t%d', tid)), e2_by_truth.(sprintf('t%d', tid)) = []; end
            e1_by_truth.(sprintf('t%d', tid))(end+1, :) = ev1';
            e2_by_truth.(sprintf('t%d', tid))(end+1, :) = ev2';
            n_dual = n_dual + 1;
        end
    end

    % ================= 3. 单站长度 =================
    [r1_frames, r2_frames] = local_frames_all(snap1s, snap2s, n_frames);

    % ================= 4. 逐真值汇总 =================
    truths = unique([r1_frames.truths; r2_frames.truths]);
    truths = truths(~isnan(truths));
    per = struct('truth', {}, 'label', {}, 'r1_len', {}, 'r2_len', {}, 'single', {}, ...
        'ct_len', {}, 'ratio', {}, 'birth_k', {}, 'confirm_k', {}, 'delay', {}, 'n_dual', {});
    for ti = 1:numel(truths)
        tid = truths(ti);
        f1 = frames_of(r1_frames, tid);
        f2 = frames_of(r2_frames, tid);
        single = max(numel(f1), numel(f2));

        % 系统航迹：该真值对应的所有系统航迹（有源帧）的并集
        sf = [];
        bidx = NaN; cidx = NaN; didx = NaN;
        ids = fieldnames(sys_frames);
        for ii = 1:numel(ids)
            cid = str2double(ids{ii}(2:end));
            if ~isfinite(sys_truth(cid)) || sys_truth(cid) ~= tid, continue; end
            sf = [sf, sys_frames.(ids{ii})]; %#ok<AGROW>
            if isempty(bidx), bidx = first_frame(sys_frames.(ids{ii})); end
        end
        sf = unique(sf);
        e = struct('truth', tid, 'label', label_of(tid, labels), ...
            'r1_len', numel(f1), 'r2_len', numel(f2), 'single', single, ...
            'ct_len', numel(sf), 'ratio', NaN, 'birth_k', bidx, ...
            'confirm_k', NaN, 'delay', NaN, 'n_dual', 0);
        if single > 0, e.ratio = numel(sf) / single; end
        per(end+1) = e; %#ok<AGROW>
    end

    % 配对统计
    n_pair = numel(p_src1);
    n_ok = sum(p_ok);
    wrong = struct('src1', {}, 'src2', {}, 'truth1', {}, 'truth2', {});
    for i = 1:n_pair
        if ~p_ok(i)
            wrong(end+1) = struct('src1', p_src1(i), 'src2', p_src2(i), ...
                'truth1', p_t1(i), 'truth2', p_t2(i)); %#ok<AGROW>
        end
    end
    n_truth_dual = numel(unique(p_t1(p_ok)));

    % ================= 5. 组装输出 =================
    m = struct();
    m.per_truth = per;
    m.n_frames = n_frames;
    if ~isempty(per)
        ct_len = [per.ct_len];  sg = [per.single];  rt = [per.ratio];
        ok = sg > 0 & ct_len > 0;
        m.length_mean_ct = mean(ct_len(ok));
        m.length_mean_single = mean(sg(ok));
        m.length_ratio_mean = mean(rt(ok));
        m.length_ratio_median = median(rt(ok));
        m.length_ratio_max = max(rt(ok));
        m.length_ratio_min = min(rt(ok));
        m.n_longer = sum(rt(ok) > 1);
        m.n_shorter = sum(rt(ok) < 1);
        m.n_targets = numel(per);
    end
    m.n_dual_frames = n_dual;
    m.d2_dual_mean = mean_or_nan(d2_dual);
    m.dis_dual_mean_km = mean_or_nan(dis_dual);
    m.dis_dual_max_km = max_or_nan(dis_dual);
    % 逐真值估计 rho，再取中位（对目标数不敏感）
    rho_list = [];
    rkeys = fieldnames(e1_by_truth);
    for i = 1:numel(rkeys)
        a = e1_by_truth.(rkeys{i});
        b = e2_by_truth.(rkeys{i});
        if size(a, 1) < 5, continue; end
        A = a - mean(a, 1);
        B = b - mean(b, 1);
        num = trace(A' * B) / (size(A,1) - 1);
        den = sqrt(trace(A' * A) / (size(A,1) - 1) * trace(B' * B) / (size(A,1) - 1));
        if den > 0, rho_list(end+1) = num / den; end %#ok<AGROW>
    end
    m.rho_dual_mean = mean_or_nan(rho_list);
    m.rho_dual_median = median_or_nan(rho_list);
    m.n_rho_targets = numel(rho_list);
    m.rmse_fused_dual = rmse_of(err_fused);
    m.rmse_r1_dual = rmse_of(err_r1);
    m.rmse_r2_dual = rmse_of(err_r2);
    if ~isempty(err_r1) && ~isempty(err_r2)
        best_dual = min(m.rmse_r1_dual, m.rmse_r2_dual);
        if best_dual > 0
            m.gain_dual_pct = (best_dual - m.rmse_fused_dual) / best_dual * 100;
        else
            m.gain_dual_pct = NaN;
        end
    end
    m.n_pair = n_pair;
    m.n_pair_correct = n_ok;
    m.pair_precision = n_pair_safe(n_ok, n_pair);
    m.pair_recall = n_pair_safe(n_ok, n_truth_dual);
    m.wrong_pairs = wrong;
    m.sys_frames = sys_frames;
    m.sys_x = sys_x;
    m.sys_truth = sys_truth;
end

% =====================================================================
function c = canon_id(id, alias)
%CANON_ID 把被合并掉的编号映射到保留编号
    c = id;
    if isempty(alias), return; end
    for it = 1:5
        hit = alias(alias(:,1) == c, 2);
        if isempty(hit), break; end
        c = hit(1);
    end
end

function v = first_frame(fr)
    if isempty(fr), v = NaN; else, v = min(fr); end
end

function [tf, ta] = truth_at(truthAll, tid, t)
    tf = NaN; ta = NaN;
    if tid < 1 || tid > numel(truthAll), return; end
    tt = truthAll{tid};
    tf = interp1(tt(:,5), tt(:,1), t, 'linear', 'extrap');
    ta = interp1(tt(:,5), tt(:,2), t, 'linear', 'extrap');
end

function trk = find_by_id(snap, id)
    trk = [];
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for i = 1:numel(snap.trackList)
        if snap.trackList{i}.id == id
            trk = snap.trackList{i};
            return;
        end
    end
end

function ids = frame_ids_of(snap)
    ids = [];
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for i = 1:numel(snap.trackList)
        trk = snap.trackList{i};
        if ~isfield(trk, 'type') || trk.type == 6 || trk.type == 7, continue; end
        if ~isfield(trk, 'truth_idx') || ~isfinite(double(trk.truth_idx)), continue; end
        ids(end+1) = double(trk.truth_idx); %#ok<AGROW>
    end
end

function [r1, r2] = local_frames_all(snap1s, snap2s, n_frames)
    % 逐字段赋值：struct(...,'frames',{}) 会得到 0x0 结构体数组
    r1 = struct();  r1.truths = [];  r1.frames = struct();
    r2 = struct();  r2.truths = [];  r2.frames = struct();
    for k = 1:n_frames
        t1 = frame_ids_of(snap1s{k});
        t2 = frame_ids_of(snap2s{k});
        for i = 1:numel(t1)
            key = sprintf('t%d', t1(i));
            if ~isfield(r1.frames, key), r1.frames.(key) = []; end
            r1.frames.(key)(end+1) = k;
        end
        for i = 1:numel(t2)
            key = sprintf('t%d', t2(i));
            if ~isfield(r2.frames, key), r2.frames.(key) = []; end
            r2.frames.(key)(end+1) = k;
        end
    end
    r1.truths = cellfun(@(s) str2double(s(2:end)), fieldnames(r1.frames));
    r2.truths = cellfun(@(s) str2double(s(2:end)), fieldnames(r2.frames));
end

function f = frames_of(rec, tid)
    key = sprintf('t%d', tid);
    if ~isfield(rec.frames, key)
        f = [];
    else
        f = unique(rec.frames.(key));
    end
end

function v = mean_or_nan(a)
    if isempty(a), v = NaN; else, v = mean(a); end
end

function v = median_or_nan(a)
    if isempty(a), v = NaN; else, v = median(a); end
end

function v = max_or_nan(a)
    if isempty(a), v = NaN; else, v = max(a); end
end

function [d2, dis] = pair_disagreement(t1, t2)
%PAIR_DISAGREEMENT 两站估计的马氏距离平方与地表距离
    d2 = NaN;  dis = NaN;
    x1 = t1.ukf.x;  x2 = t2.ukf.x;
    P1 = t1.ukf.P;  P2 = t2.ukf.P;
    if any(~isfinite(x1(1:4))) || any(~isfinite(x2(1:4))), return; end
    dis = hav(x1(1), x1(3), x2(1), x2(3));
    lat_mid = 0.5 * (x1(3) + x2(3));
    S = diag([111.32 * cosd(lat_mid), 110.57]);
    dx = S * [x1(1) - x2(1); x1(3) - x2(3)];
    Pp = [P1(1,1) + P2(1,1), P1(1,3) + P2(1,3);
          P1(3,1) + P2(3,1), P1(3,3) + P2(3,3)];
    Pp = S * Pp * S';
    Pp = 0.5 * (Pp + Pp');
    if rcond(Pp) < 1e-12, Pp = Pp + 1e-6 * eye(2); end
    d2 = dx' * (Pp \ dx);
end

function v = rmse_of(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function v = n_pair_safe(a, b)
    if b == 0, v = NaN; else, v = a / b * 100; end
end

function s = label_of(tid, labels)
    s = sprintf('#%d', tid);
    if ~isempty(labels) && tid <= numel(labels)
        s = labels{tid};
    end
end

function d = hav(lon1, lat1, lon2, lat2)
    R = 6371.0088;
    p1 = deg2rad(lat1); p2 = deg2rad(lat2);
    dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
    a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
    d = 2*R*asin(min(1, sqrt(a)));
end
