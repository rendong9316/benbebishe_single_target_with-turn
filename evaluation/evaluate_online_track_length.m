function res = evaluate_online_track_length(ctSnap, snap1s, snap2s, n_frames, labels)
%EVALUATE_ONLINE_TRACK_LENGTH 在线流程的航迹长度评估
%
%   res = evaluate_online_track_length(ctSnap, snap1s, snap2s, n_frames, labels)
%
%   对每条真值目标，比较：
%     ct_len    — 系统航迹覆盖帧数（当帧至少有一个源）
%     r1_len    — R1 单站航迹覆盖帧数（同目标取最长的那条）
%     r2_len    — R2 单站航迹覆盖帧数
%     single    — 单站基线，max(r1_len, r2_len)
%     ratio     — ct_len / single
%
%   以及系统航迹的建立帧与确认帧，用于分离"延长"和"确认延迟"两笔账。

    if nargin < 5, labels = {}; end

    % ---- 系统航迹：逐帧累计覆盖帧数 ----
    ct = struct('id', {}, 'truth_idx', {}, 'n', {}, 'birth_k', {}, ...
        'confirm_k', {}, 'first_cand_k', {});
    for k = 1:n_frames
        CTk = ctSnap{k};
        if isempty(CTk), continue; end
        for i = 1:numel(CTk)
            j = find_ct(ct, CTk(i).id);
            if isempty(j)
                ct(end+1) = struct('id', CTk(i).id, 'truth_idx', CTk(i).truth_idx, ...
                    'n', 0, 'birth_k', CTk(i).birth_k, 'confirm_k', NaN, ...
                    'first_cand_k', NaN); %#ok<AGROW>
                j = numel(ct);
            end
            if CTk(i).n_src > 0
                ct(j).n = ct(j).n + 1;
            end
            if isnan(ct(j).confirm_k) && ~isnan(CTk(i).confirm_k)
                ct(j).confirm_k = CTk(i).confirm_k;
            end
            if isnan(ct(j).first_cand_k) && isfield(CTk(i), 'first_cand_k') ...
                    && ~isnan(CTk(i).first_cand_k)
                ct(j).first_cand_k = CTk(i).first_cand_k;
            end
        end
    end

    % ---- 单站航迹：按 (真值, 航迹ID) 统计活跃帧数 ----
    r1 = track_lengths(snap1s, n_frames);
    r2 = track_lengths(snap2s, n_frames);

    % ---- 逐真值汇总 ----
    res = struct('truth', {}, 'label', {}, 'ct_id', {}, 'ct_len', {}, ...
        'r1_len', {}, 'r2_len', {}, 'single', {}, 'ratio', {}, ...
        'birth_k', {}, 'confirm_k', {}, 'delay', {});

    truths = unique([r1.truth, r2.truth]);
    truths = truths(~isnan(truths));
    for t = 1:numel(truths)
        tid = truths(t);
        e = struct('truth', tid, 'label', label_of(tid, labels), 'ct_id', NaN, ...
            'ct_len', 0, 'r1_len', max_or_zero([r1([r1.truth] == tid).n]), ...
            'r2_len', max_or_zero([r2([r2.truth] == tid).n]), ...
            'single', 0, 'ratio', NaN, 'birth_k', NaN, 'confirm_k', NaN, 'delay', NaN);
        e.single = max(e.r1_len, e.r2_len);

        best = 0; bestj = [];
        for j = 1:numel(ct)
            if ct(j).truth_idx == tid && ct(j).n > best
                best = ct(j).n;  bestj = j;
            end
        end
        if ~isempty(bestj)
            e.ct_id = ct(bestj).id;
            e.ct_len = ct(bestj).n;
            e.birth_k = ct(bestj).birth_k;
            e.confirm_k = ct(bestj).confirm_k;
            if ~isnan(e.confirm_k) && ~isnan(ct(bestj).first_cand_k)
                e.delay = e.confirm_k - ct(bestj).first_cand_k;
            elseif ~isnan(e.confirm_k)
                e.delay = e.confirm_k - e.birth_k;
            end
        end
        if e.single > 0
            e.ratio = e.ct_len / e.single;
        end
        res(end+1) = e; %#ok<AGROW>
    end
end

% ---------------------------------------------------------------------
function j = find_ct(ct, id)
    j = [];
    for i = 1:numel(ct)
        if ct(i).id == id, j = i; return; end
    end
end

function out = track_lengths(snaps, n_frames)
    out = struct('id', {}, 'truth', {}, 'n', {});
    for k = 1:n_frames
        s = snaps{k};
        if isempty(s) || ~isfield(s, 'trackList'), continue; end
        trks = s.trackList;
        for t = 1:numel(trks)
            trk = trks{t};
            if ~isfield(trk, 'type') || trk.type == 6 || trk.type == 7, continue; end
            tid = NaN;
            if isfield(trk, 'truth_idx') && isscalar(trk.truth_idx) && isfinite(double(trk.truth_idx))
                tid = double(trk.truth_idx);
            end
            if isnan(tid), continue; end
            j = [];
            for i = 1:numel(out)
                if out(i).id == trk.id, j = i; break; end
            end
            if isempty(j)
                out(end+1) = struct('id', trk.id, 'truth', tid, 'n', 0); %#ok<AGROW>
                j = numel(out);
            end
            out(j).n = out(j).n + 1;
        end
    end
end

function v = max_or_zero(a)
    if isempty(a), v = 0; else, v = max(a); end
end

function s = label_of(tid, labels)
    s = sprintf('#%d', tid);
    if ~isempty(labels) && tid <= numel(labels)
        s = labels{tid};
    end
end
