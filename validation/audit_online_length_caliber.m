function audit_online_length_caliber()
%AUDIT_ONLINE_LENGTH_CALIBER 核查航迹长度统计口径
%
%   系统航迹当前只统计 n_src>0 的帧（纯"有源帧"），
%   单站航迹统计的是航迹存在于 trackList 的帧（含 K_loss 内的 coast 帧）。
%   两种口径不一致，需要量化差异，确认 1.24 这个比值是保守还是虚高。

    S = load(fullfile('results', 'online_adsb_ns.mat'));
    fprintf('mat 变量：%s\n', strjoin(fieldnames(S)', ', '));

    ctSnap = S.ctSnap;
    snap1s = S.snap1s;
    snap2s = S.snap2s;
    n_frames = numel(ctSnap);

    % ---- 系统航迹：两种口径 ----
    ct = struct('id', {}, 'truth', {}, 'n_src_frames', {}, 'n_alive_frames', {});
    for k = 1:n_frames
        CTk = ctSnap{k};
        if isempty(CTk), continue; end
        for i = 1:numel(CTk)
            j = [];
            for m = 1:numel(ct)
                if ct(m).id == CTk(i).id, j = m; break; end
            end
            if isempty(j)
                ct(end+1) = struct('id', CTk(i).id, 'truth', CTk(i).truth_idx, ...
                    'n_src_frames', 0, 'n_alive_frames', 0); %#ok<AGROW>
                j = numel(ct);
            end
            ct(j).n_alive_frames = ct(j).n_alive_frames + 1;
            if CTk(i).n_src > 0
                ct(j).n_src_frames = ct(j).n_src_frames + 1;
            end
        end
    end

    % ---- 单站航迹：总帧数 与 有检测更新帧数 ----
    % 航迹若带 last_update 之类字段则无法回溯，这里只统计存在帧数
    r1 = local_lengths(snap1s, n_frames);
    r2 = local_lengths(snap2s, n_frames);

    truths = unique([[r1.truth], [r2.truth]]);
    truths = truths(~isnan(truths));

    fprintf('\n%-10s %6s %6s %6s %7s %7s %7s\n', ...
        '真值', 'R1', 'R2', '单站', 'CT有源', 'CT存活', '比值(存活)');
    sums = zeros(1, 6);
    for t = 1:numel(truths)
        tid = truths(t);
        a = [r1([r1.truth] == tid).n];
        b = [r2([r2.truth] == tid).n];
        single = max([a(:); b(:); 0]);
        cs = 0; ca = 0;
        for j = 1:numel(ct)
            if ct(j).truth == tid && ct(j).n_src_frames > cs
                cs = ct(j).n_src_frames;  ca = ct(j).n_alive_frames;
            end
        end
        if single > 0
            ratio_alive = ca / single;
        else
            ratio_alive = NaN;
        end
        fprintf('%-10d %6d %6d %6d %7d %7d %7.2f\n', ...
            tid, max([a(:);0]), max([b(:);0]), single, cs, ca, ratio_alive);
        sums = sums + [max([a(:);0]), max([b(:);0]), single, cs, ca, ratio_alive];
    end
    n = numel(truths);
    fprintf('\n均值：R1 %.1f  R2 %.1f  单站 %.1f  CT有源 %.1f  CT存活 %.1f  比值 %.2f\n', ...
        sums(1)/n, sums(2)/n, sums(3)/n, sums(4)/n, sums(5)/n, sums(6)/n);

    % ---- 有源帧占存活帧的比例：coast 撑了多少 ----
    tot_src = sum([ct.n_src_frames]);
    tot_alive = sum([ct.n_alive_frames]);
    fprintf('系统航迹总帧：有源 %d / 存活 %d（有源占 %.1f%%，coast 占 %.1f%%）\n', ...
        tot_src, tot_alive, 100*tot_src/tot_alive, 100*(tot_alive-tot_src)/tot_alive);
end

function out = local_lengths(snaps, n_frames)
    out = struct('id', {}, 'truth', {}, 'n', {});
    for k = 1:n_frames
        s = snaps{k};
        if isempty(s) || ~isfield(s, 'trackList'), continue; end
        trks = s.trackList;
        for t = 1:numel(trks)
            trk = trks{t};
            if ~isfield(trk, 'type') || trk.type == 6 || trk.type == 7, continue; end
            if ~isfield(trk, 'truth_idx') || ~isscalar(trk.truth_idx) ...
                    || ~isfinite(double(trk.truth_idx)), continue; end
            tid = double(trk.truth_idx);
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
