function st = evaluate_fusion_track_length(result)
%EVALUATE_FUSION_TRACK_LENGTH 融合前后航迹平均长度对比
%
%   对应开题承诺的"航迹平均长度提升 50%"考核指标。
%
%   口径（重要）：
%     1) 单站航迹长度 = 该航迹在快照序列中出现的帧数（排除 type==7 的 HISTORY）
%     2) 融合航迹长度 = 该配对在融合快照序列中"非空"的帧数
%        —— 只要有一站给出估计，融合就输出，所以融合长度天然 ≥ max(两站长度)
%     3) 因此本指标衡量的是"融合延长航迹寿命的能力"，不是"融合算法本身更准"
%     4) 同时给出"双源帧数"（source='both'），这才是真正做了融合的帧
%
%   st = evaluate_fusion_track_length(result)
%
%   输出结构：
%     st.n_pairs              配对数
%     st.methods(m)           name / mean_len / median_len / both / r1_only / r2_only
%     st.single_R1            单站 R1 真实航迹平均长度
%     st.single_R2            单站 R2 真实航迹平均长度
%     st.pairwise(m)          gain_vs_best（逐配对，相对两站中较长者）/ both_ratio
%
%   2026-10-26

    st = struct();
    fs = result.all_fused_snapshots;
    if isempty(fs)
        st.n_pairs = 0; st.methods = []; return;
    end
    n_pairs  = size(fs, 1);
    n_method = size(fs, 2);
    n_frames = numel(fs{1,1});
    st.n_pairs = n_pairs;

    %% ---------- 单站航迹长度（按 track id 索引）----------
    [lenR1, v1] = track_len_map(result.trackSnapshots_R1);
    [lenR2, v2] = track_len_map(result.aligned_R2);
    st.single_R1 = mean_or_nan(v1);
    st.single_R2 = mean_or_nan(v2);

    %% ---------- 逐配对：两站航迹长度 ----------
    pairs = result.matched_pairs;
    pair_len_R1 = nan(n_pairs,1);
    pair_len_R2 = nan(n_pairs,1);
    if ~isempty(pairs)
        for p = 1:min(n_pairs, numel(pairs))
            id1 = pairs(p).R1_track_id;
            id2 = pairs(p).R2_track_id;
            if isKey(lenR1, id1), pair_len_R1(p) = lenR1(id1); end
            if isKey(lenR2, id2), pair_len_R2(p) = lenR2(id2); end
        end
    end
    best_single = max(pair_len_R1, pair_len_R2);
    st.pair_len_R1 = pair_len_R1;
    st.pair_len_R2 = pair_len_R2;

    %% ---------- 逐方法统计 ----------
    names = result.method_names;
    if isempty(names) || numel(names) < n_method
        names = arrayfun(@(k) sprintf('M%d',k), 1:n_method, 'UniformOutput', false);
    end
    st.methods = struct('name',{},'mean_len',{},'median_len',{}, ...
                        'both',{},'r1_only',{},'r2_only',{},'empty',{});

    for m = 1:n_method
        L    = zeros(n_pairs,1);
        nb   = zeros(n_pairs,1);
        n1   = zeros(n_pairs,1);
        n2   = zeros(n_pairs,1);
        nemp = zeros(n_pairs,1);
        for p = 1:n_pairs
            snaps = fs{p,m};
            for k = 1:min(n_frames, numel(snaps))
                s = snaps{k};
                if isempty(s) || ~isfield(s,'trackList') || isempty(s.trackList)
                    nemp(p) = nemp(p) + 1;  continue;
                end
                L(p) = L(p) + 1;
                src = '';
                if isfield(s.trackList{1},'source'), src = s.trackList{1}.source; end
                switch src
                    case 'both',    nb(p) = nb(p) + 1;
                    case 'R1_only', n1(p) = n1(p) + 1;
                    case 'R2_only', n2(p) = n2(p) + 1;
                end
            end
        end
        st.methods(m).name       = names{m};
        st.methods(m).mean_len   = mean(L);
        st.methods(m).median_len = median(L);
        st.methods(m).both       = mean(nb);
        st.methods(m).r1_only    = mean(n1);
        st.methods(m).r2_only    = mean(n2);
        st.methods(m).empty      = mean(nemp);

        ok = isfinite(best_single) & isfinite(pair_len_R1) & isfinite(pair_len_R2) ...
             & best_single > 0 & pair_len_R1 > 0 & pair_len_R2 > 0;
        ok = logical(ok(:));
        Lv  = L(:); bs = best_single(:); a1 = pair_len_R1(:); a2 = pair_len_R2(:);
        st.pairwise(m).name          = names{m};
        st.pairwise(m).gain_vs_best  = 100*mean((Lv(ok) - bs(ok))./bs(ok));
        st.pairwise(m).gain_vs_R1    = 100*mean((Lv(ok) - a1(ok))./a1(ok));
        st.pairwise(m).gain_vs_R2    = 100*mean((Lv(ok) - a2(ok))./a2(ok));
        st.pairwise(m).both_ratio    = mean(nb) / max(mean(L),1);
        st.pairwise(m).n_eff         = sum(ok);
    end
end

% ---------------------------------------------------------------
function [map, n] = track_len_map(snaps)
% 按航迹 id 统计出现帧数（排除 type==7 的 HISTORY 航迹）
    map = containers.Map('KeyType','double','ValueType','double');
    for k = 1:numel(snaps)
        s = snaps{k};
        if isempty(s) || ~isfield(s,'trackList') || isempty(s.trackList), continue; end
        tl = s.trackList;
        for t = 1:numel(tl)
            tr = tl{t};
            if isfield(tr,'type') && tr.type == 7, continue; end
            id = tr.id;
            if isKey(map, id)
                map(id) = map(id) + 1;
            else
                map(id) = 1;
            end
        end
    end
    if map.Count == 0, n = []; else, n = cell2mat(values(map)); end
end

function m = mean_or_nan(x)
    if isempty(x), m = NaN; else, m = mean(x); end
end
