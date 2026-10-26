function matched_pairs = track_matcher_nn(trackSnapshots_R1, trackSnapshots_R2, params)
% track_matcher_nn — 朴素最近邻（NN）跨站航迹配对基线
%
% 【定位】作为三判据关联（track_matcher_dualgate）的对照基线，用于论文 4.4 节
%   量化"三判据比朴素关联好多少"。
%
% 【算法】仅用位置距离（两航迹共存帧的平均 Haversine 距离）对全部候选对做
%   贪心 1v1 指派，不设任何门限、不做时间连续性 / 马氏 / 速度校验。
%   对应开题所述"将一条航迹与另一传感器中距离最近的航迹关联，仅考虑位置相关性，
%   原理简单但鲁棒性较差，适用于目标稀疏场景"。
%
% 【输入/输出签名】与 track_matcher_dualgate 完全一致：
%   trackSnapshots_R1, trackSnapshots_R2 — [n_frames x 1] cell，每帧快照
%   输出 matched_pairs — struct 数组，含 R1_track_id / R2_track_id /
%                        mean_dist_km / coexist_count
    n_frames = length(trackSnapshots_R1);
    r1_ids = pairing.collect_active_ids(trackSnapshots_R1);
    r2_ids = pairing.collect_active_ids(trackSnapshots_R2);
    matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {}, 'mean_dist_km', {}, 'coexist_count', {});
    if isempty(r1_ids) || isempty(r2_ids), return; end

    % 计算所有候选对的平均共存帧位置距离
    cand = {};
    for i = 1:numel(r1_ids)
        for j = 1:numel(r2_ids)
            [md, nc] = pairing.mean_dist(trackSnapshots_R1, trackSnapshots_R2, ...
                r1_ids(i), r2_ids(j), n_frames);
            if nc >= 1 && isfinite(md)
                cand{end+1} = struct('r1', r1_ids(i), 'r2', r2_ids(j), 'md', md, 'nc', nc);  %#ok<AGROW>
            end
        end
    end
    if isempty(cand), return; end

    % 按距离升序贪心 1v1 指派（无门限）
    md_vec = zeros(1, numel(cand));
    for ci = 1:numel(cand)
        md_vec(ci) = cand{ci}.md;
    end
    [~, ord] = sort(md_vec);
    used_r1 = []; used_r2 = [];
    for idx = ord
        c = cand{idx};
        if ismember(c.r1, used_r1) || ismember(c.r2, used_r2), continue; end
        used_r1(end+1) = c.r1;  %#ok<AGROW>
        used_r2(end+1) = c.r2;  %#ok<AGROW>
        matched_pairs(end+1) = struct('R1_track_id', c.r1, 'R2_track_id', c.r2, ...
            'mean_dist_km', c.md, 'coexist_count', c.nc);  %#ok<AGROW>
    end
    if isempty(matched_pairs)
        matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {}, 'mean_dist_km', {}, 'coexist_count', {});
    end

    fprintf('[NN 基线] 候选 %d 对 -> 贪心 1v1 匹配 %d 对\n', numel(cand), numel(matched_pairs));
end
