function matched_pairs = track_matcher_sequential(trackSnapshots_R1, trackSnapshots_R2, params)
% track_matcher_sequential — 序贯最近邻（SNN）跨站航迹配对基线
%
% 【定位】介于 NN 与三判据之间的中间基线，用于论文 4.4 节展示"引入协方差门限
%   即已明显优于纯位置 NN，但缺少时间连续性 + 速度判据仍弱于三判据"。
%
% 【算法】在 NN 基础上引入马氏距离校验门限（利用滤波器协方差）：
%   仅接受共存帧中 D²<门限 占比达标的候选对（D² ~ χ²(2)），再按马氏距离均值
%   升序贪心 1v1 指派。未通过门限的航迹保持未匹配（可被漏配）。
%   复用双门限的位置粗筛 T1 与最少共现帧数参数。
%
% 【输入/输出签名】与 track_matcher_dualgate 完全一致。
    n_frames = length(trackSnapshots_R1);
    T1 = pairing.get_param_local(params, 'dualgate_T1_km', 50);
    maha_thresh = pairing.get_param_local(params, 'dualgate_maha_thresh', 5.99);
    maha_ratio  = pairing.get_param_local(params, 'dualgate_maha_ratio', 0.8);
    coexist_thresh = pairing.get_param_local(params, 'dualgate_coexist_thresh', 3);

    r1_ids = pairing.collect_active_ids(trackSnapshots_R1);
    r2_ids = pairing.collect_active_ids(trackSnapshots_R2);
    matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {}, 'mean_dist_km', {}, ...
        'coexist_count', {}, 'maha_mean', {}, 'maha_ratio', {});
    if isempty(r1_ids) || isempty(r2_ids), return; end

    % 候选生成 + 马氏门限校验
    cand = {};
    for i = 1:numel(r1_ids)
        for j = 1:numel(r2_ids)
            [md, nc, D2, valid] = pairing.pair_stats(trackSnapshots_R1, trackSnapshots_R2, ...
                r1_ids(i), r2_ids(j), n_frames);
            if nc < coexist_thresh || ~isfinite(md), continue; end
            if md >= T1, continue; end                       % 位置粗筛（与双门限一致）
            if ~any(valid), continue; end
            pass = sum(valid & D2 < maha_thresh) / sum(valid);
            if pass < maha_ratio, continue; end              % 马氏门限未通过
            cand{end+1} = struct('r1', r1_ids(i), 'r2', r2_ids(j), 'md', md, ...
                'nc', nc, 'maha', mean(D2(valid)));  %#ok<AGROW>
        end
    end
    if isempty(cand), return; end

    % 按马氏距离均值升序贪心 1v1 指派（显式循环，避免 R2023a 对 struct-cell 的 arrayfun 限制）
    mah_vec = zeros(1, numel(cand));
    for ci = 1:numel(cand)
        mah_vec(ci) = cand{ci}.maha;
    end
    [~, ord] = sort(mah_vec);
    used_r1 = []; used_r2 = [];
    for idx = ord
        c = cand{idx};
        if ismember(c.r1, used_r1) || ismember(c.r2, used_r2), continue; end
        used_r1(end+1) = c.r1;  %#ok<AGROW>
        used_r2(end+1) = c.r2;  %#ok<AGROW>
        matched_pairs(end+1) = struct('R1_track_id', c.r1, 'R2_track_id', c.r2, ...
            'mean_dist_km', c.md, 'coexist_count', c.nc, ...
            'maha_mean', c.maha, 'maha_ratio', maha_ratio);  %#ok<AGROW>
    end
    if isempty(matched_pairs)
        matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {}, 'mean_dist_km', {}, ...
            'coexist_count', {}, 'maha_mean', {}, 'maha_ratio', {});
    end

    fprintf('[Sequential 基线] 候选 %d 对（门限后）-> 匹配 %d 对\n', numel(cand), numel(matched_pairs));
end
