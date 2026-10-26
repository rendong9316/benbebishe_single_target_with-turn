function [n_true_both, r1_ids, r1_ti, r2_ids, r2_ti] = build_truth_pairs(R1, R2)
% 构建跨站真实配对基准：统计在两站都出现且 truth_idx>0 的真实目标数
    r1_ids = pairing.collect_active_ids(R1);
    r2_ids = pairing.collect_active_ids(R2);
    r1_ti = zeros(size(r1_ids));
    for i = 1:numel(r1_ids)
        r1_ti(i) = pairing.truth_idx_of(R1, r1_ids(i));
    end
    r2_ti = zeros(size(r2_ids));
    for j = 1:numel(r2_ids)
        r2_ti(j) = pairing.truth_idx_of(R2, r2_ids(j));
    end
    t1set = unique(r1_ti(r1_ti > 0));
    t2set = unique(r2_ti(r2_ti > 0));
    common = intersect(t1set, t2set);
    n_true_both = numel(common);
end
