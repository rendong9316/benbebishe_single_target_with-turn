function M = eval_pairing(pairs, R1, R2, n_true_both)
% 评估一种配对方法的配对质量（与真实配对基准对比）
%   correct : 输出对 (r1,r2) 的 truth_idx 相等且 >0
%   wrong   : 其余输出对（含跨目标错配、任意一侧为虚假航迹）
%   missed  : 在两站都出现却未被正确覆盖的真实目标数
%   precision = correct / pairs
%   recall    = 被正确覆盖的目标数 / n_true_both
    n_pairs = numel(pairs);
    n_correct = 0;
    n_wrong = 0;
    correct_ti = [];
    for p = 1:n_pairs
        t1 = pairing.truth_idx_of(R1, pairs(p).R1_track_id);
        t2 = pairing.truth_idx_of(R2, pairs(p).R2_track_id);
        if ~isnan(t1) && ~isnan(t2) && t1 == t2 && t1 > 0
            n_correct = n_correct + 1;
            correct_ti(end+1) = t1;  %#ok<AGROW>
        else
            n_wrong = n_wrong + 1;
        end
    end
    n_matched_targets = numel(unique(correct_ti));
    M.n_pairs = n_pairs;
    M.n_correct = n_correct;
    M.n_wrong = n_wrong;
    M.n_missed = max(n_true_both - n_matched_targets, 0);
    M.precision = n_correct / max(n_pairs, 1);
    M.recall = n_matched_targets / max(n_true_both, 1);
    M.match_rate = n_matched_targets / max(n_true_both, 1);
    M.wrong_rate = n_wrong / max(n_pairs, 1);
end
