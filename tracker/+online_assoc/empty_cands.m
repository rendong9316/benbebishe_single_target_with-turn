function c = empty_cands()
%EMPTY_CANDS 空候选对数组（字段与实际候选一致）
    c = struct('r1_id', {}, 'r2_id', {}, 'score', {}, 'dist_km', {}, ...
                'd2', {}, 'vdiff', {}, 'vtol', {}, 'n_coexist', {});
end
