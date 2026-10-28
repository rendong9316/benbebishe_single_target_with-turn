function A = empty_acc()
%EMPTY_ACC 空的逐对累计证据数组（字段与实际记录一致）
    t = struct('r1', 0, 'r2', 0, 'n_coexist', 0, 'sum_ed', 0, ...
               'n_maha', 0, 'n_vel', 0, 'run_ok', 0, 'max_run_ok', 0, ...
               'last_k', NaN, 'last_ev', online_assoc.empty_ev());
    A = repmat(t, 0, 1);
end
