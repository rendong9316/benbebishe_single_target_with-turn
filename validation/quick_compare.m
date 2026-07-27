function quick_compare_default_vs_adaptive()
% 对比 default vs Config#51 vs adaptive (k=2.0) × right_short
    close all;
    addpath(genpath('.'));
    rng(42);

    fprintf('========== Quick Compare: Default vs Config#51 vs Adaptive ========== %s', newline);

    fprintf('%s--- Default Config, k=0 ---%s', newline, newline);
    r1 = run('single_turn_right_short');
    fprintf('  RMSE_R1=%.2f%s', r1.errorStats_R1.overall.ukf.rms, newline);

    fprintf('%s--- Config#51, k=0 ---%s', newline, newline);
    p2 = simulation_params_oracle();
    p2.imm_cv_dwell_time_sec = 2500;
    p2.imm_ct_dwell_time_sec = 660;
    p2.imm_ct_fixed_Q_scale = 5.3;
    p2.imm_transient_gain_max = 11.0;
    r2 = run('single_turn_right_short', p2);
    fprintf('  RMSE_R1=%.2f%s', r2.errorStats_R1.overall.ukf.rms, newline);

    fprintf('%s--- Config#51, k=2.0 ---%s', newline, newline);
    p3 = p2;
    p3.k_turnrate = 2.0;
    r3 = run('single_turn_right_short', p3);
    fprintf('  RMSE_R1=%.2f%s', r3.errorStats_R1.overall.ukf.rms, newline);
end
