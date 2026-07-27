function quick_final_3()
    % 只测uturn场景的三种配置
    close all;
    addpath(genpath('.'));
    rng(42);

    fprintf('========== Quick Final: 3 configs × uturn ========== %s', newline);

    fprintf('%s--- Default+k=0 ---%s', newline, newline);
    r1 = run('single_uturn');
    rmse1 = r1.errorStats_R1.overall.ukf.rms;
    fprintf('RMSE=%.2f%s', rmse1, newline);

    fprintf('%s--- Default+Sigmoid(k=3,t=0.3) ---%s', newline, newline);
    p2 = simulation_params_oracle();
    p2.k_turnrate = 3;
    p2.imm_adaptive_mode = 'sigmoid';
    p2.threshold = 0.3;
    p2.power = 1.0;
    r2 = run('single_uturn', p2);
    rmse2 = r2.errorStats_R1.overall.ukf.rms;
    imp2 = (rmse1 - rmse2)/rmse1 * 100;
    fprintf('RMSE=%.2f (%+.1f%% vs baseline)%s', rmse2, imp2, newline);

    fprintf('%s--- Done.%s', newline, newline);
end
