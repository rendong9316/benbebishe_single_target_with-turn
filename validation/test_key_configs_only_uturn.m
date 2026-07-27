function test_key_configs_only_uturn()
    close all;
    addpath(genpath('.'));
    rng(42);

    baseline_u = run('single_uturn');
    bl_u = baseline_u.errorStats_R1.overall.ukf.rms;
    fprintf('========== Adaptive PI on U-Turn Only ========== %s', newline);
    fprintf('Baseline (default, k=0): uturn=%.2fkm%s%s', bl_u, newline, newline);

    n_test = 6;
    cfg_names = { 'exp_decay_k3_p1.5'; 'exp_decay_k5_p1.5'; 'exp_decay_k5_p2.0';
                  'sigmoid_k3_t0.3'; 'sigmoid_k5_t0.3'; 'sigmoid_k5_t0.5p2' };
    cfg_k = [3; 5; 5; 3; 5; 5];
    cfg_mode = { 'exp_decay'; 'exp_decay'; 'exp_decay'; 'sigmoid'; 'sigmoid'; 'sigmoid' };
    cfg_power = [1.5; 1.5; 2.0; 1.0; 1.0; 2.0];
    cfg_threshold = [0.4; 0.4; 0.4; 0.3; 0.3; 0.5];

    for ci = 1:n_test
        params = simulation_params_oracle();
        params.k_turnrate = cfg_k(ci);
        params.imm_adaptive_mode = cfg_mode{ci};
        if strcmp(cfg_mode{ci}, 'exp_decay'), params.power = cfg_power(ci); end
        if strcmp(cfg_mode{ci}, 'sigmoid')
            params.threshold = cfg_threshold(ci);
            params.power = cfg_power(ci);
        end

        try
            ru = run('single_uturn', params);
            rum = ru.errorStats_R1.overall.ukf.rms;
            imp_u = (bl_u - rum)/bl_u * 100;

            marker = '';
            if abs(imp_u) > 1 && imp_u > 0, marker = ' <-- IMPROVED'; end
            if abs(imp_u) > 1 && imp_u < 0, marker = ' <-- WORSE'; end

            fprintf('%-20s: k=%.0f | UT=%.2f(%+.1f%%)%s%s', ...
                cfg_names{ci}, cfg_k(ci), rum, imp_u, marker, newline);

        catch ME
            fprintf('%-20s: ERROR %s%s', cfg_names{ci}, ME.message(1:40), newline);
        end
    end
    fprintf('%sDone.%s', newline, newline);
end
