function test_key_configs()
    close all;
    addpath(genpath('.'));
    rng(42);

    baseline_r = run('single_turn_right_short');
    baseline_u = run('single_uturn');
    bl_r = baseline_r.errorStats_R1.overall.ukf.rms;
    bl_u = baseline_u.errorStats_R1.overall.ukf.rms;

    fprintf('========== Key Adaptive Configs Test ==========%s', newline);
    fprintf('Baseline: right_short=%.2fkm  uturn=%.2fkm%s%s', ...
        bl_r, bl_u, newline, newline);

    % 关键配置列表
    n_test = 6;
    cfg_names = cell(n_test, 1);
    cfg_k = zeros(n_test, 1);
    cfg_mode = cell(n_test, 1);
    cfg_power = zeros(n_test, 1);
    cfg_threshold = zeros(n_test, 1);

    cfg_names{1} = 'exp_decay_k3_p1.5';  cfg_k(1) = 3;  cfg_mode{1} = 'exp_decay';  cfg_power(1) = 1.5;  cfg_threshold(1) = 0.4;
    cfg_names{2} = 'exp_decay_k5_p1.5';  cfg_k(2) = 5;  cfg_mode{2} = 'exp_decay';  cfg_power(2) = 1.5;  cfg_threshold(2) = 0.4;
    cfg_names{3} = 'exp_decay_k5_p2.0';  cfg_k(3) = 5;  cfg_mode{3} = 'exp_decay';  cfg_power(3) = 2.0;  cfg_threshold(3) = 0.4;
    cfg_names{4} = 'sigmoid_k3_t0.3';    cfg_k(4) = 3;  cfg_mode{4} = 'sigmoid';    cfg_power(4) = 1.0;  cfg_threshold(4) = 0.3;
    cfg_names{5} = 'sigmoid_k5_t0.3';    cfg_k(5) = 5;  cfg_mode{5} = 'sigmoid';    cfg_power(5) = 1.0;  cfg_threshold(5) = 0.3;
    cfg_names{6} = 'sigmoid_k5_t0.5p2';  cfg_k(6) = 5;  cfg_mode{6} = 'sigmoid';    cfg_power(6) = 2.0;  cfg_threshold(6) = 0.5;

    for ci = 1:n_test
        params = simulation_params_oracle();
        params.k_turnrate = cfg_k(ci);
        params.imm_adaptive_mode = cfg_mode{ci};
        if strcmp(cfg_mode{ci}, 'exp_decay')
            params.power = cfg_power(ci);
        elseif strcmp(cfg_mode{ci}, 'sigmoid')
            params.threshold = cfg_threshold(ci);
            params.power = cfg_power(ci);
        end

        try
            rs = run('single_turn_right_short', params);
            ru = run('single_uturn', params);
            rsm = rs.errorStats_R1.overall.ukf.rms;
 rum = ru.errorStats_R1.overall.ukf.rms;
            imp_s = (bl_r - rsm)/bl_r * 100;
            imp_u = (bl_u - rum)/bl_u * 100;

            marker = '';
            if abs(imp_u) > 1 && imp_u > 0, marker = '* IMPROVED'; end
            if abs(imp_u) > 1 && imp_u < 0, marker = '* WORSE'; end

            fprintf('%-20s k=%.0f | RS=%.2f(%+.1f%%) UT=%.2f(%+.1f%%)%s%s', ...
                cfg_names{ci}, cfg_k(ci), rsm, imp_s, rum, imp_u, marker, newline);

        catch ME
            fprintf('%-20s ERROR: %.40s%s', cfg_names{ci}, ME.message, newline);
        end
    end
    fprintf('%sDone.%s', newline, newline);
end
