function eval_scenario(scenario_name, scenario_label)
    close all;
    addpath(genpath('.'));
    fprintf('%s--- %s ---%s', newline, scenario_label, newline);

    rb = run(scenario_name);
    bl = rb.errorStats_R1.overall.ukf.rms;
    fprintf('  Baseline: %.2f%s', bl, newline);

    modes = {'linear', 'exp_decay', 'sigmoid'};
    k_vals = [1, 3, 5];

    for mi = 1:length(modes)
        mode = modes{mi};
        for ki = 1:length(k_vals)
            k = k_vals(ki);
            cfg = sprintf('%s_k%d', mode, k);

            params = simulation_params_oracle();
            params.k_turnrate = k;
            if ~strcmp(mode, 'linear')
                params.imm_adaptive_mode = mode;
                if strcmp(mode, 'exp_decay'), params.power = 1.5; end
                if strcmp(mode, 'sigmoid'), params.threshold = 0.3; params.power = 1.0; end
            end

            try
                rr = run(scenario_name, params);
                rmse = rr.errorStats_R1.overall.ukf.rms;
                imp = (bl - rmse)/bl*100;
                marker = '';
                if abs(imp) > 1 && imp > 0, marker = '*'; end
                if abs(imp) > 1 && imp < 0, marker = '!'; end
                fprintf('  %s: %.2f (%+.1f%%)%s%s', cfg, rmse, imp, marker, newline);
            catch ME
                fprintf('  %s: ERROR %s%s', cfg, ME.message(1:30), newline);
            end
        end
    end
end
