function run_full_eval_simple()
    close all;
    addpath(genpath('.'));
    fprintf('========== Full Adaptive PI Eval ========== %s', newline);
    fprintf('%s', newline);

    scenarios = { ...
        'single_straight', 'straight'; ...
        'single_turn_right_short', 'right_short'; ...
        'single_uturn', 'uturn'; ...
        'single_turn_left_sustained', 'left_sust'; ...
        'multi_cross', 'multi_cross' ...
    };
    n_scenarios = size(scenarios, 1);
    k_vals = [1, 3, 5];
    modes = { 'linear', 'exp_decay', 'sigmoid' };

    results = NaN(n_scenarios, 3 * length(k_vals));

    for si = 1:n_scenarios
        sname = scenarios{si, 1};
        slabel = scenarios{si, 2};

        fprintf('%s--- %-12s ---%s', newline, slabel, newline);

        rb = run(sname);
        bl(si) = rb.errorStats_R1.overall.ukf.rms;
        fprintf('  Baseline: %.2f%s', bl(si), newline);

        cidx = 0;
        for ci = 1:length(modes)
            mode = modes{ci};
            for ki = 1:length(k_vals)
                k = k_vals(ki);
                cidx = cidx + 1;

                cfg_name = sprintf('%s_k%d', mode, k);
                params = simulation_params_oracle();
                if strcmp(mode, 'linear')
                    params.k_turnrate = k;
                else
                    params.k_turnrate = k;
                    params.imm_adaptive_mode = mode;
                    if strcmp(mode, 'exp_decay'), params.power = 1.5; end
                    if strcmp(mode, 'sigmoid'), params.threshold = 0.3; params.power = 1.0; end
                end

                try
                    rr = run(sname, params);
                    rmse = rr.errorStats_R1.overall.ukf.rms;
                    results(si, cidx) = rmse;
                    imp = (bl(si) - rmse)/bl(si)*100;
                    marker = '';
                    if abs(imp) > 1 && imp > 0, marker = '*'; end
                    if abs(imp) > 1 && imp < 0, marker = '!'; end
                    fprintf('  %s: %.2f (%+.1f%%)%s%s', cfg_name, rmse, imp, marker, newline);
                catch
                    results(si, cidx) = NaN;
                    fprintf('  %s: ERROR%s', cfg_name, newline);
                end
            end
        end
    end

    % Summary
    fprintf('%s=== SUMMARY ===%s', newline, newline);
    fprintf('%-14s', 'Scenario');
    for cidx = 1:9
        cfg = sprintf('%d_%s', fix((cidx-1)/3)+1, char(mod(cidx-1,3)+'l'));
        fprintf('%-8s', cfg);
    end
    fprintf('%s', newline);
    fprintf('%s', repmat('-', 86, 1));

    for si = 1:n_scenarios
        fprintf('%-14s', scenarios{si,2});
        for cidx = 1:9
            r = results(si, cidx);
            if isnan(r), fprintf('%-8s', 'N/A'); continue; end
            imp = (bl(si)-r)/bl(si)*100;
            flag = '';
            if abs(imp)>1 && imp>0, flag='*'; end
            fprintf('%-7.2f%%+-3.1f ', r, imp);
            if flag~=0, fprintf('%s', flag); end
        end
        fprintf('%s', newline);
    end
end
