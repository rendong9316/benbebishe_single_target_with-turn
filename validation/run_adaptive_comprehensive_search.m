function run_adaptive_comprehensive_search()
% 全面搜索自适应PI的参数空间和函数形态
%
% 搜索空间:
%   - 反馈函数: linear, exp_decay, sigmoid
%   - k值: [0, 1.0, 2.0, 3.0, 5.0] (k更大时指数/非线性才有优势)
%   - power(sigmoid): [1.0, 1.5, 2.0]
%   - threshold(sigmoid): [0.3, 0.4, 0.5]
%   - mode参数: in params.imm_adaptive_mode
%
% 场景: right_short, uturn (default config)
    close all;
    addpath(genpath('.'));
    rng(42);

    fprintf('========== Comprehensive Adaptive PI Search ==========%s', newline);
    fprintf('%s', newline);

    % 实验配置
    experiment_keys = { ...
        'mode', 'k_val', 'power', 'threshold';
        'linear',      2.0,     1.0, 0.4;
        'exp_decay',   2.0,     1.0, 0.4;
        'exp_decay',   3.0,     1.0, 0.4;
        'exp_decay',   5.0,     1.5, 0.4;
        'exp_decay',   5.0,     2.0, 0.4;
        'sigmoid',     2.0,     1.0, 0.3;
        'sigmoid',     3.0,     1.0, 0.4;
        'sigmoid',     5.0,     1.5, 0.4;
        'sigmoid',     5.0,     2.0, 0.5;
    };

    scenarios = {'single_turn_right_short', 'single_uturn'};
    sc_labels = {'right_short(47s)', 'uturn(180s)'};

    % baseline: default config, k=0 (两个场景各跑一次)
    fprintf('%sBaseline (default config, fixed Pi):%s', newline, newline);
    baseline_rmse = zeros(1, 2);

    for si = 1:length(scenarios)
        r = run(scenarios{si});
        baseline_rmse(si) = r.errorStats_R1.overall.ukf.rms;
        fprintf('  %-16s RMSE_R1=%.2fkm%s', sc_labels{si}, baseline_rmse(si), newline);
    end

    fprintf('%s%s%s', newline, repmat('=', 80, 1), newline);

    n_exp = size(experiment_keys, 1) - 1;
    printf_list = cell(n_exp + 1, 9);
    printf_list{1} = {'#', 'mode', 'k', 'power', 'thresh', 'right_s', 'improvement', 'scenario', 'winner'};

    results = struct();

    for ei = 1:n_exp
        mode = experiment_keys{ei+1, 1};
        k_val = str2double(experiment_keys{ei+1, 2});
        power = str2double(experiment_keys{ei+1, 3});
        threshold = str2double(experiment_keys{ei+1, 4});

        printf_list{ei+1, 1} = num2str(ei);
        printf_list{ei+1, 2} = mode;
        printf_list{ei+1, 3} = num2str(k_val);
        printf_list{ei+1, 4} = num2str(power);
        printf_list{ei+1, 5} = num2str(threshold);

        best_sc_improv = 0;
        best_sc_name = '';
        best_sc_idx = 0;

        for si = 1:length(scenarios)
            sname = scenarios{si};
            params = simulation_params_oracle();
            params.k_turnrate = k_val;
            params.imm_adaptive_mode = mode;

            if strcmp(mode, 'exp_decay')
                params.power = power;
            elseif strcmp(mode, 'sigmoid')
                params.threshold = threshold;
                params.power = power;
            end

            try
                r = run(sname, params);
                rmse = r.errorStats_R1.overall.ukf.rms;
                improv = (baseline_rmse(si) - rmse)/baseline_rmse(si)*100;

                printf_list{ei+1, 6} = sprintf('%.2f (%+.1f%%)', rmse, improv);

                if improv > best_sc_improv
                    best_sc_improv = improv;
                    best_sc_name = sc_labels{si};
                    best_sc_idx = si;
                end

            catch ME
                printf_list{ei+1, 6} = sprintf('ERROR');
            end
        end

        if best_sc_improv > 1.0
            printf_list{ei+1, 9} = '*WINNER*';
        else
            printf_list{ei+1, 9} = ' ';
        end

        results(ei).mode = mode;
        results(ei).k = k_val;
        results(ei).power = power;
        results(ei).threshold = threshold;
        results(ei).best_improvement = best_sc_improv;
        results(ei).best_scenario = best_sc_name;
    end

    % ---- 输出表格 ----
    fprintf('%-4s %-10s %-4s %-7s %-8s %-15s%s', '#', 'mode', 'k', 'power', 'thresh', 'right_short / uturn', newline);
    fprintf('%s', repmat('-', 66, 1));

    for ei = 1:n_exp
        row_str = sprintf('%-4s %-10s %-4.1f %-7.1f %-8.1f %-15s %-15s', ...
            num2str(ei), experiment_keys{ei+1,1}, str2double(experiment_keys{ei+1,2}), ...
            str2double(experiment_keys{ei+1,3}), str2double(experiment_keys{ei+1,4}), ...
            printf_list{ei+1,6}, '');
        if ~isempty(experiment_keys{ei+1,9})
            row_str = sprintf('%s [%s]', row_str, experiment_keys{ei+1,9}{1});
        end
        fprintf('%s%s', row_str, newline);
    end

    fprintf('%s%s', newline, repmat('=', 80, 1));
    fprintf('%s', newline);

    save('results/adaptive_pi_comprehensive_search.mat', ...
        'experiment_keys', 'scenarios', 'sc_labels', 'baseline_rmse', 'printf_list');
end
