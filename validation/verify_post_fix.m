function verify_post_fix()
    % 修复后快速验证：3 种子 x 4 场景 x 3 配置
    % Default(k=0) vs Sigmoid(k=3,t=0.3,p=1.0) vs Cfg51+Sigmoid(k=3)
    %
    % 目的：确认 sigmoid 符号修复后效果是否成立
    % 注意：这些 .mat 文件是 Phase 3/4 用修复后的代码跑的
    % 但如果 MATLAB 环境缓存了 .p 文件，需要先清理

    close all;
    addpath(genpath('.'));

    scenarios = {
        'right_rate_0p7', 'R rate=0.7/s (Phase1失败场景)';
        'left_rate_0p7', 'L rate=0.7/s';
        'right_rate_1p3', 'R rate=1.3/s (Phase1 Config51无效场景)';
        'left_rate_1p3', 'L rate=1.3/s';
    };
    seeds = [42, 123, 456];

    fprintf('=== Post-Fix Quick Verification ===\n');
    fprintf('Config: Default | Cfg51+Sigmoid(k3) | Sigmoid(k3 only)\n\n');

    for si = 1:size(scenarios, 1)
        sname = scenarios{si, 1};
        sdesc = scenarios{si, 2};
        fprintf('>>> %s (%s)\n', sdesc, sname);

        for seidx = 1:length(seeds)
            seed = seeds(seidx);

            % Default
            p1 = simulation_params_oracle();
            p1.random_seed = seed;
            r1 = run(sname, p1);
            rmse1 = r1.errorStats_R1.overall.ukf.rms;

            % Cfg51 + Sigmoid k=3/p=1/t=0.3
            p2 = simulation_params_oracle();
            p2.random_seed = seed;
            p2.imm_cv_dwell_time_sec = 2500;
            p2.imm_ct_dwell_time_sec = 660;
            p2.imm_ct_fixed_Q_scale = 5.3;
            p2.imm_transient_gain_max = 11.0;
            p2.k_turnrate = 3;
            p2.imm_adaptive_mode = 'sigmoid';
            p2.threshold = 0.3;
            p2.power = 1.0;
            r2 = run(sname, p2);
            rmse2 = r2.errorStats_R1.overall.ukf.rms;

            % Sigmoid only (no Cfg51 boost)
            p3 = simulation_params_oracle();
            p3.random_seed = seed;
            p3.k_turnrate = 3;
            p3.imm_adaptive_mode = 'sigmoid';
            p3.threshold = 0.3;
            p3.power = 1.0;
            r3 = run(sname, p3);
            rmse3 = r3.errorStats_R1.overall.ukf.rms;

            imp2 = (rmse1 - rmse2)/rmse1*100;
            imp3 = (rmse1 - rmse3)/rmse1*100;

            mark2 = ''; if abs(imp2)>2, mark2='*'; end
            mark3 = ''; if abs(imp3)>2, mark3='*'; end

            fprintf('  Seed %d: Def=%.3f | Cfg51+Sig=%.3f(%+.1f%%%s) | SigOnly=%.3f(%+.1f%%%s)\n', ...
                seed, rmse1, rmse2, imp2, mark2, rmse3, imp3, mark3);
        end
        fprintf('\n');
    end

    fprintf('Legend: * = improvement > 2%%\n');
    fprintf('Note: All results use the FIXED ukf_imm.m source.\n');
    fprintf('If .p files are cached, they may give stale results.\n');
end
