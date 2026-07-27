function ablation_config51_sigmoid()
% 消融实验: Config#51 + Sigmoid 是否比单独 Config#51 更好?
% 重点是 right_rate_0p7 (Config#51在该场景恶化+54%)
    close all;
    addpath(genpath('.'));

    fprintf('========== Ablation: Config#51 + Sigmoid ========== %s', newline);
    fprintf('%s', newline);

    scenarios = { ...
        'single_straight', 'straight';
        'single_turn_right_short', 'right_short';
        'single_uturn', 'uturn';
        'right_rate_0p7', 'right_rate_0p7';
        'left_rate_0p7', 'left_rate_0p7';
        'right_rate_1p3', 'right_rate_1p3';
        'left_rate_1p3', 'left_rate_1p3';
    };

    rng(42);

    for si = 1:size(scenarios, 1)
        sname = scenarios{si, 1};
        slabel = scenarios{si, 2};

        fprintf('%s--- %-18s ---%s', newline, slabel, newline);

        % A. Default baseline
        try
            rA = run(sname);
            rmseA = rA.errorStats_R1.overall.ukf.rms;
        catch ME
            rmseA = NaN;
        end

        % B. Config#51 (fixed)
        pB = simulation_params_oracle();
        pB.imm_cv_dwell_time_sec = 2500;
        pB.imm_ct_dwell_time_sec = 660;
        pB.imm_ct_fixed_Q_scale = 5.3;
        pB.imm_transient_gain_max = 11.0;
        try
            rB = run(sname, pB);
            rmseB = rB.errorStats_R1.overall.ukf.rms;
        catch ME
            rmseB = NaN;
        end

        % C. Default + Sigmoid k=3
        pC = simulation_params_oracle();
        pC.k_turnrate = 3;
        pC.imm_adaptive_mode = 'sigmoid';
        pC.threshold = 0.3;
        pC.power = 1.0;
        try
            rC = run(sname, pC);
            rmseC = rC.errorStats_R1.overall.ukf.rms;
        catch ME
            rmseC = NaN;
        end

        % D. Config#51 + Sigmoid k=3 (组合)
        pD = pB;
        pD.k_turnrate = 3;
        pD.imm_adaptive_mode = 'sigmoid';
        pD.threshold = 0.3;
        pD.power = 1.0;
        try
            rD = run(sname, pD);
            rmseD = rD.errorStats_R1.overall.ukf.rms;
        catch ME
            rmseD = NaN;
        end

        % 报告
        bl = rmseA;
        fprintf('  Default            : %.2f km%s', rmseA, newline);
        if ~isnan(rmseB)
            impB = (bl - rmseB)/bl*100;
            fprintf('  Config#51          : %.2f km (%+.1f%%)%s', rmseB, impB, newline);
        end
        if ~isnan(rmseC)
            impC = (bl - rmseC)/bl*100;
            fprintf('  Default+Sigmoid    : %.2f km (%+.1f%%)%s', rmseC, impC, newline);
        end
        if ~isnan(rmseD)
            impD = (bl - rmseD)/bl*100;
            fprintf('  Config#51+Sigmoid  : %.2f km (%+.1f%%)%s', rmseD, impD, newline);
        end

        % 最优配置标记
        rmses = [rmseA, rmseB, rmseC, rmseD];
        names = {'Default', 'Config#51', 'Def+Sigmoid', 'Cfg51+Sigmoid'};
        [best_val, best_idx] = min(rmses);
        if ~isnan(best_val) && best_val < bl - 0.05
            fprintf('  >> WINNER: %s%s', names{best_idx}, newline);
        end
    end
end
