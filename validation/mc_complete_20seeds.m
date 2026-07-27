function mc_complete_20seeds()
% 20 seeds × 4变速率场景 × 3配置 = 240次完整仿真
% 分批运行避免heap corruption: 每批1个场景×5种子
    close all;
    addpath(genpath('.'));

    fprintf('========== MC Complete: 20 Seeds × 4 Scenes × 3 Cfg ==========%s', newline);
    fprintf('%s', newline);

    seeds = 1:20;
    n_seeds = numel(seeds);

    % 4个变速率场景
    scenes_info = struct(...
        'name', {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'}, ...
        'label', {'rr0p7', 'lr0p7', 'rr1p3', 'lr1p3'});
    n_scenes = numel(scenes_info);

    % 3 configs
    % cfg 1: Default, cfg 2: Config51, cfg 3: Cfg51+Sigmoid
    all_rmse = NaN(n_seeds, n_scenes, 3);

    for sc_i = 1:n_scenes
        sname = scenes_info(sc_i).name;
        slabel = scenes_info(sc_i).label;
        fprintf('%s============================================================%s', newline, newline);
        fprintf('=== SCENE %d/%d: %s (%s) ===%s', sc_i, n_scenes, sname, slabel, newline);
        fprintf('%s============================================================%s', newline, newline);

        for si = 1:n_seeds
            seed = seeds(si);
            fprintf('  seed=%3d/%d%s', seed, n_seeds, newline);

            % --- Config 1: Default ---
            try
                p1 = simulation_params_oracle();
                p1.random_seed = seed;
                r1 = run(sname, p1);
                v1 = r1.errorStats_R1.overall.ukf.rms;
                all_rmse(si, sc_i, 1) = v1;
                fprintf('    Default: %.3f', v1);
            catch ME
                fprintf('    Default: ERROR');
            end

            % --- Config 2: Config51 ---
            try
                p2 = simulation_params_oracle();
                p2.random_seed = seed;
                p2.imm_cv_dwell_time_sec = 2500;
                p2.imm_ct_dwell_time_sec = 660;
                p2.imm_ct_fixed_Q_scale = 5.3;
                p2.imm_transient_gain_max = 11.0;
                r2 = run(sname, p2);
                v2 = r2.errorStats_R1.overall.ukf.rms;
                all_rmse(si, sc_i, 2) = v2;
                fprintf('    Config51: %.3f', v2);
            catch ME
                fprintf('    Config51: ERROR');
            end

            % --- Config 3: Cfg51+Sigmoid ---
            try
                p3 = simulation_params_oracle();
                p3.random_seed = seed;
                p3.imm_cv_dwell_time_sec = 2500;
                p3.imm_ct_dwell_time_sec = 660;
                p3.imm_ct_fixed_Q_scale = 5.3;
                p3.imm_transient_gain_max = 11.0;
                p3.k_turnrate = 3;
                p3.imm_adaptive_mode = 'sigmoid';
                p3.threshold = 0.3;
                p3.power = 1.0;
                r3 = run(sname, p3);
                v3 = r3.errorStats_R1.overall.ukf.rms;
                all_rmse(si, sc_i, 3) = v3;
                fprintf('    Cfg51+Sig: %.3f', v3);
            catch ME
                fprintf('    Cfg51+Sig: ERROR');
            end

            fprintf('%s', newline);
        end

        % Batch summary per scene
        fprintf('%s--- %s Batch Summary ---%s', newline, slabel, newline);
        for ci = 1:3
            valid = all_rmse(:, sc_i, ci);
            valid = valid(~isnan(valid));
            if ~isempty(valid)
                m = mean(valid);
                s = std(valid);
                fprintf('  %-12s: %.3f ± %.3f (n=%d)%s', ...
                    {'Default','Config51','Cfg51+Sigmoid'}{ci}, m, s, length(valid), newline);
            end
        end
    end

    %% ---- Final Cross-Scene Summary ----
    fprintf('%s%s', newline, repmat('=', 80, 1));
    fprintf('FINAL SUMMARY: 20 seeds × 4 scenes × 3 configs = %d expected runs%s', ...
        n_seeds * n_scenes * 3, newline);
    fprintf('%s%s', repmat('-', 80, 1), newline);
    fprintf('%-12s | %-14s | %-14s | %-14s | %s', ...
        'Scene', 'Default', 'Config51', 'Cfg51+Sigmoid', newline);
    fprintf('%s', repmat('-', 80, 1));

    for sc_i = 1:n_scenes
        fprintf('%-12s | ', scenes_info(sc_i).label);
        for ci = 1:3
            valid = all_rmse(:, sc_i, ci);
            valid = valid(~isnan(valid));
            if ~isempty(valid)
                m = mean(valid);
                s = std(valid);
                fprintf(' %.2f±%.2f | ', m, s);
            else
                fprintf('       -- | ');
            end
        end
        fprintf('%s', newline);
    end

    fprintf('%s', repmat('-', 80, 1));

    % Overall averages across scenes
    fprintf('%-12s | ', 'OVERALL');
    for ci = 1:3
        valid = all_rmse(: , :, ci);
        valid = valid(~isnan(valid));
        if ~isempty(valid)
            m = mean(valid);
            s = std(valid);
            fprintf(' %.2f±%.2f | ', m, s);
        end
    end
    fprintf('%s', newline);

    % 改善百分比
    fprintf('%s%s', newline, repmat('-', 80, 1));
    fprintf('IMPROVEMENT vs Default (mean across valid runs):%s', newline);
    for sc_i = 1:n_scenes
        valid_bl = all_rmse(:, sc_i, 1);
        valid_c51 = all_rmse(:, sc_i, 2);
        valid_sg = all_rmse(:, sc_i, 3);

        common = (~isnan(valid_bl)) & (~isnan(valid_c51)) & (~isnan(valid_sg));
        if any(common)
            imp_c51 = (valid_bl(common) - valid_c51(common)) ./ valid_bl(common) * 100;
            imp_sg = (valid_bl(common) - valid_sg(common)) ./ valid_bl(common) * 100;
            imp_sgin_addon = (valid_c51(common) - valid_sg(common)) ./ valid_c51(common) * 100;

            fprintf('  %s: Cfg51 vs Default = %+.1f%% | Sigmoid vs Default = %+.1f%% | Add-on = %+.1f%%%s', ...
                scenes_info(sc_i).label, mean(imp_c51), mean(imp_sg), mean(imp_sgin_addon), newline);
        end
    end

    fprintf('%s%s', newline, repmat('=', 80, 1));

    % Save all results
    save('results/mc_complete_20seeds.mat', 'all_rmse', 'seeds', 'scenes_info');
    fprintf('%sSaved to results/mc_complete_20seeds.mat%s', newline, newline);
end
