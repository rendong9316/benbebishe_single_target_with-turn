function sweep_10seeds()
    % 10-seed turn_rate sweep - simple and direct
    close all;
    addpath(genpath(pwd));

    candidates_deg = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0];
    scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};
    true_rate = [0.7, 0.7, 1.3, 1.3];
    n_cand = length(candidates_deg);
    n_sc = length(scenarios);
    n_seeds = 10;
    seeds = 1:n_seeds;
    save_file = 'results/sweep_10seeds.mat';

    % Load existing progress
    rmse_mat = NaN(n_sc, n_cand, n_seeds);
    if exist(save_file, 'file')
        try
            S = load(save_file);
            for si = 1:n_seeds
                for sci = 1:n_sc
                    for ci = 1:n_cand
                        if ~isnan(S.rmse_mat(sci,ci,si))
                            rmse_mat(sci,ci,si) = S.rmse_mat(sci,ci,si);
                        end
                    end
                end
            end
            fprintf('断点续跑: 已完成 %d/%d tasks\n', ...
                sum(~isnan(rmse_mat(:)), 'all'), numel(rmse_mat));
        catch
        end
    end

    total_runs = numel(rmse_mat);
    done_runs = sum(~isnan(rmse_mat(:)));
    fprintf('总任务: %d, 待跑: %d\n\n', total_runs, total_runs - done_runs);

    t0 = tic;
    run_count = done_runs;

    for si = 1:n_seeds
        seed = si;
        if all(~isnan(rmse_mat(:, :, si)))
            fprintf('Seed %d: 全部跳过\n', seed);
            continue;
        end
        fprintf('>>> Seed %d (%d/%d)\n', seed, si, n_seeds);

        for sci = 1:n_sc
            sname = scenarios{sci};
            for ci = 1:n_cand
                if ~isnan(rmse_mat(sci, ci, si))
                    continue;
                end
                cand = candidates_deg(ci);
                params = simulation_params_oracle();
                params.random_seed = seed;
                params.imm_turn_rate_rad_per_sec = cand * pi / 180.0;
                try
                    r = run(sname, params);
                    rmse_mat(sci, ci, si) = r.errorStats_R1.overall.ukf.rms;
                catch
                    rmse_mat(sci, ci, si) = NaN;
                end
                run_count = run_count + 1;
                save(save_file, 'rmse_mat', 'candidates_deg', 'scenarios', ...
                    'true_rate', 'seeds', '-v7');
                if mod(run_count, 20) == 0 || (ci==n_cand && sci==n_sc && si==n_seeds)
                    elapsed = toc(t0);
                    rate = run_count / max(elapsed, 1);
                    remaining = (total_runs - run_count) / max(rate, 1e-6);
                    fprintf('  [%d/%d] elapsed=%.0fs ETA=%.0fs (%.1fh)\n', ...
                        run_count, total_runs, elapsed, remaining, remaining/3600);
                end
            end
        end
        fprintf('  [Seed %d 全部完成]\n\n', seed);
    end

    sweep_summary_final(rmse_mat, candidates_deg, scenarios, true_rate, seeds);
end


function sweep_summary_final(rmse_mat, candidates_deg, scenarios, true_rate, seeds)
    n_sc = size(rmse_mat, 1);
    n_cand = size(rmse_mat, 2);
    n_seeds = size(rmse_mat, 3);

    fprintf('\n========================================\n');
    fprintf('多种子 MC 汇总 (mean ± std RMSE, km)\n');
    fprintf('========================================\n\n');

    fprintf('%-18s', '场景 \\ 速率');
    for ci = 1:n_cand
        fprintf(' | %4.1f°/s ', candidates_deg(ci));
    end
    fprintf(' | 默认 | Oracle | MM上界\n');
    fprintf('%s\n', repmat('-', 1, 130));

    default_all = []; oracle_all = []; mm_all = [];

    for sci = 1:n_sc
        fprintf('%-18s', scenarios{sci});
        row_mean = squeeze(mean(rmse_mat(sci, :, :, 3), 'omitnan'));
        row_std = squeeze(std(rmse_mat(sci, :, :, 3), 0, 'omitnan'));
        for ci = 1:n_cand
            fprintf(' | %.2f±%.2f', row_mean(ci), row_std(ci));
        end

        def_idx = find(abs(candidates_deg - 1.0) < 0.01);
        if ~isempty(def_idx)
            d_val = squeeze(rmse_mat(sci, def_idx, :))';
            d_val = d_val(~isnan(d_val));
            if ~isempty(d_val)
                fprintf(' | %.2f±%.2f', mean(d_val), std(d_val));
                default_all = [default_all; d_val];
            else
                fprintf(' |   nan   ');
            end
        else
            fprintf(' |   nan   ');
        end

        orc_idx = find(abs(candidates_deg - true_rate(sci)) < 0.01);
        if ~isempty(orc_idx)
            o_val = squeeze(rmse_mat(sci, orc_idx, :))';
            o_val = o_val(~isnan(o_val));
            if ~isempty(o_val)
                fprintf(' | %.2f±%.2f', mean(o_val), std(o_val));
                oracle_all = [oracle_all; o_val];
            else
                fprintf(' |   nan   ');
            end
        else
            fprintf(' |   nan   ');
        end

        mm_val = squeeze(min(rmse_mat(sci, :, :, 2), 'omitnan'));
        mm_val = mm_val(:)'; mm_val = mm_val(~isnan(mm_val));
        if ~isempty(mm_val)
            fprintf(' | %.2f±%.2f', mean(mm_val), std(mm_val));
            mm_all = [mm_all; mm_val];
        else
            fprintf(' |   nan   ');
        end
        fprintf('\n');
    end

    fprintf('%-18s', 'OVERALL(pooled)');
    pooled_mean = squeeze(mean(rmse_mat, [1 3], 'omitnan'));
    pooled_std = squeeze(std(rmse_mat, 0, [1 3], 'omitnan'));
    for ci = 1:n_cand
        fprintf(' | %.2f±%.2f', pooled_mean(ci), pooled_std(ci));
    end

    if ~isempty(default_all)
        dm = mean(default_all); ds = std(default_all);
        fprintf(' | %.2f±%.2f', dm, ds);
    else
        fprintf(' |   nan   ');
    end
    if ~isempty(oracle_all)
        om = mean(oracle_all); os_ = std(oracle_all);
        fprintf(' | %.2f±%.2f', om, os_);
    else
        fprintf(' |   nan   ');
    end
    if ~isempty(mm_all)
        mm = mean(mm_all); ms = std(mm_all);
        fprintf(' | %.2f±%.2f', mm, ms);
    else
        fprintf(' |   nan   ');
    end
    fprintf('\n\n');

    fprintf('========================================\n');
    fprintf('改善 vs Default (pooled)\n');
    fprintf('========================================\n');
    if ~isempty(default_all)
        d_mean = mean(default_all);
        if ~isempty(oracle_all)
            o_mean = mean(oracle_all);
        else
            o_mean = 0;
        end
        if ~isempty(mm_all)
            m_mean = mean(mm_all);
        else
            m_mean = 0;
        end
        fprintf('  Default(1°/s)   : %.3f km\n', d_mean);
        fprintf('  Oracle(真实)    : %.3f km (改善 %+1.1f%%)\n', o_mean, (d_mean-o_mean)/d_mean*100);
        fprintf('  MM 上界         : %.3f km (改善 %+1.1f%%)\n', m_mean, (d_mean-m_mean)/d_mean*100);
    end
end
