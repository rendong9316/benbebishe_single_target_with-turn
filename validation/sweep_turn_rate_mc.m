function sweep_turn_rate_mc(seeds)
    % turn_rate 扫描 - 多种子蒙特卡洛版（增量保存 + 断点续跑）
    %
    % 用法: sweep_turn_rate_mc(1:50)
    %
    % 特性:
    %   - 增量保存: 每完成一个 run 立即 save
    %   - 断点续跑: 启动时 load 已有结果，跳过非 NaN 的条目
    %   - 小批量模式: 每批最多 8 次 run (避免 heap corruption)

    if nargin < 1 || isempty(seeds)
        seeds = 1:50;
    end

    addpath(genpath(pwd));

    candidates_deg = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0];
    scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};
    true_rate = [0.7, 0.7, 1.3, 1.3];
    n_cand = length(candidates_deg);
    n_sc = length(scenarios);
    n_seeds = length(seeds);
    save_file = 'results/sweep_turn_rate_mc50.mat';
    batch_size = 15;  % 每批任务数 (now safe with sweep_mode)

    % ---- 读取已有进度 ----
    done_map = containers.Map('KeyType', 'double', 'ValueType', 'any');
    if exist(save_file, 'file')
        try
            S = load(save_file);
            for si = 1:length(S.seeds)
                for sci = 1:size(S.rmse_mat,1)
                    for ci = 1:size(S.rmse_mat,2)
                        if ~isnan(S.rmse_mat(sci,ci,si))
                            key = seeds(si)*100 + sci*10 + ci;
                            done_map(key) = S.rmse_mat(sci,ci,si);
                        end
                    end
                end
            end
            fprintf('断点续跑: 已完成 %d/%d (%.1f%%)\n', ...
                length(done_map), n_seeds*n_sc*n_cand, length(done_map)/(n_seeds*n_sc*n_cand)*100);
        catch ME
            fprintf('警告: 加载已有结果失败 (%s)，从头开始。\n', getReport(ME));
        end
    end

    % ---- 生成所有任务列表 ----
    all_tasks = {};
    tidx = 0;
    for si = 1:n_seeds
        for sci = 1:n_sc
            for ci = 1:n_cand
                tidx = tidx + 1;
                seed = seeds(si);
                key = seed*100 + sci*10 + ci;
                all_tasks{tidx} = struct('seed', seed, 'sci', sci, 'ci', ci, 'key', key);
            end
        end
    end

    % ---- 筛选未完成的任务 ----
    pending = {};
    for ti = 1:tidx
        if ~isKey(done_map, all_tasks{ti}.key)
            pending{end+1} = all_tasks{ti};
        end
    end

    total_pending = length(pending);
    fprintf('待跑 run: %d/%d\n\n', total_pending, n_seeds*n_sc*n_cand);

    if total_pending == 0
        fprintf('所有任务已完成！\n');
        sweep_summary_from_save(save_file, candidates_deg, scenarios, true_rate, seeds);
        return;
    end

    n_batches = ceil(total_pending / batch_size);
    fprintf('分为 %d 批运行 (batch_size=%d)\n\n', n_batches, batch_size);

    t0 = tic;

    for b = 1:n_batches
        batch_start = (b-1)*batch_size + 1;
        batch_end = min(b*batch_size, total_pending);

        fprintf('[Batch %d/%d] tasks %d..%d\n', b, n_batches, batch_start, batch_end);

        for bi = batch_start:batch_end
            t = pending{bi};
            cand = candidates_deg(t.ci);

            params = simulation_params_oracle();
            params.random_seed = t.seed;
            params.imm_turn_rate_rad_per_sec = cand * pi / 180.0;

            try
                r = run(scenarios{t.sci}, params);
                val = r.errorStats_R1.overall.ukf.rms;
                done_map(t.key) = val;
                rebuild_and_save(save_file, done_map, candidates_deg, scenarios, true_rate, seeds, n_seeds);
            catch ME
                fprintf('  ERROR seed=%d %s rate=%.1f: %.60s\n', t.seed, scenarios{t.sci}, cand, getReport(ME));
                continue;
            end

            elapsed = toc(t0);
            overall_done = b*batch_size - batch_start + bi - (batch_start-1) + sum(b==1);
            rate = overall_done / max(elapsed, 1);
            remaining = (total_pending - overall_done) / max(rate, 1e-6);
            fprintf('  [%d/%d] seed=%d %-20s rate=%.1f RMSE=%.3f | elapsed=%.0fs ETA=%.0fs (%.1fh)\n', ...
                overall_done, total_pending, t.seed, scenarios{t.sci}, cand, val, elapsed, remaining, remaining/3600);
        end
    end

    % ---- 最终汇总 ----
    sweep_summary_from_save(save_file, candidates_deg, scenarios, true_rate, seeds);
end


function rebuild_and_save(save_file, done_map, candidates_deg, scenarios, true_rate, seeds, n_seeds)
    n_sc = length(scenarios);
    n_cand = length(candidates_deg);
    rmse_mat = NaN(n_sc, n_cand, n_seeds);
    for k = 1:length(done_map)
        if isKey(done_map, k)
            val = done_map(k);
            ci = mod(k, 10);
            sci = mod(floor(k/10), 10);
            seed = floor(k/100);
            si = find(seeds == seed, 1);
            if ~isempty(si) && sci >= 1 && sci <= n_sc && ci >= 1 && ci <= n_cand
                rmse_mat(sci, ci, si) = val;
            end
        end
    end
    try
        save(save_file, 'rmse_mat', 'candidates_deg', 'scenarios', 'true_rate', 'seeds', '-v7');
    catch
    end
end


function sweep_summary_from_save(save_file, candidates_deg, scenarios, true_rate, seeds)
    S = load(save_file);
    rmse_mat = S.rmse_mat;
    n_sc = size(rmse_mat, 1);
    n_cand = size(rmse_mat, 2);

    fprintf('\n============================================================\n');
    fprintf('多种子 MC 汇总 (mean +/- std RMSE, km, n=%d seeds)\n', length(seeds));
    fprintf('============================================================\n');

    fprintf('%-18s', 'Scenario \ rate');
    for ci = 1:n_cand
        fprintf(' |   %4.1f deg/s   ', candidates_deg(ci));
    end
    fprintf(' |  Default  |  Oracle   |  MM-upper\n');
    fprintf('%s\n', repmat('-', 1, 110));

    default_all = []; oracle_all = []; mm_all = [];

    for sci = 1:n_sc
        fprintf('%-18s', scenarios{sci});
        row_mean = mean(rmse_mat(sci, :, :), 3, 'omitnan');
        row_std = std(rmse_mat(sci, :, :), 0, 3, 'omitnan');
        for ci = 1:n_cand
            fprintf(' | %5.2f±%.2f', row_mean(ci), row_std(ci));
        end

        default_idx = find(abs(candidates_deg - 1.0) < 1e-6);
        oracle_idx = find(abs(candidates_deg - true_rate(sci)) < 1e-6);

        d_vals = squeeze(rmse_mat(sci, default_idx, :))';
        o_vals = squeeze(rmse_mat(sci, oracle_idx, :))';

        mm_vals_per_seed = min(rmse_mat(sci, :, :), [], 2);
        mm_vals = squeeze(mm_vals_per_seed)';

        d_vals = d_vals(~isnan(d_vals));
        o_vals = o_vals(~isnan(o_vals));
        mm_vals = mm_vals(~isnan(mm_vals));

        fprintf(' | %5.2f±%.2f', mean(d_vals), std(d_vals));
        fprintf(' | %5.2f±%.2f', mean(o_vals), std(o_vals));
        fprintf(' | %5.2f±%.2f', mean(mm_vals), std(mm_vals));
        fprintf('\n');

        default_all = [default_all; d_vals];
        oracle_all = [oracle_all; o_vals];
        mm_all = [mm_all; mm_vals];
    end

    fprintf('%-18s', 'OVERALL(pooled)');
    pooled_mean = mean(rmse_mat, [1 3], 'omitnan');
    pooled_std = std(rmse_mat, 0, [1 3], 'omitnan');
    for ci = 1:n_cand
        fprintf(' | %5.2f±%.2f', pooled_mean(ci), pooled_std(ci));
    end
    fprintf(' | %5.2f±%.2f', mean(default_all), std(default_all));
    fprintf(' | %5.2f±%.2f', mean(oracle_all), std(oracle_all));
    fprintf(' | %5.2f±%.2f', mean(mm_all), std(mm_all));
    fprintf('\n');

    fprintf('\n============================================================\n');
    fprintf('改善 vs Default (pooled, %d seeds x %d scenes)\n', length(seeds), n_sc);
    fprintf('============================================================\n');
    d_mean = mean(default_all);
    o_mean = mean(oracle_all);
    m_mean = mean(mm_all);
    fprintf('  Default(1 deg/s) : %.3f km  (baseline)\n', d_mean);
    fprintf('  Oracle(真实)  : %.3f km  (improve %+.1f%%)\n', o_mean, (d_mean-o_mean)/d_mean*100);
    fprintf('  MM-upper(min) : %.3f km  (improve %+.1f%%)\n', m_mean, (d_mean-m_mean)/d_mean*100);

    fprintf('\n============================================================\n');
    fprintf('配对 t 检验 (H1: improve > 0)\n');
    fprintf('============================================================\n');
    n_pair = length(default_all);
    diff_oracle = default_all - oracle_all;
    diff_mm = default_all - mm_all;
    m_o = mean(diff_oracle); s_o = std(diff_oracle);
    m_m = mean(diff_mm); s_m = std(diff_mm);
    t_o = m_o / (s_o/sqrt(n_pair));
    t_m = m_m / (s_m/sqrt(n_pair));
    fprintf('  Oracle vs Default: t=%+.2f, df=%d, diff=%.3f±%.3f km\n', t_o, n_pair-1, m_o, s_o);
    fprintf('  MM-upper vs Default: t=%+.2f, df=%d, diff=%.3f±%.3f km\n', t_m, n_pair-1, m_m, s_m);
    fprintf('  (t > 1.65 => p<0.05 one-sided, t > 2.33 => p<0.01)\n');
end
