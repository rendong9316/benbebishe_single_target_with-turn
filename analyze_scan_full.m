% analyze_scan_full.m - analyze run_scenario_scan_full.m results
addpath(genpath('.'));
S = load('results/scan_full.mat');
results = S.results;
seeds = S.seeds;
n_scenarios = S.n_scenarios;
n_seeds = S.n_seeds;
scenario_names = {'strong_cross', 'parallel_sep', 'converge', 'speed_diff', 'cross_180'};

fprintf('=================================================================\n');
fprintf('                  SCAN FULL RESULTS ANALYSIS                     \n');
fprintf('=================================================================\n');
fprintf('Scenarios: %d, Seeds: %d, Total runs: %d\n\n', n_scenarios, n_seeds, n_scenarios*n_seeds);

fprintf('================== Table 1: Per-Scenario Overview ==================\n');
fprintf('Scenario       | Succ  | R1 RMSE T1/T2/T3 (km)    | R2 RMSE T1/T2/T3 (km)    | MatchPairs | FusionRMSE | MinRatio\n');
fprintf('%s\n', repmat('-', 1, 120));

for sc = 1:n_scenarios
    succ = 0;
    r1_acc = zeros(1,3); r2_acc = zeros(1,3);
    r1_cnt = zeros(1,3); r2_cnt = zeros(1,3);
    fusion_sum = 0; fusion_n = 0;
    pair_sum = 0; pair_n = 0;
    min_ratio = inf;
    for sd = 1:n_seeds
        m = results{sc, sd};
        if ~isstruct(m) || ~isfield(m,'success') || ~m.success
            continue;
        end
        succ = succ + 1;
        for ac = 1:3
            if ~isnan(m.all_r1_rmse(ac))
                r1_acc(ac) = r1_acc(ac) + m.all_r1_rmse(ac); r1_cnt(ac) = r1_cnt(ac) + 1;
            end
            if ~isnan(m.all_r2_rmse(ac))
                r2_acc(ac) = r2_acc(ac) + m.all_r2_rmse(ac); r2_cnt(ac) = r2_cnt(ac) + 1;
            end
            min_ratio = min([min_ratio, m.all_r1_ratio(ac), m.all_r2_ratio(ac)]);
        end
        if ~isnan(m.fusion_rmse)
            fusion_sum = fusion_sum + m.fusion_rmse; fusion_n = fusion_n + 1;
        end
        pair_sum = pair_sum + m.n_pairs; pair_n = pair_n + 1;
    end
    if succ == 0
        fprintf('%-14s | ALL FAILED\n', scenario_names{sc});
        continue;
    end
    r1_mean = r1_acc ./ max(1, r1_cnt);
    r2_mean = r2_acc ./ max(1, r2_cnt);
    pair_mean = pair_sum / pair_n;
    fusion_mean = fusion_sum / max(1, fusion_n);
    fprintf('%-14s | %d/%d  | T1=%.1f T2=%.1f T3=%.1f     | T1=%.1f T2=%.1f T3=%.1f     | %.1f        | %.1f        | %.2f\n', ...
        scenario_names{sc}, succ, n_seeds, ...
        r1_mean(1), r1_mean(2), r1_mean(3), ...
        r2_mean(1), r2_mean(2), r2_mean(3), ...
        pair_mean, fusion_mean, min_ratio);
end

fprintf('\n================== Table 2: Association Ratio per Target ==================\n');
fprintf('Target: each track association ratio >= 0.5\n\n');
fprintf('Scenario       | R1 ratios T1/T2/T3       | R2 ratios T1/T2/T3\n');
fprintf('%s\n', repmat('-', 1, 80));
for sc = 1:n_scenarios
    r1_sum = zeros(1,3); r2_sum = zeros(1,3);
    r1_cnt = zeros(1,3); r2_cnt = zeros(1,3);
    for sd = 1:n_seeds
        m = results{sc, sd};
        if ~isstruct(m) || ~isfield(m,'success') || ~m.success
            continue;
        end
        for ac = 1:3
            r1_sum(ac) = r1_sum(ac) + m.all_r1_ratio(ac); r1_cnt(ac) = r1_cnt(ac) + 1;
            r2_sum(ac) = r2_sum(ac) + m.all_r2_ratio(ac); r2_cnt(ac) = r2_cnt(ac) + 1;
        end
    end
    if sum(r1_cnt) == 0
        continue;
    end
    fprintf('%-14s | T1=%.2f T2=%.2f T3=%.2f     | T1=%.2f T2=%.2f T3=%.2f\n', ...
        scenario_names{sc}, ...
        r1_sum(1)/r1_cnt(1), r1_sum(2)/r1_cnt(2), r1_sum(3)/r1_cnt(3), ...
        r2_sum(1)/r2_cnt(1), r2_sum(2)/r2_cnt(2), r2_sum(3)/r2_cnt(3));
end

fprintf('\n================== Table 3: Track Coverage (3 = perfect) ==================\n');
fprintf('Scenario       | R1 tracks (min/max/mean)  | R2 tracks (min/max/mean)  | Pairs (min/max/mean)\n');
fprintf('%s\n', repmat('-', 1, 95));
for sc = 1:n_scenarios
    n1_list = []; n2_list = []; pair_list = [];
    for sd = 1:n_seeds
        m = results{sc, sd};
        if ~isstruct(m) || ~isfield(m,'success') || ~m.success
            continue;
        end
        n1_list(end+1, 1) = m.n1_tracks;
        n2_list(end+1, 1) = m.n2_tracks;
        pair_list(end+1, 1) = m.n_pairs;
    end
    if isempty(n1_list)
        continue;
    end
    fprintf('%-14s | %d/%d/%.1f                  | %d/%d/%.1f                  | %d/%d/%.1f\n', ...
        scenario_names{sc}, ...
        min(n1_list), max(n1_list), mean(n1_list), ...
        min(n2_list), max(n2_list), mean(n2_list), ...
        min(pair_list), max(pair_list), mean(pair_list));
end

fprintf('\n================== Table 4: Per-Seed R1 RMSE (T1) ==================\n');
fprintf('Scenario       |');
for sd = 1:n_seeds
    fprintf(' s=%-4d', seeds(sd));
end
fprintf('\n');
fprintf('%s\n', repmat('-', 1, 14 + n_seeds*8));
for sc = 1:n_scenarios
    fprintf('%-14s |', scenario_names{sc});
    for sd = 1:n_seeds
        m = results{sc, sd};
        if ~isstruct(m) || ~isfield(m,'success') || ~m.success
            fprintf(' %-7s', 'FAIL');
        else
            fprintf(' %-7.1f', m.all_r1_rmse(1));
        end
    end
    fprintf('\n');
end

fprintf('\n================== Table 5: Failed/Abnormal Runs ==================\n');
n_fail = 0;
n_low_ratio = 0;
n_low_pair = 0;
for sc = 1:n_scenarios
    for sd = 1:n_seeds
        m = results{sc, sd};
        if ~isstruct(m) || ~isfield(m,'success') || ~m.success
            fprintf('  [FAIL] %s seed=%d: %s\n', scenario_names{sc}, seeds(sd), m.error);
            n_fail = n_fail + 1;
        else
            if min([m.all_r1_ratio, m.all_r2_ratio]) < 0.5
                fprintf('  [LOW_RATIO] %s seed=%d: R1=[%.2f %.2f %.2f] R2=[%.2f %.2f %.2f]\n', ...
                    scenario_names{sc}, seeds(sd), m.all_r1_ratio(1), m.all_r1_ratio(2), m.all_r1_ratio(3), ...
                    m.all_r2_ratio(1), m.all_r2_ratio(2), m.all_r2_ratio(3));
                n_low_ratio = n_low_ratio + 1;
            end
            if m.n_pairs < 3
                fprintf('  [LOW_PAIR] %s seed=%d: pairs=%d\n', scenario_names{sc}, seeds(sd), m.n_pairs);
                n_low_pair = n_low_pair + 1;
            end
        end
    end
end
fprintf('  Total: %d fails, %d low-ratio runs, %d low-pair runs\n', n_fail, n_low_ratio, n_low_pair);

fprintf('\n================== Global Summary ==================\n');
all_r1 = []; all_r2 = []; all_ratio = []; all_pairs = []; all_fusion = [];
for sc = 1:n_scenarios
    for sd = 1:n_seeds
        m = results{sc, sd};
        if ~isstruct(m) || ~isfield(m,'success') || ~m.success
            continue;
        end
        all_r1 = [all_r1, m.all_r1_rmse];
        all_r2 = [all_r2, m.all_r2_rmse];
        all_ratio = [all_ratio, m.all_r1_ratio, m.all_r2_ratio];
        all_pairs(end+1, 1) = m.n_pairs;
        if ~isnan(m.fusion_rmse)
            all_fusion(end+1, 1) = m.fusion_rmse;
        end
    end
end
fprintf('  R1 RMSE: mean=%.1f km, median=%.1f km, p95=%.1f km, max=%.1f km\n', ...
    mean(all_r1,'omitnan'), median(all_r1,'omitnan'), quantile(all_r1,0.95), max(all_r1,[],'omitnan'));
fprintf('  R2 RMSE: mean=%.1f km, median=%.1f km, p95=%.1f km, max=%.1f km\n', ...
    mean(all_r2,'omitnan'), median(all_r2,'omitnan'), quantile(all_r2,0.95), max(all_r2,[],'omitnan'));
fprintf('  Association ratio: mean=%.2f, min=%.2f, pct runs with all >= 0.5: %.1f pct\n', ...
    mean(all_ratio,'omitnan'), min(all_ratio), 100*mean(all_ratio >= 0.5));
fprintf('  Pairs: mean=%.1f, pct runs with 3 pairs: %.1f pct\n', mean(all_pairs), 100*mean(all_pairs == 3));
fprintf('  Fusion RMSE: mean=%.1f km, max=%.1f km\n', mean(all_fusion,'omitnan'), max(all_fusion,[],'omitnan'));
