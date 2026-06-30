% analyze_mc_results.m — MC结果快速分析脚本
% 用法: analyze_mc_results('results/mc_turn_XXXX.mat')
function analyze_mc_results(matfile)
    if nargin < 1
        % 自动找最新的结果文件
        d = dir('results/mc_turn_*.mat');
        if isempty(d)
            error('No mc_turn_*.mat found in results/');
        end
        [~, idx] = max([d.datenum]);
        matfile = fullfile('results', d(idx).name);
    end
    fprintf('Loading: %s\n', matfile);
    load(matfile);

    fprintf('\n========== %d-MC 结果汇总 ==========\n', N_MC);

    % ==== A. RMSE ====
    fprintf('\n--- A. RMSE (km) ---\n');
    fprintf('%-22s %7s %7s %7s %7s %7s\n', '指标', '均值', 'std', '中位', '最小', '最大');
    fprintf('%-22s %7s %7s %7s %7s %7s\n', '---', '---', '---', '---', '---', '---');

    fields_rmse = {
        '原始点迹 R1', rmse.raw_R1;
        '原始点迹 R2', rmse.raw_R2;
        '校准后 R1', rmse.cal_R1;
        '校准后 R2', rmse.cal_R2;
        'UKF R1', rmse.ukf_R1;
        'UKF R2(对齐)', rmse.ukf_R2_aligned;
        '融合 最优', rmse.fus_best;
    };
    for i = 1:size(fields_rmse,1)
        print_row(fields_rmse{i,1}, fields_rmse{i,2});
    end

    % ==== B. 改善率 ====
    fprintf('\n--- B. 改善率 (%%) ---\n');
    fprintf('%-22s %7s %7s %7s %7s %7s\n', '指标', '均值', 'std', '中位', '最小', '最大');
    fprintf('%-22s %7s %7s %7s %7s %7s\n', '---', '---', '---', '---', '---', '---');

    cal_imp_R1 = (1 - rmse.cal_R1 ./ rmse.raw_R1) * 100;
    cal_imp_R2 = (1 - rmse.cal_R2 ./ rmse.raw_R2) * 100;
    print_row('校准改善 R1', cal_imp_R1);
    print_row('校准改善 R2', cal_imp_R2);
    print_row('UKF改善 R1', imp_ukf_R1);
    print_row('UKF改善 R2', imp_ukf_R2);
    print_row('融合 vs R1', imp_fus_vs_R1);
    print_row('融合 vs R2', imp_fus_vs_R2);

    % ==== C. MTL + 断裂 ====
    fprintf('\n--- C. MTL (帧) ---\n');
    print_row('MTL R1', mtl_R1);
    print_row('MTL R2', mtl_R2);
    print_row('MTL 融合', mtl_fus);

    fprintf('\n--- D. 断裂次数 ---\n');
    print_row('断裂 R1', brk_R1);
    print_row('断裂 R2', brk_R2);
    print_row('断裂 融合', brk_fus);

    % ==== E. CT模型概率 ====
    fprintf('\n--- E. CT模型概率 (%%) ---\n');
    print_row('CT均值 R1(%)', mu_ct_avg_R1);
    print_row('CT均值 R2(%)', mu_ct_avg_R2);
    print_row('CT转弯 R1(%)', mu_ct_turn_R1);
    print_row('CT转弯 R2(%)', mu_ct_turn_R2);
    print_row('CT占优帧 R1', mu_ct_dom_R1);
    print_row('CT占优帧 R2', mu_ct_dom_R2);

    % ==== F. 关联 + NIS ====
    fprintf('\n--- F. 关联诊断 ---\n');
    print_row('关联率 R1(%)', assoc_R1);
    print_row('关联率 R2(%)', assoc_R2);
    print_row('NIS均值 R1', nis_mean_R1);
    print_row('NIS均值 R2', nis_mean_R2);
    print_row('NIS门内 R1(%)', nis_gate_R1);
    print_row('NIS门内 R2(%)', nis_gate_R2);
    print_row('起始帧号 R1', init_frame_R1);
    print_row('起始帧号 R2', init_frame_R2);

    % ==== G. 坏种子 ====
    n_bad = sum(bad_seed);
    fprintf('\n--- G. 坏种子: %d/%d (%.1f%%) ---\n', n_bad, N_MC, n_bad/N_MC*100);
    if n_bad > 0 && n_bad <= 30
        for mc = 1:N_MC
            if bad_seed(mc)
                fprintf('  seed=%d: %s\n', SEED_BASE+mc-1, bad_reason{mc});
            end
        end
    end

    % ==== H. 融合算法分布 ====
    fprintf('\n--- H. 最优融合算法分布 ---\n');
    methods = {'SCC', 'BC', 'CI', 'FCI'};
    for m = 1:length(methods)
        cnt = sum(strcmp(fus_best_method, methods{m}));
        fprintf('  %s: %d/%d (%.0f%%)\n', methods{m}, cnt, N_MC, cnt/N_MC*100);
    end

    % ==== I. 综合评分 ====
    fprintf('\n--- I. 综合评分 ---\n');
    % 评分：RMSE越低越好，CT概率越高越好，关联率越高越好
    score.rmse_fus = nanmean(rmse.fus_best);
    score.rmse_r1 = nanmean(rmse.ukf_R1);
    score.ct_turn_r1 = nanmean(mu_ct_turn_R1);
    score.ct_turn_r2 = nanmean(mu_ct_turn_R2);
    score.assoc_r1 = nanmean(assoc_R1);
    score.assoc_r2 = nanmean(assoc_R2);
    score.mtl_fus = nanmean(mtl_fus);
    score.bad_pct = n_bad / N_MC * 100;
    score.imp_fus = nanmean(imp_fus_vs_R1);

    fprintf('  融合RMSE:     %.1f km\n', score.rmse_fus);
    fprintf('  R1 UKF RMSE:  %.1f km\n', score.rmse_r1);
    fprintf('  融合改善:     %+.1f%% vs R1\n', score.imp_fus);
    fprintf('  CT转弯 R1:    %.0f%%\n', score.ct_turn_r1);
    fprintf('  CT转弯 R2:    %.0f%%\n', score.ct_turn_r2);
    fprintf('  关联率 R1:    %.0f%%\n', score.assoc_r1);
    fprintf('  关联率 R2:    %.0f%%\n', score.assoc_r2);
    fprintf('  MTL融合:      %.0f帧\n', score.mtl_fus);
    fprintf('  坏种子率:     %.1f%%\n', score.bad_pct);

    fprintf('\nDone.\n');
end

function print_row(label, vals)
    v = vals(~isnan(vals) & ~isinf(vals));
    if isempty(v)
        fprintf('%-22s %7s %7s %7s %7s %7s\n', label, 'NaN', 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('%-22s %7.1f %7.1f %7.1f %7.1f %7.1f\n', ...
            label, mean(v), std(v), median(v), min(v), max(v));
    end
end
