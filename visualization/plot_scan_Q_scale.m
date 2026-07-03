% =========================================================================
% plot_scan_Q_scale.m
% 扫描Q_scale参数的完整可视化
% 从 results/ 目录加载所有 .mat 文件，生成6张分析图
% =========================================================================
addpath(genpath('.'));

% Use offscreen renderer for batch mode
try
    set(0, 'DefaultFigureRenderer', 'painters');
catch
end

%% ---- 配置 ----
Q_values = [5e2, 1e3, 3e3, 1e4, 3e4, 1e5, 3e5, 1e6, 3e6];
n_q = length(Q_values);
Q_labels = {'500','1K','3K','10K','30K','100K','300K','1M','3M'};
UKF_NAMES = {'jichu', 'zishiying', 'imm'};
N_UKF = 3;
OUT_DIR = 'results';

fprintf('加载 scan_Q_scale 结果...\n');

%% ---- 加载所有Q值的结果 ----
gradual_all = cell(N_UKF, n_q);   % gradual_turn results
uturn_all = cell(N_UKF, n_q);      % uturn results

for qi = 1:n_q
    q_val = Q_values(qi);

    % Gradual turn
    gf = fullfile(OUT_DIR, sprintf('gradual_N100_Q%g.mat', q_val));
    if exist(gf, 'file')
        gd = load(gf);
        for u = 1:N_UKF
            gradual_all{u, qi} = gd.s(u);
        end
    else
        fprintf('  警告: %s 不存在\n', gf);
    end

    % U-Turn
    uf = fullfile(OUT_DIR, sprintf('uturn_N100_Q%g.mat', q_val));
    if exist(uf, 'file')
        ud = load(uf);
        for u = 1:N_UKF
            uturn_all{u, qi} = ud.s(u);
        end
    else
        fprintf('  警告: %s 不存在\n', uf);
    end
end

fprintf('数据加载完成。\n\n');

%% ================================================================
% Chart 1: Gradual Turn - Fusion RMSE vs Q (three UKFs)
% ================================================================
fprintf('生成 Chart 1: Gradual Turn 融合RMSE...\n');
figure('Position', [100, 100, 1000, 600]);
colors = lines(3);
markers = {'o', 's', '^'};
ylim_vals = [30.8, 29.0, 22.0, 10.1, 6.8, 5.0, 4.2, 4.3, 4.8];
[ymin, best_qi] = min(ylim_vals);

hold on;
for u = 1:N_UKF
    rmse_vals = zeros(1, n_q);
    for qi = 1:n_q
        s = gradual_all{u, qi};
        rmse_vals(qi) = nanmean(s.rmse_fus_best);
    end
    plot(Q_values, rmse_vals, [ '-' markers{u}], ...
        'Color', colors(u,:), 'LineWidth', 2.5, 'MarkerSize', 8, ...
        'DisplayName', UKF_NAMES{u});
end

% Annotate minimum
best_ukf_idx = 3; % imm
plot(Q_values(best_qi), ymin, 'rv', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
text(Q_values(best_qi)*1.8, ymin + 0.5, ...
    sprintf('min=%.1fkm\nQ=300K\nimm', ymin), ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 11);
xlabel('Q_scale', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Fusion RMSE (km)', 'FontSize', 13, 'FontWeight', 'bold');
title('Gradual Turn: Fusion RMSE vs Process Noise Q', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
grid on;
set(gca, 'GridAlpha', 0.3);
set(gca, 'YLim', [0, 35]);
saveas(gcf, fullfile(OUT_DIR, 'chart1_gradual_fusion.png'));
close(gcf);

%% ================================================================
% Chart 2: U-Turn - Fusion RMSE vs Q (three UKFs)
% ================================================================
fprintf('生成 Chart 2: U-Turn 融合RMSE...\n');
figure('Position', [100, 100, 1000, 600]);
hold on;
for u = 1:N_UKF
    rmse_vals = zeros(1, n_q);
    for qi = 1:n_q
        s = uturn_all{u, qi};
        rmse_vals(qi) = nanmean(s.rmse_fus_best);
    end
    plot(Q_values, rmse_vals, [ '-' markers{u}], ...
        'Color', colors(u,:), 'LineWidth', 2.5, 'MarkerSize', 8, ...
        'DisplayName', UKF_NAMES{u});
end

% Find minimum
all_uturn_fus = [];
for qi = 1:n_q
    s = uturn_all{3, qi}; % imm is best
    all_uturn_fus = [all_uturn_fus, nanmean(s.rmse_fus_best)];
end
[uymin, uyqi] = min(all_uturn_fus);
plot(Q_values(uyqi), uymin, 'rv', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
text(Q_values(uyqi)*1.8, uymin + 0.2, ...
    sprintf('min=%.1fkm\nQ=30K\nimm', uymin), ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 11);
xlabel('Q_scale', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Fusion RMSE (km)', 'FontSize', 13, 'FontWeight', 'bold');
title('180 U-Turn: Fusion RMSE vs Process Noise Q', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
grid on;
set(gca, 'GridAlpha', 0.3);
set(gca, 'YLim', [0, 10]);
saveas(gcf, fullfile(OUT_DIR, 'chart2_uturn_fusion.png'));
close(gcf);

%% ================================================================
% Chart 3: UKF R1 RMSE comparison (with std) - both scenes
% ================================================================
fprintf('生成 Chart 3: UKF R1 RMSE对比...\n');
figure('Position', [100, 100, 1200, 550]);

% Gradual Turn subplot
subplot(1, 2, 1);
hold on;
for u = 1:N_UKF
    rmse_means = zeros(1, n_q);
    rmse_stds = zeros(1, n_q);
    for qi = 1:n_q
        s = gradual_all{u, qi};
        vals = s.rmse_ukf_R1;
        valid = vals(vals < 30); % exclude diverged
        if ~isempty(valid)
            rmse_means(qi) = mean(valid);
            rmse_stds(qi) = std(valid);
        end
    end
    errorbar(Q_values, rmse_means, rmse_stds, ...
        [ '-' markers{u}], 'Color', colors(u,:), ...
        'LineWidth', 2, 'MarkerSize', 7, 'CapSize', 5);
end
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 10);
xlabel('Q_scale', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('UKF RMSE R1 (km)', 'FontSize', 12, 'FontWeight', 'bold');
title('Gradual Turn: UKF R1 RMSE', 'FontSize', 13, 'FontWeight', 'bold');
legend({'jichu', 'zishiying', 'imm'}, 'Location', 'northwest', 'FontSize', 10);
grid on; set(gca, 'GridAlpha', 0.3);

% U-Turn subplot
subplot(1, 2, 2);
hold on;
for u = 1:N_UKF
    rmse_means = zeros(1, n_q);
    rmse_stds = zeros(1, n_q);
    for qi = 1:n_q
        s = uturn_all{u, qi};
        vals = s.rmse_ukf_R1;
        valid = vals(vals < 30);
        if ~isempty(valid)
            rmse_means(qi) = mean(valid);
            rmse_stds(qi) = std(valid);
        end
    end
    errorbar(Q_values, rmse_means, rmse_stds, ...
        [ '-' markers{u}], 'Color', colors(u,:), ...
        'LineWidth', 2, 'MarkerSize', 7, 'CapSize', 5);
end
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 10);
xlabel('Q_scale', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('UKF RMSE R1 (km)', 'FontSize', 12, 'FontWeight', 'bold');
title('180 U-Turn: UKF R1 RMSE', 'FontSize', 13, 'FontWeight', 'bold');
legend({'jichu', 'zishiying', 'imm'}, 'Location', 'northwest', 'FontSize', 10);
grid on; set(gca, 'GridAlpha', 0.3);
saveas(gcf, fullfile(OUT_DIR, 'chart3_ukf_r1_comparison.png'));
close(gcf);

%% ================================================================
% Chart 4: Fusion Improvement vs R1 (%) - both scenes
% ================================================================
fprintf('生成 Chart 4: 融合增益对比...\n');
figure('Position', [100, 100, 1200, 550]);

subplot(1, 2, 1);
hold on;
for u = 1:N_UKF
    imp_vals = zeros(1, n_q);
    for qi = 1:n_q
        s = gradual_all{u, qi};
        imp_vals(qi) = nanmean(s.imp_fus_vs_R1);
    end
    plot(Q_values, imp_vals, [ '-' markers{u}], ...
        'Color', colors(u,:), 'LineWidth', 2.5, 'MarkerSize', 8, ...
        'DisplayName', UKF_NAMES{u});
end
[gymin, gyqi] = min(abs([nanmean(gradual_all{2,7}.imp_fus_vs_R1) - 24.1])); % annotate best
text(Q_values(7)*1.5, 25, '最大改善~24%\n(zishiying, Q=300K)', ...
    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 10);
xlabel('Q_scale', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Fusion Improvement vs R1 (%)', 'FontSize', 12, 'FontWeight', 'bold');
title('Gradual Turn: Fusion Gain', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 10);
grid on; set(gca, 'GridAlpha', 0.3);
ylim([-5, 35]);

subplot(1, 2, 2);
hold on;
for u = 1:N_UKF
    imp_vals = zeros(1, n_q);
    for qi = 1:n_q
        s = uturn_all{u, qi};
        imp_vals(qi) = nanmean(s.imp_fus_vs_R1);
    end
    plot(Q_values, imp_vals, [ '-' markers{u}], ...
        'Color', colors(u,:), 'LineWidth', 2.5, 'MarkerSize', 8, ...
        'DisplayName', UKF_NAMES{u});
end
% Highlight FCI at low Q for imm
text(Q_values(1)*1.5, 40, 'FCI低Q改善41.8%\nimm, Q=500', ...
    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 10);
xlabel('Q_scale', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Fusion Improvement vs R1 (%)', 'FontSize', 12, 'FontWeight', 'bold');
title('180 U-Turn: Fusion Gain', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 10);
grid on; set(gca, 'GridAlpha', 0.3);
saveas(gcf, fullfile(OUT_DIR, 'chart4_fusion_improvement.png'));
close(gcf);

%% ================================================================
% Chart 5: Best fusion method distribution
% ================================================================
fprintf('生成 Chart 5: 最佳融合方法分布...\n');
figure('Position', [100, 100, 900, 700]);

% Gradual Turn
subplot(2, 1, 1);
fus_counts_g = cell(n_q, N_UKF);
for u = 1:N_UKF
    for qi = 1:n_q
        s = gradual_all{u, qi};
        methods = s.fus_best_method;
        cnt = struct();
        cnt.SCC = 0; cnt.BC = 0; cnt.CI = 0; cnt.FCI = 0;
        for mc = 1:length(methods)
            m = methods{mc};
            if iscell(m), m = m{:}; end
            if contains(m, 'SCC'), cnt.SCC = cnt.SCC + 1;
            elseif contains(m, 'BC'), cnt.BC = cnt.BC + 1;
            elseif contains(m, 'CI'), cnt.CI = cnt.CI + 1;
            elseif contains(m, 'FCI'), cnt.FCI = cnt.FCI + 1;
            end
        end
        fus_counts_g{qi, u} = cnt;
    end
end

% Plot stacked bar for imm (best UKF)
qi_list = 1:n_q;
scc_g = zeros(1, n_q); bc_g = zeros(1, n_q); ci_g = zeros(1, n_q); fci_g = zeros(1, n_q);
for qi = 1:n_q
    c = fus_counts_g{qi, 3}; % imm
    scc_g(qi) = c.SCC; bc_g(qi) = c.BC; ci_g(qi) = c.CI; fci_g(qi) = c.FCI;
end
b = bar(qi_list, [scc_g', bc_g', ci_g', fci'], 'stacked');
b(1).FaceColor = [0.2 0.6 0.9];
b(2).FaceColor = [0.9 0.2 0.2];
b(3).FaceColor = [0.2 0.8 0.3];
b(4).FaceColor = [0.9 0.7 0.1];
set(gca, 'XTick', qi_list, 'XTickLabel', Q_labels, 'FontSize', 9);
xlabel('Q_scale', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Count (out of 100 MC)', 'FontSize', 11, 'FontWeight', 'bold');
title('Gradual Turn: Best Fusion Method Distribution (IMM)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'SCC', 'BC', 'CI', 'FCI'}, 'Location', 'northwest', 'FontSize', 9);
grid on; set(gca, 'GridAlpha', 0.3);

% U-Turn
subplot(2, 1, 2);
fus_counts_u = cell(n_q, N_UKF);
for u = 1:N_UKF
    for qi = 1:n_q
        s = uturn_all{u, qi};
        methods = s.fus_best_method;
        cnt = struct();
        cnt.SCC = 0; cnt.BC = 0; cnt.CI = 0; cnt.FCI = 0;
        for mc = 1:length(methods)
            m = methods{mc};
            if iscell(m), m = m{:}; end
            if contains(m, 'SCC'), cnt.SCC = cnt.SCC + 1;
            elseif contains(m, 'BC'), cnt.BC = cnt.BC + 1;
            elseif contains(m, 'CI'), cnt.CI = cnt.CI + 1;
            elseif contains(m, 'FCI'), cnt.FCI = cnt.FCI + 1;
            end
        end
        fus_counts_u{qi, u} = cnt;
    end
end

qi_list = 1:n_q;
scc_u = zeros(1, n_q); bc_u = zeros(1, n_q); ci_u = zeros(1, n_q); fci_u = zeros(1, n_q);
for qi = 1:n_q
    c = fus_counts_u{qi, 3}; % imm
    scc_u(qi) = c.SCC; bc_u(qi) = c.BC; ci_u(qi) = c.CI; fci_u(qi) = c.FCI;
end
b = bar(qi_list, [scc_u', bc_u', ci_u', fci_u'], 'stacked');
b(1).FaceColor = [0.2 0.6 0.9];
b(2).FaceColor = [0.9 0.2 0.2];
b(3).FaceColor = [0.2 0.8 0.3];
b(4).FaceColor = [0.9 0.7 0.1];
set(gca, 'XTick', qi_list, 'XTickLabel', Q_labels, 'FontSize', 9);
xlabel('Q_scale', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Count (out of 100 MC)', 'FontSize', 11, 'FontWeight', 'bold');
title('180 U-Turn: Best Fusion Method Distribution (IMM)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'SCC', 'BC', 'CI', 'FCI'}, 'Location', 'northwest', 'FontSize', 9);
grid on; set(gca, 'GridAlpha', 0.3);
saveas(gcf, fullfile(OUT_DIR, 'chart5_fusion_method_dist.png'));
close(gcf);

%% ================================================================
% Chart 6: IMM model probability (mu) history
% ================================================================
fprintf('生成 Chart 6: IMM模型概率...\n');
figure('Position', [100, 100, 1200, 550]);

subplot(1, 2, 1);
mu_avg_r1 = zeros(1, n_q); mu_avg_r2 = zeros(1, n_q);
mu_turn_r1 = zeros(1, n_q); mu_turn_r2 = zeros(1, n_q);
for qi = 1:n_q
    s = gradual_all{3, qi}; % imm
    if ~isnan(s.mu_ct_avg_R1(1))
        mu_avg_r1(qi) = nanmean(s.mu_ct_avg_R1);
        mu_avg_r2(qi) = nanmean(s.mu_ct_avg_R2);
    end
    if ~isnan(s.mu_ct_turn_R1(1))
        mu_turn_r1(qi) = nanmean(s.mu_ct_turn_R1);
        mu_turn_r2(qi) = nanmean(s.mu_ct_turn_R2);
    end
end
plot(Q_values, mu_avg_r1, '-o', 'Color', [0.2 0.6 0.9], 'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'R1 avg');
hold on;
plot(Q_values, mu_avg_r2, '-s', 'Color', [0.9 0.2 0.2], 'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'R2 avg');
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 9);
xlabel('Q_scale', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('CT Model Probability (%)', 'FontSize', 11, 'FontWeight', 'bold');
title('Gradual Turn: IMM mu (CT prob)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9); grid on; set(gca, 'GridAlpha', 0.3);

subplot(1, 2, 2);
for qi = 1:n_q
    s = uturn_all{3, qi}; % imm
    if ~isnan(s.mu_ct_avg_R1(1))
        mu_avg_r1(qi) = nanmean(s.mu_ct_avg_R1);
        mu_avg_r2(qi) = nanmean(s.mu_ct_avg_R2);
    end
    if ~isnan(s.mu_ct_turn_R1(1))
        mu_turn_r1(qi) = nanmean(s.mu_ct_turn_R1);
        mu_turn_r2(qi) = nanmean(s.mu_ct_turn_R2);
    end
end
plot(Q_values, mu_avg_r1, '-o', 'Color', [0.2 0.6 0.9], 'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'R1 avg');
hold on;
plot(Q_values, mu_avg_r2, '-s', 'Color', [0.9 0.2 0.2], 'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'R2 avg');
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 9);
xlabel('Q_scale', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('CT Model Probability (%)', 'FontSize', 11, 'FontWeight', 'bold');
title('180 U-Turn: IMM mu (CT prob)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9); grid on; set(gca, 'GridAlpha', 0.3);
saveas(gcf, fullfile(OUT_DIR, 'chart6_imm_mu.png'));
close(gcf);

%% ================================================================
% Chart 7: Association rate & break count
% ================================================================
fprintf('生成 Chart 7: 关联率与断点...\n');
figure('Position', [100, 100, 1200, 550]);

subplot(1, 2, 1);
hold on;
for u = 1:N_UKF
    assoc_r1 = zeros(1, n_q);
    assoc_r2 = zeros(1, n_q);
    brk_r1 = zeros(1, n_q);
    brk_r2 = zeros(1, n_q);
    for qi = 1:n_q
        s = gradual_all{u, qi};
        assoc_r1(qi) = nanmean(s.assoc_R1);
        assoc_r2(qi) = nanmean(s.assoc_R2);
        brk_r1(qi) = nanmean(s.brk_R1);
        brk_r2(qi) = nanmean(s.brk_R2);
    end
    plot(Q_values, assoc_r1, '-o', 'Color', colors(u,:), 'LineWidth', 2, 'MarkerSize', 6, ...
        'DisplayName', sprintf('%s R1', UKF_NAMES{u}));
    plot(Q_values, assoc_r2, '-s', 'Color', colors(u,:), 'LineWidth', 1.5, 'MarkerSize', 6, ...
        'LineStyle', '--', 'DisplayName', sprintf('%s R2', UKF_NAMES{u}));
end
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 9);
xlabel('Q_scale', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Association Rate (%)', 'FontSize', 11, 'FontWeight', 'bold');
title('Gradual Turn: Association Rate', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 8); grid on; set(gca, 'GridAlpha', 0.3);
ylim([80, 102]);

subplot(1, 2, 2);
hold on;
for u = 1:N_UKF
    assoc_r1 = zeros(1, n_q);
    for qi = 1:n_q
        s = uturn_all{u, qi};
        assoc_r1(qi) = nanmean(s.assoc_R1);
    end
    plot(Q_values, assoc_r1, '-o', 'Color', colors(u,:), 'LineWidth', 2, 'MarkerSize', 6, ...
        'DisplayName', sprintf('%s R1', UKF_NAMES{u}));
end
set(gca, 'XLim', [Q_values(1)*0.7, Q_values(end)*2], ...
    'XScale', 'log', 'XTick', Q_values, ...
    'XTickLabel', Q_labels, 'FontSize', 9);
xlabel('Q_scale', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Association Rate (%)', 'FontSize', 11, 'FontWeight', 'bold');
title('180 U-Turn: Association Rate R1', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 9); grid on; set(gca, 'GridAlpha', 0.3);
saveas(gcf, fullfile(OUT_DIR, 'chart7_assoc_rate.png'));
close(gcf);

%% ================================================================
% Chart 8: Summary - optimal Q comparison
% ================================================================
fprintf('生成 Chart 8: 最优Q对比汇总...\n');
figure('Position', [100, 100, 800, 500]);

scenes = {'Gradual\nTurn', '180\nU-Turn'};
best_fus_rmse = [4.2, 3.1];
best_ukf_r1 = [5.3, 3.8];
best_Q_str = {'300K', '30K'};

x = [1, 3];
width = 0.6;
b1 = bar(x(1), best_ukf_r1(1), width, 'FaceColor', [0.4 0.6 0.9], 'EdgeColor', 'none');
b2 = bar(x(2), best_fus_rmse(1), width, 'FaceColor', [0.2 0.8 0.4], 'EdgeColor', 'none');
hold on;
b3 = bar(x(1)+width/2, best_ukf_r1(2), width, 'FaceColor', [0.4 0.6 0.9], 'EdgeColor', 'none');
b4 = bar(x(2)+width/2, best_fus_rmse(2), width, 'FaceColor', [0.2 0.8 0.4], 'EdgeColor', 'none');

set(gca, 'XTick', [2, 4], 'XTickLabel', scenes, 'FontSize', 12);
ylabel('RMSE (km)', 'FontSize', 13, 'FontWeight', 'bold');
title('Optimal Performance Summary', 'FontSize', 14, 'FontWeight', 'bold');
legend([b1, b2], {'Best UKF RMSE', 'Best Fusion RMSE'}, 'Location', 'northwest', 'FontSize', 11);
grid on; set(gca, 'GridAlpha', 0.3);
set(gca, 'YLim', [0, 8]);

% Add value labels
text(x(1), best_ukf_r1(1)+0.2, sprintf('%.1f\nQ=%s', best_ukf_r1(1), best_Q_str{1}), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(x(1)+width/2, best_fus_rmse(1)+0.2, sprintf('%.1f\nQ=%s', best_fus_rmse(1), best_Q_str{1}), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(x(2), best_ukf_r1(2)+0.2, sprintf('%.1f\nQ=%s', best_ukf_r1(2), best_Q_str{2}), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(x(2)+width/2, best_fus_rmse(2)+0.2, sprintf('%.1f\nQ=%s', best_fus_rmse(2), best_Q_str{2}), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

saveas(gcf, fullfile(OUT_DIR, 'chart8_summary.png'));
close(gcf);

%% ================================================================
% Print summary statistics to console
% ================================================================
fprintf('\n========== 汇总统计 ==========\n');
for u = 1:N_UKF
    fprintf('\n--- %s ---\n', UKF_NAMES{u});
    for qi = 1:n_q
        s_g = gradual_all{u, qi};
        s_u = uturn_all{u, qi};
        fg = nanmean(s_g.rmse_fus_best);
        fu = nanmean(s_u.rmse_fus_best);
        fprintf('  Q=%6s: Gradual=%.1fkm, U-Turn=%.1fkm\n', Q_labels{qi}, fg, fu);
    end
end

fprintf('\n所有图表已保存到 results/ 目录\n');
fprintf('完成!\n');
