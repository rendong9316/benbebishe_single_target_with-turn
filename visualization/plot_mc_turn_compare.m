% =========================================================================
% plot_mc_turn_compare.m
% 蒙特卡洛拐弯仿真对比可视化 — turn180 vs turn46.7
% =========================================================================
addpath(genpath('D:/Desktop/single_target_with-turn'));

%% ---- 加载数据 ----
fprintf('加载 mc_turn180_compare ...\n');
S1 = load('results/mc_turn180_compare_20260701_164750.mat');
fprintf('加载 mc_turn_compare ...\n');
S2 = load('results/mc_turn_compare_20260701_163230.mat');

UKF_NAMES = S1.UKF_NAMES;  % {'jichu','zishiying','imm'}
N_UKF = 3;
N_MC = 200;

% rmse_cal_R1 等是 N_UKF x N_MC 矩阵（每行一个UKF，每列一个seed）
% 取均值
r1_cal_mean = [mean(S1.rmse_cal_R1,2); mean(S2.rmse_cal_R1,2)];  % 2x3
r1_cal_R1_raw = [S1.rmse_cal_R1; S2.rmse_cal_R1];  % 2x200
r2_cal_mean = [mean(S1.rmse_cal_R2,2); mean(S2.rmse_cal_R2,2)];  % 2x3
r2_cal_R1_raw = [S1.rmse_cal_R2; S2.rmse_cal_R2];

raw_R1_mean = [mean(S1.rmse_raw_R1,2); mean(S2.rmse_raw_R1,2)];
raw_R2_mean = [mean(S1.rmse_raw_R2,2); mean(S2.rmse_raw_R2,2)];
cal_R1_mean = r1_cal_mean;
cal_R2_mean = r2_cal_mean;

scenes = {'turn180','turn46.7'};

%% ================================================================
% Chart 1: 三种UKF校准RMSE对比柱状图（R1，双场景并列）
% ================================================================
fprintf('生成 Chart 1: UKF校准RMSE对比 (R1) ...\n');
fig1 = figure('Position',[50 50 1100 500]);
colors = lines(N_UKF);
colors(3,:) = [0 0.45 0.85];  % imm 用蓝色突出
bw = 0.25;

for sc = 1:2
    subplot(1,2,sc);
    hold on; grid on;
    x = 1:N_UKF;
    for u = 1:N_UKF
        vals = r1_cal_mean(sc,u);
        bar(x(u), vals, bw, 'FaceColor', colors(u,:), 'EdgeColor', 'k');
        text(x(u), vals+0.1, sprintf('%.2f',vals), ...
            'HorizontalAlignment','center', 'FontSize',9, 'FontWeight','bold');
    end
    set(gca, 'XTick', x, 'XTickLabel', UKF_NAMES, 'FontSize',10);
    ylabel('R1 Cal RMSE (km)', 'FontSize',11, 'FontWeight','bold');
    title(sprintf('场景: %s', scenes{sc}), 'FontSize',12, 'FontWeight','bold');
    ylim([0 max(r1_cal_mean(sc,:))*1.3]);
end
sgtitle('Chart 1: 三种UKF校准后R1 RMSE对比', 'FontSize',13, 'FontWeight','bold');
saveas(fig1, 'results/chart1_ukf_rmse_comparison.png');
close(fig1);

%% ================================================================
% Chart 2: R2校准RMSE对比
% ================================================================
fprintf('生成 Chart 2: R2校准RMSE对比 ...\n');
fig2 = figure('Position',[50 50 1100 500]);
for sc = 1:2
    subplot(1,2,sc);
    hold on; grid on;
    x = 1:N_UKF;
    for u = 1:N_UKF
        vals = r2_cal_mean(sc,u);
        bar(x(u), vals, bw, 'FaceColor', colors(u,:), 'EdgeColor', 'k');
        text(x(u), vals+0.1, sprintf('%.2f',vals), ...
            'HorizontalAlignment','center', 'FontSize',9, 'FontWeight','bold');
    end
    set(gca, 'XTick', x, 'XTickLabel', UKF_NAMES, 'FontSize',10);
    ylabel('R2 Cal RMSE (km)', 'FontSize',11, 'FontWeight','bold');
    title(sprintf('场景: %s', scenes{sc}), 'FontSize',12, 'FontWeight','bold');
    ylim([0 max(r2_cal_mean(sc,:))*1.3]);
end
sgtitle('Chart 2: 三种UKF校准后R2 RMSE对比', 'FontSize',13, 'FontWeight','bold');
saveas(fig2, 'results/chart2_ukf_r2_rmse.png');
close(fig2);

%% ================================================================
% Chart 3: 融合最佳RMSE vs 单站RMSE（展示融合增益）
% ================================================================
fprintf('生成 Chart 3: 融合增益对比 ...\n');
fig3 = figure('Position',[50 50 1200 600]);

for sc = 1:2
    subplot(2,1,sc);
    hold on; grid on;
    x = [1, 2, 3];
    labels = {'R1 UKF (imm)','R2 UKF (imm)','Fusion (best)'};

    if sc == 1
        fus_val = nanmean(S1.s.rmse_fus_best);
        vals = [S1.rmse_cal_R1(3), S1.rmse_cal_R2(3), fus_val];
    else
        fus_val = nanmean(S2.s.rmse_fus_best);
        vals = [S2.rmse_cal_R1(3), S2.rmse_cal_R2(3), fus_val];
    end

    bar(x, vals, 'FaceColor', [0.6 0.7 0.9], 'EdgeColor', 'k');
    for i = 1:3
        text(x(i), vals(i)+0.5, sprintf('%.1f', vals(i)), ...
            'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold');
    end

    % 标注融合改善百分比
    if sc == 1
        imp = (1 - fus_val/S1.rmse_cal_R1(3)) * 100;
        y_max = max(vals);
        text(3, y_max+1, sprintf('改善 %.0f%%', imp), ...
            'HorizontalAlignment','center', 'FontSize',11, 'FontWeight','bold', 'Color','r');
    else
        imp = (1 - fus_val/S2.rmse_cal_R1(3)) * 100;
        y_max = max(vals);
        text(3, y_max+5, sprintf('改善 %.0f%%', imp), ...
            'HorizontalAlignment','center', 'FontSize',11, 'FontWeight','bold', 'Color','r');
    end

    set(gca, 'XTick', x, 'XTickLabel', labels, 'FontSize',11);
    ylabel('RMSE (km)', 'FontSize',12, 'FontWeight','bold');
    title(sprintf('Chart 3: 融合增益 — %s', scenes{sc}), 'FontSize',12, 'FontWeight','bold');
    ylim([0 y_max*1.4]);
end
saveas(fig3, 'results/chart3_fusion_gain.png');
close(fig3);

%% ================================================================
% Chart 4: 最佳UKF获胜分布（堆叠柱状图）
% ================================================================
fprintf('生成 Chart 4: 最佳UKF获胜分布 ...\n');
fig4 = figure('Position',[50 50 900 500]);

counts1 = [sum(S1.best_ukf_for_seed==1), sum(S1.best_ukf_for_seed==2), sum(S1.best_ukf_for_seed==3)];
counts2 = [sum(S2.best_ukf_for_seed==1), sum(S2.best_ukf_for_seed==2), sum(S2.best_ukf_for_seed==3)];

x = [1, 2];
b = bar(x, [counts1; counts2]', 'stacked');
b(1).FaceColor = [0.4 0.4 0.4];  % jichu 灰
b(2).FaceColor = [0.2 0.7 0.3];  % zishiying 绿
b(3).FaceColor = [0 0.45 0.85];  % imm 蓝

set(gca, 'XTick', x, 'XTickLabel', scenes, 'FontSize',11);
ylabel('获胜次数 (out of 200)', 'FontSize',11, 'FontWeight','bold');
legend({'jichu','zishiying','imm'}, 'Location','northwest', 'FontSize',10);
title('Chart 4: 最佳UKF选择分布 (200次MC)', 'FontSize',12, 'FontWeight','bold');
grid on;

% 标注数值
for u = 1:3
    text(1, sum(counts1(1:u))-counts1(u)/2, sprintf('%d',counts1(u)), ...
        'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold');
    text(2, sum(counts2(1:u))-counts2(u)/2, sprintf('%d',counts2(u)), ...
        'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold');
end
saveas(fig4, 'results/chart4_best_ukf_distribution.png');
close(fig4);

%% ================================================================
% Chart 5: 关联率与NIS对比
% ================================================================
fprintf('生成 Chart 5: 关联率与NIS对比 ...\n');
fig5 = figure('Position',[50 50 1200 500]);

assoc_R1 = [mean(S1.assoc_R1), mean(S2.assoc_R1)];
assoc_R2 = [mean(S1.assoc_R2), mean(S2.assoc_R2)];
nis_R1 = [mean(S1.nis_mean_R1), mean(S2.nis_mean_R1)];
nis_R2 = [mean(S1.nis_mean_R2), mean(S2.nis_mean_R2)];

x = [1, 2];
labels_sc = {'turn180', 'turn46.7'};

subplot(1,2,1);
hold on; grid on;
plot(x, assoc_R1, '-o', 'LineWidth',2, 'MarkerSize',8, 'Color',[0 0.5 0.9], 'DisplayName','Assoc R1');
plot(x, assoc_R2, '-s', 'LineWidth',2, 'MarkerSize',8, 'Color',[0.9 0.3 0.1], 'DisplayName','Assoc R2');
plot(x, nis_R1*100, '-^', 'LineWidth',2, 'MarkerSize',8, 'Color',[0.2 0.7 0.3], 'DisplayName','NIS R1 x100');
plot(x, nis_R2*100, '-d', 'LineWidth',2, 'MarkerSize',8, 'Color',[0.8 0.2 0.8], 'DisplayName','NIS R2 x100');
set(gca, 'XTick', x, 'XTickLabel', labels_sc, 'FontSize',10);
ylabel('值', 'FontSize',11, 'FontWeight','bold');
title('Chart 5a: 关联率与NIS (R1 vs R2)', 'FontSize',12, 'FontWeight','bold');
legend('Location','best', 'FontSize',9);

subplot(1,2,2);
hold on; grid on;
gate_R1 = [mean(S1.nis_gate_R1), mean(S2.nis_gate_R1)];
gate_R2 = [mean(S1.nis_gate_R2), mean(S2.nis_gate_R2)];
plot(x, gate_R1, '-o', 'LineWidth',2, 'MarkerSize',8, 'Color',[0 0.5 0.9], 'DisplayName','Gate R1');
plot(x, gate_R2, '-s', 'LineWidth',2, 'MarkerSize',8, 'Color',[0.9 0.3 0.1], 'DisplayName','Gate R2');
set(gca, 'XTick', x, 'XTickLabel', labels_sc, 'FontSize',10);
ylabel('门控率 (%)', 'FontSize',11, 'FontWeight','bold');
title('Chart 5b: NIS门控率对比', 'FontSize',12, 'FontWeight','bold');
legend('Location','best', 'FontSize',9);

sgtitle('Chart 5: 关联质量与NIS分析', 'FontSize',13, 'FontWeight','bold');
saveas(fig5, 'results/chart5_assoc_nis.png');
close(fig5);

%% ================================================================
% Chart 6: 跟踪寿命与中断指标
% ================================================================
fprintf('生成 Chart 6: 跟踪寿命与中断 ...\n');
fig6 = figure('Position',[50 50 1200 500]);

mtl_R1 = [mean(S1.mtl_R1), mean(S2.mtl_R1)];
mtl_R2 = [mean(S1.mtl_R2), mean(S2.mtl_R2)];
mtl_fus = [mean(S1.mtl_fus), mean(S2.mtl_fus)];

brk_R1 = [mean(S1.brk_R1), mean(S2.brk_R1)];
brk_R2 = [mean(S1.brk_R2), mean(S2.brk_R2)];
brk_fus = [mean(S1.brk_fus), mean(S2.brk_fus)];

x = [1, 2];
labels_sc = {'turn180', 'turn46.7'};

subplot(1,2,1);
hold on; grid on;
bw3 = 0.25;
bar(x(1)-bw3, mtl_R1(1), bw3, 'FaceColor',[0.3 0.6 0.9], 'EdgeColor','k');
bar(x(1), mtl_R2(1), bw3, 'FaceColor',[0.9 0.4 0.3], 'EdgeColor','k');
bar(x(1)+bw3, mtl_fus(1), bw3, 'FaceColor',[0.2 0.7 0.3], 'EdgeColor','k');
bar(x(2)-bw3, mtl_R1(2), bw3, 'FaceColor',[0.3 0.6 0.9], 'EdgeColor','k');
bar(x(2), mtl_R2(2), bw3, 'FaceColor',[0.9 0.4 0.3], 'EdgeColor','k');
bar(x(2)+bw3, mtl_fus(2), bw3, 'FaceColor',[0.2 0.7 0.3], 'EdgeColor','k');
text(x(1)-bw3, mtl_R1(1)+2, sprintf('%.0f',mtl_R1(1)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(1), mtl_R2(1)+2, sprintf('%.0f',mtl_R2(1)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(1)+bw3, mtl_fus(1)+2, sprintf('%.0f',mtl_fus(1)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(2)-bw3, mtl_R1(2)+2, sprintf('%.0f',mtl_R1(2)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(2), mtl_R2(2)+2, sprintf('%.0f',mtl_R2(2)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(2)+bw3, mtl_fus(2)+2, sprintf('%.0f',mtl_fus(2)), 'HorizontalAlignment','center', 'FontSize',8);
set(gca, 'XTick', x, 'XTickLabel', labels_sc, 'FontSize',10);
ylabel('跟踪寿命 (帧)', 'FontSize',11, 'FontWeight','bold');
legend({sprintf('MTL R1'),sprintf('MTL R2'),sprintf('MTL Fus')}, 'Location','best', 'FontSize',9);
title('Chart 6a: 跟踪寿命 (MTL)', 'FontSize',12, 'FontWeight','bold');
grid on;

subplot(1,2,2);
hold on; grid on;
bar(x(1)-bw3, brk_R1(1), bw3, 'FaceColor',[0.9 0.5 0.3], 'EdgeColor','k');
bar(x(1), brk_R2(1), bw3, 'FaceColor',[0.9 0.7 0.3], 'EdgeColor','k');
bar(x(1)+bw3, brk_fus(1), bw3, 'FaceColor',[0.7 0.7 0.7], 'EdgeColor','k');
bar(x(2)-bw3, brk_R1(2), bw3, 'FaceColor',[0.9 0.5 0.3], 'EdgeColor','k');
bar(x(2), brk_R2(2), bw3, 'FaceColor',[0.9 0.7 0.3], 'EdgeColor','k');
bar(x(2)+bw3, brk_fus(2), bw3, 'FaceColor',[0.7 0.7 0.7], 'EdgeColor','k');
text(x(1)-bw3, brk_R1(1)+0.02, sprintf('%.2f',brk_R1(1)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(1), brk_R2(1)+0.02, sprintf('%.2f',brk_R2(1)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(1)+bw3, brk_fus(1)+0.02, sprintf('%.2f',brk_fus(1)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(2)-bw3, brk_R1(2)+0.02, sprintf('%.2f',brk_R1(2)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(2), brk_R2(2)+0.02, sprintf('%.2f',brk_R2(2)), 'HorizontalAlignment','center', 'FontSize',8);
text(x(2)+bw3, brk_fus(2)+0.02, sprintf('%.2f',brk_fus(2)), 'HorizontalAlignment','center', 'FontSize',8);
set(gca, 'XTick', x, 'XTickLabel', labels_sc, 'FontSize',10);
ylabel('中断次数', 'FontSize',11, 'FontWeight','bold');
legend({sprintf('Brk R1'),sprintf('Brk R2'),sprintf('Brk Fus')}, 'Location','best', 'FontSize',9);
title('Chart 6b: 跟踪中断次数', 'FontSize',12, 'FontWeight','bold');
grid on;

saveas(fig6, 'results/chart6_mtl_breaks.png');
close(fig6);

%% ================================================================
% Chart 7: RMSE箱线图（200次MC分布）
% ================================================================
fprintf('生成 Chart 7: RMSE分布箱线图 ...\n');
fig7 = figure('Position',[50 50 1200 600]);

data_R1 = zeros(N_MC*2, N_UKF);
data_R2 = zeros(N_MC*2, N_UKF);

for u = 1:N_UKF
    data_R1(1:N_MC, u) = S1.s.rmse_ukf_R1;
    data_R1(N_MC+1:end, u) = S2.s.rmse_ukf_R1;
    data_R2(1:N_MC, u) = S1.s.rmse_ukf_R2;
    data_R2(N_MC+1:end, u) = S2.s.rmse_ukf_R2;
end

% R1箱线图
subplot(2,2,[1,3]);
boxplot(data_R1, UKF_NAMES, ...
    'Labels', UKF_NAMES, ...
    'Colors', colors, ...
    'MedianLine', [1 0 0], ...
    'Whisker', 1.5, ...
    'Notch','on');
ylabel('R1 UKF RMSE (km)', 'FontSize',11, 'FontWeight','bold');
title('Chart 7a: R1 RMSE分布 (箱线图, notch=on)', 'FontSize',12, 'FontWeight','bold');
grid on;

% R2箱线图
subplot(2,2,[2,4]);
boxplot(data_R2, UKF_NAMES, ...
    'Labels', UKF_NAMES, ...
    'Colors', colors, ...
    'MedianLine', [1 0 0], ...
    'Whisker', 1.5, ...
    'Notch','on');
ylabel('R2 UKF RMSE (km)', 'FontSize',11, 'FontWeight','bold');
title('Chart 7b: R2 RMSE分布 (箱线图, notch=on)', 'FontSize',12, 'FontWeight','bold');
grid on;

saveas(fig7, 'results/chart7_boxplot.png');
close(fig7);

%% ================================================================
% Chart 8: 原始vs校准RMSE对比 + 改善百分比
% ================================================================
fprintf('生成 Chart 8: 校准前后RMSE对比 ...\n');
fig8 = figure('Position',[50 50 1200 500]);

x = [1, 2];
labels_sc = {'turn180', 'turn46.7'};

for r = 1:2
    subplot(1,2,r);
    hold on; grid on;
    bw2 = 0.35;

    if r == 1
        y_raw = raw_R1_mean;
        y_cal = cal_R1_mean;
        y_raw2 = raw_R2_mean;
        y_cal2 = cal_R2_mean;
    else
        y_raw = raw_R2_mean;
        y_cal = cal_R2_mean;
        y_raw2 = raw_R1_mean;
        y_cal2 = cal_R1_mean;
    end

    bar(x(1)-bw2/2, y_raw(1), bw2, 'FaceColor',[0.8 0.8 0.8], 'EdgeColor','k');
    bar(x(1)+bw2/2, y_cal(1), bw2, 'FaceColor',[0.2 0.6 0.9], 'EdgeColor','k');
    bar(x(2)-bw2/2, y_raw(2), bw2, 'FaceColor',[0.8 0.8 0.8], 'EdgeColor','k');
    bar(x(2)+bw2/2, y_cal(2), bw2, 'FaceColor',[0.2 0.6 0.9], 'EdgeColor','k');

    text(x(1)-bw2/2, y_raw(1)+1, sprintf('%.0f',y_raw(1)), 'HorizontalAlignment','center', 'FontSize',8);
    text(x(1)+bw2/2, y_cal(1)+0.3, sprintf('%.1f',y_cal(1)), 'HorizontalAlignment','center', 'FontSize',8, 'FontWeight','bold');
    text(x(2)-bw2/2, y_raw(2)+1, sprintf('%.0f',y_raw(2)), 'HorizontalAlignment','center', 'FontSize',8);
    text(x(2)+bw2/2, y_cal(2)+0.3, sprintf('%.1f',y_cal(2)), 'HorizontalAlignment','center', 'FontSize',8, 'FontWeight','bold');

    imp1 = (1-y_cal(1)/y_raw(1))*100;
    imp2 = (1-y_cal(2)/y_raw(2))*100;
    text(1.5, max(y_raw(1),y_cal(1),y_raw(2),y_cal(2))+2, ...
        sprintf('改善: %.0f%% -> %.0f%%', imp1, imp2), ...
        'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold', 'Color','r');

    set(gca, 'XTick', x, 'XTickLabel', labels_sc, 'FontSize',10);
    ylabel('RMSE (km)', 'FontSize',11, 'FontWeight','bold');
    title(sprintf('R%d: Raw vs Cal', r), 'FontSize',11, 'FontWeight','bold');
    legend({'Raw','Cal'}, 'Location','northwest');
end
sgtitle('Chart 8: 校准前后RMSE对比（各UKF均值）', 'FontSize',13, 'FontWeight','bold');
saveas(fig8, 'results/chart8_raw_vs_cal.png');
close(fig8);

%% ================================================================
% 打印汇总统计
% ================================================================
fprintf('\n========== 图表生成完成 ==========\n');
fprintf('共生成 8 组图表，保存在 results/ 目录:\n');
fprintf('  chart1_ukf_rmse_comparison.png   — UKF校准RMSE (R1)\n');
fprintf('  chart2_ukf_r2_rmse.png           — UKF校准RMSE (R2)\n');
fprintf('  chart3_fusion_gain.png           — 融合增益对比\n');
fprintf('  chart4_best_ukf_distribution.png — 最佳UKF分布\n');
fprintf('  chart5_assoc_nis.png             — 关联率与NIS\n');
fprintf('  chart6_mtl_breaks.png            — 跟踪寿命与中断\n');
fprintf('  chart7_boxplot.png               — RMSE箱线图\n');
fprintf('  chart8_raw_vs_cal.png            — 校准前后对比\n');
fprintf('==================================\n\n');

drawnow;
