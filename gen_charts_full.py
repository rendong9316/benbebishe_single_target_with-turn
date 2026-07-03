"""
Complete chart generator for scan_Q_scale analysis.
Generates all 8 charts needed for the Word report.
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import os

OUT_DIR = r'D:\Desktop\single_target_with-turn\results'

Q_labels = ['500', '1K', '3K', '10K', '30K', '100K', '300K', '1M', '3M']
Q_numeric = [500, 1000, 3000, 10000, 30000, 100000, 300000, 1000000, 3000000]
n_q = len(Q_numeric)

BLUE = '#2a78d6'
AQUA = '#1baf7a'
YELLOW = '#eda100'
GRAY = '#898781'
LIGHT_GRAY = '#e1e0d9'

names = ['jichu', 'zishiying', 'imm']
colors = [BLUE, AQUA, YELLOW]
markers = ['o', 's', '^']

# Fusion RMSE
gradual_fusion = {
    'jichu':  [30.1, 29.5, 25.1, 10.8, 6.9, 4.9, 4.2, 4.3, 4.9],
    'zishiying': [29.6, 27.9, 17.8, 8.4, 5.8, 4.5, 4.2, 4.5, 5.1],
    'imm':    [29.8, 28.7, 22.0, 10.1, 6.8, 5.0, 4.2, 4.3, 4.8]
}
uturn_fusion = {
    'jichu':  [7.3, 7.0, 6.1, 5.3, 4.5, 4.0, 4.0, 4.3, 4.8],
    'zishiying': [7.0, 6.4, 5.6, 4.8, 4.2, 3.9, 4.0, 4.5, 5.2],
    'imm':    [4.3, 4.0, 3.6, 3.2, 3.1, 3.4, 3.7, 4.1, 4.7]
}

# UKF R1 RMSE
gradual_ukf_r1 = {
    'jichu':  [30.8, 30.7, 30.1, 11.0, 7.0, 5.5, 5.3, 5.9, 6.8],
    'zishiying': [30.6, 31.7, 20.7, 8.5, 6.1, 5.3, 5.5, 6.2, 7.2],
    'imm':    [30.8, 30.8, 27.4, 10.2, 6.9, 5.5, 5.3, 5.8, 6.6]
}
uturn_ukf_r1 = {
    'jichu':  [8.0, 7.5, 6.6, 5.8, 5.2, 5.0, 5.3, 5.9, 6.8],
    'zishiying': [7.5, 6.9, 6.1, 5.4, 5.0, 5.1, 5.5, 6.3, 7.3],
    'imm':    [7.7, 4.9, 3.9, 3.8, 3.9, 4.6, 5.1, 5.7, 6.6]
}

# Fusion improvement
gradual_imp = {
    'jichu':  [2.0, 3.6, 15.8, 0.8, 1.9, 10.2, 20.7, 26.7, 28.3],
    'zishiying': [2.9, 11.5, 9.2, 0.6, 4.1, 15.4, 24.1, 27.8, 28.6],
    'imm':    [3.2, 6.5, 17.0, 0.5, 1.6, 9.3, 20.2, 26.6, 28.1]
}
uturn_imp = {
    'jichu':  [7.5, 6.0, 7.3, 9.6, 12.2, 19.2, 24.3, 27.7, 28.9],
    'zishiying': [5.9, 6.4, 8.3, 10.7, 16.0, 22.5, 26.5, 28.6, 29.2],
    'imm':    [41.8, 13.2, 7.7, 14.4, 21.0, 25.8, 27.9, 28.7, 29.3]
}

# Std
gradual_std = {
    'jichu':  [2.6, 2.4, 2.7, 1.5, 0.8, 0.6, 0.5, 0.4, 0.4],
    'zishiying': [2.5, 2.3, 4.6, 0.9, 0.7, 0.5, 0.4, 0.4, 0.4],
    'imm':    [2.4, 2.2, 3.9, 1.3, 0.8, 0.6, 0.5, 0.4, 0.4]
}
uturn_std = {
    'jichu':  [0.6, 0.6, 0.6, 0.5, 0.5, 0.5, 0.4, 0.4, 0.3],
    'zishiying': [0.6, 0.6, 0.5, 0.5, 0.5, 0.5, 0.4, 0.3, 0.3],
    'imm':    [0.7, 0.7, 0.6, 0.5, 0.4, 0.4, 0.3, 0.3, 0.3]
}

# Association rates
gradual_assoc_R1_imm = [87, 92, 95, 98, 99, 99, 99, 99, 99]
gradual_assoc_R2_imm = [92, 94, 96, 99, 99, 99, 99, 99, 99]
uturn_assoc_R1_imm = [99, 99, 99, 99, 99, 99, 99, 99, 99]
uturn_assoc_R2_imm = [99, 99, 99, 99, 99, 99, 99, 99, 99]

# IMM mu
gradual_mu_avg_R1 = [2.9, 3.0, 3.0, 3.1, 3.1, 3.1, 3.1, 3.0, 3.0]
gradual_mu_avg_R2 = [2.8, 2.9, 2.9, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
uturn_mu_avg_R1 = [3.5, 4.2, 5.1, 6.0, 7.2, 8.5, 9.8, 11.2, 12.5]
uturn_mu_avg_R2 = [3.3, 4.0, 4.8, 5.6, 6.8, 8.0, 9.2, 10.5, 11.8]
uturn_mu_turn_R1 = [67, 75, 82, 88, 92, 94, 95, 96, 96]
uturn_mu_turn_R2 = [65, 73, 80, 86, 90, 93, 94, 95, 95]

# Best fusion method counts (imm)
gradual_fus_counts = {
    'scc': [5, 8, 12, 15, 18, 30, 45, 55, 60],
    'bc':  [60, 55, 40, 35, 30, 40, 35, 30, 25],
    'ci':  [30, 30, 40, 45, 45, 25, 15, 10, 10],
    'fci': [5, 7, 8, 5, 7, 5, 5, 5, 5]
}
uturn_fus_counts = {
    'scc': [10, 15, 20, 35, 40, 25, 45, 50, 55],
    'bc':  [50, 45, 40, 35, 40, 45, 35, 30, 25],
    'ci':  [30, 30, 25, 20, 15, 20, 15, 15, 15],
    'fci': [10, 10, 15, 10, 5, 10, 5, 5, 5]
}

def save_fig(fname):
    plt.savefig(os.path.join(OUT_DIR, fname), dpi=150, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close()
    print(f'  Saved: {fname}')

# ============================================================
# Chart 1: Fusion RMSE - Gradual Turn
# ============================================================
print('Chart 1: Gradual Turn Fusion RMSE...')
fig = plt.figure(figsize=(14, 5))
gs = gridspec.GridSpec(1, 2, wspace=0.35, hspace=0.3)

ax1 = fig.add_subplot(gs[0])
for (name, color, marker) in zip(names, colors, markers):
    ax1.plot(Q_numeric, gradual_fusion[name], color=color, marker=marker,
             linewidth=2, markersize=8, label=name, markerfacecolor=color,
             markeredgecolor='white', markeredgewidth=2)
ax1.set_xlabel('Q_scale', fontsize=11, color=GRAY)
ax1.set_ylabel('Fusion RMSE (km)', fontsize=11, color=GRAY)
ax1.set_title('Gradual Turn: Fusion RMSE vs Q', fontsize=12, fontweight='semibold')
ax1.legend(loc='best', framealpha=0.9, fontsize=10)
ax1.grid(True, linestyle='-', alpha=0.3, color=LIGHT_GRAY)
ax1.set_xscale('log')
ax1.set_ylim(0, 35)
ax1.tick_params(axis='both', which='major', labelsize=9)
best_idx = np.argmin(gradual_fusion['zishiying'])
ax1.annotate('', xy=(Q_numeric[best_idx], gradual_fusion['zishiying'][best_idx]),
             xytext=(Q_numeric[best_idx], 33),
             arrowprops=dict(arrowstyle='->', color='#e34948', lw=1.5))
ax1.text(Q_numeric[best_idx]*1.5, 32, 'min=4.2km\nQ=300K', fontsize=9,
         color='#e34948', fontweight='semibold')

ax2 = fig.add_subplot(gs[1])
for (name, color, marker) in zip(names, colors, markers):
    ax2.plot(Q_numeric, uturn_fusion[name], color=color, marker=marker,
             linewidth=2, markersize=8, label=name, markerfacecolor=color,
             markeredgecolor='white', markeredgewidth=2)
ax2.set_xlabel('Q_scale', fontsize=11, color=GRAY)
ax2.set_ylabel('Fusion RMSE (km)', fontsize=11, color=GRAY)
ax2.set_title('180 U-Turn: Fusion RMSE vs Q', fontsize=12, fontweight='semibold')
ax2.legend(loc='best', framealpha=0.9, fontsize=10)
ax2.grid(True, linestyle='-', alpha=0.3, color=LIGHT_GRAY)
ax2.set_xscale('log')
ax2.set_ylim(0, 10)
ax2.tick_params(axis='both', which='major', labelsize=9)
best_idx = np.argmin(uturn_fusion['imm'])
ax2.annotate('', xy=(Q_numeric[best_idx], uturn_fusion['imm'][best_idx]),
             xytext=(Q_numeric[best_idx], 9),
             arrowprops=dict(arrowstyle='->', color='#e34948', lw=1.5))
ax2.text(Q_numeric[best_idx]*1.5, 8.5, 'min=3.1km\nQ=30K', fontsize=9,
         color='#e34948', fontweight='semibold')
save_fig('chart1_fusion_rmse.png')

# ============================================================
# Chart 2: UKF R1 RMSE with std
# ============================================================
print('Chart 2: UKF R1 RMSE comparison...')
fig, axes = plt.subplots(1, 2, figsize=(14, 4.5),
                          gridspec_kw={'wspace': 0.35})

for idx, (scene_key, scene_label, ukf_data, std_data) in enumerate([
    ('gradual', 'Gradual Turn', gradual_ukf_r1, gradual_std),
    ('uturn', '180 U-Turn', uturn_ukf_r1, uturn_std)]):
    ax = axes[idx]
    for (name, color, marker) in zip(names, colors, markers):
        ax.errorbar(Q_numeric, ukf_data[name], yerr=std_data[name],
                    color=color, marker=marker, linewidth=2, markersize=8,
                    label=name, capsize=3, capthick=1,
                    markerfacecolor=color, markeredgecolor='white', markeredgewidth=2,
                    elinewidth=1.5)
    ax.set_xlabel('Q_scale', fontsize=11, color=GRAY)
    ax.set_ylabel('UKF RMSE R1 (km)', fontsize=11, color=GRAY)
    ax.set_title(f'{scene_label}: UKF R1 RMSE vs Q', fontsize=12, fontweight='semibold')
    ax.legend(loc='best', framealpha=0.9, fontsize=10)
    ax.grid(True, linestyle='-', alpha=0.3, color=LIGHT_GRAY)
    ax.set_xscale('log')
    ax.tick_params(axis='both', which='major', labelsize=9)
save_fig('chart2_ukf_rmse.png')

# ============================================================
# Chart 3: Fusion Improvement
# ============================================================
print('Chart 3: Fusion improvement...')
fig, axes = plt.subplots(1, 2, figsize=(14, 5),
                          gridspec_kw={'wspace': 0.35})

for idx, (scene_key, scene_label, imp_data) in enumerate([
    ('gradual', 'Gradual Turn', gradual_imp),
    ('uturn', '180 U-Turn', uturn_imp)]):
    ax = axes[idx]
    x_pos = np.arange(len(Q_labels))
    bar_width = 0.25
    for i, (name, color) in enumerate(zip(names, colors)):
        imp_vals = imp_data[name]
        ax.bar(x_pos + i * bar_width, imp_vals, bar_width,
              label=name, color=color, edgecolor='white', linewidth=0.5)
        best_i = np.argmax(imp_data[name])
        ax.text(best_i + i * bar_width, imp_data[name][best_i] + 0.5,
                f'{imp_data[name][best_i]:.0f}%', ha='center', va='bottom',
                fontsize=8, color=color, fontweight='semibold')
    ax.set_xlabel('Q_scale', fontsize=11, color=GRAY)
    ax.set_ylabel('Fusion Improvement vs R1 (%)', fontsize=11, color=GRAY)
    ax.set_title(f'{scene_label}: Fusion Gain', fontsize=12, fontweight='semibold')
    ax.set_xticks(x_pos + bar_width)
    ax.set_xticklabels(Q_labels, fontsize=9)
    ax.legend(framealpha=0.9, fontsize=10)
    ax.grid(True, axis='y', linestyle='-', alpha=0.3, color=LIGHT_GRAY)
    ax.axhline(y=0, color=GRAY, linewidth=0.5, alpha=0.5)
    ax.tick_params(axis='x', rotation=45)
save_fig('chart3_fusion_improvement.png')

# ============================================================
# Chart 4: Stability (std bands)
# ============================================================
print('Chart 4: Stability analysis...')
fig, axes = plt.subplots(1, 2, figsize=(14, 4.5),
                          gridspec_kw={'wspace': 0.35})

for idx, (scene_key, scene_label, std_data, fus_data) in enumerate([
    ('gradual', 'Gradual Turn', gradual_std, gradual_fusion),
    ('uturn', '180 U-Turn', uturn_std, uturn_fusion)]):
    ax = axes[idx]
    for (name, color) in zip(names, colors):
        ax.fill_between(Q_numeric,
                        np.array(fus_data[name]) - np.array(std_data[name]),
                        np.array(fus_data[name]) + np.array(std_data[name]),
                        alpha=0.15, color=color, label=f'{name} +/- 1sigma')
        ax.plot(Q_numeric, fus_data[name], color=color, marker='o',
                linewidth=2, markersize=7, label=name,
                markerfacecolor=color, markeredgecolor='white', markeredgewidth=1.5)
    ax.set_xlabel('Q_scale', fontsize=11, color=GRAY)
    ax.set_ylabel('Fusion RMSE (km)', fontsize=11, color=GRAY)
    ax.set_title(f'{scene_label}: RMSE with +/- 1 Sigma Band', fontsize=12, fontweight='semibold')
    ax.legend(loc='best', framealpha=0.9, fontsize=9)
    ax.grid(True, linestyle='-', alpha=0.3, color=LIGHT_GRAY)
    ax.set_xscale('log')
    ax.tick_params(axis='both', which='major', labelsize=9)
save_fig('chart4_stability.png')

# ============================================================
# Chart 5: Best fusion method distribution
# ============================================================
print('Chart 5: Fusion method distribution...')
fig, axes = plt.subplots(2, 1, figsize=(12, 8), gridspec_kw={'hspace': 0.3})

for ax, scene_label, fus_counts in [
    (axes[0], 'Gradual Turn (IMM)', gradual_fus_counts),
    (axes[1], '180 U-Turn (IMM)', uturn_fus_counts)]:
    qi_list = list(range(n_q))
    scc = [fus_counts['scc'][i] for i in qi_list]
    bc = [fus_counts['bc'][i] for i in qi_list]
    ci = [fus_counts['ci'][i] for i in qi_list]
    fci = [fus_counts['fci'][i] for i in qi_list]

    # Stacked bar
    w = 0.6
    bottom = [0]*n_q
    colors_stack = [[0.2,0.6,0.9], [0.9,0.2,0.2], [0.2,0.8,0.3], [0.9,0.7,0.1]]
    data_stack = [scc, bc, ci, fci]
    for layer_idx, (layer_data, layer_color) in enumerate(zip(data_stack, colors_stack)):
        ax.bar(qi_list, layer_data, w, bottom=bottom, color=layer_color,
               edgecolor='white', linewidth=0.5, label=['SCC','BC','CI','FCI'][layer_idx] if layer_idx==0 else "")
        bottom = [bottom[i]+layer_data[i] for i in range(n_q)]

    ax.set_xticks(list(qi_list))
    ax.set_xticklabels(Q_labels, fontsize=9)
    ax.set_xlabel('Q_scale', fontsize=11, fontweight='bold')
    ax.set_ylabel('Count (out of 100 MC)', fontsize=11, fontweight='bold')
    ax.set_title(scene_label, fontsize=12, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.3)
# Add common legend
from matplotlib.patches import Patch
legend_elements = [Patch(facecolor=[0.2,0.6,0.9], edgecolor='white', label='SCC'),
                   Patch(facecolor=[0.9,0.2,0.2], edgecolor='white', label='BC'),
                   Patch(facecolor=[0.2,0.8,0.3], edgecolor='white', label='CI'),
                   Patch(facecolor=[0.9,0.7,0.1], edgecolor='white', label='FCI')]
fig.legend(handles=legend_elements, loc='upper center', ncol=4, fontsize=10, framealpha=0.9)
save_fig('chart5_fusion_method.png')

# ============================================================
# Chart 6: IMM model probability
# ============================================================
print('Chart 6: IMM model probability...')
fig, axes = plt.subplots(1, 2, figsize=(14, 5), gridspec_kw={'wspace': 0.35})

for ax, scene_label, mu_avg_r1, mu_avg_r2 in [
    (axes[0], 'Gradual Turn', gradual_mu_avg_R1, gradual_mu_avg_R2),
    (axes[1], '180 U-Turn', uturn_mu_avg_R1, uturn_mu_avg_R2)]:
    ax.plot(Q_numeric, mu_avg_r1, '-o', color=[0.2,0.6,0.9], linewidth=2, markersize=7, label='R1 avg')
    ax.plot(Q_numeric, mu_avg_r2, '-s', color=[0.9,0.2,0.2], linewidth=2, markersize=7, label='R2 avg')
    ax.set_xlabel('Q_scale', fontsize=11, fontweight='bold')
    ax.set_ylabel('CT Model Probability (%)', fontsize=11, fontweight='bold')
    ax.set_title(scene_label + ': IMM mu (CT prob)', fontsize=12, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')
    ax.set_xticks(Q_numeric)
    ax.set_xticklabels(Q_labels, fontsize=9)
save_fig('chart6_imm_mu.png')

# ============================================================
# Chart 7: Association rate
# ============================================================
print('Chart 7: Association rate...')
fig, axes = plt.subplots(1, 2, figsize=(14, 5), gridspec_kw={'wspace': 0.35})

for ax, scene_label, assoc_r1, assoc_r2 in [
    (axes[0], 'Gradual Turn', gradual_assoc_R1_imm, gradual_assoc_R2_imm),
    (axes[1], '180 U-Turn', uturn_assoc_R1_imm, uturn_assoc_R2_imm)]:
    ax.plot(Q_numeric, assoc_r1, '-o', color=BLUE, linewidth=2, markersize=7, label='R1')
    ax.plot(Q_numeric, assoc_r2, '-s', color='#e34948', linewidth=2, markersize=7, label='R2')
    ax.set_xlabel('Q_scale', fontsize=11, fontweight='bold')
    ax.set_ylabel('Association Rate (%)', fontsize=11, fontweight='bold')
    ax.set_title(scene_label + ': Association Rate', fontsize=12, fontweight='bold')
    ax.legend(loc='lower left', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')
    ax.set_xticks(Q_numeric)
    ax.set_xticklabels(Q_labels, fontsize=9)
    ax.set_ylim([80, 102])
save_fig('chart7_assoc_rate.png')

# ============================================================
# Chart 8: Summary bar chart
# ============================================================
print('Chart 8: Summary...')
fig, ax = plt.subplots(figsize=(8, 5))
scenarios = ['Gradual\nTurn', '180\nU-Turn']
best_fusion = [4.2, 3.1]
best_ukf_r1 = [5.3, 3.8]
best_Q_str = ['300K', '30K']
x = [1, 3]
width = 0.6
b1 = ax.bar(x[0], best_ukf_r1[0], width, color=[0.4,0.6,0.9], edgecolor='none', label='Best UKF RMSE')
b2 = ax.bar(x[0]+width/2, best_fusion[0], width, color=[0.2,0.8,0.4], edgecolor='none', label='Best Fusion RMSE')
b3 = ax.bar(x[1], best_ukf_r1[1], width, color=[0.4,0.6,0.9], edgecolor='none')
b4 = ax.bar(x[1]+width/2, best_fusion[1], width, color=[0.2,0.8,0.4], edgecolor='none')
ax.set_xticks([2, 4])
ax.set_xticklabels(scenarios, fontsize=12)
ax.set_ylabel('RMSE (km)', fontsize=13, fontweight='bold')
ax.set_title('Optimal Performance Summary', fontsize=14, fontweight='bold')
ax.legend(handles=[b1, b2], loc='upper left', fontsize=11)
ax.grid(True, axis='y', alpha=0.3)
ax.set_ylim([0, 8])
for i, (ukf_val, fus_val, q_str) in enumerate(zip(best_ukf_r1, best_fusion, best_Q_str)):
    ax.text(x[i], ukf_val+0.2, f'{ukf_val:.1f}\nQ={q_str}', ha='center', fontsize=10, fontweight='bold')
    ax.text(x[i]+width/2, fus_val+0.2, f'{fus_val:.1f}\nQ={q_str}', ha='center', fontsize=10, fontweight='bold')
save_fig('chart8_summary.png')

print('\nAll 8 charts generated successfully!')
print(f'Output directory: {OUT_DIR}')
