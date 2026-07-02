"""
Generate scan_Q_scale analysis charts and Word document.
Data extracted from the comprehensive MATLAB output in the previous analysis.
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from docx import Document
from docx.shared import Pt, Inches, Cm, RGBColor, Emu
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.section import WD_ORIENT
from docx.oxml.ns import qn, nsdecls
from docx.oxml import parse_xml
import os

BASE = r'D:\Desktop\single_target_with-turn\results'
OUT_DIR = r'D:\Desktop\single_target_with-turn\results'

# ============================================================
# DATA
# ============================================================
Q_labels = ['500', '1K', '3K', '10K', '30K', '100K', '300K', '1M', '3M']
Q_numeric = [500, 1000, 3000, 10000, 30000, 100000, 300000, 1000000, 3000000]

# Colors from palette.md: blue=#2a78d6, aqua=#1baf7a, yellow=#eda100
BLUE = '#2a78d6'
AQUA = '#1baf7a'
YELLOW = '#eda100'
GRAY = '#898781'
LIGHT_GRAY = '#e1e0d9'

# Gradual Turn - Fusion RMSE (best per UKF)
gradual_fusion = {
    'jichu':  [30.1, 29.5, 25.1, 10.8, 6.9, 4.9, 4.2, 4.3, 4.9],
    'zishiying': [29.6, 27.9, 17.8, 8.4, 5.8, 4.5, 4.2, 4.5, 5.1],
    'imm':    [29.8, 28.7, 22.0, 10.1, 6.8, 5.0, 4.2, 4.3, 4.8]
}

# U-Turn - Fusion RMSE
uturn_fusion = {
    'jichu':  [7.3, 7.0, 6.1, 5.3, 4.5, 4.0, 4.0, 4.3, 4.8],
    'zishiying': [7.0, 6.4, 5.6, 4.8, 4.2, 3.9, 4.0, 4.5, 5.2],
    'imm':    [4.3, 4.0, 3.6, 3.2, 3.1, 3.4, 3.7, 4.1, 4.7]
}

# Gradual Turn - UKF R1 RMSE
gradual_ukf_r1 = {
    'jichu':  [30.8, 30.7, 30.1, 11.0, 7.0, 5.5, 5.3, 5.9, 6.8],
    'zishiying': [30.6, 31.7, 20.7, 8.5, 6.1, 5.3, 5.5, 6.2, 7.2],
    'imm':    [30.8, 30.8, 27.4, 10.2, 6.9, 5.5, 5.3, 5.8, 6.6]
}

# U-Turn - UKF R1 RMSE
uturn_ukf_r1 = {
    'jichu':  [8.0, 7.5, 6.6, 5.8, 5.2, 5.0, 5.3, 5.9, 6.8],
    'zishiying': [7.5, 6.9, 6.1, 5.4, 5.0, 5.1, 5.5, 6.3, 7.3],
    'imm':    [7.7, 4.9, 3.9, 3.8, 3.9, 4.6, 5.1, 5.7, 6.6]
}

# Gradual Turn - Imp fusion vs R1 (%)
gradual_imp = {
    'jichu':  [2.0, 3.6, 15.8, 0.8, 1.9, 10.2, 20.7, 26.7, 28.3],
    'zishiying': [2.9, 11.5, 9.2, 0.6, 4.1, 15.4, 24.1, 27.8, 28.6],
    'imm':    [3.2, 6.5, 17.0, 0.5, 1.6, 9.3, 20.2, 26.6, 28.1]
}

# U-Turn - Imp fusion vs R1 (%)
uturn_imp = {
    'jichu':  [7.5, 6.0, 7.3, 9.6, 12.2, 19.2, 24.3, 27.7, 28.9],
    'zishiying': [5.9, 6.4, 8.3, 10.7, 16.0, 22.5, 26.5, 28.6, 29.2],
    'imm':    [41.8, 13.2, 7.7, 14.4, 21.0, 25.8, 27.9, 28.7, 29.3]
}

# Std deviation of fusion RMSE
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

# Best fusion method (imm)
gradual_methods = ['BC','BC','CI','CI','CI','BC','BC','BC','BC']
uturn_methods = ['FCI','CI','FCI','SCC','BC','BC','SCC','BC','BC']

# ============================================================
# CHART 1: Fusion RMSE - Gradual Turn (small multiples)
# ============================================================
fig = plt.figure(figsize=(14, 5))
gs = gridspec.GridSpec(1, 2, wspace=0.35, hspace=0.3)

ax1 = fig.add_subplot(gs[0])
names = ['jichu', 'zishiying', 'imm']
colors = [BLUE, AQUA, YELLOW]
markers = ['o', 's', '^']

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

# Annotate best point
best_idx = np.argmin(gradual_fusion['zishiying'])
ax1.annotate('', xy=(Q_numeric[best_idx], gradual_fusion['zishiying'][best_idx]),
             xytext=(Q_numeric[best_idx], 33),
             arrowprops=dict(arrowstyle='->', color='#e34948', lw=1.5))
ax1.text(Q_numeric[best_idx]*1.5, 32, 'min=4.16km\nQ=300K', fontsize=9,
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
ax2.text(Q_numeric[best_idx]*1.5, 8.5, 'min=3.10km\nQ=30K', fontsize=9,
         color='#e34948', fontweight='semibold')

plt.savefig(os.path.join(OUT_DIR, 'chart_fusion_rmse.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()
print('Chart 1 saved: chart_fusion_rmse.png')

# ============================================================
# CHART 2: UKF R1 RMSE - Gradual Turn (emphasis style)
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 4.5),
                          gridspec_kw={'wspace': 0.35})

for idx, (scene_key, scene_label) in enumerate([
    ('gradual', 'Gradual Turn'), ('uturn', '180 U-Turn')]):

    ax = axes[idx]
    ukf_data = gradual_ukf_r1 if scene_key == 'gradual' else uturn_ukf_r1
    std_data = gradual_std if scene_key == 'gradual' else uturn_std

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

plt.savefig(os.path.join(OUT_DIR, 'chart_ukf_rmse.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()
print('Chart 2 saved: chart_ukf_rmse.png')

# ============================================================
# CHART 3: Fusion Improvement (%) - Diverging bar
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 5),
                          gridspec_kw={'wspace': 0.35})

for idx, (scene_key, scene_label) in enumerate([
    ('gradual', 'Gradual Turn'), ('uturn', '180 U-Turn')]):

    ax = axes[idx]
    imp_data = gradual_imp if scene_key == 'gradual' else uturn_imp
    fus_data = gradual_fusion if scene_key == 'gradual' else uturn_fusion
    ukf_r1_data = gradual_ukf_r1 if scene_key == 'gradual' else uturn_ukf_r1

    x_pos = np.arange(len(Q_labels))
    bar_width = 0.25

    for i, (name, color) in enumerate(zip(names, colors)):
        imp_vals = imp_data[name]
        bars = ax.bar(x_pos + i * bar_width, imp_vals, bar_width,
                      label=name, color=color, edgecolor='white', linewidth=0.5,
                      yerr=std_data[name] if False else None)
        # Add end labels for the best point
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

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'chart_fusion_improvement.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()
print('Chart 3 saved: chart_fusion_improvement.png')

# ============================================================
# CHART 4: Optimal Q comparison - bar chart
# ============================================================
fig, ax = plt.subplots(figsize=(8, 5))

scenarios = ['Gradual\nTurn', '180\nU-Turn']
best_fusion = [4.16, 3.10]  # zishiying gradual, imm uturn
best_ukf_r1 = [5.3, 3.8]   # imm gradual, imm uturn
best_Q_idx_gradual = 6  # 300K
best_Q_idx_uturn = 4    # 30K

x = np.arange(len(scenarios))
width = 0.35

bars1 = ax.bar(x - width/2, best_ukf_r1, width, label='Best UKF RMSE R1',
               color=BLUE, edgecolor='white', linewidth=0.5)
bars2 = ax.bar(x + width/2, best_fusion, width, label='Best Fusion RMSE',
               color=AQUA, edgecolor='white', linewidth=0.5)

ax.set_ylabel('RMSE (km)', fontsize=11, color=GRAY)
ax.set_title('Optimal Performance Summary', fontsize=12, fontweight='semibold')
ax.set_xticks(x)
ax.set_xticklabels(['Gradual Turn\n(Q=300K)', '180 U-Turn\n(Q=30K)'], fontsize=10)
ax.legend(framealpha=0.9, fontsize=10)
ax.grid(True, axis='y', linestyle='-', alpha=0.3, color=LIGHT_GRAY)
ax.set_ylim(0, 8)

for bar in bars1:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height + 0.15,
            f'{height:.1f}', ha='center', va='bottom', fontsize=10, fontweight='semibold')
for bar in bars2:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height + 0.15,
            f'{height:.1f}', ha='center', va='bottom', fontsize=10, fontweight='semibold')

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'chart_summary.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()
print('Chart 4 saved: chart_summary.png')

# ============================================================
# CHART 5: Stability comparison (std)
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 4.5),
                          gridspec_kw={'wspace': 0.35})

for idx, (scene_key, scene_label) in enumerate([
    ('gradual', 'Gradual Turn'), ('uturn', '180 U-Turn')]):

    ax = axes[idx]
    std_data = gradual_std if scene_key == 'gradual' else uturn_std
    fus_data = gradual_fusion if scene_key == 'gradual' else uturn_fusion

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

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, 'chart_stability.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()
print('Chart 5 saved: chart_stability.png')

print('\nAll charts generated.')
