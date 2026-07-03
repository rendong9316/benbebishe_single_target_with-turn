"""
plot_mc_turn_compare.py
蒙特卡洛拐弯仿真对比可视化 - turn180 vs turn46.7
生成8组图表 + Word总结报告
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import scipy.io as sio
from matplotlib.gridspec import GridSpec
import os

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

BASE = r'D:\Desktop\single_target_with-turn'
RESULTS = os.path.join(BASE, 'results')

# 加载数据
S1 = sio.loadmat(os.path.join(RESULTS, 'mc_turn180_compare_20260701_164750.mat'))
S2 = sio.loadmat(os.path.join(RESULTS, 'mc_turn_compare_20260701_163230.mat'))

UKF_NAMES = ['jichu', 'zishiying', 'imm']
UKF_LABELS = ['基础UKF', '自适应UKF', 'IMM']
N_MC = 200

def sfield(S, fn):
    return S['s'][fn][0, 0].flatten()

def extract(S):
    return {
        'rmse_cal_R1': S['rmse_cal_R1'].flatten(),
        'rmse_cal_R2': S['rmse_cal_R2'].flatten(),
        'rmse_raw_R1': S['rmse_raw_R1'].flatten(),
        'rmse_raw_R2': S['rmse_raw_R2'].flatten(),
        'best_ukf': S['best_ukf_for_seed'].flatten().astype(int),
        's_rmse_ukf_R1': sfield(S, 'rmse_ukf_R1'),
        's_rmse_ukf_R2': sfield(S, 'rmse_ukf_R2'),
        's_rmse_fus_best': sfield(S, 'rmse_fus_best'),
        's_assoc_R1': sfield(S, 'assoc_R1'),
        's_assoc_R2': sfield(S, 'assoc_R2'),
        's_nis_R1': sfield(S, 'nis_mean_R1'),
        's_nis_R2': sfield(S, 'nis_mean_R2'),
        's_gate_R1': sfield(S, 'nis_gate_R1'),
        's_gate_R2': sfield(S, 'nis_gate_R2'),
        's_mtl_R1': sfield(S, 'mtl_R1'),
        's_mtl_R2': sfield(S, 'mtl_R2'),
        's_mtl_fus': sfield(S, 'mtl_fus'),
        's_brk_R1': sfield(S, 'brk_R1').astype(float),
        's_brk_R2': sfield(S, 'brk_R2').astype(float),
        's_brk_fus': sfield(S, 'brk_fus').astype(float),
        's_imp_fus_R1': sfield(S, 'imp_fus_vs_R1'),
        's_imp_fus_R2': sfield(S, 'imp_fus_vs_R2'),
        's_bad': sfield(S, 'bad_seed').astype(bool),
    }

D1 = extract(S1)  # turn180
D2 = extract(S2)  # turn46.7

# 从之前MATLAB分析得到的每UKF均值
R1_CAL = {
    'turn180': {'jichu': 9.0988, 'zishiying': 9.4814, 'imm': 9.0915},
    'turn46.7': {'jichu': 8.9834, 'zishiying': 10.0455, 'imm': 9.5435},
}
R2_CAL = {
    'turn180': {'jichu': 10.8225, 'zishiying': 11.9104, 'imm': 10.0305},
    'turn46.7': {'jichu': 10.8337, 'zishiying': 11.9918, 'imm': 10.0989},
}
RAW_R1 = {'turn180': 81.9195, 'turn46.7': 81.3910}
RAW_R2 = {'turn180': 83.5725, 'turn46.7': 83.1841}
CAL_R1_AVG = {'turn180': 9.9838, 'turn46.7': 9.8899}
CAL_R2_AVG = {'turn180': 11.1932, 'turn46.7': 11.1562}

PALETTE = ['#2196F3', '#4CAF50', '#f44336', '#FF9800', '#9C27B0', '#607D8B']
BLUE, GREEN, RED, ORANGE, PURPLE, GRAY = PALETTE[:6]
LIGHT_BLUE = '#BBDEFB'

fig_dir = RESULTS
os.makedirs(fig_dir, exist_ok=True)

def style_ax(ax, grid=True):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    if grid:
        ax.yaxis.grid(True, alpha=0.3)
        ax.xaxis.grid(False)
    ax.set_axisbelow(True)


# ==================================================================
# Chart 1: 三种UKF校准RMSE对比 (R1 + R2)
# ==================================================================
print('生成 Chart 1...')
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
x = np.arange(3)
bw = 0.25

for sc, (ax, r1_vals, r2_vals) in enumerate([
    (axes[0], R1_CAL['turn180'], R1_CAL['turn46.7']),
    (axes[1], R2_CAL['turn180'], R2_CAL['turn46.7']),
]):
    for i, (vals, name, clr) in enumerate([
        (r1_vals, 'turn180', BLUE), (r2_vals, 'turn46.7', RED)]):
        pos = x + (i - 0.5) * bw
        bars = ax.bar(pos, [vals[k] for k in UKF_NAMES], bw,
                      color=PALETTE[:3], edgecolor='white', linewidth=0.8)
        for bar, v in zip(bars, [vals[k] for k in UKF_NAMES]):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height()+0.08,
                    f'{v:.2f}', ha='center', va='bottom', fontsize=8, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(UKF_LABELS)
    ax.legend(['turn180', 'turn46.7'], loc='upper right', fontsize=9)
    style_ax(ax)
    ymax = max(max(r1_vals.values()), max(r2_vals.values()))
    ax.set_ylim([0, ymax * 1.3])

axes[0].set_ylabel('R1 Cal RMSE (km)')
axes[0].set_title('Chart 1a: R1 校准RMSE')
axes[1].set_ylabel('R2 Cal RMSE (km)')
axes[1].set_title('Chart 1b: R2 校准RMSE')
fig.suptitle('Chart 1: 三种UKF校准后RMSE对比', fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart1_ukf_rmse.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 2: 融合增益对比
# ==================================================================
print('生成 Chart 2...')
fig, axes = plt.subplots(1, 2, figsize=(11, 5))

for sc, (D, scene_name, ax) in enumerate([
    (D1, 'turn180', axes[0]), (D2, 'turn46.7', axes[1])]):
    fus_mean = D['s_rmse_fus_best'].mean()
    imm_r1 = R1_CAL[scene_name]['imm']
    imm_r2 = R2_CAL[scene_name]['imm']

    labels = ['R1 (imm)', 'R2 (imm)', 'Fusion']
    vals = [imm_r1, imm_r2, fus_mean]
    colors = [BLUE, RED, GREEN]

    bars = ax.bar(labels, vals, color=colors, edgecolor='white', linewidth=1.2, width=0.5)
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height()+0.3,
                f'{v:.1f}', ha='center', va='bottom', fontsize=10, fontweight='bold')

    imp_r1 = (1 - fus_mean / imm_r1) * 100
    imp_r2 = (1 - fus_mean / imm_r2) * 100
    ax.text(2, max(vals)*0.7, f'R1: {imp_r1:+.1f}%\nR2: {imp_r2:+.1f}%',
            ha='center', va='center', fontsize=10, fontweight='bold',
            color='red', bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.3))

    ax.set_ylabel('RMSE (km)')
    style_ax(ax)
    ax.set_ylim([0, max(vals) * 1.5])

fig.suptitle('Chart 2: 融合增益对比 (IMM单站 vs 最佳融合)', fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart2_fusion_gain.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 3: 最佳UKF获胜分布
# ==================================================================
print('生成 Chart 3...')
fig, axes = plt.subplots(1, 2, figsize=(10, 5))

for sc, (D, scene_name, ax) in enumerate([
    (D1, 'turn180', axes[0]), (D2, 'turn46.7', axes[1])]):
    counts = np.zeros(3)
    for i in D['best_ukf']:
        idx = int(i) - 1
        if 0 <= idx < 3:
            counts[idx] += 1

    x = [0]
    bottom = 0
    for i, (cnt, clr, lbl) in enumerate(zip(counts, PALETTE[::-1], UKF_LABELS)):
        ax.barh(x, cnt, height=0.5, left=0, color=clr, label=lbl,
                edgecolor='white', linewidth=1)
        pct = cnt / 200 * 100
        ax.text(cnt/2, x[0], f'{int(cnt)} ({pct:.0f}%)',
                ha='center', va='center', fontsize=11, fontweight='bold', color='white')

    ax.set_xlim([0, 210])
    ax.set_ylim([-0.6, 0.6])
    ax.set_xticks([])
    ax.set_ylabel('场景')
    ax.legend(loc='lower right', fontsize=9)
    style_ax(ax, grid=False)

fig.suptitle('Chart 3: 最佳UKF获胜分布 (200次MC)', fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart3_best_ukf.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 4: 关联率与NIS门控率
# ==================================================================
print('生成 Chart 4...')
fig, axes = plt.subplots(2, 2, figsize=(13, 9))

metrics = [
    ('s_assoc_R1', 's_assoc_R2', 'R1', 'R2', '关联率 (%)'),
    ('s_gate_R1', 's_gate_R2', 'R1', 'R2', 'NIS门控率 (%)'),
]

for r, (k1, k2, rad1, rad2, ylabel) in enumerate(metrics):
    for c, (D, scene_name) in enumerate([(D1, 'turn180'), (D2, 'turn46.7')]):
        ax = axes[r, c]
        vals1 = D[k1].mean()
        vals2 = D[k2].mean()
        std1 = D[k1].std()
        std2 = D[k2].std()

        x = [0, 1]
        bars = ax.bar(x, [vals1, vals2], yerr=[[std1],[std2]], capsize=6,
                      color=[BLUE, RED], edgecolor='white', linewidth=1.2,
                      width=0.5, alpha=0.85)
        for bar, v in zip(bars, [vals1, vals2]):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height()+0.5,
                    f'{v:.1f}', ha='center', fontsize=9, fontweight='bold')

        ax.set_xticks(x)
        ax.set_xticklabels([rad1, rad2])
        ax.set_ylabel(ylabel)
        ax.set_title(f'{rad1} vs {rad2}')
        style_ax(ax)

fig.suptitle('Chart 4: 关联质量与NIS门控率分析', fontweight='bold', fontsize=14, y=1.01)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart4_assoc_nis.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 5: 跟踪寿命与中断
# ==================================================================
print('生成 Chart 5...')
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

mtl_keys = ['s_mtl_R1', 's_mtl_R2', 's_mtl_fus']
brk_keys = ['s_brk_R1', 's_brk_R2', 's_brk_fus']
mtl_labels = ['MTL R1', 'MTL R2', 'MTL Fusion']
brk_labels = ['Brk R1', 'Brk R2', 'Brk Fusion']

for sc, (D, scene_name, ax) in enumerate([
    (D1, 'turn180', axes[0]), (D2, 'turn46.7', axes[1])]):

    # MTL
    mtl_means = [D[k].mean() for k in mtl_keys]
    ax.barh([2, 1, 0], mtl_means, color=[BLUE, RED, GREEN],
            edgecolor='white', height=0.6)
    for i, v in enumerate(mtl_means):
        ax.text(v + 2, i, f'{v:.0f}', va='center', fontsize=9, fontweight='bold')

    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(mtl_labels)
    ax.set_xlabel('跟踪寿命 (帧)')
    ax.set_title(f'MTL - {scene_name}')
    style_ax(ax)

    # Breaks (twin)
    ax2 = ax.twinx()
    brk_means = [D[k].mean() for k in brk_keys]
    ax2.barh([2, 1, 0], brk_means, color=[ORANGE, PURPLE, GRAY],
             edgecolor='white', height=0.6, alpha=0.7)
    for i, v in enumerate(brk_means):
        ax2.text(v + 0.02, i, f'{v:.2f}', va='center', fontsize=9)

    ax2.set_yticks([0, 1, 2])
    ax2.set_yticklabels(brk_labels)
    ax2.set_ylabel('中断次数')
    ax2.set_title(f'Breaks - {scene_name}')
    style_ax(ax2)

fig.suptitle('Chart 5: 跟踪寿命与中断对比', fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart5_mtl_breaks.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 6: RMSE分布箱线图
# ==================================================================
print('生成 Chart 6...')
fig, axes = plt.subplots(2, 2, figsize=(14, 9))

box_configs = [
    (D1['s_rmse_ukf_R1'], 'turn180 R1'),
    (D2['s_rmse_ukf_R1'], 'turn46.7 R1'),
    (D1['s_rmse_ukf_R2'], 'turn180 R2'),
    (D2['s_rmse_ukf_R2'], 'turn46.7 R2'),
]

for ax, (data, title) in zip(axes.flatten(), box_configs):
    parts = ax.boxplot(data, whis=1.5, patch_artist=True,
                       medianprops=dict(color='red', linewidth=2),
                       boxprops=dict(facecolor=LIGHT_BLUE, edgecolor='black'),
                       whiskerprops=dict(color='black', linewidth=1.2),
                       capprops=dict(color='black', linewidth=1.2))
    ax.set_xticks([1])
    ax.set_xticklabels([title.split()[1]])
    ax.set_ylabel('RMSE (km)')
    ax.set_title(title)
    style_ax(ax)

fig.suptitle('Chart 6: UKF RMSE分布箱线图 (200次MC)', fontweight='bold', fontsize=14, y=1.01)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart6_boxplot.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 7: 校准效果对比 + 雷达图
# ==================================================================
print('生成 Chart 7...')
fig = plt.figure(figsize=(14, 6))
gs = GridSpec(1, 2, width_ratios=[1, 1])

# 左侧: 校准前后
ax1 = fig.add_subplot(gs[0])
scene_labels = ['turn180', 'turn46.7']
x1, x2 = 0, 1
bw2 = 0.3

raw_r1 = [RAW_R1['turn180'], RAW_R1['turn46.7']]
cal_r1 = [CAL_R1_AVG['turn180'], CAL_R1_AVG['turn46.7']]

ax1.bar(x1-bw2, raw_r1[0], bw2, color=GRAY, edgecolor='white', label='Raw R1')
ax1.bar(x1+bw2, cal_r1[0], bw2, color=BLUE, edgecolor='white', label='Cal R1')
ax1.bar(x2-bw2, raw_r1[1], bw2, color=GRAY, edgecolor='white')
ax1.bar(x2+bw2, cal_r1[1], bw2, color=RED, edgecolor='white')

for bx, v in [(x1-bw2, raw_r1[0]), (x1+bw2, cal_r1[0]), (x2-bw2, raw_r1[1]), (x2+bw2, cal_r1[1])]:
    ax1.text(bx, v+2, f'{v:.0f}' if v > 10 else f'{v:.1f}',
             ha='center', fontsize=8, fontweight='bold')

ax1.set_xticks([x1, x2])
ax1.set_xticklabels(scene_labels)
ax1.set_ylabel('RMSE (km)')
ax1.set_title('R1: Raw vs Cal 均值')
ax1.legend(fontsize=8)
style_ax(ax1)

# 右侧: 雷达图
ax2 = fig.add_subplot(gs[1], projection='polar')
cats = ['Assoc\nR1', 'Assoc\nR2', 'NIS\nGate R1', 'NIS\nGate R2',
        'MTL\nR1/10', 'MTL\nR2/10', 'MTL\nFus/10']
N = len(cats)
angles = np.linspace(0, 2*np.pi, N, endpoint=False).tolist()
angles += angles[:1]

v1 = [D1['s_assoc_R1'].mean(), D1['s_assoc_R2'].mean(),
      D1['s_gate_R1'].mean(), D1['s_gate_R2'].mean(),
      D1['s_mtl_R1'].mean()/10, D1['s_mtl_R2'].mean()/10, D1['s_mtl_fus'].mean()/10]
v1 += v1[:1]

v2 = [D2['s_assoc_R1'].mean(), D2['s_assoc_R2'].mean(),
      D2['s_gate_R1'].mean(), D2['s_gate_R2'].mean(),
      D2['s_mtl_R1'].mean()/10, D2['s_mtl_R2'].mean()/10, D2['s_mtl_fus'].mean()/10]
v2 += v2[:1]

ax2.plot(angles, v1, 'o-', linewidth=2, color=BLUE, label='turn180')
ax2.fill(angles, v1, alpha=0.15, color=BLUE)
ax2.plot(angles, v2, 's-', linewidth=2, color=RED, label='turn46.7')
ax2.fill(angles, v2, alpha=0.15, color=RED)

ax2.set_xticks(angles[:-1])
ax2.set_xticklabels(cats, fontsize=8)
ax2.set_title('综合性能雷达图', pad=20, fontsize=11)
ax2.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=8)
ax2.grid(True, alpha=0.3)

fig.suptitle('Chart 7: 校准效果与综合性能雷达图', fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart7_calibration_radar.png'), dpi=150, bbox_inches='tight')
plt.close(fig)


# ==================================================================
# Chart 8: 融合改善百分比分布
# ==================================================================
print('生成 Chart 8...')
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

for sc, (D, scene_name, ax) in enumerate([
    (D1, 'turn180', axes[0]), (D2, 'turn46.7', axes[1])]):
    imp_r1 = D['s_imp_fus_R1']
    imp_r2 = D['s_imp_fus_R2']
    mask = ~D['s_bad']
    ir1 = imp_r1[mask]
    ir2 = imp_r2[mask]

    ax.scatter(range(len(ir1)), ir1, alpha=0.3, s=15, color=BLUE, label='Fusion vs R1')
    ax.scatter(range(len(ir2)), ir2, alpha=0.3, s=15, color=RED, label='Fusion vs R2')
    ax.axhline(y=ir1.mean(), color=BLUE, linestyle='--', linewidth=1.5,
               label=f'R1 mean: {ir1.mean():.1f}%')
    ax.axhline(y=ir2.mean(), color=RED, linestyle='--', linewidth=1.5,
               label=f'R2 mean: {ir2.mean():.1f}%')

    ax.set_xlabel('Seed Index')
    ax.set_ylabel('Improvement (%)')
    ax.set_title(f'{scene_name}: Fusion Improvement (n={mask.sum()})')
    ax.legend(fontsize=7)
    style_ax(ax)

fig.suptitle('Chart 8: 融合改善百分比分布', fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'chart8_fusion_improvement.png'), dpi=150, bbox_inches='tight')
plt.close(fig)

print(f'\n所有图表已保存到 {RESULTS}/')
print('共8组PNG图表')
