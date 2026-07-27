# IMM Sigmoid 自适应修复后状态审计（2026-07-25）

## 一、代码 Bug 修复状态

### 审计报告指出的 3 个 P0 Bug

| Bug | 问题描述 | 修复前代码 | 修复后代码 | 状态 |
|-----|---------|-----------|-----------|------|
| ① 符号反向 | ct_mu 高时反而压低 | `exp(-kp*(...))` | `exp(kp*(...))` (L306) | ✅ 已修复 |
| ② threshold 硬编码 | 用户传 threshold 无效 | `threshold = 0.4` | `get_param_imm(...,'threshold',0.4)` (L303) | ✅ 已修复 |
| ③ power 未读取 | grid search 扫 power 无效 | 完全没引用 params.power | `get_param_imm(...,'power',1.0)` (L304) | ✅ 已修复 |

### 新的发现：hidden multiplier

L305: `kp = imm.adaptive_k * 3;`

- sigmoid 分支的 kp 被乘以 3，但 exp_decay 和 linear 分支直接用 `imm.adaptive_k`
- 这个乘数 3 没有文档说明，也没有参数化暴露给用户
- 意味着网格搜索扫的 k=3 实际是 kp=9, k=7 实际是 kp=21
- **这不一定是 bug（如果设计意图如此），但至少应该被记录**

### .p 文件检查

- `ukf/` 下无编译 .p 文件 → 不会干扰实验
- 仅 `build_truth_scenario.p` 在根目录 → 不影响核心算法
- Phase 3/4 MC 数据确实是用修复后的 .m 源码跑的

---

## 二、数学正确性验证

sigmoid 公式（L306）：
```
slev = base_p / (1 + exp(kp * (ct_mu^power - threshold)))
```

**物理意义检查**：

设 base_p = 0.01, threshold = 0.3, power = 1.0, kp = 9 (k=3 场景)

| ct_mu | kp*(ct_mu-threshold) | exp(...) | slev | 物理含义 |
|-------|---------------------|----------|------|---------|
| 0.0 | -2.7 | 0.067 | 0.0096 | ≈base_p，正常切换 ✓ |
| 0.2 | -0.9 | 0.407 | 0.0070 | 略降 ✓ |
| 0.5 | 1.8 | 6.05 | 0.0014 | 压低 CT→CV ✓ |
| 0.7 | 3.6 | 36.6 | 0.00027 | 大幅压低 ✓ |
| 0.9 | 5.4 | 221 | 4.5e-5 | → 钳位到 0.0005 ✓ |

**方向正确**：ct_mu 越高（转弯置信度越高），CT→CV 转移概率越低 → CT 模型更可能留住。这是论文的核心论点。

**钳位保护**：L316 `max(adj_lev, 0.05*base_p)` 防止压太低。

### 与 exp_decay 对比

exp_decay: `adj_lev = base_p * exp(-kp * ct_mu^power)`
sigmoid: `adj_lev = base_p / (1 + exp(kp*(ct_mu^power - threshold)))`

两者在 ct_mu 高时都减小 p，但形态不同：
- exp_decay 单调指数衰减
- sigmoid 有拐点（threshold），在 ct_mu < threshold 时变化平缓，超过后急剧下降

---

## 三、待执行的验证步骤

### Step 1: MC 重跑验证（需要 MATLAB）

```matlab
% 在 MATLAB 中运行:
cd D:\Desktop\single_target_with-turn
addpath(genpath('.'))
verify_post_fix()    % Quick sanity check: 3 seeds x 4 scenarios
audit_mat_data()     % Re-extract stats from existing .mat files
```

### Step 2: 修正报告数据

从 `audit_mat_data()` 的输出重算以下数字：

- Phase 3 报告中的 win rate (报告 86% vs 实际 ?)
- 最差 case (报告 -2.1% vs 实际 ?)  
- 平均改善 (报告 +8.9% vs 实际 ?)
- 配对 t 检验结果

### Step 3: 决定论文方向

| 情形 | 操作 |
|------|------|
| 修复后 Sigmoid 仍然显著改善 | 可以写论文，使用修复后数据 |
| 修复后改善消失或变差 | 需要重新设计方案，不是符号修复的问题就是机制问题 |

---

## 四、文件清理清单

| 类型 | 数量 | 操作建议 |
|------|------|---------|
| .bak 文件 | 6 | 归档或删除 |
| untracked validation/*.m | ~30 | 保留（实验脚本），或移入专门的 archive/ 子目录 |
| docs/*.md (旧报告) | ~5 | 保留但标记为历史 |
| comparison_plots/*.png | ~11 | 保留 |

建议：
```bash
# 备份旧报告
mv docs/imm_adaptive_phase*.md docs/archive/

# 归档 bak
mkdir backup && mv *.bak ukf/*.bak simulation/*.bak config/*.bak backup/

# 清理 p 文件
rm ukf_imm.p.disabled ukf_imm.p.audit_bak
```

---

## 五、结论

1. **三个 P0 bug 已确认在源码中修复**
2. **符号修复后的数学正确性已通过静态分析验证**
3. **等待 MATLAB 可用后立即运行验证脚本重跑 MC**
4. **所有报告中的数字需要以 .mat 原始数据为准进行修正**

---

*生成：2026-07-25*
*静��分析完成，MC 验证脚本已就绪待运行*
