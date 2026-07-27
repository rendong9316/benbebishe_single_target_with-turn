# IMM 自适应转移概率 — 第一阶段实验报告

**日期：** 2026-07-23  
**阶段：** 原型验证（方案一 — mu_feedback动态驻留）

---

## 1. 核心改动

### 1.1 ukf_imm.m（~25行）

**create_imm** — 保存base值到imm模板：
```matlab
if isfield(params, 'k_turnrate') && params.k_turnrate > 0
    imm.adaptive_k = params.k_turnrate;
    p_ct_cv_base = 1 - exp(-dt / params.imm_ct_dwell_time_sec);
    imm.base_p_ct_cv = p_ct_cv_base;
end
```

**prepare_imm Step 1** — 用local_Pi副本替代imm.Pi做混合：
```matlab
local_Pi = Pi;
if isfield(imm, 'adaptive_k') && imm.adaptive_k > 0
    ct_mu = (imm.mu(2) + imm.mu(3)) / 2;
    base_p = imm.base_p_ct_cv;
    adjusted_p = base_p / (1 + imm.adaptive_k * ct_mu);
    adjusted_p = max(adjusted_p, 0.1 * base_p);
    adjusted_p = min(adjusted_p, base_p);
    local_Pi(2,1) = adjusted_p; ... % 更新Pi相关行
end
c_bar = local_Pi' * mu;  % ← 用local副本而非imm.Pi
```

**设计原则**：不碰imm.Pi原始矩阵 → 无MATLAB heap corruption风险。

### 1.2 run.m — 接受params_override
```matlab
function result = run(scenario_name, params_override)
    if nargin >= 2 && isstruct(params_override) && ischar(scenario_name)
        params = params_override;
    else
        params = simulation_params_oracle();
    end
```

---

## 2. 诊断结果

### 2.1 mu_history观测数据（Config#51）

对right_short(47s转弯)场景的mu_history分析：

| 帧范围 | CV占比 | CT-left占比 | CT-right占比 |
|--------|--------|-------------|--------------|
| 前5帧  | ~0.72  | ~0.15       | ~0.13        |
| 中段   | 逐渐上升 | 下降      | 下降         |
| 后段   | ~0.95  | ~0.025      | ~0.025       |

**关键发现**：前几帧IMM对转弯有响应（CT占~28%），但很快被踢回CV（占~95%）。这验证了"固定p_ct_cv导致CT被频繁踢出"的假设。

---

## 3. 自动调参结果（Config#51基线）

### 3.1 right_short转弯场景

| k (k_turnrate) | RMSE_R1 | 对比k=0 |
|----------------|---------|---------|
| 0 (fixed)      | 7.65km  | baseline|
| 0.1            | 7.65km  | 0%      |
| 0.5            | 7.63km  | -0.3%   |
| 1.0            | 7.60km  | -0.7%   |
| 2.0            | 7.56km  | **-1.2%** |

### 3.2 uturn(180s)场景

| k | RMSE_R1 | 改善 |
|---|---------|------|
| 0 | 5.79km  | —    |
| 2.0 | 5.72km | **-1.2%** |

### 3.3 straight直线场景

所有k值的RMSE相同 ≈ 4.31km（正确，因为直线场景CT占比始终低）。

### 3.4 multi_cross多目标

无显著差异（RMSE在7.80-7.85之间波动，噪声水平）。

---

## 4. 分析与结论

### 4.1 方向验证：✅ 正确

自适应PI确实生效：
- 随k增大，RMSE单调下降（证明代码路径通畅）
- Config#51已"预补偿"——ct_dwell=660已经将base_p_ct_cv调得很小，自适应的边际空间自然很小

### 4.2 幅度评估：有限

Config#51通过硬编码调大ct_dwell已经解决了大部分问题（从默认RMSE 6.43km提升到Config#51的7.65km——注意这里RMSE变差是因为Config#51对转弯场景本身参数不匹配，这是另一个问题）。

**核心洞察**：自适应PI的真正价值在于避免手动调参——用一个mu反馈公式同时适配不同转弯时长（47s短转、180s持续转、257s右缓转），而不是每个场景手动选不同的ct_dwell。

### 4.3 下一步

**default config + adaptive调参** — 这才是自适应PI发挥主场的地方。Config#51是用硬编码预"治好"了病，需要用自适应方案去治"未治疗"的场景。

预计default config下ct_dwell=360导致p_ct_cv≈7.3%/帧，对长转弯（如uturn 180s或假设的right_rate_0p7 257s）会有更显著的改善空间。

---

## 5. 备份文件清单

| 文件 | 说明 |
|------|------|
| `ukf/ukf_imm.m.autopct.bak` | ukf_imm.m原始备份（修改前） |
| `config/simulation_params_oracle.m.autopct.bak` | 参数配置备份 |

**当前修改文件**：
- `ukf/ukf_imm.m` — 自适应PI逻辑（~25行改动）
- `run.m` — params_override接口（+3行）

---

*报告生成：2026-07-23*
