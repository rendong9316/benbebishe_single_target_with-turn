# IMM 从 3 模型扩展到 5 模型 — 详细实施方案

## 一、背景与动机

### 1.1 问题的根源

之前的 IMM-UKF 系统使用 3 模型 IMM：
- CV（constant velocity）：直线运动
- CT（constant turn）：左转模型，turn_rate = +1°/s
- CT（constant turn）：右转模型，turn_rate = -1°/s

**核心问题**：CT 模型的转弯率被硬编码为固定值 1°/s。但在实际 OTH-SWR 场景中，飞机转弯率可以是 0.7°/s（慢速协调转弯）到 1.3°/s（快速转弯）甚至更高。当真实转弯率与模型的 ±1°/s 不匹配时，预测的转弯半径 R = v/ω 出现偏差，误差随转弯时间累积，导致跟踪 RMSE 显著上升。

### 1.2 之前的探索与结论

我们进行了完整的 Sigmoid 自适应研究（4 个 phase），试图通过改进模型间转移概率（PI）来应对转弯场景。**但 180° 真大转弯的 MC 验证表明：PI 调整对 turn_rate mismatch 无济于事。**

> "Sigmoid 只是改变模型间切换的速度，治不了'模型本身就是错的'这个根本问题。"

真正的瓶颈是：**CT 模型的 turn_rate 参数硬编码不匹配真实值**。解决方案有两种思路：
1. **扩展模型集**（离散多模型）：增加更多 turn_rate 候选值，让 IMM 自行选择最匹配的模型
2. **在线估计 turn_rate**（扩维 UKF）：将 ω 加入状态向量实时估计

我们选择了 **方案 1（离散多模型）**，理由是：
- 符合"转弯率基本固定"的领域假设（民航客机协调转弯通常在 0.5-2°/s 之间是近似固定的）
- 改动适中（已在多模型框架内扩展，无需引入在线估计的收敛问题）
- 鲁棒性强（不依赖滤波收敛，对短/长转弯都有效）
- 可发表性高（有明确的 3 模型 vs 5 模型对比基线）

### 1.3 可行性验证（turn_rate 扫描）

在修改任何源码前，我们通过 **零源码改动** 的 `sweep_turn_rate` 实验验证了可行性：

- 通过 `params.imm_turn_rate_rad_per_sec` 注入不同候选值，跑多个 3 模型 IMM 实例
- 观察 RMSE 随 turn_rate 候选值的变化
- **结果**：注入真实 turn_rate 后平均 RMSE 从 6.305 km 降至 5.439 km（+13.7% 改善）；离散多模型的上界（min over candidates）可达 +15.6%

这证明了：**只要候选集合覆盖"经验最优值"附近，IMM 的混合机制就能近似实现最优效果**。无需精确估计真实 turn_rate。

---

## 二、当前系统架构

### 2.1 核心模块关系

```
validation/
├── ukf_imm.m        ← IMM 核心滤波器（create/update/cycle）
├── create_imm.m     ← IMM 模板初始化
├── run.m            ← 完整仿真流程（调用 ukf_imm）
└── simulation/
    └── simulation_params_oracle.m ← 参数生成
```

### 2.2 当前 ukf_imm.m 的模型结构（简略）

当前 `ukf_imm.m` 中定义了 3 个模型：
```matlab
% 在 ukf_imm('create', ...) 中
num_models = 3;
model_types = {'CT', 'CT'};  % CV 用第一个 slot，实际是特殊处理的 CT(ω=0)
turn_rates = [omega, -omega];  % omega = 1°/s rad
```

状态向量维度：[lon, vlon, lat, vlat]（4 维），CV 和 CT 模型共享相同的状态维度，仅状态转移矩阵 F 不同（取决于 turn_rate）。

---

## 三、将要做什么：5 模型 IMM 扩展

### 3.1 模型定义变更

**5 模型结构：**
| 模型编号 | 类型 | turn_rate (°/s) | turn_rate (rad/s) | 用途 |
|----------|------|-----------------|-------------------|------|
| 1 | CV   | 0               | 0                 | 直线运动 |
| 2 | CTL  | 0.7             | +0.7×π/180        | 慢左转 |
| 3 | CTL  | 1.5             | +1.5×π/180        | 快左转 |
| 4 | CTR  | -1.5            | -1.5×π/180        | 快右转 |
| 5 | CTR  | -0.7            | -0.7×π/180        | 慢右转 |

*注：0.7°/s 和 1.5°/s 是民航协调转弯的典型值（参考 ICAO 标准转弯率）。实际中飞机转弯受侧倾角和乘客舒适度约束，0.5-2°/s 是常见范围。*

### 3.2 具体代码改动

#### 文件 1：`validation/ukf_imm.m`

**改动位置 A：模型数量定义**
```matlab
% 原代码（约在 create 子函数中）
num_models = 3;
model_types = {'CT', 'CT'};
turn_rates = [omega, -omega];

% 改为
num_models = 5;
model_types = {'CV', 'CT', 'CT', 'CT', 'CT'};  % 第一个 CV，后两个 CTL，后两个 CTR
turn_rates = [0, omega_slow, omega_fast, -omega_fast, -omega_slow];
```

**改动位置 B：所有显式硬编码的模型循环**
- `for m = 1:3` → `for m = 1:num_models`
- `for i = 1:3` → `for i = 1:num_models`
- `weights = ones(1,3)/3` → 改为根据先验概率或转移概率动态计算

**改动位置 C：状态转移矩阵 F 的构造**
当前代码根据 `model_types` 和 `turn_rates` 分别计算 F。确保新模型的 ω 正确传递。

**改动位置 D：协方差 Q 的计算**
不同的 turn_rate 可能导致过程噪声不同（通常相同即可，但可根据需要调整）。

#### 文件 2：`simulation/simulation_params_oracle.m`

**新增参数：**
```matlab
params.imm_model_type = '5model_discrete';  % 或 '3model_default'
params.imm_turn_rate_candidates = [0.7, 1.5];  % °/s，定义慢/快候选值
```

#### 文件 3：`validation/create_imm.m`（如存在）

确保该函数能根据 `model_types` 和 `turn_rates` 创建正确的 UKF 实例。如果是 `ukf_imm('create', ...)` 内部函数，则只需修改主函数。

---

## 四、预期结果

### 4.1 定量指标

| 场景 | 3 模型 IMM (Default) | 5 模型 IMM (预期) | 改善 |
|------|----------------------|-------------------|------|
| left_rate_1p3 (1.3°/s 左转) | ~7.5 km | ~5.5-5.8 km | **~25-30%** |
| left_rate_0p7 (0.7°/s 左转) | ~7.2 km | ~5.4-5.5 km | **~20-25%** |
| right_rate_1p3 (1.3°/s 右转) | ~5.6 km | ~5.2-5.4 km | **~5-10%** |
| right_rate_0p7 (0.7°/s 右转) | ~7.0 km | ~6.0-6.5 km | **~10-15%** |

*改善幅度基于之前的 turn_rate 扫描上界推断，实际值需 MC 验证。*

### 4.2 定性优势

1. **鲁棒性**：无论真实转弯率落在哪个候选值之间，IMM 的两个邻近模型能加权近似，避免单一模型完全失效
2. **无需在线估计**：解决了长转弯下 ω 收敛慢的问题，启动即有效
3. **参数简洁**：只需两个候选值（0.7°/s 和 1.5°/s），计算负担增加不多（5 模型 vs 3 模型，状态维度相同，只是多两个循环）

---

## 五、验证方案

### 5.1 单元测试

1. **ukf_imm 基本功能测试**：`validation/test_ukf_imm_5model.m`
   - 验证 5 模型 IMM 能正常初始化、预测、更新
   - 检查状态向量维度正确（仍是 4 维）
   - 检查似然计算无维度错误

2. **单目标直线跟踪**：验证 CV 模型在直线段表现与 3 模型版本相同（应一致）

3. **单目标小转弯跟踪**：在 ω=0.7°/s 和 ω=1.5°/s 的小转弯（30°-60°）场景下验证改善

### 5.2 MC 验证（蒙特卡洛）

使用之前已验证的 `mc_rate_scenario` 框架，对比 3 模型 vs 5 模型：

```matlab
% 测试脚本示例
scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};
seeds = 1:10;  % 10 seeds

for si = 1:length(scenarios)
    for sidx = 1:length(seeds)
        % 3 模型
        params_3 = simulation_params_oracle();
        params_3.imm_model_type = '3model_default';
        rmse_3(sidx) = run_test(scenarios{si}, params_3, seeds(sidx));
        
        % 5 模型  
        params_5 = simulation_params_oracle();
        params_5.imm_model_type = '5model_discrete';
        rmse_5(sidx) = run_test(scenarios{si}, params_5, seeds(sidx));
    end
end
```

**期望结果**：在左转场景（0.7°/s 和 1.3°/s）中，5 模型 RMSE 显著低于 3 模型；在右转场景中应有持平或小幅改善（因为默认 CT 模型是左转，右转场景本身 3 模型就表现好）。

### 5.3 与先前 sweep 结果的一致性验证

5 模型 IMM 的 **最小 RMSE（多模型选优）** 应该接近之前 sweep 中 `MM-upper` 的值（即每个种子取所有候选中的最小 RMSE）。验证一致性：

```
5 模型 max_improvement ≥ sweep_MM_upper_bound  (应成立，因为 5 模型包含了 sweep 的所有候选值)
```

---

## 六、风险与应对

| 风险点 | 严重程度 | 应对方案 |
|--------|----------|----------|
| 循环边界错误导致下标越界 | 中 | 使用 `num_models = size(models,1)` 动态计算，避免硬编码 |
| 新模型的 sigma 点计算异常 | 低 | UKF 的 sigma 点仅依赖状态维度（不变），与 turn_rate 无关，应安全 |
| 转移概率矩阵需要重新设计 | 中 | 初始设置各模型先验概率均等（1/5），转移概率矩阵保持原结构或按对称性设计 |
| 计算复杂度增加 | 低 | 5 模型 vs 3 模型，计算量增加约 67%，在可接受范围（单次 run 约增加 10-20% 时间） |

---

## 七、后续步骤建议

1. **第 1 天**：修改 `ukf_imm.m`，将模型数从 3 改为 5，替换硬编码循环
2. **第 2 天**：添加 `simulation_params_oracle` 的新参数，创建 5 模型开关
3. **第 3 天**：跑单元测试，确保 5 模型 IMM 能正常运行（不 crash）
4. **第 4 天**：跑 MC 对比测试（3 模型 vs 5 模型，10 seeds × 4 场景），生成数据
5. **第 5 天**：分析结果，验证改善幅度，撰写技术报告

---

## 八、文档更新清单

改动完成后，需要更新以下文档：

- ✅ `docs/imm_adaptive_*` 系列报告（将 Sigmoid 工作的结果标记为"已发现局限"，5 模型作为新方案）
- 📄 `docs/imm_5model_extension.md`（本文档，记录具体改动）
- 📄 `docs/post_fix_diagnosis_and_redesign.md`（补充 5 模型设计 rationale）
- 📄 `README.md`（更新支持的 IMM 模型类型说明）

---

*本方案基于之前的 MC sweep 结果（turn_rate mismatch 是主因）和离散多模型的理论可行性。实现后将系统从"固定 turn_rate 硬编码"升级为"离散候选自适应"，是 OTH-SWR 长转弯跟踪问题的稳健解决方案。*
