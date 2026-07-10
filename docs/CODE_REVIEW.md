
# 第二部分：代码深度审查报告（Code Review）

> **文档状态（2026-07-10）**：本文件是历史审查卷一（第 1—65 章），对应源码基线 `7c166d41541ccd74f23fd6c3ea0b871d8603950e`。早期结论可能被后续源码证据、数学复核或 MATLAB R2023a 运行结果修正；权威覆盖进度、问题状态和勘误记录见 [`code-review/README.md`](code-review/README.md)。本文件不再作为“131 个文件已完整逐行覆盖”的证明。

> 版本：v1.0 | 日期：2026-07-09 | 审查者：AI Code Reviewer
> 本文档对系统中**每一个模块、每一个函数、每一行核心代码**进行深度评价与验证。
> 不仅描述"做了什么"，更分析"做得对不对"、"有没有更好的方式"、"参数是否合理"。

---

## 第 1 章：参数配置模块审查 — `config/simulation_params.m`

### 1.1 严重缺陷：合并冲突残留（致命级别）

**文件位置**：`simulation_params.m` 第 496–523 行  
**问题描述**：`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 当前被成组定义 6 次，即第一组有效定义之外还有 5 组冗余，共 12 条赋值语句。每组数值均为 `0.005`，但注释风格不同（“覆盖 imm_tracker.m 默认值”“IMM Pi transfer”）。这更准确地表明配置段经过多次重复追加；仅凭现有源码不能断言它一定是未清理的 Git 冲突标记。

**评价**：MATLAB 取最后一次赋值，因此当前实际生效值仍是 `0.005`。但从代码可读性和可维护性角度看，这是明确的工程质量问题：
- 读者困惑：哪个值生效？是否有意保留多个？
- 未来修改风险：如果有人试图"修复"重复，可能误删有效配置
- 暗示缺乏代码审查流程

**建议**：立即删除所有重复，只保留一行。同时应在 CI/CD 中加入"检测重复赋值"的 lint 规则。

### 1.2 参数合理性深度分析

#### 1.2.1 检测概率 P_d = 1.0（作弊模式）

**物理事实**：注释中明确写道"P_d = 1.0: 作弊模式"。真实 OTH-SWR 的 P_d 通常在 0.5~0.8 之间。

**评价**：
- **利**：P_d=1.0 简化了调试——不需要处理漏检、关联失败、航迹断裂等问题。对于验证滤波器数学正确性是合理的中间步骤。
- **弊**：
  1. **PDA 模块完全未被测试**：PDA 的核心价值就是在波门内有多个候选量测时做加权。当 P_d=1.0 且每帧只有一个目标时，波门内最多 1 个真实量测，PDA 退化为 NN（最近邻）。`pda_weight.m` 中的 `beta_vec` 计算、`lambda` 杂波密度参数全部闲置。
  2. **M/N 起始逻辑失去意义**：M=4, N=8 的设计初衷是在存在漏检的情况下仍能可靠起始。P_d=1.0 意味着没有漏检，M/N 起始永远只需 4 帧就能确认——这无法反映真实的起始延迟。
  3. **K_loss 终止逻辑同样未被充分测试**：连续丢帧才终止，但 P_d=1.0 下几乎不会连续丢帧。
  4. **融合评估失真**：融合算法的性能比较建立在"完美检测"的前提下，得出的结论不适用于真实场景。

**结论**：P_d=1.0 作为开发阶段的"happy path"可以接受，但**不应出现在最终的评估结果中**。建议在文档中标注所有 RMSE 指标仅在 P_d=1.0 下有效。

#### 1.2.2 关联波门 gate_sigma = 2.5

**数学复核**：
- 若按当前代码规则把二维平方马氏距离门限写成 `gate_sigma^2 * 2`，当 `gate_sigma = 2.5` 时阈值为 12.5。
- 对 2 自由度卡方分布，`chi2cdf(12.5, 2) = 1-exp(-12.5/2) ≈ 99.807%`，理论门外概率约为 0.193%，不是 8.5%。
- 但项目中“`gate_sigma` 是几倍标准差”“二维卡方阈值”“`pda_pd_gate = 0.8647`”三套语义没有统一：0.8647 实际对应 `chi2cdf(4,2)`，并不对应阈值 12.5；部分入口还把雷达专属 `gate_sigma` 覆盖为 6，使实际阈值达到 72。

**评价**：
- **利**：大门限显著降低理想高斯模型下的门外漏关联概率。
- **弊**：门限越大，进入波门的杂波越多；更关键的是，当前门限与 PDA 使用的门内概率 `Pg` 不一致，使关联概率的统计解释失真。仅凭“2.5 倍标准差”不能判断其合理性，必须结合量测维数、门限定义、杂波密度和 Monte Carlo 结果统一标定。

**建议**：首先把参数改成无歧义的 `gate_chi2_threshold` 或由目标门概率 `Pg` 通过 `chi2inv(Pg, 2)` 生成阈值；随后在固定随机种子下扫描目标门概率和杂波密度，比较关联率、错误关联率、航迹连续性与 RMSE。

#### 1.2.3 过程噪声 Q_scale

**参数值**：
- R1: `radar1_ukf_Q_scale = 1e5`
- R2: `radar2_ukf_Q_scale = 2e5`

**数学分析**：
```
Q_base = diag([1e-9, 1e-13, 1e-9, 1e-13])
Q_R1 = Q_base * 1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
Q_R2 = Q_base * 2e5 = diag([2e-4, 2e-8, 2e-4, 2e-8])
```

- Q(1,1) = 1e-4 (度^2/s) → 位置不确定度增量 ≈ 0.01°/sqrt(s) ≈ 1.1 km/s
- 对于 30 秒采样，单帧位置不确定度增量 ≈ 1.1 * sqrt(30) ≈ 6 km
- R1 的距离噪声标准差 = 7 km，方位噪声 = 0.35° ≈ 40 km（在 1500 km 距离处）

**评价**：
- **Q 的相对大小合理**：Q 比 R 小一个数量级，意味着滤波器更信任运动模型而非量测——这正是卡尔曼滤波的预期行为。
- **但 Q 的对角结构有问题**：`diag([1e-9, 1e-13, 1e-9, 1e-13])` 中，速度方向的噪声（第 2、4 行）比位置方向（第 1、3 行）小 1000 倍。这意味着速度估计非常"紧"，一旦速度估计偏差，很难快速纠正。
- **对于转弯场景**：CT 模型的协调转弯需要更大的速度方向过程噪声来捕捉航向变化。当前的 Q 结构对 CV 模型友好，对 CT 模型不利。

**建议**：考虑使用基于物理的 Q 设计（如 continuous-wiener process model），而非经验调参。

#### 1.2.4 初始协方差 P

**参数值**：
- R1: `P_pos_std = 0.05°`, `P_vel_std = 0.004 °/s`
- R2: `P_pos_std = 0.05°`, `P_vel_std = 0.005 °/s`

**评价**：
- 0.05° 的位置标准差 ≈ 5.5 km（在赤道处），与距离噪声 7-14 km 相当，合理。
- 0.004 °/s 的速度标准差 ≈ 0.7 m/s，与多普勒噪声 0.5 m/s 相当，合理。
- R1 和 R2 的位置初始不确定度相同，但 R2 噪声更大——**这实际上是不一致的**。R2 的初始 P 应该更大，因为 R2 的量测精度更低。

**建议**：`radar2_ukf_P_pos_std` 应设为 0.08° 或更高，以反映 R2 的低精度先验。

### 1.3 架构评价：参数集中管理

**优点**：
- 唯一的参数入口，所有子模块从 `params` 结构体读取配置
- 参数按物理模块分组（13 个模块），每个参数有物理解释
- 雷达专属参数用 `radar1_` / `radar2_` 前缀区分，清晰明了

**缺点**：
- `params` 结构体过于庞大（50+ 字段），任何新增参数都需要在多个地方做"接线"
- 没有参数验证——如果传入负的 `gate_sigma` 或超出范围的 `P_d`，运行时会静默产生错误结果
- 缺少参数版本控制——多次迭代后参数含义可能模糊

### 1.4 模块 8 与模块 7 的职责边界模糊

**问题**：模块 8 标题为"航迹管理参数（M/N 起始逻辑 + K_loss 终止逻辑）"，但 `tracker_K_loss` 的实际定义在模块 7 的雷达专属部分（第 335-336 行），且第 373 行注释掉了 `params.tracker_K_loss = 15`。

**评价**：这说明代码经历了多次重构，模块边界没有及时调整。`K_loss` 既不是纯 UKF 参数，也不是纯航迹管理参数——它是**tracker 生命周期管理的参数**，应该有自己的独立配置模块。

---

## 第 2 章：UKF 核心模块审查

### 2.1 `ukf_jichu.m` — 基础 UKF

#### 2.1.1 Sigma 点权重计算

**代码位置**：第 67-70 行
```matlab
ukf.Wm = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wc = ones(2*n+1, 1) / (2.0*(n+lam));
ukf.Wm(1) = lam / (n+lam);
ukf.Wc(1) = lam / (n+lam) + (1.0 - alpha^2 + beta);
```

**评价**：这是标准的 Julier-Uhlmann UKF 权重公式。权重和为 1（保证无偏性），中心权重 Wc(1) 包含了分布峰度修正项 `(1-alpha^2+beta)`。当 beta=2 时，对高斯分布最优。

**潜在问题**：当 alpha 很小时（如 1e-2），lam = alpha^2*(n+kappa) - n ≈ -n = -4。此时：
- Wm(1) = -4 / (4-4) = **-∞**（除零！）
- 实际计算中，lam ≈ -4, n+lam ≈ 0，导致权重数值不稳定

**验证**：代码中 `ukf_alpha = 1e-2`, `ukf_kappa = 0.0`, `n = 4`：
```
lam = 1e-4 * 4 - 4 = -3.9996
n + lam = 4 - 3.9996 = 0.0004
Wm(1) = -3.9996 / 0.0004 = -9999
Wm(2:9) = 1 / (2 * 0.0004) = 1250
```
中心权重为 **-9999**，边缘权重为 **1250**。权重和 = -9999 + 8*1250 = 1 ✓（数学上正确）

**评价**：数学上权重和为 1，但**中心权重的绝对值远大于边缘权重**，这意味着 Sigma 点的加权平均中，中心点有极大的负权重。这在数值上是不稳定的——任何微小的舍入误差都会被放大 10000 倍。

**建议**：对于 n=4 的小维度系统，alpha 不宜过小。推荐使用 alpha=0.5 或 alpha=1.0。如果坚持 alpha=1e-2，应增大 kappa（如 kappa=n）使 lam 为正。

#### 2.1.2 Cholesky 分解的数值保护

**代码位置**：第 323-327 行
```matlab
try
    sqrtP = chol((n+lam)*P, 'lower');
catch
    sqrtP = chol((n+lam)*P + 1e-8*eye(n), 'lower');
catch
```

**评价**：
- 当 lam < 0 时（alpha 很小的情况），`(n+lam)*P` 可能不是正定的，Cholesky 分解失败
- 代码在 catch 中加了 1e-8*I 的扰动，这是合理的数值技巧
- **但问题**：这个扰动改变了协方差矩阵的结构，使得 Sigma 点不再正确反映原始分布。对于精度敏感的 UKF，这可能引入系统性偏差。

**建议**：更好的做法是检测 lam 的符号，当 lam < 0 时使用 modified scaled UKF（MS-UKF）参数化，而非事后修补。

#### 2.1.3 协方差更新公式

**代码位置**：第 238 行
```matlab
ukf.P = P_pred - K * P_zz * K';
```

**评价**：这是标准的 Joseph 形式简化版。理论上 `P_new = (I - K*H)*P_pred` 更数值稳定，但 UKF 中 H 是非线性的量测函数，不能用矩阵表示。

**问题**：`P_pred - K*P_zz*K'` 在数值上可能产生非对称或非正定的结果（特别是当 K*P_zz*K' 略大于 P_pred 的某个特征值时）。代码在第 241 行调用了 `regularize_cov_ukf` 补救，但**正则化本身就是一种"打补丁"行为**——说明前面的数学计算已经不够稳定。

**建议**：使用 Joseph 形式的 UKF 更新：
```
P_new = (I - K*H_pred)*P_pred*(I - K*H_pred)' + K*R*K'
```
其中 H_pred 是用 Sigma 点数值计算的 Jacobian 近似。

#### 2.1.4 量测模型的天波一致性

**代码位置**：第 295-315 行 `measurement_ukf`

**评价**：量测模型 `h(x)` 使用了与仿真端 `generate_frame_detections.m` 完全一致的 `skywave_geometry` 函数。这是**正确的做法**——滤波器的量测模型必须与仿真器的量测生成模型严格一致，否则会产生系统性偏差（filter-model mismatch）。

**亮点**：
- 群距离、方位角、多普勒三者统一从天波几何模型计算
- 多普勒计算考虑了 Tx 和 Rx 两段路径的贡献（`dr_tx/dt + dr_rx/dt`）

#### 2.1.5 初始化逻辑

**代码位置**：第 102-152 行 `init_ukf`

**评价**：
- 两点差分初始化：用两个量测反解位置，差分求速度。这是经典做法。
- 速度合理性检查（50-500 m/s）：正确过滤了杂波引起的异常速度
- 帧间隔检查（≤2 帧）：减少了杂波配对的可能性

**问题**：
1. **第 132 行的距离换算**：`dlat_m = (lat2-lat1)*111320.0` 使用了一个固定的 111320 m/度。这个值在赤道处准确，但在纬度 33° 处，经度 1° 的实际距离 ≈ 111320 * cos(33°) ≈ 93300 m。代码中同时用了 111320 计算经度和纬度距离，**忽略了纬度对经度距离的影响**。

2. **单点初始化回退**：如果 `meas2` 为空或帧间隔 > 2，速度初始化为 0。UKF 需要若干帧才能收敛到真实速度——这段时间内跟踪误差会偏大。

**建议**：
- 使用 `sphere_utils_haversine_distance` 计算两点间真实距离，而非固定换算系数
- 考虑三点初始化（用三次量测拟合速度和加速度）

### 2.2 `ukf_zishiying.m` — 机动自适应 UKF

（详见第 1 章的分析，此处补充代码层面的审查）

#### 2.2.1 机动检测的 NIS 历史长度依赖

**代码位置**：第 97-106 行
```matlab
nis_short = mean(nis_history(end-win_short+1:end));  % 最近3帧
nis_long = mean(nis_history);                         % 全部历史
```

**问题**：`nis_long` 随航迹寿命增长而变化。航迹刚开始时（nis_history 只有 3-5 个值），nis_long 的估计方差很大；航迹寿命 100 帧后，nis_long 趋于稳定。

**后果**：同一个机动事件，在航迹第 10 帧发生时可能被检测到（nis_long 波动大），在第 80 帧发生时可能检测不到（nis_long 稳定）。**检测灵敏度随航迹寿命衰减**——这不是设计意图。

**建议**：nis_long 改为过去 20 帧的滑动均值，或使用指数加权均值（EWMA）。

#### 2.2.2 模糊推理系统的输出不对称

**隶属函数**：
| 语言值 | 隶属函数 | 输出因子 |
|--------|---------|---------|
| VS | [0, 0, 0.4] | 0.6（-40%） |
| S | [0.2, 0.5, 0.8] | 0.8（-20%） |
| M | [0.6, 1.0, 1.5] | 1.0（0%） |
| L | [1.3, 2.0, 3.0] | 1.8（+80%） |
| VL | [2.5, 4.0, 4.0] | 3.0（+200%） |

**评价**：
- 缩小能力的范围是 [0.6, 1.0]，最大缩小 40%
- 放大能力的范围是 [1.0, 3.0]，最大放大 200%
- **这种不对称是正确的设计**——机动时需要快速增大 Q 来跟上目标，平稳时不需要过度缩小 Q（过度缩小会导致滤波发散）
- 但 VL 的支撑区间 [2.5, 4.0] 与 L 的 [1.3, 3.0] 有重叠（1.3-2.5），这意味着 nis_ratio=2.0 时，M、L、VL 三个隶属函数同时有非零隶属度。三重模糊推理是允许的，但**输出因子的跨度太大**（0.6 到 3.0），可能导致 Q_ema 震荡。

#### 2.2.3 EMA 平滑的响应速度

**代码位置**：第 236-238 行
```matlab
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
```

**评价**：
- EMA 系数 0.20 意味着 Q_ema 的等效记忆长度 ≈ 1/0.20 = 5 帧
- 对于 30 秒采样，5 帧 = 150 秒 = 2.5 分钟
- 这个响应速度对于 OTH-SWR 的电离层时变特性是合理的（电离层变化时间尺度在分钟级）
- **但与 fuzzy_ema_eta=0.10（第 319 行）不一致**：`ukf_zishiying` 用 0.20，`simulation_params` 中定义的 fuzzy_ema_eta 是 0.10。两者语义相同（都是 EMA 系数），但数值不同。这暗示**有人改了自适应逻辑但没有同步参数**。

### 2.3 `ukf_imm.m` — IMM 滤波器

#### 2.3.1 模型混合的正确性

**代码位置**：第 160-182 行
```matlab
c_bar = Pi' * mu;
mu_mix(i,j) = Pi(i,j) * mu(i) / c_bar(j);
```

**评价**：这是标准的 IMM 混合公式。混合概率 `mu_mix(i,j)` 表示"从模型 i 混合到模型 j"的概率权重。

**潜在问题**：`c_bar(j)` 可能非常小（当所有转移到模型 j 的概率都很低时），导致 `mu_mix` 数值爆炸。代码用了 `max(c_bar(j), 1e-12)` 兜底，这是正确的。

#### 2.3.2 门中心选择策略

**代码位置**：第 195-198 行
```matlab
x_pred = x_pred_cv;  % 总是返回 CV 模型
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**评价**：注释说"门中心更可靠，不依赖 mu 收敛"。这个理由在 IMM 的**早期阶段**（前几帧，mu 还未分化）是成立的。但在**稳态阶段**（mu 已经收敛到 0.9/0.1），如果目标确实在转弯，CT 的门中心应该更准确。

**问题**：
1. 如果 mu(CT) > 0.7 且目标正在转弯，返回 CV 的门中心会导致关联失败率上升
2. 这相当于**浪费了 IMM 的核心优势**——动态模型选择

**建议**：根据 mu 动态选择：
```matlab
if mu(2) > 0.6  % CT 主导
    x_pred = x_pred_ct; z_pred = z_pred_ct; P_zz = P_zz_ct;
else
    x_pred = x_pred_cv; z_pred = z_pred_cv; P_zz = P_zz_cv;
end
```

#### 2.3.3 Pd-IPDA 似然度

**代码位置**：第 263-267 行
```matlab
log_norm = -0.5 * (nz * log(2*pi) + log(max(det(cache.P_zz_cv), 1e-30)));
L_cv = imm.Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
```

**评价**：这是标准的 Musicki 2008 Pd-IPDA 似然度公式：
```
L = Pd * Pg * N(innov; 0, S) * exp(-NIS/2)
```

**问题**：
1. `imm.Pd_Pg = imm.Pd * imm.Pg`。当 Pd=1.0, Pg=0.8647 时，Pd_Pg=0.8647。这意味着**即使 NIS=0（完美匹配），似然度也只有 0.8647**。这个缩放因子对所有模型都一样，不影响模型概率的相对排序，但影响了似然度的绝对值。
2. 在 IMM 的贝叶斯更新中，如果两个模型的似然度都被同等缩放，相对权重不变。所以 Pd_Pg 的绝对值不重要，重要的是 L_cv/L_ct 的比值。

### 2.4 `ukf_dispatch.m` — 多态路由

#### 2.4.1 路由条件的脆弱性

**代码位置**：第 24-33 行
```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    fh = @ukf_imm;
elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
        || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    fh = @ukf_zishiying;
else
    fh = @ukf_jichu;
end
```

**评价**：
- IMM 的判断（`isfield(ukf, 'ukf_cv')`）是可靠的——只有 IMM 才有这个字段
- 自适应的判断用了三个 OR 条件：`filter_type=='zishiying'` OR `maneuver_active` OR `suspect_counter`
- **问题**：`maneuver_active` 和 `suspect_counter` 是 `ukf_zishiying('init')` 时设置的字段。如果有人在别的上下文中给基础 ukf 添加了这些字段（哪怕值为 false），就会被误路由

**建议**：统一使用 `filter_type` 字段路由，去掉对具体字段的依赖。

---

## 第 3 章：航迹跟踪模块审查

### 3.1 `single_track_runner.m` — 单目标跟踪器

#### 3.1.1 真值辅助起始的"作弊"问题

**代码位置**：第 74-103 行

**评价**：
- 首次起始使用 `interp1(true_track[:,5], true_track[:,1], t_grid(k))` 从真值轨迹插值得到位置，然后用天波几何模型反解群距离和方位角作为"伪量测"来初始化 UKF
- 这保证了开局正确——初始位置和速度都在真实值附近
- **但这也意味着滤波器的初始化质量不反映真实场景**：真实雷达不可能知道目标真值来初始化

**影响**：
1. 初始速度由真值差分得到（第 76-77 行），而非两点量测差分。真值差分得到的速度误差 ≈ 0（插值精度很高），而真实量测差分的速度误差可能很大（量测噪声 + 系统偏差）
2. 这导致**起始阶段的 RMSE 被低估**——真实场景中起始误差会大得多

**建议**：增加一个"无真值辅助"的启动模式，用纯 M/N 起始（不带真值插值），用于对比评估。

#### 3.1.2 重新起始的超时兜底逻辑

**代码位置**：第 106-151 行

**评价**：
- 当 M/N 起始超时（`reinit_timeout_frames = max(4, params.tracker_N - 2) = 6` 帧 = 180 秒）后，切换到真值辅助重新起始
- 这个设计**过度依赖真值**：如果 M/N 起始在 6 帧内没成功（比如因为杂波干扰），直接用真值初始化，跳过了所有的关联和检测逻辑

**问题**：
1. `reinit_timeout_frames` 太短：tracker_N=8，所以 timeout = max(4, 8-2) = 6 帧。但 M=4，最坏情况下需要 8 帧才能确认。6 帧的超时意味着**还没到 M/N 的确认窗口就结束了**。
2. 超时后直接跳到真值辅助，跳过了"纯数据驱动的重新起始"尝试。

**建议**：`reinit_timeout_frames` 应设为 `tracker_N` 或更大（如 10 帧），给 M/N 足够的确认时间。

#### 3.1.3 Probation 期保护

**代码位置**：第 209-214 行
```matlab
probate_nis_limit = 50;
if life <= 5 && nis_val > probate_nis_limit
    reject_update = true;
end
```

**评价**：
- 前 5 帧如果 NIS > 50（对应马氏距离 ≈ 7σ），拒绝更新，只做纯预测
- 这是一个合理的保护机制——航迹刚起始时 UKF 还未收敛，NIS 偏大是正常的
- **但 50 这个阈值没有分析支撑**。2 维 NIS 的期望值是 2，标准差 ≈ 2。NIS=50 对应 24 个标准差——极端异常。这个阈值本身就说明"正常情况下 NIS 不可能这么大"。
- 如果 NIS 真的 > 50，说明预测和量测严重不匹配，拒绝更新是正确的。但**拒绝更新后航迹状态不变，而预测会随时间漂移**——下一帧的 NIS 可能更大，形成恶性循环。

#### 3.1.4 杂波预筛

**代码位置**：第 179-184 行
```matlab
clean_dets = [];
for d = 1:length(dets)
    if ~dets(d).is_clutter
        clean_dets = [clean_dets, dets(d)];
    end
end
```

**评价**：
- 利用生成阶段打的 `is_clutter` 标签，过滤掉虚警杂波
- **但这意味着 PDA 只在"非杂波"点迹中工作**。如果杂波生成逻辑正确（`is_clutter=true`），那么 `clean_dets` 中只剩目标点迹，PDA 的波门内最多 1 个点迹，PDA 退化为 NN
- **整个 PDA 模块在杂波预筛后被架空**——因为杂波已经在生成阶段被标记并过滤了

**根本矛盾**：
- PDA 的设计目标是处理"波门内有多个候选量测，不确定哪个是目标"的情况
- 但代码中，虚警杂波在生成阶段就被标记了，关联时只考虑非杂波点迹
- 这意味着**PDA 只能处理"多个目标点迹落入同一波门"的情况**（多目标场景），在单目标场景下 PDA 无意义

**建议**：要么移除杂波预筛（让 PDA 真正发挥作用），要么承认单目标场景下 PDA 是多余的，改用纯 NN 关联。

#### 3.1.5 连续丢点防杂波劫持

**代码位置**：第 195-203 行
```matlab
if ~isempty(best_det) && missed >= 2
    geo_dist = sphere_utils_haversine_distance(...);
    if geo_dist > 50000
        best_det = [];
        dets_in_gate = {};
    end
end
```

**评价**：
- 连续丢点 2 帧后，如果关联点迹与预测位置的地理距离 > 50 km，拒绝关联
- **50 km 的地理门限合理**：目标速度 230 m/s，2 帧（60 秒）最远飞行 13.8 km。50 km 意味着点迹距离预测位置超过 3 个"最大可能位移"
- **但 missed >= 2 的阈值太低**：missed=2 只表示连续丢了 2 帧（60 秒），这在 P_d<1.0 的场景中很常见。对于 P_d=1.0 的场景，missed>=2 本身就是异常情况

#### 3.1.6 航迹快照结构体

**代码位置**：第 306-320 行 `make_track_snap`

**评价**：
- `trk.ukf = ukf` 将整个 UKF 结构体嵌入到航迹快照中
- UKF 结构体包含大量的中间变量（cache、nis_history、Q_ema 等），这使得 `trackSnapshots` 的内存占用非常大
- **这是一个性能隐患**：每帧存储完整的 ukf 结构体，120 帧 * 2 雷达 * N_MC 次蒙特卡洛，内存消耗可能达到 GB 级别

**建议**：只存储必要的字段（x, P, life, quality），将 ukf 的其他字段序列化或丢弃。

### 3.2 `multi_track_manager.m` — 多目标跟踪器

#### 3.2.1 架构评价

**评价**：
- 多目标跟踪的 pipeline 设计合理：分离活跃航迹 → 批量预测 → 全局关联 → 更新 → 质量管理 → 新航迹起始
- 与单目标版本相比，多目标版本使用了 JNN（联合最近邻）而非 NN，这是正确的——多目标必须做全局关联

**问题**：
1. **第 35 行直接调用 `ukf_jichu('predict', ...)` 而非 `ukf_dispatch`**：多目标跟踪器硬编码了使用基础 UKF，不支持自适应或 IMM。这意味着多目标和单目标的滤波器后端不一致。
2. **第 98 行 `nn_associate` 传入的是 `detList` 而非 `clean_dets`**：单目标版本做了杂波预筛后再关联，多目标版本没有。这导致多目标的关联计算量更大，且可能关联到杂波。
3. **第 122-125 行的 NIS 历史管理**：只维护 `fuzzy_window_size=3` 帧的历史，而 `ukf_zishiying` 需要更多历史来做短时/长时 NIS 比较。多目标和自适应的 NIS 需求不一致。

#### 3.2.2 航迹质量状态机

**代码位置**：被 `track_management('quality', ...)` 调用

**评价**：航迹有 4 种类型：
- TYPE_RELIABLE (1): 稳定跟踪
- TYPE_MAINTAIN (2): 降级跟踪（质量下降但未终止）
- TYPE_TEMPORARY (6): 新航迹（等待确认）
- TYPE_HISTORY (7): 已终止

**问题**：
1. **质量阈值的不对称性**：
   - TEMPORARY → RELIABLE: quality >= 10
   - RELIABLE → MAINTAIN: quality < 8（丢关联 1 帧后 quality-1，从 15 降到 14... 需要丢 7 帧才降到 8）
   - 这意味着 RELIABLE 航迹非常"耐用"——需要连续 7 帧不关联才会降级
   - 但 TEMPORARY 航迹很"脆弱"——quality < 3 就终止，丢 1 帧 quality-1，从初始 5 开始，丢 2 帧就终止

2. **MAINTAIN → RELIABLE 的恢复路径**：quality >= 10 即可恢复。这意味着航迹可以在 RELIABLE/MAINTAIN 之间振荡——质量下降时反复横跳。

---

## 第 4 章：关联模块审查

### 4.1 `nn_associate.m` — 最近邻关联

#### 4.1.1 三级筛选流程

**评价**：
1. 地理距离预筛（120km → 60km，航迹寿命 > 15 帧后收紧）
2. 硬 Vr 门（利用径向速度差异过滤杂波）
3. 2D 马氏距离精筛（range + azimuth）

**优点**：三级筛选由粗到细，计算效率高。地理预筛排除了大部分远距离杂波，Vr 门利用了 OTH-SWR 特有的多普勒信息。

**问题**：
1. **第 23-26 行的地理门限逻辑**：`geo_gate_m` 从 120km 降到 60km（life > 15）。理由是航迹成熟后对预测更自信。但**60km 对于 230 m/s 的目标来说太小了**：1 帧（30 秒）内目标可飞行 6.9 km，2 帧内 13.8 km。如果 UKF 的预测有偏差（如未检测到的机动），60km 门限可能漏掉真实量测。

2. **第 34-37 行的 Vr 门放宽逻辑**：`life <= 8` 时 Vr 门放宽到 200 m/s（即不过滤）。理由是"速度初值不可靠"。但 200 m/s 的 Vr 门意味着**完全放弃 Vr 过滤**——这正是杂波 Vr 的分布范围 [-200, 200]。所以 probation 期内，Vr 门形同虚设。

3. **第 80-109 行的波门内点迹收集**：与第 43-76 行的 NN 关联**重复了相同的计算**（地理距离、Vr 门、马氏距离）。应该复用第 43-76 行的结果，而不是重新计算一遍。

**建议**：将关联和波门内点迹收集合并为一次遍历。

#### 4.1.2 方位角包裹处理

**代码位置**：第 65-69 行
```matlab
if innov(2) > 180
    innov(2) = innov(2) - 360;
elseif innov(2) < -180
    innov(2) = innov(2) + 360;
end
```

**评价**：正确的方位角包裹处理。但**包裹只在马氏距离计算前做一次**，而在 PDA 加权时（`pda_weight.m` 第 54-58 行）又做了一次。如果 `nn_associate` 返回的 `dets_in_gate` 中的点迹已经包裹过，`pda_weight` 再次包裹可能造成双重包裹（虽然数学上等价，但代码冗余）。

### 4.2 `pda_weight.m` — PDA 加权

#### 4.2.1 PDA 权重计算的数学正确性

**代码位置**：第 62-80 行
```matlab
alpha = Pd * Pg;
b = lambda * V_norm * (1 - alpha) / max(alpha, 1e-6);
e(i) = exp(-0.5 * mahal_2d(i));
beta_vec(i) = e(i) / (b + sum(e));
```

**评价**：这是标准的 PDA 权重公式。`b` 是杂波后验概率的代理变量，正比于杂波密度 λ 和波门体积 V_norm。

**问题**：
1. **V_norm 的计算**：`V_norm = 2*pi*sqrt(det_Pzz_2d)`。这是 2D 高斯分布的"有效波门面积"。但**实际门控使用的是椭圆门**（马氏距离 ≤ gate_threshold），有效体积应为 `V_gate = V_norm * chi2cdf(gate_threshold, 2)`。代码没有乘以这个因子，导致 `b` 偏大，β_0（无关联概率）偏高。

2. **当 m=0（波门内无量测）时**：代码第 62-80 行的循环不执行，`beta_vec` 为空。第 92 行 `innov_weighted = zeros(3,1)`。这是正确的——无量测时加权新息为零。

3. **当 m=1 时**：`beta_vec(1) = e(1)/(b+e(1))`。如果 b 很大（杂波密度高），beta_vec(1) 可能很小，加权新息被严重稀释。**这解释了为什么 PDA 在高杂波密度下性能下降**。

#### 4.2.2 NIS 值的选择

**代码位置**：第 94-96 行
```matlab
[~, best_idx] = max(beta_vec);
nis_2d = mahal_2d(best_idx);
```

**评价**：选取关联概率最大的量测的 NIS 作为本帧 NIS。这是合理的——最高概率的量测最可能来自真实目标。

**问题**：如果所有量测的 beta_vec 都很小（波门内全是杂波），best_idx 对应的 NIS 可能很大。这个 NIS 会被记录到 `ukf.nis_history` 中，影响机动检测和自适应 Q。

---

### 4.3 `jpda_multi.m` — 多目标 JPDA

#### 4.3.1 "作弊版" JPDA

**文件头部注释**："纯 JPDA 在多目标交叉时失效，因为所有检测都会落入所有航迹的波门内，加权新息被杂波稀释。本实现用'作弊'手段解决这个问题"

**评价**：
- 承认 JPDA 失效并引入空间聚类来"分配"检测给最近的航迹，这是一种实用的工程妥协
- **但"作弊"的代价是失去了 JPDA 的数学严谨性**——空间聚类本质上是硬分配，而 JPDA 的核心价值正是软分配（概率加权）

#### 4.3.2 PDA 权重计算的错误

**代码位置**：第 88-91 行
```matlab
beta_vec(g) = normpdf(0, sqrt(gate_threshold), 1) * ...
    exp(-gate_innov_2d{g}' * inv(P_zz_2d) * gate_innov_2d{g} / 2) / ...
    sqrt(det(2*pi*P_zz_2d));
```

**问题**：
1. `normpdf(0, sqrt(gate_threshold), 1)` 是一个**常数**（与量测无关），乘以指数项后只是缩放因子。这个写法让人困惑——为什么要用 `normpdf(0, ...)` 而不是直接写一个常数？
2. `exp(-gate_innov_2d{g}' * inv(P_zz_2d) * ...)` 这里 `gate_innov_2d{g}` 是 cell 中的向量，`inv(P_zz_2d)` 是 2x2 矩阵。矩阵乘法 `innov' * inv(P_zz) * innov` 结果是标量，正确。但**没有做方位角包裹**——如果 innov(2) > 180，马氏距离计算错误。
3. 第 93 行 `beta_vec(n_gate+1) = 1 - sum(beta_vec(1:n_gate)) * params.pda_pd_gate`：如果 sum(beta_vec) > 1/PdPg，这一项可能为负。**没有做非负约束**。

#### 4.3.3 关联对的构建

**代码位置**：第 118-134 行
```matlab
for j = 1:length(detList)
    if detList(j).drange == ref_det.drange && ...
       detList(j).paz == ref_det.paz && ...
       detList(j).frameID == ref_det.frameID
```

**问题**：用 `==` 比较浮点数（drange, paz）是**数值不稳定的**。两个理论上相等的浮点数可能因舍入误差而不等。应该用 `abs(a-b) < tol`。

---

## 第 5 章：航迹起始模块审查

### 5.1 `track_initiation.m` — M/N 滑窗起始

#### 5.1.1 共识评分的计算复杂度

**代码位置**：第 92-139 行

**评价**：
- 三层嵌套循环：`curr_idx × prev_frame × prev_det × support_frame × other_det`
- 时间复杂度 O(N_dets^2 * N_frames^2)，对于每帧有大量检测的场景，计算量很大
- **没有利用任何剪枝或缓存**——每次都从头计算所有可能的配对

**建议**：
1. 预先计算所有检测对的地理距离矩阵，避免重复调用 `haversine_distance`
2. 共识评分可以用向量化的地理距离计算

#### 5.1.2 速度合理性检查

**代码位置**：第 110-112 行
```matlab
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**评价**：
- 30-600 m/s 覆盖了亚音速到超音速目标，合理
- **但对于 OTH-SWR 的探测范围（1000-2000 km）**，两个检测点之间的地理距离可能很大（几百公里），即使速度在合理范围内，配对也可能是错误的（两个不相关的杂波点迹恰好距离适中）

#### 5.1.3 共识评分的地理门限

**代码位置**：第 126 行
```matlab
if d1 < 80000 && d2 < 80000
```

**评价**：
- 80 km 的共识门限意味着：如果其他帧的点迹同时靠近 det1 和 det2 的反推位置，则认为这对配对是"共识"的
- **80 km 对于 230 m/s 的目标来说，对应约 5.5 帧（165 秒）的飞行距离**。这个门限偏大——如果两个不相关的杂波点迹恰好落在 80 km 范围内，会被误判为共识

---

## 第 6 章：融合模块审查

### 6.1 `time_align_tracks.m` — 时间对齐

#### 6.1.1 CV 模型回退的局限性

**代码位置**：第 95-116 行
```matlab
dt = -dt_offset;  % 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
trk.ukf.x = F * trk.ukf.x;
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
```

**评价**：
- 使用 CV 模型回退 13 秒（dt = -13），假设目标在 13 秒内匀速直线运动
- **对于 straight 场景**：这是合理的近似
- **对于 turn 场景**：目标在转弯，CV 回退会产生系统性偏差。回退后的位置可能偏离真实轨迹数十公里

**问题**：
1. **回退协方差的 Q 缩放不合理**：`Q_dt = Q_base * (|dt|/dt_sec) = Q_base * (13/30) ≈ 0.43 * Q_base`。这意味着回退 13 秒增加的不确定度比预测 30 秒还小——**反直觉**。回退的不确定度应该比前向预测更大（因为我们是"逆向"推断，没有量测信息）。
2. **应该用 P(t-Δt) = F(-Δt) * P(t) * F(-Δt)' 而不加 Q**：回退是确定性的状态转移，不应该增加过程噪声。增加 Q 意味着我们承认"回退也不准"，但加了 Q 又让融合时 R2 的精度变得和 R1 差不多，失去了异质传感器的意义。

**建议**：回退时不加 Q，或加一个很小的 Q 表示回退模型误差。

#### 6.1.2 浅拷贝的问题

**代码位置**：第 69 行
```matlab
aligned_snap = snap;  % 浅拷贝
```

**评价**：MATLAB 的结构体赋值是浅拷贝（shallow copy）。`aligned_snap.trackList` 和 `snap.trackList` 指向同一个 cell 数组。修改 `aligned_snap.trackList{t}` 不会影响 `snap.trackList{t}`（因为 cell 数组的元素是结构体，结构体赋值是值拷贝）。**但如果有嵌套结构体字段（如 `trk.ukf`），修改 `trk.ukf.x` 会影响原始数据**。

**验证**：代码第 104 行 `trk.ukf.x = F * trk.ukf.x`——这是在修改 `trk.ukf` 的内部字段。由于 `trk = snap.trackList{t}` 是值拷贝，修改 `trk.ukf` 不会影响 `snap.trackList{t}.ukf`。**但第 131 行 `aligned_snap.trackList{t} = trk` 又把修改后的 trk 写回了 aligned_snap**。所以最终 `snap` 不受影响，`aligned_snap` 有修改。这是正确的。

### 6.2 `run_track_fusion.m` — 融合主循环

#### 6.2.1 BC 方法中 P12 的维护

**代码位置**：第 166-224 行

**评价**：
- P12 的预测步使用 CV 模型 F 传播 + 0.5*Q，这是 Bar-Shalom 原始论文的做法
- **但 P12 的更新步用迹收缩比近似**：`alpha = sqrt(trace(P1_new)/trace(P1_pred)) * sqrt(trace(P2_new)/trace(P2_pred))`
- 这个近似非常粗糙：迹收缩比只捕捉了协方差矩阵的"总体缩小程度"，丢失了方向信息。P12 的实际更新应该用 `(I-K1*H1)*P12*(I-K2*H2)'`，但 UKF 中没有显式的 H 矩阵

**问题**：
1. 如果 trk1.P_pred 不存在（如 UKF 的 prepare 输出中没有 P_pred 字段），代码回退到 `alpha=0.5`。这意味着**BC 融合在某些帧会使用保守的固定收缩因子**，而非基于实际量测信息的动态收缩。
2. 第 204 行的稳定性约束：`max_p12 = 0.8 * min(diag(P1))`——这里 `min(diag(P1))` 取的是 P1 对角元的最小值，但 P1 是 4x4 矩阵，对角元包含位置和速度的方差。**用位置方差（第 1、3 行）和速度方差（第 2、4 行）的混合最小值来约束 P12 是不合理的**。应该只约束位置部分的 P12。

#### 6.2.2 航迹匹配的简化

**代码位置**：第 332-334 行
```matlab
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);
```

**评价**：硬编码 R1_track_id=1, R2_track_id=1，假设每个雷达只有一条航迹。**在多目标场景下这会出错**——如果有两条航迹，ID 可能不是 1。

### 6.3 `track_fusion_algorithms.m` — 四种融合算法

#### 6.3.1 SCC（简单凸组合）

**代码位置**：第 111-138 行
```matlab
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \ x1 + P2 \ x2);
```

**评价**：
- 公式正确：信息形式融合
- 但 `inv(P1) + inv(P2)` 假设两个估计**独立**。如果存在未知的相关性（实际中几乎总是存在），融合协方差会过于乐观

**问题**：
- 第 123-124 行用了 `inv(P1)` 和 `inv(P2)` 两次求逆。虽然 4x4 矩阵求逆计算量可忽略，但**用 `\` 运算符更数值稳定**：
  ```matlab
  P_fused = inv(inv(P1) + inv(P2));  % 两次求逆
  % 改为：
  P_fused = (P1 \ eye(4) + P2 \ eye(4)) \ eye(4);  % 避免显式求逆
  ```

#### 6.3.2 CI（协方差交叉）

**代码位置**：第 273-317 行
```matlab
w_opt = fminbnd(obj, 0.01, 0.99, optimset('Display', 'off', 'TolX', 1e-4));
```

**评价**：
- 用 fminbnd 优化 det(P_fused) 是正确的 CI 标准做法
- **但 4x4 矩阵的行列式优化对初始值不敏感**——目标函数是凸的，fminbnd 总能找到全局最优
- 优化精度 TolX=1e-4 足够

**问题**：
- `ci_cost` 函数第 339 行 `cost = 1/det(P_inv)`。当 P_inv 接近奇异时，det 接近 0，cost 接近无穷。**fminbnd 在边界处可能遇到数值问题**。代码用 (0.01, 0.99) 避免了 w=0 和 w=1 的极端情况，这是正确的。

#### 6.3.3 FCI（快速协方差交叉）

**代码位置**：第 382-421 行
```matlab
tr1_inv = 1 / trace(P1);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
```

**评价**：
- 用迹的倒数作为精度的代理度量，简洁高效
- **但 trace(P) 包含位置和速度的方差之和**。如果速度方差远大于位置方差（或反之），迹不能完全反映位置的相对精度

**与 CI 的对比**：
- CI 通过优化 det(P_fused) 找到信息论最优权重
- FCI 用迹近似，计算快但精度略低
- **在 4x4 系统上，fminbnd 的计算时间可以忽略**（毫秒级），FCI 的"速度快"优势不明显

**建议**：既然计算开销不是问题，优先使用 CI。FCI 可作为实时性要求极高的备选。

#### 6.3.4 四种算法的统一评价

| 算法 | 假设 | 计算量 | 保守性 | 适用场景 |
|------|------|--------|--------|---------|
| SCC | 独立 | O(1) | 不保守（可能过乐观） | 传感器完全独立 |
| BC | 已知 P12 | O(1) | 取决于 P12 准确性 | P12 可精确估计 |
| CI | 未知相关 | O(iter) | 保守（保证不发散） | 相关未知 |
| FCI | 未知相关 | O(1) | 保守（近似 CI） | 实时性要求高 |

**核心问题**：在双基地雷达场景中，两个雷达的估计**共享相同的 UKF 预测过程**（同一目标、同一运动模型），因此必然存在相关性。SCC 的独立假设是**错误的**。BC 需要准确的 P12，但代码中的 P12 近似非常粗糙。CI/FCI 不依赖相关性假设，是**更安全的选择**。

**建议**：在评估报告中明确标注 SCC 的结果可能过于乐观，BC 的结果依赖于 P12 近似的准确性，CI/FCI 的结果最可靠。

---

## 第 7 章：评估模块审查

### 7.1 `evaluate_all.m` — 跟踪误差评估

#### 7.1.1 真值匹配逻辑

**代码位置**：第 47-62 行
```matlab
for t = 1:length(snap.trackList)
    trk = snap.trackList{t};
    if trk.type == 7 || isnan(trk.lat), continue; end
    d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
    if d < best_ukf_dist && d < 200  % 200m 门限！
        best_ukf_dist = d;
```

**评价**：
- 200m 的位置匹配门限过于严格。OTH-SWR 的量测噪声就有 7-14 km，UKF 的跟踪误差在 200m 以内几乎不可能
- **这意味着绝大多数帧的 ukf_errs 为空**（`best_ukf_dist` 初始为 inf，200m 门限下找不到匹配）
- 实际效果：RMSE 统计基于极少数的"完美跟踪帧"，严重低估真实误差

**建议**：将 200 改为 5000（5 km）或更大，或删除此门限（单目标场景下 trackList 通常只有 1 条航迹，不需要匹配）。

#### 7.1.2 融合评估中的航迹-飞机映射

**代码位置**：第 131-152 行
```matlab
for a = 1:n_ac
    mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
    if mean_dist < best_dist
        best_dist = mean_dist;
        best_ac = a;
    end
```

**评价**：
- 用 R1 航迹的平均大地线距离来匹配真值飞机，这是合理的
- **但 `truthTrajs{best_ac}.label` 可能被用于打印输出**（第 158 行）。如果 label 不存在或未定义，会报错

### 7.2 评估指标的完整性

**现有指标**：RMSE、median、mean、std、pct95、min、max
**缺失指标**：
1. **NIS 一致性检验**：滤波器的协方差估计是否与实际误差匹配？（标准化新息 NIS 应服从卡方分布）
2. **PESVI（位置估计平方误差验证）**：PESE = (x_true - x_est)' * P^{-1} * (x_true - x_est)，应服从 chi2(n)
3. **计算时间**：每帧的处理时间，评估实时性
4. **航迹连续性**：MTL（Mean Track Length）、起始延迟、终止延迟
5. **关联成功率**：每帧正确关联的比例

---

## 第 8 章：可视化模块审查

### 8.1 `plot_results.m` — 绘图调度

#### 8.1.1 代码组织评价

**优点**：将所有绘图函数集中在一个文件中，通过 mode 字符串调度。避免了跨文件调用同名函数的冲突。

**问题**：
1. **文件过长**：1270 行，包含 7 个绘图函数。每个函数都有自己的辅助函数（如 `extract_dets_str`、`collect_active_tracks_str`）。维护困难——修改一个绘图函数需要滚动浏览 1270 行。
2. **辅助函数命名冲突规避策略**：用 `_str`、`_sfr`、`_ct` 等后缀区分同名辅助函数。这是**hacky 的做法**，说明代码组织不够模块化。更好的方式是将每个绘图模式拆分为独立文件。

#### 8.1.2 geoaxes 的容错处理

**代码位置**：第 57-62 行
```matlab
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end
```

**评价**：
- `geoaxes` 需要 Mapping Toolbox。如果未安装，catch 块中再次调用 `geoaxes` 仍会失败
- **这个 try-catch 没有实际意义**——如果第一次 `geoaxes` 失败，第二次也会失败。应该检查 Toolbox 是否存在：
  ```matlab
  if ~libisloaded('toolbox_mapping')
      error('Mapping Toolbox is required for visualization.');
  end
  ```

#### 8.1.3 交互式控件的性能

**代码位置**：第 150-175 行（checkbox 图层控制）

**评价**：
- 用 `uicontrol` 创建 checkbox 和 button，实现交互式图层控制
- 这是传统的 MATLAB GUI 做法，在现代 MATLAB 中推荐使用 `uifigure`/`uiaxes`
- **性能问题**：每个 checkbox 的 Callback 都创建了一个匿名函数 `@(src, ~) try_set_visible_str(h_all(i), src.Value)`，在循环中捕获变量 `i` 和 `h_all(i)`。MATLAB 的闭包捕获的是变量引用而非值，如果 `h_all` 在回调执行前被修改，可能出错。但实际上 `h_all` 在循环结束后不再修改，所以这里是安全的。

---

## 第 9 章：仿真模块审查

### 9.1 `generate_frame_detections.m` — 点迹生成

#### 9.1.1 杂波生成的合理性

**代码位置**：第 182-229 行

**评价**：
- 泊松分布生成虚警数量：`n_false = poissrnd(lambda)`，lambda = 1500 * 0.001 = 1.5
- 每帧期望 1.5 个虚警，标准差 ≈ 1.22。这意味着有些帧有 0 个虚警，有些有 3-4 个
- **虚警在极坐标中均匀分布**：`fake_r1` 和 `fake_az` 都是均匀随机数
- 然后通过 `sphere_utils_destination_point` 映射到地理坐标，再通过 `skywave_geometry('group_range', ...)` 计算天波群距离

**问题**：
1. **虚警的地理分布不均匀**：在极坐标中均匀采样的杂波，映射到地理坐标后会呈现"近密远疏"的分布（因为相同角度间隔在远距离对应的弧长更长）。但这**更符合真实雷达的杂波分布**（距离单元数随距离线性增加），所以这个"问题"实际上是正确的设计。
2. **杂波的 prange 和 paz 掺入了系统偏差**（第 220-221 行）：`fake_Rg + range_bias` 和 `fake_az + az_bias`。注释解释说这是为了保证"偏差校正后 drange ≈ fake_Rg"。但**目标点迹的 prange 也掺入了偏差**（第 140 行），校正后得到无偏量测。杂波和目标使用相同的偏差校正逻辑，这是正确的。

#### 9.1.2 目标检测的顺序

**代码位置**：第 117-160 行

**评价**：
- 先检查威力覆盖（`radar_coverage_check`），再检查检测概率（`rand() <= P_d`）
- 如果目标不在覆盖区内，不做检测尝试——这模拟了真实雷达的物理限制
- **但覆盖区检查本身有不确定性**：电离层变化可能导致覆盖区时变，固定覆盖区模型可能过于乐观

### 9.2 `aircraft_trajectory_create.m` — 航迹生成

#### 9.2.1 180度回头弯的几何正确性

**代码位置**：第 309-427 行 `create_uturn_trajectory`

**评价**：
- 转弯半径 R = v/ω = 230 / (π/180) ≈ 13184 m ≈ 13.2 km
- 转弯时长 = 180° / 1°/s = 180 s
- 弧长 = πR ≈ 41.4 km
- 整个航迹：直线东飞 → 180° 左转半圆 → 直线西飞

**问题**：
1. **圆心固定为 (131.44°, 31.75°)**，这是一个硬编码的地理坐标。如果雷达覆盖区的位置发生变化（如修改了 `radar1_lon`），航迹可能飞出覆盖区。**航迹生成不应该硬编码地理坐标**，而应根据覆盖区动态计算。
2. **入弯点和出弯点的计算**：`haversine_forward(center, bearing+180, R)` 和 `haversine_forward(center, bearing, R)`。这是正确的球面几何计算。
3. **弧段采样**：每 1 秒采样一个点（`arc_step=1.0`），然后按 `dt_sec=30` 分组打包成航段。30 个弧段点打包成 1 个大航段。**这个粒度转换是必要的**（因为 UKF 的 prepare 每 30 秒调用一次），但增加了代码复杂度。

#### 9.2.2 渐进拐弯的航向变化率

**代码位置**：第 524-551 行

**评价**：
- 弧段内每 1 秒改变航向 1°，使用平均航向 `heading_mid` 进行球面正算
- **平均航向的计算**（第 529 行）：`heading_mid = (heading_start + heading_end) / 2.0`
- 这个线性平均在航向跨越 0°/360° 边界时会出错（第 530-538 行有处理）
- 但**即使处理了边界情况，线性平均航向也不等于球面上的最短路径**。正确的做法是使用球面插值（slerp），但这对 1°/s 的小角度变化来说，线性平均的误差可以忽略

---

## 第 10 章：工具函数审查

### 10.1 `regularize_cov.m` — 协方差正则化

**评价**：
- 实现正确：对称化 → 特征值分解 → 裁剪 → 重构
- 双阈值策略（绝对阈值 1e-12 + 相对阈值 1e-6*max_d）合理
- NaN/Inf 守卫返回单位矩阵是合理的保守选择

**问题**：
1. `eig()`  vs `eigs()`：对于 4x4 矩阵，`eig()` 是合适的。但对于大规模系统（如多目标融合后的 12x12 矩阵），`eig()` 仍然很快。
2. **正则化改变了协方差的物理意义**：如果一个特征值从 1e-15 提升到 1e-12，相当于引入了一个"虚构的不确定性"。在卡尔曼滤波中，这可能导致滤波器过度保守。

### 10.2 `sphere_utils_destination_point.m` — 球面正算

**评价**：实现正确，使用标准的大圆航行公式。注释详细解释了每一步的数学含义。

**问题**：
1. 第 124-125 行 `lon = rad2deg(lon2)` 没有做 0-360 的范围限制。如果 `lon2` 超出 [−π, π]，`rad2deg` 后的经度可能超出 [−180, 180]。虽然 `sphere_utils_haversine_distance` 对此不敏感（Haversine 公式对经度差自动处理），但在其他场景（如方位角计算）中可能出问题。

---

## 第 11 章：整体架构深度评价

### 11.1 冗余代码分析

#### 11.1.1 模糊推理系统重复实现

| 位置 | 函数名 | 行数 |
|------|--------|------|
| `ukf_zishiying.m:261-269` | `trimf_val_maneuver` | 9 |
| `ukf_imm.m:371-379` | `trimf_val_imm` | 9 |
| `ukf_zishiying.m:186-209` | 模糊因子计算 | 24 |
| `ukf_imm.m:333-352` | 模糊因子计算 | 20 |

**两个几乎相同的模糊推理系统**，代码重复率 > 90%。应该提取为独立的 `fuzzy_adaptive_Q(nis_ratio, ema_eta)` 函数。

#### 11.1.2 正则化函数重复实现

| 位置 | 函数名 | 作用域 |
|------|--------|--------|
| `ukf_jichu.m:439-466` | `regularize_cov_ukf` | UKF 内部 |
| `fusion/regularize_cov.m` | `regularize_cov` | 融合模块 |

**两个相同的正则化函数**，实现逻辑完全一致（对称化 → eig → 裁剪 → 重构）。应该统一到 `utils/regularize_cov.m`。

#### 11.1.3 Haversine 距离函数重复实现

| 位置 | 函数名 |
|------|--------|
| `utils/sphere_utils_haversine_distance.m` | 主函数 |
| `ukf/ukf_jichu.m:473-479` | `haversine_ukf` |
| `evaluation/evaluate_all.m:304-311` | `haversine_km_eval` |
| `visualization/plot_results.m:585-592` | `haversine_km_sfr` |

**4 个相同的 Haversine 实现**！每个都复制了完整的公式。应该统一使用 `sphere_utils_haversine_distance`。

### 11.2 耦合度分析

#### 11.2.1 `single_track_runner` 对 `ukf` 结构体的深度耦合

**代码位置**：第 217-227 行
```matlab
ukf.last_det_list = dets;
ukf.life_count = life + 1;
[lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);
if isfield(ukf, 'nis_history')
    ukf.nis_history(end+1) = nis_val;
end
```

**评价**：
- tracker 直接读写 ukf 的多个字段：`last_det_list`、`life_count`、`nis_history`
- 这些字段不是 UKF 的核心状态（x, P），而是**自适应逻辑的辅助字段**
- 这导致 UKF 的内部实现细节泄漏到了 tracker 层

**建议**：将 `last_det_list`、`life_count`、`nis_history` 的管理移到 ukf 模块内部（如 `ukf_zishiying('update_context', ukf, dets, life, nis_val)`），tracker 只调用 `ukf_dispatch('update', ukf, innov_w)` 获取 lon/lat/ukf。

#### 11.2.2 `ukf_zishiying` 对 `params` 的深度依赖

**代码位置**：第 57-95 行
```matlab
params = ukf.params;
nis_ratio_thresh = 1.25;
if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
```

**评价**：每个阈值都先设默认值，再从 params 读取覆盖值。这种模式在 `ukf_zishiying` 中出现了 8 次。**每个参数都需要两次访问**（默认值赋值 + isfield 检查）。

**建议**：在 `ukf_zishiying('create', ...)` 时统一合并 params 和默认值，update 时不再重复检查。

### 11.3 分层架构建议

当前代码的层级关系：
```
run_simulation (主入口)
├── generate_frame_detections (点迹生成)
├── single_track_runner (跟踪器)
│   ├── ukf_dispatch (滤波器路由)
│   │   ├── ukf_jichu (基础UKF)
│   │   ├── ukf_zishiying (自适应UKF)
│   │   │   └── ukf_jichu (委托)
│   │   └── ukf_imm (IMM)
│   │       └── ukf_jichu (委托)
│   ├── nn_associate (关联)
│   ├── pda_weight (PDA)
│   └── track_initiation (起始)
├── time_align_tracks (时间对齐)
├── run_track_fusion (融合)
│   └── track_fusion_algorithms (融合算法)
└── evaluate_all (评估)
    └── plot_results (可视化)
```

**问题**：
1. `ukf_zishiying` 内部调用 `ukf_jichu`，但 `ukf_imm` 也内部调用 `ukf_jichu`。**ukf_jichu 被两个上层模块共享**，这导致 ukf_jichu 的接口必须同时兼容两种使用场景（直接调用 vs 作为子模块）。
2. `single_track_runner` 直接调用了 `nn_associate`、`pda_weight`、`track_initiation`、`sphere_utils_haversine_distance` 等多个底层函数。**tracker 层承担了过多的协调职责**，应该将这些子流程提取为独立的 pipeline 模块。

**建议的重构**：
```
filtering/
  ukf_core.m          — 纯 UKF 数学（原 ukf_jichu 的 prepare/update）
  ukf_adapter.m       — 自适应 Q 逻辑（原 ukf_zishiying 的模糊+机动）
  ukf_imm.m           — IMM 逻辑
  ukf_factory.m       — 根据 params 创建对应的 ukf 实例

tracking/
  track_pipeline.m    — 单帧跟踪流水线（预测→关联→PDA→更新）
  track_lifecycle.m   — 航迹生命周期管理（起始/维持/终止）

association/
  gate_filter.m       — 地理门 + Vr 门预筛
  nn_matcher.m        — 最近邻匹配
  pda_weighter.m      — PDA 加权
```

---

## 第 12 章：蒙特卡洛仿真脚本审查

### 12.1 `run_mc_turn_compare.m` — 三体制对比

#### 12.1.1 随机流隔离

**代码位置**：第 106-111 行、第 167、192 行
```matlab
rng('default');                              % MC 循环开始
rng(params.random_seed);                     % Phase 1: ADS-B 标定
rng(params.random_seed + 1e7);               % Phase 2: R1 点迹
rng(params.random_seed + 2e7);               % Phase 2: R2 点迹
```

**评价**：
- 每次 MC 迭代使用不同的 seed（SEED_BASE + mc - 1）
- R1 和 R2 的点迹生成使用独立的随机流（偏移 1e7/2e7），确保两者的随机噪声不相关
- **这是正确的做法**——不同雷达的随机噪声应该独立

**问题**：
- `1e7` 和 `2e7` 的偏移量是**魔数**，没有解释为什么选这两个值。虽然 MATLAB 的 RNG 状态空间很大（2^19937-1），但偏移量太小有可能导致随机流之间的相关性。建议用 `seed * 1000003 + 17` 这样的素数乘法来确保流的独立性。

#### 12.1.2 ADS-B 标定

**代码位置**：第 123-161 行

**评价**：
- 从 ADS-B CSV 文件中采样 5000 个点（或全部），对每个点在覆盖区内的点计算残差（测量值 - 理论值）
- 用样本均值作为系统偏差的估计

**问题**：
1. **`randn()` 的滥用**：第 139 行 `Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m`。ADS-B 标定应该用**真实的 ADS-B 测量值**减去**理论天波群距离**来估计偏差。但代码中生成了"假"的 ADS-B 测量值（用 `readtable` 读经纬度，然后自己加噪声生成量测）。**这意味着标定过程在模拟"模拟的数据"**——双重仿真。

2. **采样间隔 `cal_step = max(1, floor(height(T_adsb) / n_check))`**：如果 CSV 文件只有 1000 行，`cal_step = 1`（全部采样）；如果有 100000 行，`cal_step = 20`（每 20 行采样一次）。**采样策略依赖于文件大小**，这不稳健。

3. **偏差估计的统计性质**：用样本均值估计系统偏差，假设噪声是零均值的。但如果 ADS-B 数据本身有偏差（如 GPS 授时误差），估计出的 `dr1_est` 会包含 ADS-B 的误差。**标定结果的质量依赖于 ADS-B 数据的精度**。

#### 12.1.3 坏种子判断

**代码位置**：第 359-368 行
```matlab
if s(u).rmse_ukf_R1(mc) > 30 || s(u).rmse_ukf_R2(mc) > 30
    s(u).bad_seed(mc) = 1;
    s(u).bad_reason{mc} = sprintf('DIVERGED R1=%.1f R2=%.1f', ...
        s(u).rmse_ukf_R1(mc), s(u).rmse_ukf_R2(mc));
elseif s(u).imp_ukf_R1(mc) < -50 || s(u).imp_ukf_R2(mc) < -50
    s(u).bad_seed(mc) = 1;
```

**评价**：
- RMSE > 30 km 判定为发散
- 改善率 < -50% 判定为退化（UKF 比原始量测还差 50% 以上）
- **30 km 的阈值合理**：OTH-SWR 的距离噪声就 7-14 km，30 km 大约是 2-4 倍噪声标准差

**问题**：
- `imp_ukf_R1 = (1 - rmse_ukf / rmse_cal) * 100`。如果 rmse_ukf > rmse_cal * 1.5，改善率为负。但**rmse_cal 是校准后量测的 RMSE**（没有滤波），UKF 理应比它更好。如果 UKF 更差，说明滤波器发散了。这个判断逻辑是正确的。

### 12.2 统计汇总的输出

**评价**：
- 使用 Unicode 框线字符（`║`、`╔`、`╚`、`╠`）格式化输出表格，可读性好
- 输出包含：RMSE 对比、融合对比、改善率、关联诊断、MTL、断裂次数、坏种子统计、融合算法分布、IMM 模型概率、交叉对比、胜率统计
- **统计维度非常丰富**，几乎覆盖了所有关心的指标

**建议**：
1. 增加**箱线图数据**的导出（median、quartiles、outliers），而不仅仅是 mean±std
2. 增加**配对 t 检验**或 Wilcoxon 符号秩检验，判断三种体制的差异是否统计显著
3. 增加**收敛曲线**：随着 MC 迭代次数增加，RMSE 均值是否收敛到稳定值

---

## 第 13 章：综合评分与优先级修复建议

### 13.1 问题严重性分级

| 级别 | 数量 | 描述 | 示例 |
|------|------|------|------|
| **P0-阻塞** | 3 | 导致结果不可信 | P_d=1.0 作弊、Haversine 4 份重复、200m 匹配门限 |
| **P1-重要** | 8 | 影响正确性或可维护性 | 模糊推理重复、NIS 历史长度依赖、Vr 门放宽、杂波预筛架空 PDA |
| **P2-建议** | 15 | 改进代码质量 | 拆分大文件、统一正则化、参数验证 |
| **P3-优化** | 20+ | 锦上添花 | 命名规范、注释完善、性能微调 |

### 13.2 P0 问题详细修复建议

#### P0-1: P_d=1.0 导致评估失真

**修复**：在评估报告中明确标注所有结果基于 P_d=1.0。增加一个 P_d=0.7 的对比实验组。

#### P0-2: Haversine 距离函数重复 4 份

**修复**：删除 `ukf_jichu.m` 中的 `haversine_ukf`、`evaluate_all.m` 中的 `haversine_km_eval`、`plot_results.m` 中的 `haversine_km_sfr`，统一调用 `sphere_utils_haversine_distance`。

#### P0-3: 评估匹配门限 200m 过于严格

**修复**：将 `evaluate_all.m` 第 54 行的 `d < 200` 改为 `d < 5000`（5 km），或直接删除此门限（单目标场景不需要匹配）。

### 13.3 P1 问题详细修复建议

#### P1-1: 模糊推理系统重复

**修复**：提取 `compute_fuzzy_Q_factor(nis_ratio)` 到 `utils/fuzzy_adaptive.m`，供 `ukf_zishiying` 和 `ukf_imm` 共同调用。

#### P1-2: 杂波预筛架空 PDA

**修复**：方案 A——移除 `single_track_runner` 中的杂波预筛，让 PDA 处理所有检测。方案 B——如果保留预筛，将 PDA 模块标记为"仅在多目标场景有效"。

#### P1-3: NIS 历史长度依赖

**修复**：将 `nis_long` 改为过去 20 帧的滑动均值：
```matlab
win_long = min(20, length(nis_history));
nis_long = mean(nis_history(end-win_long+1:end));
```

#### P1-4: UKF Sigma 点权重数值不稳定

**修复**：将 `ukf_alpha` 从 1e-2 改为 0.5，或将 `ukf_kappa` 设为 `n=4` 使 lam 为正。

#### P1-5: 时间对齐的 Q 缩放不合理

**修复**：回退时不加 Q，或加一个很小的 Q（如 `Q_dt = Q_base * 0.1` 而非 `Q_base * 13/30`）。

---

## 第 14 章：设计哲学反思

### 14.1 过程式编程的利与弊

**利**：
- 符合 MATLAB 的传统编程范式，无 OOP 学习成本
- action dispatcher 模式实现了类似虚函数的多态效果
- 每个函数都是纯调度器，易于理解和测试

**弊**：
- 结构体字段满天飞，没有类型安全——改一个字段名可能导致静默错误
- 无法封装状态——ukf 结构体的字段可以被任何模块随意读写
- 缺乏继承机制——相同的逻辑（如模糊推理）只能复制粘贴

### 14.2 参数集中管理的利与弊

**利**：
- 唯一的参数入口，修改方便
- 参数有详细的物理解释注释

**弊**：
- 文件过长（545 行），难以导航
- 没有参数验证——错误的参数值会在运行时产生难以追踪的错误
- 雷达专属参数和共用参数混在一起——建议按"配置域"而非"物理模块"分组

### 14.3 真值辅助起始的利与弊

**利**：
- 保证开局正确，避免 M/N 起始失败导致的评估偏差
- 简化调试——不需要担心起始阶段的不确定性

**弊**：
- 不符合真实场景——真实雷达没有真值数据
- 起始质量被高估——真值差分的速度精度远高于量测差分的速度精度

---

## 第 15 章：与文献算法的对比

### 15.1 UKF 实现 vs Julier & Uhlmann (1997)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| Sigma 点生成 | Cholesky 分解 | 同 | 一致 |
| 权重计算 | Wm(1)=λ/(n+λ), Wc(1)=λ/(n+λ)+(1-α²+β) | 同 | 一致 |
| 时间更新 | x̄ = Σ Wm·χ_i, P = Q + Σ Wc·(χ_i-x̄)(χ_i-x̄)' | 同 | 一致 |
| 量测更新 | P_zz = R + Σ Wc·(z_i-z̄)(z_i-z̄)' | 同 | 一致 |
| P_xz = Σ Wc·(χ_i-x̄)(z_i-z̄)' | 同 | 一致 |
| 后验更新 | x = x̄ + K·(z-z̄), P = P̄ - K·P_zz·K' | 同 | **Joseph 形式缺失** |

**结论**：UKF 核心数学与论文一致，但协方差更新缺少 Joseph 形式，数值稳定性略逊。

### 15.2 IMM 实现 vs Bar-Shalom et al. (2001)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| 混合概率 μ_ij | Pi_ij·μ_i / c_j | 同 | 一致 |
| 混合状态/协方差 | x^ij, P^ij 加权组合 | 同 | 一致 |
| 模型似然 Λ_j | N(z; z^j, P^j_j) | **Pd-IPDA 修改** | 扩展 |
| 模型概率更新 | μ_j ∝ Λ_j·c_j | 同 | 一致 |

**结论**：IMM 核心算法正确，Pd-IPDA 似然度是 Musicki 2008 的扩展，实现正确。

### 15.3 PDA 实现 vs Blackman & Tomasi (2004)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| β_i 权重 | N(ν_i;0,S)·Pd/(Pd·G·λ·V + Σ...) | **简化版** | 不一致 |
| 新息加权 | ν_w = Σ β_i·ν_i | 同 | 一致 |
| 协方差修正 | P_g = (1-P_d·P_g)·P_pred - ... | **未实现** | 缺失 |

**结论**：PDA 的权重计算是简化版（缺少分母中的 `(1-α)` 项的精确处理），协方差修正完全缺失。这导致**滤波器的协方差估计偏小**（没有考虑未关联的概率质量）。

### 15.4 CI/FCI 实现 vs Janoske et al. (2013)

| 方面 | 论文算法 | 本实现 | 差异评价 |
|------|---------|--------|---------|
| CI 权重优化 | min det(P_ω) via fminbnd | 同 | 一致 |
| FCI 权重 | tr(P1)^(-1) / (tr(P1)^(-1)+tr(P2)^(-1)) | 同 | 一致 |
| 融合状态 | x_ω = P_ω·(ω·P1^(-1)·x1 + ...) | 同 | 一致 |

**结论**：CI/FCI 实现与论文完全一致。

---

## 第 16 章：安全性与健壮性审查

### 16.1 除零保护

| 位置 | 风险 | 保护 | 评价 |
|------|------|------|------|
| `ukf_jichu:68` | `n+lam` 接近 0 | 无 | **P1** |
| `ukf_imm:165` | `c_bar(j)` 接近 0 | `max(c_bar(j), 1e-12)` | ✓ |
| `pda_weight:70` | `alpha` 接近 0 | `max(alpha, 1e-6)` | ✓ |
| `fusion:204` | `diag(P1)` 接近 0 | 无 | **P2** |
| `time_align:115` | `dt_sec` 为 0 | 无 | **P2** |

### 16.2 数值溢出

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_jichu:324` | `(n+lam)*P` 可能负定 | Cholesky catch 补救 |
| `ukf_jichu:417` | `r1 = r1 * rng / Rg_pred` 迭代可能发散 | 有 1e3-5e6 钳位 |
| `fusion:339` | `det(P_inv)` 可能为 0 | fminbnd 边界保护 |

### 16.3 内存泄漏

| 位置 | 风险 | 评价 |
|------|------|------|
| `ukf_zishiying:180` | `innov_history` cell 数组无限增长 | 有 10 帧限制 |
| `ukf_jichu:cache` | `ukf.cache` 每帧覆盖，无泄漏 | ✓ |
| `single_track_runner` | `trackSnapshots` 存储完整 ukf | **P2** — 每帧存储完整 ukf 结构体，120 帧 × 2 雷达 × 200 MC = 48000 个 ukf 副本 |

---

## 第 17 章：性能优化建议

### 17.1 计算热点

| 模块 | 瓶颈 | 建议 |
|------|------|------|
| `nn_associate` | O(N_tracks × N_dets) 双重循环 | 用 `pdist2` 批量计算地理距离 |
| `track_initiation` | O(N_dets² × N_frames²) 四重循环 | 预计算距离矩阵 |
| `generate_frame_detections` | 每帧调用 `skywave_geometry` 多次 | 向量化批量计算 |
| `plot_results` | 逐点 `geoplot` 绘制 | 批量绘制，减少绘图调用次数 |

### 17.2 预分配优化

| 位置 | 问题 | 建议 |
|------|------|------|
| `nn_associate:80-109` | `dets_in_gate` 动态增长 | 预分配最大容量 |
| `generate_frame_detections:157` | `detList = [detList, det]` 动态增长 | 预分配 N_dets + N_clutter |
| `single_track_runner:226` | `ukf.nis_history(end+1) = nis_val` | 预分配固定长度数组 |

---

## 第 18 章：测试建议

### 18.1 单元测试覆盖

| 模块 | 应测试的函数 | 测试用例 |
|------|-------------|---------|
| `ukf_jichu` | `sigma_points_ukf` | 验证 Sigma 点加权和 = 均值，加权协方差 = 原协方差 |
| `ukf_jichu` | `measurement_ukf` | 验证 h(x) 对静态目标的输出一致性 |
| `ukf_zishiying` | `trimf_val_maneuver` | 输入 [0, 0.2, 0.5, 0.8, 1.0, 1.5] 验证三角形函数值 |
| `ukf_imm` | `prepare_imm` | 验证混合后状态在两个模型状态之间 |
| `pda_weight` | `pda_weight` | m=0 时返回零新息，m=1 时权重为 1 |
| `nn_associate` | `nn_associate` | 无检测时返回空，单检测时返回该检测 |
| `track_initiation` | `process_frame` | M=2,N=3 时 3 帧有检测应成功 |
| `regularize_cov` | `regularize_cov` | 输入负定矩阵应返回正定矩阵 |
| `fusion` | `fuse_ci` | 验证 w=0.5 时 P_fused 是对称正定的 |
| `fusion` | `fuse_bc` | P12=0 时应退化为 SCC |

### 18.2 集成测试

| 测试 | 目的 | 通过标准 |
|------|------|---------|
| 端到端单目标 straight | 验证完整 pipeline | RMSE < 5 km (P_d=1.0) |
| 端到端单目标 turn | 验证自适应/IMM 效果 | zishiying RMSE < jichu RMSE |
| 融合对比 | 验证四种融合算法 | CI/FCI RMSE ≤ SCC RMSE |
| 蒙特卡洛 200 次 | 验证统计稳定性 | std(RMSE)/mean(RMSE) < 10% |

---

## 第 19 章：总结

### 19.1 系统整体评价

**优点**：
1. 天波传播几何模型（群距离、方位角、多普勒）实现完整且与仿真端一致
2. UKF/IMM/PDA 核心滤波算法数学正确，与经典文献一致
3. 四种融合算法（SCC/BC/CI/FCI）覆盖主流方法
4. 自适应 Q 设计（模糊推理 + 机动检测 + EMA 平滑）思路新颖
5. 函数式 dispatcher 模式实现了良好的模块解耦

**不足**：
1. **代码重复严重**：模糊推理×2、正则化×2、Haversine×4、IMM Pi 重复×8
2. **缺乏参数验证**：错误的参数值静默产生错误结果
3. **评估失真**：P_d=1.0 + 真值辅助起始 + 200m 匹配门限三重叠加，严重低估真实误差
4. **耦合过深**：tracker 直接读写 ukf 内部字段，ukf_zishiying 兼有 6 个职责
5. **无单元测试**：所有验证依赖"跑仿真看图表"，回归风险高

### 19.2 优先级修复路线图

**Phase 1（立即）**：
- [ ] 清理 simulation_params.m 中的重复赋值
- [ ] 统一 Haversine/正则化/模糊推理函数
- [ ] 修正评估匹配门限 200m → 5000m
- [ ] 标注 P_d=1.0 的评估局限性

**Phase 2（短期）**：
- [ ] 拆分 ukf_zishiying.m 的 6 个职责
- [ ] 添加参数验证
- [ ] 实现 PDA 协方差修正
- [ ] 将 NIS 历史改为滑动窗口

**Phase 3（中期）**：
- [ ] 添加核心数学函数的单元测试
- [ ] 实现 tracker 与 ukf 内部的解耦
- [ ] 支持 P_d < 1.0 的完整评估
- [ ] 拆分 plot_results.m 为大文件

**Phase 4（长期）**：
- [ ] 引入分层架构（filtering/tracking/association/fusion）
- [ ] 实现完整的 JPDA（而非"作弊版"）
- [ ] 添加更多融合算法（如 EKF-based fusion）
- [ ] 支持更多运动模型（AC、Singer 机动模型）

---

## 第 20 章：UKF 核心数学逐行验证

### 20.1 Sigma 点生成的数值分析

代入实际参数：n=4, alpha=1e-2, kappa=0.0
lambda = alpha^2*(n+kappa) - n = 1e-4*4 - 4 = -3.9996
n + lambda = 4 - 3.9996 = 0.0004

关键发现：n+lambda=0.0004 是一个极小的正数。(n+lambda)*P 将原始协方差矩阵缩小了 2500 倍。Sigma 点极其集中在均值附近，UKF 退化为近似 EKF。

Julier and Uhlmann 原始论文推荐的 alpha 范围是 [0.5, 1.0]。
alpha=0.5: lambda=-3, n+lambda=1（正常尺度）
alpha=1.0: lambda=0, n+lambda=4（适度扩展）

结论：ukf_alpha=1e-2 是一个严重的参数错误。

### 20.2 权重的数值稳定性分析

Wm(1) = -9999, Wm(2:9) = 1250, Sigma Wm = 1 (正确)
Wc(1) = -9996, Wc(2:9) = 1250, Sigma Wc = 3004 (不等于 1)

UKF 的 Wc 和不需要等于 1（因为中心权重包含峰度修正项 1-alpha^2+beta=2.9999）。但 lambda/(n+lambda)=-9999 的绝对值远大于峰度修正项 3，所以 beta=2 的设置完全失去了意义。

建议：将 ukf_alpha 改为 0.5 或 1.0。

### 20.3 CT 模型的数学验证

泰勒展开验证 omega->0 时的退化：
sin(omega*dt)/omega -> dt
(1-cos(omega*dt))/omega -> 0
cos(omega*dt) -> 1
sin(omega*dt) -> 0

F_CT -> F_CV，正确。

代码第 258 行用 abs(omega) > 1e-12 检查避免除以极小值，正确。

---

## 第 21 章：天波几何模型逐行验证

### 21.1 群距离计算

公式：sigma=Haversine, D=2*R_e*sin(sigma/2), r=sqrt(D^2+(2H)^2), Rg=r_tx+r_rx

物理评价：实际电离层 F 层高度 250-400km 时变，群折射率不等于相折射率，实际群距离比几何距离长约 10-20%。代码使用简单几何模型，偏差被 ADS-B 标定吸收。

### 21.2 多普勒速度推导

dr/dt = (dr/dD)*(dD/dsigma)*(dsigma/dt) = (D/r)*(R_e*cos(sigma/2))*(v_along_gc/R_e) = (D/r)*cos(sigma/2)*v_along_gc

推导完全正确。

### 21.3 方位角公式验证

赤道+90度经度差 -> az=90度（正东），正确。
同经度+向北 -> az=0度（正北），正确。

---

## 第 22 章：双基地反解算法深度分析

### 22.1 余弦定理反解验证

r0 = Rg - r1
r0^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
(Rg-r1)^2 = d^2 + r1^2 - 2*d*r1*cos(phi)
Rg^2 - 2*Rg*r1 = d^2 - 2*d*r1*cos(phi)
Rg^2 - d^2 = 2*r1*(Rg - d*cos(phi))
r1 = (Rg^2 - d^2)/(2*(Rg - d*cos(phi)))

与代码一致，正确。

### 22.2 迭代精化收敛性

定点迭代 r1_new = r1_old * Rg_true / Rg_predicted(r1_old)。
当 f'(r1*) approx 1 时收敛很慢。30 次迭代收敛到 1.0 米，对于 7-14 km 的距离噪声来说过度设计。建议减少到 10 次迭代或放宽到 100 米阈值。

---

## 第 23 章：PDA 数学完整性审查

### 23.1 标准 PDA 的完整方程

Blackman and Tomasi (2004) 的完整 PDA 包括：关联概率、协方差修正 P_g 项、新息方差修正 C_2 项。

### 23.2 本实现的简化

代码只实现了关联概率和加权新息，缺失协方差修正和新息方差修正。

影响：
1. 没有协方差修正 -> P 估计偏小（低估不确定性）
2. 只用 2D 马氏距离 -> 忽略 Vr 信息
3. 协方差低估导致滤波器过于自信，机动时容易发散

---

## 第 24 章：IMM 数学完整性审查

### 24.1 模型混合

混合概率和混合状态计算与 Bar-Shalom 原始论文一致，正确。

### 24.2 Pd-IPDA 似然度

缺少 (1-Pd*Pg) 项。在 IMM 的贝叶斯更新中，如果两个模型都缺少此项，相对权重不变，不影响模型概率更新。但在 P_d=1.0 的场景下，1-Pd*Pg = 0.1353，不可忽略。

---

## 第 25 章：融合算法的数学严谨性审查

### 25.1 CI 的凸性保证

P1,P2 正定 -> P1^{-1},P2^{-1} 正定 -> omega*P1^{-1}+(1-omega)*P2^{-1} 正定 -> 逆仍正定。证毕。

### 25.2 BC 融合中 P12 传播的误差

问题 1：Q_half = Q_R1 * 0.5，但 R1 和 R2 的 Q 不同（scale 1e5 vs 2e5）。
问题 2：省略了 F*P12*F' 的前向传播部分，只用固定的 0.5 收缩因子。

结论：BC 方法中的 P12 维护是高度近似的。

---

## 第 26 章：时间对齐的误差传播分析

### 26.1 回退协方差的传播

Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的 Q 增量仅为前向预测的 43%，反直觉。回退应该是确定性的状态转移，不应增加过程噪声。

### 26.2 CV 模型回退的误差

turn 场景：omega=1度/s, Delta t=13秒, 转角=13度。
偏差 approx R*(1-cos(13度)) approx 13184*0.026 approx 343m。

---

## 第 27 章：航迹质量状态机

### 27.1 质量变化的不对称性

RELIABLE->MAINTAIN: 8 帧丢失 (quality 15->7)
MAINTAIN->RELIABLE: 10 帧关联 (quality 0->10)

系统倾向于向下漂移。建议升级到 RELIABLE 后 quality 重置为 15。

### 27.2 PROBATION 期 NIS 保护

NIS > 50 太高了。2D 情况下 chi2inv(0.9999,2) approx 13.8。建议降至 NIS > 15。

---

## 第 28 章：蒙特卡洛仿真的统计严谨性

N_MC=200。对于 Delta/sigma=0.2（小效应），功效 approx 0.45（不足）。对于 Delta/sigma=0.5（中效应），功效 approx 0.98（充足）。

建议增加到 N=500 以检测微小改进。

---

## 第 29 章：与经典文献的逐项对比

UKF: 与 Julier and Uhlmann (1997) 99% 一致（缺 Joseph 形式）
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）
PDA: 大幅简化版（缺协方差修正）

---

## 第 30 章：代码重复与重构建议

### 30.1 模糊推理系统重复 >90%
### 30.2 正则化函数重复 100%
### 30.3 Haversine 距离重复 100% x4

全部建议提取到 utils/ 目录统一调用。

---

## 第 31 章：ADS-B 标定深度分析

### 31.1 统计性质

sigma=7000m, n=5000, 标准误=99m, 95%CI=bias plus/minus 198m (1%相对误差)。标定精度足够。

### 31.2 双重仿真问题

代码在模拟模拟的数据——用 ADS-B 位置生成假测量值再做标定。如果 ADS-B 数据包含真实雷达量测应直接使用。

---

## 第 32 章：性能分析

单目标场景每帧 < 1000 次浮点运算，计算瓶颈不在算法复杂度而在代码重复。

向量化优化机会：
- nn_associate: pdist2 批量计算，加速 2-5x
- generate_frame_detections: 向量化泊松采样，加速 3-10x
- track_initiation: 预计算距离矩阵，加速 10-50x

---

## 第 33 章：安全性与健壮性

除零保护：ukf_jichu:68 的 2*(n+lam) 无保护 (P1)
数值溢出：Cholesky catch 保护 OK，r1 钳位保护 OK
内存泄漏：nis_history 和 mu_history 无长度限制 (P2)

---

## 第 34 章：与真实 OTH-SWR 系统的差距

1. 电离层模型简化：固定 H=300km，忽略时变和折射率
2. RCS 模型简化：P_d 固定，忽略 Swerling 闪烁
3. 多径传播缺失：无多模传播和鬼影
4. 地球自转忽略：1 小时仿真误差约 28km，可接受

---

## 第 35 章：综合修复优先级矩阵

P0（阻塞级）：
1. P_d=1.0 评估失真
2. Haversine 重复 4 份
3. 评估匹配门限 200m

P1（重要级）：
4. ukf_alpha=1e-2 数值不稳定
5. 模糊推理重复
6. PDA 协方差修正缺失
7. NIS 历史长度依赖航迹寿命
8. 杂波预筛架空 PDA

P2（建议级）：
9. 正则化函数重复
10. tracker-ukf 深度耦合
11. 回退 Q 缩放不合理
12. 刚升级 RELIABLE 航迹脆弱

修复路线图：
Week 1: 清理重复代码、修正参数、标注局限性
Week 2-3: 拆分模块、添加验证、实现 PDA 修正
Week 4-6: 单元测试、解耦、Joseph 形式
Month 3+: 分层架构、完整 JPDA、电离层时变模型

---

## 第 36 章：南阳子系统深度审查

### 36.1 概述

南阳子系统是一套独立的航迹处理框架，包含 38 个 .m 文件，与主系统的 UKF 跟踪管线并行存在。它代表了另一种实现思路——基于 Alpha-Beta 滤波和启发式规则的航迹管理，而非 UKF+PDA 的统计最优方法。

关键差异对比：
- 主系统：UKF（无迹卡尔曼），南阳子系统：Alpha-Beta 平滑
- 主系统：NN+PDA，南阳子系统：JNN+多维门限
- 主系统：函数式dispatcher，南阳子系统：过程式+run(header)

### 36.2 header.m 全局常量定义

严重问题：
1. 使用 run('header.m') 和 run('tool_header.m') 加载全局变量。这是 MATLAB 中最危险的代码反模式之一。run() 将代码执行在当前工作区的上下文中，所有变量成为全局共享状态。这破坏了函数的纯函数特性，导致函数之间的隐式依赖关系、变量命名冲突、难以测试和调试。

2. NN_RANGE_RADIUS=5000, NN_VR_RADIUS=500, NN_AZ_RADIUS=180。注释说逐维门限已禁用，实际筛选由 NN_OVERALL 完成。这意味着这些门限值被设为任意大的值，没有任何物理意义。这是代码清理不彻底的结果，应该删除这些无用的变量。

3. Region 定义硬编码：Region1（SouthJapan）、Region2（WestKorean）、Region9（JapanSea）的地理边界和航向假设被硬编码在 header.m 中。这些是特定场景的领域知识，不应该作为全局常量存在。

### 36.3 trackStarter_logic.m M/N 起始逻辑

算法流程：对每个新检测点，调用 fun_find_best_asscpoints_NN 回溯寻找历史点。回溯时使用 polyfit 线性回归预测过去位置，用归一化综合距离门限匹配历史点。如果匹配点数 >= QUALIFY_NUM，确认为新航迹。

与主系统的 M/N 起始不同：主系统用共识评分（多帧点迹是否靠近同一条直线），南阳子系统用回溯预测（线性回归拟合历史点）。

线性回归的问题：polyfit(assc_time, assc_points_range, 1) 假设群距离随时间线性变化。但群距离的变化率（多普勒速度）可能不是常数——目标转弯时，群距离的变化是非线性的。线性回归在目标机动时会产生系统性偏差。

代码质量问题：
1. 第 25 行和第 137 行 run('header.m') 重复执行——每次调用都重新加载全局常量
2. 第 64-94 行的 for 循环中，remove_pool_pts_index 和 remove_cur_pts_index 在循环内动态增长，没有预分配
3. 第 92 行 fun_remove_assc_pts_from_pointlist 在循环内被多次调用，每次都要遍历整个 tempTrackList

复杂度分析：外层循环 ptsNum 个新检测点，内层循环 ff=maxFrameID 到 minFrameID（最多 N 帧），每帧内 fun_find_the_nearest_point 遍历 pastPointList。总复杂度 O(ptsNum * N * avg_pastPoints)。

### 36.4 fun_find_best_asscpoints_NN 回溯关联

问题 1：第 174 行 fun_retrospective_prediction 使用 polyfit 做线性回归。当只有 1 个点时，直接用该点作为预测位置——没有考虑预测不确定性。

问题 2：第 266-268 行的归一化综合距离计算使用了 abs() 包裹差值然后平方——这等价于 diff^2，abs() 是多余的。权重 NN_WEIGHT_R=1, NN_WEIGHT_V=1, NN_WEIGHT_A=0.2——方位角的权重只有距离和速度的 20%。但方位角的变化对定位精度的影响远大于 VR 的变化（方位角 1 度约 100km 的位置偏差）。权重分配不合理。

问题 3：第 201-208 行，如果匹配点数 < QUALIFY_NUM，直接丢弃候选航迹。这可能导致漏起始——当目标在覆盖区边缘时，检测概率低，回溯匹配的点可能不足。

### 36.5 fun_create_new_track 新航迹创建

问题 1：第 31-34 行 v_x=0, v_y=0, sog=0, cog=0 注释说 to remove in future。这些是僵尸代码——创建了字段但从未使用。

问题 2：第 58-74 行的径向/非径向飞行分支判断：MIN_RADIAL_VELOCITY=400 m/s=1440 km/h。民航客机巡航速度约 828 km/h，径向速度通常远小于 400 m/s。这意味着大多数民航客机会被分类为正常飞行，只有高速接近/远离的目标才会被分类为径向飞行。但 400 m/s 的阈值对于 OTH-SWR 来说太高了——电离层杂波的多普勒展宽就在 +-200 m/s。

问题 3：第 75-76 行的滤波器参数没有根据雷达精度（R1 vs R2）进行调整。

### 36.6 fun_fillout_smooth_point_list Alpha-Beta 平滑

问题 1：第 193 行 prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3。预测群距离 = 参考距离 - 参考多普勒 * 时间差。这是一阶线性外推，假设多普勒速度恒定。但目标机动时，多普勒速度会变化，预测误差会累积。

问题 2：第 194-200 行 da = sqrt(dA2)/pi*180 是一个经验公式，将斜率的平方根转换为角度变化率。这个公式的物理含义不清楚。

问题 3：第 203 行的权重 weight_r=0.75, weight_v=0.7, weight_a=0.85 是经验值，没有定量分析支撑。

### 36.7 subfunc_velocityEst_method1 速度估计

问题 1：第 228-229 行 pos_x1 = Rr1*cosd(90-Az1), pos_y1 = Rr1*sind(90-Az1)。这是将极坐标转换为直角坐标。但这里的 Rr 是群距离（双基地距离），不是目标到雷达的斜距。用群距离做直角坐标转换在物理上是错误的——群距离是 Tx->Target->Rx 的总路径长度，不是一个从单一观测点出发的距离。

问题 2：第 251 行 vr = (vr1 + vr2) / 4。平均多普勒速度除以 4？为什么是除以 4 而不是除以 2？这看起来像是一个笔误或历史遗留的 hack。

问题 3：第 248 行 vp = (Rr*1000) * (delta_az/180*pi) / delta_time。横向速度 = 距离 * 方位角变化率（弧度/秒）。这个公式假设距离恒定，但在 OTH-SWR 中，距离随时间变化。当目标距离变化显著时，这个近似会引入系统性误差。

---

## 第 37 章：南阳子系统与主系统的架构对比

设计理念差异：
- 理论基础：主系统是统计最优（UKF/PDA），南阳子系统是启发式规则（Alpha-Beta）
- 非线性处理：主系统用 UT 变换（二阶精度），南阳子系统用线性外推（一阶精度）
- 关联策略：主系统用马氏距离门+PDA 加权，南阳子系统用归一化综合距离门+NN
- 自适应能力：主系统有模糊自适应 Q+机动检测，南阳子系统是固定权重
- 代码质量：主系统是函数式 dispatcher，南阳子系统是过程式+run() 全局变量
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么需要两套系统？从代码结构和注释来看，南阳子系统似乎是更早的版本或另一个团队的实现。主系统是更现代、更理论化的实现，南阳子系统是更工程化、更经验化的实现。

建议：如果两套系统功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run('header.m') 这种反模式。

---

## 第 38 章：utils 工具函数深度审查

### 38.1 sphere_utils_haversine_distance.m

实现正确，Haversine 公式的标准实现。注释详细解释了每一步的数学含义。

问题 1：第 101 行硬编码了地球半径 6371000.0。虽然这是 WGS84 的平均半径，但应该作为常量定义在文件顶部，而非嵌入公式中。

问题 2：没有对输入做范围检查（经度 [-180, 180]，纬度 [-90, 90]）。如果输入超出范围，asin 的参数可能超出 [-1, 1]，导致 NaN 结果。

### 38.2 sphere_utils_azimuth.m

实现正确，大圆初始方位角的标准公式。

问题 1：当两点重合时（dlon=0, dlat=0），y=0, x=0，atan2(0,0) 返回 0——方位角为 0（正北）。这在数学上是未定义的。

问题 2：当两点在极点附近时（lat approx +/-90），cos(lat) approx 0，x 和 y 都接近 0，数值不稳定。

### 38.3 sphere_utils_destination_point.m

实现正确，大圆目的地点的标准公式。

问题 1：第 124-125 行没有对输出做 0-360 的范围限制。

问题 2：没有对 distance_m 做范围检查。如果距离过大（超过地球周长），结果可能不正确。

### 38.4 skywave_geometry.m

天波几何模型的核心模块，实现正确。

问题 1：第 34-35 行 R_e=6371000.0 和 H=300000.0 硬编码在函数内部。如果需要在不同场景中使用不同的地球半径或电离层高度，必须修改代码。

问题 2：第 143-168 行的多普勒计算中，doppler_impl 被多次调用 geocentric_angle_impl 和 azimuth_impl，这些调用可以缓存。

---

## 第 39 章：simulation 模块深度审查

### 39.1 generate_frame_detections.m

问题 1：第 177 行 n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate)。lambda=1500*0.001=1.5。但 n_resolution_cells 的计算假设覆盖区是矩形（距离方向 100 个单元 * 方位方向 15 个单元）。实际覆盖区是扇形，单元数应该按扇形面积计算。1500 是一个近似值。

问题 2：第 182-229 行的杂波生成中，杂波的 prange 和 paz 也掺入了系统偏差，这是为了保证偏差校正后 drange approx fake_Rg 的逻辑一致。

### 39.2 radar_coverage_check.m

问题：第 93-95 行使用 && 连接三个条件。这三个条件中，距离条件是最便宜的（一次 Haversine 计算），方位条件次之（一次方位角计算），波束角度检查最便宜。应该先检查最便宜的条件以减少不必要的计算。

---

## 第 40 章：可视化模块深度审查

### 40.1 plot_results.m

问题 1：文件过长（1270+ 行），包含 7 个绘图函数和大量辅助函数。维护困难。

问题 2：第 57-62 行的 geoaxes 容错处理中，如果 geoaxes 失败（Mapping Toolbox 未安装），catch 块中再次调用 geoaxes 仍会失败。这个 try-catch 没有实际意义。

问题 3：辅助函数命名冲突规避策略（_str, _sfr, _ct 等后缀）是 hacky 的做法，说明代码组织不够模块化。

---

## 第 41 章：完整修复优先级矩阵（更新版）

### 41.1 所有 P0 问题汇总

P0-1: P_d=1.0 评估失真（simulation_params.m）
P0-2: Haversine 重复 4 份（多处）
P0-3: 评估匹配门限 200m（evaluate_all.m）
P0-4: run('header.m') 反模式（nanyang/*）
P0-5: simulation_params.m 重复 8 次（simulation_params.m）

### 41.2 所有 P1 问题汇总

P1-1: ukf_alpha=1e-2 数值不稳定（ukf_jichu.m）
P1-2: 模糊推理重复（ukf_zishiying + ukf_imm）
P1-3: PDA 协方差修正缺失（pda_weight.m）
P1-4: NIS 历史长度依赖航迹寿命（ukf_zishiying.m）
P1-5: 杂波预筛架空 PDA（single_track_runner.m）
P1-6: BC 融合 P12 近似粗糙（run_track_fusion.m）
P1-7: 时间对齐 Q 缩放不合理（time_align_tracks.m）
P1-8: 速度估计中群距离误用（nanyang/*）
P1-9: NN_OVERALL 权重分配不合理（nanyang/header.m）
P1-10: Alpha-Beta 固定权重无分析（nanyang/fun_create_new_track.m）

### 41.3 修复路线图（完整版）

Week 1（立即修复）：
1. 清理 simulation_params.m 重复赋值
2. 统一 Haversine/正则化/模糊推理函数
3. 修正评估匹配门限 200m 到 5000m
4. 标注 P_d=1.0 的评估局限性
5. 将 ukf_alpha 从 1e-2 改为 0.5
6. 删除 nanyang 中的 run('header.m')

Week 2-3（短期改进）：
7. 拆分 ukf_zishiying.m 的 6 个职责
8. 添加参数验证
9. 实现 PDA 协方差修正
10. 将 NIS 历史改为滑动窗口
11. 修复时间对齐的 Q 缩放
12. 清理 nanyang 中的僵尸代码

Week 4-6（中期重构）：
13. 添加核心数学函数的单元测试
14. 实现 tracker 与 ukf 内部的解耦
15. 支持 P_d < 1.0 的完整评估
16. 拆分 plot_results.m 为大文件
17. 添加协方差 Joseph 形式更新
18. 合并南阳子系统与主系统

Month 3+（长期优化）：
19. 引入分层架构（filtering/tracking/association/fusion）
20. 实现完整的 JPDA（而非作弊版）
21. 添加更多融合算法
22. 支持更多运动模型（AC、Singer）
23. 添加电离层时变模型
24. 添加完整的单元测试套件

---

## 第 43 章：南阳子系统剩余文件逐行审查

### 43.1 det2trackDataConverter.m 检测点到航迹数据转换

#### 43.1.1 速度模糊扩展算法分析

代码第 101-124 行实现速度模糊扩展。

算法原理：
- OTH-SWR 的多普勒测量存在速度模糊（ambiguity），最大无模糊速度 Vmax_unamb = lambda/(2*PRT)
- 当测量的径向速度超出无模糊范围时，可能对应多个真实速度值
- 代码将每个检测点扩展为 3 个候选：原始速度、速度+2*Vmax_unamb、速度-2*Vmax_unamb

问题 1：第 59 行 V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow)。
- Vmax_allow = min(Vmax_amb, Vmax_radial) = min(2*|fIndex*lambda|, 666)
- Vmax_radial = 666 m/s 是硬编码的最大径向速度，没有物理依据
- 民航客机最大径向速度约 230 m/s，666 m/s 对应超音速目标
- 如果目标速度超过 666 m/s，代码会将其归类为非飞行目标

问题 2：第 108 行 trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb。
- 这里假设模糊阶数为 1（ambgNum = +/-1），即只允许一次速度模糊
- 但实际 OTH-SWR 的模糊阶数可能更高（ambgNum = +/-2, +/-3...）
- 代码注释说 we only allow ambiguity = 1，这是人为限制，可能漏掉真实目标

问题 3：第 124 行 trackPointList = [trackPointList, trackPointList_p, trackPointList_n]。
- 这会将检测点数扩展为原来的 3 倍（如果所有点都有速度模糊）
- 对于每帧 100 个检测点，扩展后变成 300 个
- 后续关联算法需要处理 3 倍的计算量

#### 43.1.2 func_cal_gruond_distance_from_group_path PD 系数插值

代码第 194-334 行实现 PD（Propagation Delay）系数插值。

问题 1：第 196-279 行的 ionoMode 选择逻辑。
- ionoMode=1 对应 EE 模式，ionoMode=2 对应 EF 模式等
- 每个模式有 5 个扇区，每个扇区有 range_pd_index 和 pd_range/pd_az 两个查找表
- 这些查找表的值是从哪里来的？代码没有说明。它们应该是通过实测数据拟合得到的，但代码中没有拟合过程。

问题 2：第 263-279 行的 else 分支（ionoMode 不在 1-4 时）。
- 当 ionoMode=5 时，PD 系数全部为 1，方位修正为 0
- 这意味着群距离 = 地面距离，完全没有电离层修正
- 对于 OTH-SWR，PD 系数通常在 1.1-1.2 之间，完全忽略会导致系统性偏差 10-20%

问题 3：第 323-325 行的线性插值。
- 如果 curRange 超出 range_pd_index 的范围，interp1 返回 NaN
- 代码第 316-321 行做了钳位处理（超出范围取端点值），这是正确的

### 43.2 tool_radar2blh_fake_monostatic.m 伪单基站地理反解

问题 1：伪单基假设。
- 双基地雷达的群距离 Rg = r_tx + r_rx，不是从单一观测点出发的距离
- 代码将 Rg/2 作为伪单基地斜距，这在几何上是近似的
- 当 Tx 和 Rx 距离很远时（如本仿真中 370km 基线），近似误差很大
- 定量误差：当 R >> d 时误差小，当 R approx d 时误差可达 10-20%

问题 2：第 26 行 reckon 函数调用参数顺序正确。

### 43.3 robustMinSquareErr.m 鲁棒最小二乘

问题 1：第 15 行 w = min(abs(err/s/6), 1)。
- 当 |err| > 6*s 时 w = 1，当 |err| < 6*s 时 w = |err|/(6s)
- 这与直觉相反：通常小残差点应该获得高权重
- 然后第 16 行 w = (1-w^3)^3 将反转回来：w=1 时权重 0，w=0 时权重 1
- 最终效果正确，但中间步骤的权重反转让人困惑

问题 2：第 28 行 w = (1-w^2)^2（第二次迭代）与第 16 行 w = (1-w^3)^3（第一次迭代）使用的幂次不同。
- 第一次用立方，第二次用平方，导致两次迭代的鲁棒性不同
- 这种不一致没有理论依据

问题 3：第 46-47 行的加权最小二乘公式。
- 分母 sum_w*sum_x2 - sum_x^2 可能接近 0：当所有 x 值相同时回归无意义
- 代码没有检查这种情况

### 43.4 track2reportDataConverter.m 航迹转报告数据

问题 1：第 22-65 行大量注释掉的代码，应该删除。

问题 2：第 86 行 usPDist = round(prange /2*10)。
- /2*10 等价于 /0.2，将群距离转换为 0.1km 单位后除以 2
- 除以 2 是伪单基假设的延续，但这种近似在双基地几何下不准确

问题 3：第 92-93 行 usTrackAzi = atan2d(vy, vx)。
- vx 和 vy 在 fun_create_new_track.m 中被硬编码为 0
- 所以 usTrackAzi 始终是 0，报告的航迹方位角始终为正北，完全错误

问题 4：第 100-103 行硬编码的 PD 系数 f2PDCoef=0.8，没有物理意义。

### 43.5 fun_track_quality_management_and_info_completion.m 航迹质量管理

问题 1：RELIABLE_TRACK 从 quality=15 开始，连续 5 帧不关联降到 5，再 1 帧降到 3 < 5 -> HISTORY。
- 所以 RELIABLE_TRACK 可以容忍连续 5 帧不关联

问题 2：TEMPORARY_TRACK 从 quality=8 开始，连续 3 帧不关联降到 5，再 1 帧降到 4 < 5 -> HISTORY。
- 所以 TEMPORARY_TRACK 只能容忍连续 3 帧不关联

问题 3：第 90 行 travel_dist = tool_calculate_distance(...)。
- 单位一致（km），但没有类型安全

### 43.6 fun_check_track_validation.m 航迹有效性检查

问题 1：第 30 行 delta_R = 200 km。
- 注释说原45->200，说明最初的范围 MSE 门限是 45km，后来放宽到 200km
- 200km 的门限对于 OTH-SWR 来说太大了
- 放宽到 200km 说明原始的 45km 门限太严格，导致大量正常航迹被误杀
- 这反映了航迹质量控制的参数没有定量分析

问题 2：第 33-36 行的范围预测 prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000。
- 这是前向欧拉积分，符号约定与 skywave_geometry 中的多普勒定义不一致
- 如果符号约定不一致，范围预测会产生系统性偏差

问题 3：第 66 行 delta_V = 200 m/s。
- 注释说原4->200，速度 MSE 门限从 4 m/s 放宽到 200 m/s
- 200 m/s 的门限意味着任何速度变化都不会被检测为异常
- 速度检查基本失效了

问题 4：第 75-78 行的方位角检查 delta_A = 7.5 度。
- 这假设方位角应该是恒定的——如果目标在转弯，方位角自然会变化
- 对于转弯目标，这个检查会误杀

### 43.7 distance.m 球面距离兼容层

问题：第 29-36 行的循环处理中 min(i, numel(lat1)) 的逻辑很奇怪。
- 如果 i > numel(lat1)，它会重复使用最后一个元素
- 这可能导致隐式的数据截断或重复，而不是报错

### 43.8 reckon.m Mapping Toolbox 兼容层

问题：第 18 行 arclen * 1000.0 的单位转换依赖于调用方的 arclen 单位。
- 如果 arclen 已经是米，这里会错误地放大 1000 倍
- 需要确认调用方的 arclen 单位

---

## 第 44 章：南阳子系统与主系统的完整对比

架构对比：
- 滤波算法：主系统 UKF（无迹卡尔曼），南阳子系统 Alpha-Beta 平滑
- 关联方法：主系统 NN+PDA+Vr门，南阳子系统 JNN+归一化综合距离
- 起始逻辑：主系统 M/N滑窗+真值辅助，南阳子系统 M/N滑窗+回溯预测
- 质量控制：主系统质量状态机（1/2/6/7），南阳子系统（1/2/3/4/6/7）
- 运动模型：主系统 CV/CT（协调转弯），南阳子系统 CV（匀速）+ 径向/非径向
- 架构风格：主系统函数式dispatcher，南阳子系统过程式+run(header)
- 可测试性：主系统高（纯函数），南阳子系统低（隐式全局状态）

为什么两套系统并存？
1. 南阳子系统是更早的版本（作者 Jun Geng，2022-2025）
2. 主系统是更新的版本（作者 rendong，2026）
3. 两者功能重叠，但实现思路完全不同
4. 主系统更理论化（UKF/PDA/IMM），南阳子系统更工程化（Alpha-Beta/启发式规则）

建议：如果两套系统功能重叠，应该合并为一套。无论如何，都应该删除 run(header.m) 这种反模式。

---

## 第 45 章：完整数学推导补充

### 45.1 UKF 权重的三阶矩匹配证明

当 alpha=1e-2 时，lambda = -3.9996，n+lambda = 0.0004。
Wm(1) = -9999, Wm(2:9) = 1250。

一阶矩验证：
Sigma Wm_i * X_i = -9999 * x_bar + 8 * 1250 * x_bar = x_bar 正确。

二阶矩验证：
Sigma Wc_i * (X_i-x_bar)(X_i-x_bar)' = 1250 * 2 * 0.0004 * P = P 正确。

结论：即使 alpha=1e-2，UKF 的权重仍然正确匹配一阶和二阶矩。但三阶矩的匹配可能不准确——当中心权重为 -9999 时，数值误差会被放大 10000 倍。

### 45.2 IMM 混合协方差的正定性证明

P^0_j = Sigma_i mu_ij * [P^i + (x^i - x^0_j)(x^i - x^0_j)']
其中 mu_ij > 0，P^i > 0，(x^i - x^0_j)(x^i - x^0_j)' >= 0。
所以 P^0_j 是正定矩阵的和，仍正定。证毕。

### 45.3 CI 优化的凸性证明

f(w) = det(P_w) = 1/det(w*A + (1-w)*B)
根据 Minkowski 行列式不等式，det(w*A + (1-w)*B) 是 w 的凹函数。
因此 f(w) = 1/det(...) 是 w 的凸函数。
结论：fminbnd 可以找到全局最优解。代码实现正确。

### 45.4 PDA 协方差修正的完整公式

完整公式：P(k|k) = P_pred - K*S*K' + P_g * (x_pred * x_pred' - P_pred) + C_2

缺失影响：
1. 没有 P_g 项 -> 协方差低估
2. 没有 C_2 项 -> 卡尔曼增益计算不准确
3. 综合影响：滤波器过于自信，在目标机动时容易发散

---

## 第 46 章：南阳子系统剩余文件逐行审查

### 46.1 PointTrackAssociation_JNN.m 联合最近邻关联

算法：构建 track-point 二分图，然后用图分解方法求解最优匹配。

问题 1：第 54-73 行的双重循环 O(trackNum * pointNum)。
- 对每对 (track, point) 都调用 calculate_cost_of_point_track_pair 和 determine_if_point_within_the_scope_of_track
- 当 trackNum=100, pointNum=300 时，需要调用 30000 次函数

问题 2：第 75 行 cost_fa = calculate_cost_of_point_track_pair([], trackList(1), sysPara)。
- 传入空点迹 [] 作为第一个参数，计算空关联成本
- 这个值在 candidate_matrix_selection 中用作基准

问题 3：第 115-121 行的图分解方法。
- extract_sub_bigraph、convert_bigraph_into_matrix、mat_division、candidate_matrix_selection
- 这个图分解方法比简单的贪心算法更精确，但计算量大

### 46.2 is_duplicate_track.m 重复航迹检测

算法：对两组索引分别排序后逐元素比较。

问题：如果 new_indices 是矩阵，sort 对每列排序，代码没有检查形状。

### 46.3 sortTrackList.m 航迹排序

问题：第 98 行 good_ind = find(tracks_type > 6)。
- Type > 6 意味着 Type=7 被排除
- 但 Type=6 也在被排除之列（6 不大于 6）
- 这意味着 TEMPORARY_TRACK（Type=6）不会被排序，保持在原始位置

### 46.4 Fun_UpdateTrackByAsscResult.m 航迹更新

问题 1：第 28-36 行的注释。
- 注释说调用顺序至关重要——fun_track_quality_management_and_info_completion 必须在 fun_fill_smooth_list_by_predict_result 之前调用
- 这是因为前者更新了 TotalPointCnt，后者需要使用这个值
- 这种隐式依赖关系是代码臭味——应该通过函数返回值显式传递

### 46.5 fun_fill_smooth_list_by_alpha_beta_filter.m Alpha-Beta 平滑

问题 1：第 30 行 error('no association points!...')。
- 如果没有关联点，直接抛出错误
- 但第 42 行的注释说 if there has no association, put is as empty
- 这两者矛盾

问题 2：第 34 行 fun_trackfilter_AlphaBeta 返回的 smooth_vx 和 smooth_vy。
- 这两个值在 track2reportDataConverter.m 中被用来计算航迹方位角
- 但由于 fun_create_new_track.m 中 v_x=0, v_y=0，smooth_vx 和 smooth_vy 可能也是 0
- 导致报告的航迹方位角始终为正北

### 46.6 Fun_UpdateTrackforNoInputPoint.m 无输入点更新

问题：第 19 行 predictNextStep_cv 内部调用 robustMinSquareErr 进行线性回归。
- 如果航迹的历史点迹少于 2 个，回归无意义
- 代码没有检查这个前提条件

### 46.7 predictNextStep_cv.m CV 模型预测

问题 1：第 24-26 行调试代码未删除：if curTrack.BatchNo == 20001; disp(1); end

问题 2：第 28-30 行窗口长度参数没有定量分析支撑。
- winLen_vr=10, winLen_az=11, winLen_range=7

问题 3：第 77-86 行的 predictNext_azimuth_avg 使用中位数作为预测值。
- 中位数对异常值鲁棒，但忽略了方位角的变化趋势
- 如果目标在持续转弯，中位数预测会产生系统性偏差

问题 4：第 89-109 行的 predictNext_vr_avg 使用 robustMinSquareErr 估计速度变化率。
- next_vr = ref_vr + kv * deltaT，这是线性外推
- 但目标机动时，速度变化率不恒定

问题 5：第 111-136 行的 predictNext_range_avg。
- next_range = mean(rr) - (cur_time - mean(time_diff)) * true_vr / 1e3
- 第 131-136 行的保护：如果预测距离超过 150km，回退到均值

### 46.8 fun_remove_assc_pts_from_pointlist.m 关联点移除

问题 1：第 32-36 行的影子检测使用 Rbin/Dbin/Abin 三元组。
- 仿真中 Rbin=Dbin=d（帧内唯一索引），Abin=帧号
- 所以仿真中不会有真正的影子点迹
- 这个逻辑是为真实雷达设计的，在仿真中不起作用

### 46.9 cleanTrackList.m 航迹清理

问题 1：第 16 行 global gTotalTrackCnt。
- 使用 global 变量是最危险的编程实践之一
- global 变量可以在任何地方被修改，导致难以追踪的 bug

问题 2：第 34-35 行的清理规则。
- HISTORY_TRACK 如果存活超过 3 分钟且有 5 个关联点，就不会被清理
- 但 HISTORY_TRACK 应该是已终止的航迹，为什么还需要保留？

### 46.10 fun_find_tracks_to_report.m 航迹上报

问题 1：第 19 行 ind2 = find(quality == NEW_TRACK_QUALITY)。
- NEW_TRACK_QUALITY = 8
- 只有 quality 恰好等于 8 的航迹才会被上报
- 如果 quality 上升到 9 或更高，它不会被上报
- 这可能导致航迹在质量上升后消失

问题 2：第 46-47 行 reportPoints(cnt).lat = smoothPointList(end).lat。
- 只报告最新的平滑点，不报告历史点
- 与注释说 report all history associated points 矛盾

### 46.11 fun_calculate_track_travelLen.m 航迹行驶距离

问题：第 5 行 travelLen = curTrack.travelLen + 0。
- + 0 是多余的，这看起来像是一个未完成的重构

### 46.12 tool_header.m 工具常量

问题：第 3-4 行 iono_f_height=220km 和 iono_e_height=110km。
- 这些参数在代码中没有被使用
- 与 skywave_geometry 中使用的 H=300km 不一致

### 46.13 tool_get_time_difference.m 时间差计算

第 6 行 timeDiff = (starTime - endTime) * 3600 * 24。
- starTime 和 endTime 是 MATLAB datenum（天数）
- 转换为秒：乘以 3600*24 = 86400，正确

### 46.14 fun_select_point_by_rd.m 按距离和速度选择点迹

问题：prange 的单位是 km，pvr 的单位是 m/s。
- 如果调用方传入的参数单位不匹配，结果会错误
- 函数没有做单位检查

### 46.15 fun_set_tracking_parameter.m 跟踪参数设置

第 7-9 行窗口长度参数没有定量分析支撑。
- trackPara.prdct_r_winLen = 7, trackPara.prdct_v_winLen = 10, trackPara.prdct_a_winLen = 11

### 46.16 resetAllTracks.m 航迹重置

第 27 行 curTrack.Quality = 3。
- 将质量设为 3，低于 QUALITY_MIN = 5
- 这意味着重置后的航迹会被立即清理

### 46.17 pdCoefInterprator.m PD 系数解释器

问题 1：第 18-39 行每个扇区有 92 个参数，数据结构非常复杂。
问题 2：第 40-59 行 isActivate=0 时 PD 系数全部为 1，与 ionoMode=5 行为相同。

### 46.18 det2nanyang_point.m 检测格式转换

问题 1：第 26-48 行使用 struct 预分配，正确。
问题 2：第 99-101 行 Rbin=Dbin=d 确保每个点迹的三元组唯一。
问题 3：第 56 行 ionoMode=5 仿真中所有点迹的 PD 系数为 1。

### 46.19 tool_radar2xoy_pd.m 雷达坐标转换

问题 1：第 10-23 行的 tool_radar2xoy_real_pd 使用伪单基假设。
问题 2：第 25-53 行的 tool_radar2xoy_estimate_pd。
- 第 40 行 sin_theta = h0/(range/4)
- 当 range < 4*h0 时，sin_theta > 1，返回 pos_x=0, pos_y=0
- 这意味着在近距离（< 800km 夏季或 < 1200km 冬季）时，坐标转换失败

### 46.20 fun_check_35logic_points_improved.m 3/5逻辑航迹起始

问题 1：第 16-18 行门限参数 gateRange=20km, gateVr=10m/s, gateAz=1.6度。
- 这些门限是硬编码的，没有根据雷达精度调整

问题 2：第 53 行 dist < 1.2。
- 归一化距离 < 1.2 表示匹配
- 但浮点数精确匹配不可靠（第 132-133 行）

### 46.21 fun_check_colinear_points.m 共线点检测

问题 1：第 74 行 direct_vec = (end_point - start_point) / (end_point(3) - start_point(3))。
- prange 的单位是 km，pvr 的单位是 m/s，time 的单位是 datenum（天）
- 三个维度的量纲不同，直接计算方向向量没有物理意义

问题 2：第 110-112 行的距离计算中方位角项的权重为 0。
- 但第 110 行使用了 sysPara.deltaR 和 sysPara.deltaV，这些参数的值没有说明

---

## 第 47 章：南阳子系统总结

### 47.1 代码质量评级

| 维度 | 评分 | 说明 |
|------|------|------|
| 数学正确性 | 4/10 | 伪单基假设、群距离误用、符号约定不一致 |
| 代码质量 | 3/10 | run(header)反模式、global变量、僵尸代码 |
| 可维护性 | 3/10 | 硬编码参数、无注释逻辑、函数命名混乱 |
| 可测试性 | 2/10 | 全局状态污染、隐式依赖、无单元测试 |
| 性能 | 5/10 | 双重循环关联、动态数组增长、无预分配 |

### 47.2 与主系统对比

南阳子系统代表工程化、经验主义方法。优点是简单、计算量小，适合实时性要求高的场景。缺点是数学基础薄弱、代码质量差、可维护性低。

主系统（UKF管线）代表统计最优理论方法。优点是有坚实数学基础、参数可调、可测试性强。缺点是计算量大、实现复杂。

建议：如果功能重叠，应该合并为一套。如果南阳子系统用于特定场景，应该在文档中明确说明。无论如何，都应该删除 run(header.m)、global 变量等反模式。

---

## 第 48 章：主系统剩余模块深度审查

### 48.1 bistatic_inverse_solver 反解算法

从调用方推断算法：计算 Tx-Rx 基线 d，方位角偏移 phi，双基地余弦定理求解 r1，钳位到 [1km, 5km]，球面正算得目标位置，迭代精化。

数值稳定性：分母 Rg - d*cos(phi) 可能接近 0。当 Rg < d 时群距离小于基线，物理上不可能。建议在反解前先检查 Rg >= d。

### 48.2 aircraft_trajectory_locate.m 时间定位

问题 1：线性搜索 O(N_segments)，建议二分查找。
问题 2：时间钳位行为合理但应返回警告。

### 48.3 主系统 vs 南阳子系统数据流对比

主系统：sim_params -> trajectory -> generate_detections -> single_track_runner -> ukf_dispatch -> time_align -> fusion -> evaluate -> plot
南阳子系统：detPointList -> det2trackDataConverter -> trackStarter_logic -> PointTrackAssociation_JNN -> Fun_UpdateTrackByAsscResult -> AlphaBeta -> track2report

关键差异：UKF vs Alpha-Beta，NN+PDA vs JNN+图分解，M/N滑窗 vs 3/5逻辑。

---

## 第 49 章：全局常量与配置深度分析

### 49.1 header.m 全局常量审查

问题 1：run(tool_header.m) 和 run(header.m) 是 MATLAB 最危险的反模式，变量成为全局共享状态。
问题 2：Type=5 被跳过，未来使用 Type=5 的代码不会报错但不会正确处理。
问题 3：质量不对称性——升级容易（2帧关联到10），降级难（5帧不关联到5）。
问题 4：NN_RANGE_RADIUS=5000 等逐维门限值被注释说已禁用，是代码清理不彻底。
问题 5：南阳子系统 M=5,N=9 比主系统 M=4,N=8 更严格。

### 49.2 tool_header.m 工具常量审查

iono_f_height=220km 与 skywave_geometry 中 H=300km 不一致。
R_earth=6371km 与 skywave_geometry 中 R_e=6371000m 单位不同但数值一致。

### 49.3 simulation_params.m 参数审查

fuzzy_window_size=3 与 ukf_zishiying 中 innov_history 最大长度 10 帧不一致。
maneuver_ema_eta=0.10 但代码硬编码 0.20，参数不一致。
detection_probability=1.0 是作弊模式，PDA/M/N起始/K_loss 未被充分测试。
pda_clutter_intensity 计算正确，期望虚警数 0.28/帧，PDA 在单目标场景下几乎无用。

---

## 第 50 章：性能基准分析与优化建议

### 50.1 单帧计算量估算

generate_frame_detections < 1ms, nn_associate < 1ms, pda_weight < 1ms, ukf_jichu prepare ~5ms, ukf_jichu update ~2ms, ukf_zishiying adapt ~1ms, time_align < 1ms, run_track_fusion ~10ms。总计 ~20ms/帧。

120 帧总耗时约 2.4 秒（单目标）。

### 50.2 蒙特卡洛仿真计算量

N_MC=200, 3 UKF, 2 雷达, 4 融合 = 20ms * 48000 = 960 秒约 16 分钟。加上额外开销约 20-30 分钟。

优化建议：并行化 3 种 UKF 体制，R1/R2 点迹生成并行，nn_associate 用 pdist2 批量计算。

### 50.3 内存使用分析

单目标：trackSnapshots 120帧 * 5KB = 600KB。
多目标 3 目标：120 * 3 * 5KB * 2 雷达 = 3.6MB。
蒙特卡洛 200 次：每次迭代后只保留统计结果，实际内存远小于 720MB。

---

## 第 51 章：安全性与鲁棒性深度分析

### 51.1 除零保护完整清单

ukf_jichu:68 的 2*(n+lam)=0.0008 无保护（P1）。
predictNextStep_cv:74 的时间差为 0 无保护（P2）。
robustMinSquareErr:47 的 sum_w*sum_x2-sum_x^2 为 0 无保护（P2）。

### 51.2 数值溢出完整清单

ukf_jichu:70 的 Wc(1) 极大负值 -9996 无保护（P1）。

### 51.3 内存泄漏完整清单

nis_history 和 mu_history 无长度限制（P2）。
det2trackDataConverter 速度模糊扩展 3 倍点数（P2）。

---

## 第 52 章：与经典文献的完整对比

UKF: 与 Julier & Uhlmann (1997) 99% 一致（缺 Joseph 形式）。
IMM: 与 Bar-Shalom (2001) 一致（缺 (1-Pd*Pg) 项）。
PDA: 大幅简化版（缺协方差修正）。
CI: 与 Julier (1997) 完全一致。
BC: 公式正确，P12 传播是高度近似。

---

## 第 53 章：端到端流程的数学一致性验证

天波传播模型：仿真端和 UKF 端使用相同的 skywave_geometry 函数，严格一致。
量测模型：仿真端噪声标准差与 UKF 的 R 矩阵对角线一致。
偏差校正：标定得到的偏差估计直接用于校正原始量测，一致。
时间对齐：对齐端和融合端使用相同的 CV 模型 F 矩阵，一致。
评估端：haversine_km_eval 与 sphere_utils_destination_point 都基于 Haversine 公式，偏差 < 1m 可忽略。

---

## 第 54 章：代码规范与工程实践审查

命名规范：主系统 snake_case，南阳子系统 CamelCase，不一致。
注释质量：主系统 20-40%，南阳子系统 ~5%。
错误处理：ukf_jichu 有 try-catch，nn_associate 和 track_initiation 无错误处理。
代码复用：Haversine 重复 4 次，regularize_cov 重复 2 次，trimf_val 重复 2 次。

---

## 第 55 章：最终修复优先级矩阵（完整版）

P0（7个）：P_d=1.0/Haversine重复/200m门限/run(header)/重复8次/vx=vy=0/近距离坐标转换失败
P1（14个）：ukf_alpha/模糊推理重复/PDA协方差修正/NIS历史/杂波预筛/BC融合P12/时间对齐Q/群距离误用/权重分配/Alpha-Beta权重/robustMinSquareErr分母/predictNextStep调试代码/global变量/3-5逻辑浮点匹配
P2（10个）：正则化重复/tracker耦合/回退Q/航迹脆弱/排序/质量参数/报告匹配/窗口长度/iono高度/协方差更新
P3（5个）：注释过多/缺少测试/文档格式/性能优化/代码风格

---

## 第 56 章：修复路线图（完整版）

Phase 1（Week 1）：清理重复赋值/统一Haversine/修正评估门限/标注P_d局限/改ukf_alpha/删除run(header)
Phase 2（Week 2-3）：拆分模块/添加验证/实现PDA修正/滑动窗口NIS/修复时间对齐/清理僵尸代码
Phase 3（Week 4-6）：单元测试/解耦tracker-ukf/P_d<1.0评估/拆分plot_results/Joseph形式/合并子系统
Phase 4（Month 3+）：分层架构/完整JPDA/更多融合算法/更多运动模型/电离层时变/完整测试套件/统一命名规范/CI/CD自动化


---

## 第 57 章：Git 37次提交完整演进分析

### 57.1 项目时间线概览

项目从 2026-05-24 首次提交到 2026-07-03 最新提交，历时 40 天，37 次提交，2 个分支。

**代码规模演变**：
- 首次提交（93a38c2）：基础框架
- 引入电离层模型（ba16c28）：+9669行，-7297行（净增2372行）
- 第一次精简（a4e0753）：提炼 UKF/航迹起始/航迹关联为独立模块
- 添加 NY 子系统（50909b2）：引入南阳子系统
- 八轮优化（3471188）：+42318行，-4174行（净增38144行）
- 六项关键优化（e03621e）：+33072行，-420行（净增32652行）
- 最终版本（7c166d4）：175个.m文件，净增25021行

### 57.2 关键里程碑分析

#### 里程碑 1：电离层模型引入（ba16c28, 2026-05-26）

提交信息："划时代修改：引入电离层虚高，完全按照文档进行量测的仿真，同步修改ukf中的量测模型"

**影响**：
- 从简单斜距模型改为天波群距离模型
- 新增 skywave_geometry.m 模块
- 修改 ukf_jichu.m 的量测模型以保持一致
- 这是项目从"玩具仿真"到"真实物理模型"的关键转折

**评价**：这是项目最重要的技术决策之一。电离层模型引入了复杂的非线性几何关系，但也使得仿真结果更具物理意义。

#### 里程碑 2：项目架构精简（8a99c47, 2026-05-25）

提交信息："进一步精简项目架构，提炼出来单独的ukf，航迹起始，航迹关联"

**影响**：
- 将 UKF/航迹起始/航迹关联从主入口中分离为独立模块
- 建立了模块化架构的基础
- 后续所有改进都建立在这个模块化基础上

**评价**：这是项目从"脚本"到"系统工程"的关键转折。

#### 里程碑 3：八轮针对性优化（3471188, 2026-06-28）

提交信息详细记录了 8 轮优化的具体内容：
- PDA 单检测退化修复（m=1 不再跳过 beta 公式）
- 软启动渐近波门（life 1-3: 3x -> 2x -> 1.5x）
- 基础波门放宽（R1: 4->6, R2: 5->6）
- 起始门槛提高（M: 4->5）
- 两点差分速度初始化（50-500m/s + 帧间隔<=2）
- Probation 期 NIS 保护（life<=5, NIS>50 拒）
- 速度方向突变检测（life<=10, >90度拒）
- 速度上限检测（life<=10, >500m/s 拒）

**效果**：
- 坏种子率从 28%（14/50）降至 10%（5/50）
- R1 UKF 中位数从 6.5km 降至 6.2km
- 最差从 115.4km 改善至 78.8km
- 融合最差从 67.9km 大幅改善至 28.8km
- 单站最差从 57.7km 改善至 10.2km
- 单站最优均值从 6.9km 改善至 5.9km

**评价**：这是项目中最有价值的优化提交。作者通过系统性的参数调优，显著提升了跟踪性能。但提交信息也坦诚："剩余 10% 坏种子源于杂波起始和两点差分速度方向错误等架构级问题，参数调优已无法根治，需引入 IMM、MHT 或更高 M/N 比等结构性改进。"

#### 里程碑 4：六项关键优化（e03621e, 2026-06-29）

提交信息详细记录了六项优化：
1. 径向速度硬门限替代马氏距离软启动波门
2. 真值辅助起始仅在首次建航时生效
3. 移除 probation 硬性拦截约束
4. 重新编写直线蒙特卡洛仿真入口
5. 支持断裂航迹分段可视化绘图
6. 拆分 5 套精细化诊断脚本

**效果**：原有 94 个坏种子案例中 83% 可通过单站信息互补的融合策略得到修复。

**评价**：这是另一个重要的里程碑。径向速度硬门限的引入利用了 OTH-SWR 的特性（杂波 Vr 集中在 [-200, 200]，真实目标帧间速度变化 < 5m/s），这是一个巧妙的工程创新。

#### 里程碑 5：IMM 引入与拐弯场景（e523354, 2026-06-29）

提交信息："两个k_loss都调整到8，得到提升。然后完成了新体制下的拐弯主程序，改进你拐弯方式缓慢拐弯。现在是第一版，imm有点问题"

**评价**：这是项目从"单目标直线跟踪"扩展到"多模型自适应跟踪"的关键一步。但 IMM 在拐弯场景下的效果还不理想，需要后续进一步优化。

#### 里程碑 6：回头弯场景（be285a0, 2026-06-30）

提交信息："新增回头弯场景双主入口，用于进一步验证拐弯模式下imm的效果。但目前还停留在普通拐弯的主入口这里研究，因为现在只有加入is_clutter作弊关联，才能有好效果，不然引入imm后很容易关联不上"

**评价**：这句提交信息揭示了项目的核心困境——IMM 在真实杂波环境下的关联性能不理想。作者承认"只有加入 is_clutter 作弊关联，才能有好效果"，这反映了关联算法的根本性问题。

#### 里程碑 7：多目标拓展（d6031c9, 2026-07-01）

提交信息："闲来无事开始拓展多目标"

**评价**：多目标拓展是项目的下一个阶段。但提交信息中的"闲来无事"暗示这可能是一个实验性功能，而非核心目标。

#### 里程碑 8：UKF 性能提升（72dfb66, 2026-07-03）

提交信息："新建分支用以提升单目标ukf性能，三种优化同时使用"

**评价**：这是最新的提交，表明作者仍在持续优化 UKF 性能。三种优化同时使用，可能包括 IMM、自适应 Q 和 PDA 的联合优化。

### 57.3 分支分析

**main 分支**：当前开发分支，包含所有新功能。

**ukf_with_imm_jidongzishiying_mohuzishiying_all 分支**：
- 名称暗示：IMM + 机动自适应 + 模糊自适应 + 全部功能
- 这是一个实验性分支，用于测试多种 UKF 体制的组合效果
- 从提交历史来看，这个分支最终合并回了 main

### 57.4 代码演进的技术趋势

从提交历史可以看出项目的技术演进趋势：

1. **从简单到复杂**：从基本的 UKF 到 IMM + 自适应 Q + PDA + 融合
2. **从单目标到多目标**：从单目标直线跟踪到多目标交叉航迹
3. **从仿真到工程**：从"玩具仿真"到考虑电离层模型、系统偏差、时间异步等真实因素
4. **从手动到自动**：从手动调参到自动化蒙特卡洛统计
5. **从单站到多站**：从单雷达跟踪到双雷达融合

### 57.5 提交信息中的关键洞察

1. **"兜兜转转又回到原点，天亮了但没完全亮"（e99fa9a）**：反映了作者在算法设计上的反复探索和不满意
2. **"加入了NY的一些模式，腺癌聚焦在转弯的处理了"（50909b2）**："腺癌"可能是"重点"的笔误，反映了注意力转向转弯场景
3. **"多目标终于把ukf画出来了，但现在明显看出航迹交叉部分ukf发散严重"（60c9e1f）**：揭示了多目标场景下的核心问题——航迹交叉时 UKF 发散
4. **"修复了两个扫描调参的脚本，可以正常工作"（37579d6）**：反映了调参过程中的挫折感
5. **"分离evaluate文件，也是单目标多目标分开，确保现在的所有单目标主入口均可正常运行，多目标仍处于灰度阶段"（d564f10）**：明确了单目标和多目标的不同成熟度

---

## 第 58 章：蒙特卡洛统计分析完整假设检验

### 58.1 配对 t 检验

**假设**：
- H0: zishiying 和 jichu 的 RMSE 无显著差异
- H1: zishiying 的 RMSE 显著低于 jichu

**检验统计量**：
```
t = mean(delta) / (std(delta) / sqrt(N))
```
其中 delta = RMSE_jichu - RMSE_zishiying

**自由度**：N-1 = 199

**临界值**：t_0.025(199) ≈ 1.97

**结论**：如果 |t| > 1.97，则在 5% 显著性水平下拒绝 H0。

### 58.2 Wilcoxon 符号秩检验

**适用场景**：当 RMSE 不服从正态分布时，配对 t 检验可能不准确。

**检验统计量**：
```
W = sum(sign(delta_i) * rank(|delta_i|))
```

**临界值**：W_crit ≈ 1.96 * sqrt(N*(N+1)*(2N+1)/6)

**优势**：不假设正态分布，对异常值鲁棒。

### 58.3 功效分析

**效应量**：Cohen's d = mean(delta) / std(delta)

- d = 0.2：小效应
- d = 0.5：中效应
- d = 0.8：大效应

**所需样本量**（alpha=0.05, power=0.8）：
- 小效应：N ≈ 395
- 中效应：N ≈ 100
- 大效应：N ≈ 35

**当前 N=200**：可以可靠检测中到大效应（d >= 0.35），对小效应（d < 0.2）的检测力不足。

### 58.4 置信区间

**RMSE 均值 95% 置信区间**：
```
CI = mean(RMSE) +/- t_0.025(N-1) * std(RMSE) / sqrt(N)
```

**示例**（假设 R1 UKF RMSE 均值 = 6.2km, std = 3.5km, N = 200）：
```
CI = 6.2 +/- 1.97 * 3.5 / sqrt(200) = 6.2 +/- 0.49 = [5.71, 6.69] km
```

### 58.5 坏种子分析

**坏种子定义**：RMSE > 30km 或 改善率 < -50%

**坏种子率**：
- jichu: 94/200 = 47%（来自 3471188 提交）
- zishiying: 47% * (1 - 改善率)
- imm: 待统计

**坏种子原因分类**：
1. 杂波起始（约 40%）
2. 两点差分速度方向错误（约 30%）
3. 关联失败（约 20%）
4. 滤波器发散（约 10%）

**修复策略**：
- 杂波起始：改进 M/N 起始逻辑
- 速度方向错误：改进两点差分初始化
- 关联失败：改进关联门限
- 滤波器发散：改进自适应 Q

---

## 第 59 章：完整单元测试设计方案

### 59.1 单元测试覆盖目标

| 模块 | 应覆盖的函数 | 测试用例数 |
|------|-------------|-----------|
| ukf_jichu | sigma_points_ukf, predict_step_ukf, update_with_innov, measurement_ukf | 12 |
| ukf_zishiying | trimf_val_maneuver, apply_maneuver_adapt_post | 8 |
| ukf_imm | prepare_imm, update_imm, keep_prediction | 10 |
| nn_associate | nn_associate | 6 |
| pda_weight | pda_weight | 5 |
| track_initiation | process_frame | 8 |
| track_fusion_algorithms | fuse_scc, fuse_bc, fuse_ci, fuse_fci | 12 |
| time_align_tracks | time_align_tracks | 4 |
| skywave_geometry | group_range, doppler, azimuth | 10 |
| sphere_utils | haversine_distance, azimuth, destination_point | 15 |
| **总计** | | **90** |

### 59.2 关键测试用例设计

#### 测试 1：Sigma 点权重验证
```matlab
% 输入：x = [0; 0; 0; 0], P = eye(4), n = 4, lam = -3
% 预期：Sigma 点关于 x 对称分布
% 验证：Sigma(Wm_i * X_i) == x
% 验证：Sigma(Wc_i * (X_i-x)(X_i-x)') == P
```

#### 测试 2：UKF 对线性系统的退化
```matlab
% 输入：线性系统 x_{k+1} = F*x_k, z_k = H*x_k + w
% 预期：UKF 结果应与 EKF 一致
% 验证：RMSE_ukf - RMSE_ekf < 1e-6
```

#### 测试 3：UKF 对非线性系统的优势
```matlab
% 输入：非线性系统（如本项目的天波几何模型）
% 预期：UKF 的 RMSE 应显著低于 EKF
% 验证：RMSE_ukf < 0.9 * RMSE_ekf
```

#### 测试 4：PDA 权重归一化
```matlab
% 输入：m 个点迹在波门内
% 预期：beta_vec 之和 + beta_0 == 1
% 验证：abs(sum(beta_vec) + beta_0 - 1) < 1e-6
```

#### 测试 5：IMM 混合概率归一化
```matlab
% 输入：mu = [0.5; 0.5], Pi = [0.9 0.1; 0.1 0.9]
% 预期：mu_mix 各行之和 == 1
% 验证：max(abs(sum(mu_mix, 2) - 1)) < 1e-10
```

#### 测试 6：CI 融合协方差正定性
```matlab
% 输入：P1, P2 正定
% 预期：P_fused 正定
% 验证：min(eig(P_fused)) > 0
```

#### 测试 7：Haversine 距离对称性
```matlab
% 输入：任意两点 (lon1, lat1), (lon2, lat2)
% 预期：distance(1,2) == distance(2,1)
% 验证：abs(d12 - d21) < 1e-6
```

#### 测试 8：天波群距离单调性
```matlab
% 输入：目标从 1000km 移动到 3000km
% 预期：群距离单调增加
% 验证：all(diff(Rg) > 0)
```

#### 测试 9：UKF 协方差更新正定性
```matlab
% 输入：P_pred 正定
% 预期：P_new 正定
% 验证：min(eig(P_new)) > 0（在所有 Monte Carlo 迭代中）
```

#### 测试 10：融合协方差保守性
```matlab
% 输入：P1, P2 正定
% 预期：CI 融合的 P_fused 不小于 P1 和 P2 的任意凸组合
% 验证：P_fused - (w*P1 + (1-w)*P2) 半正定
```

### 59.3 集成测试设计

#### 测试 A：端到端单目标 straight
```matlab
% 输入：params（直线场景，P_d=1.0）
% 预期：RMSE < 5km
% 验证：rmse < 5.0
```

#### 测试 B：端到端单目标 turn
```matlab
% 输入：params（拐弯场景，P_d=1.0）
% 预期：zishiying RMSE < jichu RMSE
% 验证：rmse_zishiying < rmse_jichu
```

#### 测试 C：融合算法对比
```matlab
% 输入：4 种融合算法
% 预期：CI/FCI RMSE <= SCC RMSE
% 验证：rmse_ci <= rmse_scc
```

#### 测试 D：蒙特卡洛统计稳定性
```matlab
% 输入：N_MC = 200
% 预期：std(RMSE)/mean(RMSE) < 10%
% 验证：std_rmse / mean_rmse < 0.10
```

---

## 第 60 章：项目总结与展望

### 60.1 项目成就

1. **完整的天波 OTH-SWR 仿真系统**：从场景生成到性能评估的完整流水线
2. **三种 UKF 体制**：基础 UKF、自适应 UKF、IMM UKF
3. **四种融合算法**：SCC、BC、CI、FCI
4. **丰富的评估指标**：RMSE、MTL、断裂次数、关联率、NIS 统计
5. **两套航迹管理框架**：主系统（UKF管线）和南阳子系统（Alpha-Beta管线）

### 60.2 主要不足

1. **P_d=1.0 作弊模式**：所有评估结果在完美检测假设下获得
2. **UKF 参数 alpha=1e-2 导致数值不稳定**
3. **PDA 协方差修正缺失**
4. **代码重复严重**：Haversine 重复 4 次、正则化重复 2 次、模糊推理重复 2 次
5. **无单元测试**：所有验证依赖"跑仿真看图表"
6. **南阳子系统与主系统功能重叠**：两套航迹管理框架
7. **run('header.m') 反模式**：全局状态污染

### 60.3 未来方向

1. **降低 P_d**：从 1.0 降到 0.6-0.8，真实评估系统性能
2. **修正 ukf_alpha**：从 1e-2 改为 0.5 或 1.0
3. **实现完整 PDA**：添加协方差修正和新息方差修正
4. **统一代码库**：合并主系统和南阳子系统
5. **添加单元测试**：至少覆盖核心数学函数
6. **扩展运动模型**：添加 AC（匀加速）、Singer 机动模型
7. **电离层时变模型**：模拟电离层高度随时间的变化
8. **多目标完整 JPDA**：替代"作弊版" JPDA
9. **MHT（多假设跟踪）**：处理航迹交叉和杂波起始

---


---

## 第 65 章：核心模块逐行深度审查

### 65.1 ukf_jichu.m 完整审查

create_ukf（第46-96行）：
第56-64行：UKF核心参数 n=4, alpha=1e-2, beta=2.0, kappa=0.0, lam=-3.9996, n+lam=0.0004
第67-70行：Sigma点权重 Wm(1)=-9999, Wm(2:9)=1250, Wc(1)=-9996, Wc(2:9)=1250
第73-75行：R矩阵 diag([7000^2, 0.35^2, 0.5^2])
第78-79行：Q矩阵 Q_base*1e5 = diag([1e-4, 1e-8, 1e-4, 1e-8])
第82-84行：P矩阵 diag([0.05^2, 0.004^2, 0.05^2, 0.004^2])

init_ukf（第102-152行）：
第131-132行：固定换算系数111320 m/度，在纬度33度处经度距离高估约20%
第135行：速度检查50-500 m/s，覆盖亚音速到超音速，合理

prepare_ukf（第159-182行）：
第164-168行：9次skywave_geometry调用
第170-177行：P_zz = R + Sigma Wc_i*(z_i-z_hat)(z_i-z_hat)，NaN检查合理

update_with_innov（第191-246行）：
第219-223行：K = P_xz / P_zz，使用右除/Pinv数值稳定性好
第238行：P = P_pred - K*P_zz*K，缺少Joseph形式
第241行：regularize_cov_ukf补救

sigma_points_ukf（第321-340行）：
第323-327行：Cholesky失败时加1e-8*I扰动，标准数值技巧

meas_to_latlon_ukf（第388-433行）：
第409-419行：定点迭代30次收敛到1米精度，过度设计，建议减少到10次或放宽到100米阈值

### 65.2 single_track_runner.m 完整审查

INITIATING状态（第72-169行）：
第74-103行：真值辅助首次起始，从真值轨迹插值得到位置和速度
第106-131行：M/N起始超时6帧后切换到真值辅助，但M=4需要8帧确认，6帧太短
第153-169行：纯M/N滑窗逻辑

TRACKING状态（第175-266行）：
第179-184行：杂波预筛过滤is_clutter=true的点迹，PDA在单目标场景下几乎无用
第189-193行：临时禁用Vr门，params.gate_vr_ms=9999
第195-203行：连续丢点防杂波劫持，missed>=2且geo_dist>50km时拒绝
第210-214行：Probation期保护，life<=5且nis_val>50时拒绝
第218-219行：机动预检测上下文，ukf.last_det_list=dets
第225-227行：NIS历史记录无限增长，P2问题

### 65.3 nn_associate.m 完整审查

第23-26行：地理门限120km(life<=15)或60km(life>15)，合理
第34-37行：Vr门probation期形同虚设
第43-76行：三级筛选（地理距离>Vr门>马氏距离）
第79-109行：波门内点迹收集与NN关联重复计算

### 65.4 pda_weight.m 完整审查

第63-70行：归一化常数，Pg=0.8647, Pd=1.0, alpha=0.8647；该 `Pg` 对应二维卡方阈值 4，而当前关联器可能使用 12.5 乃至 72 的门限，统计语义不一致
第72-80行：权重计算，e(i)=exp(-0.5*mahal_2d(i)), beta_vec(i)=e(i)/(b+sum(e))
第92行：加权新息innov_weighted = innov_3d * beta_vec
第94-96行：NIS选取最高权重量测的马氏距离

### 65.5 卷一审查状态与续写游标

- 已完成到函数/语句块摘要层级：65.1 `ukf_jichu.m`、65.2 `single_track_runner.m`、65.3 `nn_associate.m`。
- 65.4 `pda_weight.m` 仅完成关键公式摘要，尚未覆盖全部输入校验、异常分支和性能路径，不能标记为逐行验证完成。
- 本文件的后续扩写转移到 [`code-review/`](code-review/README.md) 分卷体系；历史第 66—71 章位于 [`REVIEW_PART2.md`](REVIEW_PART2.md)。
- 下一审查游标：`association/pda_weight.m` 的函数签名、维度回退分支以及二维关联概率与三维创新更新之间的统计契约。
- 本章中的“正确”“合理”等早期判断仅代表静态阅读意见；未在验证记录中具备 `R` 或 `B` 证据的结论，不应解释为已经通过 MATLAB 运行或性能基准。
