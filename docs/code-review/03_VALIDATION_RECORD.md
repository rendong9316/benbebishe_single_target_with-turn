# MATLAB R2023a 验证记录

> 基线：`7c166d41541ccd74f23fd6c3ea0b871d8603950e` 及 2026-07-10 当前工作树。  
> MATLAB：R2023a，版本号 9.14，位置 `C:\Program Files\MATLAB\R2023a`。  
> 本记录严格区分源码确认 `S`、数学确认 `M` 与实际运行确认 `R`。只有保留命令、环境和结果摘要的已执行项目才标为 `R`；计划、模板或仅由源码推断的行为不得标为 `R`。

## 1. 状态定义

- `PASS`：已按记录中的方法完成，观测结果满足预期。
- `FAIL`：已执行且结果不满足预期。
- `PENDING`：尚未执行，或现有执行证据不足。
- `BLOCKED`：存在前置缺陷，修复或隔离前不应把结果解释为目标算法性能。
- `S-CONFIRMED`：当前源码文本足以确认事实，不表示运行行为或性能已经验证。
- `M-CONFIRMED`：闭式、量纲或标准公式复核足以确认数学事实，不表示项目端到端行为已经验证。
- `R-PASS`：已在 MATLAB R2023a 实际执行并通过。

## 2. MATLAB R2023a 命令模板

以下命令均从仓库根目录 `D:\Desktop\single_target_with-turn` 执行。模板本身不代表已经运行。

### 2.1 通用批处理模板

```bash
"C:/Program Files/MATLAB/R2023a/bin/matlab" -batch "addpath(genpath('.')); <your_code>"
```

### 2.2 带版本、随机种子和失败传播的模板

```bash
"C:/Program Files/MATLAB/R2023a/bin/matlab" -batch "addpath(genpath('.')); fprintf('MATLAB=%s\n',version); rng(1,'twister'); try; <your_code>; catch ME; disp(getReport(ME,'extended')); exit(1); end"
```

### 2.3 数学常量复核模板

```bash
"C:/Program Files/MATLAB/R2023a/bin/matlab" -batch "fprintf('MATLAB=%s\n',version); x=[4 8 12.5 72]; disp(table(x(:),chi2cdf(x(:),2),'VariableNames',{'x','CDF_df2'})); R=6371; d=R*deg2rad(1); fprintf('equator_1deg_km=%.10f\n',d);"
```

### 2.4 单入口端到端模板

```bash
"C:/Program Files/MATLAB/R2023a/bin/matlab" -batch "addpath(genpath('.')); fprintf('MATLAB=%s\n',version); rng(1,'twister'); run_simulation;"
```

### 2.5 静态分析模板

```bash
"C:/Program Files/MATLAB/R2023a/bin/matlab" -batch "addpath(genpath('.')); files=dir(fullfile(pwd,'**','*.m')); nmsg=0; nfiles=0; nagrow=0; for k=1:numel(files); f=fullfile(files(k).folder,files(k).name); m=checkcode(f,'-id'); if ~isempty(m); nfiles=nfiles+1; nmsg=nmsg+numel(m); ids={m.id}; nagrow=nagrow+sum(strcmp(ids,'AGROW')); end; end; fprintf('files=%d files_with_messages=%d messages=%d AGROW=%d\n',numel(files),nfiles,nmsg,nagrow);"
```

> 若工作树中的 `.m` 文件集合发生变化，静态分析总数必须重新生成，不能沿用本记录中的 131 文件结果。

## 3. 已完成的运行验证

| 验证 ID | 关联 Issue | 类型 | 环境 / 命令摘要 | 观测结果 | 判定 |
|---|---|---|---|---|---|
| CR-VAL-001 | CR-ISSUE-002、005 | R+M | MATLAB R2023a 9.14；执行 `chi2cdf([4,8,12.5,72],2)`。 | `chi2cdf(4,2)=0.8646647168`；`chi2cdf(8,2)=0.9816843611`；`chi2cdf(12.5,2)=0.9980695459`；`chi2cdf(72,2)` 在显示精度下约为 1。 | R-PASS。历史 91.5% 结论被数值复核否定；同时证明固定 `Pg=0.8647` 对应阈值 4，而非当前门限 72。 |
| CR-VAL-002 | CR-ISSUE-004、006 | R+M | MATLAB R2023a 9.14；球半径取 6371 km，计算赤道经度差 1° 的大圆距离。 | `111.1949266446 km`。 | R-PASS。确认该类 Haversine 输出量级和单位为 km；`d<200` 应解释为 200 km。 |
| CR-VAL-003 | CR-ISSUE-001 | R+S | MATLAB R2023a 9.14；对当前 `config/simulation_params.m` 统计两个参数赋值出现次数。 | `imm_Pi_CV_to_CT` 出现 6 次；`imm_Pi_CT_to_CV` 出现 6 次。 | R-PASS。与源码静态核对一致，共 6 组、12 条赋值。 |
| CR-VAL-004 | 全仓工程质量基线 | R | MATLAB R2023a 9.14；对当前工作树 131 个 `.m` 文件执行 `checkcode` 汇总。 | 131 个文件；70 个文件有提示；共 955 条提示；其中 `AGROW` 581 条。 | R-PASS，作为静态分析基线。提示数量不等于 955 个独立缺陷，也不自动决定严重度。 |
| CR-VAL-005 | 端到端冒烟；与 CR-ISSUE-010、015 的解释边界相关 | R | MATLAB R2023a 9.14；固定 `rng(1)` 运行 `run_simulation`。 | 入口端到端成功完成并保存结果。 | R-PASS，仅证明该固定种子下入口可运行和保存；不证明算法正确、无 oracle 辅助，也不证明 `P_d=0.6` 性能。 |

## 4. 已完成的源码或数学确认

下表不是运行记录，不标 `R`。

| 验证 ID | 关联 Issue | 核对项目 | 证据与结论 | 判定 |
|---|---|---|---|---|
| CR-VAL-006 | CR-ISSUE-001 | IMM 参数重复组数 | `config/simulation_params.m:496–523` 中两参数成组出现 6 次，值均为 0.005。 | S-CONFIRMED |
| CR-VAL-007 | CR-ISSUE-002 | 二维卡方闭式 | 自由度 2 时 `F(x)=1-exp(-x/2)`，故 `F(12.5)≈0.9980695`，`F(4)≈0.8646647`。 | M-CONFIRMED；另见 CR-VAL-001 |
| CR-VAL-008 | CR-ISSUE-003、007 | `P12` 右侧转置 | `fusion/run_track_fusion.m:179–180` 明确为 `F_cv_dt*P12*F_cv_dt' + Q_half`。历史“缺转置”是转录错误；经验传播问题仍未解决。 | S-CONFIRMED |
| CR-VAL-009 | CR-ISSUE-004、006 | 评估距离单位 | `evaluation/evaluate_all.m` 的 `haversine_km_eval` 使用 `R=6371`，返回 km；`d<200` 是 200 km。 | S+M-CONFIRMED；另见 CR-VAL-002 |
| CR-VAL-010 | CR-ISSUE-005 | 三套门统计语义 | 实际门限为 `gate_sigma^2*2`，入口常用 `gate_sigma=6`；PDA 固定 `Pg=0.8647`；诊断固定 `NIS<8`，且可能重复拼接累计历史。 | S+M-CONFIRMED |
| CR-VAL-011 | CR-ISSUE-007 | BC `P12` 近似 | 源码确认 R1 `Q*0.5`、CV `F`、迹比标量收缩、缺失时 0.5、仅依赖 `P1` 最小对角方差的统一约束；未见联合块半正定检查。 | S+M-CONFIRMED |
| CR-VAL-012 | CR-ISSUE-008 | PDA 维度契约 | `pda_weight` 用 2D 新息计算 `beta` 和 NIS，再用同一 `beta` 加权 3D 新息。 | S-CONFIRMED |
| CR-VAL-013 | CR-ISSUE-009 | IMM 组合 | `P_zz_comb` 固定 0.5/0.5；门中心固定 CV；顶层 `imm.P` 缺模型均值离差外积项。 | S+M-CONFIRMED |
| CR-VAL-014 | CR-ISSUE-010 | `P_d` 元数据冲突 | 参数为 1.0，四个 MC 表头硬编码 0.6，检测生成端读取参数值。 | S-CONFIRMED |
| CR-VAL-015 | CR-ISSUE-011 | EML 结果覆盖 | `estimate_biases` 执行并显示 `fmincon` 结果后无条件 `x_opt=x0`。 | S-CONFIRMED；LS 与 EML 优劣未验证 |
| CR-VAL-016 | CR-ISSUE-012、013 | 多目标关联入口差异 | `jpda_multi` 不读取真值且不是完整联合事件 JPDA；`run_simulation_multi` 另一流程明确使用真值辅助起始、关联和跨雷达匹配。 | S-CONFIRMED |
| CR-VAL-017 | CR-ISSUE-014 | 真值速度注入 | 函数无返回值，当前结构体修改不返回；函数插值位置而非速度，且 `x(4)=lon` 与 `[lon;lon_dot;lat;lat_dot]` 状态顺序冲突。 | S+M-CONFIRMED |
| CR-VAL-018 | CR-ISSUE-015 | 默认真值起始 | `use_truth_init=true`；单目标和 IMM 起始/重起始读取 `true_track`。 | S-CONFIRMED |

## 5. 待运行验证矩阵

| 验证 ID | 关联 Issue | 待验证问题 | 最小方法与观测量 | 当前状态 |
|---|---|---|---|---|
| CR-VAL-019 | CR-ISSUE-005 | 统一门统计后，覆盖率和关联结果如何变化？ | 固定随机种子，逐帧只记录新增样本；分别统计实际关联门通过率、2D PDA NIS 的 `chi2cdf` 覆盖率、3D 更新 NIS 覆盖率；比较去重前后差异。 | PENDING |
| CR-VAL-020 | CR-ISSUE-006 | 200 km 门限掩盖多少发散和身份错误？ | 重放单目标与多目标结果；统计 5/10/20/50/200 km 门下的接纳率；多目标比较最近邻、带 ID 约束 Hungarian、OSPA/GOSPA、重复匹配率和 ID switch。 | PENDING |
| CR-VAL-021 | CR-ISSUE-007 | BC 的联合一致性与经验参数敏感度 | 每帧记录 `eig([P1 P12;P12' P2])` 最小值、正则化触发次数、NEES/NIS；扫描 `Q_half`、`alpha` 回退值和 0.8 约束；与 CI/SCC 同种子比较。 | PENDING |
| CR-VAL-022 | CR-ISSUE-008 | 2D 关联、3D 更新在径向速度冲突时是否污染状态？ | 构造两个候选：距离/方位接近、径向速度方向相反；比较现有 2D PDA、完整 3D PDA、2D 门控后条件 Vr 似然三种结果。 | PENDING |
| CR-VAL-023 | CR-ISSUE-009 | IMM 混合门与完整协方差是否改善转弯一致性？ | 转弯段记录 CV/CT 预测中心距离、模型概率、真实量测落入 CV 门与混合门的比例、顶层 NEES、融合权重和 RMSE。 | PENDING |
| CR-VAL-024 | CR-ISSUE-010 | 真正 `P_d=0.6` 下的性能 | 显式设置 `detection_probability=0.6`，表头动态打印实际参数；采用预注册种子集合重跑 MC，报告起始率、丢轨率、关联率、RMSE 和置信区间。旧 `P_d=1` 结果不可复用。 | PENDING |
| CR-VAL-025 | CR-ISSUE-011 | LS 与 EML 哪个在独立标校集更好？ | 同一训练集分别输出 `x0` 和 `x_fmincon`，在未参与优化的验证集比较残差、参数边界命中和泛化误差；记录 `exitflag`、目标值和耗时。 | PENDING |
| CR-VAL-026 | CR-ISSUE-012、013、015 | 去除 oracle 后多目标和起始性能 | 分三组运行：oracle 上界、当前数据关联基线、无真值完整 JPDA/JIPDA；统一检测序列，报告 ID switch、正确关联率、轨迹完整性、起始延迟和丢轨率。 | BLOCKED：需先隔离真值路径并明确算法身份 |
| CR-VAL-027 | CR-ISSUE-014 | `inject_truth_velocity` 所需语义及状态契约 | 先写调用前后状态测试；若保留，验证返回值、状态索引和由相邻真值点/时间差计算的速度；禁止只补返回值而不修索引。 | BLOCKED：需先决定是否允许 oracle 注入 |
| CR-VAL-028 | CR-ISSUE-001 | 删除重复配置后多入口参数一致性 | 删除冗余属于未来源码修改；修改后逐入口打印两转移概率，并运行参数扫描冒烟，确认不存在后置覆盖。 | PENDING；本轮未改源码 |

## 6. 推荐执行顺序与记录要求

1. 先执行 CR-VAL-024、026，建立没有标题失真和真值辅助污染的实验基线。
2. 再执行 CR-VAL-019、020，统一关联、统计和评估语义。
3. 再执行 CR-VAL-022、023、021，分别验证 PDA、IMM 和 BC 的数学修正。
4. 最后执行 CR-VAL-025、027、028，清理配准、注入函数和重复配置。
5. 每次运行至少记录：Git 提交/工作树摘要、MATLAB 完整版本、命令、随机种子、参数快照、开始/结束时间、退出码、关键输出、结果文件路径。
6. “程序成功结束”只证明可运行，不等于算法正确；“源码已确认”不能升级为 `R`；单个固定种子不能替代蒙特卡洛置信区间。
