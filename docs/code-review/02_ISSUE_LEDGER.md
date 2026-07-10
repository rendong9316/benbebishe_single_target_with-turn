# 代码审查问题台账

> 基线：`7c166d41541ccd74f23fd6c3ea0b871d8603950e` 及 2026-07-10 当前工作树。  
> 本台账区分源码、数学与运行证据。2026-07-10 已完成 MATLAB R2023a 的确定性数学核验、全仓 `checkcode` 汇总和 `run_simulation` 端到端冒烟；只有明确引用这些记录的条目才可增加 `R`，其余行为、性能变化、发散率或优化收益不得由静态阅读推定。

## 1. 字段与状态

- **严重度**：`P0` 表示实验真实性或结论有效性阻断；`P1` 表示高优先级算法、统计或评估风险；`P2` 表示工程质量或潜在配置漂移风险。
- **证据等级**：`S` 为源码静态确认；`M` 为数学复核确认；`R` 为 MATLAB R2023a 实际运行确认；`B` 为规范化性能基准；`L` 为文献依据；`H` 为未复核历史说法；`X` 为已证伪。当前仅验证记录明确列出的事实拥有 `R`，尚无 `B` 结论。
- **状态**：`OPEN` 为源码问题仍存在；`DOC_CORRECTED` 为历史文档事实已在勘误表更正，但不代表相关源码风险已修复；`PENDING_RUN` 为静态事实已确认、行为或性能影响仍待运行；`DISPROVED` 为历史结论被当前证据否定。
- 行号基于当前工作树；源码变化后必须重新定位符号和稳定代码片段。

## 2. 历史文档四项已确认错误

| ID | 文件 / 符号 / 行区间 | 严重度 | 证据 | 已确认事实 | 影响 | 建议 | 状态 |
|---|---|---:|---|---|---|---|---|
| CR-ISSUE-001 | `config/simulation_params.m`；`params.imm_Pi_CV_to_CT`、`params.imm_Pi_CT_to_CV`；496–523（六组分别为 497–498、502–503、507–508、512–513、517–518、522–523）。历史位置：`docs/CODE_REVIEW.md` 第 1 章及其拼接副本。 | P2 | S；`docs/code-review/04_CORRECTION_LOG.md` COR-001 | 历史稿称“重复定义 8 次且是 Git 合并冲突残留”不成立。当前源码是 6 组、12 条赋值，首组外有 5 组冗余，值均为 `0.005`；源码不足以证明来源一定是 Git 冲突。 | 当前生效值不变，但后续只修改较早一组会被后续赋值静默覆盖。 | 保留一组权威配置；扫描脚本应修改参数值而不是依赖文本中首个匹配。 | DOC_CORRECTED；源码冗余 OPEN |
| CR-ISSUE-002 | `association/nn_associate.m`；`gate_threshold`；28–29。相关配置：`config/simulation_params.m` 304–307、450–452。历史位置：`docs/CODE_REVIEW.md` 第 1.2.2 节。 | P1 | S+M+R；二维卡方闭式 `F(x)=1-exp(-x/2)`；MATLAB R2023a 数值见验证记录；COR-002 | 历史稿把 `chi2cdf(12.5,2)` 写成约 91.5%，正确值为 `1-exp(-6.25)≈99.807%`，理论门外率约 0.193%；`0.8647` 对应阈值 4，而不是 12.5。 | 错误概率会误导波门松紧和漏门率判断；源码另有实际门限与固定 `Pg` 不一致风险，见 CR-ISSUE-005。 | 文档和验证统一使用“维数、平方马氏阈值、门内概率”三元组。 | DOC_CORRECTED；数值已由 R2023a 确认 |
| CR-ISSUE-003 | `fusion/run_track_fusion.m`；BC 分支 `P12_pred`；166–180，核心 179–180。历史位置：`docs/REVIEW_PART2.md` 第 68 章。 | P1 | S；COR-003 | 历史稿漏抄右侧转置并称源码为 `F*P12*F + Q_half`；当前源码实际是 `F_cv_dt*P12_cell{p}*F_cv_dt' + Q_half`。缺转置是文档转录错误，不是当前源码缺陷。 | 若沿用错误转录会提出错误修复；但 `P12` 的经验传播与约束仍有独立风险，见 CR-ISSUE-007。 | 引用公式时直接链接源码行；把“有转置”与“传播是否统计一致”分开审查。 | DOC_CORRECTED；原缺陷判断 DISPROVED |
| CR-ISSUE-004 | `evaluation/evaluate_all.m`；`compute_tracking_errors` 25–79，核心 53–55；`haversine_km_eval` 301–311。历史位置：`docs/REVIEW_PART2.md` 第 69 章。 | P1 | S+M；函数使用 `R=6371`；COR-004 | 历史稿把 `d<200` 解释成 200 m，并建议改成 5000 表示 5 km。函数返回 km，故该阈值实际是 200 km；5000 将是 5000 km。 | 错误单位判断会导致错误修复；真正风险是 200 km 可能过宽，见 CR-ISSUE-006。 | 参数名显式携带单位；禁止把 200 机械替换成 5000；结合身份匹配和场景尺度重新设计门限。 | DOC_CORRECTED；原单位判断 DISPROVED |

## 3. 当前源码风险

| ID | 文件 / 符号 / 行区间 | 严重度 | 证据 | 已确认事实 | 影响 | 建议 | 状态 |
|---|---|---:|---|---|---|---|---|
| CR-ISSUE-005 | `config/simulation_params.m` 304–307、375–379、450–452；`association/nn_associate.m:28–29`；`association/pda_weight.m:62–70`；入口写回示例 `run_mc_turn.m:205,220`、`run_mc_multi.m:243,252`；诊断 `run_mc_turn.m:578–610`，核心 603–606。 | P1 | S+M | 主流程常把 `gate_sigma=6` 写入关联器，实际二维平方马氏门限为 `6^2*2=72`；PDA 仍使用固定 `Pg=0.8647`；“NIS门内”诊断又固定用 `<8`。诊断还逐帧拼接每个快照中的完整累计 `nis_history`，使早期样本重复计权。 | 关联门、PDA 无检测概率项和报表覆盖率具有不同统计语义，报告百分比不能解释为同一个门的命中率。 | 从实际门限和维数计算 `Pg`；分别命名并报告关联门通过率、2D PDA NIS 覆盖率和 3D 更新 NIS 覆盖率；每帧只追加新 NIS。 | OPEN；行为影响 PENDING_RUN |
| CR-ISSUE-006 | `evaluation/evaluate_all.m:48–61`，核心 53–55；`evaluation/evaluate_all_multi.m:48–60`，核心 53–55；多目标距离辅助函数约 321–330。 | P1 | S+M | 最近航迹只需位于真值 200 km 内即可被接纳。多目标版本没有身份约束和一对一分配，同一航迹可能成为多个目标的最近航迹。 | 100–199 km 的发散航迹仍可能计入；身份错误、换轨和重复匹配可能被最近距离最小化掩盖，使 RMSE 偏乐观。 | 单目标门限按场景校准；多目标采用带身份约束的一对一分配，并单列未匹配、重复匹配、ID switch、OSPA/GOSPA。 | OPEN；数值影响 PENDING_RUN |
| CR-ISSUE-007 | `fusion/run_track_fusion.m`；BC 分支 162–224：传播 166–180、迹收缩 182–199、约束 201–212；`fusion/track_fusion_algorithms.m`；`fuse_bc` 150–168、186–230。 | P1 | S+M | `P12` 仅用 R1 的 `Q*0.5` 和 CV 状态转移传播；更新用迹比得到的单一标量 `alpha`，缺预测信息时固定乘 0.5；约束以 `0.8*min(diag(P1))*eye(4)` 统一限制不同量纲状态，不使用 `P2`，也未验证联合块协方差半正定。 | `P12` 缺乏已证明的统计含义；BC 可能过度自信或依赖正则化，BC/CI/SCC 比较可能主要反映经验参数。 | 推导或明确近似假设；至少逐帧检查联合块协方差特征值、NEES/NIS，并做 `Q_half`、`alpha` 和 0.8 约束敏感性分析。 | OPEN；一致性与性能 PENDING_RUN |
| CR-ISSUE-008 | `association/pda_weight.m`；`pda_weight`；二维截取 44，二维新息/马氏距离 46–60，权重 62–80，三维新息更新 82–92，返回 2D NIS 94–96。 | P1 | S+M | 关联概率只使用距离和方位的 2D 新息及 `P_zz(1:2,1:2)`，但同一组 `beta` 随后对含径向速度的 3D 新息加权并驱动更新；返回的也是 2D NIS。 | 当候选量测位置接近但径向速度相反时，错误候选可能污染速度更新；2D NIS 还可能被误作 3D 更新一致性统计。 | 明确“2D 门控、3D 条件更新”契约；比较完整 3D PDA，或在 2D 门控后加入条件径向速度似然并重新归一化；分开记录 2D 与 3D NIS。 | OPEN；场景影响 PENDING_RUN |
| CR-ISSUE-009 | `ukf/ukf_imm.m`；`prepare_imm`、`update_imm`、`imm.P`；190–214、280–288。 | P1 | S+M | 组合测量均值使用模型概率 `mu`，但 `P_zz_comb` 固定按 0.5/0.5 加权且缺模型均值离差项；当前 tracker 未实际使用该缓存量。关联层输出又固定采用 CV 的 `x_pred_cv/z_pred_cv/P_zz_cv`，即使 CT 概率占优。顶层 `imm.P=mu(1)P_cv+mu(2)P_ct` 缺少各模型均值相对组合均值的外积项。 | 转弯时 IMM 的 CT 优势未传到波门中心；输出协方差低估模型分歧，下游时间对齐和融合可能过度自信。 | 采用完整混合矩生成门中心和测量协方差；输出协方差加入模型均值离差项；删除或正确使用无效缓存量。 | OPEN；转弯行为与数值影响 PENDING_RUN |
| CR-ISSUE-010 | 实际参数：`config/simulation_params.m:399–402`、`config/simulation_params_multi.m:11,30`。硬编码标题：`run_mc_straight.m:52–55`、`run_mc_turn.m:65–70`、`run_mc_turn_180deg.m:61–66`、`run_mc_multi.m:55–60`。检测端：`simulation/generate_frame_detections.m:123`、`simulation/generate_frame_detections_multi.m:56`。 | P0 | S | 当前参数 `detection_probability=1.0`，四个 MC 表头却打印 `Pd=0.6`；入口未见随后覆盖为 0.6，检测生成端读取实际参数。 | 实验元数据失真；现有结果不能被描述为“60% 检测概率下的性能”，漏检鲁棒性没有按标题所称条件测试。 | 表头动态读取实际参数；将 `P_d=1` oracle/debug 与 `P_d=0.6` 实验分开；若要声明 0.6 性能，必须重新运行，旧统计不可复用。 | OPEN；实际配置 S-CONFIRMED，0.6 性能未运行 |
| CR-ISSUE-011 | `registration/estimate_biases.m`；`estimate_biases`；LS 初值 417–426，约束优化 448–498，结果显示 501–505，无条件覆盖 507–515，输出 517–526。 | P1 | S | 代码执行 `fmincon` 并打印结果后，无条件执行 `x_opt=x0`，因此最终输出始终恢复为 LS 初值；“LS + EML 精化双阶段”与实际返回行为不一致。 | EML 对返回值无贡献，却增加运行时间和 Optimization Toolbox 依赖；用户可能误认输出是约束优化结果。 | 显式选择 LS-only 或经验证的 EML 输出；若保留比较，返回两组结果和独立验证指标，不要运行后静默覆盖。 | OPEN；“LS 优于 EML” PENDING_RUN |
| CR-ISSUE-012 | `association/jpda_multi.m`；`jpda_multi`；1–27、38–134。调用：`run_mc_multi.m:409`。 | P1 | S | 文件头称“真值辅助作弊版”，签名含未使用的 `truth_all`，但函数体不读取真值，唯一调用仅传四个参数。实现是逐航迹独立收门和 PDA 加权，检测可同时进入多条航迹门，不是对联合互斥事件求和的完整 JPDA。 | 名称、注释、算法和入口行为不一致，容易把独立 PDA 误报为 JPDA；多入口结果不可直接互证。 | 移除未使用真值形参或明确 oracle 版本；若声称 JPDA，应实现联合事件、互斥约束和边缘关联概率，并建立交叉目标测试。 | OPEN；算法差异 S-CONFIRMED |
| CR-ISSUE-013 | `run_simulation_multi.m`；多目标跟踪包装器：真值辅助起始 650–718，真值辅助关联 746–795，更新 817–857；跨雷达匹配默认值 386–390、真值路径 403–427。 | P0 | S | 交互式多目标入口明确使用真值位置/`ac_idx` 做“上帝视角”关联，并默认 `match_method='truth_assisted'`；该流程绕过 `jpda_multi`。 | 关联、起始和跨雷达匹配结果是 oracle 上界，不代表无真值辅助算法性能；与 `run_mc_multi` 研究的并非同一流程。 | 默认真实算法路径不得读取真值或生成标签；oracle 结果单独命名和报告；同时报告 ID switch、正确关联率和轨迹完整性。 | OPEN；真实性风险 S-CONFIRMED，性能幅度 PENDING_RUN |
| CR-ISSUE-014 | `tracker/inject_truth_velocity.m:1–12`；调用 `run_simulation_multi.m:694–697,708–711`；状态定义参照 `ukf/ukf_jichu.m:116–143`。 | P1 | S+M | 函数名声称注入速度，实际只插值经纬度，并写 `x(3)=lat`、`x(4)=lon`；状态顺序是 `[lon; lon_dot; lat; lat_dot]`，故 `x(4)` 索引语义错误。函数无返回值，结构体按值传入，当前调用修改不返回，实际为空操作。 | 当前代码误导且无效；若未来只补返回值而不修索引，会把经度写入纬度速度槽并造成严重状态污染。 | 先确定是否允许 oracle 信息；若禁止则删除调用；若仅用于测试，返回修改后的结构并按时间差计算真实速度，增加调用前后状态单元测试。 | OPEN；当前不生效 S-CONFIRMED |
| CR-ISSUE-015 | `config/simulation_params.m:327–331`；`tracker/single_track_runner.m:72–103`；`imm/imm_tracker.m:88–160,169–223`。 | P0 | S | `params.use_truth_init=true` 默认开启；单目标与 IMM 起始/重起始从 `true_track` 插值得到真值位置并构造初始化量测，IMM 还把 `has_truth` 无条件置真。 | 初始位置、速度方向、起始延迟和重捕获性能可能偏乐观；与 `P_d=1` 叠加后，M/N、漏检和重起始机制未接受真实压力测试。 | 默认关闭真值起始；仅在明确标记的 oracle/debug 模式启用；真实实验必须检测驱动起始并单独报告起始延迟和失败率。 | OPEN；真实性风险 S-CONFIRMED，性能幅度 PENDING_RUN |
| CR-ISSUE-016 | `ukf/ukf_jichu.m:55–70`；`alpha` 默认 1e-2 | P1 | S+M | `alpha=1e-2` 时 `lam≈-3.9996`，`Wm(1)≈-9999`，UKF 退化为近似 EKF，数值稳定性差。 | 滤波行为实质是 EKF 而非标准 UKF；`alpha` 不同取值对 RMSE 的影响未验证。 | 建议 `alpha=0.5` 或 `1.0`；或改用 CV/CT 的 EKF 替代方案。 | OPEN；数值影响 PENDING_RUN |
| CR-ISSUE-017 | `ukf/ukf_imm.m:193` | P1 | S+M | `P_zz_comb` 固定 0.5/0.5 加权且缺模型均值离差项。 | 组合测量协方差不反映当前模型概率，转弯时 CT 占优仍使用 CV 的协方差估计。 | 使用 `mu` 加权并加入离差项。 | OPEN |
| CR-ISSUE-018 | `ukf/ukf_imm.m:196–198` | P1 | S+M | 返回给 tracker 的门中心固定为 CV 模型预测，不随 `mu` 变化。 | 转弯时 CT 优势无法传递到关联层。 | 返回组合预测 `z_pred_comb`。 | OPEN |
| CR-ISSUE-019 | `ukf/ukf_imm.m:287` | P1 | S+M | 顶层组合协方差 `imm.P = mu(1)*P_cv + mu(2)*P_ct` 缺模型均值离差外积。 | 模型分歧大时严重低估不确定性，下游融合可能过度自信。 | 加入 `Σ mu_i * (x_i - x_comb)(x_i - x_comb)'`。 | OPEN |
| CR-ISSUE-020 | `ukf/ukf_zishiying.m:154–160` | P1 | S+M | 宽门预检测使用 `[drange; daz]` 的 2D 新息计算马氏距离，但 `P_zz(1:2,1:2)` 中 `(1,1)` 是 m²、`(2,2)` 是 deg²，量纲不一致。 | 马氏距离无统计意义，预检测可能产生大量假阳性。 | 统一量纲（如将方位差转为横向距离 m）。 | OPEN |
| CR-ISSUE-021 | `ukf/ukf_jichu.m:131–132` | P2 | S+M | 两点差分速度使用固定换算系数 `111320 m/deg`，未考虑 `cos(lat)` 纬度修正。 | 经度方向速度高估约 20%（纬度 33° 处）。 | 使用 `111320 * cos(lat)`。 | OPEN |
| CR-ISSUE-022 | `ukf/ukf_jichu.m:388–433` | P2 | S+M | 反解迭代 30 次收敛到 1 米精度，过度设计。 | 实时跟踪不需要 1 米精度，10 次迭代 + 100 米阈值足够。 | 减少迭代次数和放宽阈值。 | OPEN |
| CR-ISSUE-023 | `ukf/ukf_imm.m:371` vs `ukf/ukf_zishiying.m:261` | P3 | S | `trimf_val_imm` 与 `trimf_val_maneuver` 完全相同，重复实现。 | 维护成本增加。 | 提取为共享工具函数。 | OPEN |
| CR-ISSUE-024 | `association/pda_weight.m:27` | P1 | S | `P_zz` 维度检查要求严格 3×3，2×2 输入直接回退。 | 当 NN 返回 2×2 `P_zz_2d` 时 PDA 完全失效。 | 支持自适应维度。 | OPEN |
| CR-ISSUE-025 | `association/pda_weight.m:82-92` | P1 | S+M | 2D 关联概率 `beta_vec` 用于 3D 新息加权。 | 径向速度冲突时错误候选污染速度更新。 | 3D PDA 或 2D 门控后加 Vr 条件似然。 | OPEN |
| CR-ISSUE-026 | `fusion/run_track_fusion.m:179` | P1 | S+M | BC `Q_half` 仅用 R1 的 Q，R1/R2 Q 不同（scale 1e5 vs 2e5）时不对称。 | P12 传播物理不一致。 | 使用 `0.5*(Q1+Q2)` 或分别传播。 | OPEN |
| CR-ISSUE-027 | `fusion/run_track_fusion.m:191-194` | P1 | S+M | 迹收缩比 `alpha` 混合量纲（deg² 与 deg²/s²），迹本身无明确物理意义。 | P12 更新缺乏严格数学推导。 | 使用 Cholesky 因子或显式 (I-KH) 近似。 | OPEN |
| CR-ISSUE-028 | `fusion/run_track_fusion.m:204` | P1 | S+M | 稳定性约束 `0.8*min(diag(P1))*eye(4)` 用位置方差约束速度协方差。 | 量纲混合约束不合理。 | 分别限制位置和速度分量。 | OPEN |
| CR-ISSUE-029 | `fusion/track_fusion_algorithms.m:394-396` | P1 | S+M | FCI 迹权重 `w_fci = tr(P1)^{-1}/(tr(P1)^{-1}+tr(P2)^{-1})` 在量纲混合状态非单位不变。 | 将速度从 deg/s 改为 rad/s 会完全改变权重分配。 | 使用对角归一化或米制协方差。 | OPEN |
| CR-ISSUE-030 | `evaluation/evaluate_all.m:54` | P1 | S+M | 200 km 门限过宽，单目标可能掩盖身份错误，多目标完全失效。 | 融合 RMSE 可能基于错误航迹匹配。 | 多目标改用一对一分配+OSPA。 | OPEN |
| CR-ISSUE-031 | `simulation/bistatic_inverse_solver.m:24` | P1 | S+M | 天波群距离 Rg 代入地表大圆反解公式，系统性偏差 ~16%。 | 反解经纬度存在固定偏移，影响偏差校正精度。 | 使用天波斜距反解公式或迭代修正。 | OPEN |
| CR-ISSUE-032 | `fusion/time_align_tracks.m:115` | P1 | S+M | 回退 Q 缩放仅 43%（13/30），反直觉。 | 回退不应增加过程噪声（确定性转移）。 | 回退时不加 Q 或加极小 Q。 | OPEN |
| CR-ISSUE-033 | `initiation/track_initiation.m:115-130` | P1 | S+M | 共识评分假设直线运动，转弯场景失效。80km 门限硬编码。 | 转弯场景 M/N 起始可能失败。 | 转弯感知共识评分 + 参数化门限。 | OPEN |
| CR-ISSUE-034 | `tracker/single_track_runner.m:191` | P1 | S | Vr 门硬编码禁用（9999 m/s），原因未说明。 | 杂波 Vr 过滤能力丧失。 | 恢复 Vr 门或说明禁用原因。 | OPEN |
| CR-ISSUE-035 | `tracker/single_track_runner.m:210` | P2 | S+M | probation NIS 门槛 50 过高，形同虚设。 | 初期航迹保护失效。 | 降至 10-20。 | OPEN |
| CR-ISSUE-036 | `registration/estimate_biases.m:514` | P0 | S | fmincon 优化结果被 x_opt=x0 静默覆盖。EML 徒增运行时间。 | "LS+EML 双阶段"实际只使用 LS。 | 启用 EML 或移除优化代码。 | OPEN |
| CR-ISSUE-037 | `ukf/ukf_dispatch.m:27-28` | P2 | S | 路由依赖字段存在性而非类型标记，扩展性差。 | 新增滤波器类型需修改路由逻辑。 | 统一使用 filter_type 字段。 | OPEN |
| CR-ISSUE-038 | `fusion/regularize_cov.m:126` | P2 | S+M | max_d 为负时双阈值计算异常。 | 全负特征值矩阵被提升后丢失方向信息。 | 增加 max_d 为负时的特殊处理。 | OPEN |
| CR-ISSUE-039 | `association/pda_weight.m:67-70` | P1 | S+M | PDA 归一化使用 2D 行列式但应用于 3D 新息，维度语义不清。 | 绝对质量失真。 | 统一 2D 或 3D 体积计算。 | OPEN |
| CR-ISSUE-040 | `simulation/radar_coverage_check.m:65` | P1 | S+M | 地表大圆距离 vs 天波斜距：威力覆盖判定使用地表距离与 range_min/max_m 比较，但天波雷达的量测是群距离（含电离层斜距），系统性偏差 ~16%。 | 覆盖判定过于宽松，检测率偏乐观。 | 使用天波斜距替代地表距离进行覆盖判定。 | OPEN |
| CR-ISSUE-041 | `fusion/track_matcher.m:78` | P1 | S | 枚举法 `perms` 在 n > 10 时 OOM（3.6M! 排列）。代码用 `n_r1 <= 4` 限制规避，但注释称"匈牙利算法"不准确。 | 未来移除限制会导致 OOM；算法描述不准确。 | 改用 MATLAB 内置 `matchpairs` 或 KM 算法。 | OPEN |
| CR-ISSUE-042 | `fusion/track_matcher.m:93-96` | P1 | S | 距离门限过滤在匈牙利分配**之后**进行，分配已将远距离配对选为"最优"。 | 50 km 门限形同虚设。 | 在代价矩阵中将超出门限的配对设为 inf。 | OPEN |
| CR-ISSUE-043 | `fusion/track_matcher.m:207-208` | P1 | M | UKF 速度槽位单位假设（弧度/秒 vs 度/秒）未验证。若 UKF 内部使用弧度，`111000` 转换系数错误。 | 速度和航向计算完全错误。 | 确认 UKF 状态单位并修正转换系数。 | OPEN |
| CR-ISSUE-044 | `ukf/ukf_imm.m:330` | P1 | S+M | `nis_history` 混合 2D NIS（位置）和 3D NIS（含径向速度），期望值不同（2 vs 3），mean 统计意义不明确。 | 模糊自适应 Q 调整的输入 NIS 比率不准确。 | 分开统计 2D 和 3D NIS。 | OPEN |
| CR-ISSUE-045 | `ukf/ukf_imm.m:360` | P1 | S+M | `abs(Q_ema - 1.0) < 0.05` 魔法阈值，缺乏理论依据。 | 小幅 NIS 波动不触发 Q 调整，但 0.05 窗口的合理性未验证。 | 基于 NIS 分布的置信区间确定阈值。 | OPEN |
| CR-ISSUE-046 | `fusion/track_matcher.m:37-45` | P2 | S | `params` 参数传入但未被使用，权重和门限硬编码。 | 不同场景无法调整阈值。 | 从 params 读取权重和门限。 | OPEN |
| CR-ISSUE-047 | `simulation/radar_coverage_check.m:78-85` | P2 | S | `params.beam_width_deg` 默认值缺失，无字段存在性检查。 | 调用者未设置字段时运行时错误。 | 添加 assert 或默认值。 | OPEN |
| CR-ISSUE-048 | `simulation/radar_coverage_check.m:93-95` | P2 | S | `range_min_m`/`range_max_m` 字段名是否带 `_m` 后缀依赖 `simulation_params.m` 的实现，未在本函数验证。 | 量纲不一致时静默错误。 | 在函数入口验证字段存在和单位。 | OPEN |
| CR-ISSUE-049 | `ukf/ukf_imm.m:301-310` | P2 | S | `keep_prediction` 缺 `otherwise` 分支，无效 model 返回未修改 ukf。 | 后续代码使用过期状态。 | 添加 otherwise 报错。 | OPEN |
| CR-ISSUE-050 | `tracker/post_init_multi.m:2` | P2 | S | `ukf.dt` 无条件覆盖，无警告。 | 调用者设置的 dt 被静默覆盖。 | 添加覆盖检查或警告。 | OPEN |
| CR-ISSUE-051 | `simulation/generate_frame_detections_multi.m:56` | P1 | S | `detection_probability=1.0` 与 Pd=0.6 MC 表头矛盾 | 虚警生成中检测概率实际为 100% | 表头动态读取实际参数。 | OPEN |
| CR-ISSUE-052 | `simulation/generate_frame_detections_multi.m:82-103` | P1 | M | 虚警位置在 range-az 矩形区域均匀采样 ≠ 球面均匀分布 | 高纬度虚警密度人为偏高 | 使用球面均匀采样。 | OPEN |
| CR-ISSUE-053 | `ukf/ukf_imm.m:82-91` | P1 | S+M | Markov 转移概率 0.005（切换周期 200 帧），CT 模型贡献微弱 | 转弯时 IMM 退化为单 UKF-CV | 提高转移概率或场景自适应。 | OPEN |
| CR-ISSUE-054 | `ukf/ukf_imm.m:193` | P1 | S | `P_zz_comb` 固定 0.5/0.5 不用 mu | 组合测量协方差失真 | 使用 mu 加权。 | OPEN |
| CR-ISSUE-055 | `ukf/ukf_imm.m:196-198` | P1 | S | 门中心固定 CV 预测，CT 优势不传递 | 转弯时关联层无法利用 CT 模型 | 返回组合预测 `z_pred_comb`。 | OPEN |
| CR-ISSUE-056 | `ukf/ukf_imm.m:287` | P1 | S+M | 顶层组合协方差缺模型均值离差外积 | 模型分歧大时低估不确定性 | 加入 `Σ mu_i * (x_i - x_comb)(x_i - x_comb)'`。 | OPEN |
| CR-ISSUE-057 | `initiation/track_initiation.m:115-130` | P1 | M | 共识评分直线假设转弯失效 | 转弯场景 M/N 起始可能失败 | 转弯感知共识评分。 | OPEN |
| CR-ISSUE-058 | `initiation/track_initiation.m:108-113` | P1 | M | 反解偏差（~16%）通过 Haversine 距离传播到速度估计 | 速度检验范围 30-600 m/s 可能不适用 | 使用反解修正后的距离。 | OPEN |
| CR-ISSUE-059 | `simulation/generate_frame_detections_multi.m:83-84` | P2 | S | 虚警杂波多普勒 `[-200, 200]` 均匀分布不物理 | 杂波多普勒应以 0 为中心 | 使用高斯或 clipped uniform。 | OPEN |
| CR-ISSUE-060 | `ukf/ukf_imm.m:138` | P2 | S | `init_imm` 覆盖 `create_imm` 的初始模型概率 | 自定义初始概率被静默覆盖 | 保留 create 设置的初始概率。 | OPEN |

## 4. 处理优先级

1. **先阻断实验真实性误报**：CR-ISSUE-010、013、015、036、051、053、055、056；同时明确 CR-ISSUE-012 的算法身份。
2. **再统一评估与概率语义**：CR-ISSUE-005、006、008、025、030、043、052、057、058。
3. **再修正滤波与融合数学**：CR-ISSUE-007、009、016、017、018、019、020、024、026、027、028、029、031、032、037、038、041、044、045、054。
4. **最后清理高风险误导与配置冗余**：CR-ISSUE-001、011、014、021、022、023、033、034、035、039、040、042、046—050、059、060。
5. 所有修复后的行为与性能结论必须进入 `03_VALIDATION_RECORD.md`，只有实际使用 MATLAB R2023a 执行并保留输出摘要后才能标为 `R`。
