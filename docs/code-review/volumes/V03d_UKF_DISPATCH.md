# V03d `ukf/ukf_dispatch.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `ukf/ukf_dispatch.m` |
| 覆盖范围 | 第 1—39 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `5be9c4d6df1ac346` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 随 `run_simulation` 端到端冒烟 |
| 修改范围 | 仅新增本审查文档 |

## 2. 文件职责与接口契约

### 2.1 职责

UKF 滤波器多态路由。根据 `ukf` 结构体的内部字段特征，自动选择后端滤波器实现：`ukf_imm`（IMM）、`ukf_zishiying`（自适应）、`ukf_jichu`（基础）。[S][P]

### 2.2 路由逻辑

```matlab
if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
    → ukf_imm
elseif isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')
    → ukf_zishiying
elseif isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
    → ukf_zishiying
else
    → ukf_jichu
```

## 3. 逐语句块审查

### 3.1 IMM 检测（第 24 行）

**语句职责。** `isfield(ukf, 'ukf_cv')` 检测 IMM 类型。[S]

**数学正确性。** `ukf_imm('create', ...)` 创建的结构体包含 `ukf_cv` 和 `ukf_ct` 两个子滤波器。[M]

### 3.2 自适应检测（第 27—30 行）

**语句职责。** 通过 `filter_type` 标记或机动检测字段判断自适应类型。[S]

**代码质量。** 三重条件判断：`filter_type=='zishiying'`、`maneuver_active`、`suspect_counter`。后两个字段在 `init` 后被设置，因此 `ukf_dispatch` 可以区分"刚创建的模板"和"已初始化的实例"。[S]

**问题记录。** `V03d-P2-01`：路由逻辑依赖字段存在性而非类型标记。若未来新增滤波器类型（如 EKF），需要在此处添加新的 `isfield` 检查。建议统一使用 `filter_type` 字段，删除 `maneuver_active` 和 `suspect_counter` 的路由依赖。[S]

### 3.3 委托调用（第 37 行）

**语句职责。** `[varargout{1:nargout}] = fh(action, ukf, varargin{:})`。[S]

**数学正确性。** `nargout` 确保输出数量匹配。[M]

**代码质量。** 函数句柄 `fh = @ukf_imm` 实现多态路由，避免 `switch` 字符串比较。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V03d-P2-01 | 第 27-28 行 | P2 | 路由依赖字段存在性 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：路由逻辑正确，IMM 和自适应滤波器的字段设计保证了正确分发。
- **代码质量**：路由条件可扩展但不够优雅；建议统一使用 `filter_type` 字段。
- **性能**：`isfield` 调用开销可忽略。
- **测试充分性**：端到端冒烟通过三种后端。

## 6. 下一审查游标

- 文件：`fusion/regularize_cov.m`
- 重点：特征值裁剪、双阈值策略、NaN守卫
- 稳定指纹：`d_clip = max(d, min_allowed)`
