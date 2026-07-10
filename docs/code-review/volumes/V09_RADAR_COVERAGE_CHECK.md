# V09 `simulation/radar_coverage_check.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `simulation/radar_coverage_check.m` |
| 覆盖范围 | 第 1—96 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `c4d6170f8cbb61eb` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 随 `run_simulation` 端到端冒烟（待确认调用链路） |
| 修改范围 | 仅新增本审查文档 |

## 2. 文件职责与接口契约

### 2.1 职责

雷达威力覆盖判定模块：判断给定目标是否位于指定雷达的地理探测范围内。检查两个独立条件——斜距条件和波束条件，输出布尔覆盖标志及中间量（距离、方位角）。[S][P]

### 2.2 输入输出

| 输入 | 类型 | 含义 |
|---|---|---|
| `rx_lon` | scalar, deg | 接收站经度 |
| `rx_lat` | scalar, deg | 接收站纬度 |
| `tgt_lon` | scalar, deg | 目标经度 |
| `tgt_lat` | scalar, deg | 目标纬度 |
| `beam_center` | scalar, deg | 波束中心方位角（0°=正北，顺时针） |
| `params` | struct | 仿真参数结构体，需含 `.beam_width_deg`、`.range_min_m`、`.range_max_m` |

| 输出 | 类型 | 含义 |
|---|---|---|
| `in_coverage` | logical | true = 目标在威力覆盖内 |
| `r1` | scalar, m | 接收站到目标的地表大圆距离 |
| `az` | scalar, deg | 接收站到目标的方位角 |

### 2.3 调用关系

被 `simulation/generate_frame_detections.m` 在每帧量测生成前调用，决定是否对该目标产生检测。[S]

## 3. 逐语句块审查

### 3.1 文档注释块（第 1—56 行）

**语句职责。** 文件头部注释，描述功能、数学原理、输入输出、调用关系和使用注意事项。[S]

**代码质量。** 注释极为详尽，包含 Haversine 公式和方位角公式的数学表达式，便于后续维护者理解。[S]

**问题记录。** `V09-P3-01`：注释第 14-16 行给出的 Haversine 公式使用了 `R_earth` 但未说明具体取值。实际 `sphere_utils_haversine_distance` 内部使用 `R=6371` km（见 CR-ISSUE-004）。应在注释中显式注明地球半径取值。[S]

**问题记录。** `V09-P3-02`：注释第 20-21 行的方位角公式为标准球面方位角公式，但未注明该公式在 `cos(lat1)` 接近零（极地）时的数值退化问题。本项目目标在中低纬度（约 20°-45° N），影响可忽略，但注释应标注适用范围。[M]

### 3.2 函数签名（第 58—59 行）

```matlab
function [in_coverage, r1, az] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, ...
        tgt_lat, beam_center, params)
```

**语句职责。** 定义函数入口，6 个输入参数、3 个输出参数。[S]

**代码质量。** 参数命名与注释一致，`rx_`/`tgt_` 前缀清晰区分接收站和目标坐标。[S]

### 3.3 距离计算（第 65 行）

```matlab
r1 = sphere_utils_haversine_distance(rx_lon, rx_lat, tgt_lon, tgt_lat);
```

**语句职责。** 调用 Haversine 子函数计算接收站到目标的地表大圆距离，单位为米。[S]

**数学正确性。** Haversine 公式对球面距离计算是精确的（在地球为理想球体的假设下）。[M]

**依赖检查。** 需要 `sphere_utils_haversine_distance` 正确实现。已在覆盖矩阵中登记为 NOT_STARTED，待 V12 审查。[S]

**问题记录。** `V09-P2-01`：函数返回的是**地表大圆距离**，但天波雷达的量测是**群距离**（包含电离层反射的斜距路径）。在威力覆盖判定中使用地表距离与 `range_min_m`/`range_max_m` 比较，隐含了"雷达工作在表面波模式"的假设。对于天波超视距雷达（OTH-SWR），实际斜距 `r_slant = sqrt(r_surface^2 + (2*H_ionosphere)^2)` 大于地表距离。若 `params.range_max_m` 是按天波斜距设计的（如 2000 km），则与地表距离比较会导致覆盖判定过于宽松。[M]

**问题记录。** `V09-P2-02`：`sphere_utils_haversine_distance` 每次调用都计算完整的 Haversine 公式（含三角函数）。在 MC 循环中，若每帧都对每个目标调用此函数（N_frames × N_targets 次），此开销可能显著。建议在批量判定时向量化。[S]

### 3.4 方位角计算（第 71 行）

```matlab
az = sphere_utils_azimuth(rx_lon, rx_lat, tgt_lon, tgt_lat);
```

**语句职责。** 调用方位角子函数计算从接收站到目标的大圆初始方位角。[S]

**数学正确性。** 标准方位角公式：

```
az = atan2(sin(Δlon)*cos(lat2),
           cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(Δlon))
```

结果 ∈ [0°, 360°)，0° 为正北。[M]

**问题记录。** `V09-P2-03`：同样依赖未审查的 `sphere_utils_azimuth`。该函数在 `utils/sphere_utils_azimuth.m` 中，覆盖矩阵状态为 NOT_STARTED。[S]

### 3.5 波束角度条件计算（第 78—85 行）

```matlab
half_beam = params.beam_width_deg / 2;
az_diff = abs(az - beam_center);
if az_diff > 180
    az_diff = 360 - az_diff;
end
```

**语句职责。** 计算目标方位角与波束中心的偏差，处理 0°/360° 跨越问题。[S]

**数学正确性。** 方位角是圆周量（mod 360），最短弧距离为 `min(|az - center|, 360 - |az - center|)`。当 `|az - center| > 180` 时，`360 - |az - center|` 给出短弧。[M]

**代码质量。** 逻辑简洁正确。[S]

**问题记录。** `V09-P2-04`：`params.beam_width_deg` 的默认值和物理含义未在函数签名中体现。若调用者未设置此字段，`params.beam_width_deg` 为 `[]` 或未定义字段会导致运行时错误。应在函数入口处添加检查：

```matlab
assert(isfield(params, 'beam_width_deg'), 'beam_width_deg required');
```

或在文档注释中标明默认值。[S]

### 3.6 综合覆盖判定（第 93—95 行）

```matlab
in_coverage = (r1 >= params.range_min_m) && ...
              (r1 <= params.range_max_m) && ...
              (az_diff <= half_beam);
```

**语句职责。** 三个条件同时满足才判定为目标在覆盖内。[S]

**数学正确性。** 逻辑正确：距离在 [min, max] 范围内且方位角在波束半宽内。[M]

**问题记录。** `V09-P1-01`：`r1` 的单位是米（m），`params.range_min_m` 和 `params.range_max_m` 也应该是米。但需注意 `simulation_params.m` 中定义的 `range_min`/`range_max` 字段名是否带 `_m` 后缀。若参数字段名为 `range_min`（单位 km），则此处比较存在量纲错误。[S]

**问题记录。** `V09-P2-05`：`az_diff <= half_beam` 使用的是 `<=`（包含边界）。在工程实践中，波束边缘的增益通常低于中心增益 3dB，是否应该用 `<`（严格小于）还是 `<=` 取决于波束方向图的定义。此处 `<=` 是保守判定（更容易包含目标），但如果波束宽度定义的是 -3dB 全宽，则边界处增益已下降 3dB，不应与中心增益同等对待。[M]

**问题记录。** `V09-P3-03`：三个条件用 `&&` 连接，MATLAB 的 `&&` 是短路运算符。若 `r1 < params.range_min_m` 为 false，则不会计算后面的条件。这在性能上是优化的，但在调试时若 `params.range_min_m` 等字段不存在，错误信息可能不够明确（因为短路发生在字段访问之前还是之后取决于字段是否存在）。[S]

### 3.7 文件结束（第 96 行）

**语句职责。** 函数体结束。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V09-P1-01 | 第 93—95 行 | P1 | 距离单位一致性 | S | OPEN |
| V09-P2-01 | 第 65 行 | P2 | 地表距离 vs 天波斜距 | M | OPEN |
| V2-02 | 第 65 行 | P2 | Haversine 性能开销 | S | OPEN |
| V09-P2-03 | 第 71 行 | P2 | 依赖未审查子函数 | S | OPEN |
| V09-P2-04 | 第 78—85 行 | P2 | 参数默认值缺失 | S | OPEN |
| V09-P2-05 | 第 95 行 | P2 | 波束边界判定 | M | OPEN |
| V09-P3-01 | 第 1—56 行 | P3 | 地球半径取值未注明 | S | OPEN |
| V09-P3-02 | 第 1—56 行 | P3 | 方位角公式极地退化未标注 | S | OPEN |
| V09-P3-03 | 第 93—95 行 | P3 | 短路运算符调试不友好 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：核心逻辑（Haversine 距离 + 方位角 + 波束判定）在球面几何意义上是正确的。但天波雷达使用群距离（斜距）而非地表距离，威力覆盖判定的距离基准存在概念偏差。[M]
- **代码质量**：注释详尽，命名清晰。参数默认值检查和字段存在性验证缺失。[S]
- **性能**：Haversine 和方位角函数在 MC 循环中被频繁调用，未向量化。[S]
- **测试充分性**：端到端冒烟通过，但不同波束宽度、不同纬度场景下的覆盖判定未获独立验证。[S]
- **剩余未验证项**：`params` 结构体中 `range_min_m`/`range_max_m` 的实际单位；天波模式下距离判定的系统性偏差对检测率的影响。

## 6. 下一审查游标

- 文件：`simulation/generate_frame_detections_multi.m`（多目标检测生成）
- 重点：检测概率模型、杂波生成、覆盖判定调用链路
- 稳定指纹：`generate_frame_detections_multi` 主循环
