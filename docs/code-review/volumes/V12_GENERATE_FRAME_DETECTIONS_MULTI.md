# V12 `simulation/generate_frame_detections_multi.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `simulation/generate_frame_detections_multi.m` |
| 覆盖范围 | 第 1—104 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `ee66ed28d5ff2e6a` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 被 `run_simulation_multi.m` 每帧调用 |
| 修改范围 | 仅新增本审查文档 |

## 2. 文件职责与接口契约

### 2.1 职责

多目标单帧点迹生成函数：对 N 个目标逐个应用检测概率模型，叠加系统偏差和随机噪声生成目标点迹；同时按泊松分布生成虚警杂波。每个检测点迹带有 `aircraft_id` 字段标记所属目标。[S][P]

### 2.2 输入输出

| 输入 | 类型 | 含义 |
|---|---|---|
| `rx_lon, rx_lat` | deg | 接收站经纬度 |
| `tx_lon, tx_lat` | deg | 发射站经纬度 |
| `tgt_states` | [N×5] | 目标状态 `[lon, lat, lon_rate, lat_rate, aircraft_id]` |
| `frameID` | scalar | 帧编号 |
| `time_sec` | scalar | 仿真时间（秒） |
| `range_bias, az_bias` | m, deg | 系统偏差 |
| `beam_center` | deg | 波束中心方位角 |
| `params` | struct | 雷达参数 |
| `range_noise, az_noise` | m, deg | 噪声标准差（可选，默认从 params 读取） |

| 输出 | 类型 | 含义 |
|---|---|---|
| `detList` | struct array | 所有点迹（目标 + 杂波） |
| `has_target_dets` | [N×1] logical | 每个目标是否被检测到 |

### 2.3 调用关系

被 `run_simulation_multi.m` 在主循环中每帧调用。内部调用 `radar_coverage_check`、`skywave_geometry`、`sphere_utils_destination_point`。[S]

## 3. 逐语句块审查

### 3.1 默认参数回退（第 35—36 行）

```matlab
if nargin < 15, range_noise = params.radar1_range_noise_std_m; end
if nargin < 16, az_noise = params.radar1_azimuth_noise_std_deg; end
```

**语句职责。** 若调用者未传入噪声标准差，从 params 读取默认值。[S]

**代码质量。** 使用 `nargin` 检查可选参数是 MATLAB 惯用法。但 `nargin` 在嵌套函数中的行为依赖于调用者的 nargin，此处是顶层函数，行为正确。[S]

**问题记录。** `V12-P2-01`：`params.radar1_range_noise_std_m` 和 `params.radar1_azimuth_noise_std_deg` 字段名暗示"radar1"，但此函数是通用的单雷达检测生成函数，不区分 R1/R2。在多雷达场景下，调用者应传入对应雷达的噪声参数。若 `params` 中不存在这些字段，会引发运行时错误。[S]

### 3.2 杂波数量生成（第 43 行）

```matlab
n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate);
```

**语句职责。** 按泊松分布生成虚警数量，参数 λ = 分辨率单元数 × 虚警率。[S]

**数学正确性。** 泊松杂波模型是雷达仿真中的标准假设（Clift 1998, IEEE T-AES）。λ 的物理含义：在单位面积内，每单元的虚警概率乘以总单元数。[M]

**问题记录。** `V12-P2-02`：`params.n_resolution_cells` 的定义和计算方式未在函数中体现。若此值是固定的（如恒为 1000），而实际覆盖区面积随目标位置变化，则虚警密度不物理。[S]

### 3.3 目标检测循环（第 46—79 行）

```matlab
[in_cov, ~, ~] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, ...
    beam_center, params);
if in_cov && rand() <= params.detection_probability
```

**语句职责。** 先判定威力覆盖，再在覆盖内以 `detection_probability` 概率生成检测。[S]

**数学正确性。** 检测流程：`P_detect_total = P_coverage × P_d|coverage`。此处隐含假设"覆盖外检测概率为 0"，这是合理的。[M]

**问题记录。** `V12-P1-01`：`rand()` 的随机数状态未被固定。在 MC 实验中，若不同运行次式的随机数序列不同，结果不可复现。应在 `run_simulation_multi` 中设置固定随机种子，而非在此函数内。但函数注释应明确说明依赖全局随机状态。[S]

**问题记录。** `V12-P1-02`：`params.detection_probability` 的当前值为 `1.0`（见 CR-ISSUE-010），意味着**所有在覆盖内的目标 100% 被检测到**。但四个 MC 脚本的表头声称 `Pd=0.6`（见 CR-ISSUE-010）。此处读取的是实际参数 `1.0`，不是表头声称的 `0.6`。[S]

**问题记录。** `V12-P2-03`：第 77 行 `detList = [detList, det]` 是动态数组增长。在 MC 循环中，每帧可能生成数十到上百个点迹，此操作的累积开销为 `O(N^2)`。建议使用预分配：

```matlab
detList = repmat(struct(...), max_expected_dets, 1);
detCount = 0;
```

或在函数末尾裁剪未使用的元素。[S]

### 3.4 量测生成（第 59—67 行）

```matlab
Rg_meas = Rg_true + range_bias + randn() * range_noise;
az_meas = az_true + az_bias + randn() * az_noise;
vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms;
```

**语句职责。** 生成带系统偏差和随机噪声的量测值。[S]

**数学正确性。** 量测模型：`z = h(x) + bias + noise`，其中 noise ~ N(0, σ²)。这是标准雷达量测方程。[M]

**问题记录。** `V12-P2-04`：`range_bias` 和 `az_bias` 是外部传入的参数，但其值来自 `estimate_biases` 的输出。若偏差估计有误（见 CR-ISSUE-011 / V07b-P0-01），量测生成中的偏差校正也是错误的，形成闭环偏差传播。[S]

**问题记录。** `V12-P3-05`：`randn()` 生成的是标准正态分布 N(0,1)，乘以 `range_noise`（标准差）后得到 N(0, σ²)。MATLAB 的 `randn` 使用 Ziggurat 算法（Marsaglia & Tsang 2000），质量可靠。[M]

### 3.5 虚警杂波生成（第 82—103 行）

```matlab
fake_r1 = params.range_min_m + rand() * (params.range_max_m - params.range_min_m);
fake_az = beam_center - half_beam + rand() * params.beam_width_deg;
[clut_lon, clut_lat] = sphere_utils_destination_point(rx_lon, rx_lat, fake_r1, fake_az);
fake_Rg = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, clut_lon, clut_lat);
fake_vr = -200 + rand() * 400;
```

**语句职责。** 在雷达覆盖区内均匀随机生成虚警点迹的位置、群距离和多普勒速度。[S]

**数学正确性。** 位置生成：在 range-azimuth 矩形区域内均匀采样，然后通过 `destination_point` 映射到球面。这不是球面上的均匀分布——在相同 range/az 增量下，高纬度区域的实际面积更小，导致高纬度虚警密度更高。[M]

**问题记录。** `V12-P1-03`：虚警位置的均匀 range-az 采样 ≠ 球面上的均匀空间分布。在 OTH-SWR 场景中，虚警通常建模为在**地面投影面积**上的均匀泊松点过程。正确的做法是：先生成球面上的均匀随机点（如通过 Haversine 逆问题 + 均匀角度），再计算群距离。当前的矩形采样在高纬度产生人为的虚警密度梯度。[M]

**问题记录。** `V12-P2-05`：`fake_vr = -200 + rand() * 400` 生成 [-200, 200] m/s 的均匀分布多普勒速度。对于杂波（地面反射），真实多普勒应接近 0（考虑平台运动和地面静止）。此处均匀分布意味着所有虚警的多普勒都在 [-200, 200] 内均匀随机，这与物理实际不符。通常杂波的多普勒服从以 0 为中心的分布（如高斯或均匀在 [-V_clip, V_clip] 内，V_clip 为杂波截止速度）。[M]

**问题记录。** `V12-P3-06`：第 92—101 行生成的杂波点迹包含大量 `NaN` 字段（`range_meas`, `azimuth_meas`, `range_true` 等），因为杂波没有对应的目标量测。这种"部分填充"的结构体设计增加了下游代码处理杂波的复杂度（需检查 `~isnan` 或 `is_clutter` 字段）。[S]

### 3.6 结构体字段设计（第 69—76 行）

```matlab
det = struct('frameID', frameID, 'time_sec', time_sec, ...
    'prange', Rg_meas, 'paz', az_meas, 'pvr', vd_meas, ...
    'range_meas', Rg_meas, 'azimuth_meas', az_meas, 'radial_vel_meas', vd_meas, ...
    ...
    'is_clutter', false, ...
    'aircraft_id', int32(ac_id));
```

**语句职责。** 构建点迹结构体，包含原始量测（`prange/paz/pvr`）和处理后量测（`range_meas/azimuth_meas/radial_vel_meas`）。[S]

**问题记录。** `V12-P3-07`：`prange` 和 `range_meas` 当前值相同（都是 `Rg_meas`），字段冗余。若未来某处对 `prange` 做原始量测、对 `range_meas` 做偏差校正后的量测，此设计才有意义。当前应统一为一个字段或添加注释说明区别。[S]

**问题记录。** `V12-P3-08`：`aircraft_id` 使用 `int32` 类型。在 MATLAB 中，`int32` 与 `double` 混合运算时需要类型转换。若下游代码用 `== 0` 判断杂波，`int32(0) == 0` 返回 logical true，行为正确。[M]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V12-P1-01 | 第 56 行 | P1 | rand() 依赖全局随机状态 | S | OPEN |
| V12-P1-02 | 第 56 行 | P1 | detection_probability=1.0 与 Pd=0.6 表头矛盾 | S | OPEN |
| V12-P1-03 | 第 82—103 行 | P1 | 虚警位置非球面均匀分布 | M | OPEN |
| V12-P2-01 | 第 35—36 行 | P2 | radar1_ 字段名在多雷达场景下歧义 | S | OPEN |
| V12-P2-02 | 第 43 行 | P2 | n_resolution_cells 未定义计算方式 | S | OPEN |
| V12-P2-03 | 第 77 行 | P2 | detList 动态数组增长 O(N²) | S | OPEN |
| V12-P2-04 | 第 65—67 行 | P2 | 偏差估计错误闭环传播 | S | OPEN |
| V12-P2-05 | 第 90 行 | P2 | 杂波多普勒均匀分布不物理 | M | OPEN |
| V12-P3-01 | 第 43 行 | P3 | 泊松参数 λ 的物理含义未注释 | S | OPEN |
| V12-P3-02 | 第 69—76 行 | P3 | prange 与 range_meas 冗余 | S | OPEN |
| V12-P3-03 | 第 92—101 行 | P3 | 杂波结构体大量 NaN 增加下游复杂度 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：目标检测流程（覆盖判定 → 概率检测 → 噪声叠加）在数学上正确。但虚警位置采样的球面均匀性假设错误，导致高纬度虚警密度人为偏高。[M]
- **代码质量**：结构体设计清晰，但 `prange`/`range_meas` 冗余、杂波大量 NaN 字段增加了维护成本。动态数组增长在每帧点迹数少时可忽略。[S]
- **性能**：`detList = [detList, det]` 的 O(N²) 增长在点迹数多时（MC 场景下每帧可能 50-100 个点迹）可感知。[S]
- **测试充分性**：与 `generate_frame_detections.m` 的逻辑高度相似，但未发现针对多目标版本的独立单元测试。[S]
- **剩余未验证项**：虚警密度梯度对多目标关联的影响；`detection_probability=1.0` 与 `Pd=0.6` 表头矛盾的已有结果的有效性。

## 6. 下一审查游标

- 文件：`ukf/ukf_imm.m` 的 `create_imm` 和 `init_imm` 子函数
- 重点：IMM 模板创建、Markov 转移矩阵、IPDA 似然度
- 稳定指纹：`create_imm` 第 58—117 行
