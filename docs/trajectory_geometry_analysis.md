# 真值航迹几何分析 — 转弯场景全部"货不对板"

**日期：** 2026-07-25  
**问题发现者：** 用户观察

---

## 一、核心问题

**所有 `*_rate_*` 场景的转弯角度仅有 ~47°，远小于预期的"大角度转弯"。**

### 原因：固定的三个航路点

`aircraft_trajectory_create.m` L458-460 硬编码了三个航路点：

```matlab
W1 = [126.6685, 32.2184];  % 起点 (四川东北部)
W2 = [128.2501, 31.0887];  % 拐弯顶点 (四川盆地)
W3 = [132.0502, 31.4379];  % 终点 (重庆北部)
```

这三个点构成的入向-出向夹角只有 **~47°**。

### 计算验证

从输出日志看：
```
right_rate_0p7: 转弯角度 47°, 转弯半径 18.8km, 转弯持续 66.7s
right_rate_1p3: 转弯角度 47°, 转弯半径 10.1km, 转弯持续 35.9s
uturn:          转弯角度 180°, 转弯半径 13.2km, 转弯持续 180s
```

**rate 场景和 uturn 场景的几何完全不同**：
- `gradual_turn`（rate 系列）：固定 W1→W2→W3 三点，转弯角度 47°
- `uturn`：正东飞 → 左转半圆 → 正西飞，转弯 180°

### reverse_trajectory 做了什么

`build_truth_scenario.m` L84 对 `right_rate_*` 调用了 `reverse_trajectory_local`：
```matlab
trajs = {reverse_trajectory_local( ...
    aircraft_trajectory_create('gradual_turn', p_local))};
```

reverse 只是把 W1↔W3 互换 + 航段逆序飞行，但**转弯角度仍然是 W1-W2-W3 形成的 47°**。

### 报告中的错误描述

Phase 4 报告声称：
> | `right_rate_0p7` | 0.7°/s | **~257s** | 右缓转（航向变化180°需257s）|
> | `right_rate_1p3` | 1.3°/s | **~138s** | 右急转（航向变化180°需138s）|

这是**数学计算错误**：
- 报告用 `180°/0.7°/s = 257s` 来计算持续时间
- 但实际转弯只有 **47°**，所以真实持续时间是 `47/0.7 = 67s`
- 报告把"航向变化180°需要的时间"当成了"这个场景的实际转弯时长"

---

## 二、结论

**所有 `*_rate_*` 场景的实际转弯只有 ~47°，不是大角度转弯。**

| 指标 | 报告声称 | 实际值 |
|------|---------|-------|
| 转弯角度 | 未说明 | **~47°** |
| 转弯持续(0.7°/s) | ~257s | **67s** |
| 转弯持续(1.3°/s) | ~138s | **36s** |
| 转弯弧长(0.7°/s) | R=18.8km, θ=180° | **R=18.8km, θ=47°, 弧长≈15km** |

而唯一真正的大角度转弯场景 `uturn` 从未被纳入 rate series MC 实验。

---

## 三、修复建议

如果要产生真正的"大角度转弯"场景，需要在 `build_truth_scenario.m` 中：

1. **改用 uturn 类型轨迹**（已经存在 180° 半圆）
2. 或者修改 `create_gradual_turn_trajectory` 支持更大转弯角的航路点配置
3. 或者新增 `large_turn_left/right` 场景，使用不同 W1/W2/W3

**最简单的修法**：
```matlab
case {'right_rate_0p7'}
    p_local = params;
    p_local.truth_turn_rate_deg_per_sec = 0.7;
    trajs = {reverse_trajectory_local( ...
        aircraft_trajectory_create('uturn', p_local))};
```

这样 right_rate_0p7 就变成了 180° 右转半圆，持续 180/0.7 = 257s。

---

*生成时间：2026-07-25*
*分析内容：航迹生成代码静态审查 + MATLAB 运行时数据验证*
