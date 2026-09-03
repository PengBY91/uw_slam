# 稀疏控制点评测定义（PREP-B-06）

> 状态：v1 2026-09-03 · 第 3 周交付"定义"部分（字段 + 指标 + 骨架实现 + 单测）；
> 验收（水池关卡上稀疏控制点 RMSE 与全轨迹 ATE 相关系数 > 0.8）依赖第 4 周的
> PREP-A-08 水池关卡，尚未验证。
> 出处：`docs/ROV平台到货前准备工作规格-2026-09-02.md` PREP-B-06。

## 0. 要解决的问题

`evaluation` 层现有的 `ComputeAte`（`include/evaluation/trajectory_metrics.hpp`）
要一条稠密的真值轨迹。仿真里有（HoloOcean `PoseSensor`），**真实水池里没有**：
没人会在水池里架一套水下动捕。真实水池里能拿到的是另一种真值——池底、池壁上
若干个位置已知的标记（卷尺或全站仪量出来的）。

一次对标记的观测 + 一个估计位姿 = 一个对该标记世界位置的预测；预测和实测位置
之间的距离，就是估计器要为之负责的误差，全程不需要任何真值轨迹。

```
predicted_W = T_WB(观测时刻的估计位姿) * point_B
error       = || predicted_W − surveyed_W ||
```

## 1. 数据模型

### scenario YAML 新字段 `control_points`

在 `configs/scenario/*.yaml` 里（解析见 `src/runtime/config.cpp`，结构体
`uw::runtime::ScenarioControlPoint`）。模板见 `configs/scenario/pool_example.yaml`。

| 键 | 必填 | 含义 |
|---|---|---|
| `tag` | 是 | 唯一标识，标注观测时引用；重复 tag 在加载期直接拒绝（否则关联歧义、逐点表不可读） |
| `position_m` | 是 | 标记**中心**在世界系的位置，3 个分量，**Z-up**（3 m 深水池池底的标记 z = −3.0） |
| `size_m` | 否（默认 0.3） | 最大尺寸；随误差一起报告——5 cm 误差对 30 cm 反射板和对 5 cm 小球是两回事 |
| `reflectivity_class` | 否（默认 `strong`） | `strong`/`moderate`/`weak`，**沿用** `adapters/holoocean/scenarios/*.yaml` 的 `acoustic_reflectivity_class` 词表，同一份标记表可以同时驱动 UE5 关卡（PREP-A-06/A-08）和这个指标 |

坐标符号提醒：仓库 world/body 是 Z-up，`position_m` 的 z 是
`PressureDepthMeasurement.depth_m`（正向下）的**相反数**，两者不能混用
（CLAUDE.md 记过同一个坑）。

### 指标接口

`include/evaluation/control_point_metrics.hpp` + `src/evaluation/control_point_metrics.cpp`：

- `ControlPoint{tag, position_W, size_m}`——实测真值侧
- `ControlPointObservation{tag, timestamp_s, position_B}`——一次观测，标记在
  **base_link** 下的位置。在传感器系里做检测的调用方要先用 rig 外参转过来，
  这样指标本身不必知道是哪个传感器看到的
- `ControlPointResult`——`rmse_m`/`mean_m`/`p95_m`/`max_m`、逐观测表
  `per_observation`，以及三个计数：`num_unmatched_observations`（时间上找不到
  估计位姿）、`num_unknown_tags`（观测引用了 scenario 里没声明的标记）、
  `num_covered_control_points`（实际被覆盖的标记数）

## 2. 五条刻意的设计选择

1. **这是定位误差，不是建图误差**。观测本身的量程/方位噪声会进这个数，所以
   单点误差不可能小于传感器对该标记的精度。报告时必须同时给出传感器 sigma——
   量程分辨率 5 cm 的声呐上读到 5 cm RMSE，不等于"估计器好到 5 cm"。
2. **默认不做对齐**。`ComputeAte` 有一个可选的 Umeyama 对齐（真实录制里估计
   系和真值系不同源）；这里默认关掉，因为控制点是在 scenario 自己的世界系里
   实测的，如果估计的系不是那个系，正确做法是把锚定做对，不是把差异拟合掉。
   `align_before_scoring` 只为"估计器把 kf0 钉在原点"这一种水池情形保留，
   并且是 opt-in。
3. **对齐要求 3 个不同标记，不是 3 次观测**。同一个标记看三次定不住刚体变换。
   实现里按 distinct tag 计数，不够就静默返回未对齐结果（和 `ComputeAte`
   一样，宁可给出可解释的数也不让整次评测失败）。
4. **同一标记的多次观测各算一次误差**。一个会漂的估计器应该按"漂着看了几次"
   受罚，而不是按"看了几个标记"受罚。
5. **关联（哪个检测对应哪个标记）不在指标里**。v1 靠人在回放里标注，之后才是
   CFAR 检测 + 最近邻（规格 PREP-B-06 第 2 步）。指标只给已关联的集合打分，
   这样关联启发式的错误不会被悄悄吸收进一个看起来合理的 RMSE 里。

p95 用最近排名法（`ceil(0.95·n)` 的顺序统计量），和实时 gate、`run_report`
的延迟分位数同一约定——这个指标的样本量本来就是"几个标记 × 几次观测"，
在顺序统计量之间插值会暗示一个样本量支撑不起的精度。

## 3. 单测覆盖（`tests/evaluation/control_point_metrics_test.cpp`，10 条）

完美估计打 0 分；纯平移误差恰好等于位姿误差；姿态误差按距离放大（10 m 外
1° yaw ≈ 17 cm，直接对着 `2R·sin(θ/2)` 断言）；重复观测各自计数；时间上匹配
不到位姿报 unmatched 而不是拿旧位姿凑；未知 tag 单独计数；p95 最近排名；
默认不把整体平移拟合掉而 opt-in 对齐能；3 次同标记观测拒绝对齐；空输入不崩。

`tests/runtime/config_test.cpp` 另加 3 条覆盖 YAML 侧：正常加载与默认值、
没有该字段的旧 scenario 仍然加载、四类畸形（缺 tag／position 分量不足／重复
tag／size ≤ 0）全部抛错。

## 4. 还没做的（第 4 周起）

- **接进 `replay_demo`**：读 scenario 的 `control_points`、读一份标注好的观测
  文件、把 `ControlPointResult` 写进 run manifest 并作为可选 gate。当前指标
  是库函数，还没有调用方。
- **观测从哪来**：v1 的人工标注格式还没定；CFAR 检测 + 最近邻关联要等
  PREP-B-03 的声呐配准工作铺开。
- **验收本身**：规格要求"水池关卡上稀疏控制点 RMSE 与全轨迹 ATE 相关系数
  > 0.8"，需要 PREP-A-08 的水池关卡（有 GT 可对照）才能算。这条相关性是这个
  指标能不能替代 ATE 的唯一凭据，在跑出来之前不要把它当成已验证的替代品。
