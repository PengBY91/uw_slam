# ROV 到货前准备第四周总控计划

> 状态：v2 · 2026-09-03 · 总控索引，不直接作为代码执行清单
> 设计依据：`docs/archive/superpowers/specs/2026-09-03-week4-pre-delivery-prep-design.md`
> 当前执行计划：`docs/archive/superpowers/plans/2026-09-03-imu-preintegration-closure.md`

## 本周交付边界

第四周按工作流独立推进，不再把算法、飞控、设备伪装、UE5 关卡和飞手训练塞进同一实现批次。

**B-01 阻断已解除（2026-09-03，本地验证）**：`docs/archive/superpowers/plans/2026-09-03-imu-preintegration-closure.md` 的 7 个任务在 `.worktrees/week4-b01-clean` 上全部执行完毕。四次算法轨迹（原始 / 删除 GT / GT 位姿偏移 / GT 时间偏移）逐字节相同，三类因子均非零（imu 11 / sonar_range 36 / depth 12）、`relative_pose_factor_count=0`、`initialization=stationary`、13 迭代收敛、**ATE rmse = 0.0858 m ≤ 0.15 m**；全仓 708 条 CTest 全绿，独立代码审查的 9 条发现已全部处理。B-02 回放接线与 C-03 外部导航回灌的**本地**前置条件已满足；C-03 W5 另有"W4 adapter 已外部验证"这一条独立前置，不因本次解除。HoloOcean 200 Hz / 30 s 逐秒漂移报告仍为外部待验收，不阻断 B-02/C-03 的编码工作。

| 工作流 | 第四周目标 | 当前状态 | 本周完成门槛 |
|---|---|---|---|
| B-01 IMU 预积分 | 合法关键帧合同、初始化先验、三类因子闭环、无真值泄漏 | **本地已验证**（2026-09-03）；HoloOcean 200 Hz/30 s 外部待验收 | 已达成：无 GT/VO 控制路径，IMU 11 / depth 12 / sonar 36 因子，ATE 0.0858 m |
| B-02 航向 | 定义带 provenance 的磁航向消息；实现残差/builder | 等待独立计划 | 契约及单测可本地验证；回放集成等待 B-01 |
| C-03 W4 | MAVLink 遥测进、三类命令出 | 等待独立计划 | fake transport 单测 + 真实 SITL 回读；不含 extnav |
| C-03 W5 | 外部导航、参数、质量门、POSHOLD/fail-closed | B-01 侧前置已满足；仍等 W4 adapter 外部验证 | B-01 通过且 W4 adapter 已外部验证 |
| A-13 W4 | MAVLink SITL 路径、IMU emitter fixture | 等待 C-03 W4 计划 | fixture 可本地验证；完整 IMU 路径依赖 D-03 |
| A-06 | 养殖区关卡续作 | 外部环境工作流 | UE5 关卡、声学可辨性、scripted pilot 评分分别留证 |
| A-09 | `ir_night` + H.264 4/6/8 Mbps 离线评估 | 等待独立计划 | 三项指标表；只输出候选码率档位 |
| A-08 | 真实测试水池关卡 | 门禁中 | 先取得真实尺寸；模板不算关卡完成 |
| A-12 | 飞手训练 harness | 等待 A-06/C-02 | 本地 harness 与真实两人×三次外部验收分开记状态 |

## 执行顺序

1. ~~完成并评审 B-01 收口计划。~~ 已完成。
2. ~~在干净的第四周实现基线上按 TDD 执行 B-01；保留当前实验补丁作为诊断证据，不直接合入。~~ 已完成，见 `.worktrees/week4-b01-clean`；`.worktrees/week4-prep` 的实验补丁未合入，仅作诊断反例。
3. **当前项**：分别编写并评审 B-02 契约/因子计划与 C-03 W4 遥测/命令计划。
4. C-03 W4 通过 SITL 后，执行 A-13 MAVLink 路径；D-03 可用后再把 IMU fixture 升级为端到端伪装流。
5. A-06 与 A-09 可在不触碰 B-01 代码的独立工作树并行；A-08 等水池尺寸，A-12 等 A-06 场景。
6. C-03 W5 外部导航最后接入，并强制经过定位质量门。

## 计划文件规则

每个独立计划都必须满足：

- 一次只解决一个可独立验收的工作流。
- 列出仓库中实际存在的构建目录和 Python 环境；缺失环境时把创建步骤写入计划。
- 每项先写失败测试，再写最小实现，并给出精确测试选择器。
- 本地验证与 SITL/Windows/HoloOcean/真人外部验证分栏记录。
- 未通过上游门禁时，只允许完成契约和 isolated unit tests，不允许接入下游运行路径。

## 停止条件

出现以下任一情况时停止向下游推进并回到设计/诊断：

- 算法输出依赖 `/gt/state` 或旧 VO/relative-pose hint。
- 验收所列任一因子计数为零。
- 静止初始化失败后仍无条件固定零速度或零偏置。
- 航向测量没有 provenance，或飞控同时使用 ExternalNav yaw 形成反馈环。
- extnav 输出没有质量门，或断流后继续重发旧增量。
- 用 fake/dry-run/template 结果把任务标为“已外部验证”。

## 统一收尾检查

每个工作流完成时执行其聚焦计划中的测试，再执行：

```bash
python3 tools/lint/check_layer_dependencies.py .
python3 tools/lint/check_realtime_traceability.py docs/traceability/rov-realtime-closed-loop.csv .
git diff --check
```

只有命令输出和所需外部运行证据都存在时，主规格中的状态才能提升到相应层级。
