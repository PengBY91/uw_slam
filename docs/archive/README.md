# 归档：历史过程记录

这里放**已经执行完毕、或已被更上位文档取代的过程文档**：实施计划、一次性代码审查
记录、早期工程方案和阶段性审计快照。

**这些文档不描述当前实现，也不构成任何验收依据。** 判断“当前是什么样”请回到
[文档中心](../README.md)列出的规范、设计与代码事实类文档，以及源码本身。

保留它们的唯一理由是**可追溯性**：仓库里有约 70 处源码注释以
`docs/archive/superpowers/...`、`docs/archive/rov-realtime-closed-loop-code-review-...`
这类路径引用它们，用来说明某段代码“当初为什么这么写”“对应哪条审查发现”。删掉文档会
让这些注释变成悬空引用。

| 文档 | 原本是什么 | 被谁接替 |
|---|---|---|
| [`superpowers/plans/`](./superpowers/plans/) | 逐任务的实施计划（实时闭环四件套、前端正确性闭环、IMU 预积分闭环、第四周总控等） | 计划已执行完毕；当前范围以 [`docs/specifications/`](../specifications/) 与 [ROV 平台到货前准备工作规格](../ROV平台到货前准备工作规格-2026-09-02.md) 为准 |
| [`superpowers/specs/`](./superpowers/specs/) | 实施前的设计确认文档（求解器/空间索引开源采纳、前端正确性、路线图修订、规格体系设计） | 结论已落进[长期架构设计](../acoustic-optic-slam-platform-architecture-2026-08-17.md)、[ROV 平台落地路线图](../ROV平台落地路线图.md)和代码 |
| [rov-realtime-closed-loop-code-review-2026-08-27.md](./rov-realtime-closed-loop-code-review-2026-08-27.md) | 主线二的一次性代码审查记录（findings A1–D3） | 每条 finding 都已修复并带回归测试；测试文件里以 `finding X#` 反向索引本文 |
| [uw-slam-production-readiness-and-roadmap-2026-08-21.md](./uw-slam-production-readiness-and-roadmap-2026-08-21.md) | 2026-08-22 时点的生产就绪度审计快照与阶段路线图 | 路线部分由 [ROV 平台落地路线图](../ROV平台落地路线图.md) 接替；审计数字是当时的快照，不要当作现状 |
| [holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md](./holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md) | 第一阶段 HoloOcean → 声光 SLAM demo 的工程方案与既有代码审计 | 结构已演进为当前仓库；模块边界以[长期架构设计](../acoustic-optic-slam-platform-architecture-2026-08-17.md)为准 |

引用本目录内容时请显式说明“历史记录”，不要把其中的数字、任务状态或接口描述当成当前
事实——尤其是 ATE/延迟等实测数字，多数产生于早期噪声实现或早期代码版本。
