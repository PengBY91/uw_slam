# uw_slam 文档中心

这里是 `uw_slam` 技术文档的入口。根目录 [README](../README.md) 负责回答“项目是什么、
如何跑起来”；本页负责回答“遇到具体任务时应该读哪份文档，以及不同文档冲突时以谁
为准”。

## 从这里开始

- **第一次接触项目**：先读根目录 [README](../README.md)，运行合成数据 Demo，再回到
  本页选择专题资料。
- **准备作为新贡献者动手改代码**：读[新人上手指南](./uw-slam-newcomer-guide.md)，
  它按调用链把 `synth_bag_gen → replay_demo` 整条流程串起来，并给出"改某类代码该看
  哪个文件、对应哪类测试"的速查表。
- **准备修改或调试代码**：读[代码库参考](./uw-slam-codebase-reference-2026-08-18.md)，
  它只记录当前实现中可以从代码确认的事实。
- **准备改变模块边界或长期技术路线**：读
  [长期架构设计](./acoustic-optic-slam-platform-architecture-2026-08-17.md)。
- **追溯第一阶段方案为什么演变成当前结构**：读
  [HoloOcean Pipeline 工程方案](./holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md)。
- **验证某个功能是否跑得通、该加载什么环境**：读
  [测试与验证指南](./testing-and-verification-guide-2026-08-20.md)。
- **评估当前离生产/测试平台还有多远、安排阶段投入**：读
  [生产就绪度审计与阶段路线图](./uw-slam-production-readiness-and-roadmap-2026-08-21.md)。

## 按任务选择文档

| 任务 | 首选文档 | 补充材料 |
|---|---|---|
| 新贡献者第一次读代码、搞清楚整条调用链 | [新人上手指南](./uw-slam-newcomer-guide.md) | [代码库参考](./uw-slam-codebase-reference-2026-08-18.md) |
| 查找类型、接口、算法或 CMake target | [代码库参考](./uw-slam-codebase-reference-2026-08-18.md) | [根 README](../README.md) |
| 理解领域契约、依赖 DAG、状态机与 Gate | [长期架构设计](./acoustic-optic-slam-platform-architecture-2026-08-17.md) | [代码库参考](./uw-slam-codebase-reference-2026-08-18.md) |
| 配置或复现实验 | [配置说明](../configs/README.md) | [根 README](../README.md#运行端到端-demo) |
| 验证某项功能、判断该加载哪个运行环境 | [测试与验证指南](./testing-and-verification-guide-2026-08-20.md) | `tools/verify_pipeline.sh` |
| 评估生产就绪度、制定团队里程碑和投入计划 | [生产就绪度审计与阶段路线图](./uw-slam-production-readiness-and-roadmap-2026-08-21.md) | [长期架构设计](./acoustic-optic-slam-platform-architecture-2026-08-17.md) |
| 修改 HoloOcean Python 网关 | [HoloOcean 适配器](../adapters/holoocean/README.md) | [Pipeline 工程方案](./holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md) |
| 构建或排查 ROS2 接入 | [ROS2 适配器](../adapters/ros2/README.md) | [外部仓库恢复说明](../external_repos/README.md) |
| 理解第三方仓库角色与风险 | [外部代码概览](../external_repos/external-repos-overview.md) | [外部仓库恢复说明](../external_repos/README.md) |
| 移植第三方实现或核对许可证 | [NOTICE](../NOTICE) | [外部代码概览](../external_repos/external-repos-overview.md) |
| 查看团队开发约定与已知工程陷阱 | [CLAUDE.md](../CLAUDE.md) | [根 README](../README.md#参与开发) |

## 文档状态与权威范围

| 文档 | 状态 | 权威范围 | 最后核对 |
|---|---|---|---|
| [根 README](../README.md) | 当前入口 | 项目定位、快速开始、已验证能力与限制 | `f285e0d`，2026-08-21 |
| [新人上手指南](./uw-slam-newcomer-guide.md) | 当前说明 | 调用链、目录职责速查、常见误解边界 | 2026-08-21 |
| [代码库参考](./uw-slam-codebase-reference-2026-08-18.md) | 当前事实 | 当前类型、函数、参数、数据流、测试与工具 | `f285e0d`，2026-08-21 |
| [长期架构设计](./acoustic-optic-slam-platform-architecture-2026-08-17.md) | 已批准设计 | 长期目标、模块边界、不变量与阶段决策 | 2026-08-19 |
| [Pipeline 工程方案](./holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md) | 演进中/历史参考 | 第一阶段 baseline、方案演进与代码审计修订 | `919e1f0`，2026-08-19 |
| [测试与验证指南](./testing-and-verification-guide-2026-08-20.md) | 当前说明 | 分功能验证命令、判定标准与运行环境要求 | `f285e0d`，2026-08-21 |
| [生产就绪度审计与阶段路线图](./uw-slam-production-readiness-and-roadmap-2026-08-21.md) | 当前审计/计划 | 当前差距、成熟度、阶段优先级、验收门和投入估算 | 当前工作区，2026-08-21 |
| [配置说明](../configs/README.md) | 组件当前说明 | 四层配置字段、覆盖顺序和消费范围 | 随配置代码维护 |
| [HoloOcean 适配器](../adapters/holoocean/README.md) | 组件当前说明 | Python 网关安装、代码生成和验证边界 | 随适配器维护 |
| [ROS2 适配器](../adapters/ros2/README.md) | 组件当前说明 | ROS2 构建、目标状态和未接通边界 | 随适配器维护 |
| [外部代码概览](../external_repos/external-repos-overview.md) | 当前参考 | 上游角色、接口、审计结论和移植风险 | `919e1f0`，2026-08-19 |
| [NOTICE](../NOTICE) | 来源权威记录 | 移植文件、上游许可证、保留和排除范围 | 每次移植时更新 |

“当前”表示对现有仓库事实的描述；“已批准设计”表示应该演进到的目标；“演进中/历史
参考”表示用于理解决策过程，不应单独作为当前实现依据。

## 信息冲突时以谁为准

同一问题出现不同表述时，按信息类型判断：

1. **实际行为与字段**：源代码、测试和 `schemas/proto/` 优先。
2. **当前使用方法**：代码库参考、配置文档和适配器文档优先。
3. **项目入口和验证命令**：根 README 与 `tools/verify_pipeline.sh` 优先。
4. **未来模块边界与技术决策**：已批准的长期架构设计优先。
5. **早期串联方案和演进原因**：Pipeline 工程方案仅作历史与工程背景参考。

发现文档与代码不一致时，不要静默选择其中一个：先用测试或源码确认事实，再更新对应
的当前文档；若影响目标决策，同时更新架构文档或记录新的决策说明。

## 文档维护约定

- 仓库内链接使用相对 Markdown 链接，保证 GitHub 可以直接解析。
- 描述代码事实的长文档记录核对日期和 Git commit；只修改措辞时也要确认事实没有失效。
- 用“当前实现”“目标设计”“计划工作”“历史背景”等明确标签，不把计划写成已完成能力。
- 构建、测试和 Demo 数字以实际命令输出为准；更新数字时保留可重复执行的命令。
- 专业细节放在职责对应的文档，文档中心只负责路由，根 README 只负责上手。
- `external_repos/` 下的第三方子仓库保持只读；本仓库的补充说明只写在其顶层两份文档。
