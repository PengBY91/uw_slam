# uw_slam 术语体系整理设计

## 1. 目标

统一当前权威文档中的架构术语，使表达更符合机器人、SLAM 和多传感器融合团队的
常用语言，同时准确区分消息格式、算法接口和运行组件。

本次只调整文档表达和必要的解释性注释，不修改目录、namespace、类名、配置键、
Protobuf 字段、测试标签或程序行为。

## 2. 核心术语体系

采用两层表达：

1. **核心消息与接口**：描述模块间共享的消息类型、物理语义、算法接口和一致性规则。
2. **跨语言规范化消息模型**：专指 `schemas/proto/` 定义的 Protobuf 消息及其
   C++/Python 绑定。

`schemas/proto/` 是跨语言消息定义的唯一来源；`include/domain/` 提供生成消息的
C++ 辅助函数和语义校验；`include/measurement_api/` 定义算法模块之间的抽象接口。
三者不再笼统地称为同一个“领域契约层”。

## 3. 直接调整的表达

| 现有表达 | 统一后的表达 | 使用范围 |
|---|---|---|
| 领域契约 | 核心消息与接口 | 同时讨论消息、语义和算法边界时 |
| Protobuf 领域契约 | 跨语言规范化消息模型 | 专指 `schemas/proto/` 时 |
| 领域类型 | 核心消息类型 | 生成消息及其 C++ 辅助层 |
| 契约测试 | 消息格式与接口一致性测试 | 用户可见文档；保留 `contract.*` 测试标签 |
| canonical MCAP | 统一 MCAP 录制格式 | 中文文档中的录制与回放格式 |
| 可运行垂直切片 | 可运行端到端链路 | 描述当前工程成熟度时 |
| evidence handoff / mapping handoff | 局部地图数据交接 | 描述融合结果进入建图模块时 |
| AI information cap | 学习模型信息量上限 | 描述学习模型置信度校准与上限约束时 |

对于代码类型，正文第一次出现时保留原名并解释其含义：

- `MeasurementEvidence`：带来源、有效域和不确定度描述的量测结果；
- `MapEvidence`：保存在局部坐标系、可随关键帧位姿更新而重新变换的局部地图数据。

后续中文叙述优先使用“量测结果”和“局部地图数据”，避免在不需要强调类型时反复
使用中英混排的 `evidence`。

## 4. 保留但必须澄清的历史名称

以下标识符涉及兼容性或较大代码迁移，本次不重命名：

| 标识符 | 文档必须说明的真实含义 |
|---|---|
| `estimator_mode` | 当前选择相对位姿输入来源，并不切换 `GaussNewtonSolver` |
| `black_box_vio` | 外部或预生成里程计输入模式；在合成链路中具体是 GT+noise 相对位姿桩 |
| `proposed_noise` | 当前 FactorBuilder 实际将其作为单标量 sqrt-information 使用，数值越大权重越强 |
| `SubmapManager` | 当前是按关键帧索引的局部地图数据存储，不具备完整子地图创建、切换和生命周期管理 |
| `runtime` | 当前提供配置、MCAP、Manifest、同步、状态机和队列原语，尚未组成在线调度运行时 |
| `AcousticOpticDepthFusionFrontend` | 实际职责是声光深度融合模块；类名和所在 target 暂时保留 |
| `map_backend` | 预留的地图实现选择字段；当前只有一个受支持实现 |

这些说明用于防止读者从名字推断出当前代码并不存在的功能。代码级迁移应独立设计，
尤其是 `proposed_noise`，因为它属于 Protobuf wire schema，不能通过普通文本替换完成。

## 5. 保留的行业术语

保留 `frontend`、`factor`、`residual`、`keyframe`、`submap`、pose graph、VO、ATE 等
机器人与 SLAM 社区常用术语。第一次出现时可补充中文含义，但不强制逐处翻译，也不
把代码标识符翻译成与实现无法对应的新名称。

`backend` 仅在明确指状态估计或地图实现时使用；泛指模块后半段时改用具体组件名，
避免把估计求解器和地图实现混为一谈。

## 6. 修改范围

本次更新以下当前权威或用户入口文档：

- `README.md`、`CLAUDE.md`；
- `docs/README.md`；
- 长期架构、代码库参考、新人指南、测试指南和生产就绪度路线图；
- `configs/README.md` 和当前实验 YAML 中与上述历史字段有关的解释性注释；
- 必要的 Protobuf/C++ 字段注释，仅用于说明现有语义，不改变生成接口。

不批量修改 `docs/superpowers/plans/` 与既有 `docs/superpowers/specs/`。这些文件记录
历史设计和实施过程，重写会削弱其可追溯性。Python/C++ 文件名、类名、函数名和测试名
同样不在本次范围内。

## 7. 一致性规则

1. 同一段落必须区分“消息 Schema”和“算法接口”，不得重新合称为领域契约。
2. 中文正文使用“统一 MCAP 录制格式”；代码标识符如 `canonical_writer.py` 原样保留。
3. 引用历史标识符时使用反引号，并在首次出现处说明其当前真实行为。
4. 修改 Markdown 标题时同步更新仓库内指向该标题的锚点链接。
5. 不把目标架构描述成当前实现；涉及 runtime、submap 或多后端切换时明确成熟度边界。

## 8. 验证方式

完成调整后执行：

1. 搜索当前权威文档中的“领域契约”“垂直切片”“canonical MCAP”、中英混排的
   `evidence handoff` 等旧表达；只允许在解释旧术语或历史文件中出现。
2. 搜索 `estimator_mode`、`proposed_noise`、`SubmapManager`、`runtime` 和
   `AcousticOpticDepthFusionFrontend`，确认关键入口文档均没有夸大其职责。
3. 检查被修改标题的仓库内 Markdown 锚点引用。
4. 运行层依赖 lint，确认解释性注释修改没有意外触碰生产结构。
5. 审阅 `git diff --check` 和限定范围的 `git diff`，确认没有覆盖用户现有改动。

本次是术语与文档一致性调整，不改变可执行行为，因此不要求重新运行完整 C++/Python
测试套件。
