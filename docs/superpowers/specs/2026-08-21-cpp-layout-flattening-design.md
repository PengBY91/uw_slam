---
title: C++ 目录扁平化与集中式 CMake 设计
created: 2026-08-21
updated: 2026-08-21
type: engineering-design
status: approved-design
implementation_status: not-started
codebase_reference: 26c8b26
---

# C++ 目录扁平化与集中式 CMake 设计

## 1. 文档定位

本文定义 `uw_slam` 的 C++ 源码、公共头文件、测试和 CMake 布局重构。目标是消除当前
“每个细粒度算法都有自己的 `include/src/test/CMakeLists.txt`”所造成的过深目录和重复
构建样板，同时保留真实的架构依赖、增量编译和可选组件隔离。

本次重构只改变源码组织和构建声明，不改变算法行为、C++ namespace、protobuf schema、
配置格式、CLI 参数或 MCAP 数据格式。长期模块职责仍以
[声光 SLAM 平台架构](../../acoustic-optic-slam-platform-architecture-2026-08-17.md)为准。

## 2. 当前问题

当前 C++ 代码把几乎每个小型实现当作独立 package：

```text
algorithms/frontends/sonar_cfar_frontend/
├── CMakeLists.txt
├── include/uw/frontends/
├── src/
└── test/
```

相同模式重复出现在 frontend、factor builder、mapping 和 adapter 中，导致：

- 导航一个实现时需要穿过多层只含一个子目录的路径；
- 每个小模块重复 target、warning、include path 和测试注册样板；
- package 数量被误当作模块化程度，实际依赖关系反而分散在多个 CMake 文件里；
- 新增一个小算法通常要创建四套目录和一个 CMake 文件；
- 顶层只能看到 `add_subdirectory()` 顺序，无法快速审计完整 target 图。

现有独立 target 的大部分算法实现使用相同基础依赖、静态链接，并由应用直接构造具体
类型。仓库没有动态插件 ABI、运行时注册表或独立插件发布机制，因此每个实现独占 target
和源码根的隔离收益有限。

## 3. 目标与非目标

### 3.1 目标

1. 主 C++ 代码集中到共享 `include/` 和 `src/`，手写公共头文件路径移除多余的 `uw/`
   目录层。
2. target 按真实架构边界合并，而不是按每个具体实现拆分。
3. CMake 使用单一顶层入口和少量集中式职责文件，不在源码子目录放 `CMakeLists.txt`。
4. 保留每个 translation unit 的增量编译，并控制公共依赖传播。
5. 测试集中到顶层 `tests/`，按生产模块组织并保留单个 GTest case 的发现能力。
6. 对共享 include root 无法由 CMake 直接约束的依赖方向增加静态检查。
7. 仅对可选依赖、不同语言、独立部署或外部执行边界保留独立源码根。
8. 迁移当前工作区中的有效修改，包括新增的 stereo landmark VO frontend，不覆盖或丢弃
   未提交工作。

### 3.2 非目标

- 不改变现有 `uw::...` C++ namespace；
- 不修改 protobuf schema、wire contract 或生成语言范围；
- 不修改算法公式、参数默认值、运行时数据流或数值输出；
- 不建立动态插件系统、稳定插件 ABI 或运行时模块发现；
- 不把 CUDA、TensorRT、学习模型 runtime 等尚未引入的能力提前抽成空模块；
- 不重构 `adapters/holoocean` Python package、`configs/`、`schemas/`、`docs/` 或 `tools/`
  的内部结构；必要的路径引用更新除外；
- 不保留旧 target 名称和 `uw/...` include 路径的兼容 shim。

## 4. 方案选择

### 4.1 未采用：极致扁平

把所有头文件、源码和测试分别直接放到 `include/`、`src/`、`tests/` 根目录虽然路径最短，
但会丢失文件归属、提高命名冲突概率，并在代码增长后重新形成无序目录。

### 4.2 采用：共享源码根 + 架构层 target

主 C++ 代码共用 `include/` 和 `src/`，其下一层按 namespace/职责分类。构建 target 按
架构边界合并。物理目录和 target 边界允许不同：多个头文件分区可以共同构成一个 target，
而共享源码根中的不同分区也可以继续形成独立 target。

### 4.3 未采用：每实现 object library

为每个算法实现建立 object library 后再聚合，可以保留最大的选择性组合能力，但仍会维护
大量 target 和 CMake 样板。当前实现没有不同的可选依赖或部署边界，普通 `.cpp` 增量编译
已经满足需求，因此不采用。

## 5. 最终物理布局

```text
uw_slam/
├── CMakeLists.txt
├── cmake/
│   ├── Dependencies.cmake
│   ├── Libraries.cmake
│   ├── Applications.cmake
│   ├── Tests.cmake
│   ├── UwMcap.cmake
│   └── UwProtobuf.cmake
├── include/
│   ├── domain/
│   ├── sensor_models/
│   ├── measurement_api/
│   ├── frontends/
│   ├── factor_builders/
│   ├── estimation/
│   ├── mapping/
│   ├── runtime/
│   ├── evaluation/
│   └── adapters/
├── src/
│   ├── domain/
│   ├── sensor_models/
│   ├── frontends/
│   ├── factor_builders/
│   ├── estimation/
│   ├── mapping/
│   ├── runtime/
│   ├── evaluation/
│   └── adapters/
├── tests/
│   ├── domain/
│   ├── core/
│   ├── frontends/
│   ├── factor_builders/
│   ├── estimation/
│   ├── mapping/
│   ├── runtime/
│   ├── evaluation/
│   ├── adapters/
│   ├── contracts/
│   ├── integration/
│   └── fixtures/
├── apps/
├── adapters/
│   ├── ros2/
│   │   ├── include/
│   │   └── src/
│   └── holoocean/
├── schemas/
├── configs/
├── docs/
└── tools/
```

手写公共 include 路径移除物理 `uw/` 层，但 C++ namespace 保持不变：

```cpp
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/config.hpp"

uw::frontends::SonarCfarFrontend frontend(params);
```

由 `schemas/proto/uw/domain/` 生成的 protobuf 头仍使用 `uw/domain/*.pb.h` 路径；该路径来自
跨语言 schema package，不为目录重构而修改。

公共头文件放在 `include/<namespace-or-role>/`。只供单个实现使用的头文件优先放在相应
`src/<role>/`，不扩大公共 API。应用源码集中在 `apps/`；多源文件应用使用应用名前缀的
文件名避免冲突，而不是重新创建 `app/src` package 层。

## 6. 物理合并与独立边界

| 组件 | 物理位置 | 构建 target | 决策 |
|---|---|---|---|
| 手写 domain 代码 | `include/domain`、`src/domain` | `uw::domain` | 物理集中，target 独立 |
| sensor models + measurement API | `include/sensor_models`、`include/measurement_api`、`src/sensor_models` | `uw::core` | 合并 target |
| 所有现有 frontend | `include/frontends`、`src/frontends` | `uw::frontends` | 合并 target |
| 所有 factor builder/residual | `include/factor_builders`、`src/factor_builders` | `uw::factor_builders` | 合并 target |
| estimation | `include/estimation`、`src/estimation` | `uw::estimation` | 独立 target |
| mapping | `include/mapping`、`src/mapping` | `uw::mapping` | 独立 target |
| runtime | `include/runtime`、`src/runtime` | `uw::runtime` | 独立 target |
| evaluation | `include/evaluation`、`src/evaluation` | `uw::evaluation` | 独立 target |
| 无 ROS 的 SVIn/HoloOcean provider | `include/adapters`、`src/adapters` | `uw::adapters` | 合并 target |
| 应用 | `apps/` | 每个 executable 独立 | 源码集中，target 不合并 |

下列组件保留独立源码根：

1. `schemas/`：跨语言事实源，通过代码生成进入 C++ 和 Python。
2. `adapters/ros2/`：条件构建，依赖系统 ROS2 与 `holoocean_interfaces`，并包含独立节点。
3. `adapters/holoocean/`：独立 Python distribution，有自己的包管理和 pytest。
4. `sonar_camera_reconstruction_baseline`：执行外部仓库的 shell baseline，不是本项目 C++
   adapter；迁移到 `baselines/sonar_camera_reconstruction/`。
5. `external_repos/`：只读外部来源，继续排除在本仓库构建和版本控制之外。

代码来源本身不自动构成物理隔离理由。`sonar_cfar_frontend` 和 `sonar_range_factor` 合入
相应共享目录，继续通过源文件版权头和 `NOTICE` 记录 provenance；移动后同步修正
`NOTICE` 的路径。

未来组件只有满足以下至少一个条件时才获得独立源码根：

- 有可关闭的独立外部依赖；
- 可以从默认构建完全排除；
- 使用不同语言、部署或打包方式；
- 是外部程序执行边界；
- 有必须由构建系统隔离的许可证边界。

## 7. Target 与依赖图

真实 target 使用无前缀名称，对内和对外依赖统一引用 `uw::...` alias：

```cmake
add_library(domain STATIC ...)
add_library(uw::domain ALIAS domain)

add_library(core STATIC ...)
add_library(uw::core ALIAS core)
target_link_libraries(core PUBLIC uw::domain)
```

生产依赖图为：

```text
domain_proto
     ↑
uw::domain
     ↑
uw::core
     ↑
├── uw::frontends
├── uw::factor_builders
├── uw::estimation
├── uw::mapping
├── uw::runtime
├── uw::evaluation
└── uw::adapters
        ↑
    uw::ros2_adapters   # 仅 UW_BUILD_ROS2=ON
```

具体约束：

- `domain` 封装生成的 `domain_proto`，业务 target 不直接消费 protobuf 生成 target；
- `core` 合并 sensor models 和 header-only measurement API；
- `frontends` 包含 sonar CFAR、stereo optical depth、stereo landmark VO、声光关联与
  depth fusion；当前它们没有需要构建隔离的可选依赖；
- `factor_builders` 合并 relative-pose、depth 和 sonar-range residual/builder；
- `estimation` 只依赖 core 抽象，不依赖具体 frontend 或 factor builder；
- `mapping` 与 `evaluation` 保持不同 target，分别拥有地图语义和只读评测语义；
- `runtime` 保持独立，以隔离 MCAP、yaml-cpp 和运行控制职责；
- 无 ROS provider 合并为 `adapters`；ROS2 传输层保持条件 target；
- apps 是 composition root，可以按用途组合全部生产层。

合并 target 不启用 unity build。每个 `.cpp` 仍单独编译；修改单个实现只重编对应
translation unit，随后重新归档该层静态库并重链接受影响的应用或测试。未来某个 frontend
实际引入 CUDA、TensorRT 或独立模型 runtime 时，再将该实现拆成条件 target。

## 8. 集中式 CMake

根 `CMakeLists.txt` 只负责项目选项和集中式文件加载：

```cmake
cmake_minimum_required(VERSION 3.22)
project(uw_slam LANGUAGES CXX)

include(cmake/Dependencies.cmake)
include(cmake/Libraries.cmake)
include(cmake/Applications.cmake)

if(UW_BUILD_TESTS)
  include(cmake/Tests.cmake)
endif()
```

职责划分：

- `Dependencies.cmake`：选项以及 Eigen、Protobuf、MCAP、yaml-cpp、GTest 和条件 ROS2
  依赖；
- `Libraries.cmake`：所有生产 library、alias、source list 和 link graph；
- `Applications.cmake`：所有 executable 和其组合依赖；
- `Tests.cmake`：所有测试 executable、CTest discovery、labels 和 integration tests；
- `UwMcap.cmake`、`UwProtobuf.cmake`：保留依赖接入与代码生成细节。

本地源码目录不再包含 `CMakeLists.txt`。源码列表显式书写，不用递归 glob。允许一个小型
helper 统一 C++17、warning 和公共 include root，但 helper 不隐藏 source list 或
`target_link_libraries()`，确保依赖图可直接审计。

重新审计依赖可见性：只有公共头文件签名所暴露的依赖使用 `PUBLIC`；纯实现依赖使用
`PRIVATE`。`UW_BUILD_TESTS` 和 `UW_BUILD_ROS2` 保持现有语义。ROS2 关闭时不得查找或要求
系统 ROS2 package。

构建产物统一到：

```text
build/bin/    # executables
build/lib/    # libraries
```

脚本和文档同步使用新路径，不建立旧输出目录软链接。

## 9. 测试设计

每个生产 library 对应一个 GTest executable，例如 `frontends_tests` 和 `runtime_tests`。
同一模块的多个测试 `.cpp` 合入该 executable，但仍分别增量编译。使用
`gtest_discover_tests()` 注册单个 case：

```text
unit.domain.*
unit.core.*
unit.frontends.*
unit.factor_builders.*
unit.estimation.*
unit.mapping.*
unit.runtime.*
unit.evaluation.*
unit.adapters.*
contract.*
integration.*
```

CTest labels 同时标记测试级别和模块。原 L0 contract 测试迁入 `tests/contracts/`；回放、
确定性和 smoke shell 测试迁入 `tests/integration/`，继续使用 `$<TARGET_FILE:...>` 定位
应用，不依赖输出目录字符串。fixture 集中到 `tests/fixtures/<purpose>/`，通过明确路径访问，
不依赖测试进程当前目录。

estimation 测试可以额外链接 `uw::factor_builders` 来构造具体 residual，但该依赖只存在于
测试 target，不进入生产 `uw::estimation` 的依赖图。

## 10. 依赖边界检查

共享 `include/` 根意味着 CMake include path 无法单独阻止低层源码 include 高层公共头。
新增：

```text
tools/lint/check_layer_dependencies.py
```

检查器解析项目内 include，并执行以下规则：

- domain 只能使用手写 domain 头和生成的 `uw/domain/*.pb.h` protobuf 头；
- sensor models 只能依赖 domain；
- measurement API 可以依赖 domain 和 sensor models；
- frontends 与 factor builders 可以依赖 core；
- estimation 不得依赖具体 frontends 或 factor builders；
- mapping 不得反向依赖 runtime、evaluation 或 apps；
- runtime 不得依赖具体算法实现；
- ROS2 和 vendor message 头只能出现在 `adapters/ros2/`；
- apps 作为 composition root 可以依赖全部生产层。

现有 `tools/lint/check_no_ros_in_core.sh` 的规则并入新检查，同时保留兼容入口调用新检查，
避免已有验证命令立即失效。独立的 ROS2 include root 仍提供编译期隔离；lint 是共享 include
root 内依赖方向的主要结构约束。

## 11. 迁移顺序

迁移分阶段进行，每阶段都恢复可构建状态：

1. **记录基线**：记录现有 target、CTest/pytest 结果和 demo 输出。当前工作区有未提交修改
   及新增 stereo landmark VO frontend，全部视为有效输入，不使用 reset、restore 或覆盖。
2. **集中 CMake**：建立四个集中式 CMake 文件，暂时引用旧源码路径；验证后停止加载子级
   `CMakeLists.txt`。
3. **合并 target**：创建架构层 target 和 `uw::...` alias，更新应用和测试链接，源码仍
   保持旧位置，以隔离构建图问题。
4. **移动生产代码**：迁入共享 `include/`、`src/`，删除 include 路径中的物理 `uw/` 层；
   合并无 ROS adapter，保留 ROS2/Python 边界。
5. **集中测试**：移动测试和 fixture，按层建立 GTest executable 并启用 case discovery。
6. **更新引用**：同步 README、CLAUDE.md、架构文档、`NOTICE`、integration README、lint
   和验证脚本中的路径及 target 名。
7. **清理旧结构**：删除不再使用的子级 `CMakeLists.txt` 和空目录，不创建旧 include/target
   兼容层。
8. **全量验收**：在新的构建目录验证，保留现有 `build/` 和 `build_ros2/`，不主动删除用户
   缓存或未追踪产物。

## 12. 验收标准

完成必须同时满足：

1. 主 C++ 公共头文件只位于顶层 `include/<role>/`，主实现只位于 `src/<role>/`；
2. 除根入口和 `cmake/` 集中文件外，本地 C++ 源码目录没有 `CMakeLists.txt`；
3. 生产 library target 收敛为本文定义的架构层集合，应用与 ROS2 node 仍各自独立；
4. 业务 CMake 依赖只使用 `uw::...` alias，真实 target 不使用 `uw_` 前缀；
5. 项目源码引用手写公共头时不再使用 `uw/...` 路径；生成的 `uw/domain/*.pb.h` 是明确
   例外；C++ `uw::...` namespace 保持不变；
6. 默认 configure/build 成功；
7. `UW_BUILD_TESTS=OFF` 构建成功；
8. CTest unit、contract、integration 全部通过；
9. HoloOcean Python adapter pytest 全部通过；
10. layer dependency lint 和 ROS 隔离规则通过；
11. 合成数据生成与 replay demo 成功，CLI 和输出格式保持一致；
12. 有可用 ROS2 环境时 `UW_BUILD_ROS2=ON` 构建成功；环境不可用时明确记录未执行，不能
    报告为通过；
13. `NOTICE` 与所有文档/脚本路径均指向新布局；
14. 现有未提交功能修改和新增 frontend 完整保留。

## 13. 兼容性决策

这是内部代码库的一次性结构迁移。旧的 `uw_*` target、`uw/...` include 路径和旧构建输出
目录不提供兼容 shim。所有仓库内消费者在同一迁移中更新；运行时契约、数据格式和用户 CLI
保持兼容。旧 build tree 可能因 source/target 路径变化失效，验收使用新 build tree，旧目录
由用户决定何时清理。
