# external_repos 恢复说明

本目录存放仿真器、论文代码和历史桥接实现，只用于读取、审计、对比与合规移植。
除本 README 和 [外部代码概览](./external-repos-overview.md) 外，所有子目录都视为
只读外部来源，不在 `uw_slam` 中直接修改或提交。

第三方代码的详细角色、接口与风险见[外部代码概览](./external-repos-overview.md)；
已经移植进主仓库的文件和许可证边界以 [`NOTICE`](../NOTICE) 为准。

## 恢复清单

| 目录 | 来源或获取方式 | 许可证 | 在 `uw_slam` 中的角色 | 已知快照 |
|---|---|---|---|---|
| `HoloOcean/` | `byu-holoocean/HoloOcean` | 非 UE 代码为 MIT；UE 代码/资产受 Epic EULA 约束 | 仿真器源码与 Python API 参考 | `49e70552`（2026-02-05） |
| `holoocean-ros/` | `byu-holoocean/holoocean-ros` | MIT | 当前 HoloOcean ROS2 仿真入口与消息定义 | 本地副本未保留 `.git` 元数据 |
| `SVIn/` | `AutonomousFieldRoboticsLab/SVIn` | GPLv3 | 声呐距离残差来源与 VIO baseline 参考 | `fcda5466`（2026-08-16 本机记录） |
| `sonar_camera_reconstruction/` | `ivanacollg/sonar_camera_reconstruction` | MIT | CFAR、极坐标转换与 DBSCAN 来源 | `9acc41df`（2026-02-09 本机记录） |
| `ocean_t/` | 团队内部 GitLab | 未提供独立许可证 | 已停用的早期 HoloOcean/SLAM 原型 | `f9333e94`（2026-08-05 本机记录） |
| `holoocean_bridge/` | 同事提供的本地副本，无公开地址 | 未确认 | 旧版 HoloOcean→SVIn/重建桥接参考 | 本地副本未保留 `.git` 元数据 |

## 公开仓库恢复命令

从 `uw_slam` 仓库根目录执行：

```bash
git clone https://github.com/byu-holoocean/HoloOcean.git external_repos/HoloOcean
git clone https://github.com/byu-holoocean/holoocean-ros external_repos/holoocean-ros
git clone https://github.com/AutonomousFieldRoboticsLab/SVIn external_repos/SVIn
git clone https://github.com/ivanacollg/sonar_camera_reconstruction \
  external_repos/sonar_camera_reconstruction
```

如需复现表中的历史快照，在相应子仓库中显式 checkout 记录的 commit。没有快照记录的
目录应在恢复时补充 commit 与日期，不能只写“最新版”。

## 特殊来源

### `ocean_t`

该仓库只能从团队局域网访问：

```bash
git clone http://192.168.10.224:8929/lnlyljt/ocean_t.git external_repos/ocean_t
```

它已经被主仓库的 HoloOcean 和 ROS2 adapter 取代，只保留供历史审计，不应重新作为
当前集成入口。没有明确授权前，不要从中移植代码。

### `holoocean_bridge`

没有可公开 clone 的地址，需要向原维护者索取。该副本的 `package.xml` 与 `setup.py`
没有给出可依赖的许可证结论，因此只能参考消息、坐标和启动链路；任何代码移植都必须
先确认授权并更新 [`NOTICE`](../NOTICE)。

## 只读规则

- 不在这些子目录里修 bug、格式化代码或生成构建产物提交到主仓库。
- 需要适配时，在主仓库 `adapters/` 中实现边界；需要移植时保留版权头并登记来源。
- 更新外部副本时记录 upstream URL、commit、日期和本地 patch，不用目录内容猜版本。
- `holoocean-ros` 的 colcon workspace、ROS2 安装和生成物放在仓库外，具体见
  [ROS2 适配器说明](../adapters/ros2/README.md)。
