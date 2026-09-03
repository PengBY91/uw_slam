# `adapters/wit_imu` — HWT9053-485 IMU 数据链（PREP-D-01 / PREP-D-02）

外挂 IMU（维特 HWT9053-485）从串口到岸上 `/raw/imu` 的第一段：协议解析、
均匀时间轴重建、树莓派侧 UDP 转发服务，以及一次性的设备配置脚本。

出处：`docs/ROV平台到货前准备工作规格-2026-09-02.md` PREP-D-01 / PREP-D-02。
下游 PREP-D-03（岸上 UDP → `LiveEventSource` 的 `/raw/imu`，C++ provider）
还没做，本包只负责把 protobuf `ImuSample` 送上 tether。

## 现在能验证到哪一步

**设备还没到货**，所以这里所有代码都是对着合成字节流写的，51 条单测全部
离线可跑，不需要串口、不需要 IMU。已经覆盖的：

- 11 字节包的解析与量纲换算（加速度 m/s²、角速度 rad/s、磁场原始计数、
  四元数 w-x-y-z）
- 丢字节后的重新同步、数据段里出现 `0x55` 的假包头、校验和失败计数
- 抖动下的均匀时间轴重建、丢周期识别、长停顿重同步、周期钳位
- 一个周期的四种包如何合成一条 `ImuSample`，protobuf 两个时间戳的语义
- 每秒一条 `HealthReport` 的状态判定与掉包/校验失败计数
- 配置寄存器序列的顺序约束、链路预算（9600 波特在 200 Hz 下必然掉包）

**还没验证的**（需要硬件）：寄存器地址与速率/波特率编码是否与实机手册一致
（见下面的警告）、真实 USB-485 转换器上的抖动量级、24 小时掉包率 <0.1%。

## ⚠️ 寄存器常量尚未与手册核对

`uw_wit_imu/registers.py` 里的寄存器号和编码取自 WT901/HWT 系列通用的维特
标准寄存器表，**没有**对着合同机型随附的 HWT9053-485 手册逐条核对过。因此
`configure.py` 默认只做 dry run，并且在 `uw_wit_imu/protocol.py` 的
`MANUAL_REVISION` 还是空串时拒绝真正写设备。

到货后的流程：

```bash
python3 tools/imu/wit_configure.py            # 打印将要写的 6 个帧，对着手册逐条核对
# 核对无误后，把手册版本号填进 protocol.py 的 MANUAL_REVISION
python3 tools/imu/wit_configure.py --port /dev/ttyUSB0 --apply
# 断电重上电，再验收：
python3 tools/imu/wit_dump.py --port /dev/ttyUSB0 --baud 230400 --seconds 60
```

`wit_dump.py` 也能读一个原始抓包文件（`--replay`），离线跑同一套计数逻辑。

## 为什么是 230400 波特 / 200 Hz / 四种包

200 Hz 下四种包 = 4 × 11 = 44 字节/周期 = 8800 B/s。出厂默认 9600 波特只有
960 B/s，不到需求的 11%，而设备**没有流控**——波特率不够时它不会变慢，
只会静默丢包。115200（11520 B/s）勉强够但只剩 30% 余量；230400 留 160%。
欧拉角包被关掉：CLAUDE.md 禁止仓库里出现欧拉角，而且每关一种包就省
11 字节/周期。

## 时间戳为什么不能直接用到达时间

设备自由运行、不带序号也不带设备时间戳，Pi 能看到的只有到达时间，而串口/USB
缓冲带来的抖动是毫秒量级——和 5 ms 的采样周期同数量级。直接把到达时间当
`capture_time`，这个抖动会原样进 IMU 预积分 frontend，而它的零阶保持假设
区间边界是真的。

`timebase.py` 因此用最近一段窗口对 `t = origin + n·period` 做最小二乘拟合，
输出均匀时间轴，到达时间只作为校正量并原样保留在 `receive_time` 里，岸上可以
审计而不必盲信。规格里写的是"PLL"，这里落成窗口最小二乘：稳态行为一样，但
没有需要调的环路增益，对同一串输入逐位可复现（仓库的确定性要求），而且收敛
性可以直接写成断言。

工作区间：抖动在 ±0.25 个周期以内。再大，单靠到达时间就无法区分"这个包晚到"
和"这个包丢了"——这是信息论限制，不是实现问题，正确的解法是把链路做快，不是
把估计器做复杂。200 Hz + 230400 波特下一个周期在线上只占 1.9 ms，所以设计针
对的是亚毫秒抖动。

## 运行测试

```bash
cd adapters/wit_imu
python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"
# protobuf 绑定不入库（**/*_pb2.py 已 gitignore），先生成：
(cd ../.. && tools/codegen/gen_py.sh "$PWD/adapters/wit_imu/uw_wit_imu/schema_pb2")
.venv/bin/pytest -q
```

## BlueOS extension

`docker/Dockerfile` + `docker/manifest.json`。镜像里只有 `pyserial` +
protobuf runtime，没有编译器、没有 numpy。manifest 里两项不是默认值、且缺一
不可：`NetworkMode: host`（UDP 目标 192.168.2.1 在宿主网络上，不在 bridge 上）
和 `/dev/ttyUSB0` 设备映射 + `Privileged`（BlueOS 默认不把 USB 串口映射进扩展
容器）。`RestartPolicy: unless-stopped`，水下掉电重启后不需要岸上有人发现。

构建前要先在仓库根目录生成 protobuf 绑定（见上），build context 是
`adapters/wit_imu`。

## 目录

| 文件 | 内容 |
|---|---|
| `uw_wit_imu/protocol.py` | 11 字节包解析、重同步框架器、量纲换算 |
| `uw_wit_imu/registers.py` | 配置寄存器表、写帧构造、配置序列、链路预算 |
| `uw_wit_imu/timebase.py` | 均匀时间轴重建（窗口最小二乘 + 丢周期/重同步） |
| `uw_wit_imu/forwarder.py` | 周期合成 → `ImuSample`/`HealthReport` protobuf → UDP 帧 |
| `uw_wit_imu/service.py` | 唯一碰串口/socket/时钟的薄循环（BlueOS 入口） |
| `uw_wit_imu/configure.py` | PREP-D-01 一次性配置（默认 dry run） |
| `uw_wit_imu/dump.py` | PREP-D-01 验收计数工具（支持 `--replay`） |
| `tools/imu/wit_{configure,dump}.py` | 规格里写的路径下的薄入口 |

## UDP 帧格式

```
b"UWIM" | version u8 | message type u8 | payload length u32 BE | protobuf payload
```

magic + version 是因为这是 UDP：岸上 adapter（PREP-D-03）没有连接状态，
重启后收到的第一个数据报是 `ImuSample` 还是 `HealthReport` 无从判断，而裸
protobuf 不自描述。长度前缀对 UDP 冗余，但让同一套帧格式能直接落文件或走
TCP 重放——离线测试和 PREP-A-13 的设备伪装流就是这么消费它的。
