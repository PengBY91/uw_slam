# 双目相机到货验收与标定步骤

> 任务编号：PREP-B-08 第 2 步 · 状态：v1 草案 2026-09-03（到货后执行，到货前定稿）
> 前置文档：`stereo-mounting-constraints.md`（机械约束）。本文是"相机装上机体之后，进主线之前"的门槛：**任何一步不过，双目 VO 不进主线**，回到阶段 1（IMU 预积分 + 声呐配准）。

## 0. 验收阈值总表

| 步骤 | 指标 | 阈值 | 来源 |
|---|---|---|---|
| 1 机械复核 | 基线向量 x、z 分量 / 基线长度 | 各 < 3% | 约束 M2 |
| 2 内参标定 | 每目重投影 RMS | < 0.6 px（仿真标定为 0.43–0.46 px） | `example_auv_real_camera.yaml` 头注释 |
| 3 外参标定 | 立体重投影 RMS；相对旋转 | < 0.8 px；< 1°（三轴各） | 同上（仿真 0.51 px、R 偏差 0.0061） |
| 4 rectify 检查 | 左/右主点搬移 \|Δcx\|、\|Δcy\| 相对图像宽/高 | < 2% | 教训 (a)：真实机体 cx 256→170 即 17% |
| 4 rectify 检查 | 极线误差（rectify 后匹配点行差中位数） | < 0.5 px | — |
| 5 同步检查 | 左右帧时间戳差 | 中位数 < 1 ms，p99 < 2 ms | 约束 M6/M7 |
| 6 VO 跟踪 | 真实录制 50 个关键帧的跟踪成功数 | ≥ 40/50 | 教训 (b)：重采样后 8/50 |
| 6 VO 精度 | 有真值段（水池控制点或仿真）ATE | 不劣于 0.67 m 基线 | `real_holoocean_vo.yaml` 记录 |

## 1. 机械复核（装机当天）

1. 按 `stereo-mounting-constraints.md` 第 3 节向结构侧要 CAD 值，填入 `configs/rig/bluerov2_contract.yaml` 的 `camera_left_link`/`camera_right_link` 作为**初值**（标定后覆盖）。
2. 用卡尺量两相机窗口中心距，与 CAD 基线核对，差异 > 2 mm 记录并追问。
3. 目视/水平仪确认两光轴平行且指向机体 +x；确认相机与声呐、HWT9053 IMU 在同一刚性支架。

## 2. 内参标定（每目独立）

工具：`adapters/holoocean/uw_holoocean_adapter/calibrate_camera.py` 的 `compute_intrinsics()` 走的是标准 OpenCV plumb-bob 流程，真机用同一函数，只是图像来自录制而不是 HoloOcean 采集。

1. 标定板：7×9 内角点棋盘格（与仿真一致），方格 ≥ 0.1 m，**在水中标定**（平面窗折射改变等效焦距，空气中标定不能直接用）。
2. 采集 ≥ 25 个视角，覆盖图像四角和不同距离（0.5–3 m）、不同倾角；相机以原始格式录制（不经有损编码）。
3. 迭代剔除重投影误差最大的视角，直到每目 RMS < 0.6 px 且剩余视角 ≥ 15。
4. 结果写入 rig `cameras` 块：`k_matrix_row_major`、`distortion`（5 系数 plumb-bob）、`width/height`。

## 3. 外参标定（左右相对）

1. 同一批同步采集的左右图，用 `compute_stereo_extrinsics()` 求 `left_to_right_transform`。
2. 用 `stereo_translation_to_body_frame()` 把 OpenCV 光学系（X 右 / Y 下 / Z 前）平移转到机体系（x 前 / y 左 / z 上），与 `include/sensor_models/camera_model.cpp` 的 `OpticalFromBodyRotation()` 一致（单测 `tests/test_calibrate_camera.py` 已验证往返）。
3. 检查：相对旋转三轴各 < 1°；平移向量 x、z 分量 / 模长各 < 3%。**超过 3% 直接回到第 1 步找机械原因**，不要靠软件吸收。
4. 把机体系平移填入 `frame_tree` 的 `camera_right_link`（相对 `base_link`，即左相机位置 + 相对偏移）。

## 4. rectify 检查

`replay_demo` 用 `adapters/opencv/` 的 `StereoRectificationContext`（`cv::stereoRectify` 封装）。在进 VO 之前单独跑一次 rectify 并检查：

1. 用新 rig 构造 `StereoRectificationContext`（`alpha = 0` 与 `alpha = -1` 各一次），读取 `DerivedRig()` 里 rectified 相机的 `k_matrix_row_major`。
2. 主点搬移：\|cx_rect − cx_raw\| / width < 2%，cy 同理，两目都查。真实机体的 17% 就是在这一步露馅的。
3. 极线误差：对 ≥ 10 对 rectified 图做角点匹配（可直接复用 `stereo_landmark_vo_frontend` 的 Harris + NCC），统计匹配点行坐标差的中位数 < 0.5 px。
4. 纹理保持：比较 raw 与 rectified 图的 Harris 角点数，rectified 不少于 raw 的 70%；若明显下降，先检查是否有额外的重采样/压缩环节（教训 (b)）。

## 5. 同步检查

1. 录制 60 s 双目流（`record_session.py` 或真机接入 adapter），用 `bag_audit` 输出左右帧时间戳配对统计。
2. 时间戳差中位数 < 1 ms、p99 < 2 ms；否则检查触发线/时钟源，不进入下一步。
3. 记录抖动值到 rig `time_offset_provenance`（`measured:...`）。

## 6. VO 跟踪与精度

1. 在能见度好的水池或静水区录 ≥ 60 s 缓慢运动（≤ 0.3 m/s，含转向），得到 ≥ 50 个关键帧。
2. 跑 `replay_demo --experiment configs/experiment/real_holoocean_vo.yaml`（rig 换成新文件）：
   - 跟踪成功关键帧数 ≥ 40/50；
   - 求解器 `converged`，不得 `stalled`；
   - 若有真值（水池控制点，PREP-B-06；或仿真 stereo 配置），ATE 不劣于 0.67 m。
3. 用 `PREP-A-09` 的压缩退化配置重复一遍，确认在计划的传输码率下跟踪率不掉出阈值（这决定 `PREP-E-01` 的双目档位）。

## 7. 输出

- `configs/rig/bluerov2_contract.yaml` 更新 `cameras` + `camera_left_link`/`camera_right_link` + `time_offset_*`，`calibration_version` 递增。
- 验收记录（每步数字、采集日期、标定板、水温）作为 `docs/calibration/records/stereo-<date>.md` 归档。
- `PREP-F-03` 到货验收清单里对应的双目条目打勾。

## 8. 版本记录

- v1 2026-09-03：首版。阈值中"< 2% 主点搬移"和"≥ 40/50"为规格 PREP-B-08 原文；其余阈值以仿真标定实测值留 30–50% 余量得出，到货后按首次实测修订。
