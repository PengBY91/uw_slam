# Frontend Correctness Closure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让真实双目、声光同步、VO 连续性、不确定性、运行状态和融合门禁形成可验证的正确性闭环，消除 raw/rectified 几何混用、同步失败伪造为零时差、失败帧推进 VO 参考、固定权重掩盖量测质量以及空洞状态/地图验收。

**Architecture:** 按三个有依赖关系的实施单元推进。单元 1 新增隔离的 `opencv_adapters` 边界，在应用层把 raw 图像和原始 rig 一次性变成 rectified 图像与不可变派生 rig，纯 Eigen/frontend 层只消费仓库自有类型；单元 2 把同步决定、VO 参考连续性和真实时间/状态变成显式状态机；单元 3 从 VO covariance 与标量 sigma 构造有上限的白化权重，并用视差过滤、融合贡献计数和非零 gate 关闭“有输出但无可信贡献”的漏洞。每个单元先增加会失败的单元/集成测试，再改生产代码，单元末执行完整回归。

**Tech Stack:** C++17、Eigen3、OpenCV 4 (`core`/`calib3d`/`imgproc`)、Protocol Buffers、yaml-cpp、GoogleTest/CTest、Python 3 lint、MCAP、GitHub Actions、conda-forge。

---

## 依据、边界与执行规则

- 设计依据：`docs/archive/superpowers/specs/2026-08-23-frontend-correctness-closure-design.md`。若实施中发现本计划与该规格冲突，以规格为准并先更新计划，不在代码里静默改变语义。
- 三个单元必须顺序完成；单元 2 依赖 rectified bundle/derived rig，单元 3 依赖单元 2 的 VO 失败计数和状态语义。
- `external_repos/` 只读；OpenCV 类型只允许出现在 `adapters/opencv/` 的私有实现和该适配器自己的测试中。
- 每个红灯步骤只运行指定测试并确认失败原因就是缺失行为；实现后先跑相同测试，再跑该 target 全部测试。
- 下文的提交步骤受仓库规则约束：只有用户明确授权提交时才执行 `git add`/`git commit`；未授权时把它视为人工检查点，不创建提交。
- 所有命令从仓库根目录执行，并先设置：

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
export CMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
```

## 实施单元 1：OpenCV 双目校正与派生标定

### Task 1：建立 OpenCV 构建边界、依赖 lint 与开发环境契约

**Files:**
- Modify: `cmake/Dependencies.cmake`
- Modify: `tools/lint/check_layer_dependencies.py`
- Modify: `tests/lint/check_layer_dependencies_test.py`
- Modify: `tools/setup_dev_env.sh`
- Modify: `.github/workflows/ci.yml`
- Modify: `README.md`

- [ ] **Step 1: 先写 lint 红灯测试**

在 `tests/lint/check_layer_dependencies_test.py` 增加三个精确用例：

```python
def test_scans_opencv_adapter_and_allows_only_core_contract_dependencies(self):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        self.write(
            root,
            "adapters/opencv/src/stereo_rectifier.cpp",
            '#include <opencv2/calib3d.hpp>\n'
            '#include "measurement_api/frontend.hpp"\n'
            '#include "sensor_models/geometry.hpp"\n',
        )
        self.assertEqual(load_checker().check(root), [])

def test_rejects_opencv_header_outside_opencv_adapter(self):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        self.write(root, "src/frontends/example.cpp", "#include <opencv2/core.hpp>\n")
        errors = load_checker().check(root)
        self.assertEqual(len(errors), 1)
        self.assertIn("OpenCV header", errors[0])

def test_rejects_opencv_adapter_to_frontend_dependency(self):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        self.write(
            root,
            "adapters/opencv/src/stereo_rectifier.cpp",
            '#include "frontends/stereo_optical_depth_frontend.hpp"\n',
        )
        errors = load_checker().check(root)
        self.assertEqual(len(errors), 1)
        self.assertIn("opencv_adapters must not include frontends", errors[0])
```

- [ ] **Step 2: 运行 lint 单测并确认红灯**

Run:

```bash
python3 tests/lint/check_layer_dependencies_test.py -v
```

Expected: 新增的扫描/边界用例失败；现有六个用例仍通过。

- [ ] **Step 3: 实现 lint 所有权与 vendor 边界**

在 `tools/lint/check_layer_dependencies.py` 中：

```python
PROJECT_ROLES = {
    # existing roles...
    "opencv_adapters",
}

ALLOWED["opencv_adapters"] = {
    "opencv_adapters", "measurement_api", "sensor_models", "domain", "domain_proto"
}

def owner(root: Path, path: Path):
    parts = path.relative_to(root).parts
    if len(parts) > 2 and parts[:2] == ("adapters", "opencv"):
        return "opencv_adapters"
    # existing branches...

def source_files(root: Path):
    for relative in ("include", "src", "adapters/ros2", "adapters/opencv", "apps"):
        # existing traversal...
```

同时增加 `opencv2/` 头检查：只有 `source_owner == "opencv_adapters"` 才允许；`application` 和 `apps` 可 include `opencv_adapters/...` 自有头，但不能直接 include `opencv2/...`。

- [ ] **Step 4: 声明 OpenCV 硬依赖**

在 `cmake/Dependencies.cmake` 增加：

```cmake
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d imgproc)
```

- [ ] **Step 5: 把 OpenCV 写入环境、CI 与 README**

更新 `tools/setup_dev_env.sh`：

```bash
APT_PACKAGES=(... libopencv-dev)
CONDA_PACKAGES=(... opencv)
```

更新 `.github/workflows/ci.yml` 三个 job 的名称与安装命令，使每处 conda 安装都包含 `opencv`。在 `README.md` 构建依赖、conda 安装示例、架构依赖图和“已知边界”中说明：OpenCV 是硬依赖，但只位于 `opencv_adapters`，旧 `camera_rectifier` 不再代表完整离轴双目路径。

- [ ] **Step 6: 验证 lint 与依赖发现**

Run:

```bash
python3 tests/lint/check_layer_dependencies_test.py -v
tools/lint/check_layer_dependencies.py .
pkg-config --modversion opencv4
```

Expected: lint 全通过；OpenCV 输出 4.x 版本。

- [ ] **Step 7: 提交检查点（仅经用户授权）**

```bash
git add cmake/Dependencies.cmake tools/lint/check_layer_dependencies.py \
  tests/lint/check_layer_dependencies_test.py \
  tools/setup_dev_env.sh .github/workflows/ci.yml README.md
git commit -m "build: isolate OpenCV stereo adapter dependency"
```

### Task 2：实现不可变 StereoRectificationContext 与 identity fast path

**Files:**
- Create: `adapters/opencv/include/opencv_adapters/stereo_rectifier.hpp`
- Create: `adapters/opencv/src/stereo_rectifier.cpp`
- Create: `tests/adapters/opencv_stereo_rectifier_test.cpp`
- Modify: `cmake/Libraries.cmake`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: 写公开 API 和第一组红灯测试**

公开头不得出现任何 `cv::` 类型：

```cpp
namespace uw::opencv_adapters {

enum class RectificationCropPolicy { kFullCanvas, kCommonValidRoi };

struct StereoRectificationParams {
  std::string left_sensor_id = "camera_left";
  std::string left_frame = "camera_left_link";
  std::string right_sensor_id = "camera_right";
  std::string right_frame = "camera_right_link";
  std::string rectified_frame_suffix = "_rectified";
  double alpha = 0.0;
  RectificationCropPolicy crop_policy = RectificationCropPolicy::kFullCanvas;
};

struct RectifiedStereoBundle {
  uw::measurement_api::CameraFrameBundle images;
};

class StereoRectificationContext {
 public:
  static std::optional<StereoRectificationContext> Create(
      const uw::domain::RigCalibrationSnapshot& raw_rig,
      const StereoRectificationParams& params,
      std::string* error);

  std::optional<RectifiedStereoBundle> Process(
      const uw::domain::ImageFrame& left,
      const uw::domain::ImageFrame& right,
      std::string* error) const;

  const uw::domain::RigCalibrationSnapshot& DerivedRig() const;
  const std::string& LeftRectifiedFrame() const;
  const std::string& RightRectifiedFrame() const;

 private:
  class Impl;
  explicit StereoRectificationContext(std::shared_ptr<const Impl> impl);
  std::shared_ptr<const Impl> impl_;
};
}  // namespace uw::opencv_adapters
```

在 `tests/adapters/opencv_stereo_rectifier_test.cpp` 先覆盖：缺相机、非法 K、缺 frame edge、左右尺寸不同、非 `plumb_bob` model 均 `Create()==nullopt` 且 error 非空；已经零畸变、同内参、符合 rectified 几何的 rig 走 identity path，但仍生成两个新虚拟 frame 和新的 calibration version。

- [ ] **Step 2: 新增显式 measurement_api/OpenCV adapter targets 并注册测试**

`CameraFrameBundle` 来自 header-only 契约，但用户确认的依赖边界要求在 CMake 中也显式表达。因此先在 `core` target 之后创建 interface target，并让现有消费者逐步链接它：

```cmake
add_library(measurement_api INTERFACE)
add_library(uw::measurement_api ALIAS measurement_api)
target_include_directories(measurement_api INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(measurement_api INTERFACE uw::core)

add_library(opencv_adapters STATIC
  adapters/opencv/src/stereo_rectifier.cpp
)
add_library(uw::opencv_adapters ALIAS opencv_adapters)
target_include_directories(opencv_adapters PUBLIC
  "${PROJECT_SOURCE_DIR}/adapters/opencv/include"
)
target_link_libraries(opencv_adapters
  PUBLIC uw::measurement_api
  PRIVATE ${OpenCV_LIBS}
)
uw_apply_library_defaults(opencv_adapters)
```

把 `uw::measurement_api` 加到 `frontends`、`factor_builders`、`estimation`、`mapping`、`runtime` 的 PUBLIC link 列表，取代这些 target 通过 `uw::core` 偶然获得 header/link 依赖的现状；把 `uw::opencv_adapters` 加到 `application` 的 PRIVATE link 列表。

将测试文件加入 `adapters_tests`，并给 target 增加 `uw::opencv_adapters` link。

Run:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
cmake --build build --target adapters_tests -j"$(nproc)"
```

Expected: 因 API/实现尚不存在而编译失败。

- [ ] **Step 3: 实现严格输入验证和私有 OpenCV 缓存**

`Impl` 仅在 `.cpp` 中持有 `cv::Mat map_left_x/map_left_y/map_right_x/map_right_y`、输入尺寸、派生 rig、frame 名和 `identity_fast_path`。`Create()` 必须验证：

- sensor/frame/suffix 非空且左右不同；
- K 恰好 9 项且全部有限，`fx/fy > 0`，尺寸非零且相同；
- distortion 是 0、4、5、8、12 或 14 项且有限，model 是 `plumb_bob`；
- frame edge 恰好能解析 4x4 有限刚体变换；
- `alpha` 有限且在 `[-1, 1]`。

只在“零畸变 + 两相机 K 相同 + 原 optical 相对旋转为单位阵 + optical baseline 仅有 x 分量”时启用 identity fast path；它仍复制 raw rig、替换左右 camera intrinsics、追加 virtual frame edges、生成派生 calibration version。

- [ ] **Step 4: 保证每帧元数据语义**

`Process()` 先调用现有 `ValidateImageFrame`，只接受 MONO8、和 context 尺寸一致的左右帧。输出逐字段保留 `ObservationHeader`（capture/receive time、sensor ID、observation ID），只改 `sensor_frame`；保留 sensor ID，设置 `is_rectified=true`；identity path 逐字节复制 pixel payload，remap path 在 Task 3 完成。

- [ ] **Step 5: 运行 adapter 测试**

```bash
cmake --build build --target adapters_tests -j"$(nproc)"
build/bin/tests/adapters_tests --gtest_filter='*StereoRectificationContext*'
```

Expected: 上述输入验证、identity、metadata 测试全通过。

- [ ] **Step 6: 提交检查点（仅经用户授权）**

```bash
git add adapters/opencv cmake/Libraries.cmake cmake/Tests.cmake tests/adapters/opencv_stereo_rectifier_test.cpp
git commit -m "feat: add isolated stereo rectification context"
```

### Task 3：实现一般离轴校正、虚拟外参与几何回归

**Files:**
- Modify: `adapters/opencv/src/stereo_rectifier.cpp`
- Modify: `tests/adapters/opencv_stereo_rectifier_test.cpp`

- [ ] **Step 1: 写非平行 rig 几何红灯测试**

测试 fixture 必须同时满足：左右 K 不同、5 项 plumb-bob 畸变非零、右相机有 yaw/pitch、小幅竖直和前向平移。用已知 3D 点分别通过 raw camera model 投影并畸变，绘制可辨识标记，创建 context 后断言：

```cpp
ASSERT_TRUE(context.has_value()) << error;
const auto rectified = context->Process(raw_left, raw_right, &error);
ASSERT_TRUE(rectified.has_value()) << error;
EXPECT_TRUE(rectified->images.primary.is_rectified());
EXPECT_TRUE(rectified->images.secondary->is_rectified());

const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
    context->DerivedRig(), "camera_left", context->LeftRectifiedFrame(),
    "camera_right", context->RightRectifiedFrame());
ASSERT_TRUE(geometry.valid);
EXPECT_LT(std::abs(left_rectified_v - right_rectified_v), 0.25);
EXPECT_NEAR(geometry.left.fx * geometry.baseline_m / disparity_px,
            expected_depth_m, 0.05);
```

再增加 `kCommonValidRoi` 测试：左右输出尺寸相同，K 的 `cx/cy` 正确减去公共 ROI 原点，且图像 payload 尺寸与 stride 一致。

- [ ] **Step 2: 运行测试并确认一般路径红灯**

```bash
cmake --build build --target adapters_tests -j"$(nproc)"
build/bin/tests/adapters_tests --gtest_filter='*StereoRectificationContext.GeneralRig*:*StereoRectificationContext.CommonValidRoi*'
```

Expected: context 无法处理一般 rig 或几何断言失败。

- [ ] **Step 3: 实现 OpenCV 标定转换与 remap**

转换约定必须写入 `.cpp` 注释和测试：

```cpp
// B_T_C1opt and B_T_C2opt map optical-frame points into base_link.
// OpenCV requires C2_T_C1: p_C2 = R * p_C1 + T.
const Pose3 c2_T_c1 = b_T_c2opt.Inverse() * b_T_c1opt;
cv::stereoRectify(K1, D1, K2, D2, image_size,
                  CvRotation(c2_T_c1.rotation), CvVector(c2_T_c1.translation),
                  R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
                  params.alpha, output_size, &roi1, &roi2);
cv::initUndistortRectifyMap(K1, D1, R1, P1, output_size,
                            CV_32FC1, map_left_x, map_left_y);
```

`Process()` 用 `cv::remap(..., cv::INTER_LINEAR, cv::BORDER_CONSTANT)`。运行时尺寸变化直接返回 error，不重建或复用旧 map。

- [ ] **Step 4: 从 R/P 严格构造派生 rig**

派生 camera K 取 `P1/P2` 的 3x3 左块，distortion 清空，model 设为 `plumb_bob`，sensor ID 不变。虚拟 optical pose 使用：

```cpp
B_R_Crect = B_R_Craw * R_rect_from_raw.transpose();
B_t_Crect = B_t_Craw;
```

然后按仓库固定 `OpticalFromBodyRotation()` 约定写成 frame tree 的 `base_link_T_camera_rectified_link`。metric baseline 从 `P2(0, 3) / P2(0, 0)` 的绝对值取得，并用派生 frame translation 的 rectified optical x 分量交叉校验；不一致、非有限或非正值则 `Create()` 失败。

派生 rig 保留 raw rig 的所有原始 frame edge、sonar/IMU/depth/time offset；只替换同 sensor ID 的两项 camera calibration，并追加两个不重名 virtual edges。calibration version 使用稳定 FNV-1a：`raw_version + "+opencv_rectified_v1_" + digest`，digest 输入包含 raw rig 序列化值、左右 ID/frame、尺寸、alpha、crop policy、suffix。

- [ ] **Step 5: 运行 adapter 与 sanitizer 定向回归**

```bash
cmake --build build --target adapters_tests -j"$(nproc)"
build/bin/tests/adapters_tests --gtest_filter='*StereoRectificationContext*'
```

Expected: identity、general rig、ROI、尺寸变化、无效输入全部通过；公开头 `rg -n 'cv::|opencv2/' adapters/opencv/include` 无输出。

- [ ] **Step 6: 提交检查点（仅经用户授权）**

```bash
git add adapters/opencv/src/stereo_rectifier.cpp tests/adapters/opencv_stereo_rectifier_test.cpp
git commit -m "feat: rectify arbitrary plumb-bob stereo rigs"
```

### Task 4：收紧 StereoGeometry 与两个 stereo frontend 的 rectified 契约

**Files:**
- Modify: `include/sensor_models/camera_model.hpp`
- Modify: `src/sensor_models/camera_model.cpp`
- Modify: `tests/core/camera_model_test.cpp`
- Modify: `src/frontends/stereo_optical_depth_frontend.cpp`
- Modify: `tests/frontends/stereo_optical_depth_frontend_test.cpp`
- Modify: `src/frontends/stereo_landmark_vo_frontend.cpp`
- Modify: `tests/frontends/stereo_landmark_vo_frontend_test.cpp`
- Modify: synthetic image fixtures in tests/apps that deliberately construct rectified pairs

- [ ] **Step 1: 写契约红灯测试**

在 camera model 测试覆盖：K 项缺失/NaN、左右尺寸不同、旋转不一致、rectified optical baseline 含 y/z、零/负 metric baseline 均 invalid；有效派生 rig 返回正 baseline。两个 frontend 分别增加 `is_rectified=false` 拒绝用例，即使 rig 几何看似平行也不能继续。

- [ ] **Step 2: 确认红灯**

```bash
cmake --build build --target core_tests frontends_tests -j"$(nproc)"
build/bin/tests/core_tests --gtest_filter='*StereoGeometry*'
build/bin/tests/frontends_tests --gtest_filter='*StereoOpticalDepthFrontend*:*StereoLandmarkVoFrontend*'
```

- [ ] **Step 3: 实现纯 Eigen rectified 验证**

`StereoGeometry::Resolve()` 先构造左右 `PinholeCamera` 并验证有限性/尺寸；把两条 frame edge 转换到 optical convention 后计算 `left_T_right`。要求旋转 `isApprox(I, 1e-8)`，`abs(t.y/z) <= 1e-8`，并按当前 disparity 符号约定要求水平基线方向一致，最终只暴露正的 `baseline_m`。

两个 frontend 在任何检测/匹配之前检查：secondary 存在、左右 `is_rectified()`、header frame 与 params frame 一致、图像尺寸与 `StereoGeometry` K 尺寸一致。所有合成 fixture 显式 `set_is_rectified(true)`，避免测试通过默认 false 假装契约成立。

- [ ] **Step 4: 回归**

```bash
cmake --build build --target core_tests frontends_tests -j"$(nproc)"
build/bin/tests/core_tests --gtest_filter='*StereoGeometry*'
build/bin/tests/frontends_tests --gtest_filter='*StereoOpticalDepthFrontend*:*StereoLandmarkVoFrontend*'
```

- [ ] **Step 5: 提交检查点（仅经用户授权）**

```bash
git add include/sensor_models/camera_model.hpp src/sensor_models/camera_model.cpp \
  tests/core/camera_model_test.cpp src/frontends/stereo_optical_depth_frontend.cpp \
  tests/frontends/stereo_optical_depth_frontend_test.cpp src/frontends/stereo_landmark_vo_frontend.cpp \
  tests/frontends/stereo_landmark_vo_frontend_test.cpp
git commit -m "fix: enforce rectified stereo geometry contract"
```

### Task 5：配置化 rectification 并接入 replay 的唯一数据流

**Files:**
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `configs/defaults/platform.yaml`
- Modify: `configs/README.md`
- Modify: `include/runtime/run_manifest.hpp`
- Modify: `src/runtime/run_manifest.cpp`
- Modify: `tests/runtime/runtime_test.cpp`
- Modify: `src/application/replay_pipeline.cpp`
- Modify: `tests/application/replay_pipeline_test.cpp`

- [ ] **Step 1: 写配置和 manifest 红灯测试**

新增类型化配置：

```cpp
struct StereoRectificationConfig {
  double alpha = 0.0;
  std::string crop_policy = "full_canvas";
  std::string frame_suffix = "_rectified";
};
```

测试默认值、合法覆盖，以及 alpha 超界、空 suffix、未知 crop policy、`rectification` 下未知键启动失败。给 `RunManifest` 增加 `derived_calibration_hash` 序列化测试，确认原始 `calibration_hash` 仍存在且二者是独立 JSON 字段。

- [ ] **Step 2: 确认红灯**

```bash
cmake --build build --target runtime_tests -j"$(nproc)"
build/bin/tests/runtime_tests --gtest_filter='*Config*:*RunManifest*'
```

- [ ] **Step 3: 实现解析与严格验证**

在 `PlatformDefaultsConfig` 中加入 `stereo_rectification`。YAML 使用：

```yaml
frontends:
  stereo_rectification:
    alpha: 0.0
    crop_policy: full_canvas
    frame_suffix: _rectified
```

`LoadPlatformDefaultsConfig()` 对新增 section 调用 `RejectUnknownKeys()`，并对所有数值执行 `std::isfinite` 和范围检查。`full_canvas`/`common_valid_roi` 显式映射到 adapter enum；不接受拼写近似值。

- [ ] **Step 4: 让 replay 只向 stereo frontends 传 rectified 数据**

加载实验后立即构造一次 `StereoRectificationContext`；只要 rig 有左右相机且选择 stereo VO/depth，初始化失败就打印具体 error 并返回 1。读取 camera 时先保留 raw frame 用于 `keyframe_id -> capture_time` 和同步，再 `ConvertToMono8`，然后 `context.Process()`。VO、depth、fusion、map bridge 全部使用 `context.DerivedRig()` 和 rectified frame 名；删除现有“暂不接 camera_rectifier”的注释与 raw bundle 旁路。Sonar frontend 与同步器仍消费 raw sonar/raw image header。

同一 keyframe 只 rectification 一次，并把 `RectifiedStereoBundle` 缓存在 `rectified_by_kf`，禁止 VO pass 和 acoustic-optic pass 各自 remap 一次。

- [ ] **Step 5: 记录派生标定 provenance**

```cpp
manifest.calibration_hash = Fnv1aHex(raw_rig.SerializeAsString());
manifest.derived_calibration_hash =
    Fnv1aHex(rectification_context->DerivedRig().SerializeAsString());
```

无 rectification context 时 derived hash 为空字符串，而不是复制 raw hash。

- [ ] **Step 6: 增加应用级小型 fixture 回归**

扩展 `tests/application/replay_pipeline_test.cpp`：用临时 MCAP 写两帧非平行 raw stereo + 必要 depth/pose evidence 和实验 YAML，运行 replay 后断言返回码不是 rectification error、manifest 两个 hash 非空且不同、轨迹已输出。测试不检查 OpenCV 内部数值（由 adapter 单测负责），只证明应用没有再把 raw 图送给严格 frontend。

- [ ] **Step 7: 单元 1 验收**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -R 'unit.core|unit.adapters|unit.frontends|unit.runtime|unit.application|lint.layer'
```

Expected: 全通过；运行 `rg -n 'opencv2/|cv::' include src apps` 无输出。

- [ ] **Step 8: 提交检查点（仅经用户授权）**

```bash
git add include/runtime/config.hpp src/runtime/config.cpp tests/runtime/config_test.cpp \
  configs/defaults/platform.yaml configs/README.md include/runtime/run_manifest.hpp \
  src/runtime/run_manifest.cpp tests/runtime/runtime_test.cpp \
  src/application/replay_pipeline.cpp tests/application/replay_pipeline_test.cpp
git commit -m "feat: wire rectified stereo and derived calibration into replay"
```

## 实施单元 2：同步决定、VO 连续性与真实状态

### Task 6：用 SynchronizationDecision 替换含糊 optional，并在几何前拒绝时差

**Files:**
- Modify: `include/runtime/acoustic_optic_synchronizer.hpp`
- Modify: `src/runtime/acoustic_optic_synchronizer.cpp`
- Modify: `tests/runtime/acoustic_optic_synchronizer_test.cpp`
- Modify: `include/frontends/acoustic_optic_associator.hpp`
- Modify: `src/frontends/acoustic_optic_associator.cpp`
- Modify: `tests/frontends/acoustic_optic_associator_test.cpp`
- Modify: `src/application/replay_pipeline.cpp`

- [ ] **Step 1: 写同步状态红灯测试**

公开 API 改为：

```cpp
enum class SynchronizationStatus {
  kSynchronized,
  kNoSonar,
  kTimeDeltaExceeded,
  kInvalidTimestamp,
};

struct SynchronizationDecision {
  SynchronizationStatus status = SynchronizationStatus::kInvalidTimestamp;
  double max_pairwise_time_delta_s = 0.0;
  std::optional<SynchronizedAcousticOpticBundle> bundle;
};

SynchronizationDecision SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary,
    const std::optional<uw::domain::ImageFrame>& secondary,
    const std::optional<uw::domain::SonarFrame>& sonar,
    const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params);
```

测试四种 status、超限时仍返回真实 delta、无 sonar 不算 invalid、全零 stamp 合法；nanos=-1/1e9、空 sensor ID、offset 后非有限均为 invalid 且 bundle 为空。

- [ ] **Step 2: 写 associator 时间 gate 红灯测试**

给 `AcousticOpticAssociatorParams` 增加 `max_time_delta_s=0.05`。新增用例：即使 calibration 和像素完全一致，只要 delta=0.051，就在任何投影前返回一条 `REJECTED/TIME_DELTA` record，记录真实 delta，accepted 计数不增加。

- [ ] **Step 3: 确认红灯**

```bash
cmake --build build --target runtime_tests frontends_tests -j"$(nproc)"
build/bin/tests/runtime_tests --gtest_filter='*AcousticOpticSynchronizer*'
build/bin/tests/frontends_tests --gtest_filter='*AcousticOpticAssociator*'
```

- [ ] **Step 4: 实现时间验证和显式决定**

内部 `CorrectedTime()` 返回 `std::optional<double>`；先验证 stamp nanos、sensor ID、offset、结果有限，再计算 delta。`kNoSonar` 不生成 bundle；`kTimeDeltaExceeded` 保留 delta 但不生成 bundle；只有 `kSynchronized` 生成含 sonar 的 bundle。

Associator 的第一个可审计分支是 time gate；它发生在 `FindCamera`/`ProjectSonarArcToCamera` 之前。空 hypothesis 仍表示 no-sonar optical-only，不伪造 TIME_DELTA record。

- [ ] **Step 5: 删除 replay 的 delta=0 降级**

应用按 decision 分支：

- `kSynchronized`：产生 sonar hypothesis 并执行 fusion；
- `kNoSonar`：用空 hypothesis 产生 optical-only；
- `kTimeDeltaExceeded`：保留 sonar hypothesis，把真实 delta 传给 Fusion；Associator 的首个 time gate 直接产生 `REJECTED/TIME_DELTA`，Fusion 保持 optical-only，且不会进入几何投影；
- `kInvalidTimestamp`：丢弃该跨模态组合并增加同步错误计数，光学链仍可独立继续。

禁止存在 `sync_bundle.has_value() ? ... : 0.0`。

- [ ] **Step 6: 回归并提交检查点（仅经用户授权）**

```bash
cmake --build build --target runtime_tests frontends_tests application_tests -j"$(nproc)"
build/bin/tests/runtime_tests --gtest_filter='*AcousticOpticSynchronizer*'
build/bin/tests/frontends_tests --gtest_filter='*AcousticOpticAssociator*:*AcousticOpticDepthFusion*'
git add include/runtime/acoustic_optic_synchronizer.hpp src/runtime/acoustic_optic_synchronizer.cpp \
  tests/runtime/acoustic_optic_synchronizer_test.cpp include/frontends/acoustic_optic_associator.hpp \
  src/frontends/acoustic_optic_associator.cpp tests/frontends/acoustic_optic_associator_test.cpp \
  src/application/replay_pipeline.cpp
git commit -m "fix: preserve acoustic optic synchronization failures"
```

### Task 7：让 VO 只推进最后成功参考帧，并公开连续失败健康状态

**Files:**
- Modify: `include/frontends/stereo_landmark_vo_frontend.hpp`
- Modify: `src/frontends/stereo_landmark_vo_frontend.cpp`
- Modify: `tests/frontends/stereo_landmark_vo_frontend_test.cpp`
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `configs/defaults/platform.yaml`

- [ ] **Step 1: 写成功—失败—恢复红灯测试**

构造 `kf0` 建参考、`kf1` 故意无足够 landmark、`kf2` 与 `kf0` 可匹配。断言 `kf1` 无 evidence，`kf2` evidence 的 `from_keyframe == "kf0"`、`to_keyframe == "kf2"`。再连续输入失败帧，阈值前 Health 为 SUSPECT，达到 `max_consecutive_failures` 后为 UNAVAILABLE/reason `vo_tracking_lost`；成功拟合后清零并回到 HEALTHY。

- [ ] **Step 2: 确认红灯**

```bash
cmake --build build --target frontends_tests -j"$(nproc)"
build/bin/tests/frontends_tests --gtest_filter='*StereoLandmarkVoFrontend*'
```

- [ ] **Step 3: 分离 processed frame 与 reference keyframe**

替换成员：

```cpp
bool has_reference_ = false;
std::string reference_keyframe_id_;
std::vector<TriangulatedLandmark> reference_landmarks_;
uint64_t consecutive_failures_ = 0;
```

第一帧只有在 stereo landmarks 数达到 `min_landmarks_for_pose` 时才建参考。已经有参考时，任一缺 secondary、非法 rectified bundle、landmark 不足、temporal match 不足、RANSAC 失败都调用统一 `RecordTrackingFailure()`，不写 reference。只有 evidence 完整生成后才 `PromoteReference(current)` 并清零计数。

- [ ] **Step 4: 配置化阈值并验证**

增加 `visual_odometry.max_consecutive_failures`，默认 3，范围 `[1, 1000]`，未知键拒绝。replay 从配置传入 frontend，不再使用硬编码阈值。

- [ ] **Step 5: 回归并提交检查点（仅经用户授权）**

```bash
cmake --build build --target frontends_tests runtime_tests -j"$(nproc)"
build/bin/tests/frontends_tests --gtest_filter='*StereoLandmarkVoFrontend*'
build/bin/tests/runtime_tests --gtest_filter='*Config*'
git add include/frontends/stereo_landmark_vo_frontend.hpp src/frontends/stereo_landmark_vo_frontend.cpp \
  tests/frontends/stereo_landmark_vo_frontend_test.cpp include/runtime/config.hpp src/runtime/config.cpp \
  tests/runtime/config_test.cpp configs/defaults/platform.yaml
git commit -m "fix: retain last successful VO reference keyframe"
```

### Task 8：真实 keyframe 时间、StateSnapshot 内容和 TRACKING/DEGRADED/LOST 语义

**Files:**
- Modify: `include/application/replay_pipeline.hpp`
- Modify: `src/application/replay_pipeline.cpp`
- Modify: `tests/application/replay_pipeline_test.cpp`

- [ ] **Step 1: 提取可单测的状态决策**

在 application header 增加不依赖 IO 的内部数据结构/函数：

```cpp
struct ReplayTrackingInputs {
  bool solver_converged = false;
  bool vo_enabled = false;
  uw::domain::HealthReport::Status vo_health = uw::domain::HealthReport::STATUS_UNSPECIFIED;
};

uw::domain::StateSnapshot::TrackingStatus DecideTrackingStatus(
    const ReplayTrackingInputs& inputs);
```

红灯测试精确覆盖：converged+healthy→TRACKING；stalled→DEGRADED；VO SUSPECT→DEGRADED；VO UNAVAILABLE→LOST。solver stalled 即使有 pose 也不得 TRACKING。

- [ ] **Step 2: 写 snapshot/timestamp 红灯测试**

应用 fixture 使用非 0.2 秒间隔（例如 10.03s、10.41s、11.20s），断言输出 TUM 时间逐项等于左相机 raw capture time。读取 `StateStore` 前通过一个新 helper `BuildStateSnapshot(...)` 测试：capture timestamp、derived calibration version、state version、去重后的 contributing evidence IDs 都被填充。

- [ ] **Step 3: 建立真实元数据索引**

首次读取左 raw camera 时建立：

```cpp
std::unordered_map<std::string, uw::domain::Stamp> capture_time_by_keyframe;
std::unordered_map<std::string, std::vector<uw::domain::EvidenceId>> evidence_by_keyframe;
```

black-box relative pose path没有 camera observation ID 时，按 evidence 的 from/to keyframe 收集来源，并从对应 depth/GT/raw camera 中优先补齐真实时间；只有完全没有可用 timestamp 的兼容输入才使用 MCAP log time，并输出 health/reason，禁止再使用 `index * 0.2` 构造轨迹时间。

- [ ] **Step 4: 填充状态并保留非零 solver gate**

solver summary 和 VO Health 决定所有 snapshots 的 status；LOST 从首次达到阈值且无法连接的 keyframe 开始，之前成功 keyframe 不回溯改成 LOST。每个 snapshot 写：

VO pass 每处理一个 keyframe 就把当时的 `Health().status()` 和 `consecutive_failures` 记录到 `vo_health_by_keyframe`；构造 snapshot 时使用该时刻的值，而不是把 frontend 最终 Health 复制给全部历史 keyframe。black-box 模式没有本地 VO health，状态只由 solver/input availability 决定。

- `state_id`、单调 `state_version`；
- raw capture timestamp；
- pose；
- `tracking_status`；
- derived rig calibration version（无 rectification 时 raw version）；
- 当前 keyframe 实际消费的 relative/depth/sonar evidence IDs，排序去重以保证确定性。

保持 `require_converged=true` 默认 gate；DEGRADED 只是诚实输出，不能把 stalled 变成验收成功。

- [ ] **Step 5: 单元 2 验收**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -R 'unit.runtime|unit.frontends|unit.application|integration.replay_determinism'
```

- [ ] **Step 6: 提交检查点（仅经用户授权）**

```bash
git add include/application/replay_pipeline.hpp src/application/replay_pipeline.cpp \
  tests/application/replay_pipeline_test.cpp
git commit -m "fix: publish truthful replay timestamps and tracking state"
```

## 实施单元 3：不确定性、视差质量与融合门禁

### Task 9：让 RANSAC 返回质量摘要和 6x6 covariance

**Files:**
- Modify: `include/frontends/rigid_transform_fit.hpp`
- Modify: `src/frontends/rigid_transform_fit.cpp`
- Modify: `tests/frontends/rigid_transform_fit_test.cpp`
- Modify: `include/frontends/stereo_landmark_vo_frontend.hpp`
- Modify: `src/frontends/stereo_landmark_vo_frontend.cpp`
- Modify: `tests/frontends/stereo_landmark_vo_frontend_test.cpp`

- [ ] **Step 1: 定义结果类型并写红灯测试**

```cpp
struct RigidTransformFitResult {
  uw::sensor_models::Pose3 pose;
  std::size_t correspondence_count = 0;
  std::vector<std::size_t> inlier_indices;
  double inlier_ratio = 0.0;
  double inlier_rmse_m = 0.0;
  double normal_matrix_condition_number = 0.0;
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
};

struct CovarianceEstimationParams {
  double max_condition_number = 1.0e8;
  double residual_variance_floor_m2 = 1.0e-8;
  double singular_value_tolerance = 1.0e-10;
};
```

将 RANSAC API 改为返回 `optional<RigidTransformFitResult>`；纯 `FitRigidTransform()` 可保留 Pose3 API供最小样本内部使用。测试：有噪非退化点集返回有限对称正定 covariance、正确 inlier 数/RMSE；共线/近共线点集因秩或 condition number 返回 nullopt；相同 seed 返回逐项一致摘要。

- [ ] **Step 2: 确认红灯**

```bash
cmake --build build --target frontends_tests -j"$(nproc)"
build/bin/tests/frontends_tests --gtest_filter='*RigidTransformFitRansac*'
```

- [ ] **Step 3: 实现 minimal SE(3) 数值 Jacobian 与 SVD covariance**

对最终 inliers 的每个 3D residual `r_i = b_i - T.Apply(a_i)`，以 `[dt_x,dt_y,dt_z,dtheta_x,dtheta_y,dtheta_z]` 左扰动做中心差分，堆成 `3N x 6` Jacobian。SVD 得到 `J^T J` 的秩和 condition；

```cpp
sigma2 = std::max(residual_variance_floor_m2,
                  squared_error / std::max<std::ptrdiff_t>(1, 3 * N - 6));
covariance = sigma2 * V * singular_values_squared.cwiseInverse().asDiagonal() * V.transpose();
covariance = 0.5 * (covariance + covariance.transpose());
```

任一非有限、rank<6、condition 超阈值均返回 nullopt。

- [ ] **Step 4: VO 写 covariance 与 quality_features**

将 optical-frame covariance 通过 BODY/optical 的 SE(3) adjoint 变换到 `RelativePoseMeasurement` 的 body convention，再按 `[tx,ty,tz,rx,ry,rz]` row-major 写 36 项。写入：`correspondence_count`、`inlier_count`、`inlier_ratio`、`inlier_rmse_m`、`normal_matrix_condition_number`、`covariance_fallback=0`。

conditioning 失败必须调用 Task 7 的同一个 `RecordTrackingFailure()`：不输出 evidence、不推进 reference、计入 LOST 阈值。

- [ ] **Step 5: 配置 conditioning 阈值并回归**

把 `max_condition_number` 和 variance floor 放入 `visual_odometry` YAML，做 finite/range/unknown-key 验证，并由 replay 传入 RANSAC params。

```bash
cmake --build build --target frontends_tests runtime_tests -j"$(nproc)"
build/bin/tests/frontends_tests --gtest_filter='*RigidTransformFitRansac*:*StereoLandmarkVoFrontend*'
```

- [ ] **Step 6: 提交检查点（仅经用户授权）**

```bash
git add include/frontends/rigid_transform_fit.hpp src/frontends/rigid_transform_fit.cpp \
  tests/frontends/rigid_transform_fit_test.cpp include/frontends/stereo_landmark_vo_frontend.hpp \
  src/frontends/stereo_landmark_vo_frontend.cpp tests/frontends/stereo_landmark_vo_frontend_test.cpp \
  include/runtime/config.hpp src/runtime/config.cpp tests/runtime/config_test.cpp configs/defaults/platform.yaml
git commit -m "feat: propagate VO fit covariance and quality"
```

### Task 10：用完整 covariance 白化 RelativePose residual，并拆分平移/旋转 cap

**Files:**
- Modify: `schemas/proto/uw/domain/factor.proto`
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `configs/defaults/platform.yaml`
- Modify: `configs/README.md`
- Modify: `include/factor_builders/relative_pose_residual.hpp`
- Modify: `src/factor_builders/relative_pose_residual.cpp`
- Modify: `include/factor_builders/relative_pose_factor_builder.hpp`
- Modify: `src/factor_builders/relative_pose_factor_builder.cpp`
- Modify: `tests/factor_builders/relative_pose_residual_test.cpp`
- Create: `tests/factor_builders/relative_pose_factor_builder_test.cpp`
- Modify: `cmake/Tests.cmake`
- Modify: `src/application/replay_pipeline.cpp`

- [ ] **Step 1: 拆分配置并写兼容/非法值红灯测试**

```cpp
struct RelativePoseSqrtInformationCaps {
  double translation = 20.0;
  double rotation = 20.0;
};
struct SqrtInformationDefaults {
  RelativePoseSqrtInformationCaps relative_pose;
  double sonar_range = 15.0;
  double depth = 20.0;
};
```

YAML 迁移为：

```yaml
default_sqrt_information:
  relative_pose:
    translation: 20.0
    rotation: 20.0
  sonar_range: 15.0
  depth: 20.0
```

测试两个 cap 独立解析；零、负、NaN、未知键失败。旧 scalar `relative_pose: 20` 明确报错并提示新结构，避免静默误读。

- [ ] **Step 2: 写 residual 与 builder 红灯测试**

把 residual 构造改为：

```cpp
RelativePoseResidual(
    uw::sensor_models::Pose3 measured_relative_pose,
    Eigen::Matrix<double, 6, 6> sqrt_information);
```

测试非对角矩阵确实对白化后的 6D residual 产生耦合；零 residual 仍为零；解析 36 项合法 covariance 后 builder 产生有限 residual；缺失/非对称/非正定/NaN covariance 使用 cap 对角回退而不崩溃。

- [ ] **Step 3: 实现 covariance 到有上限 sqrt-information**

builder 验证并对称化 covariance，使用 `SelfAdjointEigenSolver` 构造 raw inverse square root。为同时保留完整 covariance 方向且满足不同单位 cap，令 `D=diag(t_cap,t_cap,t_cap,r_cap,r_cap,r_cap)`，对 `B=W_raw*D^{-1}` 做 SVD，将奇异值 clamp 到 1，再构造：

```cpp
W_capped = U * singular_values.cwiseMin(1.0).asDiagonal() * V.transpose() * D;
```

这样 `W_capped * D^{-1}` 的最大增益不超过 1，而不是把 covariance 粗暴退化成对角。payload covariance 非法或缺失时 `W_capped=D`。在 evidence `quality_features` 已有 fallback 标记时保留审计；builder 本身不修改 immutable evidence。

由于 `FactorCandidate.proposed_noise` 只有一个 wire scalar，不能单独表达 translation/rotation。builder 用构造参数接收两个 typed cap，同时仍把 candidate 的正值作为两者共同的兼容上限：

```cpp
RelativePoseFactorBuilder(double translation_cap, double rotation_cap);
```

具体为 `effective_translation_cap=min(config_translation_cap, candidate_cap)`、`effective_rotation_cap=min(config_rotation_cap, candidate_cap)`；candidate 非正时不再额外收紧。应用设置 `candidate.proposed_noise=max(config_translation_cap, config_rotation_cap)`，从而保持历史 tag 4 的“最大信息强度”语义，又不丢失两个 typed cap。避免修改跨层通用 context 只为一个 factor。

- [ ] **Step 4: 更新 schema 注释而不改 tag/type**

`FactorCandidate.proposed_noise` 保持 tag 4 和 double；注释改为“调用者允许的最大 sqrt-information/兼容回退提示，最终权重由 typed builder 结合 payload uncertainty 决定”。不改生成文件，正常 CMake build 自动生成。

- [ ] **Step 5: 定向回归**

```bash
cmake --build build --target factor_builders_tests runtime_tests application_tests -j"$(nproc)"
build/bin/tests/factor_builders_tests --gtest_filter='*RelativePose*'
build/bin/tests/runtime_tests --gtest_filter='*Config*'
```

- [ ] **Step 6: 提交检查点（仅经用户授权）**

```bash
git add schemas/proto/uw/domain/factor.proto include/runtime/config.hpp src/runtime/config.cpp \
  tests/runtime/config_test.cpp configs/defaults/platform.yaml configs/README.md \
  include/factor_builders/relative_pose_residual.hpp src/factor_builders/relative_pose_residual.cpp \
  include/factor_builders/relative_pose_factor_builder.hpp src/factor_builders/relative_pose_factor_builder.cpp \
  tests/factor_builders/relative_pose_residual_test.cpp \
  tests/factor_builders/relative_pose_factor_builder_test.cpp cmake/Tests.cmake \
  src/application/replay_pipeline.cpp
git commit -m "feat: whiten relative poses from capped covariance"
```

### Task 11：让 depth/sonar factor 消费 payload sigma，配置仅作上限/回退

**Files:**
- Modify: `src/factor_builders/depth_factor_builder.cpp`
- Modify: `src/factor_builders/sonar_range_factor_builder.cpp`
- Create: `tests/factor_builders/depth_factor_builder_test.cpp`
- Create: `tests/factor_builders/sonar_range_factor_builder_test.cpp`
- Modify: `cmake/Tests.cmake`
- Modify: `src/application/replay_pipeline.cpp`

- [ ] **Step 1: 写标量权重红灯测试**

每个 builder 覆盖：`sigma=0.2, cap=10` 得到 sqrt info 5；`sigma=0.01, cap=10` 被截为 10；sigma 为 0/负/NaN 时回退 cap；candidate cap 非正时回退 1.0。通过 residual 对单位误差的输出值验证权重，不暴露私有成员。

- [ ] **Step 2: 确认红灯**

```bash
cmake --build build --target factor_builders_tests -j"$(nproc)"
build/bin/tests/factor_builders_tests --gtest_filter='*DepthFactorBuilder*:*SonarRangeFactorBuilder*'
```

- [ ] **Step 3: 实现统一 scalar policy**

在 builder `.cpp` 的匿名 namespace 使用相同逻辑：

```cpp
double CappedSqrtInformation(double sigma, double configured_cap) {
  const double cap = std::isfinite(configured_cap) && configured_cap > 0.0
                         ? configured_cap : 1.0;
  if (!std::isfinite(sigma) || sigma <= 0.0) return cap;
  return std::min(cap, 1.0 / sigma);
}
```

Depth 使用 `PressureDepthMeasurement.sigma_m`，sonar range 使用 `SonarRangeBearing.range_sigma_m`。bearing sigma 不进入当前 range-only residual，并在 builder 注释中明确；不伪装成二维 bearing factor。

- [ ] **Step 4: 回归并提交检查点（仅经用户授权）**

```bash
cmake --build build --target factor_builders_tests -j"$(nproc)"
build/bin/tests/factor_builders_tests
git add src/factor_builders/depth_factor_builder.cpp src/factor_builders/sonar_range_factor_builder.cpp \
  tests/factor_builders/depth_factor_builder_test.cpp \
  tests/factor_builders/sonar_range_factor_builder_test.cpp cmake/Tests.cmake \
  src/application/replay_pipeline.cpp
git commit -m "feat: cap scalar factor information from sensor sigma"
```

### Task 12：补齐 BlockMatcher 的纹理、唯一性和左右一致性过滤

**Files:**
- Modify: `include/frontends/block_matcher.hpp`
- Modify: `src/frontends/block_matcher.cpp`
- Modify: `tests/frontends/block_matcher_test.cpp`
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `configs/defaults/platform.yaml`
- Modify: `src/application/replay_pipeline.cpp`

- [ ] **Step 1: 写四类红灯测试**

在保留 clean texture 成功用例基础上增加：完全平坦 block 因 variance invalid；重复周期纹理因 best/second-best margin invalid；左右遮挡/错误匹配因 LR inconsistency invalid；正确正 disparity 继续 valid。测试还覆盖 min/max disparity 边界。

- [ ] **Step 2: 扩展参数并确认红灯**

```cpp
struct BlockMatcherParams {
  // existing fields...
  double min_texture_variance = 25.0;
  double min_uniqueness_margin = 2.0;  // second-best mean SAD - best mean SAD
  double left_right_max_diff_px = 1.0;
};
```

```bash
cmake --build build --target frontends_tests -j"$(nproc)"
build/bin/tests/frontends_tests --gtest_filter='*BlockMatcher*'
```

- [ ] **Step 3: 实现单方向搜索 helper 和 LR 检查**

提取私有 `MatchOneDirection(reference, target, search_sign, u, v)`，一次遍历同时计算 block variance、best SAD、second-best SAD。左→右用 `u-d`；对候选右像素再做右→左 `u+d`，只有 `abs(d_lr-d_rl) <= threshold` 才 valid。不得通过递归调用 `Compute()`，否则重复完整图计算且边界语义难以控制。

过滤顺序固定为：纹理→搜索范围/正 disparity→max mean SAD→unique margin→LR consistency；测试断言 invalid 即可，不增加 wire-level reason mask。

同时把已有 clean-texture 测试的 `min_disparity=0` 改为 1；配置验证强制 `min_disparity>=1`、`max_disparity>=min_disparity`，使“正 disparity”不是只靠注释成立。

- [ ] **Step 4: 配置化并验证范围**

在 `stereo_matching` section 解析三个阈值：variance/margin 均 `>=0`，LR threshold `>=0` 且 finite，unknown key 拒绝。replay 用配置填充 `StereoOpticalDepthFrontendParams.matcher`，不得保留只适合 synthetic 的局部 hardcode。

- [ ] **Step 5: 回归并提交检查点（仅经用户授权）**

```bash
cmake --build build --target frontends_tests runtime_tests -j"$(nproc)"
build/bin/tests/frontends_tests --gtest_filter='*BlockMatcher*:*StereoOpticalDepthFrontend*'
build/bin/tests/runtime_tests --gtest_filter='*Config*'
git add include/frontends/block_matcher.hpp src/frontends/block_matcher.cpp \
  tests/frontends/block_matcher_test.cpp include/runtime/config.hpp src/runtime/config.cpp \
  tests/runtime/config_test.cpp configs/defaults/platform.yaml src/application/replay_pipeline.cpp
git commit -m "fix: reject ambiguous and low texture disparities"
```

### Task 13：区分 optical-only/acoustic-optic 地图贡献并增加非零 gate

**Files:**
- Modify: `include/application/replay_pipeline.hpp`
- Modify: `src/application/replay_pipeline.cpp`
- Modify: `tests/application/replay_pipeline_test.cpp`
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `configs/experiment/acoustic_optic_demo.yaml`
- Modify: `configs/experiment/synthetic_smoke.yaml`
- Modify: `configs/README.md`
- Modify: `tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh`

- [ ] **Step 1: 提取贡献统计和 gate helper，写红灯测试**

```cpp
struct MapContributionCounts {
  uint64_t optical_only_points = 0;
  uint64_t acoustic_optic_points = 0;
};

MapContributionCounts CountDepthContributions(
    const uw::domain::FusedDepthMeasurement& fused);
std::vector<std::string> EvaluateReplayGates(
    const uw::runtime::PlatformDefaultsConfig& defaults,
    const uw::estimation::SolverSummary& solver,
    const uw::evaluation::AteMetrics& ate,
    int landmarks,
    const MapContributionCounts& contributions,
    int accepted_associations);
```

测试：只有 optical-only 点时 `require_nonempty_map` 可通过，但 `min_acoustic_optic_accepted=1` 或 `min_acoustic_optic_map_points=1` 必须失败；存在 acoustic-optic contribution 时通过；solver stalled 始终产生 gate failure。

- [ ] **Step 2: 新增并验证 gate 配置**

```cpp
int min_acoustic_optic_accepted = 0;
int min_acoustic_optic_map_points = 0;
```

两个字段 `<=0` 表示禁用，正值表示最低计数；拒绝 YAML 类型错误和未知 gate key。只在明确期望可见声光目标的 `acoustic_optic_demo.yaml` 设置非零；默认 `synthetic_smoke.yaml` 的目标不在窄相机视场，保持这两个 gate 关闭并写明原因，不能为了过 gate 伪造 acceptance。

- [ ] **Step 3: 在 bridge 前按 contribution_mask 计数**

对每个 fused measurement 的 valid pixel 读取 `contribution_mask`，分别累计 optical-only 与 acoustic-optic；不要用 `map_evidence_points` 反推来源，因为 bridge 后来源信息已丢失。console summary 和 gate error 同时输出 accepted/ambiguous/conflict/rejected、两类点数和同步拒绝数。

- [ ] **Step 4: 强化场景矩阵集成断言**

对 `clean_textured`/规格中声明应当融合的场景断言 accepted 和 acoustic-optic contribution 都大于零；对 dropout/time-offset 场景断言 optical-only 继续产出且 rejection reason 正确。保持 determinism 的输出 diff。

- [ ] **Step 5: 单元 3 验收**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -R 'unit.frontends|unit.factor_builders|unit.runtime|unit.application|integration.acoustic_optic'
```

- [ ] **Step 6: 提交检查点（仅经用户授权）**

```bash
git add include/application/replay_pipeline.hpp src/application/replay_pipeline.cpp \
  tests/application/replay_pipeline_test.cpp include/runtime/config.hpp src/runtime/config.cpp \
  tests/runtime/config_test.cpp configs/experiment/acoustic_optic_demo.yaml \
  configs/experiment/synthetic_smoke.yaml configs/README.md \
  tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh
git commit -m "feat: gate replay on real acoustic optic contribution"
```

### Task 14：全链验证、文档收口与指标记录

**Files:**
- Modify: `README.md`
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Modify: `docs/archive/uw-slam-production-readiness-and-roadmap-2026-08-21.md`
- Modify: `configs/README.md`

- [ ] **Step 1: 先跑静态边界和格式检查**

```bash
tools/lint/check_layer_dependencies.py .
tools/lint/check_no_ros_in_core.sh
git diff --check
rg -n 'opencv2/|cv::' include src apps
```

Expected: 前三项成功；最后一项无输出。

- [ ] **Step 2: 完整 configure/build/test**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
(cd adapters/holoocean && .venv/bin/pytest -q)
```

Expected: 所有 CTest 和 HoloOcean adapter pytest 通过。

- [ ] **Step 3: 跑仓库完整验证脚本**

```bash
tools/verify_pipeline.sh --out-dir /tmp/uw_slam_frontend_correctness_verify
```

Expected: configure、build、ctest、pytest、lint、synthetic generate/replay 全部成功；`require_converged` 未被关闭。

- [ ] **Step 4: 跑两个有区分力的实验**

```bash
build/bin/synth_bag_gen \
  --experiment configs/experiment/synthetic_smoke_vo.yaml \
  --out /tmp/frontend_correctness_vo.mcap
build/bin/replay_demo \
  --bag /tmp/frontend_correctness_vo.mcap \
  --experiment configs/experiment/synthetic_smoke_vo.yaml \
  --out /tmp/frontend_correctness_vo

build/bin/synth_bag_gen \
  --experiment configs/experiment/acoustic_optic_demo.yaml \
  --out /tmp/frontend_correctness_fusion.mcap
build/bin/replay_demo \
  --bag /tmp/frontend_correctness_fusion.mcap \
  --experiment configs/experiment/acoustic_optic_demo.yaml \
  --out /tmp/frontend_correctness_fusion
```

Expected:

- VO 实验使用 rectified virtual frames，轨迹时间来自 raw capture time，solver converged；
- 声光 demo 的 accepted associations 与 acoustic-optic map points 均满足非零 gate；
- manifest 同时含 raw/derived calibration hash；
- 不以历史 0.0666m 当硬编码精确值，只要求配置 gate 和更新后的稳定基线范围。

- [ ] **Step 5: 更新文档为已实现事实**

README 和代码库参考文档更新数据流图：`raw camera -> opencv_adapters -> rectified bundle + derived rig -> VO/depth/fusion/map`。删除“general rectification 尚未接入”“固定 0.2 秒”“固定 isotropic relative weight”等过时描述；记录新的配置字段、状态规则、fallback 语义和两类地图贡献计数。路线图只勾选本计划实际由测试/运行证据证明的项；真实数据 solver stalled 若仍存在，必须保留为未完成 P1，不能因 DEGRADED 状态而关闭。

- [ ] **Step 6: 最终审查**

```bash
git status --short
git diff --stat
git diff --check
ctest --test-dir build -N
```

确认：没有修改 `external_repos/`；没有未注册的新测试；没有把 OpenCV 类型泄漏到 adapter 之外；没有关闭已有 gate；用户原有修改未被覆盖。

- [ ] **Step 7: 最终提交（仅经用户授权）**

```bash
git add README.md docs/uw-slam-codebase-reference-2026-08-18.md \
  docs/archive/uw-slam-production-readiness-and-roadmap-2026-08-21.md configs/README.md
git commit -m "docs: close frontend correctness implementation"
```

## 完成定义

只有同时满足以下条件，计划才算完成：

1. 任意支持的 plumb-bob、不同内参、非水平/非平行 stereo rig 在进入 frontend 前产生自洽的 rectified images + derived rig；raw/rectified 混用会被自动测试拒绝。
2. 同步器的 no-sonar、超时、非法时间和成功四种结果可区分，超时真实 delta 可审计，应用不存在失败后回填 0 秒的路径。
3. VO 单帧失败不推进参考；恢复 evidence 连接最后成功 keyframe；conditioning 失败走同一失败计数；超限状态为 LOST。
4. StateSnapshot/TUM 使用真实 capture time，状态不把 stalled 伪报 TRACKING，并填充 calibration version 与 contributing evidence IDs。
5. relative pose 使用 6x6 covariance 完整白化且 translation/rotation cap 分离；depth/sonar 使用 payload sigma，非法 uncertainty 有明确 cap 回退。
6. 平坦、重复纹理和 LR 不一致 disparity 均 invalid；正常纹理不退化。
7. 地图统计可区分 optical-only 和 acoustic-optic，期望融合实验具有非零 gate，solver stalled gate 保持非零。
8. 全量 CTest、Python adapter 测试、依赖 lint、完整 verify pipeline 和两个区分性 replay 全部通过，文档与实际行为一致。
