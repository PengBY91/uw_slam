# C++ Layout Flattening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Flatten the repository's C++ layout, centralize CMake and tests, and merge build targets along real architecture boundaries without changing runtime behavior.

**Architecture:** Hand-written C++ headers and sources move into shared top-level `include/<role>/` and `src/<role>/` roots. Production libraries become `domain`, `core`, `frontends`, `factor_builders`, `estimation`, `mapping`, `runtime`, `evaluation`, and `adapters`, referenced through `uw::...` aliases; ROS2 and Python remain isolated because they have optional or different build systems.

**Tech Stack:** C++17, CMake 3.22+, GoogleTest/CTest, Protobuf, Eigen3, yaml-cpp, MCAP, Python 3, ROS2 Jazzy (optional).

---

## Execution constraints

- Work in the current worktree: it contains valid uncommitted changes in `CMakeLists.txt`, replay/demo code, configuration documentation, camera conversion code, and the new `stereo_landmark_vo_frontend`. Do not create a clean worktree from `HEAD`, reset, restore, stash, or overwrite those changes.
- Do not edit `external_repos/`.
- Do not commit unless the user explicitly authorizes commits. The checkpoints below end with `git diff --check` and verification instead of automatic commit steps.
- Use fresh build directories under `/tmp` during migration. Do not delete `build/` or `build_ros2/`.
- Preserve copyright headers in the moved sonar CFAR and sonar range residual sources; update `NOTICE` paths after moving them.
- Treat `docs/superpowers/plans/` and older `docs/superpowers/specs/` as historical records. Update active documentation and the new layout spec, but do not rewrite old implementation plans to pretend they used the new paths.

## Final file map

### Shared public headers

```text
include/domain/               <- core/domain/include/uw/domain/
include/sensor_models/        <- core/sensor_models/include/uw/sensor_models/
include/measurement_api/      <- core/measurement_api/include/uw/measurement_api/
include/frontends/            <- algorithms/frontends/*/include/uw/frontends/
include/factor_builders/      <- algorithms/factor_builders/*/include/uw/factor_builders/
include/estimation/           <- algorithms/estimation/include/uw/estimation/
include/mapping/              <- algorithms/mapping/*/include/uw/mapping/
include/runtime/              <- runtime/include/uw/runtime/
include/evaluation/           <- evaluation/include/uw/evaluation/
include/adapters/             <- adapters/third_party/{svin_bridge,holoocean_ros_bridge}/include/uw/adapters/
```

### Shared implementations

```text
src/domain/                   <- core/domain/src/
src/sensor_models/            <- core/sensor_models/src/
src/frontends/                <- algorithms/frontends/*/src/
src/factor_builders/          <- algorithms/factor_builders/*/src/
src/estimation/               <- algorithms/estimation/src/
src/mapping/                  <- algorithms/mapping/*/src/
src/runtime/                  <- runtime/src/
src/evaluation/               <- evaluation/src/
src/adapters/                 <- adapters/third_party/{svin_bridge,holoocean_ros_bridge}/src/
```

### Isolated components

```text
adapters/ros2/include/adapters/       <- adapters/ros2/include/uw/adapters/
adapters/ros2/src/                    <- remains isolated
adapters/holoocean/                   <- unchanged Python package
baselines/sonar_camera_reconstruction/ <- adapters/third_party/sonar_camera_reconstruction_baseline/
schemas/                              <- unchanged schema source tree
```

### Tests and applications

```text
tests/core/                    <- core/sensor_models/test/
tests/frontends/               <- algorithms/frontends/*/test/
tests/factor_builders/         <- algorithms/factor_builders/*/test/
tests/estimation/              <- algorithms/estimation/test/
tests/mapping/                 <- algorithms/mapping/*/test/
tests/runtime/                 <- runtime/test/
tests/evaluation/              <- evaluation/test/
tests/adapters/                <- adapters/third_party/{svin_bridge,holoocean_ros_bridge}/test/
tests/contracts/               <- tests/l0_contracts/
tests/integration/             <- tests/l2_replay/
apps/replay_demo.cpp           <- apps/replay_demo/src/main.cpp
apps/synth_bag_gen.cpp         <- apps/tools/synth_bag_gen/src/main.cpp
apps/synth_stereo_gen.cpp      <- apps/tools/synth_stereo_gen/src/main.cpp
apps/optical_baseline_eval.cpp <- apps/tools/optical_baseline_eval/src/main.cpp
apps/acoustic_optic_scenario_matrix.cpp <- apps/tools/acoustic_optic_scenario_matrix/src/main.cpp
apps/acoustic_optic_scenarios.cpp       <- apps/tools/acoustic_optic_scenario_matrix/src/scenarios.cpp
apps/acoustic_optic_scenarios.hpp       <- apps/tools/acoustic_optic_scenario_matrix/src/scenarios.hpp
```

## Task 1: Capture a behavioral and worktree baseline

**Files:**
- Read: `CMakeLists.txt`
- Read: all currently modified/untracked files reported by Git
- Output only: `/tmp/uw_slam_layout_baseline/`

- [ ] **Step 1: Record the dirty worktree without changing it**

Run:

```bash
git status --short
git diff -- CMakeLists.txt apps/replay_demo/CMakeLists.txt apps/replay_demo/src/main.cpp \
  apps/tools/synth_bag_gen/src/main.cpp configs/README.md \
  core/measurement_api/include/uw/measurement_api/frontend.hpp
```

Expected: the known modified files and untracked stereo VO/camera conversion files are visible; no file is restored or removed.

- [ ] **Step 2: Configure a fresh baseline build**

Run:

```bash
mkdir -p /tmp/uw_slam_layout_baseline
PATH="/home/steve/miniconda3/envs/uw_slam_build/bin:$PATH" \
  cmake -S . -B /tmp/uw_slam_layout_baseline/build \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
```

Expected: configuration succeeds with `UW_BUILD_ROS2=OFF` and registers the current stereo landmark VO target.

- [ ] **Step 3: Build and run the current C++ verification surface**

Run:

```bash
cmake --build /tmp/uw_slam_layout_baseline/build -j"$(nproc)"
ctest --test-dir /tmp/uw_slam_layout_baseline/build --output-on-failure
tools/lint/check_no_ros_in_core.sh
```

Expected: build succeeds, all currently registered CTest tests pass, and the dependency lint exits zero. Record the actual CTest count rather than assuming the README count is current.

- [ ] **Step 4: Run Python tests and the end-to-end smoke path**

Run:

```bash
(cd adapters/holoocean && .venv/bin/pytest -q)
tools/verify_pipeline.sh \
  --build-dir /tmp/uw_slam_layout_baseline/pipeline-build \
  --out-dir /tmp/uw_slam_layout_baseline/pipeline
```

Expected: Python tests pass, `summary.txt` ends with `RESULT: PASSED`, and replay emits an `ATE:` line. If the baseline fails, stop and diagnose it before structural edits; do not redefine a pre-existing failure as a refactor failure.

## Task 2: Replace distributed CMake with the centralized target graph

**Files:**
- Create: `cmake/Dependencies.cmake`
- Create: `cmake/Libraries.cmake`
- Create: `cmake/Applications.cmake`
- Create: `cmake/Tests.cmake`
- Modify: `cmake/UwProtobuf.cmake`
- Modify: `CMakeLists.txt`
- Read but leave temporarily: every existing child `CMakeLists.txt`

- [ ] **Step 1: Rename the generated protobuf target**

In `cmake/UwProtobuf.cmake`, replace the target declaration and all target property calls with:

```cmake
add_library(domain_proto STATIC ${UW_DOMAIN_PROTO_GENERATED})
target_include_directories(domain_proto SYSTEM PUBLIC ${UW_PROTO_GEN_DIR})
target_link_libraries(domain_proto PUBLIC protobuf::libprotobuf)
target_link_libraries(domain_proto PUBLIC
  absl::flat_hash_map absl::hash absl::strings absl::status absl::statusor
  absl::synchronization absl::time absl::base absl::log absl::cord
)
set_target_properties(domain_proto PROPERTIES COMPILE_OPTIONS "-w")
```

Run:

```bash
rg -n 'uw_domain_proto' cmake/UwProtobuf.cmake
```

Expected: no matches.

- [ ] **Step 2: Create centralized dependency discovery**

Create `cmake/Dependencies.cmake` with:

```cmake
option(UW_BUILD_ROS2 "Build the ROS2 adapter (requires a sourced ROS2 install)" OFF)
option(UW_BUILD_TESTS "Build the contract, unit, and integration tests" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

include(UwProtobuf)
include(UwMcap)

if(UW_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)
endif()

if(UW_BUILD_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(holoocean_interfaces REQUIRED)
else()
  message(STATUS "UW_BUILD_ROS2=OFF: skipping ROS2 adapters")
endif()
```

- [ ] **Step 3: Create the architecture-layer libraries against current paths**

Create `cmake/Libraries.cmake` with explicit source lists:

```cmake
function(uw_apply_library_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
endfunction()

add_library(domain STATIC core/domain/src/domain.cpp)
add_library(uw::domain ALIAS domain)
target_include_directories(domain PUBLIC core/domain/include)
target_link_libraries(domain PUBLIC domain_proto)
uw_apply_library_defaults(domain)

add_library(core STATIC
  core/sensor_models/src/geometry.cpp
  core/sensor_models/src/sonar_beam_model.cpp
  core/sensor_models/src/camera_model.cpp
  core/sensor_models/src/sonar_arc_projector.cpp
)
add_library(uw::core ALIAS core)
target_include_directories(core PUBLIC
  core/sensor_models/include
  core/measurement_api/include
)
target_link_libraries(core PUBLIC uw::domain Eigen3::Eigen)
uw_apply_library_defaults(core)

add_library(frontends STATIC
  algorithms/frontends/sonar_cfar_frontend/src/cfar_detector.cpp
  algorithms/frontends/sonar_cfar_frontend/src/dbscan.cpp
  algorithms/frontends/sonar_cfar_frontend/src/sonar_cfar_frontend.cpp
  algorithms/frontends/stereo_optical_depth_frontend/src/block_matcher.cpp
  algorithms/frontends/stereo_optical_depth_frontend/src/stereo_optical_depth_frontend.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/src/landmark_blob_detector.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/src/patch_matcher.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/src/rigid_transform_fit.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/src/stereo_landmark_vo_frontend.cpp
  algorithms/frontends/acoustic_optic_associator/src/acoustic_optic_associator.cpp
  algorithms/frontends/acoustic_optic_depth_fusion/src/posterior_depth_optimizer.cpp
  algorithms/frontends/acoustic_optic_depth_fusion/src/acoustic_optic_depth_fusion_frontend.cpp
)
add_library(uw::frontends ALIAS frontends)
target_include_directories(frontends PUBLIC
  algorithms/frontends/sonar_cfar_frontend/include
  algorithms/frontends/stereo_optical_depth_frontend/include
  algorithms/frontends/stereo_landmark_vo_frontend/include
  algorithms/frontends/acoustic_optic_associator/include
  algorithms/frontends/acoustic_optic_depth_fusion/include
)
target_link_libraries(frontends PUBLIC uw::core)
uw_apply_library_defaults(frontends)

add_library(factor_builders STATIC
  algorithms/factor_builders/relative_pose_factor/src/relative_pose_residual.cpp
  algorithms/factor_builders/relative_pose_factor/src/relative_pose_factor_builder.cpp
  algorithms/factor_builders/depth_factor/src/depth_residual.cpp
  algorithms/factor_builders/depth_factor/src/depth_factor_builder.cpp
  algorithms/factor_builders/sonar_range_factor/src/sonar_range_residual.cpp
  algorithms/factor_builders/sonar_range_factor/src/sonar_range_factor_builder.cpp
)
add_library(uw::factor_builders ALIAS factor_builders)
target_include_directories(factor_builders PUBLIC
  algorithms/factor_builders/relative_pose_factor/include
  algorithms/factor_builders/depth_factor/include
  algorithms/factor_builders/sonar_range_factor/include
)
target_link_libraries(factor_builders PUBLIC uw::core)
uw_apply_library_defaults(factor_builders)

add_library(estimation STATIC
  algorithms/estimation/src/state_store.cpp
  algorithms/estimation/src/pose_graph_problem.cpp
  algorithms/estimation/src/gauss_newton_solver.cpp
)
add_library(uw::estimation ALIAS estimation)
target_include_directories(estimation PUBLIC algorithms/estimation/include)
target_link_libraries(estimation PUBLIC uw::core Eigen3::Eigen)
uw_apply_library_defaults(estimation)

add_library(mapping STATIC
  algorithms/mapping/submap_manager/src/submap_manager.cpp
  algorithms/mapping/acoustic_optic_map_bridge/src/acoustic_optic_map_bridge.cpp
)
add_library(uw::mapping ALIAS mapping)
target_include_directories(mapping PUBLIC
  algorithms/mapping/submap_manager/include
  algorithms/mapping/acoustic_optic_map_bridge/include
)
target_link_libraries(mapping PUBLIC uw::core)
uw_apply_library_defaults(mapping)

add_library(runtime STATIC
  runtime/src/run_manifest.cpp
  runtime/src/mcap_io.cpp
  runtime/src/config.cpp
  runtime/src/acoustic_optic_synchronizer.cpp
)
add_library(uw::runtime ALIAS runtime)
target_include_directories(runtime PUBLIC runtime/include)
target_link_libraries(runtime PUBLIC
  uw::core mcap_impl protobuf::libprotobuf yaml-cpp::yaml-cpp Eigen3::Eigen
)
uw_apply_library_defaults(runtime)

add_library(evaluation STATIC
  evaluation/src/trajectory_metrics.cpp
  evaluation/src/depth_metrics.cpp
  evaluation/src/fusion_metrics.cpp
)
add_library(uw::evaluation ALIAS evaluation)
target_include_directories(evaluation PUBLIC evaluation/include)
target_link_libraries(evaluation PUBLIC uw::core)
uw_apply_library_defaults(evaluation)

add_library(adapters STATIC
  adapters/third_party/svin_bridge/src/svin_bridge_local_odometry_provider.cpp
  adapters/third_party/holoocean_ros_bridge/src/holoocean_ros_bridge_sonar_frame_provider.cpp
)
add_library(uw::adapters ALIAS adapters)
target_include_directories(adapters PUBLIC
  adapters/third_party/svin_bridge/include
  adapters/third_party/holoocean_ros_bridge/include
)
target_link_libraries(adapters PUBLIC uw::core)
uw_apply_library_defaults(adapters)

if(UW_BUILD_ROS2)
  add_library(ros2_adapters INTERFACE)
  add_library(uw::ros2_adapters ALIAS ros2_adapters)
  target_include_directories(ros2_adapters INTERFACE adapters/ros2/include)
  target_link_libraries(ros2_adapters INTERFACE
    uw::adapters rclcpp::rclcpp ${nav_msgs_TARGETS} ${holoocean_interfaces_TARGETS}
  )
endif()
```

- [ ] **Step 4: Create centralized application declarations against current paths**

Create `cmake/Applications.cmake` with:

```cmake
function(uw_apply_application_defaults target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )
endfunction()

add_executable(replay_demo apps/replay_demo/src/main.cpp)
target_link_libraries(replay_demo PRIVATE
  uw::domain uw::core uw::runtime uw::estimation uw::evaluation
  uw::factor_builders uw::mapping uw::frontends
)
uw_apply_application_defaults(replay_demo)

add_executable(synth_bag_gen apps/tools/synth_bag_gen/src/main.cpp)
target_link_libraries(synth_bag_gen PRIVATE uw::domain uw::core uw::runtime)
uw_apply_application_defaults(synth_bag_gen)

add_executable(synth_stereo_gen apps/tools/synth_stereo_gen/src/main.cpp)
target_link_libraries(synth_stereo_gen PRIVATE uw::domain uw::runtime)
uw_apply_application_defaults(synth_stereo_gen)

add_executable(optical_baseline_eval apps/tools/optical_baseline_eval/src/main.cpp)
target_link_libraries(optical_baseline_eval PRIVATE
  uw::domain uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(optical_baseline_eval)

add_executable(acoustic_optic_scenario_matrix
  apps/tools/acoustic_optic_scenario_matrix/src/main.cpp
  apps/tools/acoustic_optic_scenario_matrix/src/scenarios.cpp
)
target_include_directories(acoustic_optic_scenario_matrix PRIVATE
  apps/tools/acoustic_optic_scenario_matrix/src
)
target_link_libraries(acoustic_optic_scenario_matrix PRIVATE
  uw::domain uw::core uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(acoustic_optic_scenario_matrix)

if(UW_BUILD_ROS2)
  add_executable(holoocean_sonar_bridge_node adapters/ros2/src/holoocean_sonar_bridge_main.cpp)
  target_link_libraries(holoocean_sonar_bridge_node PRIVATE uw::ros2_adapters)
  uw_apply_application_defaults(holoocean_sonar_bridge_node)
endif()
```

- [ ] **Step 5: Create centralized tests against current paths**

Create `cmake/Tests.cmake` with:

```cmake
include(GoogleTest)

function(uw_register_gtest target prefix labels)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
  )
  gtest_discover_tests(${target}
    TEST_PREFIX "${prefix}."
    PROPERTIES LABELS "${labels}"
  )
endfunction()

add_executable(core_tests
  core/sensor_models/test/camera_model_test.cpp
  core/sensor_models/test/sonar_arc_projector_test.cpp
)
target_link_libraries(core_tests PRIVATE uw::core GTest::gtest GTest::gtest_main)
uw_register_gtest(core_tests "unit.core" "unit;core")

add_executable(frontends_tests
  algorithms/frontends/sonar_cfar_frontend/test/cfar_detector_test.cpp
  algorithms/frontends/sonar_cfar_frontend/test/sonar_cfar_frontend_test.cpp
  algorithms/frontends/stereo_optical_depth_frontend/test/block_matcher_test.cpp
  algorithms/frontends/stereo_optical_depth_frontend/test/stereo_optical_depth_frontend_test.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/test/landmark_blob_detector_test.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/test/patch_matcher_test.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/test/rigid_transform_fit_test.cpp
  algorithms/frontends/stereo_landmark_vo_frontend/test/stereo_landmark_vo_frontend_test.cpp
  algorithms/frontends/acoustic_optic_associator/test/acoustic_optic_associator_test.cpp
  algorithms/frontends/acoustic_optic_depth_fusion/test/posterior_depth_optimizer_test.cpp
  algorithms/frontends/acoustic_optic_depth_fusion/test/acoustic_optic_depth_fusion_frontend_test.cpp
)
target_link_libraries(frontends_tests PRIVATE uw::frontends GTest::gtest GTest::gtest_main)
uw_register_gtest(frontends_tests "unit.frontends" "unit;frontends")

add_executable(factor_builders_tests
  algorithms/factor_builders/relative_pose_factor/test/relative_pose_residual_test.cpp
  algorithms/factor_builders/depth_factor/test/depth_residual_test.cpp
  algorithms/factor_builders/sonar_range_factor/test/sonar_range_residual_test.cpp
)
target_link_libraries(factor_builders_tests PRIVATE
  uw::factor_builders GTest::gtest GTest::gtest_main
)
uw_register_gtest(factor_builders_tests "unit.factor_builders" "unit;factor_builders")

add_executable(estimation_tests algorithms/estimation/test/pose_graph_solver_test.cpp)
target_link_libraries(estimation_tests PRIVATE
  uw::estimation uw::factor_builders GTest::gtest GTest::gtest_main
)
uw_register_gtest(estimation_tests "unit.estimation" "unit;estimation")

add_executable(mapping_tests
  algorithms/mapping/submap_manager/test/submap_manager_test.cpp
  algorithms/mapping/acoustic_optic_map_bridge/test/acoustic_optic_map_bridge_test.cpp
)
target_link_libraries(mapping_tests PRIVATE uw::mapping GTest::gtest GTest::gtest_main)
uw_register_gtest(mapping_tests "unit.mapping" "unit;mapping")

add_executable(runtime_tests
  runtime/test/runtime_test.cpp
  runtime/test/mcap_io_test.cpp
  runtime/test/config_test.cpp
  runtime/test/acoustic_optic_synchronizer_test.cpp
)
target_compile_definitions(runtime_tests PRIVATE UW_REPO_ROOT="${PROJECT_SOURCE_DIR}")
target_link_libraries(runtime_tests PRIVATE
  uw::runtime GTest::gtest GTest::gtest_main Threads::Threads
)
uw_register_gtest(runtime_tests "unit.runtime" "unit;runtime")

add_executable(evaluation_tests
  evaluation/test/trajectory_metrics_test.cpp
  evaluation/test/depth_metrics_test.cpp
  evaluation/test/fusion_metrics_test.cpp
)
target_link_libraries(evaluation_tests PRIVATE uw::evaluation GTest::gtest GTest::gtest_main)
uw_register_gtest(evaluation_tests "unit.evaluation" "unit;evaluation")

add_executable(adapters_tests
  adapters/third_party/svin_bridge/test/svin_bridge_test.cpp
  adapters/third_party/holoocean_ros_bridge/test/holoocean_ros_bridge_sonar_frame_provider_test.cpp
)
target_link_libraries(adapters_tests PRIVATE
  uw::adapters GTest::gtest GTest::gtest_main Threads::Threads
)
uw_register_gtest(adapters_tests "unit.adapters" "unit;adapters")

add_executable(contract_tests
  tests/l0_contracts/domain_contract_test.cpp
  tests/l0_contracts/measurement_api_contract_test.cpp
)
target_link_libraries(contract_tests PRIVATE uw::domain uw::core GTest::gtest GTest::gtest_main)
uw_register_gtest(contract_tests "contract" "contract")

add_test(
  NAME integration.replay_determinism
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/l2_replay/determinism_test.sh
          $<TARGET_FILE:synth_bag_gen> $<TARGET_FILE:replay_demo>
)
set_tests_properties(integration.replay_determinism PROPERTIES LABELS "integration;replay")

add_test(
  NAME integration.optical_baseline_smoke
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/l2_replay/optical_baseline_smoke_test.sh
          $<TARGET_FILE:synth_stereo_gen> $<TARGET_FILE:optical_baseline_eval>
          ${PROJECT_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)
set_tests_properties(integration.optical_baseline_smoke PROPERTIES LABELS "integration;replay")

add_test(
  NAME integration.acoustic_optic_scenario_matrix_determinism
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/l2_replay/acoustic_optic_scenario_matrix_determinism_test.sh
          $<TARGET_FILE:acoustic_optic_scenario_matrix>
          ${PROJECT_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)
set_tests_properties(integration.acoustic_optic_scenario_matrix_determinism
  PROPERTIES LABELS "integration;replay")
```

- [ ] **Step 6: Replace the root build entry point**

Replace `CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.22)
project(uw_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(cmake/Dependencies.cmake)
include(cmake/Libraries.cmake)
include(cmake/Applications.cmake)

if(UW_BUILD_TESTS)
  include(cmake/Tests.cmake)
endif()
```

- [ ] **Step 7: Verify the centralized graph before moving files**

Run:

```bash
PATH="/home/steve/miniconda3/envs/uw_slam_build/bin:$PATH" \
  cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
cmake --build /tmp/uw_slam_layout_cmake -j"$(nproc)"
ctest --test-dir /tmp/uw_slam_layout_cmake --output-on-failure
cmake --build /tmp/uw_slam_layout_cmake --target help | \
  rg '(^| )(domain|core|frontends|factor_builders|estimation|mapping|runtime|evaluation|adapters)($| )'
```

Expected: configure/build/test pass; architecture-layer targets exist; old per-implementation targets are no longer declared even though their old CMake files still exist but are not loaded.

- [ ] **Step 8: Check the CMake-only delta**

Run:

```bash
git diff --check
git status --short
```

Expected: centralized CMake files and the protobuf target rename are visible; pre-existing user changes remain present.

## Task 3: Move domain, sensor models, and measurement APIs

**Files:**
- Move: `core/domain/include/uw/domain/*.hpp` -> `include/domain/`
- Move: `core/domain/src/*.cpp` -> `src/domain/`
- Move: `core/sensor_models/include/uw/sensor_models/*.hpp` -> `include/sensor_models/`
- Move: `core/sensor_models/src/*.cpp` -> `src/sensor_models/`
- Move: `core/measurement_api/include/uw/measurement_api/*.hpp` -> `include/measurement_api/`
- Modify: all C++ files that include hand-written domain, sensor model, or measurement API headers
- Modify: `cmake/Libraries.cmake`

- [ ] **Step 1: Create the shared roots and move foundational files**

Create these directories and move the files without changing contents:

```text
include/domain/
include/sensor_models/
include/measurement_api/
src/domain/
src/sensor_models/
```

Expected mappings include:

```text
core/domain/include/uw/domain/domain.hpp -> include/domain/domain.hpp
core/domain/src/domain.cpp -> src/domain/domain.cpp
core/sensor_models/include/uw/sensor_models/camera_model.hpp -> include/sensor_models/camera_model.hpp
core/measurement_api/include/uw/measurement_api/frontend.hpp -> include/measurement_api/frontend.hpp
```

The modified `frontend.hpp` must arrive byte-for-byte except for include-path edits; its newly added visual odometry interface must remain.

- [ ] **Step 2: Rewrite only hand-written foundational include prefixes**

Apply these literal mappings across `include/`, `src/`, the still-unmoved algorithms/runtime/evaluation/adapters/apps/tests trees:

```text
"uw/domain/domain.hpp"                  -> "domain/domain.hpp"
"uw/sensor_models/                      -> "sensor_models/
"uw/measurement_api/                    -> "measurement_api/
```

Do not rewrite generated protobuf includes such as:

```cpp
#include "uw/domain/calibration.pb.h"
#include "uw/domain/measurement.pb.h"
```

- [ ] **Step 3: Point `domain` and `core` at the shared roots**

In `cmake/Libraries.cmake`, make the foundational blocks exactly:

```cmake
add_library(domain STATIC src/domain/domain.cpp)
add_library(uw::domain ALIAS domain)
target_include_directories(domain PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(domain PUBLIC domain_proto)
uw_apply_library_defaults(domain)

add_library(core STATIC
  src/sensor_models/geometry.cpp
  src/sensor_models/sonar_beam_model.cpp
  src/sensor_models/camera_model.cpp
  src/sensor_models/sonar_arc_projector.cpp
)
add_library(uw::core ALIAS core)
target_include_directories(core PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(core PUBLIC uw::domain Eigen3::Eigen)
uw_apply_library_defaults(core)
```

- [ ] **Step 4: Build the foundational and downstream targets**

Run:

```bash
cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
cmake --build /tmp/uw_slam_layout_cmake \
  --target domain core frontends factor_builders runtime -j"$(nproc)"
```

Expected: all listed targets build; no missing hand-written `uw/domain`, `uw/sensor_models`, or `uw/measurement_api` header errors occur.

- [ ] **Step 5: Verify generated protobuf paths are the only foundational `uw/` includes**

Run:

```bash
rg -n '#include [<"]uw/(domain/domain\.hpp|sensor_models|measurement_api)' \
  include src core algorithms runtime evaluation adapters apps tests || true
```

Expected: no matches. Generated `uw/domain/*.pb.h` matches are allowed and must remain.

## Task 4: Move and merge all algorithm code

**Files:**
- Move: `algorithms/frontends/*/include/uw/frontends/*.hpp` -> `include/frontends/`
- Move: `algorithms/frontends/*/src/*.cpp` -> `src/frontends/`
- Move: `algorithms/factor_builders/*/include/uw/factor_builders/*.hpp` -> `include/factor_builders/`
- Move: `algorithms/factor_builders/*/src/*.cpp` -> `src/factor_builders/`
- Move: `algorithms/estimation/include/uw/estimation/*.hpp` -> `include/estimation/`
- Move: `algorithms/estimation/src/*.cpp` -> `src/estimation/`
- Move: `algorithms/mapping/*/include/uw/mapping/*.hpp` -> `include/mapping/`
- Move: `algorithms/mapping/*/src/*.cpp` -> `src/mapping/`
- Modify: all C++ include directives for those hand-written headers
- Modify: `cmake/Libraries.cmake`

- [ ] **Step 1: Move frontend headers and implementations into the shared roots**

Move every header and implementation from all five current frontend directories, including the untracked stereo landmark VO files, into:

```text
include/frontends/
src/frontends/
```

Expected: filenames are unique; `landmark_blob_detector.*`, `patch_matcher.*`, `rigid_transform_fit.*`, and `stereo_landmark_vo_frontend.*` are all present after the move.

- [ ] **Step 2: Move factor builder, estimation, and mapping code**

Move all files into:

```text
include/factor_builders/  src/factor_builders/
include/estimation/       src/estimation/
include/mapping/          src/mapping/
```

Preserve the original copyright headers in `sonar_range_residual.hpp` and `sonar_range_residual.cpp`.

- [ ] **Step 3: Rewrite algorithm include prefixes**

Apply these exact literal prefix mappings across all C++ sources and tests:

```text
"uw/frontends/       -> "frontends/
"uw/factor_builders/ -> "factor_builders/
"uw/estimation/      -> "estimation/
"uw/mapping/         -> "mapping/
```

Expected: namespace declarations remain `uw::frontends`, `uw::factor_builders`, `uw::estimation`, and `uw::mapping`.

- [ ] **Step 4: Replace the four algorithm source blocks in `Libraries.cmake`**

Use these final source lists and a single public include root for each target:

```cmake
add_library(frontends STATIC
  src/frontends/cfar_detector.cpp
  src/frontends/dbscan.cpp
  src/frontends/sonar_cfar_frontend.cpp
  src/frontends/block_matcher.cpp
  src/frontends/stereo_optical_depth_frontend.cpp
  src/frontends/landmark_blob_detector.cpp
  src/frontends/patch_matcher.cpp
  src/frontends/rigid_transform_fit.cpp
  src/frontends/stereo_landmark_vo_frontend.cpp
  src/frontends/acoustic_optic_associator.cpp
  src/frontends/posterior_depth_optimizer.cpp
  src/frontends/acoustic_optic_depth_fusion_frontend.cpp
)
add_library(uw::frontends ALIAS frontends)
target_include_directories(frontends PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(frontends PUBLIC uw::core)
uw_apply_library_defaults(frontends)

add_library(factor_builders STATIC
  src/factor_builders/relative_pose_residual.cpp
  src/factor_builders/relative_pose_factor_builder.cpp
  src/factor_builders/depth_residual.cpp
  src/factor_builders/depth_factor_builder.cpp
  src/factor_builders/sonar_range_residual.cpp
  src/factor_builders/sonar_range_factor_builder.cpp
)
add_library(uw::factor_builders ALIAS factor_builders)
target_include_directories(factor_builders PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(factor_builders PUBLIC uw::core)
uw_apply_library_defaults(factor_builders)

add_library(estimation STATIC
  src/estimation/state_store.cpp
  src/estimation/pose_graph_problem.cpp
  src/estimation/gauss_newton_solver.cpp
)
add_library(uw::estimation ALIAS estimation)
target_include_directories(estimation PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(estimation PUBLIC uw::core Eigen3::Eigen)
uw_apply_library_defaults(estimation)

add_library(mapping STATIC
  src/mapping/submap_manager.cpp
  src/mapping/acoustic_optic_map_bridge.cpp
)
add_library(uw::mapping ALIAS mapping)
target_include_directories(mapping PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(mapping PUBLIC uw::core)
uw_apply_library_defaults(mapping)
```

- [ ] **Step 5: Build every algorithm layer and its still-unmoved tests**

Run:

```bash
cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
cmake --build /tmp/uw_slam_layout_cmake \
  --target frontends factor_builders estimation mapping \
           frontends_tests factor_builders_tests estimation_tests mapping_tests \
  -j"$(nproc)"
ctest --test-dir /tmp/uw_slam_layout_cmake \
  -L unit --output-on-failure
```

Expected: all four layer libraries and test executables build, and all discovered unit cases pass.

## Task 5: Move runtime, evaluation, portable adapters, and ROS2 headers

**Files:**
- Move: `runtime/include/uw/runtime/*.hpp` -> `include/runtime/`
- Move: `runtime/src/*.cpp` -> `src/runtime/`
- Move: `evaluation/include/uw/evaluation/*.hpp` -> `include/evaluation/`
- Move: `evaluation/src/*.cpp` -> `src/evaluation/`
- Move: portable bridge headers -> `include/adapters/`
- Move: portable bridge sources -> `src/adapters/`
- Move: `adapters/ros2/include/uw/adapters/*.hpp` -> `adapters/ros2/include/adapters/`
- Modify: `cmake/Libraries.cmake`
- Modify: relevant C++ includes

- [ ] **Step 1: Move runtime and evaluation production files**

Move all runtime/evaluation public headers and implementations to:

```text
include/runtime/    src/runtime/
include/evaluation/ src/evaluation/
```

Then rewrite:

```text
"uw/runtime/    -> "runtime/
"uw/evaluation/ -> "evaluation/
```

- [ ] **Step 2: Merge the two portable providers into shared adapters**

Move:

```text
svin_bridge_local_odometry_provider.hpp             -> include/adapters/
holoocean_ros_bridge_sonar_frame_provider.hpp       -> include/adapters/
svin_bridge_local_odometry_provider.cpp             -> src/adapters/
holoocean_ros_bridge_sonar_frame_provider.cpp       -> src/adapters/
```

Rewrite hand-written includes from `"uw/adapters/` to `"adapters/` in portable adapters, ROS2 code, tests, and consumers.

- [ ] **Step 3: Keep ROS2's include root physically isolated**

Move the two ROS2 headers to:

```text
adapters/ros2/include/adapters/ros2_svin_odometry_bridge.hpp
adapters/ros2/include/adapters/ros2_holoocean_sonar_bridge.hpp
```

Do not move `adapters/ros2/src/holoocean_sonar_bridge_main.cpp` into shared `src/`; it remains part of the optional ROS2 deployment boundary.

- [ ] **Step 4: Point the four target blocks at final paths**

Replace their definitions with:

```cmake
add_library(runtime STATIC
  src/runtime/run_manifest.cpp
  src/runtime/mcap_io.cpp
  src/runtime/config.cpp
  src/runtime/acoustic_optic_synchronizer.cpp
)
add_library(uw::runtime ALIAS runtime)
target_include_directories(runtime PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(runtime PUBLIC
  uw::core mcap_impl protobuf::libprotobuf yaml-cpp::yaml-cpp Eigen3::Eigen
)
uw_apply_library_defaults(runtime)

add_library(evaluation STATIC
  src/evaluation/trajectory_metrics.cpp
  src/evaluation/depth_metrics.cpp
  src/evaluation/fusion_metrics.cpp
)
add_library(uw::evaluation ALIAS evaluation)
target_include_directories(evaluation PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(evaluation PUBLIC uw::core)
uw_apply_library_defaults(evaluation)

add_library(adapters STATIC
  src/adapters/svin_bridge_local_odometry_provider.cpp
  src/adapters/holoocean_ros_bridge_sonar_frame_provider.cpp
)
add_library(uw::adapters ALIAS adapters)
target_include_directories(adapters PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(adapters PUBLIC uw::core)
uw_apply_library_defaults(adapters)

if(UW_BUILD_ROS2)
  add_library(ros2_adapters INTERFACE)
  add_library(uw::ros2_adapters ALIAS ros2_adapters)
  target_include_directories(ros2_adapters INTERFACE adapters/ros2/include)
  target_link_libraries(ros2_adapters INTERFACE
    uw::adapters rclcpp::rclcpp ${nav_msgs_TARGETS} ${holoocean_interfaces_TARGETS}
  )
endif()
```

- [ ] **Step 5: Build portable layers and tests**

Run:

```bash
cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
cmake --build /tmp/uw_slam_layout_cmake \
  --target runtime evaluation adapters runtime_tests evaluation_tests adapters_tests \
  -j"$(nproc)"
ctest --test-dir /tmp/uw_slam_layout_cmake \
  -R '^(unit\.runtime|unit\.evaluation|unit\.adapters)' --output-on-failure
```

Expected: targets build and all selected cases pass without ROS2 being installed or enabled.

## Task 6: Flatten application sources and standardize output paths

**Files:**
- Move/rename: all files listed under “Tests and applications” above
- Modify: `cmake/Applications.cmake`
- Modify: `apps/acoustic_optic_scenario_matrix.cpp`
- Modify: `apps/acoustic_optic_scenarios.cpp`

- [ ] **Step 1: Move the five executable entry points**

Move each old `main.cpp` to its explicit final filename:

```text
apps/replay_demo.cpp
apps/synth_bag_gen.cpp
apps/synth_stereo_gen.cpp
apps/optical_baseline_eval.cpp
apps/acoustic_optic_scenario_matrix.cpp
```

Expected: the modified replay and synthetic bag generator files retain all current worktree changes.

- [ ] **Step 2: Move and rename scenario matrix helpers**

Move:

```text
scenarios.cpp -> apps/acoustic_optic_scenarios.cpp
scenarios.hpp -> apps/acoustic_optic_scenarios.hpp
```

Replace both occurrences of:

```cpp
#include "scenarios.hpp"
```

with:

```cpp
#include "acoustic_optic_scenarios.hpp"
```

- [ ] **Step 3: Replace application source paths**

Replace `cmake/Applications.cmake` with this final content:

```cmake
function(uw_apply_application_defaults target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )
endfunction()

add_executable(replay_demo apps/replay_demo.cpp)
target_link_libraries(replay_demo PRIVATE
  uw::domain uw::core uw::runtime uw::estimation uw::evaluation
  uw::factor_builders uw::mapping uw::frontends
)
uw_apply_application_defaults(replay_demo)

add_executable(synth_bag_gen apps/synth_bag_gen.cpp)
target_link_libraries(synth_bag_gen PRIVATE uw::domain uw::core uw::runtime)
uw_apply_application_defaults(synth_bag_gen)

add_executable(synth_stereo_gen apps/synth_stereo_gen.cpp)
target_link_libraries(synth_stereo_gen PRIVATE uw::domain uw::runtime)
uw_apply_application_defaults(synth_stereo_gen)

add_executable(optical_baseline_eval apps/optical_baseline_eval.cpp)
target_link_libraries(optical_baseline_eval PRIVATE
  uw::domain uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(optical_baseline_eval)

add_executable(acoustic_optic_scenario_matrix
  apps/acoustic_optic_scenario_matrix.cpp
  apps/acoustic_optic_scenarios.cpp
)
target_include_directories(acoustic_optic_scenario_matrix PRIVATE "${PROJECT_SOURCE_DIR}/apps")
target_link_libraries(acoustic_optic_scenario_matrix PRIVATE
  uw::domain uw::core uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(acoustic_optic_scenario_matrix)

if(UW_BUILD_ROS2)
  add_executable(holoocean_sonar_bridge_node adapters/ros2/src/holoocean_sonar_bridge_main.cpp)
  target_link_libraries(holoocean_sonar_bridge_node PRIVATE uw::ros2_adapters)
  uw_apply_application_defaults(holoocean_sonar_bridge_node)
endif()
```

- [ ] **Step 4: Build every application and confirm output placement**

Run:

```bash
cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
cmake --build /tmp/uw_slam_layout_cmake \
  --target replay_demo synth_bag_gen synth_stereo_gen optical_baseline_eval \
           acoustic_optic_scenario_matrix -j"$(nproc)"
find /tmp/uw_slam_layout_cmake/bin -maxdepth 1 -type f -executable -printf '%f\n' | sort
```

Expected: all five executable names appear directly under `/tmp/uw_slam_layout_cmake/bin`.

## Task 7: Centralize all C++ tests and integration scripts

**Files:**
- Move: all C++ test files according to the final file map
- Move: `tests/l0_contracts/*.cpp` -> `tests/contracts/`
- Move: `tests/l2_replay/*.sh` -> `tests/integration/`
- Modify: `cmake/Tests.cmake`
- Modify: source comments that mention old test paths

- [ ] **Step 1: Move unit tests by production layer**

Create the target directories and move each test source without renaming its basename:

```text
tests/core/{camera_model_test.cpp,sonar_arc_projector_test.cpp}
tests/frontends/{cfar_detector_test.cpp,sonar_cfar_frontend_test.cpp,
  block_matcher_test.cpp,stereo_optical_depth_frontend_test.cpp,
  landmark_blob_detector_test.cpp,patch_matcher_test.cpp,rigid_transform_fit_test.cpp,
  stereo_landmark_vo_frontend_test.cpp,acoustic_optic_associator_test.cpp,
  posterior_depth_optimizer_test.cpp,acoustic_optic_depth_fusion_frontend_test.cpp}
tests/factor_builders/{relative_pose_residual_test.cpp,depth_residual_test.cpp,
  sonar_range_residual_test.cpp}
tests/estimation/pose_graph_solver_test.cpp
tests/mapping/{submap_manager_test.cpp,acoustic_optic_map_bridge_test.cpp}
tests/runtime/{runtime_test.cpp,mcap_io_test.cpp,config_test.cpp,
  acoustic_optic_synchronizer_test.cpp}
tests/evaluation/{trajectory_metrics_test.cpp,depth_metrics_test.cpp,fusion_metrics_test.cpp}
tests/adapters/{svin_bridge_test.cpp,holoocean_ros_bridge_sonar_frame_provider_test.cpp}
```

- [ ] **Step 2: Move contract and integration tests**

Move:

```text
tests/l0_contracts/*.cpp -> tests/contracts/
tests/l2_replay/*.sh     -> tests/integration/
```

Preserve executable bits on shell scripts.

- [ ] **Step 3: Replace all test source paths in `Tests.cmake`**

Apply these exact path mappings in `cmake/Tests.cmake`; target link libraries, discovery prefixes,
labels, and compile definitions remain unchanged because this step changes only file locations:

```text
core/sensor_models/test/*.cpp                         -> tests/core/*.cpp
algorithms/frontends/*/test/*.cpp                    -> tests/frontends/*.cpp
algorithms/factor_builders/*/test/*.cpp              -> tests/factor_builders/*.cpp
algorithms/estimation/test/*.cpp                     -> tests/estimation/*.cpp
algorithms/mapping/*/test/*.cpp                      -> tests/mapping/*.cpp
runtime/test/*.cpp                                   -> tests/runtime/*.cpp
evaluation/test/*.cpp                                -> tests/evaluation/*.cpp
adapters/third_party/{svin_bridge,holoocean_ros_bridge}/test/*.cpp
                                                       -> tests/adapters/*.cpp
tests/l0_contracts/*.cpp                             -> tests/contracts/*.cpp
tests/l2_replay/*.sh                                 -> tests/integration/*.sh
```

Every source basename remains unchanged. The three `add_test()` commands must point to:

```text
tests/integration/determinism_test.sh
tests/integration/optical_baseline_smoke_test.sh
tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh
```

- [ ] **Step 4: Verify test discovery and labels**

Run:

```bash
cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
cmake --build /tmp/uw_slam_layout_cmake -j"$(nproc)"
ctest --test-dir /tmp/uw_slam_layout_cmake -N
ctest --test-dir /tmp/uw_slam_layout_cmake --print-labels
ctest --test-dir /tmp/uw_slam_layout_cmake --output-on-failure
```

Expected: individual GTest cases are listed with `unit.<layer>.` or `contract.` prefixes; integration tests retain three independent entries; all pass.

## Task 8: Add an executable layer-dependency lint

**Files:**
- Create: `tools/lint/check_layer_dependencies.py`
- Create: `tests/lint/check_layer_dependencies_test.py`
- Modify: `tools/lint/check_no_ros_in_core.sh`
- Modify: `cmake/Dependencies.cmake`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write focused failing tests for the lint rules**

Create `tests/lint/check_layer_dependencies_test.py`:

```python
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "tools" / "lint" / "check_layer_dependencies.py"


def load_checker():
    spec = importlib.util.spec_from_file_location("layer_checker", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LayerDependencyTest(unittest.TestCase):
    def write(self, root: Path, relative: str, contents: str) -> None:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    def test_allows_frontend_to_use_core_and_generated_domain_headers(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(
                root,
                "src/frontends/example.cpp",
                '#include "measurement_api/frontend.hpp"\n'
                '#include "sensor_models/geometry.hpp"\n'
                '#include "uw/domain/measurement.pb.h"\n',
            )
            self.assertEqual(load_checker().check(root), [])

    def test_rejects_estimation_to_frontend_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/estimation/example.cpp", '#include "frontends/example.hpp"\n')
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("estimation must not include frontends", errors[0])

    def test_rejects_ros_header_outside_ros2_adapter(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/frontends/example.cpp", "#include <rclcpp/rclcpp.hpp>\n")
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("ROS/vendor header", errors[0])

    def test_rejects_old_handwritten_uw_include(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write(root, "src/runtime/example.cpp", '#include "uw/runtime/config.hpp"\n')
            errors = load_checker().check(root)
            self.assertEqual(len(errors), 1)
            self.assertIn("legacy hand-written include", errors[0])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify the checker is missing**

Run:

```bash
python3 tests/lint/check_layer_dependencies_test.py -v
```

Expected: FAIL because `tools/lint/check_layer_dependencies.py` does not exist.

- [ ] **Step 3: Implement the dependency checker**

Create `tools/lint/check_layer_dependencies.py`:

```python
#!/usr/bin/env python3
import re
import sys
from pathlib import Path


INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
GENERATED_PROTO_RE = re.compile(r"^uw/domain/[^/]+\.pb\.h$")
ROS_VENDOR_PREFIXES = (
    "rclcpp/",
    "ros/",
    "rmw/",
    "nav_msgs/",
    "sensor_msgs/",
    "geometry_msgs/",
    "holoocean_interfaces/",
    "okvis/",
    "sonar_oculus/",
)
PROJECT_ROLES = {
    "domain",
    "sensor_models",
    "measurement_api",
    "frontends",
    "factor_builders",
    "estimation",
    "mapping",
    "runtime",
    "evaluation",
    "adapters",
}
ALLOWED = {
    "domain": {"domain", "domain_proto"},
    "sensor_models": {"sensor_models", "domain", "domain_proto"},
    "measurement_api": {"measurement_api", "sensor_models", "domain", "domain_proto"},
    "frontends": {"frontends", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "factor_builders": {
        "factor_builders", "measurement_api", "sensor_models", "domain", "domain_proto"
    },
    "estimation": {"estimation", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "mapping": {"mapping", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "runtime": {"runtime", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "evaluation": {"evaluation", "sensor_models", "domain", "domain_proto"},
    "adapters": {"adapters", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "ros2": {"adapters", "measurement_api", "sensor_models", "domain", "domain_proto"},
    "apps": PROJECT_ROLES | {"domain_proto"},
}


def owner(root: Path, path: Path):
    parts = path.relative_to(root).parts
    if not parts:
        return None
    if parts[0] in {"include", "src"} and len(parts) > 1:
        return parts[1] if parts[1] in PROJECT_ROLES else None
    if parts[0] == "adapters" and len(parts) > 1 and parts[1] == "ros2":
        return "ros2"
    if parts[0] == "apps":
        return "apps"
    return None


def included_role(header: str):
    if GENERATED_PROTO_RE.match(header):
        return "domain_proto"
    if header.startswith("uw/"):
        return "legacy"
    first = header.split("/", 1)[0]
    return first if first in PROJECT_ROLES else None


def source_files(root: Path):
    for relative in ("include", "src", "adapters/ros2", "apps"):
        base = root / relative
        if not base.exists():
            continue
        for suffix in ("*.hpp", "*.h", "*.cpp", "*.cc"):
            yield from base.rglob(suffix)


def check(root: Path):
    root = root.resolve()
    errors = []
    for path in sorted(set(source_files(root))):
        source_owner = owner(root, path)
        if source_owner is None:
            continue
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue
            header = match.group(1)
            location = f"{path.relative_to(root)}:{line_number}"
            if header.startswith(ROS_VENDOR_PREFIXES) and source_owner != "ros2":
                errors.append(f"{location}: ROS/vendor header {header} is only allowed in adapters/ros2")
                continue
            dependency = included_role(header)
            if dependency == "legacy":
                errors.append(f"{location}: legacy hand-written include {header}")
                continue
            if dependency and dependency not in ALLOWED[source_owner]:
                errors.append(f"{location}: {source_owner} must not include {dependency} ({header})")
    return errors


def main(argv):
    root = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parents[2]
    errors = check(root)
    if errors:
        print("Layer dependency check failed:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1
    print("OK: C++ layer dependencies and ROS/vendor boundaries are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
```

- [ ] **Step 4: Run unit tests and the checker against the repository**

Run:

```bash
python3 tests/lint/check_layer_dependencies_test.py -v
python3 tools/lint/check_layer_dependencies.py .
```

Expected: four unit tests pass and the repository check prints the `OK:` line.

- [ ] **Step 5: Turn the old lint command into a compatibility entry point**

Replace `tools/lint/check_no_ros_in_core.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 "$ROOT/tools/lint/check_layer_dependencies.py" "$ROOT"
```

Keep the executable bit set.

- [ ] **Step 6: Register lint tests in CTest**

Inside the `UW_BUILD_TESTS` branch in `cmake/Dependencies.cmake`, add:

```cmake
find_package(Python3 REQUIRED COMPONENTS Interpreter)
```

Append to `cmake/Tests.cmake`:

```cmake
add_test(
  NAME lint.layer_dependency_unit
  COMMAND ${Python3_EXECUTABLE} tests/lint/check_layer_dependencies_test.py -v
)
set_tests_properties(lint.layer_dependency_unit PROPERTIES
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  LABELS "lint"
)

add_test(
  NAME lint.layer_dependencies
  COMMAND ${Python3_EXECUTABLE} tools/lint/check_layer_dependencies.py .
)
set_tests_properties(lint.layer_dependencies PROPERTIES
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  LABELS "lint"
)
```

- [ ] **Step 7: Verify lint through both public entry points**

Run:

```bash
tools/lint/check_no_ros_in_core.sh
cmake -S . -B /tmp/uw_slam_layout_cmake \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build
ctest --test-dir /tmp/uw_slam_layout_cmake -L lint --output-on-failure
```

Expected: compatibility script and both CTest lint entries pass.

## Task 9: Move the external baseline and flatten adapter documentation

**Files:**
- Move: `adapters/third_party/sonar_camera_reconstruction_baseline/` -> `baselines/sonar_camera_reconstruction/`
- Move: `adapters/third_party/svin_bridge/README.md` -> `adapters/svin_bridge.md`
- Move: `adapters/third_party/holoocean_ros_bridge/README.md` -> `adapters/holoocean_ros_bridge.md`
- Modify: moved README files
- Modify: `adapters/ros2/README.md`
- Modify: `adapters/datasets/README.md`

- [ ] **Step 1: Move the shell baseline as a separate execution boundary**

Move both `README.md` and `run_baseline.sh` to:

```text
baselines/sonar_camera_reconstruction/
```

Keep `run_baseline.sh` executable. It currently contains no repository-relative path calculation, so
its behavior remains unchanged; update only README prose that names the former core/algorithm paths.
Run:

```bash
bash -n baselines/sonar_camera_reconstruction/run_baseline.sh
```

Expected: shell syntax passes; do not execute the external baseline unless its external dependencies are already available.

- [ ] **Step 2: Preserve portable adapter guidance without package directories**

Move the two adapter READMEs to the adapter root:

```text
adapters/svin_bridge.md
adapters/holoocean_ros_bridge.md
```

Update their code links to:

```text
include/adapters/svin_bridge_local_odometry_provider.hpp
include/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp
src/adapters/
tests/adapters/
adapters/ros2/
baselines/sonar_camera_reconstruction/
```

- [ ] **Step 3: Update remaining adapter-local references**

Update `adapters/ros2/README.md` and `adapters/datasets/README.md` so no active link points into `adapters/third_party`, `apps/tools`, or `apps/replay_demo/src`.

Run:

```bash
rg -n 'adapters/third_party|apps/tools|apps/replay_demo/src' \
  adapters --glob '!holoocean/**' || true
```

Expected: no stale active path matches.

## Task 10: Remove obsolete package scaffolding and update active references

**Files:**
- Delete: all child C++ `CMakeLists.txt` files now superseded by `cmake/*.cmake`
- Remove when empty: old `core/`, `algorithms/`, per-app, per-test, runtime/evaluation package directories
- Modify: `README.md`
- Modify: `CLAUDE.md`
- Modify: `NOTICE`
- Modify: `configs/README.md`
- Modify: `configs/defaults/platform.yaml`
- Modify: active `docs/*.md`
- Modify: schema/source comments containing old paths
- Modify: `tools/verify_pipeline.sh`
- Modify: `docs/superpowers/specs/2026-08-21-cpp-layout-flattening-design.md` only if final names differ

- [ ] **Step 1: Delete superseded CMake files and empty package directories**

Delete every local C++ child `CMakeLists.txt` under the old `core/`, `algorithms/`, `runtime/`, `evaluation/`, `adapters/third_party/`, `adapters/ros2/`, `apps/`, and `tests/` package roots. Remove directories only after confirming they contain no user files.

Run:

```bash
find . \
  -path './.git' -prune -o \
  -path './build*' -prune -o \
  -path './external_repos' -prune -o \
  -name CMakeLists.txt -print
```

Expected: only `./CMakeLists.txt` remains for repository-owned C++ code.

- [ ] **Step 2: Update executable paths in verification automation**

In `tools/verify_pipeline.sh`, replace:

```text
$BUILD_DIR/apps/tools/synth_bag_gen/synth_bag_gen -> $BUILD_DIR/bin/synth_bag_gen
$BUILD_DIR/apps/replay_demo/replay_demo            -> $BUILD_DIR/bin/replay_demo
uw_holoocean_sonar_bridge_node                     -> holoocean_sonar_bridge_node
```

Also update comments that claim tests are registered by per-module CMake files.

- [ ] **Step 3: Update provenance and developer instructions**

Update `NOTICE` to point to:

```text
src/factor_builders/sonar_range_residual.cpp
include/factor_builders/sonar_range_residual.hpp
src/frontends/{cfar_detector.cpp,dbscan.cpp,sonar_cfar_frontend.cpp}
include/frontends/
```

Update `CLAUDE.md` build commands to `build/bin/<executable>`, module paths to the shared roots, target names to `uw::...` aliases where CMake concepts are discussed, and the adapter references to `include/adapters/` plus `adapters/ros2/`.

- [ ] **Step 4: Update active user and architecture documentation**

Update current paths in:

```text
README.md
configs/README.md
configs/defaults/platform.yaml
docs/README.md
docs/uw-slam-newcomer-guide.md
docs/uw-slam-codebase-reference-2026-08-18.md
docs/testing-and-verification-guide-2026-08-20.md
docs/acoustic-optic-slam-platform-architecture-2026-08-17.md
docs/holoocean-to-acoustic-optic-slam-pipeline-2026-08-05.md
schemas/proto/uw/domain/*.proto comments
all moved C++ source comments
```

Do not mechanically rewrite historical files under `docs/superpowers/plans/` or older specs; those documents describe the layout that existed when they were authored.

- [ ] **Step 5: Scan active files for stale paths and old target names**

Run:

```bash
rg -n 'core/(domain|sensor_models|measurement_api)|algorithms/(frontends|factor_builders|estimation|mapping)|adapters/third_party|apps/tools|apps/replay_demo/src|build/apps|uw_(domain|sensor_models|measurement_api|runtime|estimation|evaluation|relative_pose_factor|sonar_range_factor|depth_factor|submap_manager|sonar_cfar_frontend|stereo_optical_depth_frontend|stereo_landmark_vo_frontend|acoustic_optic_depth_fusion|acoustic_optic_map_bridge|svin_bridge|holoocean_ros_bridge)' \
  README.md CLAUDE.md NOTICE configs schemas include src apps adapters baselines tools \
  docs/*.md || true
```

Expected: no stale structural references. Legitimate historical prose must be rewritten to explicitly say “former path” if it is intentionally retained.

- [ ] **Step 6: Check formatting and preservation**

Run:

```bash
git diff --check
git status --short
rg -n 'Copyright|Ported from|sonar_camera_reconstruction|SVIn' \
  include/frontends src/frontends include/factor_builders src/factor_builders NOTICE
```

Expected: no whitespace errors; provenance headers and NOTICE entries remain discoverable; all original user modifications are still represented as modified or moved content.

## Task 11: Full clean verification and incremental-build audit

**Files:**
- Verify only: all files changed by Tasks 2–10
- Output only: `/tmp/uw_slam_flat_verify/`

- [ ] **Step 1: Configure and build with tests enabled**

Run:

```bash
PATH="/home/steve/miniconda3/envs/uw_slam_build/bin:$PATH" \
  cmake -S . -B /tmp/uw_slam_flat_verify/build \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build \
  -DUW_BUILD_TESTS=ON -DUW_BUILD_ROS2=OFF
cmake --build /tmp/uw_slam_flat_verify/build -j"$(nproc)"
```

Expected: all production libraries, five applications, grouped test executables, and generated protobuf code build successfully into `bin/` and `lib/`.

- [ ] **Step 2: Run all CTest suites and lint**

Run:

```bash
ctest --test-dir /tmp/uw_slam_flat_verify/build --output-on-failure
tools/lint/check_no_ros_in_core.sh
python3 tests/lint/check_layer_dependencies_test.py -v
```

Expected: unit, contract, integration, and lint tests all pass.

- [ ] **Step 3: Verify a production-only build**

Run:

```bash
PATH="/home/steve/miniconda3/envs/uw_slam_build/bin:$PATH" \
  cmake -S . -B /tmp/uw_slam_flat_verify/build-no-tests \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build \
  -DUW_BUILD_TESTS=OFF -DUW_BUILD_ROS2=OFF
cmake --build /tmp/uw_slam_flat_verify/build-no-tests -j"$(nproc)"
```

Expected: production libraries and applications build without requiring GTest, Threads test wiring, or test discovery.

- [ ] **Step 4: Run Python adapter tests**

Run:

```bash
(cd adapters/holoocean && .venv/bin/pytest -q)
```

Expected: all current Python adapter tests pass; their package layout is unchanged.

- [ ] **Step 5: Run the full verification pipeline using new output paths**

Run:

```bash
tools/verify_pipeline.sh \
  --build-dir /tmp/uw_slam_flat_verify/pipeline-build \
  --out-dir /tmp/uw_slam_flat_verify/pipeline
```

Expected: `summary.txt` ends in `RESULT: PASSED`, synthetic bag generation uses `bin/synth_bag_gen`, replay uses `bin/replay_demo`, and replay emits an `ATE:` line with the same accepted tolerance as the baseline.

- [ ] **Step 6: Audit incremental recompilation**

Run:

```bash
cmake --build /tmp/uw_slam_flat_verify/build --target frontends -j"$(nproc)"
cmake -E touch src/frontends/cfar_detector.cpp
cmake --build /tmp/uw_slam_flat_verify/build --target frontends --verbose -j1
git diff --exit-code -- src/frontends/cfar_detector.cpp
```

Expected: the first command reports no work; after touching one implementation, the verbose build recompiles `cfar_detector.cpp` and re-archives `frontends` without recompiling unrelated translation units; Git reports no content change.

- [ ] **Step 7: Verify ROS2 when the environment is available**

If both `/opt/ros/jazzy/setup.bash` and `/home/steve/ros2_ws/install/setup.bash` exist, source them and run:

```bash
PATH="/home/steve/miniconda3/envs/uw_slam_build/bin:$PATH" \
  cmake -S . -B /tmp/uw_slam_flat_verify/build-ros2 \
  -DCMAKE_PREFIX_PATH=/home/steve/miniconda3/envs/uw_slam_build \
  -DUW_BUILD_ROS2=ON
cmake --build /tmp/uw_slam_flat_verify/build-ros2 \
  --target holoocean_sonar_bridge_node -j"$(nproc)"
```

Expected: ROS2 node builds. If the environment files are absent, record `SKIP: ROS2 environment unavailable`; do not report a pass.

- [ ] **Step 8: Perform the final worktree safety review**

Run:

```bash
git diff --check
git status --short
git diff --stat
rg -n '#include [<"]uw/(?!domain/[^/]+\.pb\.h)' \
  include src apps adapters/ros2 --pcre2 || true
find . \
  -path './.git' -prune -o \
  -path './build*' -prune -o \
  -path './external_repos' -prune -o \
  -name CMakeLists.txt -print
```

Expected: no whitespace errors; only generated protobuf includes retain `uw/domain/*.pb.h`; repository-owned C++ has only the root `CMakeLists.txt`; no pre-existing user change is missing. Do not commit without explicit user authorization.
