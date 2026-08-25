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
  tests/core/camera_model_test.cpp
  tests/core/camera_rectifier_test.cpp
  tests/core/sonar_arc_projector_test.cpp
)
target_link_libraries(core_tests PRIVATE uw::core GTest::gtest GTest::gtest_main)
uw_register_gtest(core_tests "unit.core" "unit;core")

add_executable(frontends_tests
  tests/frontends/cfar_detector_test.cpp
  tests/frontends/sonar_cfar_frontend_test.cpp
  tests/frontends/sonar_target_extractor_test.cpp
  tests/frontends/block_matcher_test.cpp
  tests/frontends/stereo_optical_depth_frontend_test.cpp
  tests/frontends/harris_corner_detector_test.cpp
  tests/frontends/landmark_blob_detector_test.cpp
  tests/frontends/patch_matcher_test.cpp
  tests/frontends/rigid_transform_fit_test.cpp
  tests/frontends/stereo_landmark_vo_frontend_test.cpp
  tests/frontends/acoustic_optic_associator_test.cpp
  tests/frontends/posterior_depth_optimizer_test.cpp
  tests/frontends/acoustic_optic_depth_fusion_frontend_test.cpp
)
target_link_libraries(frontends_tests PRIVATE uw::frontends GTest::gtest GTest::gtest_main)
uw_register_gtest(frontends_tests "unit.frontends" "unit;frontends")

add_executable(factor_builders_tests
  tests/factor_builders/relative_pose_residual_test.cpp
  tests/factor_builders/relative_pose_factor_builder_test.cpp
  tests/factor_builders/depth_residual_test.cpp
  tests/factor_builders/depth_factor_builder_test.cpp
  tests/factor_builders/sonar_range_residual_test.cpp
  tests/factor_builders/sonar_range_factor_builder_test.cpp
)
target_link_libraries(factor_builders_tests PRIVATE
  uw::factor_builders GTest::gtest GTest::gtest_main
)
uw_register_gtest(factor_builders_tests "unit.factor_builders" "unit;factor_builders")

add_executable(estimation_tests tests/estimation/pose_graph_solver_test.cpp)
target_link_libraries(estimation_tests PRIVATE
  uw::estimation uw::factor_builders GTest::gtest GTest::gtest_main
)
uw_register_gtest(estimation_tests "unit.estimation" "unit;estimation")

if(UW_BUILD_CERES_SOLVER)
  add_executable(adapters_ceres_tests tests/adapters/ceres/ceres_pose_graph_solver_test.cpp)
  target_link_libraries(adapters_ceres_tests PRIVATE
    uw::adapters_ceres uw::estimation uw::factor_builders GTest::gtest GTest::gtest_main
  )
  uw_register_gtest(adapters_ceres_tests "unit.adapters_ceres" "unit;estimation")
endif()

add_executable(mapping_tests
  tests/mapping/submap_manager_test.cpp
  tests/mapping/acoustic_optic_map_bridge_test.cpp
  tests/mapping/surfel_map_test.cpp
)
target_link_libraries(mapping_tests PRIVATE uw::mapping GTest::gtest GTest::gtest_main)
uw_register_gtest(mapping_tests "unit.mapping" "unit;mapping")

add_executable(runtime_tests
  tests/runtime/runtime_test.cpp
  tests/runtime/mcap_io_test.cpp
  tests/runtime/config_test.cpp
  tests/runtime/acoustic_optic_synchronizer_test.cpp
  tests/runtime/bag_audit_checks_test.cpp
  tests/runtime/synthetic_sonar_test.cpp
  tests/runtime/canonical_event_test.cpp
  tests/runtime/canonical_event_validation_test.cpp
  tests/runtime/mcap_event_source_test.cpp
  tests/runtime/live_event_source_test.cpp
  tests/runtime/rolling_latency_test.cpp
)
target_compile_definitions(runtime_tests PRIVATE UW_REPO_ROOT="${PROJECT_SOURCE_DIR}")
target_link_libraries(runtime_tests PRIVATE
  uw::runtime GTest::gtest GTest::gtest_main Threads::Threads
)
uw_register_gtest(runtime_tests "unit.runtime" "unit;runtime")

add_executable(evaluation_tests
  tests/evaluation/trajectory_metrics_test.cpp
  tests/evaluation/depth_metrics_test.cpp
  tests/evaluation/fusion_metrics_test.cpp
  tests/evaluation/map_metrics_test.cpp
)
target_link_libraries(evaluation_tests PRIVATE uw::evaluation GTest::gtest GTest::gtest_main)
uw_register_gtest(evaluation_tests "unit.evaluation" "unit;evaluation")

add_executable(adapters_tests
  tests/adapters/svin_bridge_test.cpp
  tests/adapters/holoocean_ros_bridge_sonar_frame_provider_test.cpp
  tests/adapters/opencv_stereo_rectifier_test.cpp
  tests/adapters/opencv_visual_assist_frontend_test.cpp
)
target_link_libraries(adapters_tests PRIVATE
  uw::adapters uw::opencv_adapters GTest::gtest GTest::gtest_main Threads::Threads
)
uw_register_gtest(adapters_tests "unit.adapters" "unit;adapters")

add_executable(spatial_index_adapters_tests
  tests/adapters/spatial_index/nanoflann_surfel_index_test.cpp
)
target_link_libraries(spatial_index_adapters_tests PRIVATE
  uw::spatial_index_adapters GTest::gtest GTest::gtest_main
)
uw_register_gtest(spatial_index_adapters_tests "unit.spatial_index_adapters" "unit;mapping")

add_executable(application_tests
  tests/application/replay_pipeline_test.cpp
  tests/application/event_pump_test.cpp
  tests/application/replay_input_accumulator_test.cpp
)
target_link_libraries(application_tests PRIVATE
  uw::application uw::domain uw::core uw::runtime GTest::gtest GTest::gtest_main
)
uw_register_gtest(application_tests "unit.application" "unit;application")

add_executable(contract_tests
  tests/contracts/domain_contract_test.cpp
  tests/contracts/measurement_api_contract_test.cpp
)
target_link_libraries(contract_tests PRIVATE uw::domain uw::core GTest::gtest GTest::gtest_main)
uw_register_gtest(contract_tests "contract" "contract")

add_executable(event_source_parity_test tests/integration/event_source_parity_test.cpp)
target_link_libraries(event_source_parity_test PRIVATE uw::application uw::domain uw::core uw::runtime)
set_target_properties(event_source_parity_test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
)
add_test(NAME integration.event_source_parity COMMAND event_source_parity_test)
set_tests_properties(integration.event_source_parity PROPERTIES LABELS "integration;replay;runtime")

add_test(
  NAME integration.replay_determinism
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/integration/determinism_test.sh
          $<TARGET_FILE:synth_bag_gen> $<TARGET_FILE:replay_demo>
)
set_tests_properties(integration.replay_determinism PROPERTIES LABELS "integration;replay")

add_test(
  NAME integration.optical_baseline_smoke
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/integration/optical_baseline_smoke_test.sh
          $<TARGET_FILE:synth_stereo_gen> $<TARGET_FILE:optical_baseline_eval>
          ${PROJECT_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)
set_tests_properties(integration.optical_baseline_smoke PROPERTIES LABELS "integration;replay")

add_test(
  NAME integration.acoustic_optic_scenario_matrix_determinism
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh
          $<TARGET_FILE:acoustic_optic_scenario_matrix>
          ${PROJECT_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)
set_tests_properties(integration.acoustic_optic_scenario_matrix_determinism
  PROPERTIES LABELS "integration;replay")

add_test(
  NAME integration.acoustic_optic_scenario_matrix_config
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/integration/acoustic_optic_scenario_matrix_config_test.sh
          $<TARGET_FILE:acoustic_optic_scenario_matrix>
          ${PROJECT_SOURCE_DIR}
)
set_tests_properties(integration.acoustic_optic_scenario_matrix_config
  PROPERTIES LABELS "integration;replay")

add_test(
  NAME integration.live_ingress_smoke
  COMMAND bash ${PROJECT_SOURCE_DIR}/tests/integration/live_ingress_smoke_test.sh
          ${CMAKE_BINARY_DIR}/bin/live_ingress_smoke
)
set_tests_properties(integration.live_ingress_smoke PROPERTIES
  LABELS "integration;runtime;live"
  TIMEOUT 15
)

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
