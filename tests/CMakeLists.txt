# Cross-cutting tests that don't belong to a single module (platform
# architecture section 15's L0/L2 rows). Per-module L1 unit tests live next
# to their module (e.g. algorithms/factor_builders/sonar_range_factor/test)
# and are registered by that module's own CMakeLists.txt.

add_executable(uw_l0_domain_contract_test l0_contracts/domain_contract_test.cpp)
target_link_libraries(uw_l0_domain_contract_test PRIVATE uw_domain GTest::gtest GTest::gtest_main)
add_test(NAME uw_l0_domain_contract_test COMMAND uw_l0_domain_contract_test)

add_executable(uw_l0_measurement_api_contract_test
  l0_contracts/measurement_api_contract_test.cpp
)
target_link_libraries(uw_l0_measurement_api_contract_test PRIVATE
  uw_measurement_api GTest::gtest GTest::gtest_main
)
add_test(NAME uw_l0_measurement_api_contract_test
  COMMAND uw_l0_measurement_api_contract_test
)

add_test(
  NAME uw_l2_replay_determinism_test
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/l2_replay/determinism_test.sh
          $<TARGET_FILE:synth_bag_gen> $<TARGET_FILE:replay_demo>
)

add_test(
  NAME uw_l2_optical_baseline_smoke_test
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/l2_replay/optical_baseline_smoke_test.sh
          $<TARGET_FILE:synth_stereo_gen> $<TARGET_FILE:optical_baseline_eval>
          ${CMAKE_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)

add_test(
  NAME uw_l2_acoustic_optic_scenario_matrix_determinism_test
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/l2_replay/acoustic_optic_scenario_matrix_determinism_test.sh
          $<TARGET_FILE:acoustic_optic_scenario_matrix>
          ${CMAKE_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)
