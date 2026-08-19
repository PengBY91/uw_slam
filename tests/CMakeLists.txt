# Cross-cutting tests that don't belong to a single module (platform
# architecture section 15's L0/L2 rows). Per-module L1 unit tests live next
# to their module (e.g. algorithms/factor_builders/sonar_range_factor/test)
# and are registered by that module's own CMakeLists.txt.

add_executable(uw_l0_domain_contract_test l0_contracts/domain_contract_test.cpp)
target_link_libraries(uw_l0_domain_contract_test PRIVATE uw_domain GTest::gtest GTest::gtest_main)
add_test(NAME uw_l0_domain_contract_test COMMAND uw_l0_domain_contract_test)

add_test(
  NAME uw_l2_replay_determinism_test
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/l2_replay/determinism_test.sh
          $<TARGET_FILE:synth_bag_gen> $<TARGET_FILE:replay_demo>
)
