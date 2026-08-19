add_library(uw_relative_pose_factor STATIC
  src/relative_pose_residual.cpp
  src/relative_pose_factor_builder.cpp
)
target_include_directories(uw_relative_pose_factor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_relative_pose_factor PUBLIC uw_measurement_api)
target_compile_options(uw_relative_pose_factor PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_relative_pose_factor_test test/relative_pose_residual_test.cpp)
  target_link_libraries(uw_relative_pose_factor_test PRIVATE uw_relative_pose_factor GTest::gtest GTest::gtest_main)
  add_test(NAME uw_relative_pose_factor_test COMMAND uw_relative_pose_factor_test)
endif()
