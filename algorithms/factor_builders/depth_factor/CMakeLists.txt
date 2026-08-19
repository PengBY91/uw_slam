add_library(uw_depth_factor STATIC
  src/depth_residual.cpp
  src/depth_factor_builder.cpp
)
target_include_directories(uw_depth_factor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_depth_factor PUBLIC uw_measurement_api)
target_compile_options(uw_depth_factor PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_depth_factor_test test/depth_residual_test.cpp)
  target_link_libraries(uw_depth_factor_test PRIVATE uw_depth_factor GTest::gtest GTest::gtest_main)
  add_test(NAME uw_depth_factor_test COMMAND uw_depth_factor_test)
endif()
