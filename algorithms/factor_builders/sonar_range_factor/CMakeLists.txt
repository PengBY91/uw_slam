add_library(uw_sonar_range_factor STATIC
  src/sonar_range_residual.cpp
  src/sonar_range_factor_builder.cpp
)
target_include_directories(uw_sonar_range_factor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_sonar_range_factor PUBLIC uw_measurement_api)
target_compile_options(uw_sonar_range_factor PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_sonar_range_factor_test test/sonar_range_residual_test.cpp)
  target_link_libraries(uw_sonar_range_factor_test PRIVATE uw_sonar_range_factor GTest::gtest GTest::gtest_main)
  add_test(NAME uw_sonar_range_factor_test COMMAND uw_sonar_range_factor_test)
endif()
