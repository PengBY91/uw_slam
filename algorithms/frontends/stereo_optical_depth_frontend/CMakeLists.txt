add_library(uw_stereo_optical_depth_frontend STATIC
  src/block_matcher.cpp
  src/stereo_optical_depth_frontend.cpp
)
target_include_directories(uw_stereo_optical_depth_frontend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_stereo_optical_depth_frontend PUBLIC uw_measurement_api uw_sensor_models)
target_compile_options(uw_stereo_optical_depth_frontend PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_stereo_optical_depth_frontend_test
    test/block_matcher_test.cpp
    test/stereo_optical_depth_frontend_test.cpp
  )
  target_link_libraries(uw_stereo_optical_depth_frontend_test PRIVATE uw_stereo_optical_depth_frontend GTest::gtest GTest::gtest_main)
  add_test(NAME uw_stereo_optical_depth_frontend_test COMMAND uw_stereo_optical_depth_frontend_test)
endif()
