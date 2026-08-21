add_library(uw_stereo_landmark_vo_frontend STATIC
  src/harris_corner_detector.cpp
  src/landmark_blob_detector.cpp
  src/patch_matcher.cpp
  src/rigid_transform_fit.cpp
  src/stereo_landmark_vo_frontend.cpp
)
target_include_directories(uw_stereo_landmark_vo_frontend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_stereo_landmark_vo_frontend PUBLIC uw_measurement_api uw_sensor_models)
target_compile_options(uw_stereo_landmark_vo_frontend PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_stereo_landmark_vo_frontend_test
    test/harris_corner_detector_test.cpp
    test/landmark_blob_detector_test.cpp
    test/patch_matcher_test.cpp
    test/rigid_transform_fit_test.cpp
    test/stereo_landmark_vo_frontend_test.cpp
  )
  target_link_libraries(uw_stereo_landmark_vo_frontend_test PRIVATE uw_stereo_landmark_vo_frontend GTest::gtest GTest::gtest_main)
  add_test(NAME uw_stereo_landmark_vo_frontend_test COMMAND uw_stereo_landmark_vo_frontend_test)
endif()
