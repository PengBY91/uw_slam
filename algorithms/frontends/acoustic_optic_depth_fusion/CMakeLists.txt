add_library(uw_acoustic_optic_depth_fusion STATIC
  src/posterior_depth_optimizer.cpp
  src/acoustic_optic_depth_fusion_frontend.cpp
)
target_include_directories(uw_acoustic_optic_depth_fusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_acoustic_optic_depth_fusion PUBLIC uw_acoustic_optic_associator uw_sensor_models)
target_compile_options(uw_acoustic_optic_depth_fusion PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_acoustic_optic_depth_fusion_test
    test/posterior_depth_optimizer_test.cpp
    test/acoustic_optic_depth_fusion_frontend_test.cpp
  )
  target_link_libraries(uw_acoustic_optic_depth_fusion_test PRIVATE uw_acoustic_optic_depth_fusion GTest::gtest GTest::gtest_main)
  add_test(NAME uw_acoustic_optic_depth_fusion_test COMMAND uw_acoustic_optic_depth_fusion_test)
endif()
