add_library(uw_sensor_models STATIC
  src/geometry.cpp
  src/sonar_beam_model.cpp
  src/camera_model.cpp
  src/sonar_arc_projector.cpp
)
target_include_directories(uw_sensor_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_sensor_models PUBLIC uw_domain Eigen3::Eigen)
target_compile_options(uw_sensor_models PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_sensor_models_camera_model_test test/camera_model_test.cpp)
  target_link_libraries(uw_sensor_models_camera_model_test PRIVATE uw_sensor_models GTest::gtest GTest::gtest_main)
  add_test(NAME uw_sensor_models_camera_model_test COMMAND uw_sensor_models_camera_model_test)

  add_executable(uw_sensor_models_sonar_arc_projector_test test/sonar_arc_projector_test.cpp)
  target_link_libraries(uw_sensor_models_sonar_arc_projector_test PRIVATE uw_sensor_models GTest::gtest GTest::gtest_main)
  add_test(NAME uw_sensor_models_sonar_arc_projector_test COMMAND uw_sensor_models_sonar_arc_projector_test)
endif()
