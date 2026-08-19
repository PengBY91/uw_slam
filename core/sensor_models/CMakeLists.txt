add_library(uw_sensor_models STATIC
  src/geometry.cpp
  src/sonar_beam_model.cpp
)
target_include_directories(uw_sensor_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_sensor_models PUBLIC uw_domain Eigen3::Eigen)
target_compile_options(uw_sensor_models PRIVATE -Wall -Wextra)
