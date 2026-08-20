add_executable(acoustic_optic_scenario_matrix
  src/scenarios.cpp
  src/main.cpp
)
target_link_libraries(acoustic_optic_scenario_matrix PRIVATE
  uw_domain
  uw_runtime
  uw_evaluation
  uw_sensor_models
  uw_sonar_cfar_frontend
  uw_stereo_optical_depth_frontend
  uw_acoustic_optic_depth_fusion
)
target_compile_options(acoustic_optic_scenario_matrix PRIVATE -Wall -Wextra)
