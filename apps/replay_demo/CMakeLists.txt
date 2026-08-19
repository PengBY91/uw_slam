add_executable(replay_demo src/main.cpp)
target_link_libraries(replay_demo PRIVATE
  uw_domain
  uw_sensor_models
  uw_runtime
  uw_estimation
  uw_evaluation
  uw_relative_pose_factor
  uw_sonar_range_factor
  uw_depth_factor
  uw_submap_manager
  uw_sonar_cfar_frontend
)
target_compile_options(replay_demo PRIVATE -Wall -Wextra)
