add_executable(optical_baseline_eval src/main.cpp)
target_link_libraries(optical_baseline_eval PRIVATE
  uw_domain
  uw_runtime
  uw_evaluation
  uw_stereo_optical_depth_frontend
)
target_compile_options(optical_baseline_eval PRIVATE -Wall -Wextra)
