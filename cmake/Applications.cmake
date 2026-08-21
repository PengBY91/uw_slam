function(uw_apply_application_defaults target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )
endfunction()

add_executable(replay_demo apps/replay_demo.cpp)
target_link_libraries(replay_demo PRIVATE
  uw::domain uw::core uw::runtime uw::estimation uw::evaluation
  uw::factor_builders uw::mapping uw::frontends
)
uw_apply_application_defaults(replay_demo)

add_executable(synth_bag_gen apps/synth_bag_gen.cpp)
target_link_libraries(synth_bag_gen PRIVATE uw::domain uw::core uw::runtime)
uw_apply_application_defaults(synth_bag_gen)

add_executable(synth_stereo_gen apps/synth_stereo_gen.cpp)
target_link_libraries(synth_stereo_gen PRIVATE uw::domain uw::runtime)
uw_apply_application_defaults(synth_stereo_gen)

add_executable(optical_baseline_eval apps/optical_baseline_eval.cpp)
target_link_libraries(optical_baseline_eval PRIVATE
  uw::domain uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(optical_baseline_eval)

add_executable(acoustic_optic_scenario_matrix
  apps/acoustic_optic_scenario_matrix.cpp
  apps/acoustic_optic_scenarios.cpp
)
target_include_directories(acoustic_optic_scenario_matrix PRIVATE "${PROJECT_SOURCE_DIR}/apps")
target_link_libraries(acoustic_optic_scenario_matrix PRIVATE
  uw::domain uw::core uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(acoustic_optic_scenario_matrix)

if(UW_BUILD_ROS2)
  add_executable(holoocean_sonar_bridge_node adapters/ros2/src/holoocean_sonar_bridge_main.cpp)
  target_link_libraries(holoocean_sonar_bridge_node PRIVATE uw::ros2_adapters)
  uw_apply_application_defaults(holoocean_sonar_bridge_node)
endif()
