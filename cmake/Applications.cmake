# RunManifest provenance (docs/uw-slam-production-readiness-and-roadmap-
# 2026-08-21.md section 5.6): captured once at CMake configure time, not at
# build/run time, so a fresh commit needs a reconfigure (not just a rebuild)
# to show up in a manifest — acceptable for v1, and simpler than plumbing a
# regenerate-every-build custom command through every app target.
find_package(Git QUIET)
set(UW_GIT_COMMIT "unknown")
if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE UW_GIT_COMMIT_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE UW_GIT_COMMIT_RESULT
  )
  if(UW_GIT_COMMIT_RESULT EQUAL 0 AND NOT UW_GIT_COMMIT_RAW STREQUAL "")
    set(UW_GIT_COMMIT "${UW_GIT_COMMIT_RAW}")
    execute_process(
      COMMAND ${GIT_EXECUTABLE} status --porcelain
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE UW_GIT_DIRTY_CHECK
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(NOT UW_GIT_DIRTY_CHECK STREQUAL "")
      set(UW_GIT_COMMIT "${UW_GIT_COMMIT}-dirty")
    endif()
  endif()
endif()

function(uw_apply_application_defaults target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  target_compile_definitions(${target} PRIVATE UW_GIT_COMMIT="${UW_GIT_COMMIT}")
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )
endfunction()

add_executable(replay_demo apps/replay_demo.cpp)
target_link_libraries(replay_demo PRIVATE uw::application)
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
  uw::application uw::domain uw::core uw::runtime uw::evaluation uw::frontends
)
uw_apply_application_defaults(acoustic_optic_scenario_matrix)

add_executable(bag_audit apps/bag_audit.cpp)
target_link_libraries(bag_audit PRIVATE uw::domain uw::core uw::runtime)
uw_apply_application_defaults(bag_audit)

add_executable(live_ingress_smoke apps/live_ingress_smoke.cpp)
target_link_libraries(live_ingress_smoke PRIVATE
  uw::application uw::runtime Threads::Threads
)
uw_apply_application_defaults(live_ingress_smoke)

add_executable(online_assist_smoke apps/online_assist_smoke.cpp)
target_link_libraries(online_assist_smoke PRIVATE
  uw::application uw::runtime uw::core uw::frontends uw::opencv_adapters Threads::Threads
)
uw_apply_application_defaults(online_assist_smoke)

if(UW_BUILD_ROS2)
  add_executable(holoocean_sonar_bridge_node adapters/ros2/src/holoocean_sonar_bridge_main.cpp)
  target_link_libraries(holoocean_sonar_bridge_node PRIVATE uw::ros2_adapters)
  uw_apply_application_defaults(holoocean_sonar_bridge_node)

  add_executable(holoocean_realtime_node adapters/ros2/src/holoocean_realtime_node.cpp)
  target_link_libraries(holoocean_realtime_node PRIVATE uw::ros2_adapters uw::application)
  uw_apply_application_defaults(holoocean_realtime_node)
endif()
