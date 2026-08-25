function(uw_apply_library_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
endfunction()

add_library(domain STATIC src/domain/domain.cpp)
add_library(uw::domain ALIAS domain)
target_include_directories(domain PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(domain PUBLIC domain_proto)
uw_apply_library_defaults(domain)

add_library(core STATIC
  src/sensor_models/geometry.cpp
  src/sensor_models/sonar_beam_model.cpp
  src/sensor_models/camera_model.cpp
  src/sensor_models/camera_rectifier.cpp
  src/sensor_models/sonar_arc_projector.cpp
)
add_library(uw::core ALIAS core)
target_include_directories(core PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(core PUBLIC uw::domain Eigen3::Eigen)
uw_apply_library_defaults(core)

add_library(measurement_api INTERFACE)
add_library(uw::measurement_api ALIAS measurement_api)
target_include_directories(measurement_api INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(measurement_api INTERFACE uw::core)

add_library(opencv_adapters STATIC
  adapters/opencv/src/stereo_rectifier.cpp
  adapters/opencv/src/opencv_visual_assist_frontend.cpp
  adapters/opencv/src/operator_overlay_renderer.cpp
)
add_library(uw::opencv_adapters ALIAS opencv_adapters)
target_include_directories(opencv_adapters PUBLIC
  "${PROJECT_SOURCE_DIR}/adapters/opencv/include"
  PRIVATE "${PROJECT_SOURCE_DIR}/adapters/opencv/include/adapters"
)
target_link_libraries(opencv_adapters
  PUBLIC uw::measurement_api
  PRIVATE ${OpenCV_LIBS}
)
uw_apply_library_defaults(opencv_adapters)

add_library(frontends STATIC
  src/frontends/cfar_detector.cpp
  src/frontends/dbscan.cpp
  src/frontends/sonar_cfar_frontend.cpp
  src/frontends/sonar_target_extractor.cpp
  src/frontends/target_associator.cpp
  src/frontends/target_tracker.cpp
  src/frontends/target_fusion_components.cpp
  src/frontends/block_matcher.cpp
  src/frontends/stereo_optical_depth_frontend.cpp
  src/frontends/harris_corner_detector.cpp
  src/frontends/landmark_blob_detector.cpp
  src/frontends/patch_matcher.cpp
  src/frontends/rigid_transform_fit.cpp
  src/frontends/stereo_landmark_vo_frontend.cpp
  src/frontends/acoustic_optic_associator.cpp
  src/frontends/posterior_depth_optimizer.cpp
  src/frontends/acoustic_optic_depth_fusion_frontend.cpp
)
add_library(uw::frontends ALIAS frontends)
target_include_directories(frontends PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(frontends PUBLIC uw::core uw::measurement_api)
uw_apply_library_defaults(frontends)

add_library(factor_builders STATIC
  src/factor_builders/relative_pose_residual.cpp
  src/factor_builders/relative_pose_factor_builder.cpp
  src/factor_builders/depth_residual.cpp
  src/factor_builders/depth_factor_builder.cpp
  src/factor_builders/sonar_range_residual.cpp
  src/factor_builders/sonar_range_factor_builder.cpp
)
add_library(uw::factor_builders ALIAS factor_builders)
target_include_directories(factor_builders PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(factor_builders PUBLIC uw::core uw::measurement_api)
uw_apply_library_defaults(factor_builders)

add_library(estimation STATIC
  src/estimation/state_store.cpp
  src/estimation/pose_graph_problem.cpp
  src/estimation/gauss_newton_solver.cpp
)
add_library(uw::estimation ALIAS estimation)
target_include_directories(estimation PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(estimation PUBLIC uw::core uw::measurement_api Eigen3::Eigen)
uw_apply_library_defaults(estimation)

add_library(mapping STATIC
  src/mapping/submap_manager.cpp
  src/mapping/acoustic_optic_map_bridge.cpp
  src/mapping/surfel_map.cpp
)
add_library(uw::mapping ALIAS mapping)
target_include_directories(mapping PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(mapping PUBLIC uw::core uw::measurement_api)
uw_apply_library_defaults(mapping)

add_library(runtime STATIC
  src/runtime/run_manifest.cpp
  src/runtime/mcap_io.cpp
  src/runtime/config.cpp
  src/runtime/acoustic_optic_synchronizer.cpp
  src/runtime/acoustic_optic_buffer.cpp
  src/runtime/bag_audit_checks.cpp
  src/runtime/synthetic_sonar.cpp
  src/runtime/mcap_event_source.cpp
  src/runtime/canonical_event_validation.cpp
  src/runtime/live_event_source.cpp
  src/runtime/rolling_latency.cpp
)
add_library(uw::runtime ALIAS runtime)
target_include_directories(runtime PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(runtime PUBLIC
  uw::core uw::measurement_api mcap_impl protobuf::libprotobuf yaml-cpp::yaml-cpp Eigen3::Eigen
)
uw_apply_library_defaults(runtime)

add_library(evaluation STATIC
  src/evaluation/trajectory_metrics.cpp
  src/evaluation/depth_metrics.cpp
  src/evaluation/fusion_metrics.cpp
  src/evaluation/map_metrics.cpp
)
add_library(uw::evaluation ALIAS evaluation)
target_include_directories(evaluation PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(evaluation PUBLIC uw::core)
uw_apply_library_defaults(evaluation)

add_library(adapters STATIC
  src/adapters/svin_bridge_local_odometry_provider.cpp
  src/adapters/holoocean_ros_bridge_sonar_frame_provider.cpp
)
add_library(uw::adapters ALIAS adapters)
target_include_directories(adapters PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(adapters PUBLIC uw::core)
uw_apply_library_defaults(adapters)

add_library(spatial_index_adapters STATIC
  adapters/spatial_index/src/nanoflann_surfel_index.cpp
)
add_library(uw::spatial_index_adapters ALIAS spatial_index_adapters)
target_include_directories(spatial_index_adapters PUBLIC
  "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/adapters/spatial_index/include"
)
target_link_libraries(spatial_index_adapters PUBLIC uw::mapping nanoflann)
uw_apply_library_defaults(spatial_index_adapters)

if(UW_BUILD_CERES_SOLVER)
  add_library(adapters_ceres STATIC
    adapters/ceres/src/ceres_pose_graph_solver.cpp
  )
  add_library(uw::adapters_ceres ALIAS adapters_ceres)
  target_include_directories(adapters_ceres PUBLIC
    "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/adapters/ceres/include"
  )
  target_link_libraries(adapters_ceres PUBLIC uw::estimation Ceres::ceres)
  uw_apply_library_defaults(adapters_ceres)
endif()

add_library(application STATIC
  src/application/replay_pipeline.cpp
  src/application/event_pump.cpp
  src/application/replay_input_accumulator.cpp
  src/application/online_assist_pipeline.cpp
  src/application/latest_assist_sink.cpp
)
add_library(uw::application ALIAS application)
target_include_directories(application PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(application
  PUBLIC uw::domain uw::core
  PRIVATE uw::runtime uw::estimation uw::evaluation
          uw::factor_builders uw::mapping uw::frontends uw::opencv_adapters
          uw::spatial_index_adapters
)
uw_apply_library_defaults(application)
if(UW_BUILD_CERES_SOLVER)
  # replay_pipeline.cpp #ifdef-guards its Ceres call site on this macro so
  # the same source file builds correctly whether or not Ceres is present
  # (see docs/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md
  # §8: selecting solver_backend: ceres_v1 in a binary built without this
  # must fail loudly at startup, not silently fall back).
  target_compile_definitions(application PRIVATE UW_HAVE_CERES_SOLVER)
  target_link_libraries(application PRIVATE uw::adapters_ceres)
endif()

if(UW_BUILD_ROS2)
  add_library(ros2_adapters INTERFACE)
  add_library(uw::ros2_adapters ALIAS ros2_adapters)
  target_include_directories(ros2_adapters INTERFACE adapters/ros2/include)
  target_link_libraries(ros2_adapters INTERFACE
    uw::adapters rclcpp::rclcpp ${nav_msgs_TARGETS} ${holoocean_interfaces_TARGETS}
  )
endif()
