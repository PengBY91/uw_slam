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

add_library(frontends STATIC
  src/frontends/cfar_detector.cpp
  src/frontends/dbscan.cpp
  src/frontends/sonar_cfar_frontend.cpp
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
target_link_libraries(frontends PUBLIC uw::core)
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
target_link_libraries(factor_builders PUBLIC uw::core)
uw_apply_library_defaults(factor_builders)

add_library(estimation STATIC
  src/estimation/state_store.cpp
  src/estimation/pose_graph_problem.cpp
  src/estimation/gauss_newton_solver.cpp
)
add_library(uw::estimation ALIAS estimation)
target_include_directories(estimation PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(estimation PUBLIC uw::core Eigen3::Eigen)
uw_apply_library_defaults(estimation)

add_library(mapping STATIC
  src/mapping/submap_manager.cpp
  src/mapping/acoustic_optic_map_bridge.cpp
  src/mapping/surfel_map.cpp
)
add_library(uw::mapping ALIAS mapping)
target_include_directories(mapping PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(mapping PUBLIC uw::core)
uw_apply_library_defaults(mapping)

add_library(runtime STATIC
  src/runtime/run_manifest.cpp
  src/runtime/mcap_io.cpp
  src/runtime/config.cpp
  src/runtime/acoustic_optic_synchronizer.cpp
  src/runtime/bag_audit_checks.cpp
)
add_library(uw::runtime ALIAS runtime)
target_include_directories(runtime PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(runtime PUBLIC
  uw::core mcap_impl protobuf::libprotobuf yaml-cpp::yaml-cpp Eigen3::Eigen
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

if(UW_BUILD_ROS2)
  add_library(ros2_adapters INTERFACE)
  add_library(uw::ros2_adapters ALIAS ros2_adapters)
  target_include_directories(ros2_adapters INTERFACE adapters/ros2/include)
  target_link_libraries(ros2_adapters INTERFACE
    uw::adapters rclcpp::rclcpp ${nav_msgs_TARGETS} ${holoocean_interfaces_TARGETS}
  )
endif()
