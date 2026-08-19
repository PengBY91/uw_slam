add_library(uw_estimation STATIC
  src/state_store.cpp
  src/pose_graph_problem.cpp
  src/gauss_newton_solver.cpp
)
target_include_directories(uw_estimation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_estimation PUBLIC uw_measurement_api Eigen3::Eigen)
target_compile_options(uw_estimation PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_estimation_test test/pose_graph_solver_test.cpp)
  target_link_libraries(uw_estimation_test PRIVATE
    uw_estimation
    uw_relative_pose_factor
    uw_depth_factor
    GTest::gtest
    GTest::gtest_main
  )
  add_test(NAME uw_estimation_test COMMAND uw_estimation_test)
endif()
