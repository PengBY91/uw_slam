add_library(uw_runtime STATIC
  src/run_manifest.cpp
  src/mcap_io.cpp
  src/config.cpp
)
target_include_directories(uw_runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_runtime PUBLIC uw_domain mcap_impl protobuf::libprotobuf yaml-cpp::yaml-cpp Eigen3::Eigen)
target_compile_options(uw_runtime PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  find_package(Threads REQUIRED)
  add_executable(uw_runtime_test test/runtime_test.cpp)
  target_link_libraries(uw_runtime_test PRIVATE uw_runtime GTest::gtest GTest::gtest_main Threads::Threads)
  add_test(NAME uw_runtime_test COMMAND uw_runtime_test)

  add_executable(uw_runtime_mcap_io_test test/mcap_io_test.cpp)
  target_link_libraries(uw_runtime_mcap_io_test PRIVATE uw_runtime GTest::gtest GTest::gtest_main)
  add_test(NAME uw_runtime_mcap_io_test COMMAND uw_runtime_mcap_io_test)

  add_executable(uw_runtime_config_test test/config_test.cpp)
  target_compile_definitions(uw_runtime_config_test PRIVATE UW_REPO_ROOT="${CMAKE_SOURCE_DIR}")
  target_link_libraries(uw_runtime_config_test PRIVATE uw_runtime GTest::gtest GTest::gtest_main)
  add_test(NAME uw_runtime_config_test COMMAND uw_runtime_config_test)
endif()
