add_library(uw_acoustic_optic_map_bridge STATIC
  src/acoustic_optic_map_bridge.cpp
)
target_include_directories(uw_acoustic_optic_map_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_acoustic_optic_map_bridge PUBLIC uw_sensor_models)
target_compile_options(uw_acoustic_optic_map_bridge PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_acoustic_optic_map_bridge_test test/acoustic_optic_map_bridge_test.cpp)
  target_link_libraries(uw_acoustic_optic_map_bridge_test PRIVATE
    uw_acoustic_optic_map_bridge uw_submap_manager GTest::gtest GTest::gtest_main
  )
  add_test(NAME uw_acoustic_optic_map_bridge_test COMMAND uw_acoustic_optic_map_bridge_test)
endif()
