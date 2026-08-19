add_library(uw_sonar_cfar_frontend STATIC
  src/cfar_detector.cpp
  src/dbscan.cpp
  src/sonar_cfar_frontend.cpp
)
target_include_directories(uw_sonar_cfar_frontend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_sonar_cfar_frontend PUBLIC uw_measurement_api)
target_compile_options(uw_sonar_cfar_frontend PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_sonar_cfar_frontend_test
    test/cfar_detector_test.cpp
    test/sonar_cfar_frontend_test.cpp
  )
  target_link_libraries(uw_sonar_cfar_frontend_test PRIVATE uw_sonar_cfar_frontend GTest::gtest GTest::gtest_main)
  add_test(NAME uw_sonar_cfar_frontend_test COMMAND uw_sonar_cfar_frontend_test)
endif()
