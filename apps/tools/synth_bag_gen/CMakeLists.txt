add_executable(synth_bag_gen src/main.cpp)
target_link_libraries(synth_bag_gen PRIVATE uw_domain uw_sensor_models uw_runtime)
target_compile_options(synth_bag_gen PRIVATE -Wall -Wextra)
