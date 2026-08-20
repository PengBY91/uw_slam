add_executable(synth_stereo_gen src/main.cpp)
target_link_libraries(synth_stereo_gen PRIVATE uw_domain uw_runtime)
target_compile_options(synth_stereo_gen PRIVATE -Wall -Wextra)
