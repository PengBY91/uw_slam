add_library(uw_domain STATIC src/domain.cpp)
target_include_directories(uw_domain PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_domain PUBLIC uw_domain_proto)
target_compile_options(uw_domain PRIVATE -Wall -Wextra)
