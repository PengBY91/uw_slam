# MCAP has no apt/conda package suitable for a from-scratch CMake build and
# no CMakeLists.txt of its own (upstream consumes it via Bazel/Conan/vendor
# copy) — its C++ SDK is a header-only, single-TU-implementation library
# (mcap/mcap.hpp guarded by #ifdef MCAP_IMPLEMENTATION). We fetch just the
# source (FetchContent_Populate, not MakeAvailable, since there is nothing
# to add_subdirectory) and provide our own two targets:
#   mcap       - INTERFACE: include dir + MCAP_COMPRESSION_NO_{ZSTD,LZ4}
#                (v1 writes uncompressed chunks; fine for small synthetic
#                bags, avoids a zstd/lz4 dependency entirely)
#   mcap_impl  - STATIC: the single translation unit compiled with
#                MCAP_IMPLEMENTATION defined. Link against this (not `mcap`
#                directly) from anything that calls into mcap::McapWriter /
#                McapReader.
include(FetchContent)

# Pinned to the commit this workspace was already building against (verified
# working) rather than tracking `main`, so a from-scratch checkout can't
# silently pick up an incompatible upstream change (P0 gate: pin external
# deps, see docs/archive/uw-slam-production-readiness-and-roadmap-2026-08-21.md).
# Bump deliberately, not implicitly, when there's a reason to.
FetchContent_Declare(
  mcap_src
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG b6b85df9927fb8ccb7c76e5437cc0d0bfa966ceb
  GIT_SHALLOW FALSE
)
FetchContent_GetProperties(mcap_src)
if(NOT mcap_src_POPULATED)
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)  # allow the classic Populate-without-MakeAvailable call below
  endif()
  FetchContent_Populate(mcap_src)
endif()

add_library(mcap INTERFACE)
target_include_directories(mcap SYSTEM INTERFACE ${mcap_src_SOURCE_DIR}/cpp/mcap/include)
target_compile_definitions(mcap INTERFACE MCAP_COMPRESSION_NO_ZSTD MCAP_COMPRESSION_NO_LZ4)

add_library(mcap_impl STATIC ${CMAKE_SOURCE_DIR}/cmake/mcap_impl.cpp)
target_link_libraries(mcap_impl PUBLIC mcap)
