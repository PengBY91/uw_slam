# nanoflann is a single-header, template-only KD-tree library with no
# CMakeLists.txt dependency footprint worth building from a package manager
# (no compiled artifact, no transitive deps) — same rationale as MCAP's SDK
# (see UwMcap.cmake): fetch just the header via FetchContent_Populate and
# expose it as a plain INTERFACE include-dir target, nothing to
# add_subdirectory or link against.
include(FetchContent)

# Pinned to the v1.10.0 tag (latest stable as of 2026-08-23, verified via the
# GitHub tags API) rather than tracking main — same external-dependency
# pinning discipline as MCAP. Bump deliberately, not implicitly.
FetchContent_Declare(
  nanoflann_src
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG 57d2d8c83c862b66f4689b082423e31314ad27ec  # v1.10.0
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(nanoflann_src)
if(NOT nanoflann_src_POPULATED)
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)  # allow the classic Populate-without-MakeAvailable call below
  endif()
  FetchContent_Populate(nanoflann_src)
endif()

add_library(nanoflann INTERFACE)
target_include_directories(nanoflann SYSTEM INTERFACE ${nanoflann_src_SOURCE_DIR}/include)
