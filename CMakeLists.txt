cmake_minimum_required(VERSION 3.22)
project(uw_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Sanitizer flags apply to the complete link graph, including generated and
# FetchContent sources. Mixing sanitized and unsanitized objects can fail at
# link or runtime. ASan and UBSan are combined; TSan must remain separate.
#
# NOTE on UW_SANITIZER=thread: verified 2026-08-22 that this currently
# reports heap-use-after-free/race findings on plain protobuf getter calls
# in genuinely single-threaded test bodies (e.g. DomainContract.
# ImageFrameRoundTripsWithCanonicalHeader) — while the SAME code passes
# clean under UW_SANITIZER=address (which is at least as sensitive to a
# real use-after-free). Root cause: this environment's protobuf/gtest come
# from prebuilt conda-forge shared libraries (`ldd` on any test binary
# shows libprotobuf.so.35.1.0/libgtest.so.1.18.0 are dynamically linked),
# not rebuilt with -fsanitize=thread — TSan cannot see synchronization
# inside an uninstrumented library and reports spurious races on its
# internal atomics/arena allocation. Fixing this needs a from-source
# protobuf+gtest rebuild under TSan, out of scope for this CMake option;
# UW_SANITIZER=thread remains available for specialized toolchains, but is
# not wired into CI until that prerequisite is addressed. This sandbox's TSan
# additionally requires ASLR disabled to even start (`setarch $(uname -m)
# -R` before both the build — gtest_discover_tests runs binaries at build
# time — and any run) — a "ThreadSanitizer: unexpected memory mapping"
# FATAL error is this environment issue, not a code problem.
set(UW_SANITIZER "OFF" CACHE STRING "Build with a sanitizer: OFF, address (ASan+UBSan), or thread (TSan)")
set_property(CACHE UW_SANITIZER PROPERTY STRINGS OFF address thread)
if(NOT UW_SANITIZER STREQUAL "OFF")
  if(UW_SANITIZER STREQUAL "address")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    # GCC's null/nonnull UBSan modes make non-weak function addresses appear
    # nullable during constant evaluation.  That triggers GCC PR 71962 in
    # Abseil's hash_policy_traits (abseil-cpp#1634) before our code can build.
    # Keep ASan and every other UBSan check enabled; only disable the three
    # GCC modes that independently reproduce the compiler error.  Clang does
    # not have this issue and retains the complete undefined group.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      add_compile_options(
        -fno-sanitize=null,nonnull-attribute,returns-nonnull-attribute)
    endif()
    add_link_options(-fsanitize=address,undefined)
  elseif(UW_SANITIZER STREQUAL "thread")
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
  else()
    message(FATAL_ERROR "UW_SANITIZER must be OFF, address, or thread (got: ${UW_SANITIZER})")
  endif()
endif()

# Opt-in gcc/gcov coverage instrumentation. It remains separate from
# UW_SANITIZER because the options are orthogonal; their combination is not
# part of the supported verification matrix.
option(UW_COVERAGE "Build with gcov coverage instrumentation (--coverage)" OFF)
if(UW_COVERAGE)
  add_compile_options(--coverage -fno-omit-frame-pointer)
  add_link_options(--coverage)
endif()

include(cmake/Dependencies.cmake)
include(cmake/Libraries.cmake)
include(cmake/Applications.cmake)

if(UW_BUILD_TESTS)
  include(cmake/Tests.cmake)
endif()
