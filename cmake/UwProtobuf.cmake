# Generates C++ bindings for the uw.domain protobuf schemas (the single source
# for the cross-language normalized message model, see schemas/proto/) and
# exposes them as the domain_proto static library. Every core/algorithms/adapters
# target that needs domain types links against this instead of hand-rolling
# equivalent structs.

find_package(Protobuf REQUIRED)
find_package(absl CONFIG REQUIRED)

set(UW_PROTO_ROOT ${CMAKE_SOURCE_DIR}/schemas/proto)
file(GLOB UW_DOMAIN_PROTO_FILES CONFIGURE_DEPENDS ${UW_PROTO_ROOT}/uw/domain/*.proto)

set(UW_PROTO_GEN_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${UW_PROTO_GEN_DIR})

set(UW_DOMAIN_PROTO_GENERATED)
foreach(UW_PROTO_FILE IN LISTS UW_DOMAIN_PROTO_FILES)
  file(RELATIVE_PATH UW_PROTO_RELATIVE ${UW_PROTO_ROOT} ${UW_PROTO_FILE})
  string(REGEX REPLACE "\\.proto$" ".pb.cc" UW_PROTO_CC_RELATIVE ${UW_PROTO_RELATIVE})
  string(REGEX REPLACE "\\.proto$" ".pb.h" UW_PROTO_H_RELATIVE ${UW_PROTO_RELATIVE})
  set(UW_PROTO_CC ${UW_PROTO_GEN_DIR}/${UW_PROTO_CC_RELATIVE})
  set(UW_PROTO_H ${UW_PROTO_GEN_DIR}/${UW_PROTO_H_RELATIVE})

  add_custom_command(
    OUTPUT ${UW_PROTO_CC} ${UW_PROTO_H}
    COMMAND protobuf::protoc
    ARGS --cpp_out=${UW_PROTO_GEN_DIR}
         --proto_path=${UW_PROTO_ROOT}
         ${UW_PROTO_RELATIVE}
    DEPENDS ${UW_DOMAIN_PROTO_FILES} protobuf::protoc
    WORKING_DIRECTORY ${UW_PROTO_ROOT}
    COMMENT "Generating C++ protobuf bindings for ${UW_PROTO_RELATIVE}"
    VERBATIM
  )
  list(APPEND UW_DOMAIN_PROTO_GENERATED ${UW_PROTO_CC} ${UW_PROTO_H})
endforeach()

add_library(domain_proto STATIC ${UW_DOMAIN_PROTO_GENERATED})
target_include_directories(domain_proto SYSTEM PUBLIC ${UW_PROTO_GEN_DIR})
target_link_libraries(domain_proto PUBLIC protobuf::libprotobuf)

# protobuf::libprotobuf declares absl::* in its INTERFACE_LINK_LIBRARIES,
# but on this conda-forge toolchain that doesn't reliably make it onto the
# final link command for executables several static-library hops away
# ("DSO missing from command line" for libabsl_hash.so and similar) — link
# the same absl component set explicitly so every consumer gets it
# unconditionally, regardless of how deep in the static-lib chain it sits.
target_link_libraries(domain_proto PUBLIC
  absl::flat_hash_map absl::hash absl::strings absl::status absl::statusor
  absl::synchronization absl::time absl::base absl::log absl::cord
  absl::log_internal_check_op
)
# Generated code triggers warnings we don't control; keep it out of -Werror.
set_target_properties(domain_proto PROPERTIES COMPILE_OPTIONS "-w")
