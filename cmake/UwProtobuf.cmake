# Generates C++ bindings for the uw.domain protobuf schemas (the single
# source of truth for domain contracts, see schemas/proto/) and exposes them
# as the domain_proto static library. Every core/algorithms/adapters
# target that needs domain types links against this instead of hand-rolling
# equivalent structs.

find_package(Protobuf REQUIRED)
find_package(absl CONFIG REQUIRED)

set(UW_PROTO_ROOT ${CMAKE_SOURCE_DIR}/schemas/proto)
file(GLOB UW_DOMAIN_PROTO_FILES CONFIGURE_DEPENDS ${UW_PROTO_ROOT}/uw/domain/*.proto)

set(UW_PROTO_GEN_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${UW_PROTO_GEN_DIR})

protobuf_generate(
  LANGUAGE cpp
  OUT_VAR UW_DOMAIN_PROTO_GENERATED
  IMPORT_DIRS ${UW_PROTO_ROOT}
  PROTOC_OUT_DIR ${UW_PROTO_GEN_DIR}
  PROTOS ${UW_DOMAIN_PROTO_FILES}
)

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
)
# Generated code triggers warnings we don't control; keep it out of -Werror.
set_target_properties(domain_proto PROPERTIES COMPILE_OPTIONS "-w")
