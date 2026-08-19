// The single translation unit that instantiates MCAP's header-only
// implementation. See UwMcap.cmake. Do not add MCAP_IMPLEMENTATION to any
// other file — the mcap headers are not written to tolerate multiple
// implementation TUs.
#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>
