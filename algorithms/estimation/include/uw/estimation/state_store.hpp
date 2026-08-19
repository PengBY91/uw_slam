#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "uw/domain/domain.hpp"

namespace uw::estimation {

// The single authoritative, versioned state store (platform architecture
// section 7.7 / section 21 invariant #2). Single-writer (Commit) by
// contract — not lock-free, but readers (Latest/AtVersion) never block a
// concurrent Commit for more than a short critical section, and never see a
// partially-written snapshot.
class StateStore {
 public:
  explicit StateStore(std::size_t history_capacity = 256);

  uw::domain::StateVersion Commit(uw::domain::StateSnapshot snapshot);

  std::optional<uw::domain::StateSnapshot> Latest() const;
  std::optional<uw::domain::StateSnapshot> AtVersion(uint64_t version) const;

 private:
  mutable std::mutex mutex_;
  std::deque<uw::domain::StateSnapshot> history_;
  std::size_t history_capacity_;
  uint64_t next_version_ = 1;
};

}  // namespace uw::estimation
