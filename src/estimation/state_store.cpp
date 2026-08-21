#include "estimation/state_store.hpp"

namespace uw::estimation {

StateStore::StateStore(std::size_t history_capacity) : history_capacity_(history_capacity) {}

uw::domain::StateVersion StateStore::Commit(uw::domain::StateSnapshot snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  uw::domain::StateVersion version;
  version.set_value(next_version_++);
  *snapshot.mutable_state_version() = version;

  history_.push_back(std::move(snapshot));
  while (history_.size() > history_capacity_) history_.pop_front();

  return version;
}

std::optional<uw::domain::StateSnapshot> StateStore::Latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (history_.empty()) return std::nullopt;
  return history_.back();
}

std::optional<uw::domain::StateSnapshot> StateStore::AtVersion(uint64_t version) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (it->state_version().value() == version) return *it;
  }
  return std::nullopt;
}

}  // namespace uw::estimation
