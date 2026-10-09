// Project addition, 2026-10-05. Distributed with FAST-LIVO2 under GPL-2.0.
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>

namespace livo_memory {
// Reading a root never mutates a map or LRU container during an OMP read phase.
struct AccessStamp {
  inline static std::atomic<uint64_t> clock{0};
  std::atomic<uint64_t> last{0};
  std::atomic<bool> pinned{false};
  void touch() { last.store(++clock); pinned.store(true); }
};
struct LiveObjects {
  inline static std::atomic<size_t> trees{0}, planes{0}, points{0}, features{0}, warps{0};
};
struct ScopeExit {
  std::function<void()> done;
  explicit ScopeExit(std::function<void()> value) : done(std::move(value)) {}
  ~ScopeExit() { done(); }
  ScopeExit(const ScopeExit&) = delete;
};
template<class Map> auto oldestUnpinned(Map& map) {
  auto oldest = map.end();
  for (auto it = map.begin(); it != map.end(); ++it)
    if (!it->second->access.pinned.load() &&
        (oldest == map.end() || it->second->access.last.load() < oldest->second->access.last.load())) oldest = it;
  return oldest;
}
template<class Map> void unpin(Map& map) { for (auto& entry : map) entry.second->access.pinned.store(false); }
}
