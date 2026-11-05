#pragma once

#include "base/logging.hpp"
#include "base/timer.hpp"

#include <cstdint>
#include <string>
#include <utility>

// Comaps++ routing timing. Every line is logged with the "[plus-timing]" tag.
// "visited" is the number of vertices the A* searches settled inside the scope.
namespace routing::plus_timing
{
inline thread_local uint64_t g_visited = 0;

inline void OnVisit() { ++g_visited; }

class Scope
{
public:
  explicit Scope(std::string name) : m_name(std::move(name)), m_startVisited(g_visited) {}

  Scope(Scope const &) = delete;
  Scope & operator=(Scope const &) = delete;

  ~Scope()
  {
    LOG(LINFO, ("[plus-timing]", m_name, "ms:", m_timer.ElapsedMilliseconds(), "visited:", g_visited - m_startVisited));
  }

private:
  std::string m_name;
  base::Timer m_timer;
  uint64_t m_startVisited;
};
}  // namespace routing::plus_timing
