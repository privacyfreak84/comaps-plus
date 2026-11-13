#pragma once

#include "base/logging.hpp"
#include "base/timer.hpp"

#include <cstdint>
#include <string>
#include <utility>

// Tabula routing timing. Every line is logged with the "[plus-timing]" tag.
// "visited" is the number of vertices the A* searches settled inside the scope.
namespace routing::plus_timing
{
inline thread_local uint64_t g_visited = 0;
inline thread_local uint64_t g_roadLoads = 0;

inline void OnVisit() { ++g_visited; }
inline void OnRoadLoad() { ++g_roadLoads; }

class Scope
{
public:
  explicit Scope(std::string name) : m_name(std::move(name)), m_startVisited(g_visited), m_startLoads(g_roadLoads) {}

  Scope(Scope const &) = delete;
  Scope & operator=(Scope const &) = delete;

  ~Scope()
  {
    LOG(LINFO, ("[plus-timing]", m_name, "ms:", m_timer.ElapsedMilliseconds(), "visited:", g_visited - m_startVisited,
                "loads:", g_roadLoads - m_startLoads));
  }

private:
  std::string m_name;
  base::Timer m_timer;
  uint64_t m_startVisited;
  uint64_t m_startLoads;
};
}  // namespace routing::plus_timing
