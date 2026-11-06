#pragma once

// Via-way turn restrictions (from -> via way(s) -> to) as an Aho-Corasick automaton over the sequence
// of distinct features a route passes. A state of depth >= 2 that is not terminal means "inside a
// restricted sequence"; the segments of its feature get a copy per such state, and a step into a
// terminal state is a forbidden transition. Shared by the tools, the builder and the runtime.

#include <cstdint>
#include <map>
#include <vector>

namespace plus_overlay
{
class ViaWayAutomaton
{
public:
  explicit ViaWayAutomaton(std::vector<std::vector<uint32_t>> const & patterns)
  {
    NewNode(0, 0);
    for (auto const & pat : patterns)
    {
      uint32_t n = 0;
      for (uint32_t f : pat)
      {
        auto it = m_next[n].find(f);
        if (it == m_next[n].end())
        {
          uint32_t const c = NewNode(f, m_depth[n] + 1);
          m_next[n][f] = c;
          n = c;
        }
        else
        {
          n = it->second;
        }
      }
      m_term[n] = 1;
    }
    std::vector<uint32_t> queue;
    for (auto const & [f, c] : m_next[0])
      queue.push_back(c);
    for (size_t i = 0; i < queue.size(); ++i)
    {
      uint32_t const u = queue[i];
      m_term[u] |= m_term[m_fail[u]];
      for (auto const & [f, c] : m_next[u])
      {
        m_fail[c] = (u == 0) ? 0 : Step(m_fail[u], f);
        queue.push_back(c);
      }
    }
  }

  uint32_t Step(uint32_t n, uint32_t f) const
  {
    while (true)
    {
      auto it = m_next[n].find(f);
      if (it != m_next[n].end())
        return it->second;
      if (n == 0)
        return 0;
      n = m_fail[n];
    }
  }
  uint32_t Start(uint32_t f) const { return Step(0, f); }
  uint32_t Size() const { return static_cast<uint32_t>(m_depth.size()); }
  uint32_t Depth(uint32_t n) const { return m_depth[n]; }
  uint32_t Label(uint32_t n) const { return m_label[n]; }
  bool Terminal(uint32_t n) const { return m_term[n]; }

private:
  uint32_t NewNode(uint32_t label, uint32_t depth)
  {
    m_next.emplace_back();
    m_fail.push_back(0);
    m_depth.push_back(depth);
    m_label.push_back(label);
    m_term.push_back(0);
    return Size() - 1;
  }
  std::vector<std::map<uint32_t, uint32_t>> m_next;
  std::vector<uint32_t> m_fail, m_depth, m_label;
  std::vector<uint8_t> m_term;
};
}  // namespace plus_overlay
