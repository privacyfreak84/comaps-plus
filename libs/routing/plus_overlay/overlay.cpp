#include "routing/plus_overlay/overlay.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <atomic>
#include <mutex>
#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <utility>

namespace plus_overlay
{
namespace
{
using Item = Scratch::Side::Item;

void HeapPush(std::vector<Item> & heap, double d, VertexId v)
{
  heap.push_back({d, v});
  std::push_heap(heap.begin(), heap.end(), std::greater<Item>());
}

Item HeapPop(std::vector<Item> & heap)
{
  std::pop_heap(heap.begin(), heap.end(), std::greater<Item>());
  Item it = heap.back();
  heap.pop_back();
  return it;
}

double Top(std::vector<Item> const & heap)
{
  return heap.empty() ? kInf : heap.front().d;
}
}  // namespace

// Graph -------------------------------------------------------------------------------------------

void Graph::BuildReverse()
{
  revOff.assign(n + 1, 0);
  for (VertexId t : fwdTo)
    ++revOff[t + 1];
  for (uint32_t v = 0; v < n; ++v)
    revOff[v + 1] += revOff[v];
  revFrom.assign(fwdTo.size(), 0);
  revW.assign(fwdTo.size(), 0.0);
  if (!fwdK.empty())
    revK.assign(fwdTo.size(), 0);
  else
    revK.clear();
  std::vector<uint64_t> pos(revOff.begin(), revOff.end() - 1);
  for (uint32_t u = 0; u < n; ++u)
  {
    for (uint64_t e = fwdOff[u]; e < fwdOff[u + 1]; ++e)
    {
      uint64_t const r = pos[fwdTo[e]]++;
      revFrom[r] = u;
      revW[r] = fwdW[e];
      if (!fwdK.empty())
        revK[r] = fwdK[e];
    }
  }
}

void Graph::SetWeight(VertexId u, VertexId v, double w)
{
  for (uint64_t e = fwdOff[u]; e < fwdOff[u + 1]; ++e)
    if (fwdTo[e] == v)
      fwdW[e] = w;
  for (uint64_t e = revOff[v]; e < revOff[v + 1]; ++e)
    if (revFrom[e] == u)
      revW[e] = w;
}

// Chain compression -------------------------------------------------------------------------------

Compressed CompressChains(Graph const & raw, std::vector<uint8_t> const & valid)
{
  uint32_t const n = raw.n;
  std::vector<uint32_t> indeg(n, 0);
  for (VertexId t : raw.fwdTo)
    ++indeg[t];

  auto outdeg = [&](VertexId v) { return raw.fwdOff[v + 1] - raw.fwdOff[v]; };

  std::vector<uint32_t> cid(n, kNone);
  Compressed c;
  for (uint32_t v = 0; v < n; ++v)
  {
    if (!valid[v])
      continue;
    uint64_t const od = outdeg(v);
    if (indeg[v] == 0 && od == 0)
      continue;  // isolated
    if (indeg[v] == 1 && od == 1)
      continue;  // chain interior
    cid[v] = static_cast<uint32_t>(c.origId.size());
    c.origId.push_back(v);
  }

  Graph & g = c.g;
  g.n = static_cast<uint32_t>(c.origId.size());
  g.fwdOff.assign(g.n + 1, 0);
  g.lat.resize(g.n);
  g.lon.resize(g.n);

  std::vector<std::pair<VertexId, double>> tmp;
  for (uint32_t cu = 0; cu < g.n; ++cu)
  {
    VertexId const u = c.origId[cu];
    g.lat[cu] = raw.lat[u];
    g.lon[cu] = raw.lon[u];
    tmp.clear();
    for (uint64_t e = raw.fwdOff[u]; e < raw.fwdOff[u + 1]; ++e)
    {
      VertexId x = raw.fwdTo[e];
      double w = raw.fwdW[e];
      uint32_t steps = 0;
      while (cid[x] == kNone && steps++ < n)
      {
        if (outdeg(x) != 1)
          break;
        uint64_t const e2 = raw.fwdOff[x];
        w += raw.fwdW[e2];
        x = raw.fwdTo[e2];
      }
      if (cid[x] == kNone || x == u)
        continue;
      tmp.emplace_back(cid[x], w);
    }
    std::sort(tmp.begin(), tmp.end());
    for (size_t i = 0; i < tmp.size(); ++i)
    {
      if (i > 0 && tmp[i].first == tmp[i - 1].first)
        continue;  // keep the cheapest parallel edge (sorted by weight within target)
      g.fwdTo.push_back(tmp[i].first);
      g.fwdW.push_back(tmp[i].second);
    }
    g.fwdOff[cu + 1] = g.fwdTo.size();
  }
  g.BuildReverse();
  return c;
}

std::vector<uint8_t> StronglyConnectedFrom(Graph const & g, VertexId seed, uint32_t & size)
{
  std::vector<uint8_t> mark(g.n, 0);
  std::vector<VertexId> stack{seed};
  mark[seed] = 1;
  while (!stack.empty())
  {
    VertexId v = stack.back();
    stack.pop_back();
    for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
      if (!(mark[g.fwdTo[e]] & 1))
      {
        mark[g.fwdTo[e]] |= 1;
        stack.push_back(g.fwdTo[e]);
      }
  }
  stack.push_back(seed);
  mark[seed] |= 2;
  while (!stack.empty())
  {
    VertexId v = stack.back();
    stack.pop_back();
    for (uint64_t e = g.revOff[v]; e < g.revOff[v + 1]; ++e)
      if (!(mark[g.revFrom[e]] & 2))
      {
        mark[g.revFrom[e]] |= 2;
        stack.push_back(g.revFrom[e]);
      }
  }
  size = 0;
  for (auto & m : mark)
  {
    m = (m == 3) ? 1 : 0;
    size += m;
  }
  return mark;
}

// Partition ---------------------------------------------------------------------------------------

namespace
{
class Bisector
{
public:
  Bisector(Graph const & g, PartitionParams const & params, Partition & p)
    : m_g(g), m_params(params), m_p(p), m_memb(new std::atomic<uint32_t>[g.n])
  {
    for (uint32_t v = 0; v < g.n; ++v)
      m_memb[v].store(0, std::memory_order_relaxed);
    m_rank.assign(g.n, 0);
    double latSum = 0;
    for (uint32_t v = 0; v < g.n; ++v)
      latSum += g.lat[v];
    // Projection reads lat/lon directly (no x/y copies): x = lon * k, y = lat.
    m_k = static_cast<float>(std::cos((g.n ? latSum / g.n : 0.0) * M_PI / 180.0));
    m_freeThreads = static_cast<int>(std::max(1u, params.threads)) - 1;
  }

  // Subtrees touch disjoint vertex sets, so they can run on different threads: m_rank is only written
  // for the subtree's own vertices, and m_memb (read for neighbours) holds a stamp unique to the call.
  // Returns the node of |verts| in the partition tree.
  uint32_t Run(std::vector<VertexId> verts, uint64_t parentSize)
  {
    auto const & cs = m_params.cellSizes;
    size_t const m = verts.size();
    uint32_t node;
    {
      std::lock_guard<std::mutex> lock(m_cellMutex);
      node = static_cast<uint32_t>(m_p.tree.nodes.size());
      m_p.tree.nodes.emplace_back();
    }
    for (size_t li = 0; li < cs.size(); ++li)
    {
      if (m <= cs[li] && parentSize > cs[li])
      {
        {
          std::lock_guard<std::mutex> lock(m_cellMutex);
          m_p.tree.nodes[node].closes |= static_cast<uint8_t>(1u << li);
        }
        uint32_t id;
        {
          std::lock_guard<std::mutex> lock(m_cellMutex);
          id = m_p.numCells[li]++;
        }
        for (VertexId v : verts)
          m_p.cell[li][v] = id;
      }
    }
    if (m <= cs[0] || m < 2)
    {
      if (m_params.progress && m_g.n > 0)
        m_params.progress(static_cast<double>(m_placed.fetch_add(m) + m) / m_g.n);
      return node;
    }
    if (m_params.cancelled && m_params.cancelled())
      return node;

    uint32_t const stamp = m_stamp.fetch_add(1) + 1;
    for (VertexId v : verts)
      m_memb[v].store(stamp, std::memory_order_relaxed);

    size_t const kLo = std::max<size_t>(1, static_cast<size_t>(std::ceil(m_params.minBalance * m)));
    size_t const kHi = std::min<size_t>(m - 1, static_cast<size_t>(std::floor((1.0 - m_params.minBalance) * m)));

    std::vector<std::pair<float, VertexId>> kv(m);
    std::vector<int32_t> diff(m + 2);
    std::vector<VertexId> bestOrder;
    int64_t bestCut = std::numeric_limits<int64_t>::max();
    size_t bestK = m / 2;
    int bestDir = -1;
    float bestThreshold = 0;

    for (int d = 0; d < m_params.directions; ++d)
    {
      double const a = M_PI * d / m_params.directions;
      float const c = static_cast<float>(std::cos(a)) * m_k;
      float const s = static_cast<float>(std::sin(a));
      for (size_t i = 0; i < m; ++i)
        kv[i] = {m_g.lon[verts[i]] * c + m_g.lat[verts[i]] * s, verts[i]};
      std::sort(kv.begin(), kv.end());
      for (size_t i = 0; i < m; ++i)
        m_rank[kv[i].second] = static_cast<uint32_t>(i);

      std::fill(diff.begin(), diff.end(), 0);
      for (VertexId v : verts)
      {
        uint32_t const rv = m_rank[v];
        for (uint64_t e = m_g.fwdOff[v]; e < m_g.fwdOff[v + 1]; ++e)
        {
          VertexId const w = m_g.fwdTo[e];
          if (m_memb[w].load(std::memory_order_relaxed) != stamp)
            continue;
          uint32_t lo = rv, hi = m_rank[w];
          if (lo > hi)
            std::swap(lo, hi);
          ++diff[lo + 1];
          --diff[hi + 1];
        }
      }
      int64_t cut = 0;
      for (size_t k = 1; k <= kHi; ++k)
      {
        cut += diff[k];
        if (k < kLo)
          continue;
        bool const better = cut < bestCut || (cut == bestCut && std::llabs(static_cast<long long>(k) - static_cast<long long>(m / 2)) <
                                                                      std::llabs(static_cast<long long>(bestK) - static_cast<long long>(m / 2)));
        if (better)
        {
          bestCut = cut;
          bestK = k;
          bestThreshold = kv[k - 1].first + (kv[k].first - kv[k - 1].first) / 2;
          if (bestOrder.size() != m || d != bestDir)
          {
            bestOrder.resize(m);
            for (size_t i = 0; i < m; ++i)
              bestOrder[i] = kv[i].second;
            bestDir = d;
          }
        }
      }
    }
    std::vector<std::pair<float, VertexId>>().swap(kv);
    std::vector<int32_t>().swap(diff);

    std::vector<VertexId> left(bestOrder.begin(), bestOrder.begin() + bestK);
    std::vector<VertexId> right(bestOrder.begin() + bestK, bestOrder.end());
    std::vector<VertexId>().swap(verts);
    std::vector<VertexId>().swap(bestOrder);

    {
      std::lock_guard<std::mutex> lock(m_cellMutex);
      m_p.tree.nodes[node].dir = static_cast<uint8_t>(std::max(0, bestDir));
      m_p.tree.nodes[node].threshold = bestThreshold;
    }
    // Hand the left half to another thread if one is free and the half is worth it.
    bool spawn = false;
    if (left.size() > 20000)
    {
      int f = m_freeThreads.load();
      while (f > 0 && !m_freeThreads.compare_exchange_weak(f, f - 1))
      {
      }
      spawn = f > 0;
    }
    uint32_t leftNode = 0, rightNode = 0;
    if (spawn)
    {
      std::thread t([this, l = std::move(left), m, &leftNode]() mutable { leftNode = Run(std::move(l), m); });
      rightNode = Run(std::move(right), m);
      t.join();
      m_freeThreads.fetch_add(1);
    }
    else
    {
      leftNode = Run(std::move(left), m);
      rightNode = Run(std::move(right), m);
    }
    std::lock_guard<std::mutex> lock(m_cellMutex);
    m_p.tree.nodes[node].left = leftNode;
    m_p.tree.nodes[node].right = rightNode;
    return node;
  }

  float K() const { return m_k; }

private:
  Graph const & m_g;
  PartitionParams const & m_params;
  Partition & m_p;
  std::unique_ptr<std::atomic<uint32_t>[]> m_memb;
  std::vector<uint32_t> m_rank;
  float m_k = 1;
  std::atomic<uint32_t> m_stamp{0};
  std::atomic<int> m_freeThreads{0};
  std::atomic<uint64_t> m_placed{0};
  std::mutex m_cellMutex;
};
}  // namespace

Partition Bisect(Graph const & g, PartitionParams const & params)
{
  Partition p;
  size_t const levels = params.cellSizes.size();
  p.cell.assign(levels, std::vector<uint32_t>(g.n, kNone));
  p.numCells.assign(levels, 0);
  std::vector<VertexId> all(g.n);
  for (uint32_t v = 0; v < g.n; ++v)
    all[v] = v;
  Bisector b(g, params, p);
  p.tree.root = b.Run(std::move(all), std::numeric_limits<uint64_t>::max());
  p.tree.k = b.K();
  p.tree.directions = params.directions;
  p.tree.cellSizes = params.cellSizes;
  return p;
}

bool ReplayPartition(Graph const & g, PartitionTree const & tree, PartitionParams const & params, Partition & p,
                     std::string * why)
{
  auto fail = [&](std::string const & s)
  {
    if (why)
      *why = s;
    return false;
  };
  if (tree.Empty() || tree.directions != params.directions || tree.cellSizes != params.cellSizes ||
      tree.root >= tree.nodes.size())
    return fail("partition tree made with other parameters");
  size_t const levels = params.cellSizes.size();
  p = {};
  p.cell.assign(levels, std::vector<uint32_t>(g.n, kNone));
  p.numCells.assign(levels, 0);
  std::vector<VertexId> all(g.n);
  for (uint32_t v = 0; v < g.n; ++v)
    all[v] = v;
  // Iterative walk; empty subtrees get no cell ids.
  std::vector<std::pair<uint32_t, std::vector<VertexId>>> stack;
  stack.emplace_back(tree.root, std::move(all));
  uint64_t steps = 0;
  while (!stack.empty())
  {
    auto [node, verts] = std::move(stack.back());
    stack.pop_back();
    if (verts.empty())
      continue;
    if (node >= tree.nodes.size() || ++steps > 4 * tree.nodes.size() + 16)
      return fail("damaged partition tree");
    auto const & nd = tree.nodes[node];
    for (size_t li = 0; li < levels; ++li)
    {
      if (!(nd.closes & (1u << li)))
        continue;
      uint32_t const id = p.numCells[li]++;
      for (VertexId v : verts)
        p.cell[li][v] = id;
    }
    if (nd.dir == PartitionTree::kLeaf)
      continue;
    if (nd.dir >= tree.directions)
      return fail("damaged partition tree");
    double const a = M_PI * nd.dir / tree.directions;
    float const c = static_cast<float>(std::cos(a)) * tree.k;
    float const s = static_cast<float>(std::sin(a));
    std::vector<VertexId> left, right;
    for (VertexId v : verts)
      (g.lon[v] * c + g.lat[v] * s < nd.threshold ? left : right).push_back(v);
    std::vector<VertexId>().swap(verts);
    stack.emplace_back(nd.right, std::move(right));
    stack.emplace_back(nd.left, std::move(left));
  }
  // Every vertex needs a cell at every level, and level-1 cells mustn't grow much (query speed).
  std::vector<uint32_t> size1(p.numCells[0], 0);
  for (size_t li = 0; li < levels; ++li)
    for (uint32_t v = 0; v < g.n; ++v)
      if (p.cell[li][v] == kNone)
        return fail("partition tree doesn't cover every road");
  for (uint32_t v = 0; v < g.n; ++v)
    ++size1[p.cell[0][v]];
  uint32_t const biggest = size1.empty() ? 0 : *std::max_element(size1.begin(), size1.end());
  if (biggest > 2 * params.cellSizes[0])
    return fail("roads changed too much for the old cells (a cell has " + std::to_string(biggest) + " points)");
  p.tree = tree;
  return true;
}

namespace
{
constexpr char kTreeMagic[8] = {'P', 'L', 'U', 'S', 'T', 'R', 'E', '1'};
}

bool SavePartitionTree(PartitionTree const & t, std::string const & path)
{
  std::string const tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out)
      return false;
    uint32_t const nNodes = static_cast<uint32_t>(t.nodes.size()), nSizes = static_cast<uint32_t>(t.cellSizes.size());
    int32_t const dirs = t.directions;
    out.write(kTreeMagic, sizeof(kTreeMagic));
    out.write(reinterpret_cast<char const *>(&nNodes), 4);
    out.write(reinterpret_cast<char const *>(&t.root), 4);
    out.write(reinterpret_cast<char const *>(&t.k), 4);
    out.write(reinterpret_cast<char const *>(&dirs), 4);
    out.write(reinterpret_cast<char const *>(&nSizes), 4);
    out.write(reinterpret_cast<char const *>(t.cellSizes.data()), nSizes * 4);
    out.write(reinterpret_cast<char const *>(t.nodes.data()), static_cast<std::streamsize>(nNodes * sizeof(PartitionTree::Node)));
    if (!out)
      return false;
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

bool LoadPartitionTree(std::string const & path, PartitionTree & t)
{
  std::ifstream in(path, std::ios::binary);
  char magic[8];
  uint32_t nNodes = 0, nSizes = 0;
  int32_t dirs = 0;
  t = {};
  if (!in.read(magic, 8) || std::memcmp(magic, kTreeMagic, 8) != 0)
    return false;
  in.read(reinterpret_cast<char *>(&nNodes), 4);
  in.read(reinterpret_cast<char *>(&t.root), 4);
  in.read(reinterpret_cast<char *>(&t.k), 4);
  in.read(reinterpret_cast<char *>(&dirs), 4);
  in.read(reinterpret_cast<char *>(&nSizes), 4);
  if (!in || nSizes > 16 || nNodes > (1u << 26))
    return false;
  t.directions = dirs;
  t.cellSizes.resize(nSizes);
  in.read(reinterpret_cast<char *>(t.cellSizes.data()), nSizes * 4);
  t.nodes.resize(nNodes);
  in.read(reinterpret_cast<char *>(t.nodes.data()), static_cast<std::streamsize>(nNodes * sizeof(PartitionTree::Node)));
  if (!in)
  {
    t = {};
    return false;
  }
  return true;
}

// Customization -----------------------------------------------------------------------------------

Scratch::Scratch(uint32_t n)
{
  for (auto & s : m_side)
  {
    s.dist.assign(n, kInf);
    s.stamp.assign(n, 0);
    s.parent.assign(n, kNone);
    s.parentLevel.assign(n, 0);
    s.heap.reserve(1 << 16);
  }
  m_done[0].assign(n, 0);
  m_done[1].assign(n, 0);
}

void BuildBoundaries(Graph const & g, Partition const & p, Overlay & ov, std::vector<uint8_t> const * extraRole,
                     std::vector<uint32_t> const * special)
{
  ov.levels.assign(p.Levels(), {});
  std::vector<std::vector<uint8_t>> open;
  if (special)
    open = OpenCells(p.numCells, *special, [&p](size_t li, uint32_t v) { return p.cell[li][v]; });
  for (size_t li = 0; li < p.Levels(); ++li)
  {
    auto const & cell = p.cell[li];
    LevelOverlay & L = ov.levels[li];
    uint32_t const nc = p.numCells[li];
    // bit 1 entry, bit 2 exit. Forward edges only: an edge leaving the cell makes its source an exit and
    // its target an entry of the target's cell.
    std::vector<uint8_t> role = extraRole ? *extraRole : std::vector<uint8_t>(g.n, 0);
    for (uint32_t v = 0; v < g.n; ++v)
      for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
        if (cell[g.fwdTo[e]] != cell[v])
        {
          role[v] |= 2;
          role[g.fwdTo[e]] |= 1;
        }
    if (special)
    {
      for (uint32_t v : *special)
        role[v] |= 3;
      for (uint32_t v = 0; v < g.n; ++v)
        if (open[li][cell[v]])
          role[v] = 0;
    }
    L.entryOff.assign(nc + 1, 0);
    L.exitOff.assign(nc + 1, 0);
    for (uint32_t v = 0; v < g.n; ++v)
    {
      if (role[v] & 1)
        ++L.entryOff[cell[v] + 1];
      if (role[v] & 2)
        ++L.exitOff[cell[v] + 1];
    }
    for (uint32_t c = 0; c < nc; ++c)
    {
      L.entryOff[c + 1] += L.entryOff[c];
      L.exitOff[c + 1] += L.exitOff[c];
    }
    L.entries.assign(L.entryOff[nc], 0);
    L.exits.assign(L.exitOff[nc], 0);
    std::vector<uint64_t> ep(L.entryOff.begin(), L.entryOff.end() - 1);
    std::vector<uint64_t> xp(L.exitOff.begin(), L.exitOff.end() - 1);
    for (uint32_t v = 0; v < g.n; ++v)
    {
      uint32_t const c = cell[v];
      if (role[v] & 1)
        L.entries[ep[c]++] = v;
      if (role[v] & 2)
        L.exits[xp[c]++] = v;
    }
    L.role = std::move(role);
    L.matOff.assign(nc + 1, 0);
    for (uint32_t c = 0; c < nc; ++c)
      L.matOff[c + 1] = L.matOff[c] + (L.entryOff[c + 1] - L.entryOff[c]) * (L.exitOff[c + 1] - L.exitOff[c]);
    L.mat.assign(L.matOff[nc], kUnreachable);
    L.matK.assign(L.matOff[nc], 0);
  }
}

namespace
{
// Per-thread work arrays sized to one cell's vertex set, not to the whole graph.
struct CellScratch
{
  std::vector<double> dist;
  std::vector<uint8_t> k;  // penalty units of the path to each vertex
  std::vector<uint32_t> stamp, done;
  std::vector<std::pair<double, uint32_t>> heap;
  uint32_t cur = 0;

  void Reset(size_t n)
  {
    if (dist.size() < n)
    {
      dist.resize(n);
      k.resize(n);
      stamp.resize(n, 0);
      done.resize(n, 0);
    }
  }
};

// Vertices taking part in a cell's searches: at level 1 the cell's vertices, above that the entries and
// exits of its child cells. Cells of one level are disjoint, so |localId| can be shared by all threads.
struct LevelCells
{
  std::vector<uint64_t> off;
  std::vector<VertexId> verts;
};

LevelCells CellVertexLists(Graph const & g, Partition const & p, Overlay const & ov, size_t li)
{
  LevelCells lc;
  uint32_t const nc = p.numCells[li];
  lc.off.assign(nc + 1, 0);
  auto const & cell = p.cell[li];
  if (li == 0)
  {
    for (uint32_t v = 0; v < g.n; ++v)
      if (cell[v] != kNone)
        ++lc.off[cell[v] + 1];
    for (uint32_t c = 0; c < nc; ++c)
      lc.off[c + 1] += lc.off[c];
    lc.verts.resize(lc.off[nc]);
    std::vector<uint64_t> pos(lc.off.begin(), lc.off.end() - 1);
    for (uint32_t v = 0; v < g.n; ++v)
      if (cell[v] != kNone)
        lc.verts[pos[cell[v]]++] = v;
    return lc;
  }
  LevelOverlay const & C = ov.levels[li - 1];
  uint32_t const ncc = p.numCells[li - 1];
  // Child boundary vertices, each once (a vertex can be both entry and exit of its child).
  std::vector<VertexId> all;
  for (uint32_t cc = 0; cc < ncc; ++cc)
  {
    for (uint64_t i = C.entryOff[cc]; i < C.entryOff[cc + 1]; ++i)
      all.push_back(C.entries[i]);
    for (uint64_t i = C.exitOff[cc]; i < C.exitOff[cc + 1]; ++i)
      if (!C.IsEntry(C.exits[i]))
        all.push_back(C.exits[i]);
  }
  for (VertexId v : all)
    ++lc.off[cell[v] + 1];
  for (uint32_t c = 0; c < nc; ++c)
    lc.off[c + 1] += lc.off[c];
  lc.verts.resize(lc.off[nc]);
  std::vector<uint64_t> pos(lc.off.begin(), lc.off.end() - 1);
  for (VertexId v : all)
    lc.verts[pos[cell[v]]++] = v;
  return lc;
}

// One cell: Dijkstra from every entry, restricted to the cell. Level 1 runs on original edges,
// higher levels on the children's cliques plus original edges between children.
void ComputeCell(Graph const & g, Partition const & p, Overlay & ov, size_t li, uint32_t c, LevelCells const & lc,
                 std::vector<uint32_t> & localId, CellScratch & sc)
{
  LevelOverlay & L = ov.levels[li];
  auto const & cellOf = p.cell[li];
  uint64_t const e0 = L.entryOff[c], nEn = L.entryOff[c + 1] - e0;
  uint64_t const x0 = L.exitOff[c], nEx = L.exitOff[c + 1] - x0;
  if (nEn == 0 || nEx == 0)
    return;
  uint64_t const v0 = lc.off[c], nv = lc.off[c + 1] - v0;
  for (uint64_t i = 0; i < nv; ++i)
    localId[lc.verts[v0 + i]] = static_cast<uint32_t>(i);
  sc.Reset(nv);
  auto push = [&](double d, uint32_t l)
  {
    sc.heap.emplace_back(d, l);
    std::push_heap(sc.heap.begin(), sc.heap.end(), std::greater<>());
  };

  for (uint64_t i = 0; i < nEn; ++i)
  {
    uint32_t const st = ++sc.cur;
    VertexId const src = L.entries[e0 + i];
    sc.heap.clear();
    uint32_t const ls = localId[src];
    sc.stamp[ls] = st;
    sc.dist[ls] = 0;
    sc.k[ls] = 0;
    push(0, ls);
    uint64_t remaining = nEx;

    auto relax = [&](VertexId y, double d, uint32_t k)
    {
      uint32_t const ly = localId[y];
      if (sc.stamp[ly] != st || d < sc.dist[ly])
      {
        sc.stamp[ly] = st;
        sc.dist[ly] = d;
        sc.k[ly] = static_cast<uint8_t>(std::min<uint32_t>(k, 255));
        push(d, ly);
      }
    };

    while (!sc.heap.empty())
    {
      std::pop_heap(sc.heap.begin(), sc.heap.end(), std::greater<>());
      auto const [d, lx] = sc.heap.back();
      sc.heap.pop_back();
      if (sc.done[lx] == st)
        continue;
      sc.done[lx] = st;
      VertexId const x = lc.verts[v0 + lx];
      if (L.IsExit(x) && --remaining == 0)
        break;
      uint32_t const kx = sc.k[lx];
      if (li == 0)
      {
        for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
          if (cellOf[g.fwdTo[e]] == c && g.fwdW[e] < kInf)
            relax(g.fwdTo[e], d + g.fwdW[e], kx + g.FwdK(e));
        continue;
      }
      LevelOverlay const & C = ov.levels[li - 1];
      uint32_t const cc = p.cell[li - 1][x];
      uint64_t const cx0 = C.exitOff[cc], cnEx = C.exitOff[cc + 1] - cx0;
      if (C.IsEntry(x))
      {
        uint64_t const base = C.matOff[cc] + C.EntryIdx(x, cc) * cnEx;
        for (uint64_t j = 0; j < cnEx; ++j)
          if (C.mat[base + j] < kInf)
            relax(C.exits[cx0 + j], d + C.mat[base + j], kx + (C.matK.empty() ? 0 : C.matK[base + j]));
      }
      if (C.IsExit(x))
      {
        for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
        {
          VertexId const y = g.fwdTo[e];
          if (cellOf[y] == c && p.cell[li - 1][y] != cc && g.fwdW[e] < kInf)
            relax(y, d + g.fwdW[e], kx + g.FwdK(e));
        }
      }
    }
    for (uint64_t j = 0; j < nEx; ++j)
    {
      uint32_t const ly = localId[L.exits[x0 + j]];
      bool const reached = sc.done[ly] == st;
      L.mat[L.matOff[c] + i * nEx + j] = reached ? static_cast<float>(sc.dist[ly]) : kUnreachable;
      if (!L.matK.empty())
        L.matK[L.matOff[c] + i * nEx + j] = reached ? sc.k[ly] : 0;
    }
  }
}
}  // namespace

std::vector<double> Customize(Graph const & g, Partition const & p, Overlay & ov, unsigned threads,
                              std::function<bool()> const & cancelled, std::function<void(double)> const & progress)
{
  threads = std::max(1u, threads);
  std::vector<double> seconds;
  std::vector<uint32_t> localId(g.n, kNone);
  std::vector<CellScratch> scratch(threads);
  for (size_t li = 0; li < p.Levels(); ++li)
  {
    auto const t0 = std::chrono::steady_clock::now();
    uint32_t const nc = p.numCells[li];
    LevelCells const lc = CellVertexLists(g, p, ov, li);
    std::atomic<uint32_t> next{0};
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; ++t)
    {
      pool.emplace_back([&, t]()
      {
        for (uint32_t c = next.fetch_add(1); c < nc; c = next.fetch_add(1))
        {
          if ((c & 63) == 0)
          {
            if (cancelled && cancelled())
            {
              next.store(nc);
              break;
            }
            if (progress && nc > 0)
              progress((li + static_cast<double>(c) / nc) / p.Levels());
          }
          ComputeCell(g, p, ov, li, c, lc, localId, scratch[t]);
        }
      });
    }
    for (auto & th : pool)
      th.join();
    seconds.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  }
  return seconds;
}

uint64_t Recustomize(Graph const & g, Partition const & p, Overlay & ov, std::vector<VertexId> const & touched,
                     Scratch & /* sc */)
{
  uint64_t count = 0;
  std::vector<uint32_t> localId(g.n, kNone);
  CellScratch scratch;
  for (size_t li = 0; li < p.Levels(); ++li)
  {
    std::vector<uint32_t> cells;
    for (VertexId v : touched)
      cells.push_back(p.cell[li][v]);
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    LevelCells const lc = CellVertexLists(g, p, ov, li);
    for (uint32_t c : cells)
      ComputeCell(g, p, ov, li, c, lc, localId, scratch);
    count += cells.size();
  }
  return count;
}

// Queries -----------------------------------------------------------------------------------------

namespace
{
template <typename ScanF, typename ScanB>
QueryResult Bidirectional(Graph const & g, Seeds const & sources, Seeds const & targets, Scratch & sc, ScanF && scanF,
                          ScanB && scanB)
{
  QueryResult r;
  auto & F = sc.Fwd();
  auto & B = sc.Bwd();
  auto & doneF = sc.Done(0);
  auto & doneB = sc.Done(1);
  uint32_t const st = sc.NewStamp();
  F.heap.clear();
  B.heap.clear();
  for (auto const & [v, d] : sources)
  {
    if (F.stamp[v] == st && F.dist[v] <= d)
      continue;
    F.stamp[v] = st;
    F.dist[v] = d;
    F.parent[v] = kNone;
    HeapPush(F.heap, d, v);
  }
  for (auto const & [v, d] : targets)
  {
    if (B.stamp[v] == st && B.dist[v] <= d)
      continue;
    B.stamp[v] = st;
    B.dist[v] = d;
    B.parent[v] = kNone;
    HeapPush(B.heap, d, v);
  }
  double mu = kInf;
  VertexId meet = kNone;
  for (auto const & [v, d] : targets)
  {
    if (F.stamp[v] == st && F.dist[v] + B.dist[v] < mu)
    {
      mu = F.dist[v] + B.dist[v];
      meet = v;
    }
  }

  auto relaxF = [&](VertexId v, double d, VertexId par, uint8_t lvl)
  {
    if (F.stamp[v] != st || d < F.dist[v])
    {
      F.stamp[v] = st;
      F.dist[v] = d;
      F.parent[v] = par;
      F.parentLevel[v] = lvl;
      HeapPush(F.heap, d, v);
      if (B.stamp[v] == st && d + B.dist[v] < mu)
      {
        mu = d + B.dist[v];
        meet = v;
      }
    }
  };
  auto relaxB = [&](VertexId v, double d, VertexId par, uint8_t lvl)
  {
    if (B.stamp[v] != st || d < B.dist[v])
    {
      B.stamp[v] = st;
      B.dist[v] = d;
      B.parent[v] = par;
      B.parentLevel[v] = lvl;
      HeapPush(B.heap, d, v);
      if (F.stamp[v] == st && d + F.dist[v] < mu)
      {
        mu = d + F.dist[v];
        meet = v;
      }
    }
  };

  while (!F.heap.empty() || !B.heap.empty())
  {
    double const tf = Top(F.heap), tb = Top(B.heap);
    if (tf + tb >= mu)
      break;
    if (tf <= tb)
    {
      Item const it = HeapPop(F.heap);
      if (doneF[it.v] == st)
        continue;
      doneF[it.v] = st;
      ++r.settled;
      scanF(it.v, it.d, relaxF);
    }
    else
    {
      Item const it = HeapPop(B.heap);
      if (doneB[it.v] == st)
        continue;
      doneB[it.v] = st;
      ++r.settled;
      scanB(it.v, it.d, relaxB);
    }
  }
  r.weight = mu;
  if (meet != kNone)
    r.path.push_back(meet);  // caller decides whether to expand
  return r;
}
}  // namespace

QueryResult BidirectionalDijkstra(Graph const & g, VertexId s, VertexId t, Scratch & sc, bool wantPath)
{
  auto scanF = [&](VertexId x, double d, auto & relax)
  {
    for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
      if (g.fwdW[e] < kInf)
        relax(g.fwdTo[e], d + g.fwdW[e], x, 0);
  };
  auto scanB = [&](VertexId x, double d, auto & relax)
  {
    for (uint64_t e = g.revOff[x]; e < g.revOff[x + 1]; ++e)
      if (g.revW[e] < kInf)
        relax(g.revFrom[e], d + g.revW[e], x, 0);
  };
  QueryResult r = Bidirectional(g, Seeds{{s, 0.0}}, Seeds{{t, 0.0}}, sc, scanF, scanB);
  if (!wantPath || r.path.empty())
  {
    r.path.clear();
    return r;
  }
  VertexId const meet = r.path.front();
  r.path.clear();
  for (VertexId v = meet; v != kNone; v = sc.Fwd().parent[v])
    r.path.push_back(v);
  std::reverse(r.path.begin(), r.path.end());
  for (VertexId v = sc.Bwd().parent[meet]; v != kNone; v = sc.Bwd().parent[v])
    r.path.push_back(v);
  return r;
}

namespace
{
struct Hop
{
  VertexId u, v;
  uint8_t level;  // 0 = graph edge, k = clique of the level-k cell containing u and v
};

// Replaces one clique hop by the path it stands for: a Dijkstra from u to v inside the same cell,
// one level down, then recursively for every clique hop found there. Returns false if v is not
// reached, which would mean the clique and the cell contents disagree.
bool UnpackHop(Graph const & g, Partition const & p, Overlay const & ov, Hop const & h, Scratch & sc,
               std::vector<VertexId> & out)
{
  if (h.level == 0)
  {
    out.push_back(h.v);
    return true;
  }
  size_t const li = h.level - 1;
  uint32_t const c = p.cell[li][h.u];
  auto & S = sc.Fwd();
  auto & done = sc.Done(0);
  uint32_t const st = sc.NewStamp();
  S.heap.clear();
  S.stamp[h.u] = st;
  S.dist[h.u] = 0;
  S.parent[h.u] = kNone;
  S.parentLevel[h.u] = 0;
  HeapPush(S.heap, 0, h.u);
  auto relax = [&](VertexId y, double d, VertexId par, uint8_t lvl)
  {
    if (S.stamp[y] != st || d < S.dist[y])
    {
      S.stamp[y] = st;
      S.dist[y] = d;
      S.parent[y] = par;
      S.parentLevel[y] = lvl;
      HeapPush(S.heap, d, y);
    }
  };
  while (!S.heap.empty())
  {
    Item const it = HeapPop(S.heap);
    VertexId const x = it.v;
    if (done[x] == st)
      continue;
    done[x] = st;
    if (x == h.v)
      break;
    double const d = it.d;
    if (li == 0)
    {
      for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
        if (p.cell[0][g.fwdTo[e]] == c && g.fwdW[e] < kInf)
          relax(g.fwdTo[e], d + g.fwdW[e], x, 0);
      continue;
    }
    LevelOverlay const & C = ov.levels[li - 1];
    uint32_t const cc = p.cell[li - 1][x];
    uint64_t const cx0 = C.exitOff[cc], cnEx = C.exitOff[cc + 1] - cx0;
    if (C.IsEntry(x))
    {
      uint64_t const base = C.matOff[cc] + C.EntryIdx(x, cc) * cnEx;
      for (uint64_t j = 0; j < cnEx; ++j)
        if (C.mat[base + j] < kInf)
          relax(C.exits[cx0 + j], d + C.mat[base + j], x, static_cast<uint8_t>(li));
    }
    if (C.IsExit(x))
    {
      for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
      {
        VertexId const y = g.fwdTo[e];
        if (p.cell[li][y] == c && p.cell[li - 1][y] != cc && g.fwdW[e] < kInf)
          relax(y, d + g.fwdW[e], x, 0);
      }
    }
  }
  if (done[h.v] != st)
    return false;
  std::vector<Hop> sub;
  for (VertexId y = h.v; y != h.u; y = S.parent[y])
    sub.push_back({S.parent[y], y, S.parentLevel[y]});
  std::reverse(sub.begin(), sub.end());
  for (auto const & s2 : sub)
    if (!UnpackHop(g, p, ov, s2, sc, out))
      return false;
  return true;
}
}  // namespace

QueryResult OverlayQuery(Graph const & g, Partition const & p, Overlay const & ov, VertexId s, VertexId t,
                         Scratch & sc, bool wantPath)
{
  return OverlayQueryMulti(g, p, ov, Seeds{{s, 0.0}}, Seeds{{t, 0.0}}, sc, wantPath);
}

QueryResult OverlayQueryMulti(Graph const & g, Partition const & p, Overlay const & ov, Seeds const & sources,
                              Seeds const & targets, Scratch & sc, bool wantPath)
{
  size_t const levels = p.Levels();
  // A vertex is searched at the highest level whose cell holds no source and no target.
  std::vector<std::vector<uint32_t>> endCells(levels);
  for (size_t li = 0; li < levels; ++li)
  {
    for (auto const & sd : sources)
      endCells[li].push_back(p.cell[li][sd.first]);
    for (auto const & sd : targets)
      endCells[li].push_back(p.cell[li][sd.first]);
    std::sort(endCells[li].begin(), endCells[li].end());
    endCells[li].erase(std::unique(endCells[li].begin(), endCells[li].end()), endCells[li].end());
  }
  auto queryLevel = [&](VertexId v) -> size_t
  {
    for (size_t li = levels; li-- > 0;)
    {
      uint32_t const c = p.cell[li][v];
      if (!std::binary_search(endCells[li].begin(), endCells[li].end(), c))
        return li + 1;
    }
    return 0;
  };

  auto scanF = [&](VertexId x, double d, auto & relax)
  {
    size_t const lvl = queryLevel(x);
    if (lvl == 0)
    {
      for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
        if (g.fwdW[e] < kInf)
          relax(g.fwdTo[e], d + g.fwdW[e], x, 0);
      return;
    }
    size_t const li = lvl - 1;
    LevelOverlay const & L = ov.levels[li];
    uint32_t const c = p.cell[li][x];
    if (L.IsEntry(x))
    {
      uint64_t const x0 = L.exitOff[c], nEx = L.exitOff[c + 1] - x0;
      uint64_t const base = L.matOff[c] + L.EntryIdx(x, c) * nEx;
      for (uint64_t j = 0; j < nEx; ++j)
        if (L.mat[base + j] < kInf)
          relax(L.exits[x0 + j], d + L.mat[base + j], x, static_cast<uint8_t>(lvl));
    }
    for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
      if (p.cell[li][g.fwdTo[e]] != c && g.fwdW[e] < kInf)
        relax(g.fwdTo[e], d + g.fwdW[e], x, 0);
  };
  auto scanB = [&](VertexId x, double d, auto & relax)
  {
    size_t const lvl = queryLevel(x);
    if (lvl == 0)
    {
      for (uint64_t e = g.revOff[x]; e < g.revOff[x + 1]; ++e)
        if (g.revW[e] < kInf)
          relax(g.revFrom[e], d + g.revW[e], x, 0);
      return;
    }
    size_t const li = lvl - 1;
    LevelOverlay const & L = ov.levels[li];
    uint32_t const c = p.cell[li][x];
    if (L.IsExit(x))
    {
      uint64_t const e0 = L.entryOff[c], nEn = L.entryOff[c + 1] - e0;
      uint64_t const nEx = L.exitOff[c + 1] - L.exitOff[c];
      uint64_t const col = L.ExitIdx(x, c);
      for (uint64_t i = 0; i < nEn; ++i)
      {
        double const w = L.mat[L.matOff[c] + i * nEx + col];
        if (w < kInf)
          relax(L.entries[e0 + i], d + w, x, static_cast<uint8_t>(lvl));
      }
    }
    for (uint64_t e = g.revOff[x]; e < g.revOff[x + 1]; ++e)
      if (p.cell[li][g.revFrom[e]] != c && g.revW[e] < kInf)
        relax(g.revFrom[e], d + g.revW[e], x, 0);
  };
  QueryResult r = Bidirectional(g, sources, targets, sc, scanF, scanB);
  if (!wantPath || r.weight >= kInf || r.path.empty())
  {
    r.path.clear();
    return r;
  }
  VertexId const meet = r.path.front();
  std::vector<Hop> hops;
  VertexId first = meet;
  for (VertexId v = meet; sc.Fwd().parent[v] != kNone; v = sc.Fwd().parent[v])
  {
    hops.push_back({sc.Fwd().parent[v], v, sc.Fwd().parentLevel[v]});
    first = sc.Fwd().parent[v];
  }
  std::reverse(hops.begin(), hops.end());
  for (VertexId v = meet; sc.Bwd().parent[v] != kNone; v = sc.Bwd().parent[v])
    hops.push_back({v, sc.Bwd().parent[v], sc.Bwd().parentLevel[v]});
  r.path.assign(1, first);
  for (auto const & h : hops)
  {
    if (!UnpackHop(g, p, ov, h, sc, r.path))
    {
      r.path.clear();  // signals an unpacking failure to the caller
      break;
    }
  }
  return r;
}

OverlayStorage MeasureStorage(Graph const & g, Partition const & p, Overlay const & ov)
{
  OverlayStorage s;
  for (auto const & L : ov.levels)
  {
    s.matrixEntries += L.mat.size();
    for (float w : L.mat)
      s.finiteEntries += (w < kInf);
    s.boundaryIds += L.entries.size() + L.exits.size();
  }
  s.cellIdBytes = static_cast<uint64_t>(g.n) * p.Levels() * sizeof(uint32_t);
  return s;
}
}  // namespace plus_overlay
