#include "routing/plus_overlay/overlay.hpp"

#include <algorithm>
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
  std::vector<uint64_t> pos(revOff.begin(), revOff.end() - 1);
  for (uint32_t u = 0; u < n; ++u)
  {
    for (uint64_t e = fwdOff[u]; e < fwdOff[u + 1]; ++e)
    {
      uint64_t const r = pos[fwdTo[e]]++;
      revFrom[r] = u;
      revW[r] = fwdW[e];
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
  Bisector(Graph const & g, PartitionParams const & params, Partition & p) : m_g(g), m_params(params), m_p(p)
  {
    m_memb.assign(g.n, 0);
    m_rank.assign(g.n, 0);
    double latSum = 0;
    for (uint32_t v = 0; v < g.n; ++v)
      latSum += g.lat[v];
    double const k = std::cos((g.n ? latSum / g.n : 0.0) * M_PI / 180.0);
    m_x.resize(g.n);
    m_y.resize(g.n);
    for (uint32_t v = 0; v < g.n; ++v)
    {
      m_x[v] = static_cast<float>(g.lon[v] * k);
      m_y[v] = g.lat[v];
    }
  }

  void Run(std::vector<VertexId> verts, uint64_t parentSize)
  {
    auto const & cs = m_params.cellSizes;
    size_t const m = verts.size();
    for (size_t li = 0; li < cs.size(); ++li)
    {
      if (m <= cs[li] && parentSize > cs[li])
      {
        uint32_t const id = m_p.numCells[li]++;
        for (VertexId v : verts)
          m_p.cell[li][v] = id;
      }
    }
    if (m <= cs[0] || m < 2)
      return;

    ++m_stamp;
    for (VertexId v : verts)
      m_memb[v] = m_stamp;

    size_t const kLo = std::max<size_t>(1, static_cast<size_t>(std::ceil(m_params.minBalance * m)));
    size_t const kHi = std::min<size_t>(m - 1, static_cast<size_t>(std::floor((1.0 - m_params.minBalance) * m)));

    std::vector<std::pair<float, VertexId>> kv(m);
    std::vector<int64_t> diff(m + 2);
    std::vector<VertexId> bestOrder;
    int64_t bestCut = std::numeric_limits<int64_t>::max();
    size_t bestK = m / 2;

    for (int d = 0; d < m_params.directions; ++d)
    {
      double const a = M_PI * d / m_params.directions;
      float const c = static_cast<float>(std::cos(a));
      float const s = static_cast<float>(std::sin(a));
      for (size_t i = 0; i < m; ++i)
        kv[i] = {m_x[verts[i]] * c + m_y[verts[i]] * s, verts[i]};
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
          if (m_memb[w] != m_stamp)
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
          if (bestOrder.size() != m || d != m_bestDir)
          {
            bestOrder.resize(m);
            for (size_t i = 0; i < m; ++i)
              bestOrder[i] = kv[i].second;
            m_bestDir = d;
          }
        }
      }
    }
    m_bestDir = -1;

    std::vector<VertexId> left(bestOrder.begin(), bestOrder.begin() + bestK);
    std::vector<VertexId> right(bestOrder.begin() + bestK, bestOrder.end());
    std::vector<VertexId>().swap(verts);
    std::vector<VertexId>().swap(bestOrder);
    Run(std::move(left), m);
    Run(std::move(right), m);
  }

private:
  Graph const & m_g;
  PartitionParams const & m_params;
  Partition & m_p;
  std::vector<uint32_t> m_memb, m_rank;
  std::vector<float> m_x, m_y;
  uint32_t m_stamp = 0;
  int m_bestDir = -1;
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
  b.Run(std::move(all), std::numeric_limits<uint64_t>::max());
  return p;
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

void BuildBoundaries(Graph const & g, Partition const & p, Overlay & ov)
{
  ov.levels.assign(p.Levels(), {});
  for (size_t li = 0; li < p.Levels(); ++li)
  {
    auto const & cell = p.cell[li];
    LevelOverlay & L = ov.levels[li];
    uint32_t const nc = p.numCells[li];
    std::vector<uint8_t> role(g.n, 0);  // bit 1 entry, bit 2 exit
    L.entryOff.assign(nc + 1, 0);
    L.exitOff.assign(nc + 1, 0);
    for (uint32_t v = 0; v < g.n; ++v)
    {
      for (uint64_t e = g.revOff[v]; e < g.revOff[v + 1]; ++e)
        if (cell[g.revFrom[e]] != cell[v])
        {
          role[v] |= 1;
          break;
        }
      for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
        if (cell[g.fwdTo[e]] != cell[v])
        {
          role[v] |= 2;
          break;
        }
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
    L.entryIdx.assign(g.n, kNone);
    L.exitIdx.assign(g.n, kNone);
    std::vector<uint64_t> ep(L.entryOff.begin(), L.entryOff.end() - 1);
    std::vector<uint64_t> xp(L.exitOff.begin(), L.exitOff.end() - 1);
    for (uint32_t v = 0; v < g.n; ++v)
    {
      uint32_t const c = cell[v];
      if (role[v] & 1)
      {
        L.entryIdx[v] = static_cast<uint32_t>(ep[c] - L.entryOff[c]);
        L.entries[ep[c]++] = v;
      }
      if (role[v] & 2)
      {
        L.exitIdx[v] = static_cast<uint32_t>(xp[c] - L.exitOff[c]);
        L.exits[xp[c]++] = v;
      }
    }
    L.matOff.assign(nc + 1, 0);
    for (uint32_t c = 0; c < nc; ++c)
      L.matOff[c + 1] = L.matOff[c] + (L.entryOff[c + 1] - L.entryOff[c]) * (L.exitOff[c + 1] - L.exitOff[c]);
    L.mat.assign(L.matOff[nc], kInf);
  }
}

namespace
{
// One cell: Dijkstra from every entry, restricted to the cell. Level 1 runs on original edges,
// higher levels on the children's cliques plus original edges between children.
void ComputeCell(Graph const & g, Partition const & p, Overlay & ov, size_t li, uint32_t c, Scratch & sc)
{
  LevelOverlay & L = ov.levels[li];
  auto const & cellOf = p.cell[li];
  uint64_t const e0 = L.entryOff[c], nEn = L.entryOff[c + 1] - e0;
  uint64_t const x0 = L.exitOff[c], nEx = L.exitOff[c + 1] - x0;
  if (nEn == 0 || nEx == 0)
    return;

  auto & S = sc.Fwd();
  auto & done = sc.Done(0);
  for (uint64_t i = 0; i < nEn; ++i)
  {
    uint32_t const st = sc.NewStamp();
    VertexId const src = L.entries[e0 + i];
    S.heap.clear();
    S.stamp[src] = st;
    S.dist[src] = 0;
    HeapPush(S.heap, 0, src);
    uint64_t remaining = nEx;

    auto relax = [&](VertexId y, double d)
    {
      if (S.stamp[y] != st || d < S.dist[y])
      {
        S.stamp[y] = st;
        S.dist[y] = d;
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
      if (L.exitIdx[x] != kNone && --remaining == 0)
        break;
      double const d = it.d;
      if (li == 0)
      {
        for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
          if (cellOf[g.fwdTo[e]] == c && g.fwdW[e] < kInf)
            relax(g.fwdTo[e], d + g.fwdW[e]);
        continue;
      }
      LevelOverlay const & C = ov.levels[li - 1];
      uint32_t const cc = p.cell[li - 1][x];
      uint64_t const cx0 = C.exitOff[cc], cnEx = C.exitOff[cc + 1] - cx0;
      if (C.entryIdx[x] != kNone)
      {
        uint64_t const base = C.matOff[cc] + C.entryIdx[x] * cnEx;
        for (uint64_t j = 0; j < cnEx; ++j)
          if (C.mat[base + j] < kInf)
            relax(C.exits[cx0 + j], d + C.mat[base + j]);
      }
      if (C.exitIdx[x] != kNone)
      {
        for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
        {
          VertexId const y = g.fwdTo[e];
          if (cellOf[y] == c && p.cell[li - 1][y] != cc && g.fwdW[e] < kInf)
            relax(y, d + g.fwdW[e]);
        }
      }
    }
    for (uint64_t j = 0; j < nEx; ++j)
    {
      VertexId const y = L.exits[x0 + j];
      L.mat[L.matOff[c] + i * nEx + j] = (done[y] == st) ? S.dist[y] : kInf;
    }
  }
}
}  // namespace

std::vector<double> Customize(Graph const & g, Partition const & p, Overlay & ov, unsigned threads)
{
  threads = std::max(1u, threads);
  std::vector<double> seconds;
  std::vector<std::unique_ptr<Scratch>> scratch;
  for (unsigned t = 0; t < threads; ++t)
    scratch.push_back(std::make_unique<Scratch>(g.n));
  for (size_t li = 0; li < p.Levels(); ++li)
  {
    auto const t0 = std::chrono::steady_clock::now();
    uint32_t const nc = p.numCells[li];
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; ++t)
    {
      pool.emplace_back([&, t]()
      {
        for (uint32_t c = t; c < nc; c += threads)
          ComputeCell(g, p, ov, li, c, *scratch[t]);
      });
    }
    for (auto & th : pool)
      th.join();
    seconds.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  }
  return seconds;
}

uint64_t Recustomize(Graph const & g, Partition const & p, Overlay & ov, std::vector<VertexId> const & touched,
                     Scratch & sc)
{
  uint64_t count = 0;
  for (size_t li = 0; li < p.Levels(); ++li)
  {
    std::vector<uint32_t> cells;
    for (VertexId v : touched)
      cells.push_back(p.cell[li][v]);
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    for (uint32_t c : cells)
      ComputeCell(g, p, ov, li, c, sc);
    count += cells.size();
  }
  return count;
}

// Queries -----------------------------------------------------------------------------------------

namespace
{
template <typename ScanF, typename ScanB>
QueryResult Bidirectional(Graph const & g, VertexId s, VertexId t, Scratch & sc, ScanF && scanF, ScanB && scanB)
{
  QueryResult r;
  auto & F = sc.Fwd();
  auto & B = sc.Bwd();
  auto & doneF = sc.Done(0);
  auto & doneB = sc.Done(1);
  uint32_t const st = sc.NewStamp();
  F.heap.clear();
  B.heap.clear();
  F.stamp[s] = st;
  F.dist[s] = 0;
  F.parent[s] = kNone;
  B.stamp[t] = st;
  B.dist[t] = 0;
  B.parent[t] = kNone;
  HeapPush(F.heap, 0, s);
  HeapPush(B.heap, 0, t);
  double mu = (s == t) ? 0.0 : kInf;
  VertexId meet = (s == t) ? s : kNone;

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
  QueryResult r = Bidirectional(g, s, t, sc, scanF, scanB);
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
    if (C.entryIdx[x] != kNone)
    {
      uint64_t const base = C.matOff[cc] + C.entryIdx[x] * cnEx;
      for (uint64_t j = 0; j < cnEx; ++j)
        if (C.mat[base + j] < kInf)
          relax(C.exits[cx0 + j], d + C.mat[base + j], x, static_cast<uint8_t>(li));
    }
    if (C.exitIdx[x] != kNone)
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
  size_t const levels = p.Levels();
  uint32_t cs[16], ct[16];
  for (size_t li = 0; li < levels; ++li)
  {
    cs[li] = p.cell[li][s];
    ct[li] = p.cell[li][t];
  }
  auto queryLevel = [&](VertexId v) -> size_t
  {
    for (size_t li = levels; li-- > 0;)
    {
      uint32_t const c = p.cell[li][v];
      if (c != cs[li] && c != ct[li])
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
    if (L.entryIdx[x] != kNone)
    {
      uint64_t const x0 = L.exitOff[c], nEx = L.exitOff[c + 1] - x0;
      uint64_t const base = L.matOff[c] + L.entryIdx[x] * nEx;
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
    if (L.exitIdx[x] != kNone)
    {
      uint64_t const e0 = L.entryOff[c], nEn = L.entryOff[c + 1] - e0;
      uint64_t const nEx = L.exitOff[c + 1] - L.exitOff[c];
      uint64_t const col = L.exitIdx[x];
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
  QueryResult r = Bidirectional(g, s, t, sc, scanF, scanB);
  if (!wantPath || r.weight == kInf || r.path.empty())
  {
    r.path.clear();
    return r;
  }
  VertexId const meet = r.path.front();
  std::vector<Hop> hops;
  for (VertexId v = meet; v != s; v = sc.Fwd().parent[v])
    hops.push_back({sc.Fwd().parent[v], v, sc.Fwd().parentLevel[v]});
  std::reverse(hops.begin(), hops.end());
  for (VertexId v = meet; v != t; v = sc.Bwd().parent[v])
    hops.push_back({v, sc.Bwd().parent[v], sc.Bwd().parentLevel[v]});
  r.path.assign(1, s);
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
    for (double w : L.mat)
      s.finiteEntries += (w < kInf);
    s.boundaryIds += L.entries.size() + L.exits.size();
  }
  s.cellIdBytes = static_cast<uint64_t>(g.n) * p.Levels() * sizeof(uint32_t);
  return s;
}
}  // namespace plus_overlay
