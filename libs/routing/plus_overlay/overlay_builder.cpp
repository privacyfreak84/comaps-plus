#include "routing/plus_overlay/overlay_builder.hpp"

#include "routing/plus_overlay/dynamic_edges.hpp"
#include "routing/plus_overlay/overlay.hpp"
#include "routing/plus_overlay/overlay_file.hpp"
#include "routing/plus_overlay/via_way_automaton.hpp"

#include "routing/cross_mwm_index_graph.hpp"
#include "routing/data_source.hpp"
#include "routing/edge_estimator.hpp"
#include "routing/index_graph.hpp"
#include "routing/index_graph_loader.hpp"
#include "routing/restriction_loader.hpp"
#include "routing/routing_options.hpp"
#include "routing/segment.hpp"

#include "routing_common/car_model.hpp"
#include "routing_common/num_mwm_id.hpp"

#include "indexer/data_source.hpp"

#include "platform/platform.hpp"

#include "base/file_name_utils.hpp"
#include "base/logging.hpp"

#include <malloc.h>

#include <algorithm>
#include <functional>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>
#include <tuple>
#include <unordered_map>

namespace
{
// Hands memory freed by a finished stage back to the system, so the phone counts only what the build
// still uses (the allocator otherwise keeps it, and the app looks bigger than it is).
void ReleaseFreedMemory()
{
#if defined(__GLIBC__)
  malloc_trim(0);
#elif defined(__ANDROID__) && defined(M_PURGE) && __ANDROID_API__ >= 28
  mallopt(M_PURGE, 0);  // older Android builds: the allocator releases freed pages on its own schedule
#endif
}
}  // namespace

namespace plus_overlay
{
namespace
{
using Clock = std::chrono::steady_clock;
double Since(Clock::time_point t0)
{
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

// VmRSS / VmHWM in kB (0 where /proc is unavailable).
void ReadMem(uint64_t & rssKb, uint64_t & peakKb)
{
  rssKb = peakKb = 0;
  std::ifstream f("/proc/self/status");
  std::string line;
  while (std::getline(f, line))
  {
    if (line.rfind("VmRSS:", 0) == 0)
      rssKb = std::strtoull(line.c_str() + 6, nullptr, 10);
    if (line.rfind("VmHWM:", 0) == 0)
      peakKb = std::strtoull(line.c_str() + 6, nullptr, 10);
  }
}

std::string Mem()
{
  uint64_t rss, peak;
  ReadMem(rss, peak);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "rss %.0f MB, peak %.0f MB", rss / 1024.0, peak / 1024.0);
  return buf;
}

struct Cancelled
{
};

// Progress over the build, by the share of time each stage took on a desktop (UP: extraction ~35%,
// partition ~35%, customize ~30%).
struct Reporter
{
  BuildOptions const & o;
  void Log(std::string const & s) const
  {
    if (o.log)
      o.log(s);
    else
      LOG(LINFO, ("Tabula overlay build:", s));
  }
  void Progress(double f, std::string const & stage) const
  {
    if (o.progress)
      o.progress(std::clamp(f, 0.0, 1.0), stage);
  }
  bool IsCancelled() const { return o.cancelled && o.cancelled(); }
  void CheckCancelled() const
  {
    if (IsCancelled())
      throw Cancelled();
  }
};

struct LeanGraph
{
  std::vector<uint32_t> featBase;  // segment index of the first segment of feature f
  std::vector<uint64_t> keptBits;  // per raw slot (2 * segment + direction)
  std::vector<uint32_t> keptRank;  // kept slots before each 64-slot word
  std::vector<uint32_t> origId;    // compressed vertex -> raw slot (base vertices only)
  std::vector<uint32_t> copyOrig;  // via-way copies: compressed id of the original
  std::vector<uint32_t> copyState; // and their automaton state
  uint32_t baseN = 0;
  Graph g;                       // forward edges + lat/lon
  std::vector<uint8_t> interior; // per edge: the chain behind it passed at least one non-kept segment
  // Per base vertex: RoutingOptions road types (tolls, motorways, ...) of the roads its edges step onto,
  // chains included. A cell whose vertices have none of the avoided types has the same cliques with and
  // without those avoid options (FileContents::roads).
  std::vector<uint8_t> roads;
  // Base vertices the search must be able to stand on at any level: ends of time-dependent edges.
  std::vector<uint32_t> allLevels;
  uint64_t rawSlots = 0, rawValid = 0, rawEdges = 0, dynamicEdges = 0;
  // Turn-arounds (routing::IndexGraph::IsPlusTurnAround), in raw slots: from, number of pieces k, the k
  // pieces, to; one after the other. LeanExtract finds them and keeps their segments;
  // LeanExpandTurnArounds turns them into copies.
  std::vector<uint32_t> turnPaths;
  size_t numTurnPaths = 0;
  uint64_t turnsByPieces[5] = {0, 0, 0, 0, 0};  // found turn-arounds by number of pieces (4: 4 or more)
  std::vector<FileContents::TurnAround> turnArounds;

  bool Kept(uint32_t v) const { return (keptBits[v >> 6] >> (v & 63)) & 1; }
  uint32_t Rank(uint32_t v) const
  {
    uint64_t const word = keptBits[v >> 6] & ((uint64_t(1) << (v & 63)) - 1);
    return keptRank[v >> 6] + static_cast<uint32_t>(__builtin_popcountll(word));
  }
  bool Has(routing::Segment const & s) const
  {
    auto const f = s.GetFeatureId();
    return f + 1 < featBase.size() && s.GetSegmentIdx() < featBase[f + 1] - featBase[f];
  }
  uint32_t Slot(routing::Segment const & s) const
  {
    return 2 * (featBase[s.GetFeatureId()] + s.GetSegmentIdx()) + (s.IsForward() ? 0 : 1);
  }
  uint32_t FeatureOfSlot(uint32_t v) const
  {
    return static_cast<uint32_t>(std::upper_bound(featBase.begin(), featBase.end(), v / 2) - featBase.begin()) - 1;
  }
};

// What LeanExtract treats specially besides the border transitions.
struct Special
{
  // Time-dependent edges (conditional access) stay out of the graph: the router evaluates them per route.
  routing::RoadAccess const * access = nullptr;

  bool Dynamic(routing::Segment const & u, routing::Segment const & v) const
  {
    return access && IsTimeDependentEdge(*access, u, v);
  }
};

// The partition needs only the graph's shape, not its weights: they wait in a file meanwhile, which lowers
// the build's peak memory (the partition stage is the largest). The file is deleted in any case.
class ParkedWeights
{
public:
  explicit ParkedWeights(std::string path) : m_path(std::move(path)) {}
  ~ParkedWeights() { std::remove(m_path.c_str()); }

  bool Park(Graph & g)
  {
    std::ofstream out(m_path, std::ios::binary | std::ios::trunc);
    m_edges = g.fwdW.size();
    m_hasK = !g.fwdK.empty();
    m_hasR = !g.fwdR.empty();
    out.write(reinterpret_cast<char const *>(g.fwdW.data()), static_cast<std::streamsize>(m_edges * sizeof(double)));
    if (m_hasK)
      out.write(reinterpret_cast<char const *>(g.fwdK.data()), static_cast<std::streamsize>(m_edges));
    if (m_hasR)
      out.write(reinterpret_cast<char const *>(g.fwdR.data()), static_cast<std::streamsize>(m_edges));
    if (!out.good())
      return false;  // keep them in memory
    std::vector<double>().swap(g.fwdW);
    std::vector<uint8_t>().swap(g.fwdK);
    std::vector<uint8_t>().swap(g.fwdR);
    m_parked = true;
    return true;
  }

  bool Restore(Graph & g)
  {
    if (!m_parked)
      return true;
    std::ifstream in(m_path, std::ios::binary);
    g.fwdW.resize(m_edges);
    in.read(reinterpret_cast<char *>(g.fwdW.data()), static_cast<std::streamsize>(m_edges * sizeof(double)));
    if (m_hasK)
    {
      g.fwdK.resize(m_edges);
      in.read(reinterpret_cast<char *>(g.fwdK.data()), static_cast<std::streamsize>(m_edges));
    }
    if (m_hasR)
    {
      g.fwdR.resize(m_edges);
      in.read(reinterpret_cast<char *>(g.fwdR.data()), static_cast<std::streamsize>(m_edges));
    }
    m_parked = false;
    return in.good();
  }

private:
  std::string m_path;
  uint64_t m_edges = 0;
  bool m_hasK = false;
  bool m_hasR = false;
  bool m_parked = false;
};

// Z-order (Morton) number of a point, 16 bits per axis within the box of all points given.
uint32_t ZOrder(double x, double y, double minX, double minY, double spanX, double spanY)
{
  auto spread = [](uint32_t v)
  {
    v &= 0xFFFF;
    v = (v | (v << 8)) & 0x00FF00FF;
    v = (v | (v << 4)) & 0x0F0F0F0F;
    v = (v | (v << 2)) & 0x33333333;
    v = (v | (v << 1)) & 0x55555555;
    return v;
  };
  auto cell = [](double v, double mn, double span)
  { return static_cast<uint32_t>(std::clamp(span > 0 ? (v - mn) / span * 65535.0 : 0.0, 0.0, 65535.0)); };
  return spread(cell(x, minX, spanX)) | (spread(cell(y, minY, spanY)) << 1);
}

// Two walks over CoMaps' IndexGraph: degrees, then chains between kept segments.
//
// Both walks go through the roads in Z-order of their first point, not by feature id: a segment's edges need
// the geometry of the roads it meets, CoMaps keeps only the last 10,000 roads it read, and feature ids say
// little about where a road is. Going by place, the roads met are mostly ones just read. The result doesn't
// depend on the order.
void LeanExtract(routing::IndexGraph & ig, routing::NumMwmId mwmId, std::vector<routing::Segment> const & forced,
                 std::vector<std::vector<uint32_t>> const & viaWay, Special const & sp, LeanGraph & lg,
                 Reporter const & rep)
{
  auto const t0 = Clock::now();
  std::vector<uint32_t> roads;
  ig.ForEachRoad([&](uint32_t f, routing::RoadJointIds const &) { roads.push_back(f); });
  // A road that crosses the border but has no junction inside the map has no joints, so ForEachRoad
  // skips it; CoMaps still routes along it (in at one border transition, out at the next).
  for (auto const & s : forced)
    roads.push_back(s.GetFeatureId());
  std::sort(roads.begin(), roads.end());
  roads.erase(std::unique(roads.begin(), roads.end()), roads.end());
  uint32_t const maxF = roads.empty() ? 0 : roads.back();
  std::vector<uint32_t> segs(maxF + 1, 0);
  std::vector<uint8_t> oneway(maxF + 1, 0);
  std::vector<float> firstLat(roads.size(), 0), firstLon(roads.size(), 0);
  for (size_t r = 0; r < roads.size(); ++r)
  {
    uint32_t const f = roads[r];
    auto const & road = ig.GetRoadGeometry(f);
    if (!road.IsValid() || road.GetPointsCount() < 2)
      continue;
    segs[f] = road.GetPointsCount() - 1;
    oneway[f] = road.IsOneWay() ? 1 : 0;
    auto const & pt = road.GetPoint(0);
    firstLat[r] = static_cast<float>(pt.m_lat);
    firstLon[r] = static_cast<float>(pt.m_lon);
  }
  // The roads by place (see above).
  std::vector<uint32_t> byPlace;
  {
    double minLat = 1e9, maxLat = -1e9, minLon = 1e9, maxLon = -1e9;
    for (size_t r = 0; r < roads.size(); ++r)
    {
      if (segs[roads[r]] == 0)
        continue;
      minLat = std::min<double>(minLat, firstLat[r]);
      maxLat = std::max<double>(maxLat, firstLat[r]);
      minLon = std::min<double>(minLon, firstLon[r]);
      maxLon = std::max<double>(maxLon, firstLon[r]);
    }
    std::vector<std::pair<uint32_t, uint32_t>> keyed;
    keyed.reserve(roads.size());
    for (size_t r = 0; r < roads.size(); ++r)
      if (segs[roads[r]] != 0)
        keyed.emplace_back(ZOrder(firstLon[r], firstLat[r], minLon, minLat, maxLon - minLon, maxLat - minLat),
                           roads[r]);
    std::sort(keyed.begin(), keyed.end());
    byPlace.reserve(keyed.size());
    for (auto const & kf : keyed)
      byPlace.push_back(kf.second);
  }
  std::vector<float>().swap(firstLat);
  std::vector<float>().swap(firstLon);
  lg.featBase.assign(maxF + 2, 0);
  for (uint32_t f = 0; f <= maxF; ++f)
    lg.featBase[f + 1] = lg.featBase[f] + segs[f];
  uint32_t const n = 2 * lg.featBase[maxF + 1];
  lg.rawSlots = n;

  // Calls fn for every valid directed segment; polls cancellation and reports progress over |span|.
  auto forEachValid = [&](double from, double span, std::string const & stage, auto && fn)
  {
    size_t done = 0;
    for (uint32_t f : byPlace)
    {
      if ((++done & 4095) == 0)
      {
        rep.CheckCancelled();
        rep.Progress(from + span * done / byPlace.size(), stage);
      }
      for (uint32_t i = 0; i < segs[f]; ++i)
        for (bool fwd : {true, false})
          if (fwd || !oneway[f])
            fn(routing::Segment(mwmId, f, i, fwd));
    }
  };

  // Pass 1: degrees.
  std::vector<uint8_t> indeg(n, 0), outdeg(n, 0), valid(n, 0);
  std::vector<uint8_t> special(n, 0);  // kept, and a boundary at every level
  routing::IndexGraph::SegmentEdgeListT edges;
  // Depth-first over the pieces after |u|: every run of up to kPlusTurnAroundMaxPieces segments off u's road,
  // at most kPlusTurnAroundMaxConnectorM long, followed by a segment that completes a turn-around.
  std::vector<routing::IndexGraph::SegmentEdgeListT> onward(routing::IndexGraph::kPlusTurnAroundMaxPieces);
  std::vector<routing::Segment> pieces;
  auto pieceLen = [&](routing::Segment const & p)
  {
    return ms::DistanceOnEarth(ig.GetPoint(p, false /* front */), ig.GetPoint(p, true /* front */));
  };
  std::function<void(routing::Segment const &, double)> dive = [&](routing::Segment const & u, double len)
  {
    size_t const depth = pieces.size();
    auto & next = onward[depth - 1];
    next.clear();
    ig.GetEdgeList(pieces.back(), true /* isOutgoing */, true /* useRoutingOptions */, next);
    for (size_t i = 0; i < next.size(); ++i)
    {
      routing::Segment const w = onward[depth - 1][i].GetTarget();
      uint32_t const fw = w.GetFeatureId();
      if (fw > maxF || w == u || sp.Dynamic(pieces.back(), w) ||
          std::find(pieces.begin(), pieces.end(), w) != pieces.end())
        continue;
      if (ig.IsPlusTurnAround(u, pieces.data(), depth, w))
      {
        ++lg.turnsByPieces[std::min<size_t>(depth, 4)];
        lg.turnPaths.push_back(lg.Slot(u));
        lg.turnPaths.push_back(static_cast<uint32_t>(depth));
        for (auto const & p : pieces)
          lg.turnPaths.push_back(lg.Slot(p));
        lg.turnPaths.push_back(lg.Slot(w));
        ++lg.numTurnPaths;
        continue;
      }
      if (fw == u.GetFeatureId() || depth >= routing::IndexGraph::kPlusTurnAroundMaxPieces)
        continue;
      double const l = len + pieceLen(w);
      if (l > routing::IndexGraph::kPlusTurnAroundMaxConnectorM)
        continue;
      pieces.push_back(w);
      dive(u, l);
      pieces.pop_back();
    }
  };
  auto findTurnArounds = [&](routing::Segment const & u, routing::Segment const & c)
  {
    double const l = pieceLen(c);
    if (l > routing::IndexGraph::kPlusTurnAroundMaxConnectorM)
      return;
    pieces.assign(1, c);
    dive(u, l);
  };
  ig.SetPlusEdgesOnly(true);  // this walk only counts edges
  forEachValid(0.0, 0.12, "reading roads", [&](routing::Segment const & s)
  {
    uint32_t const v = lg.Slot(s);
    valid[v] = 1;
    ++lg.rawValid;
    edges.clear();
    ig.GetEdgeList(s, true /* isOutgoing */, true /* useRoutingOptions */, edges);
    outdeg[v] = static_cast<uint8_t>(std::min<size_t>(255, edges.size()));
    lg.rawEdges += edges.size();
    uint32_t const f = s.GetFeatureId();
    for (auto const & e : edges)
    {
      routing::Segment const c = e.GetTarget();
      uint32_t const t = lg.Slot(c);
      if (indeg[t] < 2)
        ++indeg[t];
      if (sp.Dynamic(s, c))
      {
        special[v] = special[t] = 1;
        ++lg.dynamicEdges;
        continue;
      }
      // Turn-arounds starting on |s| with |c| as the first piece.
      if (c.GetFeatureId() != f && c.GetFeatureId() <= maxF)
        findTurnArounds(s, c);
    }
  });
  ig.SetPlusEdgesOnly(false);
  {
    std::ostringstream os;
    os << "pass 1 (degrees): " << lg.rawValid << " directed segments, " << lg.rawEdges << " edges in " << Since(t0)
       << " s, " << Mem();
    rep.Log(os.str());
  }

  // Kept slots: everything but chain interiors and isolated segments, plus forced ones.
  lg.keptBits.assign((n + 63) / 64 + 1, 0);
  auto keep = [&](uint32_t v) { lg.keptBits[v >> 6] |= uint64_t(1) << (v & 63); };
  for (uint32_t v = 0; v < n; ++v)
  {
    if (!valid[v])
      continue;
    bool const isolated = indeg[v] == 0 && outdeg[v] == 0;
    bool const interiorV = indeg[v] == 1 && outdeg[v] == 1;
    if ((!isolated && !interiorV) || special[v])
      keep(v);
  }
  for (auto const & s : forced)
  {
    if (!lg.Has(s))
      continue;
    uint32_t const v = lg.Slot(s);
    if (valid[v])
      keep(v);
  }
  for (auto const & pat : viaWay)
    for (uint32_t f : pat)
      if (f + 1 < lg.featBase.size())
        for (uint32_t v = 2 * lg.featBase[f]; v < 2 * lg.featBase[f + 1]; ++v)
          if (valid[v])
            keep(v);
  // The segments of a turn-around are vertices, so its steps are plain edges.
  for (size_t i = 0; i < lg.turnPaths.size();)
  {
    uint32_t const k = lg.turnPaths[i + 1];
    keep(lg.turnPaths[i]);
    for (uint32_t j = 0; j <= k; ++j)
      keep(lg.turnPaths[i + 2 + j]);
    i += 3 + k;
  }
  lg.keptRank.assign(lg.keptBits.size(), 0);
  for (size_t w = 1; w < lg.keptBits.size(); ++w)
    lg.keptRank[w] = lg.keptRank[w - 1] + static_cast<uint32_t>(__builtin_popcountll(lg.keptBits[w - 1]));
  uint32_t const kn = lg.keptRank.back() + static_cast<uint32_t>(__builtin_popcountll(lg.keptBits.back()));
  std::vector<uint8_t>().swap(valid);
  std::vector<uint8_t>().swap(indeg);

  // Pass 2: chains.
  auto const t1 = Clock::now();
  Graph & g = lg.g;
  g.n = kn;
  lg.baseN = kn;
  // The walk isn't in vertex order, so every kept vertex gets room for as many edges as its segment has
  // (pass 1), writes into it, and the few unused places are closed up afterwards.
  lg.origId.assign(kn, 0);
  g.fwdOff.assign(kn + 1, 0);
  for (uint32_t v = 0, cu = 0; v < n; ++v)
    if (lg.Kept(v))
    {
      lg.origId[cu] = v;
      g.fwdOff[cu + 1] = g.fwdOff[cu] + outdeg[v];
      ++cu;
    }
  uint64_t const room = g.fwdOff[kn];
  std::vector<uint8_t> used(kn, 0);
  g.lat.assign(kn, 0);
  g.lon.assign(kn, 0);
  g.fwdTo.assign(room, 0);
  g.fwdW.assign(room, 0);
  lg.interior.assign(room, 0);
  g.fwdK.assign(room, 0);
  g.fwdR.assign(room, 0);
  g.selfR.assign(kn, 0);
  lg.roads.assign(kn, 0);
  auto roadBits = [&ig](routing::Segment const & x)
  { return static_cast<uint8_t>(ig.GetRoadGeometry(x.GetFeatureId()).GetRoutingOptions().GetOptions()); };
  struct Out
  {
    uint32_t to;
    uint8_t interior;
    double w;
    uint32_t k;
    uint8_t r;
    bool operator<(Out const & o) const { return std::tie(to, interior, w) < std::tie(o.to, o.interior, o.w); }
  };
  std::vector<Out> tmp;
  routing::IndexGraph::SegmentEdgeListT chainEdges;
  uint64_t dropped = 0;
  forEachValid(0.12, 0.23, "joining road pieces", [&](routing::Segment const & s)
  {
    uint32_t const v = lg.Slot(s);
    if (!lg.Kept(v))
      return;
    uint32_t const cu = lg.Rank(v);
    if (special[v])
      lg.allLevels.push_back(cu);
    auto const & pt = ig.GetPoint(s, true /* front */);
    g.lat[cu] = static_cast<float>(pt.m_lat);
    g.lon[cu] = static_cast<float>(pt.m_lon);
    g.selfR[cu] = roadBits(s);
    edges.clear();
    ig.GetEdgeList(s, true, true, edges);
    tmp.clear();
    uint8_t roads = 0;
    for (auto const & e : edges)
    {
      routing::Segment x = e.GetTarget();
      if (sp.Dynamic(s, x))
        continue;  // evaluated by the router at the time the route gets there
      double w = e.GetWeight().GetIntegratedWeight();
      uint32_t k = PenaltyUnits(e.GetWeight());
      uint8_t r = roadBits(x);
      uint8_t inner = 0;
      uint32_t xs = lg.Slot(x);
      bool ok = true;
      while (!lg.Kept(xs))
      {
        if (outdeg[xs] != 1)
        {
          ok = false;
          break;
        }
        chainEdges.clear();
        ig.GetEdgeList(x, true, true, chainEdges);
        CHECK_EQUAL(chainEdges.size(), 1u, (x));
        inner = 1;
        w += chainEdges[0].GetWeight().GetIntegratedWeight();
        k += PenaltyUnits(chainEdges[0].GetWeight());
        x = chainEdges[0].GetTarget();
        r |= roadBits(x);
        xs = lg.Slot(x);
      }
      if (!ok || xs == v)
      {
        ++dropped;
        continue;
      }
      roads |= r;
      tmp.push_back({lg.Rank(xs), inner, w, k, r});
    }
    std::sort(tmp.begin(), tmp.end());
    uint64_t at = g.fwdOff[cu];
    for (size_t i = 0; i < tmp.size(); ++i)
    {
      if (i > 0 && tmp[i].to == tmp[i - 1].to && tmp[i].interior == tmp[i - 1].interior)
        continue;  // cheapest parallel edge
      CHECK_LESS(at, g.fwdOff[cu + 1], (s));
      g.fwdTo[at] = tmp[i].to;
      g.fwdW[at] = tmp[i].w;
      g.fwdK[at] = static_cast<uint8_t>(std::min<uint32_t>(tmp[i].k, 255));
      g.fwdR[at] = tmp[i].r;
      lg.interior[at] = tmp[i].interior;
      ++at;
    }
    used[cu] = static_cast<uint8_t>(at - g.fwdOff[cu]);
    lg.roads[cu] = roads;
  });
  // Close up the unused places (in vertex order, moving edges only towards the front).
  {
    uint64_t at = 0;
    for (uint32_t cu = 0; cu < kn; ++cu)
    {
      uint64_t const from = g.fwdOff[cu];
      g.fwdOff[cu] = at;
      for (uint64_t i = 0; i < used[cu]; ++i, ++at)
      {
        g.fwdTo[at] = g.fwdTo[from + i];
        g.fwdW[at] = g.fwdW[from + i];
        g.fwdK[at] = g.fwdK[from + i];
        g.fwdR[at] = g.fwdR[from + i];
        lg.interior[at] = lg.interior[from + i];
      }
    }
    g.fwdOff[kn] = at;
    g.fwdTo.resize(at);
    g.fwdW.resize(at);
    g.fwdK.resize(at);
    g.fwdR.resize(at);
    lg.interior.resize(at);
  }
  std::sort(lg.allLevels.begin(), lg.allLevels.end());
  std::vector<uint8_t>().swap(outdeg);
  std::vector<uint8_t>().swap(special);
  std::ostringstream os;
  os << "pass 2 (chains): " << kn << " kept, " << g.NumEdges() << " compressed edges, " << dropped
     << " dead-end chains, " << lg.dynamicEdges << " time-dependent steps (" << lg.allLevels.size()
     << " special vertices) in " << Since(t1) << " s, " << Mem();
  rep.Log(os.str());
}

// Turn-arounds on the compressed graph, after the via-way copies. A vertex here is a base vertex or a
// via-way copy of one ("under" vertex). The turn-arounds form a tree per |from| (base vertex of u): a node
// per run of pieces c_1..c_i, holding the base vertices that complete a turn-around after it. Every node
// that is entered gets a copy per under vertex of its last piece: an edge from a vertex of |from| onto an
// under vertex y of c_1 leads to the copy (from, y) instead, and an edge of a copy onto an under vertex of a
// child node's piece to the child's copy; a copy has its under vertex's edges otherwise, and those onto
// vertices that complete a turn-around cost IndexGraph::PlusTurnAroundExtraS() more.
void LeanExpandTurnArounds(LeanGraph & lg, Reporter const & rep)
{
  if (lg.turnPaths.empty())
    return;
  Graph & g = lg.g;
  struct Node
  {
    std::map<uint32_t, uint32_t> children;  // base vertex of the next piece -> node
    std::vector<uint32_t> charged;          // base vertices completing a turn-around after this node
  };
  std::vector<Node> nodes;
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> roots;  // (from, base of c_1) -> node
  for (size_t i = 0; i < lg.turnPaths.size();)
  {
    uint32_t const from = lg.Rank(lg.turnPaths[i]);
    uint32_t const k = lg.turnPaths[i + 1];
    uint32_t node = 0;
    for (uint32_t j = 0; j < k; ++j)
    {
      uint32_t const c = lg.Rank(lg.turnPaths[i + 2 + j]);
      uint32_t * slot;
      if (j == 0)
        slot = &roots.emplace(std::make_pair(from, c), uint32_t(-1)).first->second;
      else
        slot = &nodes[node].children.emplace(c, uint32_t(-1)).first->second;
      if (*slot == uint32_t(-1))
      {
        *slot = static_cast<uint32_t>(nodes.size());
        nodes.emplace_back();
      }
      node = *slot;
    }
    nodes[node].charged.push_back(lg.Rank(lg.turnPaths[i + 2 + k]));
    i += 3 + k;
  }
  std::vector<uint32_t>().swap(lg.turnPaths);
  for (auto & nd : nodes)
  {
    std::sort(nd.charged.begin(), nd.charged.end());
    nd.charged.erase(std::unique(nd.charged.begin(), nd.charged.end()), nd.charged.end());
  }

  uint32_t const firstCopy = g.n;
  auto baseOf = [&](uint32_t v) { return v < lg.baseN ? v : lg.copyOrig[v - lg.baseN]; };
  struct Copy
  {
    uint32_t node, under, key;
  };
  std::vector<Copy> copies;
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> copyOf;  // (key, under) -> copy
  auto copyFor = [&](uint32_t key, uint32_t node, uint32_t under)
  {
    auto const ins = copyOf.emplace(std::make_pair(key, under), firstCopy + static_cast<uint32_t>(copies.size()));
    if (ins.second)
      copies.push_back({node, under, key});
    return ins.first->second;
  };
  auto usable = [&](uint64_t e) { return !lg.interior[e] && g.fwdW[e] < kInf; };
  // Entering the trees: edges of the vertices of each |from|.
  for (uint32_t x = 0; x < firstCopy; ++x)
  {
    uint32_t const from = baseOf(x);
    auto const lo = roots.lower_bound({from, 0});
    if (lo == roots.end() || lo->first.first != from)
      continue;
    for (uint64_t e = g.fwdOff[x]; e < g.fwdOff[x + 1]; ++e)
    {
      if (!usable(e))
        continue;
      uint32_t const y = g.fwdTo[e];
      auto const it = roots.find({from, baseOf(y)});
      if (it != roots.end())
        g.fwdTo[e] = copyFor(from, it->second, y);
    }
  }
  // The copies, in id order (a copy made here gets a higher id and its turn later).
  uint64_t chargedEdges = 0;
  for (size_t i = 0; i < copies.size(); ++i)
  {
    Copy const cp = copies[i];
    uint32_t const id = firstCopy + static_cast<uint32_t>(i);
    Node const & nd = nodes[cp.node];
    uint32_t const y = cp.under;
    for (uint64_t e = g.fwdOff[y]; e < g.fwdOff[y + 1]; ++e)
    {
      uint32_t to = g.fwdTo[e];
      double w = g.fwdW[e];
      if (usable(e))
      {
        // A target that is a copy of another tree stands for its under vertex here.
        uint32_t const toUnder = to >= firstCopy ? copies[to - firstCopy].under : to;
        uint32_t const toBase = baseOf(toUnder);
        if (std::binary_search(nd.charged.begin(), nd.charged.end(), toBase))
        {
          w += routing::IndexGraph::PlusTurnAroundExtraS();
          ++chargedEdges;
        }
        auto const child = nd.children.find(toBase);
        if (child != nd.children.end())
          to = copyFor(id, child->second, toUnder);
      }
      g.fwdTo.push_back(to);
      g.fwdW.push_back(w);
      if (!g.fwdK.empty())
        g.fwdK.push_back(g.fwdK[e]);
      if (!g.fwdR.empty())
        g.fwdR.push_back(g.fwdR[e]);
      lg.interior.push_back(lg.interior[e]);
    }
    g.fwdOff.push_back(g.fwdTo.size());
    if (!g.lat.empty())
    {
      g.lat.push_back(g.lat[y]);
      g.lon.push_back(g.lon[y]);
    }
    if (!g.selfR.empty())
      g.selfR.push_back(g.selfR[y]);
    FileContents::TurnAround ta;
    ta.copy = id;
    ta.key = cp.key;
    ta.under = y;
    ta.charged = nd.charged;
    lg.turnArounds.push_back(std::move(ta));
    lg.copyOrig.push_back(baseOf(y));
    lg.copyState.push_back(kTurnAroundCopyState);
  }
  g.n += static_cast<uint32_t>(copies.size());
  std::ostringstream os;
  os << "turn-arounds by pieces 1/2/3/4+: " << lg.turnsByPieces[1] << "/" << lg.turnsByPieces[2] << "/"
     << lg.turnsByPieces[3] << "/" << lg.turnsByPieces[4] << "; " << roots.size() << " starts, " << nodes.size() << " piece runs, " << copies.size()
     << " copies, " << chargedEdges << " edges cost " << routing::IndexGraph::PlusTurnAroundExtraS() << " s more";
  rep.Log(os.str());
}

// Via-way restrictions on the compressed graph. All segments of features in a pattern are kept, so a
// chain interior is never a pattern feature: passing one resets the automaton.
void LeanExpandViaWay(LeanGraph & lg, std::vector<std::vector<uint32_t>> const & patterns, Reporter const & rep)
{
  if (patterns.empty())
    return;
  auto const t0 = Clock::now();
  ViaWayAutomaton ac(patterns);
  Graph & g = lg.g;
  std::vector<uint32_t> feat(g.n);
  for (uint32_t v = 0; v < g.n; ++v)
    feat[v] = lg.FeatureOfSlot(lg.origId[v]);

  std::vector<std::pair<uint32_t, VertexId>> copies;
  std::map<std::pair<uint32_t, VertexId>, VertexId> copyId;
  for (uint32_t s = 0; s < ac.Size(); ++s)
  {
    if (ac.Depth(s) < 2 || ac.Terminal(s))
      continue;
    uint32_t const f = ac.Label(s);
    for (uint32_t r = 2 * lg.featBase[f]; r < 2 * lg.featBase[f + 1]; ++r)
      if (lg.Kept(r))
      {
        VertexId const v = lg.Rank(r);
        copyId[{s, v}] = g.n + static_cast<VertexId>(copies.size());
        copies.emplace_back(s, v);
      }
  }
  // In place, to avoid a second copy of the graph: copies first (they need the original targets), then
  // base vertices rewrite their own edges (forbidden ones get an infinite weight instead of being
  // removed, so the edge arrays keep their layout), then the copies are appended.
  uint64_t dropped = 0, redirected = 0;
  auto step = [&](VertexId v, uint64_t e, uint32_t state, VertexId & target)
  {
    VertexId const t = g.fwdTo[e];
    uint32_t next = state;
    if (lg.interior[e])
      next = ac.Start(feat[t]);
    else if (feat[t] != feat[v])
      next = ac.Step(state, feat[t]);
    if (ac.Terminal(next))
      return false;
    target = (ac.Depth(next) >= 2) ? copyId.at({next, t}) : t;
    return true;
  };
  std::vector<uint64_t> copyOff{0};
  std::vector<VertexId> copyTo;
  std::vector<double> copyW;
  std::vector<uint8_t> copyK;
  std::vector<uint8_t> copyR;
  std::vector<uint8_t> copyInterior;
  for (auto const & [state, v] : copies)
  {
    for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
    {
      VertexId target;
      if (!step(v, e, state, target))
      {
        ++dropped;
        continue;
      }
      redirected += target >= g.n;
      copyTo.push_back(target);
      copyW.push_back(g.fwdW[e]);
      copyK.push_back(static_cast<uint8_t>(g.FwdK(e)));
      copyR.push_back(g.FwdR(e));
      copyInterior.push_back(lg.interior[e]);
    }
    copyOff.push_back(copyTo.size());
  }
  for (VertexId v = 0; v < g.n; ++v)
  {
    uint32_t const state = ac.Start(feat[v]);
    for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
    {
      VertexId target;
      if (!step(v, e, state, target))
      {
        g.fwdW[e] = kInf;
        ++dropped;
        continue;
      }
      redirected += target >= g.n;
      g.fwdTo[e] = target;
    }
  }
  uint64_t const base = g.fwdTo.size();
  for (size_t i = 0; i < copies.size(); ++i)
    g.fwdOff.push_back(base + copyOff[i + 1]);
  g.fwdTo.insert(g.fwdTo.end(), copyTo.begin(), copyTo.end());
  g.fwdW.insert(g.fwdW.end(), copyW.begin(), copyW.end());
  if (!g.fwdK.empty())
    g.fwdK.insert(g.fwdK.end(), copyK.begin(), copyK.end());
  if (!g.fwdR.empty())
    g.fwdR.insert(g.fwdR.end(), copyR.begin(), copyR.end());
  lg.interior.insert(lg.interior.end(), copyInterior.begin(), copyInterior.end());
  for (auto const & [st, v] : copies)
  {
    if (!g.lat.empty())
    {
      g.lat.push_back(g.lat[v]);
      g.lon.push_back(g.lon[v]);
    }
    if (!g.selfR.empty())
      g.selfR.push_back(g.selfR[v]);
    lg.copyOrig.push_back(v);
    lg.copyState.push_back(st);
  }
  g.n += static_cast<uint32_t>(copies.size());
  std::ostringstream os;
  os << "via-way: " << patterns.size() << " patterns, " << ac.Size() << " states, " << copies.size() << " copies, "
     << redirected << " edges redirected, " << dropped << " forbidden (" << Since(t0) << " s)";
  rep.Log(os.str());
}

// CoMaps' car graph for one map, as IndexRouter loads it.
struct MapGraph
{
  FrozenDataSource dataSource;
  std::shared_ptr<routing::NumMwmIds> numMwmIds = std::make_shared<routing::NumMwmIds>();
  std::unique_ptr<routing::MwmDataSource> mwmDataSource;
  std::shared_ptr<routing::VehicleModelFactoryInterface> factory;
  std::shared_ptr<routing::EdgeEstimator> estimator;
  std::unique_ptr<routing::IndexGraphLoader> loader;
  routing::NumMwmId mwmId = 0;
  routing::IndexGraph * graph = nullptr;
  int64_t mwmVersion = 0;
  std::vector<std::vector<uint32_t>> viaWay;  // forbidden feature chains [from, via..., to], 3+ features

  bool Init(platform::LocalCountryFile const & localFile,
            routing::VehicleModelFactory::CountryParentNameGetterFn const & parentGetter, std::string & error,
            uint32_t avoidMask = 0)
  {
    auto const res = dataSource.RegisterMap(localFile);
    if (res.second != MwmSet::RegResult::Success)
    {
      error = "can't register the map";
      return false;
    }
    mwmVersion = res.first.GetInfo()->GetVersion();
    auto const & countryFile = localFile.GetCountryFile();
    numMwmIds->RegisterFile(countryFile);
    mwmId = numMwmIds->GetId(countryFile);
    mwmDataSource = std::make_unique<routing::MwmDataSource>(dataSource, numMwmIds);
    factory = std::make_shared<routing::CarModelFactory>(parentGetter);
    double const maxSpeed = factory->GetVehicleModelForCountry(countryFile.GetName())->GetMaxWeightSpeed();
    estimator = routing::EdgeEstimator::Create(routing::VehicleType::Car, maxSpeed,
                                               factory->GetVehicleModel()->GetOffroadSpeed(), nullptr /* traffic */,
                                               &dataSource, numMwmIds);
    auto const fixedTime = std::time(nullptr);
    loader = routing::IndexGraphLoader::Create(routing::VehicleType::Car, false /* loadAltitudes */, factory,
                                               estimator, *mwmDataSource,
                                               routing::RoutingOptions(
                                                   static_cast<routing::RoutingOptions::OptionType>(avoidMask),
                                                   routing::VehicleType::Car),
                                               [fixedTime]() { return fixedTime; });
    graph = &loader->GetIndexGraph(mwmId);

    routing::RestrictionLoader rl(mwmDataSource->GetMwmValue(mwmId), *graph);
    for (auto & r : rl.StealRestrictions())
      if (r.size() >= 3)
        viaWay.push_back(std::move(r));
    return true;
  }
};

}  // namespace

BuildResult BuildOverlayFile(platform::LocalCountryFile const & localFileIn,
                             routing::VehicleModelFactory::CountryParentNameGetterFn const & countryParentNameGetter,
                             std::string const & outPathIn, BuildOptions const & options)
{
  auto const tAll = Clock::now();
  Reporter const rep{options};
  BuildResult result;
  platform::LocalCountryFile localFile = localFileIn;
  localFile.SyncWithDisk();
  result.path = outPathIn.empty() ? OverlayPathForMap(localFile.GetPath(MapFileType::Map)) : outPathIn;
  unsigned const threads = std::max(1u, options.threads);
  rep.Log("building " + localFile.GetCountryName() + " -> " + result.path + " on " + std::to_string(threads) +
          " threads");
  try
  {
    rep.Progress(0, "loading the map");
    auto mg = std::make_unique<MapGraph>();
    if (!mg->Init(localFile, countryParentNameGetter, result.error))
      return result;
    result.mwmVersion = mg->mwmVersion;
    auto const viaWay = mg->viaWay;

    // Border transitions (both directions): kept, and boundary vertices at every level.
    std::vector<routing::Segment> enters, exits;
    std::vector<uint64_t> enterOsm, exitOsm;
    {
      routing::CrossMwmIndexGraph<base::GeoObjectId> cross(*mg->mwmDataSource, routing::VehicleType::Car);
      auto const & c = cross.GetCrossMwmConnectorWithTransitions(mg->mwmId);
      c.ForEachEnter([&](uint32_t, routing::Segment const & s)
      {
        enters.push_back(s);
        enterOsm.push_back(c.GetCrossMwmId(s).GetEncodedId());
      });
      c.ForEachExit([&](uint32_t, routing::Segment const & s)
      {
        exits.push_back(s);
        exitOsm.push_back(c.GetCrossMwmId(s).GetEncodedId());
      });
    }
    std::vector<routing::Segment> forced(enters);
    forced.insert(forced.end(), exits.begin(), exits.end());
    Special sp;
    sp.access = &mg->graph->GetRoadAccess();

    auto t0 = Clock::now();
    LeanGraph lg;
    LeanExtract(*mg->graph, mg->mwmId, forced, viaWay, sp, lg, rep);
    rep.CheckCancelled();
    mg.reset();  // the router keeps its own IndexGraph; this one was only needed for extraction
    ReleaseFreedMemory();
    LeanExpandViaWay(lg, viaWay, rep);
    LeanExpandTurnArounds(lg, rep);
    std::vector<uint8_t>().swap(lg.interior);
    ReleaseFreedMemory();
    Graph & g = lg.g;
    double const extractSec = Since(t0);
    {
      std::ostringstream os;
      os << "compressed graph: " << g.n << " vertices, " << g.NumEdges() << " edges in " << extractSec << " s, "
         << Mem();
      rep.Log(os.str());
    }
    rep.CheckCancelled();

    // Special vertices (overlay.hpp), with their via-way copies.
    std::vector<uint32_t> special = lg.allLevels;
    {
      std::vector<uint8_t> isSpecial(g.n, 0);
      for (uint32_t v : special)
        isSpecial[v] = 1;
      for (uint32_t v = lg.baseN; v < g.n; ++v)
        if (isSpecial[lg.copyOrig[v - lg.baseN]])
          special.push_back(v);
      std::sort(special.begin(), special.end());
      special.erase(std::unique(special.begin(), special.end()), special.end());
    }
    std::vector<uint32_t>().swap(lg.allLevels);
    std::vector<uint8_t> extra(g.n, 0);
    for (auto const & s : enters)
      if (lg.Has(s))
        if (uint32_t const r = lg.Slot(s); lg.Kept(r))
          extra[lg.Rank(r)] |= 1;
    for (auto const & s : exits)
      if (lg.Has(s))
        if (uint32_t const r = lg.Slot(s); lg.Kept(r))
          extra[lg.Rank(r)] |= 2;

    ParkedWeights parked(result.path + ".weights.tmp");
    if (parked.Park(g))
      ReleaseFreedMemory();
    rep.Progress(0.35, "dividing the map into cells");
    PartitionParams pp;
    pp.cellSizes = DefaultCellSizes();
    pp.minBalance = kDefaultMinBalance;
    pp.directions = kDefaultDirections;
    pp.threads = threads;
    pp.cancelled = options.cancelled;
    pp.progress = [&rep](double f) { rep.Progress(0.35 + 0.35 * f, "dividing the map into cells"); };
    t0 = Clock::now();
    Partition part;
    if (!options.partitionTreePath.empty())
    {
      // A map update: same roads mostly, new feature ids. Reuse the cells of the previous version.
      PartitionTree tree;
      std::string why;
      if (LoadPartitionTree(options.partitionTreePath, tree))
      {
        if (ReplayPartition(g, tree, pp, part, &why))
          result.reusedCells = true;
        else
          rep.Log("not reusing the previous cells: " + why);
      }
    }
    if (!result.reusedCells)
      part = Bisect(g, pp);
    rep.CheckCancelled();
    // Kept right away: if this build is stopped later (the system stopping the app, a full storage), the
    // next attempt replays these cells instead of dividing the map again (the slowest step).
    if (!options.partitionTreePath.empty() && !result.reusedCells && !SavePartitionTree(part.tree, options.partitionTreePath))
      rep.Log("couldn't save the partition tree to " + options.partitionTreePath);
    double const partSec = Since(t0);
    {
      std::ostringstream os;
      os << (result.reusedCells ? "cells reused from the previous map version," : "cells per level");
      for (auto nc : part.numCells)
        os << " " << nc;
      os << " in " << partSec << " s, " << Mem();
      rep.Log(os.str());
    }
    if (!options.checkFile)
    {
      std::vector<float>().swap(g.lat);
      std::vector<float>().swap(g.lon);
    }
    ReleaseFreedMemory();

    rep.Progress(0.70, "computing shortcuts");
    Overlay ov;
    t0 = Clock::now();
    BuildBoundaries(g, part, ov, &extra, &special);
    std::vector<uint8_t>().swap(extra);
    if (!parked.Restore(g))
    {
      result.error = "can't read the parked edge weights back";
      return result;
    }
    {
      std::ostringstream os;
      os << "boundaries per level (entries/exits/clique entries):";
      for (auto const & L : ov.levels)
        os << " " << L.entries.size() << "/" << L.exits.size() << "/" << L.matOff.back();
      rep.Log(os.str());
    }
    Customize(g, part, ov, threads, options.cancelled,
              [&rep](double f) { rep.Progress(0.70 + 0.29 * f, "computing shortcuts"); });
    rep.CheckCancelled();
    double const custSec = Since(t0);
    {
      std::ostringstream os;
      os << "shortcuts in " << custSec << " s, " << Mem();
      rep.Log(os.str());
    }
    // Cells whose cliques avoid options make unusable, per level (tolls, motorways, ferries, dirt roads).
    for (size_t li = 0; li < ov.levels.size() && !ov.levels[li].cellRoads.empty(); ++li)
    {
      std::ostringstream os;
      os << "level " << li + 1 << ": " << part.numCells[li] << " cells, with paths on tolls/motorways/ferries/dirt:";
      for (uint8_t bit : {2, 4, 8, 16})
        os << " " << std::count_if(ov.levels[li].cellRoads.begin(), ov.levels[li].cellRoads.end(),
                                   [bit](uint8_t r) { return (r & bit) != 0; });
      auto const & L = ov.levels[li];
      os << "; of " << L.entryPathRoads.size() << " entries, rows on them:";
      for (uint8_t bit : {2, 4, 8, 16})
        os << " " << std::count_if(L.entryPathRoads.begin(), L.entryPathRoads.end(),
                                   [bit](uint8_t r) { return (r & bit) != 0; });
      rep.Log(os.str());
    }

    if (!options.checkFile)
    {
      // Only the cliques and the vertex tables go into the file.
      std::vector<uint64_t>().swap(g.fwdOff);
      std::vector<VertexId>().swap(g.fwdTo);
      std::vector<double>().swap(g.fwdW);
      std::vector<uint8_t>().swap(g.fwdK);
      std::vector<uint8_t>().swap(g.fwdR);
      ReleaseFreedMemory();
    }
    rep.Progress(0.99, "writing");
    FileContents fc;
    fc.mwmVersion = result.mwmVersion;
    fc.paramsHash = DefaultParamsHash();
    fc.featBase = lg.featBase;
    fc.keptBits = lg.keptBits;
    fc.keptRank = lg.keptRank;
    fc.numSlots = static_cast<uint32_t>(lg.rawSlots);
    fc.baseN = lg.baseN;
    fc.n = g.n;
    fc.copyOrig = lg.copyOrig;
    fc.copyState = lg.copyState;
    fc.turnArounds = lg.turnArounds;
    fc.patterns = viaWay;
    auto addTransitions = [&](std::vector<routing::Segment> const & segs, std::vector<uint64_t> const & osm,
                              bool isEnter)
    {
      for (size_t i = 0; i < segs.size(); ++i)
      {
        if (!lg.Has(segs[i]))
          continue;  // not a car road in this map's graph (no geometry): CoMaps can't route on it either
        uint32_t const r = lg.Slot(segs[i]);
        if (!lg.Kept(r))
          continue;
        FileTransition t;
        t.osmId = osm[i];
        t.local = lg.Rank(r);
        t.segIdx = segs[i].GetSegmentIdx();
        t.isEnter = isEnter ? 1 : 0;
        fc.transitions.push_back(t);
      }
    };
    fc.special = special;
    fc.timeDependent = lg.dynamicEdges > 0;
    fc.roads.resize(g.n);
    for (uint32_t v = 0; v < g.n; ++v)
      fc.roads[v] = lg.roads[v < lg.baseN ? v : lg.copyOrig[v - lg.baseN]];
    fc.selfRoads = g.selfR;
    addTransitions(enters, enterOsm, true);
    addTransitions(exits, exitOsm, false);
    if (!WriteOverlayFile(result.path, fc, part, ov, &result.error))
      return result;
    auto const mo = MappedOverlay::Open(result.path, result.mwmVersion, fc.paramsHash, &result.error);
    if (!mo)
    {
      Platform::RemoveFileIfExists(result.path);
      return result;
    }
    result.fileBytes = mo->FileBytes();

    if (options.checkFile)
    {
      // Structural round trip: everything a query reads must come back identical (cliques as float32).
      uint64_t bad = 0;
      for (uint32_t v = 0; v < g.n; ++v)
      {
        if (v < lg.baseN && mo->SlotOf(v) != lg.origId[v])
          ++bad;
        for (size_t li = 0; li < part.Levels(); ++li)
          if (mo->Cell(li, v) != part.cell[li][v] || mo->IsEntry(li, v) != ov.levels[li].IsEntry(v) ||
              mo->IsExit(li, v) != ov.levels[li].IsExit(v))
            ++bad;
      }
      double maxAbs = 0;
      for (size_t li = 0; li < part.Levels(); ++li)
      {
        auto const & M = ov.levels[li];
        for (uint32_t c = 0; c < part.numCells[li]; ++c)
        {
          uint64_t const nEn = M.entryOff[c + 1] - M.entryOff[c], nEx = M.exitOff[c + 1] - M.exitOff[c];
          for (uint64_t i = 0; i < nEn; ++i)
            for (uint64_t j = 0; j < nEx; ++j)
            {
              double const want = M.mat[M.matOff[c] + i * nEx + j];
              float const got = mo->Clique(li, c, static_cast<uint32_t>(i), static_cast<uint32_t>(j));
              bool const inf0 = want >= kInf, inf1 = got >= 1e29f;
              if (inf0 != inf1)
                ++bad;
              else if (!inf0)
              {
                // Stored as steps of the cell's scale: off by at most half a step (plus float rounding).
                double const err = std::abs(want - got);
                maxAbs = std::max(maxAbs, err);
                if (err > 0.5 * mo->CellScale(li, c) + 1e-4 * std::max(1.0, want))
                  ++bad;
              }
            }
        }
      }
      std::ostringstream os;
      os << "file check: " << bad << " mismatches in ids/cells/roles/cliques; largest rounding " << maxAbs << " s";
      rep.Log(os.str());
      if (bad != 0)
      {
        Platform::RemoveFileIfExists(result.path);
        result.error = "file check failed";
        return result;
      }
    }

    uint64_t rss;
    ReadMem(rss, result.peakRssKb);
    result.seconds = Since(tAll);
    result.status = BuildResult::Status::Ok;
    std::ostringstream os;
    os << "done: " << (result.fileBytes / 1048576.0) << " MB in " << result.seconds << " s (extraction " << extractSec
       << ", partition " << partSec << ", shortcuts " << custSec << "), " << Mem();
    rep.Log(os.str());
    rep.Progress(1.0, "done");
    return result;
  }
  catch (Cancelled const &)
  {
    result.status = BuildResult::Status::Cancelled;
    result.error = "cancelled";
    result.seconds = Since(tAll);
    rep.Log("cancelled after " + std::to_string(result.seconds) + " s");
    return result;
  }
  catch (std::exception const & e)
  {
    result.status = BuildResult::Status::Failed;
    result.error = e.what();
    return result;
  }
}

size_t DeleteOrphanedOverlayFiles(std::string const & dir)
{
  Platform::FilesList files;
  Platform::GetFilesByExt(dir, kFileExtension, files);
  size_t deleted = 0;
  for (auto const & name : files)
  {
    std::string const path = base::JoinPath(dir, name);
    std::string stem = path.substr(0, path.size() - std::string(kFileExtension).size());
    // <map>.avoid<mask>.plusovl belongs to <map>.mwm too.
    auto const dot = stem.rfind('.');
    // Avoid-option files (<map>.avoid<mask>.plusovl) are no longer used: avoid options need no file.
    bool const isAvoid = dot != std::string::npos && stem.compare(dot, 6, ".avoid") == 0 && dot + 6 < stem.size() &&
                         stem.find_first_not_of("0123456789", dot + 6) == std::string::npos;
    bool const orphan = !Platform::IsFileExistsByFullPath(stem + DATA_FILE_EXTENSION);
    if ((orphan || isAvoid) && Platform::RemoveFileIfExists(path))
      ++deleted;
  }
  // Interrupted builds.
  files.clear();
  Platform::GetFilesByExt(dir, std::string(kFileExtension) + ".tmp", files);
  for (auto const & name : files)
    if (Platform::RemoveFileIfExists(base::JoinPath(dir, name)))
      ++deleted;
  return deleted;
}
}  // namespace plus_overlay
