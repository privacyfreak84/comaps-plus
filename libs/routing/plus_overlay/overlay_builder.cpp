#include "routing/plus_overlay/overlay_builder.hpp"

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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>
#include <tuple>

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

// Progress over the build, by the share of time each stage took on a desktop (UP: extraction ~30%,
// partition ~55%, customize ~15%).
struct Reporter
{
  BuildOptions const & o;
  void Log(std::string const & s) const
  {
    if (o.log)
      o.log(s);
    else
      LOG(LINFO, ("Comaps++ overlay build:", s));
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
  uint64_t rawSlots = 0, rawValid = 0, rawEdges = 0;

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

// Two walks over CoMaps' IndexGraph: degrees, then chains between kept segments.
void LeanExtract(routing::IndexGraph & ig, routing::NumMwmId mwmId, std::vector<routing::Segment> const & forced,
                 std::vector<std::vector<uint32_t>> const & viaWay, LeanGraph & lg, Reporter const & rep)
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
  for (uint32_t f : roads)
  {
    auto const & road = ig.GetRoadGeometry(f);
    if (!road.IsValid() || road.GetPointsCount() < 2)
      continue;
    segs[f] = road.GetPointsCount() - 1;
    oneway[f] = road.IsOneWay() ? 1 : 0;
  }
  lg.featBase.assign(maxF + 2, 0);
  for (uint32_t f = 0; f <= maxF; ++f)
    lg.featBase[f + 1] = lg.featBase[f] + segs[f];
  uint32_t const n = 2 * lg.featBase[maxF + 1];
  lg.rawSlots = n;

  // Calls fn for every valid directed segment; polls cancellation and reports progress over |span|.
  auto forEachValid = [&](double from, double span, std::string const & stage, auto && fn)
  {
    size_t done = 0;
    for (uint32_t f : roads)
    {
      if ((++done & 4095) == 0)
      {
        rep.CheckCancelled();
        rep.Progress(from + span * done / roads.size(), stage);
      }
      for (uint32_t i = 0; i < segs[f]; ++i)
        for (bool fwd : {true, false})
          if (fwd || !oneway[f])
            fn(routing::Segment(mwmId, f, i, fwd));
    }
  };

  // Pass 1: degrees.
  std::vector<uint8_t> indeg(n, 0), outdeg(n, 0), valid(n, 0);
  routing::IndexGraph::SegmentEdgeListT edges;
  forEachValid(0.0, 0.12, "reading roads", [&](routing::Segment const & s)
  {
    uint32_t const v = lg.Slot(s);
    valid[v] = 1;
    ++lg.rawValid;
    edges.clear();
    ig.GetEdgeList(s, true /* isOutgoing */, true /* useRoutingOptions */, edges);
    outdeg[v] = static_cast<uint8_t>(std::min<size_t>(2, edges.size()));
    lg.rawEdges += edges.size();
    for (auto const & e : edges)
    {
      uint32_t const t = lg.Slot(e.GetTarget());
      if (indeg[t] < 2)
        ++indeg[t];
    }
  });
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
    if (!isolated && !interiorV)
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
  lg.origId.reserve(kn);
  g.fwdOff.assign(kn + 1, 0);
  g.lat.reserve(kn);
  g.lon.reserve(kn);
  // Compressed edges per kept vertex are ~2.2 on road graphs; a close reserve avoids the vector's
  // doubling growth (a transient 1.5-2x of the largest arrays).
  uint64_t const guess = static_cast<uint64_t>(kn * 2.3) + 1024;
  g.fwdTo.reserve(guess);
  g.fwdW.reserve(guess);
  lg.interior.reserve(guess);
  struct Out
  {
    uint32_t to;
    uint8_t interior;
    double w;
    bool operator<(Out const & o) const { return std::tie(to, interior, w) < std::tie(o.to, o.interior, o.w); }
  };
  std::vector<Out> tmp;
  routing::IndexGraph::SegmentEdgeListT chainEdges;
  uint64_t dropped = 0;
  forEachValid(0.12, 0.18, "joining road pieces", [&](routing::Segment const & s)
  {
    uint32_t const v = lg.Slot(s);
    if (!lg.Kept(v))
      return;
    uint32_t const cu = static_cast<uint32_t>(lg.origId.size());
    lg.origId.push_back(v);
    auto const & pt = ig.GetPoint(s, true /* front */);
    g.lat.push_back(static_cast<float>(pt.m_lat));
    g.lon.push_back(static_cast<float>(pt.m_lon));
    edges.clear();
    ig.GetEdgeList(s, true, true, edges);
    tmp.clear();
    for (auto const & e : edges)
    {
      routing::Segment x = e.GetTarget();
      double w = e.GetWeight().GetIntegratedWeight();
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
        x = chainEdges[0].GetTarget();
        xs = lg.Slot(x);
      }
      if (!ok || xs == v)
      {
        ++dropped;
        continue;
      }
      tmp.push_back({lg.Rank(xs), inner, w});
    }
    std::sort(tmp.begin(), tmp.end());
    for (size_t i = 0; i < tmp.size(); ++i)
    {
      if (i > 0 && tmp[i].to == tmp[i - 1].to && tmp[i].interior == tmp[i - 1].interior)
        continue;  // cheapest parallel edge
      g.fwdTo.push_back(tmp[i].to);
      g.fwdW.push_back(tmp[i].w);
      lg.interior.push_back(tmp[i].interior);
    }
    g.fwdOff[cu + 1] = g.fwdTo.size();
  });
  CHECK_EQUAL(lg.origId.size(), kn, ());
  std::vector<uint8_t>().swap(outdeg);
  std::ostringstream os;
  os << "pass 2 (chains): " << kn << " kept, " << g.NumEdges() << " compressed edges, " << dropped
     << " dead-end chains in " << Since(t1) << " s, " << Mem();
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
  lg.interior.insert(lg.interior.end(), copyInterior.begin(), copyInterior.end());
  for (auto const & [st, v] : copies)
  {
    if (!g.lat.empty())
    {
      g.lat.push_back(g.lat[v]);
      g.lon.push_back(g.lon[v]);
    }
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

// The compressed graph of |base| (same vertices: its kept segments and via-way copies, same ids) with the
// edges of |ig|, which has avoid options applied. Edges onto avoided roads are missing in |ig|, so chains
// only get shorter: a segment that is a chain interior without options has at most one successor here.
bool ExtractForBase(routing::IndexGraph & ig, routing::NumMwmId mwmId, MappedOverlay const & base,
                    std::vector<std::vector<uint32_t>> const & viaWay, LeanGraph & lg, Reporter const & rep,
                    std::string & error)
{
  auto const t0 = Clock::now();
  uint32_t const nf = base.NumFeatures();
  lg.featBase.resize(nf + 1);
  for (uint32_t f = 0; f <= nf; ++f)
    lg.featBase[f] = base.FeatBase(f);
  lg.keptBits.assign(base.KeptBitsData(), base.KeptBitsData() + base.NumKeptWords());
  lg.keptRank.assign(base.KeptRankData(), base.KeptRankData() + base.NumKeptWords());
  lg.rawSlots = 2ULL * lg.featBase[nf];
  uint32_t const kn = base.BaseN();
  Graph & g = lg.g;
  g.n = kn;
  lg.baseN = kn;
  lg.origId.reserve(kn);
  g.fwdOff.assign(kn + 1, 0);
  uint64_t const guess = static_cast<uint64_t>(kn * 2.3) + 1024;
  g.fwdTo.reserve(guess);
  g.fwdW.reserve(guess);
  lg.interior.reserve(guess);
  struct Out
  {
    uint32_t to;
    uint8_t interior;
    double w;
    bool operator<(Out const & o) const { return std::tie(to, interior, w) < std::tie(o.to, o.interior, o.w); }
  };
  std::vector<Out> tmp;
  routing::IndexGraph::SegmentEdgeListT edges, chainEdges;
  for (uint32_t f = 0; f < nf; ++f)
  {
    if ((f & 4095) == 0)
    {
      rep.CheckCancelled();
      rep.Progress(0.6 * f / std::max(1u, nf), "reading roads");
    }
    for (uint32_t slot = 2 * lg.featBase[f]; slot < 2 * lg.featBase[f + 1]; ++slot)
    {
      if (!lg.Kept(slot))
        continue;
      uint32_t const cu = static_cast<uint32_t>(lg.origId.size());
      if (cu != lg.Rank(slot))
      {
        error = "vertex numbering differs from the base file";
        return false;
      }
      lg.origId.push_back(slot);
      routing::Segment const s(mwmId, f, slot / 2 - lg.featBase[f], slot % 2 == 0);
      edges.clear();
      ig.GetEdgeList(s, true, true, edges);
      tmp.clear();
      for (auto const & e : edges)
      {
        routing::Segment x = e.GetTarget();
        if (!lg.Has(x))
          continue;
        double w = e.GetWeight().GetIntegratedWeight();
        uint8_t inner = 0;
        uint32_t xs = lg.Slot(x);
        bool ok = true;
        while (!lg.Kept(xs))
        {
          chainEdges.clear();
          ig.GetEdgeList(x, true, true, chainEdges);
          if (chainEdges.size() != 1 || !lg.Has(chainEdges[0].GetTarget()))
          {
            ok = false;  // dead end (avoided road ahead)
            break;
          }
          inner = 1;
          w += chainEdges[0].GetWeight().GetIntegratedWeight();
          x = chainEdges[0].GetTarget();
          xs = lg.Slot(x);
        }
        if (!ok || xs == slot)
          continue;
        tmp.push_back({lg.Rank(xs), inner, w});
      }
      std::sort(tmp.begin(), tmp.end());
      for (size_t i = 0; i < tmp.size(); ++i)
      {
        if (i > 0 && tmp[i].to == tmp[i - 1].to && tmp[i].interior == tmp[i - 1].interior)
          continue;
        g.fwdTo.push_back(tmp[i].to);
        g.fwdW.push_back(tmp[i].w);
        lg.interior.push_back(tmp[i].interior);
      }
      g.fwdOff[cu + 1] = g.fwdTo.size();
    }
  }
  if (lg.origId.size() != kn)
  {
    error = "vertex count differs from the base file";
    return false;
  }
  LeanExpandViaWay(lg, viaWay, rep);
  if (g.n != base.NumVertices())
  {
    error = "via-way copies differ from the base file";
    return false;
  }
  for (uint32_t v = kn; v < g.n; ++v)
  {
    if (lg.copyOrig[v - kn] != base.CopyOrig(v) || lg.copyState[v - kn] != base.CopyState(v))
    {
      error = "via-way copies differ from the base file";
      return false;
    }
  }
  std::ostringstream os;
  os << "graph with avoid options: " << g.n << " vertices, " << g.NumEdges() << " edges in " << Since(t0) << " s, "
     << Mem();
  rep.Log(os.str());
  return true;
}

// Partition and cell boundaries as stored in |base|.
void PartitionFromFile(MappedOverlay const & base, Partition & p, Overlay & ov)
{
  size_t const levels = base.Levels();
  uint32_t const n = base.NumVertices();
  p.cell.assign(levels, std::vector<uint32_t>(n));
  p.numCells.assign(levels, 0);
  ov.levels.assign(levels, {});
  for (size_t li = 0; li < levels; ++li)
  {
    auto const & F = base.GetLevel(li);
    p.numCells[li] = F.numCells;
    for (uint32_t v = 0; v < n; ++v)
      p.cell[li][v] = base.Cell(li, v);
    auto & L = ov.levels[li];
    L.entryOff.assign(F.entryOff, F.entryOff + F.numCells + 1);
    L.exitOff.assign(F.exitOff, F.exitOff + F.numCells + 1);
    L.entries.assign(F.entries, F.entries + F.entryOff[F.numCells]);
    L.exits.assign(F.exits, F.exits + F.exitOff[F.numCells]);
    L.role.assign(n, 0);
    for (uint32_t v = 0; v < n; ++v)
      L.role[v] = static_cast<uint8_t>((base.IsEntry(li, v) ? 1 : 0) | (base.IsExit(li, v) ? 2 : 0));
    L.matOff.assign(F.numCells + 1, 0);
    for (uint32_t c = 0; c < F.numCells; ++c)
      L.matOff[c + 1] = L.matOff[c] + (L.entryOff[c + 1] - L.entryOff[c]) * (L.exitOff[c + 1] - L.exitOff[c]);
    L.mat.assign(L.matOff[F.numCells], kInf);
  }
}
}  // namespace

BuildResult BuildAvoidCliquesFile(platform::LocalCountryFile const & localFileIn,
                                  routing::VehicleModelFactory::CountryParentNameGetterFn const & countryParentNameGetter,
                                  uint32_t avoidMask, std::string const & basePathIn, std::string const & outPathIn,
                                  BuildOptions const & options)
{
  auto const tAll = Clock::now();
  Reporter const rep{options};
  BuildResult result;
  platform::LocalCountryFile localFile = localFileIn;
  localFile.SyncWithDisk();
  std::string const mwmPath = localFile.GetPath(MapFileType::Map);
  std::string const basePath = basePathIn.empty() ? OverlayPathForMap(mwmPath) : basePathIn;
  result.path = outPathIn.empty() ? AvoidOverlayPathForMap(mwmPath, avoidMask) : outPathIn;
  unsigned const threads = std::max(1u, options.threads);
  rep.Log("avoid options " + std::to_string(avoidMask) + " for " + localFile.GetCountryName() + " -> " + result.path);
  try
  {
    rep.Progress(0, "loading the map");
    auto mg = std::make_unique<MapGraph>();
    if (!mg->Init(localFile, countryParentNameGetter, result.error, avoidMask))
      return result;
    result.mwmVersion = mg->mwmVersion;
    auto const base = MappedOverlay::Open(basePath, mg->mwmVersion, DefaultParamsHash(), &result.error);
    if (!base)
    {
      result.error = "base overlay file: " + result.error;
      return result;
    }

    LeanGraph lg;
    if (!ExtractForBase(*mg->graph, mg->mwmId, *base, mg->viaWay, lg, rep, result.error))
      return result;
    std::vector<uint8_t>().swap(lg.interior);
    mg.reset();
    rep.CheckCancelled();

    Partition part;
    Overlay ov;
    PartitionFromFile(*base, part, ov);
    rep.CheckCancelled();
    rep.Progress(0.6, "computing shortcuts");
    auto const t0 = Clock::now();
    Customize(lg.g, part, ov, threads, options.cancelled,
              [&rep](double f) { rep.Progress(0.6 + 0.39 * f, "computing shortcuts"); });
    rep.CheckCancelled();
    {
      std::ostringstream os;
      os << "shortcuts in " << Since(t0) << " s, " << Mem();
      rep.Log(os.str());
    }

    if (!WriteAvoidCliquesFile(result.path, *base, avoidMask, ov, &result.error))
      return result;
    // Read it back through the router's own path.
    auto const check = MappedOverlay::Open(basePath, result.mwmVersion, DefaultParamsHash(), &result.error);
    if (!check || !check->AttachAvoidCliques(result.path, avoidMask, &result.error))
    {
      Platform::RemoveFileIfExists(result.path);
      return result;
    }
    result.fileBytes = check->AvoidFileBytes();
    if (options.checkFile)
    {
      uint64_t bad = 0;
      double maxRel = 0;
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
              float const got = check->Clique(li, c, static_cast<uint32_t>(i), static_cast<uint32_t>(j));
              bool const inf0 = want >= kInf, inf1 = got >= 1e29f;
              if (inf0 != inf1)
                ++bad;
              else if (!inf0)
                maxRel = std::max(maxRel, std::abs(want - got) / std::max(1.0, want));
            }
        }
      }
      std::ostringstream os;
      os << "file check: " << bad << " clique mismatches; max relative float32 error " << maxRel;
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
    os << "done: " << (result.fileBytes / 1048576.0) << " MB in " << result.seconds << " s, " << Mem();
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

    auto t0 = Clock::now();
    LeanGraph lg;
    LeanExtract(*mg->graph, mg->mwmId, forced, viaWay, lg, rep);
    rep.CheckCancelled();
    LeanExpandViaWay(lg, viaWay, rep);
    std::vector<uint8_t>().swap(lg.interior);
    mg.reset();  // the router keeps its own IndexGraph; this one was only needed for extraction
    Graph & g = lg.g;
    double const extractSec = Since(t0);
    {
      std::ostringstream os;
      os << "compressed graph: " << g.n << " vertices, " << g.NumEdges() << " edges in " << extractSec << " s, "
         << Mem();
      rep.Log(os.str());
    }
    rep.CheckCancelled();

    std::vector<uint8_t> extra(g.n, 0);
    for (auto const & s : enters)
      if (lg.Has(s))
        if (uint32_t const r = lg.Slot(s); lg.Kept(r))
          extra[lg.Rank(r)] |= 1;
    for (auto const & s : exits)
      if (lg.Has(s))
        if (uint32_t const r = lg.Slot(s); lg.Kept(r))
          extra[lg.Rank(r)] |= 2;

    rep.Progress(0.30, "dividing the map into cells");
    PartitionParams pp;
    pp.cellSizes = DefaultCellSizes();
    pp.minBalance = kDefaultMinBalance;
    pp.directions = kDefaultDirections;
    pp.threads = threads;
    pp.cancelled = options.cancelled;
    pp.progress = [&rep](double f) { rep.Progress(0.30 + 0.55 * f, "dividing the map into cells"); };
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

    rep.Progress(0.85, "computing shortcuts");
    Overlay ov;
    t0 = Clock::now();
    BuildBoundaries(g, part, ov, &extra);
    std::vector<uint8_t>().swap(extra);
    Customize(g, part, ov, threads, options.cancelled,
              [&rep](double f) { rep.Progress(0.85 + 0.14 * f, "computing shortcuts"); });
    rep.CheckCancelled();
    double const custSec = Since(t0);
    {
      std::ostringstream os;
      os << "shortcuts in " << custSec << " s, " << Mem();
      rep.Log(os.str());
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
    if (!options.partitionTreePath.empty() && !result.reusedCells && !SavePartitionTree(part.tree, options.partitionTreePath))
      rep.Log("couldn't save the partition tree to " + options.partitionTreePath);

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
      double maxRel = 0;
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
                maxRel = std::max(maxRel, std::abs(want - got) / std::max(1.0, want));
            }
        }
      }
      std::ostringstream os;
      os << "file check: " << bad << " mismatches in ids/cells/roles/cliques; max relative float32 error " << maxRel;
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

size_t DeleteOrphanedOverlayFiles(std::string const & dir, int64_t keepAvoidMask)
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
    uint32_t mask = 0;
    bool const isAvoid = dot != std::string::npos && stem.compare(dot, 6, ".avoid") == 0 && dot + 6 < stem.size() &&
                         stem.find_first_not_of("0123456789", dot + 6) == std::string::npos;
    if (isAvoid)
    {
      mask = static_cast<uint32_t>(std::stoul(stem.substr(dot + 6)));
      stem.resize(dot);
    }
    bool const orphan = !Platform::IsFileExistsByFullPath(stem + DATA_FILE_EXTENSION);
    bool const unwanted = isAvoid && keepAvoidMask >= 0 && mask != static_cast<uint32_t>(keepAvoidMask);
    if ((orphan || unwanted) && Platform::RemoveFileIfExists(path))
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
