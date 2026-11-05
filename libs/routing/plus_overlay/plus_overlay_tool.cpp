// Comaps++ cell-overlay prototype driver.
//
// 1. Loads one MWM's car IndexGraph exactly as the router does and extracts the full segment
//    graph (vertex = directed segment, edge weights from IndexGraph::GetEdgeList).
// 2. Compresses non-junction chains, partitions into nested cells, customizes the overlay.
// 3. Answers in-MWM queries with the overlay and with plain bidirectional Dijkstra, compares.
// 4. Checks the plain answers against CoMaps' own AStarAlgorithm on IndexGraph.
// 5. Avoid-place test: blocks an area, recustomizes only the touched cells, re-checks.
//
// Example:
//   plus_overlay_tool --resources_path=data --maps_path=~/comaps-bench/maps
//     --mwm="India_Uttar Pradesh" --mwm_version=261004 --cache=/tmp/up.graph   (one line)

#include "routing/plus_overlay/overlay.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/base/astar_graph.hpp"
#include "routing/base/routing_result.hpp"
#include "routing/data_source.hpp"
#include "routing/edge_estimator.hpp"
#include "routing/index_graph.hpp"
#include "routing/index_graph_loader.hpp"
#include "routing/restriction_loader.hpp"
#include "routing/road_access.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"

#include "routing_common/car_model.hpp"
#include "routing_common/num_mwm_id.hpp"

#include "storage/country_parent_getter.hpp"

#include "indexer/classificator_loader.hpp"
#include "indexer/data_source.hpp"

#include "platform/local_country_file.hpp"
#include "platform/platform.hpp"

#include "base/cancellable.hpp"
#include "base/logging.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>

DEFINE_string(resources_path, "", "CoMaps data/ directory (classificator, countries.txt).");
DEFINE_string(maps_path, "", "Directory that contains <version>/<mwm>.mwm.");
DEFINE_string(mwm, "India_Uttar Pradesh", "Map to load.");
DEFINE_int64(mwm_version, 261004, "Map version directory.");
DEFINE_string(cache, "", "Raw segment graph cache file (written after extraction, read if present).");
DEFINE_string(cell_sizes, "256,4096,65536,1048576", "Max vertices per cell, level 1 first.");
DEFINE_double(min_balance, 0.3, "Smallest share of a cell each bisection half may get.");
DEFINE_uint32(threads, 2, "Customization threads.");
DEFINE_uint32(random_queries, 100, "Random in-component query pairs.");
DEFINE_uint32(verify, 3, "Named pairs to check against CoMaps' AStarAlgorithm (slow, in order of the list).");
DEFINE_bool(avoid_test, true, "Run the avoid-place recustomization test.");
DEFINE_double(avoid_radius_m, 1500, "Radius of the avoided area.");
DEFINE_uint64(seed, 42, "Random seed.");

using namespace plus_overlay;

namespace
{
using Clock = std::chrono::steady_clock;
double Since(Clock::time_point t0)
{
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

std::string Mem()
{
  std::ifstream f("/proc/self/status");
  std::string line, rss, hwm;
  while (std::getline(f, line))
  {
    if (line.rfind("VmRSS:", 0) == 0)
      rss = line.substr(6);
    if (line.rfind("VmHWM:", 0) == 0)
      hwm = line.substr(6);
  }
  auto trim = [](std::string s)
  {
    s.erase(0, s.find_first_not_of(" \t"));
    return s;
  };
  return "rss " + trim(rss) + ", peak " + trim(hwm);
}

std::string MB(uint64_t bytes)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.1f MB", bytes / 1048576.0);
  return buf;
}

// Raw extracted graph + how to map a raw vertex back to a CoMaps Segment.
struct Raw
{
  Graph g;
  std::vector<uint8_t> valid;
  std::vector<uint32_t> featBase;  // segment index of the first segment of feature f; size maxF + 2

  // Vertices >= baseN are copies of segments made for via-way restriction states.
  uint32_t baseN = 0;
  std::vector<VertexId> copyOrig;

  VertexId Orig(VertexId v) const { return v < baseN ? v : copyOrig[v - baseN]; }
  uint32_t FeatureOf(VertexId v) const
  {
    uint32_t const segIdx = Orig(v) / 2;
    return static_cast<uint32_t>(std::upper_bound(featBase.begin(), featBase.end(), segIdx) - featBase.begin()) - 1;
  }
  routing::Segment ToSegment(routing::NumMwmId mwm, VertexId v) const
  {
    v = Orig(v);
    uint32_t const segIdx = v / 2;
    auto it = std::upper_bound(featBase.begin(), featBase.end(), segIdx);
    uint32_t const f = static_cast<uint32_t>(it - featBase.begin()) - 1;
    return routing::Segment(mwm, f, segIdx - featBase[f], (v % 2) == 0);
  }
  VertexId FromSegment(routing::Segment const & s) const
  {
    return 2 * (featBase[s.GetFeatureId()] + s.GetSegmentIdx()) + (s.IsForward() ? 0 : 1);
  }
};

template <typename T>
void WriteVec(std::ofstream & out, std::vector<T> const & v)
{
  uint64_t const n = v.size();
  out.write(reinterpret_cast<char const *>(&n), sizeof(n));
  out.write(reinterpret_cast<char const *>(v.data()), n * sizeof(T));
}
template <typename T>
void ReadVec(std::ifstream & in, std::vector<T> & v)
{
  uint64_t n = 0;
  in.read(reinterpret_cast<char *>(&n), sizeof(n));
  v.resize(n);
  in.read(reinterpret_cast<char *>(v.data()), n * sizeof(T));
}

constexpr uint64_t kMagic = 0x31474F4C52455650ULL;  // "PVERLOG1"

void SaveRaw(Raw const & r, std::string const & path)
{
  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<char const *>(&kMagic), sizeof(kMagic));
  out.write(reinterpret_cast<char const *>(&r.g.n), sizeof(r.g.n));
  WriteVec(out, r.featBase);
  WriteVec(out, r.valid);
  WriteVec(out, r.g.fwdOff);
  WriteVec(out, r.g.fwdTo);
  WriteVec(out, r.g.fwdW);
  WriteVec(out, r.g.lat);
  WriteVec(out, r.g.lon);
}

bool LoadRaw(Raw & r, std::string const & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in)
    return false;
  uint64_t magic = 0;
  in.read(reinterpret_cast<char *>(&magic), sizeof(magic));
  if (magic != kMagic)
    return false;
  in.read(reinterpret_cast<char *>(&r.g.n), sizeof(r.g.n));
  ReadVec(in, r.featBase);
  ReadVec(in, r.valid);
  ReadVec(in, r.g.fwdOff);
  ReadVec(in, r.g.fwdTo);
  ReadVec(in, r.g.fwdW);
  ReadVec(in, r.g.lat);
  ReadVec(in, r.g.lon);
  return static_cast<bool>(in);
}

// CoMaps side: data source, loader and the IndexGraph of one MWM, built like IndexRouter does.
struct CoMaps
{
  FrozenDataSource dataSource;
  std::shared_ptr<routing::NumMwmIds> numMwmIds = std::make_shared<routing::NumMwmIds>();
  std::unique_ptr<routing::MwmDataSource> mwmDataSource;
  std::shared_ptr<routing::VehicleModelFactoryInterface> factory;
  std::shared_ptr<routing::EdgeEstimator> estimator;
  std::unique_ptr<routing::IndexGraphLoader> loader;
  routing::NumMwmId mwmId = 0;
  routing::IndexGraph * graph = nullptr;
  std::shared_ptr<storage::CountryParentGetter> cpg;
  time_t fixedTime = 0;
  std::vector<std::vector<uint32_t>> viaWay;  // forbidden feature chains [from, via..., to], 3+ features

  void Init()
  {
    auto & pl = GetPlatform();
    pl.SetResourceDir(FLAGS_resources_path + "/");
    pl.SetWritableDirForTests(FLAGS_maps_path + "/");
    classificator::Load();

    platform::CountryFile const countryFile(FLAGS_mwm);
    platform::LocalCountryFile localFile(FLAGS_maps_path + "/" + std::to_string(FLAGS_mwm_version), countryFile,
                                         FLAGS_mwm_version);
    localFile.SyncWithDisk();
    auto const res = dataSource.RegisterMap(localFile);
    CHECK_EQUAL(res.second, MwmSet::RegResult::Success, ("Can't register", localFile));
    numMwmIds->RegisterFile(countryFile);
    mwmId = numMwmIds->GetId(countryFile);
    mwmDataSource = std::make_unique<routing::MwmDataSource>(dataSource, numMwmIds);

    cpg = std::make_shared<storage::CountryParentGetter>();
    auto parentGetter = [cpg = cpg](std::string const & id) { return (*cpg)(id); };
    factory = std::make_shared<routing::CarModelFactory>(parentGetter);
    double const maxSpeed = factory->GetVehicleModelForCountry(FLAGS_mwm)->GetMaxWeightSpeed();
    estimator = routing::EdgeEstimator::Create(routing::VehicleType::Car, maxSpeed,
                                               factory->GetVehicleModel()->GetOffroadSpeed(), nullptr /* traffic */,
                                               &dataSource, numMwmIds);
    fixedTime = std::time(nullptr);
    loader = routing::IndexGraphLoader::Create(routing::VehicleType::Car, false /* loadAltitudes */, factory,
                                               estimator, *mwmDataSource, routing::RoutingOptions(),
                                               [t = fixedTime]() { return t; });
    graph = &loader->GetIndexGraph(mwmId);
  }

  void ReportRestrictionsAndAccess()
  {
    auto const & value = mwmDataSource->GetMwmValue(mwmId);
    routing::RestrictionLoader rl(value, *graph);
    auto restrictions = rl.StealRestrictions();
    size_t bySize[6] = {};
    for (auto const & r : restrictions)
    {
      ++bySize[std::min<size_t>(r.size(), 5)];
      if (r.size() >= 3)
        viaWay.push_back(r);
    }
    std::cout << "  restrictions (as 'no' chains): total " << restrictions.size() << ", 2-feature " << bySize[2]
              << ", via-way (3+ features) " << (bySize[3] + bySize[4] + bySize[5]) << "\n";
    routing::RoadAccess ra;
    if (routing::ReadRoadAccessFromMwm(value, routing::VehicleType::Car, ra))
    {
      std::cout << "  conditional access: ways " << ra.GetWayToAccessConditional().size() << ", points "
                << ra.GetPointToAccessConditional().size() << "\n";
    }
  }
};

void Extract(CoMaps & cm, Raw & raw)
{
  auto const t0 = Clock::now();
  routing::IndexGraph & ig = *cm.graph;

  std::vector<uint32_t> roads;
  ig.ForEachRoad([&](uint32_t f, routing::RoadJointIds const &) { roads.push_back(f); });
  std::sort(roads.begin(), roads.end());
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
  raw.featBase.assign(maxF + 2, 0);
  for (uint32_t f = 0; f <= maxF; ++f)
    raw.featBase[f + 1] = raw.featBase[f] + segs[f];
  uint32_t const numSeg = raw.featBase[maxF + 1];
  Graph & g = raw.g;
  g.n = 2 * numSeg;
  raw.valid.assign(g.n, 0);
  g.fwdOff.assign(g.n + 1, 0);
  g.lat.assign(g.n, 0);
  g.lon.assign(g.n, 0);
  g.fwdTo.reserve(static_cast<size_t>(g.n) * 1.1);
  g.fwdW.reserve(static_cast<size_t>(g.n) * 1.1);
  std::cout << "  roads " << roads.size() << ", segments " << numSeg << ", directed vertex slots " << g.n << " ("
            << Since(t0) << " s)\n";

  uint64_t penalized = 0;
  VertexId next = 0;  // next vertex whose offset is not yet written
  routing::IndexGraph::SegmentEdgeListT edges;
  size_t done = 0;
  for (uint32_t f : roads)
  {
    if (++done % (roads.size() / 10 + 1) == 0)
      std::cout << "  extracting " << (100 * done / roads.size()) << "% (" << Since(t0) << " s, " << Mem() << ")\n"
                << std::flush;
    for (uint32_t i = 0; i < segs[f]; ++i)
    {
      for (bool fwd : {true, false})
      {
        if (!fwd && oneway[f])
          continue;
        routing::Segment const s(cm.mwmId, f, i, fwd);
        VertexId const v = raw.FromSegment(s);
        while (next <= v)
          g.fwdOff[++next] = g.fwdTo.size();  // close earlier (possibly empty) vertices
        raw.valid[v] = 1;
        auto const & pt = ig.GetPoint(s, true /* front */);
        g.lat[v] = static_cast<float>(pt.m_lat);
        g.lon[v] = static_cast<float>(pt.m_lon);
        edges.clear();
        ig.GetEdgeList(s, true /* isOutgoing */, true /* useRoutingOptions */, edges);
        for (auto const & e : edges)
        {
          auto const & w = e.GetWeight();
          double const iw = w.GetIntegratedWeight();
          if (iw != w.GetWeight())
            ++penalized;
          g.fwdTo.push_back(raw.FromSegment(e.GetTarget()));
          g.fwdW.push_back(iw);
        }
        g.fwdOff[v + 1] = g.fwdTo.size();
      }
    }
  }
  while (next < g.n)
    g.fwdOff[++next] = g.fwdTo.size();
  uint64_t validCount = 0;
  for (auto x : raw.valid)
    validCount += x;
  std::cout << "  extracted " << validCount << " directed segments, " << g.NumEdges() << " edges (" << penalized
            << " with pass-through/access penalties) in " << Since(t0) << " s, " << Mem() << "\n";
}


// Via-way restrictions ---------------------------------------------------------------------------
//
// CoMaps forbids a sequence of distinct consecutive features [F0, F1, ..., Fk] (k >= 2) by looking at
// the A* parents. A parent-dependent check is not exact for a label-setting search, so here the
// state is made explicit instead: an Aho-Corasick automaton over the feature sequence, and a copy
// of a segment for every automaton state of depth >= 2 it can be in. Forbidden transitions become
// missing edges, so everything downstream (compression, cells, queries) stays a plain graph search.
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

void ExpandViaWay(Raw & raw, std::vector<std::vector<uint32_t>> const & patterns)
{
  auto const t0 = Clock::now();
  Graph & g = raw.g;
  raw.baseN = g.n;
  if (patterns.empty())
    return;
  ViaWayAutomaton ac(patterns);

  // Copies: (state, original vertex) for every non-terminal state of depth >= 2.
  std::vector<std::pair<uint32_t, VertexId>> copies;
  std::map<std::pair<uint32_t, VertexId>, VertexId> copyId;
  for (uint32_t n = 0; n < ac.Size(); ++n)
  {
    if (ac.Depth(n) < 2 || ac.Terminal(n))
      continue;
    uint32_t const f = ac.Label(n);
    for (uint32_t v = 2 * raw.featBase[f]; v < 2 * raw.featBase[f + 1]; ++v)
    {
      if (!raw.valid[v])
        continue;
      copyId[{n, v}] = g.n + static_cast<VertexId>(copies.size());
      copies.emplace_back(n, v);
    }
  }

  std::vector<uint32_t> feat(g.n);
  for (VertexId v = 0; v < g.n; ++v)
    feat[v] = raw.FeatureOf(v);

  uint64_t dropped = 0, redirected = 0;
  Graph out;
  out.n = g.n + static_cast<uint32_t>(copies.size());
  out.fwdOff.assign(out.n + 1, 0);
  out.fwdTo.reserve(g.fwdTo.size() + copies.size() * 3);
  out.fwdW.reserve(g.fwdTo.size() + copies.size() * 3);
  auto emit = [&](VertexId v, uint32_t state)
  {
    for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
    {
      VertexId const t = g.fwdTo[e];
      uint32_t next = state;
      if (feat[t] != feat[v])
        next = ac.Step(state, feat[t]);
      if (ac.Terminal(next))
      {
        ++dropped;
        continue;
      }
      VertexId target = t;
      if (ac.Depth(next) >= 2)
      {
        target = copyId.at({next, t});
        ++redirected;
      }
      out.fwdTo.push_back(target);
      out.fwdW.push_back(g.fwdW[e]);
    }
  };
  for (VertexId v = 0; v < g.n; ++v)
  {
    if (raw.valid[v])
      emit(v, ac.Start(feat[v]));
    out.fwdOff[v + 1] = out.fwdTo.size();
  }
  for (size_t i = 0; i < copies.size(); ++i)
  {
    emit(copies[i].second, copies[i].first);
    out.fwdOff[g.n + i + 1] = out.fwdTo.size();
  }
  out.lat = g.lat;
  out.lon = g.lon;
  raw.copyOrig.clear();
  for (auto const & [n, v] : copies)
  {
    out.lat.push_back(g.lat[v]);
    out.lon.push_back(g.lon[v]);
    raw.copyOrig.push_back(v);
  }
  raw.valid.resize(out.n, 1);
  g = std::move(out);
  std::cout << "  via-way restrictions: " << patterns.size() << " patterns, automaton states " << ac.Size()
            << ", segment copies " << copies.size() << ", edges redirected " << redirected << ", forbidden "
            << dropped << " (" << Since(t0) << " s)\n";
}

// Cheapest raw edge u -> v, or kInf.
double RawEdge(Graph const & g, VertexId u, VertexId v)
{
  double w = kInf;
  for (uint64_t e = g.fwdOff[u]; e < g.fwdOff[u + 1]; ++e)
    if (g.fwdTo[e] == v)
      w = std::min(w, g.fwdW[e]);
  return w;
}

// Compressed vertex path -> raw vertex path, walking the chain behind every compressed edge.
std::vector<VertexId> ToRawPath(Graph const & raw, Compressed const & comp, std::vector<uint32_t> const & rawToComp,
                                std::vector<VertexId> const & path)
{
  std::vector<VertexId> out;
  if (path.empty())
    return out;
  out.push_back(comp.origId[path[0]]);
  std::vector<VertexId> chain, best;
  for (size_t i = 1; i < path.size(); ++i)
  {
    VertexId const ru = comp.origId[path[i - 1]], rw = comp.origId[path[i]];
    double bestW = kInf;
    for (uint64_t e = raw.fwdOff[ru]; e < raw.fwdOff[ru + 1]; ++e)
    {
      chain.assign(1, raw.fwdTo[e]);
      double w = raw.fwdW[e];
      VertexId x = raw.fwdTo[e];
      while (rawToComp[x] == kNone && raw.fwdOff[x + 1] - raw.fwdOff[x] == 1 && chain.size() <= raw.n)
      {
        w += raw.fwdW[raw.fwdOff[x]];
        x = raw.fwdTo[raw.fwdOff[x]];
        chain.push_back(x);
      }
      if (x == rw && w < bestW)
      {
        bestW = w;
        best = chain;
      }
    }
    out.insert(out.end(), best.begin(), best.end());
  }
  return out;
}

double RawPathWeight(Graph const & raw, std::vector<VertexId> const & p)
{
  double w = 0;
  for (size_t i = 1; i < p.size(); ++i)
    w += RawEdge(raw, p[i - 1], p[i]);
  return w;
}

// Distinct consecutive features along a segment path.
std::vector<uint32_t> FeatureSequence(std::vector<routing::Segment> const & segs)
{
  std::vector<uint32_t> out;
  for (auto const & s : segs)
    if (out.empty() || out.back() != s.GetFeatureId())
      out.push_back(s.GetFeatureId());
  return out;
}

// Number of forbidden via-way chains that appear in the path.
size_t CountViolations(std::vector<routing::Segment> const & segs, std::vector<std::vector<uint32_t>> const & patterns)
{
  auto const fs = FeatureSequence(segs);
  size_t n = 0;
  for (auto const & p : patterns)
    for (size_t i = 0; i + p.size() <= fs.size(); ++i)
      if (std::equal(p.begin(), p.end(), fs.begin() + i))
        ++n;
  return n;
}

// CoMaps reference: AStarAlgorithm (zero heuristic) directly on IndexGraph, with parents so
// via-way restrictions are applied the way the router applies them.
class IndexGraphAStar final : public routing::AStarGraph<routing::Segment, routing::SegmentEdge, routing::RouteWeight>
{
public:
  // ignoreParents: behave as if no via-way restriction existed (parents are what CoMaps uses to
  // check them). Used to show which test routes the restrictions actually change.
  IndexGraphAStar(routing::IndexGraph & g, bool ignoreParents = false) : m_g(g), m_ignoreParents(ignoreParents) {}
  Weight HeuristicCostEstimate(Vertex const &, Vertex const &) override { return routing::GetAStarWeightZero<Weight>(); }
  void GetOutgoingEdgesList(routing::astar::VertexData<Vertex, Weight> const & vd, EdgeListT & edges) override
  {
    edges.clear();
    m_g.GetEdgeList(vd, true, true, edges, (m_fwd && !m_ignoreParents) ? *m_fwd : m_empty);
  }
  void GetIngoingEdgesList(routing::astar::VertexData<Vertex, Weight> const & vd, EdgeListT & edges) override
  {
    edges.clear();
    m_g.GetEdgeList(vd, false, true, edges, (m_bwd && !m_ignoreParents) ? *m_bwd : m_empty);
  }
  void SetAStarParents(bool forward, Parents & parents) override { (forward ? m_fwd : m_bwd) = &parents; }
  void DropAStarParents() override { m_fwd = m_bwd = nullptr; }
  // Same epsilon IndexGraphStarter uses for single-MWM double precision.
  Weight GetAStarWeightEpsilon() override { return routing::RouteWeight(1e-6); }

private:
  routing::IndexGraph & m_g;
  bool m_ignoreParents = false;
  Parents * m_fwd = nullptr;
  Parents * m_bwd = nullptr;
  Parents m_empty;
};

struct Place
{
  char const * name;
  double lat, lon;
};
Place const kPlaces[] = {
    {"Lucknow", 26.8467, 80.9462},   {"Kanpur", 26.4499, 80.3319},    {"Agra", 27.1767, 78.0081},
    {"Varanasi", 25.3176, 82.9739},  {"Gorakhpur", 26.7606, 83.3732}, {"Noida", 28.5355, 77.3910},
    {"Jhansi", 25.4484, 78.5685},    {"Prayagraj", 25.4358, 81.8463}, {"Bareilly", 28.3670, 79.4304},
    {"Saharanpur", 29.9680, 77.5510},
};
std::pair<int, int> const kPairs[] = {{0, 1}, {0, 2}, {0, 3}, {5, 4}, {9, 3}, {6, 4}, {8, 7}, {2, 0}};

VertexId Nearest(Graph const & g, std::vector<uint8_t> const & inScc, double lat, double lon)
{
  double best = 1e30;
  VertexId bv = kNone;
  double const k = std::cos(lat * M_PI / 180.0);
  for (uint32_t v = 0; v < g.n; ++v)
  {
    if (!inScc[v])
      continue;
    double const dy = g.lat[v] - lat, dx = (g.lon[v] - lon) * k;
    double const d = dx * dx + dy * dy;
    if (d < best)
    {
      best = d;
      bv = v;
    }
  }
  return bv;
}

bool Same(double a, double b)
{
  if (a == kInf || b == kInf)
    return a == b;
  return std::abs(a - b) <= 1e-6 * std::max(1.0, std::abs(a));
}

double Percentile(std::vector<double> v, double q)
{
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, static_cast<size_t>(q * v.size()))];
}

std::vector<uint32_t> ParseSizes(std::string const & s)
{
  std::vector<uint32_t> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ','))
    out.push_back(static_cast<uint32_t>(std::stoul(tok)));
  return out;
}

std::string Hms(double seconds)
{
  char buf[32];
  int const s = static_cast<int>(std::lround(seconds));
  std::snprintf(buf, sizeof(buf), "%dh%02dm", s / 3600, (s % 3600) / 60);
  return buf;
}
}  // namespace

int main(int argc, char ** argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << std::unitbuf;
  std::cout.setf(std::ios::fixed);
  std::cout.precision(3);
  std::cout << "== Comaps++ overlay prototype: " << FLAGS_mwm << " v" << FLAGS_mwm_version << "\n";

  std::unique_ptr<CoMaps> cm;
  auto ensureCoMaps = [&]()
  {
    if (!cm)
    {
      auto const t0 = Clock::now();
      cm = std::make_unique<CoMaps>();
      cm->Init();
      std::cout << "  CoMaps IndexGraph loaded in " << Since(t0) << " s, " << Mem() << "\n";
      cm->ReportRestrictionsAndAccess();
    }
  };

  // 1. Raw graph.
  std::cout << "\n[1] Raw segment graph\n";
  Raw raw;
  ensureCoMaps();
  if (FLAGS_cache.empty() || !LoadRaw(raw, FLAGS_cache))
  {
    Extract(*cm, raw);
    if (!FLAGS_cache.empty())
      SaveRaw(raw, FLAGS_cache);
  }
  else
  {
    std::cout << "  loaded from cache " << FLAGS_cache << "\n";
  }
  ExpandViaWay(raw, cm->viaWay);
  std::cout << "  vertices " << raw.g.n << ", edges " << raw.g.NumEdges() << "\n";

  // 2. Compression.
  std::cout << "\n[2] Chain compression\n";
  auto t0 = Clock::now();
  Compressed comp = CompressChains(raw.g, raw.valid);
  Graph & g = comp.g;
  std::cout << "  " << g.n << " vertices, " << g.NumEdges() << " edges in " << Since(t0) << " s, " << Mem() << "\n";
  // The raw graph stays: it turns overlay answers into segment lists.
  std::vector<uint32_t> rawToComp(raw.g.n, kNone);
  for (uint32_t cv = 0; cv < g.n; ++cv)
    rawToComp[comp.origId[cv]] = cv;

  // Main strongly connected component, seeded near Lucknow.
  std::vector<uint8_t> all(g.n, 1);
  VertexId const seed = Nearest(g, all, kPlaces[0].lat, kPlaces[0].lon);
  uint32_t sccSize = 0;
  auto inScc = StronglyConnectedFrom(g, seed, sccSize);
  // Routes start and end on real segments, never on via-way state copies.
  std::vector<uint8_t> startOk(g.n, 0);
  for (uint32_t v = 0; v < g.n; ++v)
    startOk[v] = inScc[v] && comp.origId[v] < raw.baseN;
  std::cout << "  main strongly connected component: " << sccSize << " vertices (" << (100.0 * sccSize / g.n)
            << "%)\n";

  // 3. Partition.
  std::cout << "\n[3] Partition\n";
  PartitionParams pp;
  pp.cellSizes = ParseSizes(FLAGS_cell_sizes);
  pp.minBalance = FLAGS_min_balance;
  t0 = Clock::now();
  Partition part = Bisect(g, pp);
  double const partSec = Since(t0);
  std::cout << "  done in " << partSec << " s, " << Mem() << "\n";

  // 4. Customization.
  std::cout << "\n[4] Overlay\n";
  Overlay ov;
  t0 = Clock::now();
  BuildBoundaries(g, part, ov);
  std::cout << "  boundaries in " << Since(t0) << " s\n";
  for (size_t li = 0; li < part.Levels(); ++li)
  {
    auto const & L = ov.levels[li];
    uint64_t maxEn = 0, maxEx = 0;
    for (uint32_t c = 0; c < part.numCells[li]; ++c)
    {
      maxEn = std::max<uint64_t>(maxEn, L.entryOff[c + 1] - L.entryOff[c]);
      maxEx = std::max<uint64_t>(maxEx, L.exitOff[c + 1] - L.exitOff[c]);
    }
    std::cout << "  level " << (li + 1) << ": cells " << part.numCells[li] << ", entries " << L.entries.size()
              << " (avg " << double(L.entries.size()) / part.numCells[li] << ", max " << maxEn << "), exits "
              << L.exits.size() << " (avg " << double(L.exits.size()) / part.numCells[li] << ", max " << maxEx
              << "), matrix entries " << L.mat.size() << "\n";
  }
  t0 = Clock::now();
  auto levelSec = Customize(g, part, ov, FLAGS_threads);
  double const custSec = Since(t0);
  for (size_t li = 0; li < levelSec.size(); ++li)
    std::cout << "  customize level " << (li + 1) << ": " << levelSec[li] << " s\n";
  std::cout << "  customize total " << custSec << " s (" << FLAGS_threads << " threads), " << Mem() << "\n";

  auto const st = MeasureStorage(g, part, ov);
  uint64_t const graphBytes = (g.n + 1ULL) * 8 + g.NumEdges() * (4 + 8);
  std::cout << "  storage: matrices " << st.matrixEntries << " entries (" << st.finiteEntries << " finite) = "
            << MB(st.matrixEntries * 8) << " as double, " << MB(st.matrixEntries * 4) << " as float32\n"
            << "           boundary ids " << MB(st.boundaryIds * 4) << ", cell ids " << MB(st.cellIdBytes)
            << " (uint32 per level; packable)\n"
            << "           compressed base graph (prototype CSR, not needed on device) " << MB(graphBytes) << "\n";

  // 5. Queries.
  std::cout << "\n[5] Queries (overlay vs plain bidirectional Dijkstra on the same graph)\n";
  Scratch sc(g.n);
  std::vector<VertexId> placeV;
  for (auto const & pl : kPlaces)
    placeV.push_back(Nearest(g, startOk, pl.lat, pl.lon));

  // Overlay answer -> raw path -> CoMaps segments, with consistency checks.
  auto toSegments = [&](std::vector<VertexId> const & compPath)
  {
    std::vector<routing::Segment> segs;
    for (VertexId rv : ToRawPath(raw.g, comp, rawToComp, compPath))
      segs.push_back(raw.ToSegment(cm->mwmId, rv));
    return segs;
  };
  auto pathReport = [&](QueryResult const & r, std::vector<routing::Segment> & segs)
  {
    if (r.weight == kInf)
      return std::string("no route");
    if (r.path.empty())
      return std::string("UNPACK FAILED");
    auto const rawPath = ToRawPath(raw.g, comp, rawToComp, r.path);
    segs.clear();
    for (VertexId rv : rawPath)
      segs.push_back(raw.ToSegment(cm->mwmId, rv));
    double const pw = RawPathWeight(raw.g, rawPath);
    size_t const viol = CountViolations(segs, cm->viaWay);
    std::string out = std::to_string(segs.size()) + " segments, path weight " +
                      (Same(pw, r.weight) ? "ok" : "WRONG (" + std::to_string(pw) + ")");
    if (viol)
      out += ", " + std::to_string(viol) + " VIA-WAY VIOLATIONS";
    return out;
  };

  uint64_t mismatches = 0;
  double maxAbsDiff = 0;
  struct Named
  {
    int a, b;
    double w;
    std::vector<routing::Segment> segs;
  };
  std::vector<Named> named;
  for (auto const & pr : kPairs)
  {
    VertexId const s = placeV[pr.first], t = placeV[pr.second];
    auto t1 = Clock::now();
    auto base = BidirectionalDijkstra(g, s, t, sc);
    double const bs = Since(t1);
    t1 = Clock::now();
    auto ovr = OverlayQuery(g, part, ov, s, t, sc);
    double const os = Since(t1);
    t1 = Clock::now();
    auto withPath = OverlayQuery(g, part, ov, s, t, sc, true /* wantPath */);
    double const ps = Since(t1);
    bool const ok = Same(base.weight, ovr.weight);
    mismatches += !ok;
    if (base.weight < kInf)
      maxAbsDiff = std::max(maxAbsDiff, std::abs(base.weight - ovr.weight));
    named.push_back({pr.first, pr.second, base.weight, {}});
    std::string const pr2 = pathReport(withPath, named.back().segs);
    std::cout << "  " << kPlaces[pr.first].name << " -> " << kPlaces[pr.second].name << ": weight "
              << Hms(base.weight) << " (" << base.weight << " s) | dijkstra " << bs * 1000 << " ms, "
              << base.settled << " settled | overlay " << os * 1000 << " ms, " << ovr.settled << " settled | "
              << (ok ? "MATCH" : "MISMATCH") << "\n"
              << "      with path unpacking " << ps * 1000 << " ms: " << pr2 << "\n";
  }

  std::vector<VertexId> sccList;
  for (uint32_t v = 0; v < g.n; ++v)
    if (startOk[v])
      sccList.push_back(v);
  std::mt19937_64 rng(FLAGS_seed);
  std::vector<double> bT, oT, bS, oS;
  for (uint32_t q = 0; q < FLAGS_random_queries; ++q)
  {
    VertexId const s = sccList[rng() % sccList.size()], t = sccList[rng() % sccList.size()];
    auto t1 = Clock::now();
    auto base = BidirectionalDijkstra(g, s, t, sc);
    bT.push_back(Since(t1) * 1000);
    bS.push_back(base.settled);
    t1 = Clock::now();
    auto ovr = OverlayQuery(g, part, ov, s, t, sc);
    oT.push_back(Since(t1) * 1000);
    oS.push_back(ovr.settled);
    if (!Same(base.weight, ovr.weight))
    {
      ++mismatches;
      std::cout << "  MISMATCH random " << s << " -> " << t << ": " << base.weight << " vs " << ovr.weight << "\n";
    }
    if (base.weight < kInf)
      maxAbsDiff = std::max(maxAbsDiff, std::abs(base.weight - ovr.weight));
  }
  std::cout << "  random pairs: " << FLAGS_random_queries << ", mismatches total " << mismatches
            << ", max |diff| " << maxAbsDiff << " s\n"
            << "  dijkstra: median " << Percentile(bT, 0.5) << " ms, p90 " << Percentile(bT, 0.9)
            << " ms, median settled " << Percentile(bS, 0.5) << "\n"
            << "  overlay:  median " << Percentile(oT, 0.5) << " ms, p90 " << Percentile(oT, 0.9)
            << " ms, median settled " << Percentile(oS, 0.5) << "\n";

  // 6. CoMaps reference.
  if (FLAGS_verify > 0)
  {
    std::cout << "\n[6] Check against CoMaps AStarAlgorithm on IndexGraph (bidirectional, zero heuristic)\n";
    ensureCoMaps();
    IndexGraphAStar astarGraph(*cm->graph);
    routing::AStarAlgorithm<routing::Segment, routing::SegmentEdge, routing::RouteWeight> algo;
    base::Cancellable cancellable;
    for (uint32_t i = 0; i < FLAGS_verify && i < named.size(); ++i)
    {
      auto const & nm = named[i];
      auto const s = raw.ToSegment(cm->mwmId, comp.origId[placeV[nm.a]]);
      auto const t = raw.ToSegment(cm->mwmId, comp.origId[placeV[nm.b]]);
      routing::AStarAlgorithm<routing::Segment, routing::SegmentEdge, routing::RouteWeight>::Params<> params(
          astarGraph, s, t, cancellable);
      routing::RoutingResult<routing::Segment, routing::RouteWeight> res;
      auto const t1 = Clock::now();
      auto const code = algo.FindPathBidirectional(params, res);
      double const secs = Since(t1);
      double const cw = code == decltype(code)::OK ? res.m_distance.GetIntegratedWeight() : kInf;
      std::string pathCmp = "path IDENTICAL";
      if (res.m_path != nm.segs)
      {
        size_t i0 = 0;
        while (i0 < res.m_path.size() && i0 < nm.segs.size() && res.m_path[i0] == nm.segs[i0])
          ++i0;
        pathCmp = "path differs from segment " + std::to_string(i0) + " (equal-weight alternative)";
      }
      size_t const cViol = CountViolations(res.m_path, cm->viaWay);
      std::cout << "  " << kPlaces[nm.a].name << " -> " << kPlaces[nm.b].name << ": CoMaps " << cw
                << " s in " << secs << " s (" << res.m_path.size() << " segments) | prototype " << nm.w << " s | "
                << (Same(cw, nm.w) ? "MATCH" : "DIFF " + std::to_string(cw - nm.w) + " s") << " | " << pathCmp
                << (cViol ? " | CoMaps path has via-way violations" : "") << "\n";
    }
  }


  // 6b. Routes through every via-way restriction.
  if (!cm->viaWay.empty())
  {
    std::cout << "\n[6b] Via-way restrictions: a short route through each one\n"
              << "     (weights in s; 'ignoring' = CoMaps with the restriction switched off; CoMaps capped at 1800 s)\n";
    using Algo = routing::AStarAlgorithm<routing::Segment, routing::SegmentEdge, routing::RouteWeight>;
    IndexGraphAStar strict(*cm->graph), loose(*cm->graph, true /* ignoreParents */);
    Algo algo;
    base::Cancellable cancellable;
    auto comaps = [&](IndexGraphAStar & gr, routing::Segment const & a, routing::Segment const & b, bool bidir,
                      std::vector<routing::Segment> & path)
    {
      // Capped: these are local routes through one junction; an uncapped search on an unreachable
      // pair would sweep the whole state several times over.
      struct Cap
      {
        bool operator()(routing::RouteWeight const & w) const { return w.GetIntegratedWeight() <= 1800.0; }
      };
      Algo::Params<routing::astar::DefaultVisitor, Cap> params(gr, a, b, cancellable, routing::astar::DefaultVisitor(),
                                                              Cap());
      routing::RoutingResult<routing::Segment, routing::RouteWeight> res;
      auto const code = bidir ? algo.FindPathBidirectional(params, res) : algo.FindPath(params, res);
      path = res.m_path;
      return code == Algo::Result::OK ? res.m_distance.GetIntegratedWeight() : kInf;
    };

    // Start/end candidates per feature, among real (non-copy) compressed vertices.
    std::map<uint32_t, std::vector<VertexId>> byFeature;
    for (auto const & pat : cm->viaWay)
      for (uint32_t f : pat)
        byFeature[f];
    for (uint32_t v = 0; v < g.n; ++v)
    {
      if (!startOk[v])
        continue;
      auto it = byFeature.find(raw.FeatureOf(comp.origId[v]));
      if (it != byFeature.end())
        it->second.push_back(v);
    }
    auto centroid = [&](uint32_t f)
    {
      double la = 0, lo = 0;
      uint32_t n = 0;
      for (VertexId v = 2 * raw.featBase[f]; v < 2 * raw.featBase[f + 1]; ++v)
      {
        la += raw.g.lat[v];
        lo += raw.g.lon[v];
        ++n;
      }
      return std::make_pair(n ? la / n : 0.0, n ? lo / n : 0.0);
    };
    auto nearestOf = [&](uint32_t f, std::pair<double, double> c)
    {
      VertexId best = kNone;
      double bd = 1e30;
      for (VertexId v : byFeature[f])
      {
        double const d = std::pow(g.lat[v] - c.first, 2) + std::pow(g.lon[v] - c.second, 2);
        if (d < bd)
        {
          bd = d;
          best = v;
        }
      }
      return best;
    };

    uint32_t idx = 0, agree = 0, changed = 0, tested = 0;
    for (auto const & pat : cm->viaWay)
    {
      ++idx;
      VertexId const s = nearestOf(pat.front(), centroid(pat[1]));
      VertexId const t = nearestOf(pat.back(), centroid(pat[pat.size() - 2]));
      std::cout << "  #" << idx << " (" << pat.size() << " features): ";
      if (s == kNone || t == kNone || s == t)
      {
        std::cout << "skipped, no usable start/end segment on the first/last feature\n";
        continue;
      }
      ++tested;
      auto const base = BidirectionalDijkstra(g, s, t, sc);
      auto const ovr = OverlayQuery(g, part, ov, s, t, sc, true);
      std::vector<routing::Segment> ours;
      std::string const rep = pathReport(ovr, ours);
      auto const a = raw.ToSegment(cm->mwmId, comp.origId[s]);
      auto const b = raw.ToSegment(cm->mwmId, comp.origId[t]);
      std::vector<routing::Segment> pUni, pBi, pLoose;
      double const cUni = comaps(strict, a, b, false, pUni);
      double const cBi = comaps(strict, a, b, true, pBi);
      double const cLoose = comaps(loose, a, b, false, pLoose);
      bool const ok = Same(base.weight, ovr.weight);
      mismatches += !ok;
      double const capped = ovr.weight <= 1800.0 ? ovr.weight : kInf;
      agree += Same(cUni, capped);
      changed += cLoose < ovr.weight - 1e-6;
      std::cout << "overlay " << ovr.weight << (ok ? "" : " MISMATCH vs dijkstra") << " (" << rep << ") | CoMaps uni "
                << cUni << (CountViolations(pUni, cm->viaWay) ? " [violates]" : "") << ", bidir " << cBi
                << (CountViolations(pBi, cm->viaWay) ? " [violates]" : "") << " | ignoring " << cLoose
                << (CountViolations(pLoose, cm->viaWay) ? " [uses the forbidden turn]" : "") << "\n";
    }
    std::cout << "  tested " << tested << ", restriction changed the best route in " << changed
              << ", CoMaps unidirectional agrees with the overlay in " << agree << "\n";
  }

  // 7. Avoid a place: block every vertex near the middle of Lucknow -> Agra, recustomize.
  if (FLAGS_avoid_test)
  {
    std::cout << "\n[7] Avoid-place test\n";
    VertexId const s = placeV[0], t = placeV[2];
    auto before = BidirectionalDijkstra(g, s, t, sc, true /* wantPath */);
    VertexId const mid = before.path[before.path.size() / 2];
    double const k = std::cos(g.lat[mid] * M_PI / 180.0);
    double const r2 = std::pow(FLAGS_avoid_radius_m / 111320.0, 2);
    std::vector<VertexId> touched;
    uint64_t blocked = 0;
    for (uint32_t v = 0; v < g.n; ++v)
    {
      double const dy = g.lat[v] - g.lat[mid], dx = (g.lon[v] - g.lon[mid]) * k;
      if (dx * dx + dy * dy > r2)
        continue;
      ++blocked;
      touched.push_back(v);
      std::vector<VertexId> preds;
      for (uint64_t e = g.revOff[v]; e < g.revOff[v + 1]; ++e)
        preds.push_back(g.revFrom[e]);
      for (VertexId u : preds)
      {
        g.SetWeight(u, v, kInf);
        touched.push_back(u);
      }
    }
    auto t1 = Clock::now();
    uint64_t const cells = Recustomize(g, part, ov, touched, sc);
    double const rs = Since(t1);
    std::cout << "  blocked " << blocked << " vertices within " << FLAGS_avoid_radius_m << " m of ("
              << g.lat[mid] << ", " << g.lon[mid] << "); recomputed " << cells << " cells in " << rs * 1000
              << " ms\n";
    uint64_t avoidMismatch = 0;
    for (auto const & pr : kPairs)
    {
      VertexId const a = placeV[pr.first], b = placeV[pr.second];
      auto base = BidirectionalDijkstra(g, a, b, sc);
      auto ovr = OverlayQuery(g, part, ov, a, b, sc);
      avoidMismatch += !Same(base.weight, ovr.weight);
      if (pr.first == 0 && pr.second == 2)
        std::cout << "  Lucknow -> Agra: " << Hms(before.weight) << " before, " << Hms(base.weight)
                  << " with the area avoided\n";
    }
    for (uint32_t q = 0; q < 50; ++q)
    {
      VertexId const a = sccList[rng() % sccList.size()], b = sccList[rng() % sccList.size()];
      avoidMismatch += !Same(BidirectionalDijkstra(g, a, b, sc).weight, OverlayQuery(g, part, ov, a, b, sc).weight);
    }
    std::cout << "  overlay vs Dijkstra after avoid: " << avoidMismatch << " mismatches over "
              << (std::size(kPairs) + 50) << " queries\n";
  }

  std::cout << "\nDone. " << Mem() << "\n";
  return mismatches == 0 ? 0 : 1;
}
