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
//   plus_overlay_tool --resources_path=data --maps_path=~/comaps-bench/maps \
//     --mwm="India_Uttar Pradesh" --mwm_version=261004 --cache=/tmp/up.graph

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

  routing::Segment ToSegment(routing::NumMwmId mwm, VertexId v) const
  {
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
      ++bySize[std::min<size_t>(r.size(), 5)];
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

// CoMaps reference: AStarAlgorithm (zero heuristic) directly on IndexGraph, with parents so
// via-way restrictions are applied the way the router applies them.
class IndexGraphAStar final : public routing::AStarGraph<routing::Segment, routing::SegmentEdge, routing::RouteWeight>
{
public:
  explicit IndexGraphAStar(routing::IndexGraph & g) : m_g(g) {}
  Weight HeuristicCostEstimate(Vertex const &, Vertex const &) override { return routing::GetAStarWeightZero<Weight>(); }
  void GetOutgoingEdgesList(routing::astar::VertexData<Vertex, Weight> const & vd, EdgeListT & edges) override
  {
    edges.clear();
    m_g.GetEdgeList(vd, true, true, edges, m_fwd ? *m_fwd : m_empty);
  }
  void GetIngoingEdgesList(routing::astar::VertexData<Vertex, Weight> const & vd, EdgeListT & edges) override
  {
    edges.clear();
    m_g.GetEdgeList(vd, false, true, edges, m_bwd ? *m_bwd : m_empty);
  }
  void SetAStarParents(bool forward, Parents & parents) override { (forward ? m_fwd : m_bwd) = &parents; }
  void DropAStarParents() override { m_fwd = m_bwd = nullptr; }
  // Same epsilon IndexGraphStarter uses for single-MWM double precision.
  Weight GetAStarWeightEpsilon() override { return routing::RouteWeight(1e-6); }

private:
  routing::IndexGraph & m_g;
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
  if (FLAGS_cache.empty() || !LoadRaw(raw, FLAGS_cache))
  {
    ensureCoMaps();
    Extract(*cm, raw);
    if (!FLAGS_cache.empty())
      SaveRaw(raw, FLAGS_cache);
  }
  else
  {
    std::cout << "  loaded from cache " << FLAGS_cache << "\n";
  }
  std::cout << "  vertices " << raw.g.n << ", edges " << raw.g.NumEdges() << "\n";

  // 2. Compression.
  std::cout << "\n[2] Chain compression\n";
  auto t0 = Clock::now();
  Compressed comp = CompressChains(raw.g, raw.valid);
  Graph & g = comp.g;
  std::cout << "  " << g.n << " vertices, " << g.NumEdges() << " edges in " << Since(t0) << " s, " << Mem() << "\n";
  {
    // Raw edges are no longer needed; keep featBase/valid for Segment decoding.
    Graph().fwdOff.swap(raw.g.fwdOff);
    std::vector<VertexId>().swap(raw.g.fwdTo);
    std::vector<double>().swap(raw.g.fwdW);
    std::vector<float>().swap(raw.g.lat);
    std::vector<float>().swap(raw.g.lon);
  }

  // Main strongly connected component, seeded near Lucknow.
  std::vector<uint8_t> all(g.n, 1);
  VertexId const seed = Nearest(g, all, kPlaces[0].lat, kPlaces[0].lon);
  uint32_t sccSize = 0;
  auto inScc = StronglyConnectedFrom(g, seed, sccSize);
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
    placeV.push_back(Nearest(g, inScc, pl.lat, pl.lon));

  uint64_t mismatches = 0;
  double maxAbsDiff = 0;
  struct Named
  {
    int a, b;
    double w;
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
    bool const ok = Same(base.weight, ovr.weight);
    mismatches += !ok;
    if (base.weight < kInf)
      maxAbsDiff = std::max(maxAbsDiff, std::abs(base.weight - ovr.weight));
    named.push_back({pr.first, pr.second, base.weight});
    std::cout << "  " << kPlaces[pr.first].name << " -> " << kPlaces[pr.second].name << ": weight "
              << Hms(base.weight) << " (" << base.weight << " s) | dijkstra " << bs * 1000 << " ms, "
              << base.settled << " settled | overlay " << os * 1000 << " ms, " << ovr.settled << " settled | "
              << (ok ? "MATCH" : "MISMATCH") << "\n";
  }

  std::vector<VertexId> sccList;
  for (uint32_t v = 0; v < g.n; ++v)
    if (inScc[v])
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
      std::cout << "  " << kPlaces[nm.a].name << " -> " << kPlaces[nm.b].name << ": CoMaps " << cw
                << " s in " << secs << " s (" << res.m_path.size() << " segments) | prototype " << nm.w << " s | "
                << (Same(cw, nm.w) ? "MATCH" : "DIFF " + std::to_string(cw - nm.w) + " s") << ", " << Mem()
                << "\n"
                << std::flush;
    }
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
