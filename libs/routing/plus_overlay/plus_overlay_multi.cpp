// Comaps++ cell-overlay prototype across several MWMs (states).
//
// Every MWM's segment graph is extracted as in plus_overlay_tool. The graphs are put into one
// combined graph (vertex ids offset per MWM) and joined the way CoMaps' world graph joins them:
// an exit transition segment of MWM A gets a zero-weight edge to its twin (the same segment in
// MWM B), from where B's own edges continue. Each MWM is partitioned on its own, so no cell spans
// a border, and border segments become ordinary boundary vertices of their cells. The query is the
// unchanged multi-level search: in an MWM that holds neither end it climbs to that MWM's top level.
//
// Modes:
//   --mode=overlay  build everything, answer --routes (and random cross-border pairs) with the
//                   overlay, write answers; with --router_in, compare with the router's routes.
//   --mode=router   only CoMaps: IndexRouter with COMAPS_PLUS_FORCE_JOINTS=1 (one exact search over
//                   all loaded maps) for the same routes; writes the segment paths to --router_out.
// The router runs in its own process to stay inside a small machine's memory.

#define PLUS_OVERLAY_TOOL_NO_MAIN
#include "plus_overlay_tool.cpp"

#include "routing/cross_mwm_index_graph.hpp"
#include "routing/road_access.hpp"
#include "routing/index_graph_loader.hpp"
#include "indexer/feature_data.hpp"
#include "indexer/feature.hpp"
#include "indexer/ftypes_matcher.hpp"
#include "routing/plus_overlay/overlay_router.hpp"

#include <cstdlib>
#include <optional>
#include <set>
#include <sstream>

#include "base/scope_guard.hpp"
#include "base/string_utils.hpp"

DEFINE_string(mwms, "India_Uttar Pradesh;India_Madhya Pradesh", "';'-separated maps to load.");
DEFINE_string(cache_dir, "", "Directory for per-map raw graph caches (<map>.graph).");
DEFINE_string(mode, "overlay", "overlay | router");
DEFINE_string(routes, "", "Routes file: 'name lat lon lat lon' per line. Empty: built-in list.");
DEFINE_uint32(random_pairs, 10, "Random pairs with the two ends in different maps (overlay mode).");
DEFINE_uint32(router_pairs, 4, "How many of the random pairs also go into the routes file for the router.");
DEFINE_string(routes_out, "", "overlay mode: write the routes used (named + router_random) here.");
DEFINE_string(router_out, "", "router mode: output file.");
DEFINE_string(router_in, "", "overlay mode: router results to compare with.");
DEFINE_string(overlay_dir, "", "runtime mode: directory with <map>.plusovl files.");
DEFINE_string(vehicle, "car", "router mode: car | pedestrian | bicycle.");
DEFINE_string(reroute_offsets, "300,1500,4000", "reroute mode: metres off the route (comma-separated).");
DEFINE_bool(wrong_turn, false, "reroute mode: leave the route by taking a wrong turn at a junction and driving "
                               "on for the --reroute_offsets distances, instead of jumping sideways.");
DEFINE_bool(avoid_toll, false, "router mode: set the 'avoid toll roads' option for the run (restored after).");
DEFINE_int32(avoid_mask, -1, "router mode: avoid options for the run as RoutingOptions bits (2 toll, 4 motorway, "
                             "8 ferry, 16 dirt); -1: leave the settings alone.");
DEFINE_double(start_dir_deg, -1, "router mode: start direction (compass degrees, 0 = north); < 0: none.");
DEFINE_bool(force_joints, true, "router mode: COMAPS_PLUS_FORCE_JOINTS=1 for the normal search.");

namespace
{
std::vector<std::string> SplitList(std::string const & s, char sep)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, sep))
    if (!item.empty())
      out.push_back(item);
  return out;
}

struct RouteSpec
{
  std::string name;
  ms::LatLon a, b;
  std::vector<ms::LatLon> via;  // intermediate stops (router mode)
};

std::vector<RouteSpec> BuiltinRoutes()
{
  struct P
  {
    char const * name;
    double lat, lon;
  };
  static P const kP[] = {
      {"Lucknow", 26.8467, 80.9462}, {"Kanpur", 26.4499, 80.3319}, {"Agra", 27.1767, 78.0081},
      {"Varanasi", 25.3176, 82.9739}, {"Jhansi", 25.4484, 78.5685}, {"Bhopal", 23.2599, 77.4126},
      {"Gwalior", 26.2183, 78.1828}, {"Jabalpur", 23.1815, 79.9864}, {"Indore", 22.7196, 75.8577},
      {"Sagar", 23.8388, 78.7378}, {"Rewa", 24.5362, 81.3037}, {"Pune", 18.5204, 73.8567},
      {"Nagpur", 21.1458, 79.0882}, {"Mumbai", 19.0760, 72.8777}};
  auto at = [](char const * n)
  {
    for (auto const & p : kP)
      if (std::string(p.name) == n)
        return ms::LatLon(p.lat, p.lon);
    CHECK(false, (n));
    return ms::LatLon();
  };
  std::vector<std::pair<char const *, char const *>> pairs = {
      {"Agra", "Gwalior"},   {"Jhansi", "Sagar"},   {"Varanasi", "Rewa"},  {"Kanpur", "Jabalpur"},
      {"Lucknow", "Bhopal"}, {"Lucknow", "Indore"}, {"Lucknow", "Nagpur"}, {"Lucknow", "Pune"},
      {"Agra", "Nagpur"},    {"Bhopal", "Pune"}};
  std::vector<RouteSpec> out;
  for (auto const & [x, y] : pairs)
    out.push_back({std::string(x) + "-" + y, at(x), at(y)});
  return out;
}

// All maps in one data source, with the loader/world graph IndexRouter would build.
struct Multi
{
  FrozenDataSource dataSource;
  std::shared_ptr<routing::NumMwmIds> numMwmIds = std::make_shared<routing::NumMwmIds>();
  std::unique_ptr<routing::MwmDataSource> mwmDataSource;
  std::shared_ptr<storage::CountryParentGetter> cpg;
  std::shared_ptr<routing::VehicleModelFactoryInterface> factory;
  std::shared_ptr<routing::EdgeEstimator> estimator;
  std::unique_ptr<routing::SingleVehicleWorldGraph> world;
  std::vector<std::string> names;
  std::vector<routing::NumMwmId> ids;

  void Init(std::vector<std::string> const & mwms)
  {
    auto & pl = GetPlatform();
    pl.SetResourceDir(FLAGS_resources_path + "/");
    pl.SetWritableDirForTests(FLAGS_maps_path + "/");
    classificator::Load();
    // "Name" or "Name@version" (maps of different versions side by side, as after updating one state).
    for (auto const & spec : mwms)
    {
      auto const at = spec.find('@');
      std::string const name = spec.substr(0, at);
      int64_t const version = at == std::string::npos ? FLAGS_mwm_version : std::stoll(spec.substr(at + 1));
      names.push_back(name);
      platform::CountryFile const countryFile(name);
      platform::LocalCountryFile localFile(FLAGS_maps_path + "/" + std::to_string(version), countryFile, version);
      localFile.SyncWithDisk();
      auto const res = dataSource.RegisterMap(localFile);
      CHECK_EQUAL(res.second, MwmSet::RegResult::Success, ("Can't register", localFile));
      numMwmIds->RegisterFile(countryFile);
      ids.push_back(numMwmIds->GetId(countryFile));
    }
    mwmDataSource = std::make_unique<routing::MwmDataSource>(dataSource, numMwmIds);
    cpg = std::make_shared<storage::CountryParentGetter>();
    auto parentGetter = [cpg = cpg](std::string const & id) { return (*cpg)(id); };
    factory = std::make_shared<routing::CarModelFactory>(parentGetter);
    double maxSpeed = 0;
    for (auto const & name : names)
      maxSpeed = std::max(maxSpeed, factory->GetVehicleModelForCountry(name)->GetMaxWeightSpeed());
    estimator = routing::EdgeEstimator::Create(routing::VehicleType::Car, maxSpeed,
                                               factory->GetVehicleModel()->GetOffroadSpeed(), nullptr /* traffic */,
                                               &dataSource, numMwmIds);
    auto loader = routing::IndexGraphLoader::Create(routing::VehicleType::Car, false /* loadAltitudes */, factory,
                                                    estimator, *mwmDataSource, routing::RoutingOptions(),
                                                    [t = std::time(nullptr)]() { return t; });
    world = std::make_unique<routing::SingleVehicleWorldGraph>(nullptr /* crossMwmGraph */, std::move(loader),
                                                               estimator, routing::MwmHierarchyHandler());
    world->SetMode(routing::WorldGraphMode::SingleMwm);
  }

  std::vector<std::vector<uint32_t>> ViaWay(routing::NumMwmId id)
  {
    routing::RestrictionLoader rl(mwmDataSource->GetMwmValue(id), world->GetIndexGraph(id));
    std::vector<std::vector<uint32_t>> out;
    for (auto const & r : rl.StealRestrictions())
      if (r.size() >= 3)
        out.push_back(r);
    return out;
  }

  std::unique_ptr<routing::IndexRouter> MakeRouter(traffic::TrafficCache & traffic,
                                                   std::shared_ptr<storage::CountryInfoGetter> & cig)
  {
    cig = storage::CountryInfoReader::CreateCountryInfoGetter(GetPlatform());
    auto const countryFileGetter = [cig](m2::PointD const & pt) { return cig->GetRegionCountryId(pt); };
    auto const rectGetter = [cig](std::string const & id) { return cig->GetLimitRectForLeaf(id); };
    auto const vt = FLAGS_vehicle == "pedestrian" ? routing::VehicleType::Pedestrian
                    : FLAGS_vehicle == "bicycle"  ? routing::VehicleType::Bicycle
                                                  : routing::VehicleType::Car;
    // As the app's RoutingManager: altitudes for everything but cars.
    return std::make_unique<routing::IndexRouter>(vt, vt != routing::VehicleType::Car /* loadAltitudes */, *cpg,
                                                  countryFileGetter, rectGetter, numMwmIds,
                                                  routing::MakeNumMwmTree(*numMwmIds, *cig), traffic, dataSource);
  }
};

// One MWM's place in the combined graph.
struct Part
{
  routing::NumMwmId id = 0;
  std::string name;
  VertexId base = 0;  // first global vertex
  uint32_t n = 0;     // vertices incl. via-way copies
  uint32_t baseN = 0;
  std::vector<uint32_t> featBase;
  std::vector<VertexId> copyOrig;  // local ids
  std::vector<std::vector<uint32_t>> viaWay;
};

struct Combined
{
  std::vector<Part> parts;
  Graph g;
  std::vector<uint8_t> valid;
  uint64_t twinLinks = 0;
  // Features that are the same OSM way in two maps (from the twin links), both directions.
  std::map<std::pair<routing::NumMwmId, uint32_t>, std::set<std::pair<routing::NumMwmId, uint32_t>>> twinFeature;

  // The same (feature index, segment, direction) on the twin feature in |mwm|, if that segment exists.
  std::optional<routing::Segment> OnTwin(routing::Segment const & s, routing::NumMwmId mwm) const
  {
    auto it = twinFeature.find({s.GetMwmId(), s.GetFeatureId()});
    if (it == twinFeature.end())
      return {};
    for (auto const & [m, f] : it->second)
    {
      if (m != mwm)
        continue;
      auto const & q = parts[PartOf(m)];
      if (f + 1 < q.featBase.size() && s.GetSegmentIdx() < q.featBase[f + 1] - q.featBase[f])
        return routing::Segment(m, f, s.GetSegmentIdx(), s.IsForward());
    }
    return {};
  }

  size_t PartOf(VertexId v) const
  {
    size_t lo = 0, hi = parts.size();
    while (hi - lo > 1)
    {
      size_t const mid = (lo + hi) / 2;
      (parts[mid].base <= v ? lo : hi) = mid;
    }
    return lo;
  }
  size_t PartOf(routing::NumMwmId id) const
  {
    for (size_t i = 0; i < parts.size(); ++i)
      if (parts[i].id == id)
        return i;
    CHECK(false, (id));
    return 0;
  }
  VertexId Orig(VertexId v) const
  {
    auto const & p = parts[PartOf(v)];
    VertexId const l = v - p.base;
    return l < p.baseN ? v : p.base + p.copyOrig[l - p.baseN];
  }
  bool IsCopy(VertexId v) const
  {
    auto const & p = parts[PartOf(v)];
    return v - p.base >= p.baseN;
  }
  routing::Segment ToSegment(VertexId v) const
  {
    auto const & p = parts[PartOf(v)];
    VertexId l = v - p.base;
    if (l >= p.baseN)
      l = p.copyOrig[l - p.baseN];
    uint32_t const segIdx = l / 2;
    auto it = std::upper_bound(p.featBase.begin(), p.featBase.end(), segIdx);
    uint32_t const f = static_cast<uint32_t>(it - p.featBase.begin()) - 1;
    return routing::Segment(p.id, f, segIdx - p.featBase[f], (l % 2) == 0);
  }
  VertexId FromSegment(routing::Segment const & s) const
  {
    auto const & p = parts[PartOf(s.GetMwmId())];
    return p.base + 2 * (p.featBase[s.GetFeatureId()] + s.GetSegmentIdx()) + (s.IsForward() ? 0 : 1);
  }
};

void BuildCombined(Multi & m, Combined & c)
{
  auto const t0 = Clock::now();
  std::vector<Raw> raws(m.ids.size());
  for (size_t i = 0; i < m.ids.size(); ++i)
  {
    Part p;
    p.id = m.ids[i];
    p.name = m.names[i];
    std::string const cache = FLAGS_cache_dir.empty() ? "" : FLAGS_cache_dir + "/" + p.name + ".graph";
    Raw & raw = raws[i];
    if (cache.empty() || !LoadRaw(raw, cache))
    {
      std::cout << "  " << p.name << ": extracting\n";
      Extract(m.world->GetIndexGraph(p.id), p.id, raw);
      if (!cache.empty())
        SaveRaw(raw, cache);
    }
    p.viaWay = m.ViaWay(p.id);
    ExpandViaWay(raw, p.viaWay);
    p.n = raw.g.n;
    p.baseN = raw.baseN;
    p.featBase = raw.featBase;
    p.copyOrig = raw.copyOrig;
    p.base = c.parts.empty() ? 0 : c.parts.back().base + c.parts.back().n;
    std::cout << "  " << p.name << ": " << p.n << " vertices, " << raw.g.NumEdges() << " edges, via-way patterns "
              << p.viaWay.size() << ", " << Mem() << "\n";
    c.parts.push_back(std::move(p));
  }
  uint64_t const total = c.parts.back().base + c.parts.back().n;
  CHECK_LESS(total, uint64_t(kNone), ());

  // Twin links: every exit transition segment -> the same segment in the neighbouring loaded map.
  routing::CrossMwmIndexGraph<base::GeoObjectId> cross(*m.mwmDataSource, routing::VehicleType::Car);
  for (auto id : m.ids)
    cross.LoadCrossMwmConnectorWithTransitions(id);
  std::vector<std::vector<std::pair<VertexId, VertexId>>> links(c.parts.size());  // per part: (local u, global v)
  for (size_t i = 0; i < c.parts.size(); ++i)
  {
    auto const & p = c.parts[i];
    std::vector<routing::NumMwmId> neighbors;
    for (auto id : m.ids)
      if (id != p.id)
        neighbors.push_back(id);
    uint64_t exits = 0, linked = 0;
    cross.GetCrossMwmConnectorWithTransitions(p.id).ForEachExit([&](uint32_t, routing::Segment const & s)
    {
      ++exits;
      std::vector<routing::Segment> twins;
      cross.GetTwinsByCrossMwmId(s, true /* isOutgoing */, neighbors, twins);
      VertexId const u = c.FromSegment(s) - p.base;
      if (u >= p.baseN || !raws[i].valid[u])
        return;
      for (auto const & t : twins)
      {
        auto const & q = c.parts[c.PartOf(t.GetMwmId())];
        VertexId const lv = 2 * (q.featBase[t.GetFeatureId()] + t.GetSegmentIdx()) + (t.IsForward() ? 0 : 1);
        if (lv >= q.baseN || !raws[c.PartOf(t.GetMwmId())].valid[lv])
          continue;
        links[i].emplace_back(u, q.base + lv);
        c.twinFeature[{s.GetMwmId(), s.GetFeatureId()}].insert({t.GetMwmId(), t.GetFeatureId()});
        c.twinFeature[{t.GetMwmId(), t.GetFeatureId()}].insert({s.GetMwmId(), s.GetFeatureId()});
        ++linked;
      }
    });
    std::sort(links[i].begin(), links[i].end());
    c.twinLinks += linked;
    std::cout << "  " << p.name << ": exits " << exits << ", linked to a loaded map " << linked << "\n";
  }

  // Concatenate array by array, freeing each part's copy as soon as it is used.
  Graph & g = c.g;
  g.n = static_cast<uint32_t>(total);
  uint64_t edges = c.twinLinks;
  for (auto const & r : raws)
    edges += r.g.NumEdges();
  g.fwdOff.assign(g.n + 1, 0);
  for (size_t i = 0; i < raws.size(); ++i)
  {
    auto const & p = c.parts[i];
    auto const & off = raws[i].g.fwdOff;
    size_t k = 0;
    for (VertexId l = 0; l < p.n; ++l)
    {
      uint64_t extra = 0;
      while (k < links[i].size() && links[i][k].first == l)
        ++extra, ++k;
      g.fwdOff[p.base + l + 1] = off[l + 1] - off[l] + extra;
    }
  }
  for (VertexId v = 0; v < g.n; ++v)
    g.fwdOff[v + 1] += g.fwdOff[v];
  CHECK_EQUAL(g.fwdOff[g.n], edges, ());
  g.fwdTo.resize(edges);
  for (size_t i = 0; i < raws.size(); ++i)
  {
    auto const & p = c.parts[i];
    auto & r = raws[i].g;
    size_t k = 0;
    for (VertexId l = 0; l < p.n; ++l)
    {
      uint64_t o = g.fwdOff[p.base + l];
      for (uint64_t e = r.fwdOff[l]; e < r.fwdOff[l + 1]; ++e)
        g.fwdTo[o++] = p.base + r.fwdTo[e];
      while (k < links[i].size() && links[i][k].first == l)
        g.fwdTo[o++] = links[i][k++].second;
    }
    std::vector<VertexId>().swap(r.fwdTo);
  }
  g.fwdW.resize(edges);
  for (size_t i = 0; i < raws.size(); ++i)
  {
    auto const & p = c.parts[i];
    auto & r = raws[i].g;
    size_t k = 0;
    for (VertexId l = 0; l < p.n; ++l)
    {
      uint64_t o = g.fwdOff[p.base + l];
      for (uint64_t e = r.fwdOff[l]; e < r.fwdOff[l + 1]; ++e)
        g.fwdW[o++] = r.fwdW[e];
      while (k < links[i].size() && links[i][k].first == l)
        g.fwdW[o++] = 0.0, ++k;
    }
    std::vector<double>().swap(r.fwdW);
    std::vector<uint64_t>().swap(r.fwdOff);
  }
  g.lat.resize(g.n);
  g.lon.resize(g.n);
  c.valid.resize(g.n);
  for (size_t i = 0; i < raws.size(); ++i)
  {
    auto const & p = c.parts[i];
    std::copy(raws[i].g.lat.begin(), raws[i].g.lat.end(), g.lat.begin() + p.base);
    std::copy(raws[i].g.lon.begin(), raws[i].g.lon.end(), g.lon.begin() + p.base);
    std::copy(raws[i].valid.begin(), raws[i].valid.end(), c.valid.begin() + p.base);
    raws[i] = Raw();
  }
  std::cout << "  combined: " << g.n << " vertices, " << g.NumEdges() << " edges (" << c.twinLinks
            << " twin links) in " << Since(t0) << " s, " << Mem() << "\n";
}

std::string SegStr(routing::Segment const & s)
{
  return std::to_string(s.GetMwmId()) + ":" + std::to_string(s.GetFeatureId()) + ":" +
         std::to_string(s.GetSegmentIdx()) + ":" + (s.IsForward() ? "1" : "0");
}

routing::Segment ParseSeg(std::string const & t)
{
  auto const f = SplitList(t, ':');
  CHECK_EQUAL(f.size(), 4u, (t));
  return routing::Segment(static_cast<routing::NumMwmId>(std::stoul(f[0])), std::stoul(f[1]), std::stoul(f[2]),
                          f[3] == "1");
}

std::vector<RouteSpec> ReadRoutes(std::string const & path)
{
  // name lat lon [lat lon ...] lat lon: start, optional stops, finish.
  std::vector<RouteSpec> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line))
  {
    std::istringstream ls(line);
    std::string name;
    if (!(ls >> name))
      continue;
    std::vector<ms::LatLon> pts;
    double lat, lon;
    while (ls >> lat >> lon)
      pts.emplace_back(lat, lon);
    if (pts.size() < 2)
      continue;
    RouteSpec r{name, pts.front(), pts.back(), {}};
    r.via.assign(pts.begin() + 1, pts.end() - 1);
    out.push_back(std::move(r));
  }
  return out;
}

// --- Router process ------------------------------------------------------------------------------
// --- Runtime: only the per-map overlay files + CoMaps' IndexGraph, as in the app ---------------------
int RunRuntime(Multi & m)
{
  auto const routes = FLAGS_routes.empty() ? BuiltinRoutes() : ReadRoutes(FLAGS_routes);
  traffic::TrafficCache traffic;
  std::shared_ptr<storage::CountryInfoGetter> cig;
  auto router = m.MakeRouter(traffic, cig);
  PartitionParams pp;
  pp.cellSizes = ParseSizes(FLAGS_cell_sizes);
  pp.minBalance = FLAGS_min_balance;
  OverlayRouter ovr(BuildParamsHash(pp.cellSizes, pp.minBalance, pp.directions));
  std::vector<OverlayMap> maps;
  for (size_t i = 0; i < m.ids.size(); ++i)
    maps.push_back({m.ids[i], FLAGS_overlay_dir + "/" + m.names[i] + kFileExtension, FLAGS_mwm_version});
  ovr.SetMaps(maps);
  std::vector<routing::NumMwmId> missing;
  std::cout << "  all maps ready: " << (ovr.AllMapsReady(&missing) ? "yes" : "NO") << ", " << Mem() << "\n";
  auto & world = *m.world;
  using Parents = routing::AStarGraph<routing::Segment, routing::SegmentEdge, routing::RouteWeight>::Parents;

  uint32_t ok = 0, consistent = 0;
  std::vector<double> ms;
  std::ofstream out(FLAGS_router_out.empty() ? "/dev/null" : FLAGS_router_out);
  out.precision(9);
  for (auto const & r : routes)
  {
    routing::FakeEnding se, fe;
    bool codir = false;
    char head[96];
    std::snprintf(head, sizeof(head), "  %-18s %6.0f km: ", r.name.c_str(), ms::DistanceOnEarth(r.a, r.b) / 1000);
    std::cout << head;
    if (router->PlusSnap(mercator::FromLatLon(r.a), mercator::FromLatLon(r.b), se, fe, codir) != 0)
    {
      std::cout << "snap failed\n";
      continue;
    }
    routing::IndexGraphStarter starter(se, fe, 0 /* fakeNumerationStart */, codir /* strictForward */, world);
    std::vector<routing::Segment> path;
    OverlayRouter::Stats st;
    auto const res = ovr.FindPath(starter, path, st);
    if (res != OverlayRouter::Result::Ok)
    {
      std::cout << (res == OverlayRouter::Result::NoRoute ? "no route" : "not applicable: " + st.reason) << " ("
                << st.ms << " ms)\n";
      continue;
    }
    ++ok;
    ms.push_back(st.ms);
    // Re-score the path step by step with CoMaps' edges (starter for fake parts, IndexGraph for real
    // ones, twin + neighbour edge at a map change).
    Parents pf;
    starter.SetAStarParents(true, pf);
    double w = 0, wSrc = 0, wTgt = 0;
    size_t bad = 0, firstBad = 0;
    routing::IndexGraphStarter::EdgeListT es;
    routing::IndexGraph::SegmentEdgeListT ies;
    for (size_t k = 1; k < path.size(); ++k)
    {
      auto const & a = path[k - 1];
      auto const & b = path[k];
      double best = kInf;
      if (routing::IndexGraphStarter::IsFakeSegment(a) || routing::IndexGraphStarter::IsFakeSegment(b))
      {
        es.clear();
        routing::astar::VertexData<routing::Segment, routing::RouteWeight> const vd(a, routing::RouteWeight(0));
        starter.GetOutgoingEdgesList(vd, es);
        for (auto const & e : es)
          if (e.GetTarget() == b)
            best = std::min(best, e.GetWeight().GetIntegratedWeight());
      }
      else
      {
        routing::Segment from = a;
        if (a.GetMwmId() != b.GetMwmId())
        {
          routing::Segment twin;
          if (ovr.TwinOfExit(a, twin))
            from = twin;
        }
        ies.clear();
        world.GetIndexGraph(from.GetMwmId()).GetEdgeList(from, true, true, ies);
        for (auto const & e : ies)
          if (e.GetTarget() == b)
            best = std::min(best, e.GetWeight().GetIntegratedWeight());
      }
      if (best >= kInf)
      {
        if (bad++ == 0)
          firstBad = k;
        continue;
      }
      w += best;
      if (k <= st.srcEnd)
        wSrc += best;
      if (st.tgtBegin > 0 && k > st.tgtBegin)
        wTgt += best;
    }
    starter.DropAStarParents();
    size_t viol = 0;
    for (size_t i = 0; i < m.ids.size(); ++i)
    {
      std::vector<routing::Segment> sub;
      for (auto const & sg : path)
        if (!routing::IndexGraphStarter::IsFakeSegment(sg) && sg.GetMwmId() == m.ids[i])
          sub.push_back(sg);
      viol += CountViolations(sub, m.ViaWay(m.ids[i]));
    }
    bool const same = bad == 0 && std::abs(w - st.weight) <= 1e-3 * std::max(1.0, st.weight);
    consistent += same;
    std::cout << "overlay " << Hms(st.weight) << " (" << st.weight << " s) in " << st.ms << " ms, " << st.settled
              << " settled, " << st.baseScans << " edge lists, " << path.size() << " segments, " << st.mapsUsed
              << " maps | path re-scored " << w << " s"
              << (bad ? " [" + std::to_string(bad) + " BROKEN steps, first at " + std::to_string(firstBad) + "]"
                      : (same ? " OK" : " MISMATCH"))
              << (viol ? " [" + std::to_string(viol) + " via-way VIOLATIONS]" : "")
              << (std::getenv("PLUS_OVERLAY_DEBUG") ? " | start part " + std::to_string(st.srcCost) + " vs " +
                                                          std::to_string(wSrc) + ", finish part " +
                                                          std::to_string(st.tgtCost) + " vs " + std::to_string(wTgt)
                                                    : std::string())
              << "\n";
    out << r.name << " " << st.weight << " " << w << " " << st.ms << "\n";
  }
  std::sort(ms.begin(), ms.end());
  std::cout << "  runtime: " << ok << "/" << routes.size() << " routed, " << consistent
            << " re-scored consistently; median " << (ms.empty() ? 0 : ms[ms.size() / 2]) << " ms, max "
            << (ms.empty() ? 0 : ms.back()) << " ms; " << Mem() << "\n";
  return 0;
}

int RunRouter(Multi & m)
{
  if (FLAGS_force_joints)
    setenv("COMAPS_PLUS_FORCE_JOINTS", "1", 1);
  auto const savedOptions = routing::RoutingOptions::LoadOptionsFromSettings(routing::VehicleType::Car);
  if (FLAGS_avoid_toll || FLAGS_avoid_mask >= 0)
  {
    routing::RoutingOptions o(FLAGS_avoid_mask >= 0 ? static_cast<routing::RoutingOptions::OptionType>(FLAGS_avoid_mask)
                                                    : routing::RoutingOptions::OptionType(0),
                              routing::VehicleType::Car);
    if (FLAGS_avoid_toll)
      o.Add(routing::RoutingOptions::Toll);
    routing::RoutingOptions::SaveOptionsToSettings(o);
  }
  auto const routes = FLAGS_routes.empty() ? BuiltinRoutes() : ReadRoutes(FLAGS_routes);
  traffic::TrafficCache traffic;
  std::shared_ptr<storage::CountryInfoGetter> cig;
  auto router = m.MakeRouter(traffic, cig);
  routing::RouterDelegate delegate;
  std::ofstream out(FLAGS_router_out);
  out.precision(9);
  for (auto const & r : routes)
  {
    routing::Route route("" /* router */, 0 /* routeId */);
    delegate.SetTimeout(3600);
    auto const t0 = Clock::now();
    m2::PointD dir = m2::PointD::Zero();
    if (FLAGS_start_dir_deg >= 0)
    {
      double const a = FLAGS_start_dir_deg * M_PI / 180.0;
      dir = m2::PointD(std::sin(a), std::cos(a));  // mercator x east, y north
    }
    std::vector<m2::PointD> pts{mercator::FromLatLon(r.a)};
    for (auto const & v : r.via)
      pts.push_back(mercator::FromLatLon(v));
    pts.push_back(mercator::FromLatLon(r.b));
    auto const code = router->CalculateRoute(
        routing::Checkpoints(std::move(pts)), dir,
        false /* adjustToPrevRoute */, delegate, route);
    if (code == routing::RouterResultCode::NoError && FLAGS_start_dir_deg >= 0)
    {
      // Bearing of the first 30 m of the route.
      auto const & poly = route.GetPoly();
      m2::PointD const p0 = poly.Front();
      m2::PointD p1 = p0;
      for (size_t i = 1; i < poly.GetSize(); ++i)
      {
        p1 = poly.GetPoint(i);
        if (mercator::DistanceOnEarth(p0, p1) > 30)
          break;
      }
      double const b = std::fmod(std::atan2(p1.x - p0.x, p1.y - p0.y) * 180.0 / M_PI + 360.0, 360.0);
      std::cout << "  " << r.name << ": requested start direction " << FLAGS_start_dir_deg << " deg, route starts at "
                << b << " deg\n";
    }
    double const secs = Since(t0);
    std::vector<routing::Segment> segs;
    if (code == routing::RouterResultCode::NoError)
      for (auto const & rs : route.GetRouteSegments())
      {
        auto const & sg = rs.GetSegment();
        if (sg.IsRealSegment() && (segs.empty() || !(segs.back() == sg)))
          segs.push_back(sg);
      }
    std::cout << "  " << r.name << ": " << routing::DebugPrint(code) << " in " << secs << " s, ETA "
              << (code == routing::RouterResultCode::NoError ? Hms(route.GetTotalTimeSec()) : "-") << ", "
              << segs.size() << " segments, " << Mem() << "\n";
    out << r.name << " " << (code == routing::RouterResultCode::NoError ? 1 : 0) << " " << secs << " "
        << (code == routing::RouterResultCode::NoError ? route.GetTotalTimeSec() : 0.0) << " " << segs.size();
    for (auto const & s : segs)
      out << " " << SegStr(s);
    out << "\n";
  }
  routing::RoutingOptions::SaveOptionsToSettings(savedOptions);
  return 0;
}

// Rerouting: the driver leaves the route at several places (kept going straight, ended up off the
// route by some distance) and the app asks for a new route with adjustToPrevRoute, as RoutingManager
// does on RouteNeedsRebuild. Compared with a fresh route from the same place (the best route).
int RunReroute(Multi & m)
{
  auto const savedOptions = routing::RoutingOptions::LoadOptionsFromSettings(routing::VehicleType::Car);
  if (FLAGS_avoid_mask >= 0)
    routing::RoutingOptions::SaveOptionsToSettings(routing::RoutingOptions(
        static_cast<routing::RoutingOptions::OptionType>(FLAGS_avoid_mask), routing::VehicleType::Car));
  SCOPE_GUARD(restoreOptions, [&] { routing::RoutingOptions::SaveOptionsToSettings(savedOptions); });
  auto const routes = FLAGS_routes.empty() ? BuiltinRoutes() : ReadRoutes(FLAGS_routes);
  traffic::TrafficCache traffic;
  std::shared_ptr<storage::CountryInfoGetter> cig, cig2;
  auto router = m.MakeRouter(traffic, cig);
  auto fresh = m.MakeRouter(traffic, cig2);
  routing::RouterDelegate delegate;
  delegate.SetTimeout(3600);
  std::vector<double> fractions{0.1, 0.3, 0.5, 0.7, 0.9};
  std::vector<double> offsets;
  for (auto const & t : strings::Tokenize(FLAGS_reroute_offsets, ","))
    offsets.push_back(std::stod(std::string(t)));
  std::cout.precision(2);
  std::cout.setf(std::ios::fixed);
  for (auto const & r : routes)
  {
    m2::PointD const a = mercator::FromLatLon(r.a), b = mercator::FromLatLon(r.b);
    routing::Route base("", 0);
    auto code = router->CalculateRoute(routing::Checkpoints(a, b), m2::PointD::Zero(), false, delegate, base);
    if (code != routing::RouterResultCode::NoError)
    {
      std::cout << r.name << ": first route failed " << routing::DebugPrint(code) << "\n";
      continue;
    }
    // Cumulative lengths along the polyline.
    auto const & poly = base.GetPoly();
    std::vector<double> cum{0};
    for (size_t i = 1; i < poly.GetSize(); ++i)
      cum.push_back(cum.back() + mercator::DistanceOnEarth(poly.GetPoint(i - 1), poly.GetPoint(i)));
    std::cout << r.name << " (" << cum.back() / 1000 << " km, ETA " << Hms(base.GetTotalTimeSec()) << ")\n";
    // Real segments of the route, in order (for wrong turns).
    std::vector<routing::Segment> rsegs;
    for (auto const & rs : base.GetRouteSegments())
      if (rs.GetSegment().IsRealSegment() && (rsegs.empty() || !(rsegs.back() == rs.GetSegment())))
        rsegs.push_back(rs.GetSegment());
    std::set<routing::Segment> onRoute(rsegs.begin(), rsegs.end());
    auto pointOf = [&](routing::Segment const & sg, bool front)
    {
      auto const ll = m.world->GetIndexGraph(sg.GetMwmId()).GetPoint(sg, front);
      return mercator::FromLatLon(ll);
    };
    for (double f : fractions)
    {
      size_t i = std::lower_bound(cum.begin(), cum.end(), f * cum.back()) - cum.begin();
      i = std::min(std::max<size_t>(i, 1), poly.GetSize() - 1);
      m2::PointD const p = poly.GetPoint(i);
      m2::PointD dir = poly.GetPoint(i) - poly.GetPoint(i - 1);
      if (dir.Length() > 0)
        dir = dir.Normalize();
      for (double off : offsets)
      {
        // Off to the right of the direction of travel, still heading the same way.
        m2::PointD const right(dir.y, -dir.x);
        m2::PointD q = mercator::GetSmPoint(p, right.x * off, right.y * off);
        std::string how;
        if (FLAGS_wrong_turn)
        {
          // From the route segment nearest to this fraction, find the next junction with a road that
          // leaves the route, turn into it and keep going (straightest continuation) for |off| metres.
          size_t k = static_cast<size_t>(f * rsegs.size());
          routing::IndexGraph::SegmentEdgeListT edges;
          bool turned = false;
          routing::Segment cur;
          for (; k + 1 < rsegs.size() && !turned; ++k)
          {
            edges.clear();
            m.world->GetIndexGraph(rsegs[k].GetMwmId()).GetEdgeList(rsegs[k], true, true, edges);
            for (auto const & e : edges)
            {
              auto const & t = e.GetTarget();
              if (t == rsegs[k + 1] || onRoute.count(t) || (t.GetFeatureId() == rsegs[k].GetFeatureId() &&
                                                            t.GetSegmentIdx() == rsegs[k].GetSegmentIdx()))
                continue;
              cur = t;
              turned = true;
              break;
            }
          }
          if (!turned)
          {
            std::cout << "  at " << int(f * 100) << "%: no junction to turn wrong at\n";
            continue;
          }
          double driven = mercator::DistanceOnEarth(pointOf(cur, false), pointOf(cur, true));
          for (int guard = 0; guard < 100000 && driven < off; ++guard)
          {
            edges.clear();
            m.world->GetIndexGraph(cur.GetMwmId()).GetEdgeList(cur, true, true, edges);
            m2::PointD const d0 = (pointOf(cur, true) - pointOf(cur, false)).Normalize();
            double bestCos = -2;
            routing::Segment next;
            for (auto const & e : edges)
            {
              auto const & t = e.GetTarget();
              if (t.GetFeatureId() == cur.GetFeatureId() && t.GetSegmentIdx() == cur.GetSegmentIdx())
                continue;  // U-turn
              m2::PointD const d1 = (pointOf(t, true) - pointOf(t, false)).Normalize();
              double const c = m2::DotProduct(d0, d1);
              if (c > bestCos)
                bestCos = c, next = t;
            }
            if (bestCos < -1.5)
              break;  // dead end
            cur = next;
            driven += mercator::DistanceOnEarth(pointOf(cur, false), pointOf(cur, true));
          }
          q = pointOf(cur, true);
          dir = (pointOf(cur, true) - pointOf(cur, false)).Normalize();
          char buf[64];
          std::snprintf(buf, sizeof(buf), "wrong turn, %.1f km driven", driven / 1000);
          how = buf;
        }
        // The app reroutes from the route it is following: restore it first.
        routing::Route again("", 0);
        router->CalculateRoute(routing::Checkpoints(a, b), m2::PointD::Zero(), false, delegate, again);
        routing::Route adj("", 0), best("", 0);
        auto t0 = Clock::now();
        auto const c1 = router->CalculateRoute(routing::Checkpoints(q, b), dir, true /* adjustToPrevRoute */,
                                               delegate, adj);
        double const adjSec = Since(t0);
        bool const adjusted = router->PlusLastWasAdjusted();
        t0 = Clock::now();
        auto const c2 = fresh->CalculateRoute(routing::Checkpoints(q, b), dir, false, delegate, best);
        double const bestSec = Since(t0);
        std::cout << "  at " << int(f * 100) << "%, " << (how.empty() ? std::to_string(int(off)) + " m off" : how)
                  << ": ";
        if (c1 != routing::RouterResultCode::NoError || c2 != routing::RouterResultCode::NoError)
        {
          std::cout << "reroute " << routing::DebugPrint(c1) << ", fresh " << routing::DebugPrint(c2) << "\n";
          continue;
        }
        double const diff = adj.GetTotalTimeSec() - best.GetTotalTimeSec();
        std::cout << (adjusted ? "rejoined old route" : "new route       ") << " in " << adjSec << " s, ETA "
                  << Hms(adj.GetTotalTimeSec()) << " | best route " << bestSec << " s, ETA "
                  << Hms(best.GetTotalTimeSec()) << " | reroute worse by " << diff / 60 << " min\n";
      }
    }
  }
  return 0;
}

// Stress routes: real places of each kind in the loaded maps (roundabouts, ramps, bridges/tunnels, one-way
// and restricted roads, dead ends, small islands, border crossings), each paired with a random road point
// 1-30 km away; plus random short and long pairs and the farthest pairs. Written as a routes file.
int RunGenStress(Multi & m)
{
  std::mt19937_64 rng(FLAGS_seed);
  std::map<std::string, std::vector<m2::PointD>> cat;  // category -> points (mercator)
  std::vector<m2::PointD> roadPts;                      // random car-road points, all maps
  std::map<std::string, uint64_t> seen;
  auto reservoir = [&](std::vector<m2::PointD> & v, m2::PointD const & p, std::string const & key, size_t cap)
  {
    uint64_t const k = seen[key]++;
    if (v.size() < cap)
      v.push_back(p);
    else if (std::uniform_int_distribution<uint64_t>(0, k)(rng) < cap)
      v[std::uniform_int_distribution<size_t>(0, cap - 1)(rng)] = p;
  };
  for (size_t mi = 0; mi < m.names.size(); ++mi)
  {
    auto const id = m.ids[mi];
    auto const mwmId = m.dataSource.GetMwmIdByCountryFile(platform::CountryFile(m.names[mi]));
    FeaturesLoaderGuard guard(m.dataSource, mwmId);
    auto const model = m.factory->GetVehicleModelForCountry(m.names[mi]);
    auto & ig = m.world->GetIndexGraph(id);
    routing::RoadAccess ra;
    routing::ReadRoadAccessFromMwm(m.mwmDataSource->GetMwmValue(id), routing::VehicleType::Car, ra);
    std::vector<uint32_t> roadFeatures;
    for (uint32_t f = 0; f < guard.GetNumFeatures(); ++f)
    {
      auto ft = guard.GetFeatureByIndex(f);
      if (!ft || ft->GetGeomType() != feature::GeomType::Line)
        continue;
      feature::TypesHolder const th(*ft);
      if (!model->IsRoad(th))
        continue;
      auto const & road = ig.GetRoadGeometry(f);
      if (!road.IsValid() || road.GetPointsCount() < 2)
        continue;
      ft->ParseGeometry(FeatureType::BEST_GEOMETRY);
      m2::PointD const p = ft->GetPoint(ft->GetPointsCount() / 2);
      roadFeatures.push_back(f);
      reservoir(roadPts, p, "road", 20000);
      if (ftypes::IsRoundAboutChecker::Instance()(th))
        reservoir(cat["roundabout"], p, "roundabout", 30);
      if (ftypes::IsLinkChecker::Instance()(th))
        reservoir(cat["ramp"], p, "ramp", 30);
      if (ftypes::IsBridgeOrTunnelChecker::Instance()(th))
        reservoir(cat["bridge_tunnel"], p, "bridge_tunnel", 30);
      if (road.IsOneWay())
        reservoir(cat["oneway"], p, "oneway", 30);
      auto const acc = ra.GetAccessWithoutConditional(f).first;
      if (acc == routing::RoadAccess::Type::No || acc == routing::RoadAccess::Type::Private ||
          acc == routing::RoadAccess::Type::Destination)
        reservoir(cat["restricted"], p, "restricted", 30);
      // Dead end: nothing continues from the last point of the road.
      routing::IndexGraph::SegmentEdgeListT edges;
      routing::Segment const last(id, f, road.GetPointsCount() - 2, true);
      ig.GetEdgeList(last, true, true, edges);
      if (edges.empty())
        reservoir(cat["deadend"], mercator::FromLatLon(road.GetPoint(road.GetPointsCount() - 1)), "deadend", 30);
    }
    // Small islands: roads whose connected part (either direction) has fewer than 300 segments.
    std::shuffle(roadFeatures.begin(), roadFeatures.end(), rng);
    size_t islands = 0;
    for (size_t k = 0; k < std::min<size_t>(roadFeatures.size(), 4000) && islands < 30; ++k)
    {
      routing::Segment const s0(id, roadFeatures[k], 0, true);
      std::set<routing::Segment> visited{s0};
      std::vector<routing::Segment> todo{s0};
      routing::IndexGraph::SegmentEdgeListT edges;
      while (!todo.empty() && visited.size() < 300)
      {
        auto const s = todo.back();
        todo.pop_back();
        for (bool out : {true, false})
        {
          edges.clear();
          ig.GetEdgeList(s, out, true, edges);
          for (auto const & e : edges)
            if (visited.insert(e.GetTarget()).second)
              todo.push_back(e.GetTarget());
        }
      }
      if (visited.size() < 300)
      {
        cat["island"].push_back(mercator::FromLatLon(ig.GetPoint(s0, true)));
        ++islands;
      }
    }
    // Border crossings.
    routing::CrossMwmIndexGraph<base::GeoObjectId> cross(*m.mwmDataSource, routing::VehicleType::Car);
    auto const & c = cross.GetCrossMwmConnectorWithTransitions(id);
    c.ForEachEnter([&](uint32_t, routing::Segment const & sg)
    {
      if (ig.GetRoadGeometry(sg.GetFeatureId()).IsValid())
        reservoir(cat["border"], mercator::FromLatLon(ig.GetPoint(sg, true)), "border", 40);
    });
    std::cout << "  " << m.names[mi] << ": " << roadFeatures.size() << " car roads\n";
  }

  std::ofstream out(FLAGS_routes_out);
  out.precision(9);
  auto write = [&](std::string const & name, m2::PointD const & a, m2::PointD const & b)
  {
    auto const la = mercator::ToLatLon(a), lb = mercator::ToLatLon(b);
    out << name << " " << la.m_lat << " " << la.m_lon << " " << lb.m_lat << " " << lb.m_lon << "\n";
  };
  auto partner = [&](m2::PointD const & p, double minKm, double maxKm, m2::PointD & q)
  {
    for (int t = 0; t < 200000; ++t)
    {
      q = roadPts[std::uniform_int_distribution<size_t>(0, roadPts.size() - 1)(rng)];
      double const km = mercator::DistanceOnEarth(p, q) / 1000;
      if (km >= minKm && km <= maxKm)
        return true;
    }
    return false;
  };
  size_t total = 0;
  for (auto const & [name, pts] : cat)
  {
    size_t i = 0;
    for (auto const & p : pts)
    {
      m2::PointD q;
      if (!partner(p, 1, 30, q))
        continue;
      if (i % 2 == 0)
        write(name + "-" + std::to_string(i), p, q);
      else
        write(name + "-" + std::to_string(i), q, p);
      ++i, ++total;
    }
    std::cout << "  " << name << ": " << i << " routes\n";
  }
  // Random pairs: short (0.2-50 km) and long (300+ km).
  for (int i = 0; i < 30; ++i)
  {
    m2::PointD const p = roadPts[std::uniform_int_distribution<size_t>(0, roadPts.size() - 1)(rng)];
    m2::PointD q;
    if (partner(p, 0.2, 50, q))
      write("random_short-" + std::to_string(i), p, q), ++total;
  }
  for (int i = 0; i < 15; ++i)
  {
    m2::PointD const p = roadPts[std::uniform_int_distribution<size_t>(0, roadPts.size() - 1)(rng)];
    m2::PointD q;
    if (partner(p, 300, 5000, q))
      write("random_long-" + std::to_string(i), p, q), ++total;
  }
  // Farthest pairs among a sample.
  std::vector<std::tuple<double, size_t, size_t>> far;
  for (size_t i = 0; i < std::min<size_t>(roadPts.size(), 3000); i += 3)
    for (size_t j = i + 1; j < std::min<size_t>(roadPts.size(), 3000); j += 7)
      far.emplace_back(mercator::DistanceOnEarth(roadPts[i], roadPts[j]), i, j);
  std::sort(far.rbegin(), far.rend());
  std::set<size_t> used;
  int nf = 0;
  for (auto const & [d, i, j] : far)
  {
    if (nf >= 6 || used.count(i) || used.count(j))
      continue;
    used.insert(i), used.insert(j);
    write("extreme-" + std::to_string(nf++), roadPts[i], roadPts[j]), ++total;
  }
  std::cout << "  " << total << " routes written to " << FLAGS_routes_out << "\n";
  return 0;
}

struct RouterAnswer
{
  bool ok = false;
  double secs = 0, eta = 0;
  std::vector<routing::Segment> segs;
};

std::map<std::string, RouterAnswer> ReadRouter(std::string const & path)
{
  std::map<std::string, RouterAnswer> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line))
  {
    std::stringstream ss(line);
    std::string name;
    RouterAnswer a;
    int ok = 0;
    size_t n = 0;
    ss >> name >> ok >> a.secs >> a.eta >> n;
    a.ok = ok != 0;
    for (size_t i = 0; i < n; ++i)
    {
      std::string t;
      ss >> t;
      a.segs.push_back(ParseSeg(t));
    }
    out[name] = std::move(a);
  }
  return out;
}
}  // namespace

int main(int argc, char ** argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << std::unitbuf;
  std::cout.setf(std::ios::fixed);
  std::cout.precision(3);
  auto const mwms = SplitList(FLAGS_mwms, ';');
  std::cout << "== Comaps++ multi-map overlay (" << FLAGS_mode << "): " << FLAGS_mwms << " v" << FLAGS_mwm_version
            << "\n";
  Multi m;
  m.Init(mwms);

  if (FLAGS_mode == "gen_stress")
    return RunGenStress(m);
  if (FLAGS_mode == "reroute")
  {
    if (FLAGS_force_joints)
      setenv("COMAPS_PLUS_FORCE_JOINTS", "1", 1);
    return RunReroute(m);
  }
  if (FLAGS_mode == "router")
    return RunRouter(m);
  if (FLAGS_mode == "runtime")
    return RunRuntime(m);

  // 1. Combined graph.
  std::cout << "\n[1] Combined segment graph\n";
  Combined c;
  BuildCombined(m, c);

  // 2. Compression.
  std::cout << "\n[2] Chain compression\n";
  auto t0 = Clock::now();
  Compressed comp = CompressChains(c.g, c.valid);
  Graph & g = comp.g;
  std::cout << "  " << g.n << " vertices, " << g.NumEdges() << " edges in " << Since(t0) << " s, " << Mem() << "\n";
  std::vector<uint32_t> rawToComp(c.g.n, kNone);
  for (uint32_t cv = 0; cv < g.n; ++cv)
    rawToComp[comp.origId[cv]] = cv;

  // 3. Partition: every map on its own, then cell ids offset so that no cell spans two maps.
  std::cout << "\n[3] Partition (per map)\n";
  PartitionParams pp;
  pp.cellSizes = ParseSizes(FLAGS_cell_sizes);
  pp.minBalance = FLAGS_min_balance;
  pp.threads = FLAGS_threads;
  Partition part;
  part.cell.assign(pp.cellSizes.size(), std::vector<uint32_t>(g.n, kNone));
  part.numCells.assign(pp.cellSizes.size(), 0);
  std::vector<uint32_t> partOfComp(g.n);
  for (uint32_t v = 0; v < g.n; ++v)
    partOfComp[v] = static_cast<uint32_t>(c.PartOf(comp.origId[v]));
  double partSec = 0;
  for (size_t pi = 0; pi < c.parts.size(); ++pi)
  {
    auto const tp = Clock::now();
    std::vector<VertexId> local;  // compressed ids of this map
    std::vector<uint32_t> toLocal(g.n, kNone);
    for (uint32_t v = 0; v < g.n; ++v)
      if (partOfComp[v] == pi)
      {
        toLocal[v] = static_cast<uint32_t>(local.size());
        local.push_back(v);
      }
    Graph sub;
    sub.n = static_cast<uint32_t>(local.size());
    sub.fwdOff.assign(sub.n + 1, 0);
    for (uint32_t i = 0; i < sub.n; ++i)
    {
      VertexId const v = local[i];
      for (uint64_t e = g.fwdOff[v]; e < g.fwdOff[v + 1]; ++e)
        if (toLocal[g.fwdTo[e]] != kNone)
        {
          sub.fwdTo.push_back(toLocal[g.fwdTo[e]]);
          sub.fwdW.push_back(g.fwdW[e]);
        }
      sub.fwdOff[i + 1] = sub.fwdTo.size();
      sub.lat.push_back(g.lat[v]);
      sub.lon.push_back(g.lon[v]);
    }
    sub.BuildReverse();
    Partition const sp = Bisect(sub, pp);
    for (size_t li = 0; li < sp.Levels(); ++li)
    {
      for (uint32_t i = 0; i < sub.n; ++i)
        part.cell[li][local[i]] = part.numCells[li] + sp.cell[li][i];
      part.numCells[li] += sp.numCells[li];
    }
    double const s = Since(tp);
    partSec += s;
    std::cout << "  " << c.parts[pi].name << ": " << sub.n << " vertices, cells per level";
    for (auto nc : sp.numCells)
      std::cout << " " << nc;
    std::cout << " in " << s << " s, " << Mem() << "\n";
  }

  // 4. Customization.
  std::cout << "\n[4] Overlay\n";
  Overlay ov;
  t0 = Clock::now();
  BuildBoundaries(g, part, ov);
  for (size_t li = 0; li < part.Levels(); ++li)
  {
    auto const & L = ov.levels[li];
    std::cout << "  level " << (li + 1) << ": cells " << part.numCells[li] << ", entries " << L.entries.size()
              << ", exits " << L.exits.size() << ", matrix entries " << L.mat.size() << "\n";
  }
  auto const levelSec = Customize(g, part, ov, FLAGS_threads);
  std::cout << "  customize " << Since(t0) << " s (" << FLAGS_threads << " threads), " << Mem() << "\n";
  auto const st = MeasureStorage(g, part, ov);
  std::cout << "  matrices " << st.matrixEntries << " entries (" << st.finiteEntries << " finite) = "
            << MB(st.matrixEntries * 8) << " as double\n";
  (void)levelSec;

  // 5. Point-to-point machinery (as plus_overlay_tool [6c]/[6d], over the combined graph).
  std::cout << "\n[5] Routes\n";
  Scratch sc(g.n);
  traffic::TrafficCache traffic;
  std::shared_ptr<storage::CountryInfoGetter> cig;
  auto router = m.MakeRouter(traffic, cig);
  auto & world = *m.world;

  std::vector<VertexId> pred(c.g.n, kNone);  // chain predecessor of every non-kept raw vertex
  for (VertexId u = 0; u < c.g.n; ++u)
    for (uint64_t e = c.g.fwdOff[u]; e < c.g.fwdOff[u + 1]; ++e)
      if (rawToComp[c.g.fwdTo[e]] == kNone)
        pred[c.g.fwdTo[e]] = u;
  std::unordered_map<VertexId, std::vector<VertexId>> copiesOf;
  for (auto const & p : c.parts)
    for (VertexId l = p.baseN; l < p.n; ++l)
      copiesOf[p.base + p.copyOrig[l - p.baseN]].push_back(p.base + l);

  auto walkForward = [&](VertexId r, double cost, std::vector<VertexId> & chain)
  {
    chain.assign(1, r);
    VertexId x = r;
    while (rawToComp[x] == kNone && c.g.fwdOff[x + 1] - c.g.fwdOff[x] == 1 && chain.size() < 100000)
    {
      cost += c.g.fwdW[c.g.fwdOff[x]];
      x = c.g.fwdTo[c.g.fwdOff[x]];
      chain.push_back(x);
    }
    return std::make_pair(rawToComp[x], cost);
  };

  using Parents = routing::AStarGraph<routing::Segment, routing::SegmentEdge, routing::RouteWeight>::Parents;
  auto fakeSearch = [&](routing::IndexGraphStarter & starter, bool forward, std::map<routing::Segment, double> & real,
                        double & direct)
  {
    auto const src = forward ? starter.GetStartSegment() : starter.GetFinishSegment();
    auto const dst = forward ? starter.GetFinishSegment() : starter.GetStartSegment();
    std::map<routing::Segment, double> dist{{src, 0.0}};
    std::set<std::pair<double, routing::Segment>> pq{{0.0, src}};
    std::set<routing::Segment> done;
    routing::IndexGraphStarter::EdgeListT edges;
    while (!pq.empty())
    {
      auto const [d, x] = *pq.begin();
      pq.erase(pq.begin());
      if (!done.insert(x).second)
        continue;
      if (x == dst)
      {
        direct = std::min(direct, d);
        continue;
      }
      if (!routing::IndexGraphStarter::IsFakeSegment(x))
      {
        auto it = real.find(x);
        if (it == real.end() || d < it->second)
          real[x] = d;
        continue;
      }
      edges.clear();
      routing::astar::VertexData<routing::Segment, routing::RouteWeight> const vd(x, routing::RouteWeight(d));
      if (forward)
        starter.GetOutgoingEdgesList(vd, edges);
      else
        starter.GetIngoingEdgesList(vd, edges);
      for (auto const & e : edges)
      {
        double const nd = d + e.GetWeight().GetIntegratedWeight();
        auto it = dist.find(e.GetTarget());
        if (it == dist.end() || nd < it->second)
        {
          dist[e.GetTarget()] = nd;
          pq.insert({nd, e.GetTarget()});
        }
      }
    }
  };

  struct Answer
  {
    double weight = kInf, ms = 0;
    uint64_t settled = 0;
    std::vector<routing::Segment> segs;
  };
  auto solve = [&](std::map<routing::Segment, double> const & srcReal,
                   std::map<routing::Segment, double> const & tgtReal, double direct)
  {
    std::unordered_map<VertexId, double> tRaw;
    for (auto const & [seg, cst] : tgtReal)
    {
      VertexId const b = c.FromSegment(seg);
      tRaw[b] = cst;
      if (auto it = copiesOf.find(b); it != copiesOf.end())
        for (VertexId x : it->second)
          tRaw[x] = cst;
    }
    std::map<uint32_t, std::pair<double, std::vector<VertexId>>> srcChain, tgtChain;
    std::vector<VertexId> chain, directPath;
    for (auto const & [seg, cst] : srcReal)
    {
      auto const [kc, cost] = walkForward(c.FromSegment(seg), cst, chain);
      double acc = cst;
      for (size_t i = 0; i < chain.size(); ++i)
      {
        if (i > 0)
          acc += RawEdge(c.g, chain[i - 1], chain[i]);
        auto it = tRaw.find(chain[i]);
        if (it != tRaw.end() && acc + it->second < direct)
        {
          direct = acc + it->second;
          directPath.assign(chain.begin(), chain.begin() + i + 1);
        }
      }
      if (kc != kNone && (!srcChain.count(kc) || cost < srcChain[kc].first))
        srcChain[kc] = {cost, chain};
    }
    for (auto const & [target, cst] : tRaw)
    {
      VertexId x = target;
      double cost = cst;
      std::vector<VertexId> back{x};
      while (rawToComp[x] == kNone && pred[x] != kNone && back.size() < 100000)
      {
        cost += RawEdge(c.g, pred[x], x);
        x = pred[x];
        back.push_back(x);
      }
      if (rawToComp[x] == kNone)
        continue;
      std::reverse(back.begin(), back.end());
      uint32_t const kc = rawToComp[x];
      if (!tgtChain.count(kc) || cost < tgtChain[kc].first)
        tgtChain[kc] = {cost, back};
    }
    Seeds src, tgt;
    for (auto const & [kc, v] : srcChain)
      src.emplace_back(kc, v.first);
    for (auto const & [kc, v] : tgtChain)
      tgt.emplace_back(kc, v.first);
    auto const ovr = OverlayQueryMulti(g, part, ov, src, tgt, sc, true /* wantPath */);
    Answer out;
    out.settled = ovr.settled;
    out.weight = std::min(ovr.weight, direct);
    std::vector<VertexId> rawPath;
    if (direct <= ovr.weight)
    {
      rawPath = directPath;
    }
    else if (!ovr.path.empty())
    {
      auto const & pre = srcChain[ovr.path.front()].second;
      rawPath.assign(pre.begin(), pre.end() - 1);
      auto const mid = ToRawPath(c.g, comp, rawToComp, ovr.path);
      rawPath.insert(rawPath.end(), mid.begin(), mid.end());
      auto const & suf = tgtChain[ovr.path.back()].second;
      rawPath.insert(rawPath.end(), suf.begin() + 1, suf.end());
    }
    // A twin link joins the same segment in two maps; CoMaps lists it once.
    for (VertexId rv : rawPath)
    {
      auto const s = c.ToSegment(rv);
      if (!out.segs.empty() && out.segs.back().GetFeatureId() == s.GetFeatureId() &&
          out.segs.back().GetSegmentIdx() == s.GetSegmentIdx() && out.segs.back().IsForward() == s.IsForward() &&
          out.segs.back().GetMwmId() != s.GetMwmId())
        continue;
      out.segs.push_back(s);
    }
    return out;
  };

  // CoMaps' weight between consecutive real segments of a route (a map change may go through a
  // twin link first).
  auto pairWeight = [&](routing::Segment const & a, routing::Segment const & b)
  {
    VertexId const u = c.FromSegment(a), v = c.FromSegment(b);
    double best = kInf;
    for (uint64_t e = c.g.fwdOff[u]; e < c.g.fwdOff[u + 1]; ++e)
    {
      VertexId const x = c.g.fwdTo[e];
      if (c.Orig(x) == v)
        best = std::min(best, c.g.fwdW[e]);
      else if (c.g.fwdW[e] == 0.0 && c.PartOf(x) != c.PartOf(u))  // twin link
        for (uint64_t f = c.g.fwdOff[x]; f < c.g.fwdOff[x + 1]; ++f)
          if (c.Orig(c.g.fwdTo[f]) == v)
            best = std::min(best, c.g.fwdW[f]);
    }
    // CoMaps' joints search may also switch maps at any junction of a road that exists in both maps:
    // the segment is relabelled to the twin feature (same segment index); the edge weight comes from the
    // map being left. Score that as the edge a -> (b on a's map) or (a on b's map) -> b.
    if (best >= kInf && a.GetMwmId() != b.GetMwmId())
    {
      if (auto b2 = c.OnTwin(b, a.GetMwmId()))
      {
        VertexId const v2 = c.FromSegment(*b2);
        for (uint64_t e = c.g.fwdOff[u]; e < c.g.fwdOff[u + 1]; ++e)
          if (c.Orig(c.g.fwdTo[e]) == v2)
            best = std::min(best, c.g.fwdW[e]);
      }
      if (auto a2 = c.OnTwin(a, b.GetMwmId()))
      {
        VertexId const u2 = c.FromSegment(*a2);
        for (uint64_t e = c.g.fwdOff[u2]; e < c.g.fwdOff[u2 + 1]; ++e)
          if (c.Orig(c.g.fwdTo[e]) == v)
            best = std::min(best, c.g.fwdW[e]);
      }
    }
    return best;
  };
  // Driving time of a segment path as CoMaps' Route computes it (no penalties).
  auto etaOf = [&](std::vector<routing::Segment> const & p)
  {
    if (p.empty())
      return 0.0;
    double t = world.CalculateETAWithoutPenalty(p[0]);
    for (size_t k = 1; k < p.size(); ++k)
      t += world.CalculateETA(p[k - 1], p[k]);
    return t;
  };
  // Map changes along a path, for the report.
  auto mapChanges = [&](std::vector<routing::Segment> const & p)
  {
    std::string out;
    int shown = 0;
    for (size_t k = 1; k < p.size() && shown < 3; ++k)
      if (p[k].GetMwmId() != p[k - 1].GetMwmId())
      {
        out += " [" + std::to_string(k) + "] " + SegStr(p[k - 1]) + " -> " + SegStr(p[k]);
        ++shown;
      }
    return out.empty() ? std::string(" none") : out;
  };

  struct Point2Point
  {
    bool snapped = false;
    Answer ans;
    std::map<routing::Segment, double> srcReal, tgtReal;
    double direct = kInf;
  };
  auto pointToPoint = [&](ms::LatLon const & a, ms::LatLon const & b)
  {
    Point2Point r;
    routing::FakeEnding se, fe;
    bool codir = false;
    if (router->PlusSnap(mercator::FromLatLon(a), mercator::FromLatLon(b), se, fe, codir) != 0)
      return r;
    r.snapped = true;
    auto const tq = Clock::now();
    routing::IndexGraphStarter starter(se, fe, 0 /* fakeNumerationStart */, codir /* strictForward */, world);
    Parents pf, pb;
    starter.SetAStarParents(true, pf);
    starter.SetAStarParents(false, pb);
    fakeSearch(starter, true, r.srcReal, r.direct);
    fakeSearch(starter, false, r.tgtReal, r.direct);
    starter.DropAStarParents();
    r.ans = solve(r.srcReal, r.tgtReal, r.direct);
    r.ans.ms = Since(tq) * 1000;
    return r;
  };

  auto routes = FLAGS_routes.empty() ? BuiltinRoutes() : ReadRoutes(FLAGS_routes);
  // Random pairs with the ends in different maps.
  {
    std::mt19937_64 rng(FLAGS_seed);
    std::vector<uint8_t> all(g.n, 1);
    uint32_t sccSize = 0;
    VertexId seed = 0;
    for (uint32_t v = 0; v < g.n; ++v)
      if (partOfComp[v] == 0)
      {
        seed = v;
        break;
      }
    auto const inScc = StronglyConnectedFrom(g, seed, sccSize);
    std::uniform_int_distribution<uint32_t> pick(0, g.n - 1);
    uint32_t added = 0;
    for (int attempt = 0; added < FLAGS_random_pairs && attempt < 1000000; ++attempt)
    {
      uint32_t const u = pick(rng), v = pick(rng);
      if (!inScc[u] || !inScc[v] || partOfComp[u] == partOfComp[v] || c.IsCopy(comp.origId[u]) ||
          c.IsCopy(comp.origId[v]))
        continue;
      ++added;
      routes.push_back({"random" + std::to_string(added), ms::LatLon(g.lat[u], g.lon[u]),
                        ms::LatLon(g.lat[v], g.lon[v])});
    }
  }
  if (!FLAGS_routes_out.empty())
  {
    std::ofstream out(FLAGS_routes_out);
    out.precision(7);
    size_t const named = FLAGS_routes.empty() ? BuiltinRoutes().size() : ReadRoutes(FLAGS_routes).size();
    for (size_t i = 0; i < routes.size() && i < named + FLAGS_router_pairs; ++i)
      out << std::fixed << routes[i].name << " " << routes[i].a.m_lat << " " << routes[i].a.m_lon << " "
          << routes[i].b.m_lat << " " << routes[i].b.m_lon << "\n";
  }
  auto const routerAnswers = FLAGS_router_in.empty() ? std::map<std::string, RouterAnswer>() : ReadRouter(FLAGS_router_in);

  uint32_t compared = 0, match = 0, identical = 0, routerWorse = 0, overlayWorse = 0;
  std::vector<double> qms;
  for (auto const & r : routes)
  {
    double const km = ms::DistanceOnEarth(r.a, r.b) / 1000;
    auto const pp2 = pointToPoint(r.a, r.b);
    char head[96];
    std::snprintf(head, sizeof(head), "  %-18s %6.0f km: ", r.name.c_str(), km);
    std::cout << head;
    if (!pp2.snapped || pp2.ans.weight >= kInf)
    {
      std::cout << (pp2.snapped ? "overlay: no route" : "snap failed") << "\n";
      continue;
    }
    qms.push_back(pp2.ans.ms);
    std::set<routing::NumMwmId> maps;
    for (auto const & s : pp2.ans.segs)
      maps.insert(s.GetMwmId());
    std::cout << "overlay " << Hms(pp2.ans.weight) << " (" << pp2.ans.weight << " s) in " << pp2.ans.ms << " ms, "
              << pp2.ans.settled << " settled, " << pp2.ans.segs.size() << " segments, " << maps.size() << " maps";
    auto const it = routerAnswers.find(r.name);
    if (it == routerAnswers.end())
    {
      std::cout << "\n";
      continue;
    }
    auto const & ra = it->second;
    if (!ra.ok)
    {
      std::cout << " | router FAILED\n";
      continue;
    }
    // Router total on the same starter: cheapest valid fake/real split (a lower bound on its cost).
    auto const & rs = ra.segs;
    double bestSplit = kInf, runMin = kInf;
    size_t runArg = 0, bi = 0, bj = 0;
    for (size_t k = 0; k < rs.size(); ++k)
    {
      if (k > 0 && runMin < kInf)
      {
        double const w = pairWeight(rs[k - 1], rs[k]);
        runMin = (w < kInf) ? runMin + w : kInf;
      }
      if (auto s = pp2.srcReal.find(rs[k]); s != pp2.srcReal.end() && s->second < runMin)
      {
        runMin = s->second;
        runArg = k;
      }
      if (auto t = pp2.tgtReal.find(rs[k]); t != pp2.tgtReal.end() && runMin + t->second < bestSplit)
      {
        bestSplit = runMin + t->second;
        bi = runArg;
        bj = k;
      }
    }
    ++compared;
    bool const same = Same(bestSplit, pp2.ans.weight);
    std::string firstGap;
    for (size_t k = 1; k < rs.size() && firstGap.empty(); ++k)
      if (pairWeight(rs[k - 1], rs[k]) >= kInf)
        firstGap = " [" + std::to_string(k) + "] " + SegStr(rs[k - 1]) + " -> " + SegStr(rs[k]);
    bool const ident = same && std::vector<routing::Segment>(rs.begin() + bi, rs.begin() + bj + 1) == pp2.ans.segs;
    match += same;
    identical += ident;
    if (!same && bestSplit > pp2.ans.weight)
      ++routerWorse;
    if (!same && bestSplit < pp2.ans.weight)
      ++overlayWorse;
    std::cout << " | router (joints) " << bestSplit << " s in " << ra.secs << " s | "
              << (same ? (ident ? "MATCH, identical path" : "MATCH, different path of equal weight")
                       : (bestSplit < kInf && bestSplit > pp2.ans.weight
                              ? "ROUTER WORSE by " + std::to_string(bestSplit - pp2.ans.weight) + " s"
                              : "OVERLAY WORSE / router path not scorable (" + std::to_string(bestSplit) + ")"))
              << "\n      ETA (real segments): overlay " << Hms(etaOf(pp2.ans.segs)) << ", router "
              << Hms(etaOf(rs)) << " (router's own " << Hms(ra.eta) << ")"
              << "\n      map changes: router" << mapChanges(rs) << " | overlay" << mapChanges(pp2.ans.segs)
              << (firstGap.empty() ? "" : "\n      first unscorable router step:" + firstGap) << "\n";
  }
  std::sort(qms.begin(), qms.end());
  std::cout << "  overlay queries: " << qms.size() << ", median " << (qms.empty() ? 0 : qms[qms.size() / 2])
            << " ms, max " << (qms.empty() ? 0 : qms.back()) << " ms\n";
  if (compared)
    std::cout << "  vs router: " << compared << " compared, weight match " << match << " (identical path "
              << identical << "), router worse " << routerWorse << ", overlay worse/unscorable " << overlayWorse
              << "\n";
  std::cout << "\nDone. " << Mem() << "\n";
  return 0;
}
