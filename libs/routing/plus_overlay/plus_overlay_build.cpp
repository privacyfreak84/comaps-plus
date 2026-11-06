// Comaps++ overlay builder for one map, written the way a phone would have to run it.
//
// No raw segment graph is ever held in memory. Instead:
//   pass 1  one walk over IndexGraph::GetEdgeList for every directed segment, keeping only an in-degree
//           and an out-degree (one byte each, saturated at 2);
//   pass 2  from every kept segment (a junction end, a border transition, or any segment of a feature in
//           a via-way restriction) walk the chain of single-successor segments to the next kept one and
//           emit one compressed edge with the summed weight;
// then via-way restrictions are expanded on the compressed graph (automaton states as copies, as in
// plus_overlay_tool), the map is partitioned on all threads, border transitions are marked as boundary
// vertices at every level (so maps can be joined at query time), the cell cliques are computed with
// cell-local work arrays, and everything is written to one file.
//
// --verify_raw additionally builds the old raw-graph pipeline (much more memory) and checks that
// shortest-path weights between random segments agree exactly.

#define PLUS_OVERLAY_TOOL_NO_MAIN
#include "plus_overlay_tool.cpp"

#include "routing/cross_mwm_index_graph.hpp"
#include "routing/plus_overlay/overlay_file.hpp"

#include <tuple>

DEFINE_string(out, "", "Overlay file to write (empty: don't write).");
DEFINE_bool(verify_raw, false, "Also run the raw-graph pipeline and compare shortest-path weights exactly.");
DEFINE_uint32(verify_pairs, 300, "Random segment pairs for --verify_raw.");
DEFINE_uint32(check_file, 0, "After writing --out, load it back and compare this many random queries with the "
                             "in-memory (double) overlay.");

namespace
{
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
  uint32_t Slot(routing::Segment const & s) const
  {
    return 2 * (featBase[s.GetFeatureId()] + s.GetSegmentIdx()) + (s.IsForward() ? 0 : 1);
  }
  uint32_t FeatureOfSlot(uint32_t v) const
  {
    return static_cast<uint32_t>(std::upper_bound(featBase.begin(), featBase.end(), v / 2) - featBase.begin()) - 1;
  }
};

void LeanExtract(routing::IndexGraph & ig, routing::NumMwmId mwmId, std::vector<routing::Segment> const & forced,
                 std::vector<std::vector<uint32_t>> const & viaWay, LeanGraph & lg)
{
  auto const t0 = Clock::now();
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
  lg.featBase.assign(maxF + 2, 0);
  for (uint32_t f = 0; f <= maxF; ++f)
    lg.featBase[f + 1] = lg.featBase[f] + segs[f];
  uint32_t const n = 2 * lg.featBase[maxF + 1];
  lg.rawSlots = n;

  auto forEachValid = [&](auto && fn)
  {
    for (uint32_t f : roads)
      for (uint32_t i = 0; i < segs[f]; ++i)
        for (bool fwd : {true, false})
          if (fwd || !oneway[f])
            fn(routing::Segment(mwmId, f, i, fwd));
  };

  // Pass 1: degrees.
  std::vector<uint8_t> indeg(n, 0), outdeg(n, 0), valid(n, 0);
  routing::IndexGraph::SegmentEdgeListT edges;
  forEachValid([&](routing::Segment const & s)
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
  std::cout << "  pass 1 (degrees): " << lg.rawValid << " directed segments, " << lg.rawEdges << " edges in "
            << Since(t0) << " s, " << Mem() << "\n";

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
  uint64_t forcedCount = 0;
  for (auto const & s : forced)
  {
    uint32_t const v = lg.Slot(s);
    if (v < n && valid[v] && !lg.Kept(v))
      keep(v), ++forcedCount;
  }
  for (auto const & pat : viaWay)
    for (uint32_t f : pat)
      if (f + 1 < lg.featBase.size())
        for (uint32_t v = 2 * lg.featBase[f]; v < 2 * lg.featBase[f + 1]; ++v)
          if (valid[v] && !lg.Kept(v))
            keep(v), ++forcedCount;
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
  forEachValid([&](routing::Segment const & s)
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
  std::cout << "  pass 2 (chains): " << kn << " kept (" << forcedCount << " forced: border transitions and via-way"
            << " features), " << g.NumEdges() << " compressed edges, " << dropped << " dead-end chains in "
            << Since(t1) << " s, " << Mem() << "\n";
}

// Via-way restrictions on the compressed graph. All segments of features in a pattern are kept, so a
// chain interior is never a pattern feature: passing one resets the automaton.
void LeanExpandViaWay(LeanGraph & lg, std::vector<std::vector<uint32_t>> const & patterns)
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
    g.lat.push_back(g.lat[v]);
    g.lon.push_back(g.lon[v]);
    lg.copyOrig.push_back(v);
    lg.copyState.push_back(st);
  }
  g.n += static_cast<uint32_t>(copies.size());
  std::cout << "  via-way: " << patterns.size() << " patterns, " << ac.Size() << " states, " << copies.size()
            << " copies, " << redirected << " edges redirected, " << dropped << " forbidden (" << Since(t0) << " s)\n";
}

void g_shrink(Graph & g)
{
  g.fwdTo.shrink_to_fit();
  g.fwdW.shrink_to_fit();
}

template <typename T>
uint64_t VecBytes(std::vector<T> const & v)
{
  return 8 + v.size() * sizeof(T);
}
}  // namespace

int main(int argc, char ** argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << std::unitbuf;
  std::cout.setf(std::ios::fixed);
  std::cout.precision(3);
  std::cout << "== Comaps++ lean overlay build: " << FLAGS_mwm << " v" << FLAGS_mwm_version << ", " << FLAGS_threads
            << " threads\n";
  auto const tAll = Clock::now();

  auto cm = std::make_unique<CoMaps>();
  cm->Init();
  cm->ReportRestrictionsAndAccess();
  std::cout << "  CoMaps IndexGraph ready, " << Mem() << "\n";
  auto const viaWay = cm->viaWay;

  // Border transitions (both directions): kept, and boundary vertices at every level.
  std::vector<routing::Segment> enters, exits;
  std::vector<uint64_t> enterOsm, exitOsm;
  {
    routing::CrossMwmIndexGraph<base::GeoObjectId> cross(*cm->mwmDataSource, routing::VehicleType::Car);
    auto const & c = cross.GetCrossMwmConnectorWithTransitions(cm->mwmId);
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

  std::cout << "\n[1] Lean extraction\n";
  LeanGraph lg;
  auto t0 = Clock::now();
  LeanExtract(*cm->graph, cm->mwmId, forced, cm->viaWay, lg);
  LeanExpandViaWay(lg, cm->viaWay);
  std::vector<uint8_t>().swap(lg.interior);
  double const extractSec = Since(t0);
  Graph & g = lg.g;
  if (!FLAGS_verify_raw)
  {
    // The phone keeps its router's IndexGraph anyway; here it is only needed for extraction.
    cm.reset();
  }
  std::cout << "  compressed graph: " << g.n << " vertices, " << g.NumEdges() << " edges; extraction total "
            << extractSec << " s, " << Mem() << "\n";

  std::vector<uint8_t> extra(g.n, 0);
  uint64_t borderMarked = 0;
  for (auto const & s : enters)
    if (uint32_t const r = lg.Slot(s); lg.Kept(r))
      extra[lg.Rank(r)] |= 1, ++borderMarked;
  for (auto const & s : exits)
    if (uint32_t const r = lg.Slot(s); lg.Kept(r))
      extra[lg.Rank(r)] |= 2, ++borderMarked;

  std::cout << "\n[2] Partition (" << FLAGS_threads << " threads)\n";
  PartitionParams pp;
  pp.cellSizes = ParseSizes(FLAGS_cell_sizes);
  pp.minBalance = FLAGS_min_balance;
  pp.threads = FLAGS_threads;
  t0 = Clock::now();
  Partition part = Bisect(g, pp);
  double const partSec = Since(t0);
  std::cout << "  cells per level";
  for (auto nc : part.numCells)
    std::cout << " " << nc;
  std::cout << " in " << partSec << " s, " << Mem() << "\n";
  if (!FLAGS_verify_raw && FLAGS_check_file == 0)
  {
    std::vector<float>().swap(g.lat);
    std::vector<float>().swap(g.lon);
  }

  std::cout << "\n[3] Overlay (" << FLAGS_threads << " threads)\n";
  Overlay ov;
  t0 = Clock::now();
  BuildBoundaries(g, part, ov, &extra);
  double const boundSec = Since(t0);
  std::cout << "  boundaries in " << boundSec << " s (" << borderMarked << " border transitions marked), " << Mem()
            << "\n";
  std::vector<uint8_t>().swap(extra);
  t0 = Clock::now();
  auto const levelSec = Customize(g, part, ov, FLAGS_threads);
  double const custSec = Since(t0);
  for (size_t li = 0; li < levelSec.size(); ++li)
    std::cout << "  level " << (li + 1) << ": cells " << part.numCells[li] << ", entries "
              << ov.levels[li].entries.size() << ", exits " << ov.levels[li].exits.size() << ", matrix "
              << ov.levels[li].mat.size() << " in " << levelSec[li] << " s\n";
  std::cout << "  customize " << custSec << " s, " << Mem() << "\n";

  // Sizes of what a device keeps, as written and in packed variants.
  std::cout << "\n[4] Storage\n";
  uint64_t matEntries = 0, finite = 0, boundaryIds = 0, cellOffs = 0;
  for (auto const & L : ov.levels)
  {
    matEntries += L.mat.size();
    for (double x : L.mat)
      finite += x < kInf;
    boundaryIds += L.entries.size() + L.exits.size();
    cellOffs += L.entryOff.size() + L.exitOff.size();
  }
  uint64_t const graphDouble = (g.n + 1) * 4ULL + g.NumEdges() * (4 + 8);
  uint64_t const graphFloat = (g.n + 1) * 4ULL + g.NumEdges() * (4 + 4);
  uint64_t const ids = lg.origId.size() * 4ULL + lg.copyOrig.size() * 4ULL;
  uint64_t const cellsPacked = g.n * 4ULL;  // level-1 cell id per vertex; higher levels via a parent table
  std::cout << "  matrices: " << matEntries << " entries (" << finite << " finite): " << MB(matEntries * 8)
            << " double, " << MB(matEntries * 4) << " float32, ~" << MB(finite * 4 + matEntries / 8)
            << " sparse float32 (+1 bit per entry)\n"
            << "  boundary ids + offsets: " << MB(boundaryIds * 4 + cellOffs * 4) << "\n"
            << "  cell ids: " << MB(cellsPacked) << " (level 1 per vertex) + parent tables\n"
            << "  compressed base graph (for the search near the ends and for unpacking): " << MB(graphDouble)
            << " double weights, " << MB(graphFloat) << " float32\n"
            << "  segment ids of compressed vertices: " << MB(ids) << "\n";

  if (!FLAGS_out.empty())
  {
    t0 = Clock::now();
    FileContents fc;
    fc.mwmVersion = FLAGS_mwm_version;
    fc.paramsHash = BuildParamsHash(pp.cellSizes, pp.minBalance, pp.directions);
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
    std::string err;
    CHECK(WriteOverlayFile(FLAGS_out, fc, part, ov, &err), (err));
    auto const mo = MappedOverlay::Open(FLAGS_out, FLAGS_mwm_version, fc.paramsHash, &err);
    CHECK(mo, (err));
    std::cout << "  written " << FLAGS_out << ": " << MB(mo->FileBytes()) << " (format v" << kFileFormatVersion
              << ", " << fc.transitions.size() << " border transitions) in " << Since(t0) << " s\n";
    if (FLAGS_check_file > 0)
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
        auto const & L = mo->GetLevel(li);
        auto const & M = ov.levels[li];
        for (size_t i = 0; i < M.mat.size(); ++i)
        {
          bool const inf0 = M.mat[i] >= kInf, inf1 = L.mat[i] >= 1e29f;
          if (inf0 != inf1)
            ++bad;
          else if (!inf0)
            maxRel = std::max(maxRel, std::abs(M.mat[i] - L.mat[i]) / std::max(1.0, M.mat[i]));
        }
      }
      std::cout << "  file check: " << bad << " mismatches in ids/cells/roles; cliques max relative float32 error "
                << maxRel << "\n";
      CHECK_EQUAL(bad, 0, ());
    }
  }

  std::cout << "\n[5] Build total " << Since(tAll) << " s (extraction " << extractSec << ", partition " << partSec
            << ", boundaries " << boundSec << ", customize " << custSec << "), " << Mem() << "\n";

  if (!FLAGS_verify_raw)
    return 0;

  // Exactness against the raw-graph pipeline of plus_overlay_tool.
  std::cout << "\n[6] Verify against the raw-graph pipeline\n";
  Raw raw;
  if (FLAGS_cache.empty() || !LoadRaw(raw, FLAGS_cache))
    Extract(*cm, raw);
  ExpandViaWay(raw, cm->viaWay);
  Compressed comp = CompressChains(raw.g, raw.valid);
  std::cout << "  raw pipeline: " << comp.g.n << " compressed vertices, " << comp.g.NumEdges() << " edges; lean "
            << g.n << ", " << g.NumEdges() << "; raw slots " << raw.baseN << " vs lean " << lg.rawSlots << "\n";
  CHECK_EQUAL(raw.baseN, lg.rawSlots, ());
  std::vector<uint32_t> rawToComp(raw.g.n, kNone);
  for (uint32_t cv = 0; cv < comp.g.n; ++cv)
    rawToComp[comp.origId[cv]] = cv;
  g.BuildReverse();
  Scratch scL(g.n), scC(comp.g.n);
  std::mt19937_64 rng(FLAGS_seed);
  std::uniform_int_distribution<uint32_t> pick(0, lg.baseN - 1);
  uint32_t run = 0, same = 0, bothInf = 0;
  double maxDiff = 0;
  std::vector<double> lms;
  for (int attempt = 0; run < FLAGS_verify_pairs && attempt < 1000000; ++attempt)
  {
    uint32_t const a = pick(rng), b = pick(rng);
    uint32_t const ra = lg.origId[a], rb = lg.origId[b];
    if (rawToComp[ra] == kNone || rawToComp[rb] == kNone)
      continue;
    ++run;
    auto const rc = BidirectionalDijkstra(comp.g, rawToComp[ra], rawToComp[rb], scC);
    auto const tq = Clock::now();
    auto const rl = OverlayQuery(g, part, ov, a, b, scL);
    lms.push_back(Since(tq) * 1000);
    if (rc.weight >= kInf && rl.weight >= kInf)
      ++bothInf, ++same;
    else if (Same(rc.weight, rl.weight))
      ++same;
    else
      maxDiff = std::max(maxDiff, std::abs(rc.weight - rl.weight));
  }
  std::sort(lms.begin(), lms.end());
  std::cout << "  " << run << " random pairs: " << same << " identical weights (" << bothInf
            << " unreachable on both), max |diff| " << maxDiff << " s; lean overlay median "
            << (lms.empty() ? 0 : lms[lms.size() / 2]) << " ms\n";
  return same == run ? 0 : 1;
}
