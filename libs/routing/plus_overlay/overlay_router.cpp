#include "routing/plus_overlay/overlay_router.hpp"

#include "routing/base/astar_graph.hpp"
#include "routing/base/astar_vertex_data.hpp"
#include "routing/cross_mwm_graph.hpp"
#include "routing/index_graph.hpp"
#include "routing/index_graph_starter.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"
#include "routing/world_graph.hpp"

#include "base/logging.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace plus_overlay
{
namespace
{
std::mutex g_reportMutex;
EngineReport g_report;
}  // namespace

void SetLastEngineReport(EngineReport const & r)
{
  std::lock_guard<std::mutex> lock(g_reportMutex);
  g_report = r;
}

EngineReport GetLastEngineReport()
{
  std::lock_guard<std::mutex> lock(g_reportMutex);
  return g_report;
}

namespace
{
using routing::Segment;
using Key = uint64_t;  // map index << 32 | overlay vertex
constexpr Key kNoKey = std::numeric_limits<Key>::max();

Key MakeKey(uint32_t m, uint32_t v) { return (uint64_t(m) << 32) | v; }
uint32_t MapOf(Key k) { return static_cast<uint32_t>(k >> 32); }
uint32_t LocalOf(Key k) { return static_cast<uint32_t>(k); }

// Raw keys: a real segment of a map without a usable overlay file (crossed approximately, see FindPath).
constexpr Key kRawBit = Key(1) << 62;
bool IsRaw(Key k) { return k != kNoKey && (k & kRawBit) != 0; }

enum EdgeKind : uint8_t
{
  kBaseEdge = 0,  // a chain of real segments (overlay) or one real edge
  kTwinEdge = 1,  // the same physical segment in the next map
  kLeapEdge = 2,  // across a map without overlay: enter -> exit on CoMaps' border-to-border time
};

struct BaseEdge
{
  Key to;
  double w;
  uint8_t kind;
  double pen = 0;    // penalty part of w (seconds), see dynamic_edges.hpp
  bool dyn = false;  // time-dependent: not in any clique, relaxed even inside the cell
};

struct Label
{
  double d = kInf;
  double pen = 0;  // penalty part of d: d - pen is the travel time, which conditional access is checked at
  Key parent = kNoKey;
  uint8_t level = 0;
  bool done = false;
};


using Heap = std::priority_queue<std::pair<double, Key>, std::vector<std::pair<double, Key>>, std::greater<>>;

struct MapRt
{
  routing::NumMwmId id = 0;
  std::string path;
  int64_t version = 0;
  uint32_t avoidMask = 0;
  std::string avoidPath;
  bool tried = false;
  std::unique_ptr<MappedOverlay> ovl;
  std::string error;
  std::vector<uint32_t> exits, enters;  // sorted overlay vertices that are border transitions
};
}  // namespace

struct OverlayRouter::Impl
{
  uint64_t paramsHash = 0;
  std::vector<MapRt> maps;
  std::unordered_map<routing::NumMwmId, uint32_t> mapIdx;
  bool twinsBuilt = false;
  std::unordered_map<Key, std::vector<Key>> twinOut;  // exit -> enters of other maps
  std::unordered_map<Key, std::vector<Key>> twinIn;   // enter -> exits of other maps

  // Per query: search forward only (arrival times known), because a map has time-dependent edges.
  bool forwardOnly = false;

  // Per query.
  routing::IndexGraphStarter * starter = nullptr;
  std::vector<routing::IndexGraph *> graphs;
  uint64_t baseScans = 0;
  routing::IndexGraph::SegmentEdgeListT edges, chainEdges;

  MappedOverlay const * Ovl(uint32_t m)
  {
    auto & mr = maps[m];
    if (!mr.tried)
    {
      mr.tried = true;
      bool damaged = false;
      mr.ovl = MappedOverlay::Open(mr.path, mr.version, paramsHash, &mr.error, &damaged);
      // A damaged file would make every route through this map use the normal search until something
      // rebuilds it: delete it, so the app's background job does (it builds whatever is missing).
      if (damaged)
      {
        LOG(LWARNING, ("Comaps++ overlay: damaged file deleted:", mr.path, mr.error));
        std::remove(mr.path.c_str());
      }
      if (mr.ovl && mr.avoidMask != 0 &&
          !mr.ovl->AttachAvoidCliques(mr.avoidPath, mr.avoidMask, &mr.error, &damaged))
      {
        if (damaged)
        {
          LOG(LWARNING, ("Comaps++ overlay: damaged file deleted:", mr.avoidPath, mr.error));
          std::remove(mr.avoidPath.c_str());
        }
        mr.ovl.reset();
      }
      if (mr.ovl)
      {
        for (uint32_t i = 0; i < mr.ovl->NumTransitions(); ++i)
        {
          auto const & t = mr.ovl->Transition(i);
          (t.isEnter ? mr.enters : mr.exits).push_back(t.local);
        }
        std::sort(mr.exits.begin(), mr.exits.end());
        std::sort(mr.enters.begin(), mr.enters.end());
      }
    }
    return mr.ovl.get();
  }

  // Set when the search settles a border vertex whose neighbouring map has no usable overlay: the
  // best route could pass through that map, so the overlay can't answer the query exactly.
  bool needsMissingMap = false;
  routing::NumMwmId missingMap = 0;
  std::vector<Segment> twinScratch;

  // Twins of a border vertex, found with CoMaps' own cross-map graph (exactly the joins the normal
  // search makes). Appends overlay keys; a twin in a map without a usable overlay sets needsMissingMap.
  void Twins(uint32_t m, uint32_t v, Segment const & s, bool outgoing, std::vector<BaseEdge> & out,
             std::vector<std::vector<Segment>> * chains)
  {
    auto const & set = outgoing ? maps[m].exits : maps[m].enters;
    if (!std::binary_search(set.begin(), set.end(), v))
      return;
    twinScratch.clear();
    if (!starter->GetGraph().GetCrossMwmGraph().IsTransition(s, outgoing))
    {
      // The file lists a border vertex CoMaps doesn't know: the file doesn't match the map.
      LOG(LWARNING, ("Comaps++ overlay: not a border transition in the map:", s, outgoing));
      needsMissingMap = true;
      missingMap = s.GetMwmId();
      return;
    }
    starter->GetGraph().GetTwinsInner(s, outgoing, twinScratch);
    auto const twins = twinScratch;  // KeyOfTwin may reuse the scratch list
    for (auto const & t : twins)
    {
      // A neighbour without a usable overlay is crossed approximately (raw keys and CoMaps' leaps).
      out.push_back({KeyOfTwin(t), 0.0, kTwinEdge});
      if (chains)
        chains->emplace_back();
    }
  }

  // Per query: real segments used as raw keys.
  std::vector<Segment> rawSegs;
  std::unordered_map<Segment, uint32_t> rawIdx;
  std::set<routing::NumMwmId> approxTouched;  // maps without overlay the search crossed

  Key RawKey(Segment const & s)
  {
    auto [it, inserted] = rawIdx.emplace(s, static_cast<uint32_t>(rawSegs.size()));
    if (inserted)
      rawSegs.push_back(s);
    approxTouched.insert(s.GetMwmId());
    return kRawBit | it->second;
  }
  Segment const & RawSeg(Key k) const { return rawSegs[static_cast<size_t>(k & ~kRawBit)]; }

  // Key of a real segment reached by a twin link: its overlay vertex, or a raw key if its map has no
  // usable overlay (or the segment isn't a vertex there).
  Key KeyOfTwin(Segment const & t)
  {
    auto it = mapIdx.find(t.GetMwmId());
    MappedOverlay const * o = it == mapIdx.end() ? nullptr : Ovl(it->second);
    if (o && o->HasSegment(t.GetFeatureId(), t.GetSegmentIdx()))
    {
      uint32_t const slot = o->Slot(t.GetFeatureId(), t.GetSegmentIdx(), t.IsForward());
      if (o->Kept(slot))
        return MakeKey(it->second, o->Rank(slot));
    }
    return RawKey(t);
  }

  // Edges of a raw key: twins at a border exit (outgoing) / enter (ingoing), CoMaps' cross-map leaps from
  // an enter to the map's exits (outgoing) / to an exit from the map's enters (ingoing).
  void RawEdges(Key x, bool outgoing, std::vector<BaseEdge> & out, std::vector<std::vector<Segment>> * chains)
  {
    Segment const s = RawSeg(x);
    auto & cross = starter->GetGraph().GetCrossMwmGraph();
    if (cross.IsTransition(s, outgoing))
    {
      twinScratch.clear();
      starter->GetGraph().GetTwinsInner(s, outgoing, twinScratch);
      for (auto const & t : twinScratch)
      {
        out.push_back({KeyOfTwin(t), 0.0, kTwinEdge});
        if (chains)
          chains->emplace_back();
      }
    }
    if (cross.IsTransition(s, !outgoing))
    {
      routing::CrossMwmGraph::EdgeListT leaps;
      if (outgoing)
        cross.GetOutgoingEdgeList(s, leaps);
      else
        cross.GetIngoingEdgeList(s, leaps);
      for (auto const & e : leaps)
      {
        out.push_back({RawKey(e.GetTarget()), e.GetWeight().GetIntegratedWeight(), kLeapEdge});
        if (chains)
          chains->emplace_back();
      }
    }
  }

  // A* on the starter inside one map (the world graph in single-map mode, so no twins): from |from| (fake
  // or real) to |to|. Appends the segments after |from| up to and including |to|. Restrictions are
  // checked the way CoMaps does (with the search's parents). Bounded: false if it grows too large.
  OverlayRouter::LegRefiner legRefiner;

  bool RefineInMap(Segment const & from, Segment const & to, std::vector<Segment> & out, double * cost = nullptr,
                   bool anyMap = false)
  {
    if (legRefiner)
    {
      std::vector<Segment> leg;
      double w = 0;
      if (!legRefiner(from, to, anyMap, leg, w) || leg.empty() || !(leg.back() == to))
      {
        LOG(LINFO, ("Comaps++ overlay: no leg from", from, "to", to));
        return false;
      }
      out.insert(out.end(), leg.begin() + (leg.front() == from ? 1 : 0), leg.end());
      if (cost)
        *cost = w;
      return true;
    }
    auto & st = *starter;
    auto & g = st.GetGraph();
    auto const prevMode = g.GetMode();
    g.SetMode(anyMap ? routing::WorldGraphMode::NoLeaps : routing::WorldGraphMode::SingleMwm);
    using Parents = routing::AStarGraph<Segment, routing::SegmentEdge, routing::RouteWeight>::Parents;
    Parents parents;
    st.SetAStarParents(true, parents);
    ms::LatLon const target = st.GetPoint(to, true /* front */);
    std::unordered_map<Segment, double> dist{{from, 0.0}};
    std::unordered_set<Segment> closed;
    std::priority_queue<std::pair<double, Segment>, std::vector<std::pair<double, Segment>>, std::greater<>> q;
    q.push({0.0, from});
    routing::IndexGraphStarter::EdgeListT es;
    bool found = false;
    constexpr size_t kMaxSettled = 6000000;
    while (!q.empty() && closed.size() < kMaxSettled)
    {
      Segment const x = q.top().second;
      q.pop();
      if (!closed.insert(x).second)
        continue;
      if (x == to)
      {
        found = true;
        break;
      }
      double const dx = dist[x];
      es.clear();
      routing::astar::VertexData<Segment, routing::RouteWeight> const vd(x, routing::RouteWeight(dx));
      st.GetOutgoingEdgesList(vd, es);
      ++baseScans;
      for (auto const & e : es)
      {
        Segment const & t = e.GetTarget();
        if (!routing::IndexGraphStarter::IsFakeSegment(t) && !routing::IndexGraphStarter::IsFakeSegment(to) &&
            t.GetMwmId() != to.GetMwmId())
          continue;
        double const nd = dx + e.GetWeight().GetIntegratedWeight();
        auto it = dist.find(t);
        if (it == dist.end() || nd < it->second)
        {
          dist[t] = nd;
          parents[t] = x;
          q.push({nd + st.HeuristicCostEstimate(t, target).GetIntegratedWeight(), t});
        }
      }
    }
    std::vector<Segment> rev;
    if (found)
    {
      for (Segment y = to; !(y == from); y = parents.at(y))
        rev.push_back(y);
      if (cost)
        *cost = dist[to];
    }
    st.DropAStarParents();
    g.SetMode(prevMode);
    if (!found)
    {
      return false;
    }
    out.insert(out.end(), rev.rbegin(), rev.rend());
    return true;
  }

  routing::IndexGraph & Graph(uint32_t m)
  {
    if (!graphs[m])
      graphs[m] = &starter->GetGraph().GetIndexGraph(maps[m].id);
    return *graphs[m];
  }

  Segment SegOf(uint32_t m, uint32_t local)
  {
    auto const & o = *maps[m].ovl;
    uint32_t f, s;
    bool fwd;
    o.Decode(o.SlotOf(local), f, s, fwd);
    return Segment(maps[m].id, f, s, fwd);
  }

  // Overlay map index of a real segment, or kNone if its map has no usable overlay.
  uint32_t MapOfSegment(Segment const & s)
  {
    auto it = mapIdx.find(s.GetMwmId());
    if (it == mapIdx.end() || !Ovl(it->second))
      return kNone;
    return it->second;
  }

  void BuildTwins()
  {
    if (twinsBuilt)
      return;
    twinsBuilt = true;
    // (osm way, segment index) -> enter / exit vertices of every map.
    std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, Key>>> enters, exits;
    auto tkey = [](FileTransition const & t) { return t.osmId * 1000003ULL + t.segIdx; };
    for (uint32_t m = 0; m < maps.size(); ++m)
    {
      auto const * o = Ovl(m);
      if (!o)
        continue;
      for (uint32_t i = 0; i < o->NumTransitions(); ++i)
      {
        auto const & t = o->Transition(i);
        (t.isEnter ? enters : exits)[tkey(t)].emplace_back(m, MakeKey(m, t.local));
      }
    }
    for (auto const & [k, ex] : exits)
    {
      auto it = enters.find(k);
      if (it == enters.end())
        continue;
      for (auto const & [mx, kx] : ex)
        for (auto const & [me, ke] : it->second)
          if (mx != me)
          {
            twinOut[kx].push_back(ke);
            twinIn[ke].push_back(kx);
          }
    }
  }

  uint32_t StartState(MappedOverlay const & o, uint32_t local)
  {
    auto const * ac = o.Automaton();
    if (!ac)
      return 0;
    if (o.IsCopy(local))
      return o.CopyState(local);
    return ac->Start(o.FeatureOfLocal(local));
  }

  // The overlay target of stepping from (state, feature) onto overlay vertex |base| of |feature t|.
  // kNone if the step is forbidden by a via-way restriction.
  uint32_t StepTarget(MappedOverlay const & o, uint32_t state, uint32_t fromFeature, uint32_t base, uint32_t toFeature,
                      bool inner)
  {
    auto const * ac = o.Automaton();
    if (!ac)
      return base;
    uint32_t next = state;
    if (inner)
      next = ac->Start(toFeature);
    else if (toFeature != fromFeature)
      next = ac->Step(state, toFeature);
    if (ac->Terminal(next))
      return kNone;
    if (ac->Depth(next) >= 2)
      return o.CopyId(next, base);
    return base;
  }

  // Forward chain from real segment |s| in map |m| to the next overlay vertex. |segs| (optional)
  // receives the segments after |s| up to and including the overlay vertex's segment.
  // false if the chain dead-ends.
  bool WalkForward(uint32_t m, Segment s, double & w, uint32_t & kept, bool & inner, std::vector<Segment> * segs,
                   double * pen = nullptr)
  {
    auto const & o = *maps[m].ovl;
    inner = false;
    for (int guard = 0; guard < 1000000; ++guard)
    {
      if (!o.HasSegment(s.GetFeatureId(), s.GetSegmentIdx()))
        return false;
      uint32_t const slot = o.Slot(s.GetFeatureId(), s.GetSegmentIdx(), s.IsForward());
      if (o.Kept(slot))
      {
        kept = o.Rank(slot);
        return true;
      }
      chainEdges.clear();
      ++baseScans;
      Graph(m).GetEdgeList(s, true /* isOutgoing */, true /* useRoutingOptions */, chainEdges);
      if (chainEdges.size() != 1)
        return false;
      inner = true;
      w += chainEdges[0].GetWeight().GetIntegratedWeight();
      if (pen)
        *pen += PenaltyUnits(chainEdges[0].GetWeight()) * kPenaltyUnitS;
      s = chainEdges[0].GetTarget();
      if (segs)
        segs->push_back(s);
    }
    return false;
  }

  // Backward chain: from real segment |s| to the previous overlay vertex. |segs| (optional) receives
  // the segments from just after that vertex up to |s|, in travel order.
  bool WalkBackward(uint32_t m, Segment s, double & w, uint32_t & kept, bool & inner, std::vector<Segment> * segs)
  {
    auto const & o = *maps[m].ovl;
    inner = false;
    std::vector<Segment> rev;
    for (int guard = 0; guard < 1000000; ++guard)
    {
      if (!o.HasSegment(s.GetFeatureId(), s.GetSegmentIdx()))
        return false;
      uint32_t const slot = o.Slot(s.GetFeatureId(), s.GetSegmentIdx(), s.IsForward());
      if (o.Kept(slot))
      {
        kept = o.Rank(slot);
        if (segs)
          segs->assign(rev.rbegin(), rev.rend());
        return true;
      }
      rev.push_back(s);
      chainEdges.clear();
      ++baseScans;
      Graph(m).GetEdgeList(s, false /* isOutgoing */, true /* useRoutingOptions */, chainEdges);
      if (chainEdges.size() != 1)
        return false;
      inner = true;
      w += chainEdges[0].GetWeight().GetIntegratedWeight();
      s = chainEdges[0].GetTarget();
    }
    return false;
  }

  // Overlay out-edges of |x|. |chains| (optional) gets each edge's segments after x's segment.
  // |search|: also the edges no clique holds (time-dependent steps, checked at |travelTime| when given).
  // Without it, exactly the edges the cliques were computed from.
  void OutEdges(Key x, std::vector<BaseEdge> & out, std::vector<std::vector<Segment>> * chains,
                bool search = false, double const * travelTime = nullptr)
  {
    out.clear();
    if (chains)
      chains->clear();
    if (IsRaw(x))
    {
      RawEdges(x, true /* outgoing */, out, chains);
      return;
    }
    uint32_t const m = MapOf(x), v = LocalOf(x);
    auto const & o = *maps[m].ovl;
    uint32_t const base = o.IsCopy(v) ? o.CopyOrig(v) : v;
    Segment const s = SegOf(m, base);
    uint32_t const fx = s.GetFeatureId();
    uint32_t const state = StartState(o, v);
    bool const special = o.IsSpecial(base);
    bool const timeDep = special && o.HasTimeDependentEdges();
    auto const & access = Graph(m).GetRoadAccess();
    edges.clear();
    ++baseScans;
    if (timeDep && search && travelTime)
      Graph(m).GetEdgeList(routing::astar::VertexData<Segment, routing::RouteWeight>(s, routing::RouteWeight(*travelTime)),
                           true /* isOutgoing */, true /* useRoutingOptions */, edges);
    else
      Graph(m).GetEdgeList(s, true /* isOutgoing */, true /* useRoutingOptions */, edges);
    auto const list = edges;  // WalkForward reuses the scratch lists
    std::vector<Segment> segs;
    for (auto const & e : list)
    {
      Segment t = e.GetTarget();
      bool const dyn = timeDep && IsTimeDependentEdge(access, s, t);
      if (dyn && !search)
        continue;
      double w = e.GetWeight().GetIntegratedWeight();
      double pen = PenaltyUnits(e.GetWeight()) * kPenaltyUnitS;
      segs.assign(1, t);
      uint32_t kept;
      bool inner;
      if (!WalkForward(m, t, w, kept, inner, chains ? &segs : nullptr, &pen))
        continue;
      if (kept == base && !inner && t == s)
        continue;
      Segment const kseg = chains ? segs.back() : SegOf(m, kept);
      if (kseg == s)
        continue;  // chain back to itself
      uint32_t const target = StepTarget(o, state, fx, kept, kseg.GetFeatureId(), inner);
      if (target == kNone)
        continue;
      out.push_back({MakeKey(m, target), w, kBaseEdge, pen, dyn});
      if (chains)
        chains->push_back(segs);
    }
    if (!o.IsCopy(v))
      Twins(m, v, s, true /* outgoing */, out, chains);
  }

  // Overlay in-edges of |x| (each as the edge from the predecessor to x), as OutEdges with |search| and no
  // time: the bidirectional search runs only when no map has time-dependent edges.
  void InEdges(Key x, std::vector<BaseEdge> & out)
  {
    out.clear();
    if (IsRaw(x))
    {
      RawEdges(x, false /* outgoing */, out, nullptr);
      return;
    }
    uint32_t const m = MapOf(x), v = LocalOf(x);
    auto const & o = *maps[m].ovl;
    uint32_t const base = o.IsCopy(v) ? o.CopyOrig(v) : v;
    Segment const s = SegOf(m, base);
    uint32_t const fx = s.GetFeatureId();
    edges.clear();
    ++baseScans;
    Graph(m).GetEdgeList(s, false /* isOutgoing */, true /* useRoutingOptions */, edges);
    auto const list = edges;
    for (auto const & e : list)
    {
      Segment p = e.GetTarget();
      if (o.HasTimeDependentEdges() && IsTimeDependentEdge(Graph(m).GetRoadAccess(), p, s))
        continue;
      double w = e.GetWeight().GetIntegratedWeight();
      uint32_t kept;
      bool inner;
      if (!WalkBackward(m, p, w, kept, inner, nullptr))
        continue;
      Segment const pseg = SegOf(m, kept);
      if (pseg == s)
        continue;
      uint32_t const fp = pseg.GetFeatureId();
      auto consider = [&](uint32_t cand)
      {
        uint32_t const st = StartState(o, cand);
        if (StepTarget(o, st, fp, base, fx, inner) == v)
          out.push_back({MakeKey(m, cand), w, false});
      };
      consider(kept);
      if (auto const * copies = o.CopiesOf(kept))
        for (uint32_t c : *copies)
          consider(c);
    }
    if (!o.IsCopy(v))
      Twins(m, v, s, false /* outgoing */, out, nullptr);
  }

  // Search state of one query.
  std::vector<std::vector<std::vector<uint32_t>>> endCells;  // [map][level] sorted cells holding an end
  std::vector<uint8_t> mapHasEnd;

  size_t QueryLevel(Key x)
  {
    uint32_t const m = MapOf(x), v = LocalOf(x);
    auto const & o = *maps[m].ovl;
    size_t const levels = o.Levels();
    // The highest level whose cell holds no end and isn't open (both hold for the cell's parents too).
    for (size_t li = levels; li-- > 0;)
    {
      uint32_t const c = o.Cell(li, v);
      if (o.IsOpen(li, c))
        continue;
      if (mapHasEnd[m])
      {
        auto const & ec = endCells[m][li];
        if (std::binary_search(ec.begin(), ec.end(), c))
          continue;
      }
      return li + 1;
    }
    return 0;
  }

  template <class Relax>
  void ScanF(Key x, double d, double pen, Relax && relax, std::vector<BaseEdge> & tmp)
  {
    double const travelTime = d - pen;
    if (IsRaw(x))
    {
      OutEdges(x, tmp, nullptr);
      for (auto const & e : tmp)
        relax(e.to, d + e.w, pen + e.pen, x, 0);
      return;
    }
    size_t const lvl = QueryLevel(x);
    OutEdges(x, tmp, nullptr, true /* search */, &travelTime);
    if (lvl == 0)
    {
      for (auto const & e : tmp)
        relax(e.to, d + e.w, pen + e.pen, x, 0);
      return;
    }
    size_t const li = lvl - 1;
    uint32_t const m = MapOf(x), v = LocalOf(x);
    auto const & o = *maps[m].ovl;
    uint32_t const c = o.Cell(li, v);
    if (o.IsEntry(li, v))
      o.ForEachFromEntry(li, c, v, [&](uint32_t exit, float w, uint32_t k)
                         { relax(MakeKey(m, exit), d + w, pen + k * kPenaltyUnitS, x, static_cast<uint8_t>(lvl)); });
    for (auto const & e : tmp)
      if (e.dyn || MapOf(e.to) != m || o.Cell(li, LocalOf(e.to)) != c)
        relax(e.to, d + e.w, pen + e.pen, x, 0);
  }

  template <class Relax>
  void ScanB(Key x, double d, Relax && relax, std::vector<BaseEdge> & tmp)
  {
    if (IsRaw(x))
    {
      InEdges(x, tmp);
      for (auto const & e : tmp)
        relax(e.to, d + e.w, x, 0);
      return;
    }
    size_t const lvl = QueryLevel(x);
    InEdges(x, tmp);
    if (lvl == 0)
    {
      for (auto const & e : tmp)
        relax(e.to, d + e.w, x, 0);
      return;
    }
    size_t const li = lvl - 1;
    uint32_t const m = MapOf(x), v = LocalOf(x);
    auto const & o = *maps[m].ovl;
    uint32_t const c = o.Cell(li, v);
    if (o.IsExit(li, v))
      o.ForEachToExit(li, c, v, [&](uint32_t entry, float w)
                      { relax(MakeKey(m, entry), d + w, x, static_cast<uint8_t>(lvl)); });
    for (auto const & e : tmp)
      if (MapOf(e.to) != m || o.Cell(li, LocalOf(e.to)) != c)
        relax(e.to, d + e.w, x, 0);
  }

  // Unpacks hop u -> v found at |level| into segments after u's segment (appended to |out|).
  bool UnpackHop(Key u, Key v, uint8_t level, std::vector<Segment> & out)
  {
    if (level == 0)
    {
      std::vector<BaseEdge> es;
      std::vector<std::vector<Segment>> chains;
      OutEdges(u, es, &chains, true /* search: the hop may be a time-dependent step */);
      int best = -1;
      for (size_t i = 0; i < es.size(); ++i)
        if (es[i].to == v && (best < 0 || es[i].w < es[best].w))
          best = static_cast<int>(i);
      if (best < 0)
        return false;
      // A twin link is the same physical segment in the next map: CoMaps lists it once.
      if (es[best].kind == kLeapEdge)
        return RefineInMap(RawSeg(u), RawSeg(v), out);
      if (es[best].kind != kTwinEdge)
        out.insert(out.end(), chains[best].begin(), chains[best].end());
      return true;
    }
    size_t const li = level - 1;
    uint32_t const m = MapOf(u);
    if (MapOf(v) != m)
      return false;
    auto const & o = *maps[m].ovl;
    uint32_t const c = o.Cell(li, LocalOf(u));
    std::unordered_map<Key, Label> lab;
    Heap heap;
    lab[u].d = 0;
    heap.push({0.0, u});
    std::vector<BaseEdge> tmp;
    auto relax = [&](Key y, double d, Key par, uint8_t lvl)
    {
      auto & L = lab[y];
      if (d < L.d)
      {
        L.d = d;
        L.parent = par;
        L.level = lvl;
        heap.push({d, y});
      }
    };
    bool found = false;
    while (!heap.empty())
    {
      auto const [d, x] = heap.top();
      heap.pop();
      auto & lx = lab[x];
      if (lx.done)
        continue;
      lx.done = true;
      if (x == v)
      {
        found = true;
        break;
      }
      uint32_t const xv = LocalOf(x);
      OutEdges(x, tmp, nullptr);
      if (li == 0)
      {
        for (auto const & e : tmp)
          if (MapOf(e.to) == m && o.Cell(0, LocalOf(e.to)) == c)
            relax(e.to, d + e.w, x, 0);
        continue;
      }
      uint32_t const cc = o.Cell(li - 1, xv);
      if (o.IsEntry(li - 1, xv))
        o.ForEachFromEntry(li - 1, cc, xv, [&](uint32_t exit, float w)
                           { relax(MakeKey(m, exit), d + w, x, static_cast<uint8_t>(li)); });
      if (o.IsExit(li - 1, xv))
        for (auto const & e : tmp)
          if (MapOf(e.to) == m && o.Cell(li, LocalOf(e.to)) == c && o.Cell(li - 1, LocalOf(e.to)) != cc)
            relax(e.to, d + e.w, x, 0);
    }
    if (!found)
      return false;
    std::vector<std::tuple<Key, Key, uint8_t>> sub;
    for (Key y = v; y != u; y = lab[y].parent)
      sub.emplace_back(lab[y].parent, y, lab[y].level);
    std::reverse(sub.begin(), sub.end());
    for (auto const & [a, b, l] : sub)
      if (!UnpackHop(a, b, l, out))
        return false;
    return true;
  }

  // Dijkstra over the starter's fake vertices from the start (forward) or the finish (backward).
  // |real| gets the first real segments reached with their costs; |parent| the fake predecessors.
  // |realPen| (optional) gets the penalty part of each real segment's cost (dynamic_edges.hpp).
  void FakeSearch(bool forward, std::map<Segment, double> & real, std::map<Segment, Segment> & parent,
                  double & direct, Segment & directLast, std::map<Segment, double> * realPen = nullptr)
  {
    std::map<Segment, double> pen;
    auto & st = *starter;
    auto const src = forward ? st.GetStartSegment() : st.GetFinishSegment();
    auto const dst = forward ? st.GetFinishSegment() : st.GetStartSegment();
    std::map<Segment, double> dist{{src, 0.0}};
    std::set<std::pair<double, Segment>> pq{{0.0, src}};
    std::set<Segment> done;
    routing::IndexGraphStarter::EdgeListT es;
    while (!pq.empty())
    {
      auto const [d, x] = *pq.begin();
      pq.erase(pq.begin());
      if (!done.insert(x).second)
        continue;
      if (x == dst)
      {
        if (d < direct)
        {
          direct = d;
          directLast = x;
        }
        continue;
      }
      if (!routing::IndexGraphStarter::IsFakeSegment(x))
      {
        auto it = real.find(x);
        if (it == real.end() || d < it->second)
        {
          real[x] = d;
          if (realPen)
            (*realPen)[x] = pen[x];
        }
        continue;
      }
      es.clear();
      routing::astar::VertexData<Segment, routing::RouteWeight> const vd(x, routing::RouteWeight(d));
      if (forward)
        st.GetOutgoingEdgesList(vd, es);
      else
        st.GetIngoingEdgesList(vd, es);
      for (auto const & e : es)
      {
        double const nd = d + e.GetWeight().GetIntegratedWeight();
        auto it = dist.find(e.GetTarget());
        if (it == dist.end() || nd < it->second)
        {
          dist[e.GetTarget()] = nd;
          pen[e.GetTarget()] = pen[x] + PenaltyUnits(e.GetWeight()) * kPenaltyUnitS;
          parent[e.GetTarget()] = x;
          pq.insert({nd, e.GetTarget()});
        }
      }
    }
  }
};

OverlayRouter::OverlayRouter(uint64_t paramsHash) : m_impl(std::make_unique<Impl>())
{
  m_impl->paramsHash = paramsHash;
}

OverlayRouter::~OverlayRouter() = default;

void OverlayRouter::SetLegRefiner(LegRefiner refiner)
{
  m_impl->legRefiner = std::move(refiner);
}

void OverlayRouter::SetMaps(std::vector<OverlayMap> const & maps)
{
  // Keep already opened files of maps that are unchanged.
  std::vector<MapRt> next;
  for (auto const & om : maps)
  {
    MapRt mr;
    mr.id = om.id;
    mr.path = om.path;
    mr.version = om.mwmVersion;
    mr.avoidMask = om.avoidMask;
    mr.avoidPath = om.avoidPath;
    auto it = m_impl->mapIdx.find(om.id);
    if (it != m_impl->mapIdx.end())
    {
      auto & old = m_impl->maps[it->second];
      // A map whose file was missing or stale is tried again (its overlay may have been built since).
      if (old.path == om.path && old.version == om.mwmVersion && old.avoidMask == om.avoidMask &&
          old.avoidPath == om.avoidPath && old.ovl)
        mr = std::move(old);
    }
    next.push_back(std::move(mr));
  }
  m_impl->maps = std::move(next);
  m_impl->mapIdx.clear();
  for (uint32_t i = 0; i < m_impl->maps.size(); ++i)
    m_impl->mapIdx[m_impl->maps[i].id] = i;
  m_impl->twinsBuilt = false;
  m_impl->twinOut.clear();
  m_impl->twinIn.clear();
}

bool OverlayRouter::AllMapsReady(std::vector<routing::NumMwmId> * missing)
{
  bool all = true;
  for (uint32_t m = 0; m < m_impl->maps.size(); ++m)
  {
    if (!m_impl->Ovl(m))
    {
      all = false;
      if (missing)
        missing->push_back(m_impl->maps[m].id);
    }
  }
  return all;
}

bool OverlayRouter::TwinOfExit(routing::Segment const & exitSegment, routing::Segment & twin)
{
  auto & I = *m_impl;
  I.BuildTwins();
  uint32_t const m = I.MapOfSegment(exitSegment);
  if (m == kNone)
    return false;
  auto const & o = *I.maps[m].ovl;
  if (!o.HasSegment(exitSegment.GetFeatureId(), exitSegment.GetSegmentIdx()))
    return false;
  uint32_t const slot = o.Slot(exitSegment.GetFeatureId(), exitSegment.GetSegmentIdx(), exitSegment.IsForward());
  if (!o.Kept(slot))
    return false;
  auto it = I.twinOut.find(MakeKey(m, o.Rank(slot)));
  if (it == I.twinOut.end() || it->second.empty())
    return false;
  twin = I.SegOf(MapOf(it->second[0]), LocalOf(it->second[0]));
  return true;
}

OverlayRouter::Result OverlayRouter::FindPath(routing::IndexGraphStarter & starter,
                                              std::vector<routing::Segment> & path, Stats & stats)
{
  auto const t0 = std::chrono::steady_clock::now();
  auto & I = *m_impl;
  path.clear();
  stats = {};
  I.starter = &starter;
  I.graphs.assign(I.maps.size(), nullptr);
  I.baseScans = 0;
  I.rawSegs.clear();
  I.rawIdx.clear();
  I.approxTouched.clear();
  I.needsMissingMap = false;
  // Conditional access is checked at the time the route gets there, which only a forward search knows.
  I.forwardOnly = false;
  for (uint32_t m = 0; m < I.maps.size(); ++m)
    if (auto const * o = I.Ovl(m); o && o->HasTimeDependentEdges())
      I.forwardOnly = true;
  stats.forwardOnly = I.forwardOnly;
  auto finish = [&](Result r, std::string const & why)
  {
    stats.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    stats.baseScans = I.baseScans;
    stats.reason = why;
    I.starter = nullptr;
    return r;
  };

  // 1. Fake parts of the starter.
  using Parents = routing::AStarGraph<Segment, routing::SegmentEdge, routing::RouteWeight>::Parents;
  Parents pf, pb;
  starter.SetAStarParents(true, pf);
  starter.SetAStarParents(false, pb);
  std::map<Segment, double> srcReal, tgtReal;
  std::map<Segment, Segment> parF, parB;
  double direct = kInf;
  Segment dummy;
  std::map<Segment, double> srcPen;
  I.FakeSearch(true, srcReal, parF, direct, dummy, &srcPen);
  double directB = kInf;
  I.FakeSearch(false, tgtReal, parB, directB, dummy);
  // The backward search ran on ingoing weights, which for fake segments differ slightly from the
  // forward weights CoMaps scores a route with: re-cost every finish part forward.
  {
    routing::IndexGraphStarter::EdgeListT es;
    for (auto & [y, cost] : tgtReal)
    {
      double fw = 0;
      bool ok = true;
      Segment x = y;
      for (int guard = 0; guard < 100000 && x != starter.GetFinishSegment(); ++guard)
      {
        auto it = parB.find(x);
        if (it == parB.end())
        {
          ok = false;
          break;
        }
        es.clear();
        routing::astar::VertexData<Segment, routing::RouteWeight> const vd(x, routing::RouteWeight(0));
        starter.GetOutgoingEdgesList(vd, es);
        double w = kInf;
        for (auto const & e : es)
          if (e.GetTarget() == it->second)
            w = std::min(w, e.GetWeight().GetIntegratedWeight());
        if (w >= kInf)
        {
          ok = false;
          break;
        }
        fw += w;
        x = it->second;
      }
      if (ok)
        cost = fw;
    }
  }
  starter.DropAStarParents();
  if (srcReal.empty() && tgtReal.empty() && direct >= kInf)
    return finish(Result::NoRoute, "");

  auto lap = [&, last = t0](double & into) mutable
  {
    auto const now = std::chrono::steady_clock::now();
    into = std::chrono::duration<double, std::milli>(now - last).count();
    last = now;
  };
  lap(stats.fakeMs);
  // 2. Seeds: walk chains from the real segments to overlay vertices.
  struct Seed
  {
    Segment real;
    double cost;
    std::vector<Segment> chain;  // real segments from |real| to the overlay vertex (both included)
    bool leap = false;           // a border transition of an end's map without overlay (cost estimated)
    double pen = 0;              // penalty part of cost (start seeds; dynamic_edges.hpp)
  };
  std::unordered_map<Key, Seed> src, tgt;
  // Ends in maps without a usable overlay: like CoMaps' long-route mode, the end connects to that map's
  // border transitions with an estimated time (straight line at top speed, scaled by kEndLegFactor since
  // roads inside a map take several times longer). The search only picks the corridor: the part near such
  // an end is then found by CoMaps' own search from a handover point on the overlay route (see below).
  constexpr double kEndLegFactor = 4.0;
  std::set<routing::NumMwmId> rawStart, rawFinish;
  for (auto const & [s, c] : srcReal)
    if (I.MapOfSegment(s) == kNone)
      rawStart.insert(s.GetMwmId());
  for (auto const & [s, c] : tgtReal)
    if (I.MapOfSegment(s) == kNone)
      rawFinish.insert(s.GetMwmId());
  auto & worldGraph = starter.GetGraph();
  ms::LatLon const startLL = starter.GetStartJunction().GetLatLon();
  ms::LatLon const finishLL = starter.GetFinishJunction().GetLatLon();
  // Both ends in the same map without overlay: the in-map route is one candidate; routes that leave the
  // map are compared with it after the search.
  // Both ends in the same map without overlay: no overlay can shorten the search much, so this is CoMaps'
  // own joint search end to end (across maps, so routes that leave the map and come back are found).
  for (auto const id : rawStart)
  {
    if (!rawFinish.count(id))
      continue;
    path.assign(1, starter.GetStartSegment());
    if (!I.RefineInMap(starter.GetStartSegment(), starter.GetFinishSegment(), path, &stats.weight,
                       true /* anyMap */))
      return finish(Result::NoRoute, "");
    std::set<routing::NumMwmId> usedMaps;
    for (auto const & sg : path)
      if (!routing::IndexGraphStarter::IsFakeSegment(sg))
        usedMaps.insert(sg.GetMwmId());
    stats.mapsUsed = usedMaps.size();
    for (auto const used : usedMaps)
    {
      auto it = I.mapIdx.find(used);
      if (it == I.mapIdx.end() || !I.Ovl(it->second))
        stats.approxMaps.push_back(used);
    }
    return finish(Result::Ok, "");
  }
  for (auto const id : rawStart)
    worldGraph.GetCrossMwmGraph().ForEachTransition(id, false /* isEnter */, [&](Segment const & e)
    {
      double const w =
          kEndLegFactor * worldGraph.HeuristicCostEstimate(startLL, starter.GetPoint(e, true)).GetIntegratedWeight();
      Key const k = I.RawKey(e);
      auto it = src.find(k);
      if (it == src.end() || w < it->second.cost)
        src[k] = {e, w, {}, true};
    });
  for (auto const id : rawFinish)
    worldGraph.GetCrossMwmGraph().ForEachTransition(id, true /* isEnter */, [&](Segment const & e)
    {
      double const w =
          kEndLegFactor * worldGraph.HeuristicCostEstimate(starter.GetPoint(e, true), finishLL).GetIntegratedWeight();
      Key const k = I.RawKey(e);
      auto it = tgt.find(k);
      if (it == tgt.end() || w < it->second.cost)
        tgt[k] = {e, w, {}, true};
    });
  for (auto const & [s, c] : srcReal)
  {
    uint32_t const m = I.MapOfSegment(s);
    if (m == kNone)
      continue;
    double w = c;
    double pen = srcPen[s];
    uint32_t kept;
    bool inner;
    std::vector<Segment> chain{s};
    if (!I.WalkForward(m, s, w, kept, inner, &chain, &pen))
      continue;
    Key const k = MakeKey(m, kept);
    auto it = src.find(k);
    if (it == src.end() || w < it->second.cost)
      src[k] = {s, w, chain, false, pen};
  }
  for (auto const & [s, c] : tgtReal)
  {
    uint32_t const m = I.MapOfSegment(s);
    if (m == kNone)
      continue;
    double w = c;
    uint32_t kept;
    bool inner;
    std::vector<Segment> chain;  // segments after the overlay vertex up to and including |s| (empty if |s| is it)
    if (!I.WalkBackward(m, s, w, kept, inner, &chain))
      continue;
    auto const & o = *I.maps[m].ovl;
    auto add = [&](uint32_t local)
    {
      Key const k = MakeKey(m, local);
      auto it = tgt.find(k);
      if (it == tgt.end() || w < it->second.cost)
        tgt[k] = {s, w, chain, false};
    };
    add(kept);
    if (auto const * copies = o.CopiesOf(kept))
      for (uint32_t cpy : *copies)
        add(cpy);
  }

  // A target real segment on a source chain: a route that never reaches an overlay vertex.
  double bestDirect = direct;
  std::vector<Segment> directReal;
  Segment directSrc, directTgt;
  for (auto const & [k, sd] : src)
  {
    if (sd.leap)
      continue;
    double acc = srcReal.at(sd.real);
    for (size_t i = 0; i < sd.chain.size(); ++i)
    {
      if (i > 0)
      {
        // Weight of chain[i-1] -> chain[i].
        I.edges.clear();
        I.Graph(I.MapOfSegment(sd.chain[i - 1])).GetEdgeList(sd.chain[i - 1], true, true, I.edges);
        double w = kInf;
        for (auto const & e : I.edges)
          if (e.GetTarget() == sd.chain[i])
            w = std::min(w, e.GetWeight().GetIntegratedWeight());
        acc += w;
      }
      auto it = tgtReal.find(sd.chain[i]);
      if (it != tgtReal.end() && acc + it->second < bestDirect)
      {
        bestDirect = acc + it->second;
        directReal.assign(sd.chain.begin(), sd.chain.begin() + i + 1);
        directSrc = sd.real;
        directTgt = sd.chain[i];
      }
    }
  }

  lap(stats.seedMs);
  // 3. Overlay search.
  I.endCells.assign(I.maps.size(), {});
  I.mapHasEnd.assign(I.maps.size(), 0);
  auto noteEnd = [&](Key k)
  {
    if (IsRaw(k))
      return;
    uint32_t const m = MapOf(k);
    auto const & o = *I.maps[m].ovl;
    if (I.endCells[m].empty())
      I.endCells[m].resize(o.Levels());
    I.mapHasEnd[m] = 1;
    for (size_t li = 0; li < o.Levels(); ++li)
      I.endCells[m][li].push_back(o.Cell(li, LocalOf(k)));
  };
  for (auto const & [k, sd] : src)
    noteEnd(k);
  for (auto const & [k, sd] : tgt)
    noteEnd(k);
  for (auto & per : I.endCells)
    for (auto & v : per)
    {
      std::sort(v.begin(), v.end());
      v.erase(std::unique(v.begin(), v.end()), v.end());
    }

  std::unordered_map<Key, Label> F, B;
  Heap hf, hb;
  for (auto const & [k, sd] : src)
  {
    F[k].d = sd.cost;
    F[k].pen = sd.pen;
    hf.push({sd.cost, k});
  }
  for (auto const & [k, sd] : tgt)
  {
    B[k].d = sd.cost;
    hb.push({sd.cost, k});
  }
  double mu = kInf;
  Key meet = kNoKey;
  for (auto const & [k, sd] : tgt)
  {
    auto it = F.find(k);
    if (it != F.end() && it->second.d + sd.cost < mu)
    {
      mu = it->second.d + sd.cost;
      meet = k;
    }
  }
  auto relaxF = [&](Key y, double d, double pen, Key par, uint8_t lvl)
  {
    auto & L = F[y];
    if (d < L.d)
    {
      L.d = d;
      L.pen = pen;
      L.parent = par;
      L.level = lvl;
      hf.push({d, y});
      auto it = B.find(y);
      if (it != B.end() && d + it->second.d < mu)
      {
        mu = d + it->second.d;
        meet = y;
      }
    }
  };
  auto relaxB = [&](Key y, double d, Key par, uint8_t lvl)
  {
    auto & L = B[y];
    if (d < L.d)
    {
      L.d = d;
      L.parent = par;
      L.level = lvl;
      hb.push({d, y});
      auto it = F.find(y);
      if (it != F.end() && d + it->second.d < mu)
      {
        mu = d + it->second.d;
        meet = y;
      }
    }
  };
  std::vector<BaseEdge> tmp;
  auto top = [](Heap const & h) { return h.empty() ? kInf : h.top().first; };
  // Forward only: the targets' finish costs stay the only backward labels, so mu is checked as the forward
  // search reaches a target, and min(target cost) bounds what any target still adds.
  double const minTarget = top(hb);
  while (!hf.empty() || (!I.forwardOnly && !hb.empty()))
  {
    double const tf = top(hf), tb = I.forwardOnly ? minTarget : top(hb);
    if (tf + tb >= mu || tf >= kInf || (!I.forwardOnly && tb >= kInf))
      break;
    if (I.forwardOnly || tf <= tb)
    {
      auto const [d, x] = hf.top();
      hf.pop();
      auto & lx = F[x];
      if (lx.done || d > lx.d)
        continue;
      lx.done = true;
      ++stats.settled;
      I.ScanF(x, d, lx.pen, relaxF, tmp);
      if (I.needsMissingMap)
      {
        stats.hasMissingMap = true;
        stats.missingMap = I.missingMap;
        return finish(Result::NotApplicable, "the route may pass through a map without an overlay");
      }
    }
    else
    {
      auto const [d, x] = hb.top();
      hb.pop();
      auto & lx = B[x];
      if (lx.done || d > lx.d)
        continue;
      lx.done = true;
      ++stats.settled;
      I.ScanB(x, d, relaxB, tmp);
      if (I.needsMissingMap)
      {
        stats.hasMissingMap = true;
      stats.missingMap = I.missingMap;
        return finish(Result::NotApplicable, "the route may pass through a map without an overlay");
      }
    }
  }

  lap(stats.searchMs);
  // 4. Assemble the starter path.
  auto fakePrefix = [&](Segment const & firstReal)
  {
    std::vector<Segment> pre;
    for (Segment x = parF.at(firstReal);; x = parF.at(x))
    {
      pre.push_back(x);
      if (x == starter.GetStartSegment())
        break;
    }
    std::reverse(pre.begin(), pre.end());
    return pre;
  };
  auto fakeSuffix = [&](Segment const & lastReal)
  {
    std::vector<Segment> suf;
    for (Segment x = parB.at(lastReal);; x = parB.at(x))
    {
      suf.push_back(x);
      if (x == starter.GetFinishSegment())
        break;
    }
    return suf;
  };

  if (mu >= kInf && bestDirect >= kInf)
    return finish(Result::NoRoute, "");

  std::set<routing::NumMwmId> used;
  if (bestDirect <= mu)
  {
    stats.weight = bestDirect;
    if (directReal.empty())
    {
      // Fake vertices only: rebuild from the forward fake parents.
      std::vector<Segment> pre;
      for (Segment x = starter.GetFinishSegment();; x = parF.at(x))
      {
        pre.push_back(x);
        if (x == starter.GetStartSegment())
          break;
      }
      path.assign(pre.rbegin(), pre.rend());
    }
    else
    {
      path = fakePrefix(directSrc);
      path.insert(path.end(), directReal.begin(), directReal.end());
      auto const suf = fakeSuffix(directTgt);
      path.insert(path.end(), suf.begin(), suf.end());
    }
  }
  else
  {
    stats.weight = mu;
    // Hops from the source seed to the meeting vertex and on to the target seed.
    std::vector<std::tuple<Key, Key, uint8_t>> hops;
    Key first = meet;
    for (Key y = meet; F[y].parent != kNoKey; y = F[y].parent)
    {
      hops.emplace_back(F[y].parent, y, F[y].level);
      first = F[y].parent;
    }
    std::reverse(hops.begin(), hops.end());
    Key last = meet;
    for (Key y = meet; B[y].parent != kNoKey; y = B[y].parent)
    {
      hops.emplace_back(y, B[y].parent, B[y].level);
      last = B[y].parent;
    }
    auto const & s0 = src.at(first);
    auto const & t0 = tgt.at(last);
    if (s0.leap)
    {
      // From the start (in a map without overlay) to the border exit the search chose: replaced below.
      path = {starter.GetStartSegment(), s0.real};
    }
    else
    {
      path = fakePrefix(s0.real);
      path.insert(path.end(), s0.chain.begin(), s0.chain.end());
    }
    stats.srcCost = s0.cost;
    stats.srcEnd = path.size() - 1;
    static bool const debug = [] {
      char const * e = std::getenv("PLUS_OVERLAY_DEBUG");
      return e && std::string(e) == "1";
    }();
    double claimed[8] = {}, actual[8] = {};
    size_t count[8] = {};
    for (auto const & [a, b, l] : hops)
    {
      size_t const before = path.size();
      if (!I.UnpackHop(a, b, l, path))
        return finish(Result::NotApplicable, "unpacking failed");
      if (!debug)
        continue;
      bool const fwdHop = F.count(b) && F[b].parent == a;
      double const claim = fwdHop ? F[b].d - F[a].d : B[a].d - B[b].d;
      double act = 0;
      for (size_t k = before; k < path.size(); ++k)
      {
        Segment const & from = path[k - 1];
        I.edges.clear();
        I.Graph(I.mapIdx.at(from.GetMwmId())).GetEdgeList(from, true, true, I.edges);
        double w = kInf;
        for (auto const & e : I.edges)
          if (e.GetTarget() == path[k])
            w = std::min(w, e.GetWeight().GetIntegratedWeight());
        act += w;
      }
      claimed[l] += claim;
      actual[l] += act;
      ++count[l];
    }
    if (debug)
      for (int l = 0; l < 8; ++l)
        if (count[l])
          fprintf(stderr, "plus_overlay hops level %d: %zu, claimed %.3f s, unpacked %.3f s, excess %.3f s\n", l,
                  count[l], claimed[l], actual[l], claimed[l] - actual[l]);
    stats.tgtCost = t0.cost;
    stats.tgtBegin = path.size() - 1;
    if (!t0.leap)
    {
      path.insert(path.end(), t0.chain.begin(), t0.chain.end());
      auto const suf = fakeSuffix(t0.real);
      path.insert(path.end(), suf.begin(), suf.end());
    }
    if (s0.leap || t0.leap)
    {
      // An end in a map without overlay. The border crossing the search chose rests on an estimate, so
      // the route keeps the overlay part only up to a handover point a good distance before that crossing
      // (after it, at the start), and CoMaps' own search across maps finds the rest exactly: it may pick
      // another crossing, or leave and re-enter the map. Without a handover point (the overlay part is
      // short), CoMaps' search does the whole route.
      constexpr double kHandoverM = 30000;
      stats.srcEnd = stats.tgtBegin = 0;
      // Any real road segment will do, also one inside a map crossed on estimates further away (its leg
      // there was already found on real roads).
      auto usable = [&](Segment const & sg) { return !routing::IndexGraphStarter::IsFakeSegment(sg); };
      auto whole = [&]() {
        path.assign(1, starter.GetStartSegment());
        return I.RefineInMap(starter.GetStartSegment(), starter.GetFinishSegment(), path, nullptr,
                             true /* anyMap */);
      };
      bool done = false;
      if (t0.leap)
      {
        ms::LatLon const at = starter.GetPoint(t0.real, true);
        size_t j = path.size();
        for (size_t k = path.size(); k-- > (s0.leap ? 2 : 0);)
          if (usable(path[k]) && ms::DistanceOnEarth(starter.GetPoint(path[k], true), at) >= kHandoverM)
          {
            j = k;
            break;
          }
        if (j == path.size())
        {
          if (!whole())
            return finish(Result::NoRoute, "");
          done = true;
        }
        else
        {
          path.resize(j + 1);
          if (!I.RefineInMap(path[j], starter.GetFinishSegment(), path, nullptr, true /* anyMap */))
            return finish(Result::NotApplicable, "no way into the finish");
        }
      }
      if (s0.leap && !done)
      {
        ms::LatLon const at = starter.GetPoint(s0.real, true);
        size_t i = 0;
        for (size_t k = 2; k < path.size(); ++k)
          if (usable(path[k]) && ms::DistanceOnEarth(starter.GetPoint(path[k], true), at) >= kHandoverM)
          {
            i = k;
            break;
          }
        if (i == 0)
        {
          if (!whole())
            return finish(Result::NoRoute, "");
        }
        else
        {
          std::vector<Segment> np{starter.GetStartSegment()};
          if (!I.RefineInMap(starter.GetStartSegment(), path[i], np, nullptr, true /* anyMap */))
            return finish(Result::NotApplicable, "no way out of the start map");
          np.insert(np.end(), path.begin() + i + 1, path.end());
          path = std::move(np);
        }
      }
    }
  }
  for (auto const & s : path)
    if (!routing::IndexGraphStarter::IsFakeSegment(s))
      used.insert(s.GetMwmId());
  stats.mapsUsed = used.size();
  // Maps the route crosses without an overlay: that part of the route is approximate.
  for (auto const id : used)
  {
    auto it = I.mapIdx.find(id);
    if (it == I.mapIdx.end() || !I.Ovl(it->second))
      stats.approxMaps.push_back(id);
  }
  lap(stats.unpackMs);
  return finish(Result::Ok, "");
}
}  // namespace plus_overlay
