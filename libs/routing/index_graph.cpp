#include "routing/index_graph.hpp"

#include "routing/base/astar_vertex_data.hpp"
#include "routing/restrictions_serialization.hpp"
#include "routing/road_point.hpp"
#include "routing/routing_options.hpp"
#include "routing/world_graph.hpp"

#include "indexer/feature_meta.hpp"

#include "geometry/distance_on_sphere.hpp"
#include "geometry/point3d.hpp"

#include "base/assert.hpp"
#include "base/checked_cast.hpp"
#include "base/logging.hpp"
#include "base/math.hpp"
#include "base/timer.hpp"

#include <algorithm>
#include <unordered_map>
#include <limits>
#include <utility>

namespace routing
{
using namespace base;
using namespace std;

bool IsUTurn(Segment const & u, Segment const & v)
{
  return u.GetFeatureId() == v.GetFeatureId() && u.GetSegmentIdx() == v.GetSegmentIdx() &&
         u.IsForward() != v.IsForward();
}

bool IsBoarding(bool prevIsFerry, bool nextIsFerry)
{
  return !prevIsFerry && nextIsFerry;
}

IndexGraph::IndexGraph(shared_ptr<Geometry> geometry, shared_ptr<EdgeEstimator> estimator,
                       RoutingOptions routingOptions, feature::RegionData const * regionData)
  : m_geometry(std::move(geometry))
  , m_estimator(std::move(estimator))
  , m_avoidRoutingOptions(routingOptions)
{
  CHECK(m_geometry, ());
  CHECK(m_estimator, ());
  if (regionData)
  {
    auto const driving_side = regionData->Get(feature::RegionData::RD_DRIVING);
    m_isLeftHandTraffic = (driving_side == "l");
  }
  else
  {
    m_isLeftHandTraffic = false;
  }
}

bool IndexGraph::IsJoint(RoadPoint const & roadPoint) const
{
  return m_roadIndex.GetJointId(roadPoint) != Joint::kInvalidId;
}

bool IndexGraph::IsJointOrEnd(Segment const & segment, bool fromStart) const
{
  if (IsJoint(segment.GetRoadPoint(fromStart)))
    return true;

  // For features, that ends out of mwm. In this case |m_graph.IsJoint| returns false, but we should
  // think, that it's Joint anyway.
  uint32_t const pointId = segment.GetPointId(fromStart);
  if (pointId == 0)
    return true;

  uint32_t const pointsNumber = GetRoadGeometry(segment.GetFeatureId()).GetPointsCount();
  return pointId + 1 == pointsNumber;
}

void IndexGraph::GetEdgeList(astar::VertexData<Segment, RouteWeight> const & vertexData, bool isOutgoing,
                             bool useRoutingOptions, SegmentEdgeListT & edges, Parents<Segment> const & parents) const
{
  GetEdgeListImpl(vertexData, isOutgoing, useRoutingOptions, true /* useAccessConditional */, edges, parents);
}

void IndexGraph::GetEdgeList(Segment const & segment, bool isOutgoing, bool useRoutingOptions, SegmentEdgeListT & edges,
                             Parents<Segment> const & parents) const
{
  GetEdgeListImpl({segment, Weight(0.0)} /* vertexData */, isOutgoing, useRoutingOptions,
                  false /* useAccessConditional */, edges, parents);
}

void IndexGraph::GetEdgeListImpl(astar::VertexData<Segment, RouteWeight> const & vertexData, bool isOutgoing,
                                 bool useRoutingOptions, bool useAccessConditional, SegmentEdgeListT & edges,
                                 Parents<Segment> const & parents) const
{
  auto const & segment = vertexData.m_vertex;

  RoadPoint const roadPoint = segment.GetRoadPoint(isOutgoing);
  Joint::Id const jointId = m_roadIndex.GetJointId(roadPoint);

  if (jointId != Joint::kInvalidId)
  {
    m_jointIndex.ForEachPoint(jointId, [&](RoadPoint const & rp)
    { GetNeighboringEdges(vertexData, rp, isOutgoing, useRoutingOptions, edges, parents, useAccessConditional); });
  }
  else
  {
    GetNeighboringEdges(vertexData, roadPoint, isOutgoing, useRoutingOptions, edges, parents, useAccessConditional);
  }
}

void IndexGraph::GetLastPointsForJoint(SegmentListT const & children, bool isOutgoing, PointIdListT & lastPoints) const
{
  ASSERT(lastPoints.empty(), ());

  lastPoints.reserve(children.size());
  for (auto const & child : children)
  {
    uint32_t const startPointId = child.GetPointId(!isOutgoing /* front */);
    uint32_t const pointsNumber = GetRoadGeometry(child.GetFeatureId()).GetPointsCount();
    CHECK_LESS(startPointId, pointsNumber, (child));

    uint32_t endPointId;
    // child.IsForward() == isOutgoing
    //   This is the direction, indicating the end of the road,
    //   where we should go for building JointSegment.
    // You can retrieve such result if bust possible options of |child.IsForward()| and |isOutgoing|.
    bool forward = child.IsForward() == isOutgoing;
    if (IsRoad(child.GetFeatureId()))
    {
      endPointId = GetRoad(child.GetFeatureId()).FindNeighbor(startPointId, forward, pointsNumber).second;
    }
    else
    {
      // child.GetFeatureId() can be not road in this case:
      // -->--> { -->-->--> } -->
      // Where { ... } - borders of mwm
      // Here one feature, which enter from mwm A to mwm B and then exit from B to A.
      // And in mwm B no other feature cross this one, those feature is in geometry
      // and absent in roadIndex.
      // In this case we just return endPointNumber.
      endPointId = forward ? pointsNumber - 1 : 0;
    }

    lastPoints.push_back(endPointId);
  }
}

void IndexGraph::GetEdgeList(astar::VertexData<JointSegment, RouteWeight> const & parentVertexData,
                             Segment const & parent, bool isOutgoing, JointEdgeListT & edges,
                             WeightListT & parentWeights, Parents<JointSegment> const & parents) const
{
  GetEdgeListImpl(parentVertexData, parent, isOutgoing, true /* useAccessConditional */, edges, parentWeights, parents);
}

void IndexGraph::GetEdgeList(JointSegment const & parentJoint, Segment const & parent, bool isOutgoing,
                             JointEdgeListT & edges, WeightListT & parentWeights,
                             Parents<JointSegment> const & parents) const
{
  GetEdgeListImpl({parentJoint, Weight(0.0)}, parent, isOutgoing, false /* useAccessConditional */, edges,
                  parentWeights, parents);
}

void IndexGraph::GetEdgeListImpl(astar::VertexData<JointSegment, RouteWeight> const & parentVertexData,
                                 Segment const & parent, bool isOutgoing, bool useAccessConditional,
                                 JointEdgeListT & edges, WeightListT & parentWeights,
                                 Parents<JointSegment> const & parents) const
{
  SegmentListT possibleChildren;
  GetSegmentCandidateForJoint(parent, isOutgoing, possibleChildren);

  PointIdListT lastPoints;
  GetLastPointsForJoint(possibleChildren, isOutgoing, lastPoints);

  ReconstructJointSegment(parentVertexData, parent, possibleChildren, lastPoints, isOutgoing, edges, parentWeights,
                          parents);
}

optional<JointEdge> IndexGraph::GetJointEdgeByLastPoint(Segment const & parent, Segment const & firstChild,
                                                        bool isOutgoing, uint32_t lastPoint) const
{
  SegmentListT const possibleChildren = {firstChild};
  PointIdListT const lastPoints = {lastPoint};

  JointEdgeListT edges;
  WeightListT parentWeights;
  Parents<JointSegment> emptyParents;
  ReconstructJointSegment(astar::VertexData<JointSegment, RouteWeight>::Zero(), parent, possibleChildren, lastPoints,
                          isOutgoing, edges, parentWeights, emptyParents);

  CHECK_LESS_OR_EQUAL(edges.size(), 1, ());
  if (edges.size() == 1)
    return {edges.back()};

  return {};
}

void IndexGraph::Build(uint32_t numJoints)
{
  m_jointIndex.Build(m_roadIndex, numJoints);
}

void IndexGraph::Import(vector<Joint> const & joints)
{
  m_roadIndex.Import(joints);
  CHECK_LESS_OR_EQUAL(joints.size(), numeric_limits<uint32_t>::max(), ());
  Build(checked_cast<uint32_t>(joints.size()));
}

void IndexGraph::SetRestrictions(RestrictionVec && restrictions)
{
  m_restrictionsForward.clear();
  m_restrictionsBackward.clear();

  base::HighResTimer timer;
  for (auto const & restriction : restrictions)
  {
    ASSERT(!restriction.empty(), ());

    auto & forward = m_restrictionsForward[restriction.back()];
    forward.emplace_back(restriction.begin(), prev(restriction.end()));
    reverse(forward.back().begin(), forward.back().end());

    m_restrictionsBackward[restriction.front()].emplace_back(next(restriction.begin()), restriction.end());
  }

  LOG(LDEBUG, ("Restrictions are loaded in:", timer.ElapsedMilliseconds(), "ms"));
}

void IndexGraph::SetUTurnRestrictions(vector<RestrictionUTurn> && noUTurnRestrictions)
{
  for (auto const & noUTurn : noUTurnRestrictions)
    if (noUTurn.m_viaIsFirstPoint)
      m_noUTurnRestrictions[noUTurn.m_featureId].m_atTheBegin = true;
    else
      m_noUTurnRestrictions[noUTurn.m_featureId].m_atTheEnd = true;
}

void IndexGraph::SetRoadAccess(RoadAccess && roadAccess)
{
  m_roadAccess = std::move(roadAccess);
  m_roadAccess.SetCurrentTimeGetter(m_currentTimeGetter);
}

void IndexGraph::SetRoadPenalty(RoadPenalty && roadPenalty)
{
  m_roadPenalty = std::move(roadPenalty);
}

void IndexGraph::GetNeighboringEdges(astar::VertexData<Segment, RouteWeight> const & fromVertexData,
                                     RoadPoint const & rp, bool isOutgoing, bool useRoutingOptions,
                                     SegmentEdgeListT & edges, Parents<Segment> const & parents,
                                     bool useAccessConditional) const
{
  RoadGeometry const & road = GetRoadGeometry(rp.GetFeatureId());

  if (!road.IsValid())
    return;

  if (useRoutingOptions && !road.SuitableForOptions(m_avoidRoutingOptions))
    return;

  bool const bidirectional = !road.IsOneWay();
  auto const & from = fromVertexData.m_vertex;
  bool const blocked = useRoutingOptions && m_plusBlocked;
  if ((isOutgoing || bidirectional) && rp.GetPointId() + 1 < road.GetPointsCount() &&
      !(blocked && m_plusBlocked->IsBlocked(rp.GetFeatureId(), rp.GetPointId())))
  {
    GetNeighboringEdge(fromVertexData, Segment(from.GetMwmId(), rp.GetFeatureId(), rp.GetPointId(), isOutgoing),
                       isOutgoing, edges, parents, useAccessConditional);
  }

  if ((!isOutgoing || bidirectional) && rp.GetPointId() > 0 &&
      !(blocked && m_plusBlocked->IsBlocked(rp.GetFeatureId(), rp.GetPointId() - 1)))
  {
    GetNeighboringEdge(fromVertexData, Segment(from.GetMwmId(), rp.GetFeatureId(), rp.GetPointId() - 1, !isOutgoing),
                       isOutgoing, edges, parents, useAccessConditional);
  }
}

void IndexGraph::GetSegmentCandidateForRoadPoint(RoadPoint const & rp, NumMwmId numMwmId, bool isOutgoing,
                                                 SegmentListT & children) const
{
  RoadGeometry const & road = GetRoadGeometry(rp.GetFeatureId());
  if (!road.IsValid())
    return;

  if (!road.SuitableForOptions(m_avoidRoutingOptions))
    return;

  bool const bidirectional = !road.IsOneWay();
  auto const pointId = rp.GetPointId();

  if ((isOutgoing || bidirectional) && pointId + 1 < road.GetPointsCount() &&
      !IsPlusBlocked(rp.GetFeatureId(), pointId))
    children.emplace_back(numMwmId, rp.GetFeatureId(), pointId, isOutgoing);

  if ((!isOutgoing || bidirectional) && pointId > 0 && !IsPlusBlocked(rp.GetFeatureId(), pointId - 1))
    children.emplace_back(numMwmId, rp.GetFeatureId(), pointId - 1, !isOutgoing);
}

void IndexGraph::GetSegmentCandidateForJoint(Segment const & parent, bool isOutgoing, SegmentListT & children) const
{
  RoadPoint const roadPoint = parent.GetRoadPoint(isOutgoing);
  Joint::Id const jointId = m_roadIndex.GetJointId(roadPoint);

  if (jointId == Joint::kInvalidId)
    return;

  m_jointIndex.ForEachPoint(jointId, [&](RoadPoint const & rp)
  { GetSegmentCandidateForRoadPoint(rp, parent.GetMwmId(), isOutgoing, children); });
}

/// \brief Prolongs segments from |parent| to |firstChildren| directions in order to
///        create JointSegments.
/// \param |firstChildren| - vector of neighbouring segments from parent.
/// \param |lastPointIds| - vector of the end numbers of road points for |firstChildren|.
/// \param |jointEdges| - the result vector with JointEdges.
/// \param |parentWeights| - see |IndexGraphStarterJoints::GetEdgeList| method about this argument.
///                          Shortly - in case of |isOutgoing| == false, method saves here the weights
///                                   from parent to firstChildren.
double IndexGraph::PlusTurnAroundExtraS()
{
  static double const extra = []
  {
    char const * e = getenv("COMAPS_PLUS_TURNAROUND_S");
    return e ? atof(e) : 90.0;
  }();
  return extra;
}

namespace
{
thread_local std::unordered_map<JointSegment, std::vector<Segment>, std::hash<JointSegment>> g_plusFakeJoints;

// Compass-free direction of a segment (x east, y north, in degrees of latitude).
std::pair<double, double> PlusDir(ms::LatLon const & a, ms::LatLon const & b)
{
  double const k = cos(b.m_lat * math::pi / 180.0);
  return {(b.m_lon - a.m_lon) * k, b.m_lat - a.m_lat};
}

double PlusCos(std::pair<double, double> const & a, std::pair<double, double> const & b)
{
  double const la = sqrt(a.first * a.first + a.second * a.second);
  double const lb = sqrt(b.first * b.first + b.second * b.second);
  if (la <= 0.0 || lb <= 0.0)
    return 0.0;
  return (a.first * b.first + a.second * b.second) / (la * lb);
}
}  // namespace

void IndexGraph::PlusNoteFakeJoint(JointSegment const & fake, std::vector<Segment> reals)
{
  g_plusFakeJoints[fake] = std::move(reals);
}

void IndexGraph::PlusClearFakeJoints()
{
  g_plusFakeJoints.clear();
}

std::vector<Segment> const * IndexGraph::PlusFakeJointReals(JointSegment const & fake)
{
  auto const it = g_plusFakeJoints.find(fake);
  return it == g_plusFakeJoints.end() ? nullptr : &it->second;
}

bool IndexGraph::IsPlusTurnAround(Segment const & u, Segment const * pieces, size_t k, Segment const & v) const
{
  if (k == 0 || k > kPlusTurnAroundMaxPieces || u.GetMwmId() != v.GetMwmId())
    return false;
  uint32_t const fu = u.GetFeatureId(), fv = v.GetFeatureId();
  // Values, not references: the road cache may drop a road when another one is read.
  ms::LatLon const u0 = GetPoint(u, false /* front */);
  ms::LatLon const u1 = GetPoint(u, true /* front */);
  ms::LatLon const v0 = GetPoint(v, false /* front */);
  ms::LatLon const v1 = GetPoint(v, true /* front */);
  auto const du = PlusDir(u0, u1);
  double const cosMax = cos(kPlusTurnAroundMaxDeg * math::pi / 180.0);
  if (PlusCos(du, PlusDir(v0, v1)) > -cosMax)
    return false;
  auto reversal = [](Segment const & a, Segment const & b)
  {
    return a.GetFeatureId() == b.GetFeatureId() && a.GetSegmentIdx() == b.GetSegmentIdx() &&
           a.IsForward() != b.IsForward();
  };
  if (reversal(u, pieces[0]) || reversal(pieces[k - 1], v))
    return false;
  double len = 0.0;
  for (size_t i = 0; i < k; ++i)
  {
    Segment const & c = pieces[i];
    if (c.GetMwmId() != u.GetMwmId() || c.GetFeatureId() == fu || c.GetFeatureId() == fv ||
        (i > 0 && reversal(pieces[i - 1], c)))
      return false;
    len += ms::DistanceOnEarth(GetPoint(c, false /* front */), GetPoint(c, true /* front */));
    if (len > kPlusTurnAroundMaxConnectorM)
      return false;
  }
  bool const uOneWay = GetRoadGeometry(fu).IsOneWay();
  bool const vOneWay = GetRoadGeometry(fv).IsOneWay();
  if (fu == fv)
  {
    // Back over the part of the road u came along.
    if (uOneWay || u.IsForward() == v.IsForward())
      return false;
    return u.IsForward() ? v.GetSegmentIdx() <= u.GetSegmentIdx() : v.GetSegmentIdx() >= u.GetSegmentIdx();
  }
  if (!uOneWay || !vOneWay)
    return false;
  // How far to the side of u's direction v starts, from where u ends.
  double const gap = ms::DistanceOnEarth(u1, v0);
  double const along = PlusCos(du, PlusDir(u1, v0));
  double const side = gap * sqrt(std::max(0.0, 1.0 - along * along));
  return side <= kPlusTurnAroundMaxSideM;
}

bool IndexGraph::PlusTurnAroundEndsAt(std::vector<Segment> const & seq) const
{
  size_t const n = seq.size();
  for (size_t k = 1; k <= kPlusTurnAroundMaxPieces && k + 2 <= n; ++k)
    if (IsPlusTurnAround(seq[n - k - 2], &seq[n - k - 1], k, seq[n - 1]))
      return true;
  return false;
}

bool IndexGraph::PlusTurnAroundStartsAt(std::vector<Segment> const & seq) const
{
  size_t const n = seq.size();
  for (size_t k = 1; k <= kPlusTurnAroundMaxPieces && k + 2 <= n; ++k)
    if (IsPlusTurnAround(seq[0], &seq[1], k, seq[k + 1]))
      return true;
  return false;
}

bool IndexGraph::PlusJointTurnAround(JointSegment const & parentJoint, Segment const & parent,
                                     Segment const & firstChild, bool isOutgoing,
                                     Parents<JointSegment> const & parents) const
{
  // The segments the search came along, nearest first, back to the pieces' longest possible run and the
  // segment before it: forward these come before |parent| (driving order reversed), backward after it.
  std::vector<Segment> came;
  size_t const want = kPlusTurnAroundMaxPieces + 1;
  JointSegment j = parentJoint;
  for (size_t hops = 0; hops <= want && came.size() < want; ++hops)
  {
    if (j.IsFake())
    {
      // Its real segments, outward from the end next to the new step.
      if (auto const * reals = PlusFakeJointReals(j))
      {
        auto push = [&](Segment const & r)
        {
          if (came.size() < want && (came.empty() || !(came.back() == r)))
            came.push_back(r);
        };
        if (isOutgoing)
          for (auto it = reals->rbegin(); it != reals->rend(); ++it)
            push(*it);
        else
          for (auto const & r : *reals)
            push(r);
      }
      break;
    }
    // j's segments from the end next to the new step outward.
    Segment s = j.GetSegment(!isOutgoing /* start */);
    Segment const other = j.GetSegment(isOutgoing /* start */);
    bool const inc = s.GetSegmentIdx() < other.GetSegmentIdx();
    while (came.size() < want)
    {
      came.push_back(s);
      if (s == other)
        break;
      s.Next(inc);
    }
    auto const it = parents.find(j);
    if (it == parents.end())
      break;
    j = it->second;
  }
  if (came.empty() || !(came.front() == parent))
    return false;
  std::vector<Segment> seq;
  seq.reserve(came.size() + 1);
  if (isOutgoing)
  {
    seq.assign(came.rbegin(), came.rend());
    seq.push_back(firstChild);
    return PlusTurnAroundEndsAt(seq);
  }
  seq.push_back(firstChild);
  seq.insert(seq.end(), came.begin(), came.end());
  return PlusTurnAroundStartsAt(seq);
}

void IndexGraph::ReconstructJointSegment(astar::VertexData<JointSegment, RouteWeight> const & parentVertexData,
                                         Segment const & parent, SegmentListT const & firstChildren,
                                         PointIdListT const & lastPointIds, bool isOutgoing,
                                         JointEdgeListT & jointEdges, WeightListT & parentWeights,
                                         Parents<JointSegment> const & parents) const
{
  CHECK_EQUAL(firstChildren.size(), lastPointIds.size(), ());

  auto const & weightTimeToParent = parentVertexData.m_realDistance;
  auto const & parentJoint = parentVertexData.m_vertex;
  for (size_t i = 0; i < firstChildren.size(); ++i)
  {
    auto const & firstChild = firstChildren[i];
    auto const lastPointId = lastPointIds[i];

    uint32_t currentPointId = firstChild.GetPointId(!isOutgoing /* front */);
    CHECK_NOT_EQUAL(currentPointId, lastPointId,
                    ("Invariant violated, can not build JointSegment,"
                     "started and ended in the same point."));

    auto const increment = [currentPointId, lastPointId](uint32_t pointId)
    { return currentPointId < lastPointId ? pointId + 1 : pointId - 1; };

    if (IsAccessNoForSure(firstChild.GetFeatureId(), weightTimeToParent, true /* useAccessConditional */))
      continue;

    if (IsAccessNoForSure(parent.GetRoadPoint(isOutgoing), weightTimeToParent, true /* useAccessConditional */))
      continue;

    if (IsUTurn(parent, firstChild) && IsUTurnAndRestricted(parent, firstChild, isOutgoing))
      continue;

    if (IsRestricted(parentJoint, parent.GetFeatureId(), firstChild.GetFeatureId(), isOutgoing, parents))
      continue;

    RouteWeight summaryWeight;
    // Check current JointSegment for bad road access between segments.
    RoadPoint rp = firstChild.GetRoadPoint(isOutgoing);
    uint32_t start = currentPointId;
    bool noRoadAccess = false;
    do
    {
      // This is optimization: we calculate accesses of road points before calculating weight of
      // segments between these road points. Because of that we make fewer calculations when some
      // points have RoadAccess::Type::No.
      // And using |weightTimeToParent| is not fair in fact, because we should calculate weight
      // until this |rp|. But we assume that segments have small length and inaccuracy will not
      // affect user.
      if (IsAccessNoForSure(rp, weightTimeToParent, true /* useAccessConditional */))
      {
        noRoadAccess = true;
        break;
      }

      start = increment(start);
      rp.SetPointId(start);
    }
    while (start != lastPointId);

    if (noRoadAccess)
      continue;

    // Tabula: a joint segment passing a segment of an avoided place.
    if (m_plusBlocked)
    {
      bool blocked = false;
      Segment s = firstChild;
      for (uint32_t p = currentPointId; p != lastPointId && !blocked; p = increment(p))
      {
        blocked = m_plusBlocked->IsBlocked(s.GetFeatureId(), s.GetSegmentIdx());
        s.Next(currentPointId < lastPointId);
      }
      if (blocked)
        continue;
    }

    bool forward = currentPointId < lastPointId;
    Segment current = firstChild;
    Segment prev = parent;

    do
    {
      RouteWeight weight =
          CalculateEdgeWeight(EdgeEstimator::Purpose::Weight, isOutgoing, prev, current, weightTimeToParent);
      // Tabula: completing a turn-around (IsPlusTurnAround) with this step, judged from the segments the
      // search came along (its parents; from a fake joint, the segment the route starts or finishes on).
      if (prev == parent && PlusTurnAroundExtraS() > 0.0 && PlusJointTurnAround(parentJoint, parent, firstChild, isOutgoing, parents))
        weight += RouteWeight(PlusTurnAroundExtraS());

      if (isOutgoing || prev != parent)
        summaryWeight += weight;

      if (prev == parent)
        parentWeights.emplace_back(weight);

      prev = current;
      current.Next(forward);
      currentPointId = increment(currentPointId);
    }
    while (currentPointId != lastPointId);

    jointEdges.emplace_back(isOutgoing ? JointSegment(firstChild, prev) : JointSegment(prev, firstChild),
                            summaryWeight);
  }
}

void IndexGraph::GetNeighboringEdge(astar::VertexData<Segment, RouteWeight> const & fromVertexData, Segment const & to,
                                    bool isOutgoing, SegmentEdgeListT & edges, Parents<Segment> const & parents,
                                    bool useAccessConditional) const
{
  auto const & from = fromVertexData.m_vertex;
  auto const & weightToFrom = fromVertexData.m_realDistance;

  if (IsUTurn(from, to) && IsUTurnAndRestricted(from, to, isOutgoing))
    return;

  if (IsRestricted(from, from.GetFeatureId(), to.GetFeatureId(), isOutgoing, parents))
    return;

  if (IsAccessNoForSure(to.GetFeatureId(), weightToFrom, useAccessConditional))
    return;

  RoadPoint const rp = from.GetRoadPoint(isOutgoing);
  if (IsAccessNoForSure(rp, weightToFrom, useAccessConditional))
    return;

  if (m_plusEdgesOnly)
  {
    edges.emplace_back(to, RouteWeight(0.0));
    return;
  }
  auto const weight = CalculateEdgeWeight(EdgeEstimator::Purpose::Weight, isOutgoing, from, to, weightToFrom);

  edges.emplace_back(to, weight);
}

IndexGraph::PenaltyData IndexGraph::GetRoadPenaltyData(Segment const & segment) const
{
  auto const & road = GetRoadGeometry(segment.GetFeatureId());
  return {road.IsPassThroughAllowed(), road.GetRoutingOptions().Has(RoutingOptions::Option::Ferry)};
}

RouteWeight IndexGraph::GetPenalties(EdgeEstimator::Purpose purpose, Segment const & u, Segment const & v,
                                     optional<RouteWeight> const & prevWeight) const
{
  auto const & fromPenaltyData = GetRoadPenaltyData(u);
  auto const & toPenaltyData = GetRoadPenaltyData(v);
  // Route crosses border of pass-through/non-pass-through area if |u| and |v| have different
  // pass through restrictions.
  int8_t const passThroughPenalty = fromPenaltyData.m_passThroughAllowed == toPenaltyData.m_passThroughAllowed ? 0 : 1;

  int8_t accessPenalty = 0;
  int8_t accessConditionalPenalties = 0;

  if (u.GetFeatureId() != v.GetFeatureId())
  {
    // We do not distinguish between RoadAccess::Type::Private and RoadAccess::Type::Destination for
    // now.
    auto const [fromAccess, fromConfidence] = prevWeight ? m_roadAccess.GetAccess(u.GetFeatureId(), *prevWeight)
                                                         : m_roadAccess.GetAccessWithoutConditional(u.GetFeatureId());

    auto const [toAccess, toConfidence] = prevWeight ? m_roadAccess.GetAccess(v.GetFeatureId(), *prevWeight)
                                                     : m_roadAccess.GetAccessWithoutConditional(v.GetFeatureId());

    if (fromConfidence == RoadAccess::Confidence::Sure && toConfidence == RoadAccess::Confidence::Sure)
    {
      bool const fromAccessAllowed = fromAccess == RoadAccess::Type::Yes;
      bool const toAccessAllowed = toAccess == RoadAccess::Type::Yes;
      // Route crosses border of access=yes/access={private, destination} area if |u| and |v| have
      // different access restrictions.
      accessPenalty = fromAccessAllowed == toAccessAllowed ? 0 : 1;
    }
    else if (toConfidence == RoadAccess::Confidence::Maybe)
    {
      accessConditionalPenalties = 1;
    }
  }

  // RoadPoint between u and v is front of u.
  auto const rp = u.GetRoadPoint(true /* front */);
  auto const [rpAccessType, rpConfidence] =
      prevWeight ? m_roadAccess.GetAccess(rp, *prevWeight) : m_roadAccess.GetAccessWithoutConditional(rp);
  // Get penalty from new penalty system if MWM supports it
  auto penalty = m_roadPenalty.GetPenalty(rp);
  uint16_t penaltyTime = 0;
  if (penalty)
    penaltyTime = penalty->m_timeSeconds;

  switch (rpConfidence)
  {
  case RoadAccess::Confidence::Sure:
  {
    if (rpAccessType != RoadAccess::Type::Yes)
      accessPenalty = 1;
    break;
  }
  case RoadAccess::Confidence::Maybe:
  {
    accessConditionalPenalties = 1;
    break;
  }
  }

  double weightPenalty = 0.0;
  if (IsUTurn(u, v))
    weightPenalty += m_estimator->GetUTurnPenalty(purpose);

  if (IsBoarding(fromPenaltyData.m_isFerry, toPenaltyData.m_isFerry))
    weightPenalty += m_estimator->GetFerryLandingPenalty(purpose);

  return {penaltyTime + weightPenalty /* weight */, passThroughPenalty, accessPenalty, accessConditionalPenalties,
          0.0 /* transitTime */};
}

WorldGraphMode IndexGraph::GetMode() const
{
  return WorldGraphMode::Undefined;
}

bool IndexGraph::IsUTurnAndRestricted(Segment const & parent, Segment const & child, bool isOutgoing) const
{
  ASSERT(IsUTurn(parent, child), ());

  uint32_t const featureId = parent.GetFeatureId();
  uint32_t const turnPoint = parent.GetPointId(isOutgoing);
  auto const & roadGeometry = GetRoadGeometry(featureId);

  RoadPoint const rp = parent.GetRoadPoint(isOutgoing);
  if (m_roadIndex.GetJointId(rp) == Joint::kInvalidId && !roadGeometry.IsEndPointId(turnPoint))
    return true;

  auto const it = m_noUTurnRestrictions.find(featureId);
  if (it == m_noUTurnRestrictions.cend())
    return false;

  auto const & uTurn = it->second;
  if (uTurn.m_atTheBegin && turnPoint == 0)
    return true;

  uint32_t const n = roadGeometry.GetPointsCount();
  ASSERT_GREATER_OR_EQUAL(n, 1, ());
  return uTurn.m_atTheEnd && turnPoint == n - 1;
}

RouteWeight IndexGraph::CalculateEdgeWeight(EdgeEstimator::Purpose purpose, bool isOutgoing, Segment const & from,
                                            Segment const & to,
                                            std::optional<RouteWeight const> const & prevWeight) const
{
  auto const & to_segment = isOutgoing ? to : from;
  auto const & from_segment = isOutgoing ? from : to;
  auto const & to_road = GetRoadGeometry(to_segment.GetFeatureId());
  auto const weight = RouteWeight(m_estimator->CalcSegmentWeight(to_segment, to_road, purpose));
  auto const penalties = GetPenalties(purpose, isOutgoing ? from : to, isOutgoing ? to : from, prevWeight);
  auto const turn_penalty = getTurnPenalty(purpose, from_segment, to_segment);
  return weight + penalties + turn_penalty;
}

RouteWeight IndexGraph::getTurnPenalty(EdgeEstimator::Purpose purpose, Segment const & from, Segment const & to) const
{
  if (from.GetFeatureId() == to.GetFeatureId())
    return {0, 0, 0, 0, 0};
  auto const & to_road = GetRoadGeometry(to.GetFeatureId());
  auto const & from_road = GetRoadGeometry(from.GetFeatureId());
  auto v1 = ms::ToVector(GetPoint(from, true)) - ms::ToVector(GetPoint(from, false));
  auto v2 = ms::ToVector(GetPoint(to, true)) - ms::ToVector(GetPoint(to, false));
  auto const dotLen = v1.Length() * v2.Length();
  if (dotLen == 0)
    return {0, 0, 0, 0, 0};
  auto normal = ms::ToVector(GetPoint(from, true)) + ms::ToVector(GetPoint(to, false));
  normal = normal / normal.Length();
  auto const signed_angle =
      math::RadToDeg(atan2(m3::DotProduct(m3::CrossProduct(v1, v2), normal), m3::DotProduct(v1, v2)));
  return {m_estimator->GetTurnPenalty(purpose, signed_angle, from_road, to_road, m_isLeftHandTraffic), 0, 0, 0, 0};
}
}  // namespace routing
