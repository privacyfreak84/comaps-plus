#pragma once

#include "routing/base/astar_graph.hpp"
#include "routing/base/small_list.hpp"
#include "routing/edge_estimator.hpp"
#include "routing/geometry.hpp"
#include "routing/joint.hpp"
#include "routing/joint_index.hpp"
#include "routing/plus_avoid_places.hpp"
#include "routing/joint_segment.hpp"
#include "routing/restrictions_serialization.hpp"
#include "routing/road_access.hpp"
#include "routing/road_index.hpp"
#include "routing/road_penalty.hpp"
#include "routing/routing_options.hpp"
#include "routing/segment.hpp"

#include "base/assert.hpp"

#include <memory>
#include <optional>
#include <vector>

#include "3party/ankerl/unordered_dense.h"

namespace routing
{
/**
 * @brief Whether a maneuver between two segments is a U turn.
 *
 * A maneuver between two segments `u` and `v` is a U turn if, and only if, both segments differ in
 * direction but are otherwise identical.
 *
 * For U turns on multiple-carriageway roads, i.e. from one carriageway onto a crossing road (or
 * connecting cariageway) and further to the opposite cariageway, this function returns false.
 *
 * @param u The first segment
 * @param v The second segment
 *
 * @return True if the maneuver is a U turn on the same segment, false otherwise.
 */
class RouteWeight;

using NumMwmId = std::uint16_t;

bool IsUTurn(Segment const & u, Segment const & v);

enum class WorldGraphMode;

class IndexGraph final
{
public:
  // AStarAlgorithm types aliases:
  using Vertex = Segment;
  using Edge = SegmentEdge;
  using Weight = RouteWeight;

  template <typename VertexType>
  using Parents = typename AStarGraph<VertexType, void, void>::Parents;

  using Restrictions = ankerl::unordered_dense::map<uint32_t, std::vector<std::vector<uint32_t>>>;

  using SegmentEdgeListT = SmallList<SegmentEdge>;
  using JointEdgeListT = SmallList<JointEdge>;
  using WeightListT = SmallList<RouteWeight>;
  using SegmentListT = SmallList<Segment>;
  using PointIdListT = SmallList<uint32_t>;

  IndexGraph() = default;
  IndexGraph(std::shared_ptr<Geometry> geometry, std::shared_ptr<EdgeEstimator> estimator,
             RoutingOptions routingOptions = RoutingOptions(), feature::RegionData const * regionData = nullptr);

  // Put outgoing (or ingoing) egdes for segment to the 'edges' vector.
  void GetEdgeList(astar::VertexData<Segment, RouteWeight> const & vertexData, bool isOutgoing, bool useRoutingOptions,
                   SegmentEdgeListT & edges, Parents<Segment> const & parents = {}) const;
  void GetEdgeList(Segment const & segment, bool isOutgoing, bool useRoutingOptions, SegmentEdgeListT & edges,
                   Parents<Segment> const & parents = {}) const;

  void GetEdgeList(astar::VertexData<JointSegment, RouteWeight> const & parentVertexData, Segment const & parent,
                   bool isOutgoing, JointEdgeListT & edges, WeightListT & parentWeights,
                   Parents<JointSegment> const & parents) const;
  void GetEdgeList(JointSegment const & parentJoint, Segment const & parent, bool isOutgoing, JointEdgeListT & edges,
                   WeightListT & parentWeights, Parents<JointSegment> const & parents) const;

  std::optional<JointEdge> GetJointEdgeByLastPoint(Segment const & parent, Segment const & firstChild, bool isOutgoing,
                                                   uint32_t lastPoint) const;

  Joint::Id GetJointId(RoadPoint const & rp) const { return m_roadIndex.GetJointId(rp); }

  bool IsRoad(uint32_t featureId) const { return m_roadIndex.IsRoad(featureId); }
  RoadJointIds const & GetRoad(uint32_t featureId) const { return m_roadIndex.GetRoad(featureId); }
  RoadGeometry const & GetRoadGeometry(uint32_t featureId) const { return m_geometry->GetRoad(featureId); }

  Geometry & GetGeometry() const { return *m_geometry; }

  // Tabula: the overlay needs the conditional-access tables (time-dependent edges).
  RoadAccess const & GetRoadAccess() const { return m_roadAccess; }

  // Tabula: road segments of avoided places (plus_avoid_places.hpp). Edges onto them are left out
  // wherever avoid options apply (useRoutingOptions).
  void SetPlusBlocked(std::shared_ptr<plus_avoid::Blocked const> blocked) { m_plusBlocked = std::move(blocked); }
  std::shared_ptr<plus_avoid::Blocked const> const & GetPlusBlocked() const { return m_plusBlocked; }
  // Tabula: turning around. CoMaps charges its U-turn penalty only for reversing on one segment; turning
  // around over other roads (a gap in a median, a loop, a lay-by, a small roundabout) paid ordinary turns.
  // A turn-around is u, c_1..c_k, v: consecutive segments, 1 <= k <= kPlusTurnAroundMaxPieces pieces on
  // other roads than u's and v's, together at most kPlusTurnAroundMaxConnectorM long, no step a reversal on
  // one segment (CoMaps charges those already), v running the opposite way of u (within
  // kPlusTurnAroundMaxDeg), and either
  //  - u and v on two different one-way roads, v starting at most kPlusTurnAroundMaxSideM to the side of
  //    where u ends (the other carriageway of a divided road, not a parallel street around a block); or
  //  - u and v on the same two-way road, v going back over the part u came along (a loop, a lay-by, a
  //    turning circle, a roundabout). A hairpin bend goes on along its road, so it isn't one.
  // Completing one costs PlusTurnAroundExtraS() more: in the fast-route data (overlay builder and router)
  // and, from the search's parents, in the joint search used for maps without that data.
  static constexpr double kPlusTurnAroundMaxConnectorM = 50.0;
  static constexpr double kPlusTurnAroundMaxDeg = 30.0;
  static constexpr double kPlusTurnAroundMaxSideM = 30.0;
  static constexpr size_t kPlusTurnAroundMaxPieces = 8;
  // 90 s; COMAPS_PLUS_TURNAROUND_S overrides it (tests: 0 gives CoMaps' own routes back).
  static double PlusTurnAroundExtraS();
  bool IsPlusTurnAround(Segment const & u, Segment const * pieces, size_t k, Segment const & v) const;
  // Whether |seq| (segments in driving order) ends with a turn-around onto seq.back(), or starts with one
  // from seq.front(); seq may be longer than needed.
  bool PlusTurnAroundEndsAt(std::vector<Segment> const & seq) const;
  bool PlusTurnAroundStartsAt(std::vector<Segment> const & seq) const;
  // Tabula: the real segments under a fake joint of the joint search (the parts of the roads the start or
  // finish snapped to, and the real segment it ends at), in driving order, noted by
  // IndexGraphStarterJoints for the turn-around rule above.
  static void PlusNoteFakeJoint(JointSegment const & fake, std::vector<Segment> reals);
  static void PlusClearFakeJoints();
  static std::vector<Segment> const * PlusFakeJointReals(JointSegment const & fake);
  bool PlusJointTurnAround(JointSegment const & parentJoint, Segment const & parent, Segment const & firstChild,
                           bool isOutgoing, Parents<JointSegment> const & parents) const;

  // Tabula: while set, edge lists carry no weights (all zero): which edges exist doesn't depend on them,
  // and the overlay builder's first walk only counts edges.
  void SetPlusEdgesOnly(bool edgesOnly) { m_plusEdgesOnly = edgesOnly; }
  bool IsPlusBlocked(uint32_t featureId, uint32_t segmentIdx) const
  {
    return m_plusBlocked && m_plusBlocked->IsBlocked(featureId, segmentIdx);
  }

  RoadAccess::Type GetAccessType(Segment const & segment) const
  {
    return m_roadAccess.GetAccessWithoutConditional(segment.GetFeatureId()).first;
  }

  uint32_t GetNumRoads() const { return m_roadIndex.GetSize(); }
  uint32_t GetNumJoints() const { return m_jointIndex.GetNumJoints(); }
  uint32_t GetNumPoints() const { return m_jointIndex.GetNumPoints(); }

  void Build(uint32_t numJoints);
  void Import(std::vector<Joint> const & joints);

  void SetRestrictions(RestrictionVec && restrictions);
  void SetUTurnRestrictions(std::vector<RestrictionUTurn> && noUTurnRestrictions);
  void SetRoadAccess(RoadAccess && roadAccess);
  void SetRoadPenalty(RoadPenalty && roadPenalty);

  void PushFromSerializer(Joint::Id jointId, RoadPoint const & rp) { m_roadIndex.PushFromSerializer(jointId, rp); }

  template <typename F>
  void ForEachRoad(F && f) const
  {
    m_roadIndex.ForEachRoad(std::forward<F>(f));
  }

  template <typename F>
  void ForEachPoint(Joint::Id jointId, F && f) const
  {
    m_jointIndex.ForEachPoint(jointId, std::forward<F>(f));
  }

  bool IsJoint(RoadPoint const & roadPoint) const;
  bool IsJointOrEnd(Segment const & segment, bool fromStart) const;
  void GetLastPointsForJoint(SegmentListT const & children, bool isOutgoing, PointIdListT & lastPoints) const;

  WorldGraphMode GetMode() const;
  ms::LatLon const & GetPoint(Segment const & segment, bool front) const
  {
    return GetRoadGeometry(segment.GetFeatureId()).GetPoint(segment.GetPointId(front));
  }

  /// \brief Check, that we can go to |currentFeatureId|.
  /// We pass |parentFeatureId| and don't use |parent.GetFeatureId()| because
  /// |parent| can be fake sometimes but |parentFeatureId| is almost non-fake.
  template <typename ParentVertex>
  bool IsRestricted(ParentVertex const & parent, uint32_t parentFeatureId, uint32_t currentFeatureId, bool isOutgoing,
                    Parents<ParentVertex> const & parents) const;

  bool IsUTurnAndRestricted(Segment const & parent, Segment const & child, bool isOutgoing) const;

  /// @param[in]  isOutgoing true, when movig from -> to, false otherwise.
  /// @param[in]  prevWeight used for fetching access:conditional.
  /// I suppose :) its time when user will be at the end of |from| (|to| if \a isOutgoing == false) segment.
  /// @return Transition weight + |to| (|from| if \a isOutgoing == false) segment's weight.
  RouteWeight CalculateEdgeWeight(EdgeEstimator::Purpose purpose, bool isOutgoing, Segment const & from,
                                  Segment const & to,
                                  std::optional<RouteWeight const> const & prevWeight = std::nullopt) const;

  template <typename T>
  void SetCurrentTimeGetter(T && t)
  {
    m_currentTimeGetter = std::forward<T>(t);
  }

private:
  void GetEdgeListImpl(astar::VertexData<Segment, RouteWeight> const & vertexData, bool isOutgoing,
                       bool useRoutingOptions, bool useAccessConditional, SegmentEdgeListT & edges,
                       Parents<Segment> const & parents) const;

  void GetEdgeListImpl(astar::VertexData<JointSegment, RouteWeight> const & parentVertexData, Segment const & parent,
                       bool isOutgoing, bool useAccessConditional, JointEdgeListT & edges, WeightListT & parentWeights,
                       Parents<JointSegment> const & parents) const;

  void GetNeighboringEdges(astar::VertexData<Segment, RouteWeight> const & fromVertexData, RoadPoint const & rp,
                           bool isOutgoing, bool useRoutingOptions, SegmentEdgeListT & edges,
                           Parents<Segment> const & parents, bool useAccessConditional) const;
  void GetNeighboringEdge(astar::VertexData<Segment, RouteWeight> const & fromVertexData, Segment const & to,
                          bool isOutgoing, SegmentEdgeListT & edges, Parents<Segment> const & parents,
                          bool useAccessConditional) const;

  struct PenaltyData
  {
    PenaltyData(bool passThroughAllowed, bool isFerry) : m_passThroughAllowed(passThroughAllowed), m_isFerry(isFerry) {}

    bool m_passThroughAllowed;
    bool m_isFerry;
  };

  PenaltyData GetRoadPenaltyData(Segment const & segment) const;

  /// \brief Calculates penalties for moving from |u| to |v|.
  /// \param |prevWeight| uses for fetching access:conditional. In fact it is time when user
  /// will be at |u|. This time is based on start time of route building and weight of calculated
  /// path until |u|.
  RouteWeight GetPenalties(EdgeEstimator::Purpose purpose, Segment const & u, Segment const & v,
                           std::optional<RouteWeight> const & prevWeight) const;

  void GetSegmentCandidateForRoadPoint(RoadPoint const & rp, NumMwmId numMwmId, bool isOutgoing,
                                       SegmentListT & children) const;
  void GetSegmentCandidateForJoint(Segment const & parent, bool isOutgoing, SegmentListT & children) const;
  void ReconstructJointSegment(astar::VertexData<JointSegment, RouteWeight> const & parentVertexData,
                               Segment const & parent, SegmentListT const & firstChildren,
                               PointIdListT const & lastPointIds, bool isOutgoing, JointEdgeListT & jointEdges,
                               WeightListT & parentWeights, Parents<JointSegment> const & parents) const;

  template <typename AccessPositionType>
  bool IsAccessNoForSure(AccessPositionType const & accessPositionType, RouteWeight const & weight,
                         bool useAccessConditional) const;

  std::shared_ptr<Geometry> m_geometry;
  std::shared_ptr<EdgeEstimator> m_estimator;
  RoadIndex m_roadIndex;
  JointIndex m_jointIndex;
  Restrictions m_restrictionsForward;
  Restrictions m_restrictionsBackward;
  // u_turn can be in both sides of feature.
  struct UTurnEnding
  {
    bool m_atTheBegin = false;
    bool m_atTheEnd = false;
  };
  // Stored featureId and it's UTurnEnding, which shows where is
  // u_turn restriction is placed - at the beginning or at the ending of feature.
  //
  // If m_noUTurnRestrictions.count(featureId) == 0, that means, that there are no any
  // no_u_turn restriction at the feature with id = featureId.
  ankerl::unordered_dense::map<uint32_t, UTurnEnding> m_noUTurnRestrictions;
  RoadAccess m_roadAccess;
  RoadPenalty m_roadPenalty;
  RoutingOptions m_avoidRoutingOptions;
  std::shared_ptr<plus_avoid::Blocked const> m_plusBlocked;
  bool m_plusEdgesOnly = false;
  bool m_isLeftHandTraffic;

  std::function<time_t()> m_currentTimeGetter = []() { return GetCurrentTimestamp(); };
  RouteWeight getTurnPenalty(EdgeEstimator::Purpose purpose, Segment const & from, Segment const & to) const;
};

template <typename AccessPositionType>
bool IndexGraph::IsAccessNoForSure(AccessPositionType const & accessPositionType, RouteWeight const & weight,
                                   bool useAccessConditional) const
{
  auto const [accessType, confidence] = useAccessConditional
                                          ? m_roadAccess.GetAccess(accessPositionType, weight)
                                          : m_roadAccess.GetAccessWithoutConditional(accessPositionType);
  return accessType == RoadAccess::Type::No && confidence == RoadAccess::Confidence::Sure;
}

template <typename ParentVertex>
bool IndexGraph::IsRestricted(ParentVertex const & parent, uint32_t parentFeatureId, uint32_t currentFeatureId,
                              bool isOutgoing, Parents<ParentVertex> const & parents) const
{
  if (parentFeatureId == currentFeatureId)
    return false;

  auto const & restrictions = isOutgoing ? m_restrictionsForward : m_restrictionsBackward;
  auto const it = restrictions.find(currentFeatureId);
  if (it == restrictions.cend())
    return false;

  std::vector<ParentVertex> parentsFromCurrent;
  // Finds the first featureId from parents, that differ from |p.GetFeatureId()|.
  auto const appendNextParent = [&parents](ParentVertex const & p, auto & parentsVector)
  {
    uint32_t prevFeatureId = p.GetFeatureId();
    uint32_t curFeatureId = prevFeatureId;

    auto nextParent = parents.end();
    auto * curParent = &p;
    while (curFeatureId == prevFeatureId)
    {
      auto const parentIt = parents.find(*curParent);
      if (parentIt == parents.cend())
        return false;

      curFeatureId = parentIt->second.GetFeatureId();
      nextParent = parentIt;
      curParent = &nextParent->second;
    }

    ASSERT(nextParent != parents.end(), ());
    parentsVector.emplace_back(nextParent->second);
    return true;
  };

  for (std::vector<uint32_t> const & restriction : it->second)
  {
    bool const prevIsParent = restriction[0] == parentFeatureId;
    if (!prevIsParent)
      continue;

    if (restriction.size() == 1)
      return true;

    // If parents are empty we process only two feature restrictions.
    if (parents.empty())
      continue;

    if (!appendNextParent(parent, parentsFromCurrent))
      continue;

    for (size_t i = 1; i < restriction.size(); ++i)
    {
      ASSERT_GREATER_OR_EQUAL(i, 1, ("Unexpected overflow."));
      if (i - 1 == parentsFromCurrent.size() && !appendNextParent(parentsFromCurrent.back(), parentsFromCurrent))
        break;

      if (parentsFromCurrent.back().GetFeatureId() != restriction[i])
        break;

      if (i + 1 == restriction.size())
        return true;
    }
  }

  return false;
}
}  // namespace routing
