#pragma once

// Edges the overlay can't precompute, shared by the builder and the router.

#include "routing/road_access.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"

#include <cstdint>

namespace plus_overlay
{
// CoMaps' penalties (pass-through 30 min, access change 2 h, conditional "maybe" 1 h) are multiples of
// 30 min. A route weight's penalty part, in these units, is what separates its integrated weight (what
// searches minimise) from its travel time (what conditional access is checked against).
constexpr double kPenaltyUnitS = 1800.0;
// Penalty units kept per clique value in the file (3 bits); more are stored as this.
constexpr uint32_t kMaxStoredPenaltyUnits = 7;

inline uint32_t PenaltyUnits(routing::RouteWeight const & w)
{
  double const p = w.GetIntegratedWeight() - w.GetWeight();
  return p <= 0 ? 0 : static_cast<uint32_t>(p / kPenaltyUnitS + 0.5);
}

// Conditional access (open only at some times) on either road of the step u -> v (forward), or at the
// point between them. CoMaps checks it at the time the route gets there, so such a step's existence and
// weight depend on the departure time and can't be part of a precomputed clique.
inline bool IsTimeDependentEdge(routing::RoadAccess const & ra, routing::Segment const & u,
                                routing::Segment const & v)
{
  auto const & ways = ra.GetWayToAccessConditional();
  if (!ways.empty() && (ways.count(u.GetFeatureId()) != 0 || ways.count(v.GetFeatureId()) != 0))
    return true;
  auto const & points = ra.GetPointToAccessConditional();
  return !points.empty() && points.count(u.GetRoadPoint(true /* front */)) != 0;
}

inline bool HasTimeDependentAccess(routing::RoadAccess const & ra)
{
  return !ra.GetWayToAccessConditional().empty() || !ra.GetPointToAccessConditional().empty();
}
}  // namespace plus_overlay
