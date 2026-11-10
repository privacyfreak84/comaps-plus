#pragma once

// Comaps++ roadmap item 1: roads and areas the user chose to keep out of car routes.
//
// Nothing is prepared for them. Each car route looks up the road segments a place covers in the maps
// (PlusResolveAvoidPlaces in IndexRouter) and leaves them out of the road graph (IndexGraph). The fast-route
// search treats the map cells holding those segments like cells it has no times for and searches their
// roads (overlay_router.cpp), so routes stay exact.

#include "routing_common/num_mwm_id.hpp"
#include "routing_common/vehicle_model.hpp"

#include "indexer/feature_decl.hpp"

#include "geometry/point2d.hpp"

#include "3party/ankerl/unordered_dense.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class DataSource;

namespace routing
{
namespace plus_avoid
{
struct Place
{
  enum class Kind : uint8_t
  {
    Area = 0,  // every road segment within radiusM of the point
    Road = 1,  // the road nearest to the point (one OSM way, every segment of it)
  };

  Kind kind = Kind::Area;
  double lat = 0;
  double lon = 0;
  uint32_t radiusM = 0;  // Area only
  bool enabled = true;
  // Shown in the list. For a road: its name or number, preferred over a nearer road when the road is
  // looked up again in a later route or map version.
  std::string name;

  bool operator==(Place const & o) const
  {
    return kind == o.kind && lat == o.lat && lon == o.lon && radiusM == o.radiusM && enabled == o.enabled &&
           name == o.name;
  }
};

constexpr uint32_t kMinAreaRadiusM = 20;
constexpr uint32_t kMaxAreaRadiusM = 20000;
// A road is picked within this distance of the point the user chose.
constexpr double kRoadPickRadiusM = 50.0;
// A place this close to a route's start, finish or stop doesn't apply to that route: the route has to
// get there (or leave from there) anyway.
constexpr double kCheckpointMarginM = 30.0;

// The saved list (settings key PlusAvoidPlaces), safe from any thread.
std::vector<Place> GetPlaces();
// Replaces and saves the list.
void SetPlaces(std::vector<Place> const & places);
// Changes whenever the list changes.
uint64_t Generation();

std::string Serialize(std::vector<Place> const & places);
bool Deserialize(std::string const & s, std::vector<Place> & places);

// A car road near a point.
struct RoadNearby
{
  FeatureID id;
  double distM = 0;                  // from the point to the road's nearest segment
  std::string name, ref;             // the road's name (default language) and number
  std::vector<m2::PointD> points;    // geometry (mercator), as routing reads it
  std::vector<uint32_t> segsWithin;  // segments within the search radius
};

// Car roads (as |models| define them) with a segment within |radiusM| of |center|, in every loaded map.
std::vector<RoadNearby> FindRoads(DataSource const & dataSource, VehicleModelFactoryInterface & models,
                                  m2::PointD const & center, double radiusM);

// The road a Kind::Road place stands for, among roads found around its point: the nearest one named or
// numbered |name| (if any is), else the nearest; with its copies in neighbouring maps (same name and number,
// within a metre). Indices into |roads|, empty if |roads| is.
std::vector<size_t> PickRoad(std::vector<RoadNearby> const & roads, std::string const & name);

// Name for a road in the list: its name, else its number, else empty.
std::string RoadLabel(RoadNearby const & road);

// Distance from |p| to the polyline |points| (metres).
double DistanceToPolyline(m2::PointD const & p, std::vector<m2::PointD> const & points);

// Road segments left out of car routes in one map: feature id -> segment indices, ascending (empty: every
// segment of the feature).
struct Blocked
{
  ankerl::unordered_dense::map<uint32_t, std::vector<uint32_t>> features;

  bool IsBlocked(uint32_t featureId, uint32_t segmentIdx) const
  {
    auto const it = features.find(featureId);
    if (it == features.end())
      return false;
    auto const & segs = it->second;
    return segs.empty() || std::binary_search(segs.begin(), segs.end(), segmentIdx);
  }
  bool Empty() const { return features.empty(); }
};

using BlockedByMap = std::unordered_map<NumMwmId, std::shared_ptr<Blocked const>>;
}  // namespace plus_avoid
}  // namespace routing
