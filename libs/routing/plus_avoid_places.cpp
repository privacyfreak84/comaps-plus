#include "routing/plus_avoid_places.hpp"

#include "routing/index_graph_starter.hpp"
#include "routing/route.hpp"

#include "indexer/data_source.hpp"
#include "indexer/feature.hpp"
#include "indexer/feature_data.hpp"
#include "indexer/scales.hpp"

#include "i18n/localisation.hpp"

#include "platform/settings.hpp"

#include "geometry/mercator.hpp"
#include "geometry/parametrized_segment.hpp"

#include "coding/url.hpp"

#include "base/string_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace routing
{
namespace plus_avoid
{
namespace
{
// Where earlier versions kept a permanent list. Avoided places now belong to a route; the old list is
// removed the first time the list is read.
std::string_view constexpr kOldSettingsKey = "PlusAvoidPlaces";

std::mutex g_mutex;
bool g_loaded = false;
std::vector<Place> g_places;
std::function<void()> g_listener;

void LoadLocked()
{
  if (g_loaded)
    return;
  g_loaded = true;
  if (char const * env = std::getenv("COMAPS_PLUS_AVOID_PLACES"))
  {
    Deserialize(env, g_places);
    return;
  }
  std::string old;
  if (settings::Get(kOldSettingsKey, old))
    settings::Delete(kOldSettingsKey);
}
}  // namespace

std::vector<Place> GetPlaces()
{
  std::lock_guard lock(g_mutex);
  LoadLocked();
  return g_places;
}

void SetPlaces(std::vector<Place> const & places)
{
  std::function<void()> listener;
  {
    std::lock_guard lock(g_mutex);
    LoadLocked();
    if (places == g_places)
      return;
    g_places = places;
    listener = g_listener;
  }
  if (listener)
    listener();
}

void ClearPlaces()
{
  SetPlaces({});
}

void SetChangeListener(std::function<void()> listener)
{
  std::lock_guard lock(g_mutex);
  g_listener = std::move(listener);
}

// One place per entry, entries separated by ';':
// <version 2>,<kind 0|1>,<lat>,<lon>,<radius m>,<url-encoded name>
// Version 1 (the old permanent list, still used in test scripts) had <enabled 0|1> before the name; switched
// off entries are dropped.
std::string Serialize(std::vector<Place> const & places)
{
  std::ostringstream os;
  os << std::setprecision(9);
  for (size_t i = 0; i < places.size(); ++i)
  {
    auto const & p = places[i];
    if (i > 0)
      os << ';';
    os << 2 << ',' << static_cast<int>(p.kind) << ',' << p.lat << ',' << p.lon << ',' << p.radiusM << ','
       << url::UrlEncode(p.name);
  }
  return os.str();
}

bool Deserialize(std::string const & s, std::vector<Place> & places)
{
  auto split = [](std::string const & str, char sep)
  {
    std::vector<std::string> out(1);
    for (char c : str)
      if (c == sep)
        out.emplace_back();
      else
        out.back() += c;
    return out;
  };
  places.clear();
  if (s.empty())
    return true;
  bool ok = true;
  for (auto const & entry : split(s, ';'))
  {
    auto const f = split(entry, ',');
    int version = 0, kind = 0, enabled = 1;
    unsigned radius = 0;
    Place p;
    bool const known = !f.empty() && strings::to_int(f[0], version) && (version == 1 || version == 2) &&
                       f.size() >= (version == 1 ? 7u : 6u);
    if (!known || !strings::to_int(f[1], kind) || kind < 0 || kind > 1 ||
        !strings::to_double(f[2].c_str(), p.lat) || !strings::to_double(f[3].c_str(), p.lon) ||
        !strings::to_uint(f[4], radius) || (version == 1 && !strings::to_int(f[5], enabled)) ||
        std::fabs(p.lat) > 90 || std::fabs(p.lon) > 180)
    {
      ok = false;  // skip what this version can't read, keep the rest
      continue;
    }
    if (enabled == 0)
      continue;
    p.kind = static_cast<Place::Kind>(kind);
    p.radiusM = radius;
    p.name = url::UrlDecode(f[version == 1 ? 6 : 5]);
    places.push_back(std::move(p));
  }
  return ok;
}
namespace
{
double SegmentDistance(m2::PointD const & c, m2::PointD const & a, m2::PointD const & b)
{
  return mercator::DistanceOnEarth(c, m2::ParametrizedSegment<m2::PointD>(a, b).ClosestPointTo(c));
}
}  // namespace

double DistanceToPolyline(m2::PointD const & p, std::vector<m2::PointD> const & points)
{
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < points.size(); ++i)
    best = std::min(best, SegmentDistance(p, points[i], points[i + 1]));
  return best;
}

std::vector<RoadNearby> FindRoads(DataSource const & dataSource, VehicleModelFactoryInterface & models,
                                  m2::PointD const & center, double radiusM)
{
  std::vector<RoadNearby> roads;
  dataSource.ForEachInRect([&](FeatureType & ft)
  {
    if (ft.GetGeomType() != feature::GeomType::Line)
      return;
    auto const & mwmId = ft.GetID().m_mwmId;
    if (!mwmId.IsAlive())
      return;
    auto const model = models.GetVehicleModelForCountry(mwmId.GetInfo()->GetCountryName());
    if (!model || !model->IsRoad(feature::TypesHolder(ft)))
      return;
    ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
    size_t const n = ft.GetPointsCount();
    if (n < 2)
      return;
    RoadNearby r;
    r.id = ft.GetID();
    r.distM = std::numeric_limits<double>::max();
    r.points.reserve(n);
    for (size_t i = 0; i < n; ++i)
      r.points.push_back(ft.GetPoint(i));
    for (size_t i = 0; i + 1 < n; ++i)
    {
      double const d = SegmentDistance(center, r.points[i], r.points[i + 1]);
      r.distM = std::min(r.distM, d);
      if (d <= radiusM)
        r.segsWithin.push_back(static_cast<uint32_t>(i));
    }
    if (r.segsWithin.empty())
      return;
    r.name = std::string(ft.GetName(localisation::kDefaultNameIndex));
    r.ref = ft.GetRef();
    roads.push_back(std::move(r));
  }, mercator::RectByCenterXYAndSizeInMeters(center, radiusM), scales::GetUpperScale());
  return roads;
}

std::vector<size_t> PickRoad(std::vector<RoadNearby> const & roads, std::string const & name)
{
  auto const matches = [&name](RoadNearby const & r)
  { return !name.empty() && (r.name == name || r.ref == name || RoadLabel(r) == name); };
  bool const anyMatch = std::any_of(roads.begin(), roads.end(), matches);
  RoadNearby const * best = nullptr;
  for (auto const & r : roads)
    if ((!anyMatch || matches(r)) && (!best || r.distM < best->distM))
      best = &r;
  std::vector<size_t> picked;
  for (size_t i = 0; best && i < roads.size(); ++i)
  {
    auto const & r = roads[i];
    if (&r == best || (r.distM <= best->distM + 1.0 && r.name == best->name && r.ref == best->ref &&
                       r.id.m_mwmId != best->id.m_mwmId))
      picked.push_back(i);
  }
  return picked;
}

std::optional<Place> RouteAvoidOption(Route const & route, std::vector<m2::PointD> const & checkpoints,
                                      Tapped const & tapped, DataSource const & dataSource,
                                      VehicleModelFactoryInterface & models)
{
  auto const & poly = route.GetPoly().GetPoints();
  auto const & segments = route.GetRouteSegments();
  if (poly.size() < 2)
    return {};
  auto nearEnd = [&](m2::PointD const & p, double margin)
  {
    return std::any_of(checkpoints.begin(), checkpoints.end(),
                       [&](m2::PointD const & c) { return mercator::DistanceOnEarth(c, p) <= margin; });
  };

  if (tapped.settlement)
  {
    double const radius = std::clamp<double>(tapped.settlementRadiusM, kMinAreaRadiusM, kMaxAreaRadiusM);
    if (DistanceToPolyline(tapped.point, poly) > radius || nearEnd(tapped.point, radius + kCheckpointMarginM))
      return {};
    auto const ll = mercator::ToLatLon(tapped.point);
    Place p;
    p.kind = Place::Kind::Area;
    p.lat = ll.m_lat;
    p.lon = ll.m_lon;
    p.radiusM = static_cast<uint32_t>(std::lround(radius));
    p.name = tapped.settlementName;
    return p;
  }

  // The route's segment nearest to the point.
  size_t best = 0;
  double bestDist = std::numeric_limits<double>::max();
  size_t const n = std::min(segments.size(), poly.size() - 1);
  for (size_t i = 0; i < n; ++i)
  {
    double const d = SegmentDistance(tapped.point, poly[i], poly[i + 1]);
    if (d < bestDist)
    {
      bestDist = d;
      best = i;
    }
  }
  if (bestDist > kOnRouteM)
    return {};
  auto const & seg = segments[best].GetSegment();
  if (IndexGraphStarter::IsFakeSegment(seg))
    return {};
  // Saved at the middle of that segment, which lies on the route's road (a point at a junction could pick a
  // crossing road later), with the road's name or number as the maps store it.
  m2::PointD const mid = (poly[best] + poly[best + 1]) / 2;
  auto const roads = FindRoads(dataSource, models, mid, kRoadPickRadiusM);
  RoadNearby const * road = nullptr;
  for (auto const & r : roads)
    if (r.id.m_index == seg.GetFeatureId() && (!road || r.distM < road->distM))
      road = &r;
  if (!road)
    return {};
  for (auto const & c : checkpoints)
    if (DistanceToPolyline(c, road->points) <= kCheckpointMarginM)
      return {};
  auto const ll = mercator::ToLatLon(mid);
  Place p;
  p.kind = Place::Kind::Road;
  p.lat = ll.m_lat;
  p.lon = ll.m_lon;
  p.name = RoadLabel(*road);
  return p;
}

std::optional<size_t> PlaceAt(std::vector<Place> const & places, m2::PointD const & point,
                              DataSource const & dataSource, VehicleModelFactoryInterface & models)
{
  for (size_t i = 0; i < places.size(); ++i)
  {
    auto const & place = places[i];
    m2::PointD const center = mercator::FromLatLon(place.lat, place.lon);
    if (place.kind == Place::Kind::Area)
    {
      double const radius = std::clamp<double>(place.radiusM, kMinAreaRadiusM, kMaxAreaRadiusM);
      if (mercator::DistanceOnEarth(center, point) <= radius)
        return i;
      continue;
    }
    auto const roads = FindRoads(dataSource, models, center, kRoadPickRadiusM);
    for (size_t r : PickRoad(roads, place.name))
      if (DistanceToPolyline(point, roads[r].points) <= kOnRouteM)
        return i;
  }
  return {};
}

std::string RoadLabel(RoadNearby const & road)
{
  if (!road.name.empty())
    return road.name;
  // A road with several numbers ("NH135A;IN:NH/135A"): the first.
  return road.ref.substr(0, road.ref.find(';'));
}
}  // namespace plus_avoid
}  // namespace routing
