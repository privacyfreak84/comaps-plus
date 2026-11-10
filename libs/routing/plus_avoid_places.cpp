#include "routing/plus_avoid_places.hpp"

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

#include <atomic>
#include <cmath>
#include <cstdlib>
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
std::string_view constexpr kSettingsKey = "PlusAvoidPlaces";

std::mutex g_mutex;
bool g_loaded = false;
std::vector<Place> g_places;
std::atomic<uint64_t> g_generation{1};

void LoadLocked()
{
  if (g_loaded)
    return;
  g_loaded = true;
  // Desktop tools and tests set the list through the environment.
  if (char const * env = std::getenv("COMAPS_PLUS_AVOID_PLACES"))
  {
    Deserialize(env, g_places);
    return;
  }
  std::string s;
  if (settings::Get(kSettingsKey, s))
    Deserialize(s, g_places);
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
  std::lock_guard lock(g_mutex);
  g_loaded = true;
  if (places == g_places)
    return;
  g_places = places;
  settings::Set(kSettingsKey, Serialize(g_places));
  ++g_generation;
}

uint64_t Generation()
{
  return g_generation.load();
}

// One place per entry, entries separated by ';':
// <version 1>,<kind 0|1>,<lat>,<lon>,<radius m>,<enabled 0|1>,<url-encoded name>
std::string Serialize(std::vector<Place> const & places)
{
  std::ostringstream os;
  os << std::setprecision(9);
  for (size_t i = 0; i < places.size(); ++i)
  {
    auto const & p = places[i];
    if (i > 0)
      os << ';';
    os << 1 << ',' << static_cast<int>(p.kind) << ',' << p.lat << ',' << p.lon << ',' << p.radiusM << ','
       << (p.enabled ? 1 : 0) << ',' << url::UrlEncode(p.name);
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
    if (f.size() < 7 || !strings::to_int(f[0], version) || version != 1 || !strings::to_int(f[1], kind) ||
        kind < 0 || kind > 1 || !strings::to_double(f[2].c_str(), p.lat) ||
        !strings::to_double(f[3].c_str(), p.lon) || !strings::to_uint(f[4], radius) ||
        !strings::to_int(f[5], enabled) || std::fabs(p.lat) > 90 || std::fabs(p.lon) > 180)
    {
      ok = false;  // skip what this version can't read, keep the rest
      continue;
    }
    p.kind = static_cast<Place::Kind>(kind);
    p.radiusM = radius;
    p.enabled = enabled != 0;
    p.name = url::UrlDecode(f[6]);
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
  auto const matches = [&name](RoadNearby const & r) { return !name.empty() && (r.name == name || r.ref == name); };
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

std::string RoadLabel(RoadNearby const & road)
{
  return road.name.empty() ? road.ref : road.name;
}
}  // namespace plus_avoid
}  // namespace routing
