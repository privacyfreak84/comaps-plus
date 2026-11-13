// Tabula: roads and areas kept out of the current car route (roadmap item 1). The list lives in the core
// (routing/plus_avoid_places.hpp) for as long as the route does; nothing is prepared for it and nothing is
// stored apart from the route.

#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "routing/plus_avoid_places.hpp"
#include "routing/plus_overlay/overlay_router.hpp"
#include "routing/route.hpp"

#include "indexer/data_source.hpp"
#include "indexer/feature.hpp"
#include "indexer/ftypes_matcher.hpp"

#include "routing_common/car_model.hpp"

#include "geometry/mercator.hpp"

#include <optional>
#include <string>
#include <vector>

namespace
{
// What the place page offers for the place shown in it, kept for nativeApplyPending: avoiding the settlement
// the route passes through or the route's road at that point (routing::plus_avoid::RouteAvoidOption), or no
// longer avoiding a place this route avoids (the point lies in it).
std::optional<routing::plus_avoid::Place> g_pending;
bool g_pendingRemoval = false;
}  // namespace

extern "C"
{
JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativePrepareRouteAvoid(JNIEnv *, jclass)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  g_pending.reset();
  g_pendingRemoval = false;
  auto & rm = frm()->GetRoutingManager();
  if (!frm()->HasPlacePageInfo() || rm.GetCurrentRouterType() != routing::RouterType::Vehicle)
    return false;
  auto const & info = frm()->GetCurrentPlacePageInfo();
  if (info.IsMyPosition())
    return false;
  auto & storage = frm()->GetStorage();
  routing::CarModelFactory models([&storage](std::string const & id) { return storage.GetParentIdFor(id); });

  // A place this route already avoids (drawn in red): offer to stop avoiding it. Works without a route too,
  // so a route that avoided places left impossible can be fixed.
  auto const places = routing::plus_avoid::GetPlaces();
  if (auto const i = routing::plus_avoid::PlaceAt(places, info.GetMercator(), frm()->GetDataSource(), models))
  {
    g_pending = places[*i];
    g_pendingRemoval = true;
    return true;
  }

  auto const route = rm.RoutingSession().GetRoute();
  if (!route || !route->IsValid())
    return false;
  routing::plus_avoid::Tapped tapped;
  tapped.point = info.GetMercator();
  if (info.IsFeature())
  {
    auto const type = ftypes::IsLocalityChecker::Instance().GetType(info.GetTypes());
    if (type == ftypes::LocalityType::City || type == ftypes::LocalityType::Town ||
        type == ftypes::LocalityType::Village)
    {
      FeaturesLoaderGuard guard(frm()->GetDataSource(), info.GetID().m_mwmId);
      auto ft = guard.GetFeatureByIndex(info.GetID().m_index);
      uint64_t const population = ft ? ftypes::GetPopulation(*ft) : ftypes::GetDefPopulation(type);
      tapped.settlement = true;
      tapped.settlementRadiusM = ftypes::GetRadiusByPopulationForRouting(population, type);
      tapped.settlementName = info.GetTitle();
    }
  }
  std::vector<m2::PointD> checkpoints;
  for (auto const & p : rm.GetRoutePoints())
    checkpoints.push_back(p.m_position);
  g_pending = routing::plus_avoid::RouteAvoidOption(*route, checkpoints, tapped, frm()->GetDataSource(), models);
  return g_pending.has_value();
}

// Whether the prepared place is one the route already avoids (the place page offers Don't avoid).
JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeIsPendingRemoval(JNIEnv *, jclass)
{
  return g_pending && g_pendingRemoval;
}

// {kind, radius in metres} of the prepared place (empty if none).
JNIEXPORT jdoubleArray JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativePendingNumbers(JNIEnv * env,
                                                                                                    jclass)
{
  std::vector<double> n;
  if (g_pending)
    n = {static_cast<double>(g_pending->kind), static_cast<double>(g_pending->radiusM)};
  jdoubleArray result = env->NewDoubleArray(static_cast<jsize>(n.size()));
  env->SetDoubleArrayRegion(result, 0, static_cast<jsize>(n.size()), n.data());
  return result;
}

JNIEXPORT jstring JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativePendingName(JNIEnv * env, jclass)
{
  return jni::ToJavaString(env, g_pending ? g_pending->name : std::string());
}

// Adds the prepared place to the current route's avoided places, or removes it from them. The map redraws them
// by itself; the caller recalculates the route.
JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeApplyPending(JNIEnv *, jclass)
{
  if (!g_pending)
    return;
  auto places = routing::plus_avoid::GetPlaces();
  if (g_pendingRemoval)
    std::erase(places, *g_pending);
  else
    places.push_back(*g_pending);
  routing::plus_avoid::SetPlaces(places);
  g_pending.reset();
  g_pendingRemoval = false;
}

// How many places the current route avoids.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeCount(JNIEnv *, jclass)
{
  return static_cast<jint>(routing::plus_avoid::GetPlaces().size());
}

// How many avoided places applied to the last car route: they covered roads and weren't at its start, finish or
// a stop.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeLastApplied(JNIEnv *, jclass)
{
  return static_cast<jint>(plus_overlay::GetLastEngineReport().avoidPlaces);
}
}  // extern "C"
