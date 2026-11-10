// Comaps++: roads and areas kept out of car routes (roadmap item 1). The list lives in the core
// (routing/plus_avoid_places.hpp), which every car route reads; nothing is prepared for it.

#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "routing/plus_avoid_places.hpp"
#include "routing/plus_overlay/overlay_router.hpp"

#include "routing_common/car_model.hpp"

#include "geometry/mercator.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace
{
constexpr size_t kNumbersPerPlace = 5;  // kind, lat, lon, radius (m), enabled (0/1)
}  // namespace

extern "C"
{
// kNumbersPerPlace numbers per place, in list order.
JNIEXPORT jdoubleArray JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeGetNumbers(JNIEnv * env, jclass)
{
  auto const places = routing::plus_avoid::GetPlaces();
  std::vector<double> numbers;
  for (auto const & p : places)
  {
    numbers.push_back(static_cast<double>(p.kind));
    numbers.push_back(p.lat);
    numbers.push_back(p.lon);
    numbers.push_back(p.radiusM);
    numbers.push_back(p.enabled ? 1.0 : 0.0);
  }
  jdoubleArray result = env->NewDoubleArray(static_cast<jsize>(numbers.size()));
  env->SetDoubleArrayRegion(result, 0, static_cast<jsize>(numbers.size()), numbers.data());
  return result;
}

JNIEXPORT jobjectArray JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeGetNames(JNIEnv * env, jclass)
{
  std::vector<std::string> names;
  for (auto const & p : routing::plus_avoid::GetPlaces())
    names.push_back(p.name);
  return jni::ToJavaStringArray(env, names);
}

// Replaces and saves the list. |numbers| as nativeGetNumbers returns them, |names| one per place.
JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeSet(JNIEnv * env, jclass,
                                                                                 jdoubleArray numbers,
                                                                                 jobjectArray names)
{
  jsize const count = env->GetArrayLength(names);
  if (env->GetArrayLength(numbers) != count * static_cast<jsize>(kNumbersPerPlace))
    return;
  std::vector<double> n(static_cast<size_t>(count) * kNumbersPerPlace);
  env->GetDoubleArrayRegion(numbers, 0, static_cast<jsize>(n.size()), n.data());
  std::vector<routing::plus_avoid::Place> places;
  for (jsize i = 0; i < count; ++i)
  {
    double const * v = n.data() + i * kNumbersPerPlace;
    routing::plus_avoid::Place p;
    p.kind = v[0] == 1 ? routing::plus_avoid::Place::Kind::Road : routing::plus_avoid::Place::Kind::Area;
    p.lat = v[1];
    p.lon = v[2];
    p.radiusM = static_cast<uint32_t>(std::max(0.0, v[3]));
    p.enabled = v[4] != 0;
    auto * name = static_cast<jstring>(env->GetObjectArrayElement(names, i));
    p.name = name ? jni::ToNativeString(env, name) : std::string();
    if (name)
      env->DeleteLocalRef(name);
    places.push_back(std::move(p));
  }
  routing::plus_avoid::SetPlaces(places);
}

// The car road a "avoid this road" at (lat, lon) would pick: its name or number ("" if it has neither), or
// null if no car road is within reach.
JNIEXPORT jstring JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeRoadAt(JNIEnv * env, jclass,
                                                                                       jdouble lat, jdouble lon)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  auto & storage = frm()->GetStorage();
  routing::CarModelFactory models([&storage](std::string const & id) { return storage.GetParentIdFor(id); });
  auto const roads = routing::plus_avoid::FindRoads(frm()->GetDataSource(), models, mercator::FromLatLon(lat, lon),
                                                    routing::plus_avoid::kRoadPickRadiusM);
  auto const picked = routing::plus_avoid::PickRoad(roads, {});
  if (picked.empty())
    return nullptr;
  return jni::ToJavaString(env, routing::plus_avoid::RoadLabel(roads[picked.front()]));
}

// How many avoided places applied to the last car route: they covered roads and weren't at its start, finish or
// a stop.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusAvoidPlaces_nativeLastApplied(JNIEnv *, jclass)
{
  return static_cast<jint>(plus_overlay::GetLastEngineReport().avoidPlaces);
}
}  // extern "C"
