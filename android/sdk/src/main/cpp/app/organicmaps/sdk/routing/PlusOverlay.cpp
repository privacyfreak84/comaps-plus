// Comaps++: building the per-map overlay files that make long car routes fast.
// Called from app.organicmaps.background.PlusOverlayWork on a WorkManager thread.

#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "routing/plus_overlay/overlay_builder.hpp"
#include "routing/plus_overlay/overlay_file.hpp"
#include "routing/plus_overlay/overlay_router.hpp"

#include "routing/routing_options.hpp"

#include "indexer/data_source.hpp"
#include "indexer/mwm_set.hpp"

#include "platform/local_country_file.hpp"
#include "platform/platform.hpp"

#include "geometry/mercator.hpp"

#include "base/file_name_utils.hpp"
#include "base/logging.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <tuple>
#include <string>
#include <vector>

namespace
{
std::atomic<bool> g_cancel{false};
std::atomic<uint32_t> g_buildingAvoidMask{0};  // avoid mask of the file being built (0: base file)
std::atomic<double> g_progress{0};
std::mutex g_stageMutex;
std::string g_stage;

struct DownloadedMap
{
  std::string countryId;
  platform::LocalCountryFile localFile;
  int64_t version = 0;
  std::string overlayPath;
  std::string avoidPath;  // for the current avoid options (empty without)
  m2::RectD rect;         // the map's bounds (mercator)
};

uint32_t CurrentAvoidMask()
{
  return static_cast<uint32_t>(
      routing::RoutingOptions::LoadOptionsFromSettings(routing::VehicleType::Car).GetOptions());
}

std::vector<DownloadedMap> DownloadedMaps(uint32_t avoidMask)
{
  std::vector<DownloadedMap> maps;
  std::vector<std::shared_ptr<MwmInfo>> infos;
  frm()->GetDataSource().GetMwmsInfo(infos);
  for (auto const & info : infos)
  {
    if (!info || info->GetType() != MwmInfo::COUNTRY)
      continue;
    auto const & lf = info->GetLocalFile();
    auto const mwmPath = lf.GetPath(MapFileType::Map);
    maps.push_back({lf.GetCountryName(), lf, info->GetVersion(), plus_overlay::OverlayPathForMap(mwmPath),
                    avoidMask ? plus_overlay::AvoidOverlayPathForMap(mwmPath, avoidMask) : std::string(),
                    info->m_bordersRect});
  }
  return maps;
}

// Partition splits per map, kept across map versions so that a map update reuses the cells.
std::string TreeDir()
{
  return base::JoinPath(GetPlatform().WritableDir(), "plus_overlay");
}

std::string TreePath(std::string const & countryId)
{
  return base::JoinPath(TreeDir(), countryId + ".tree");
}

// The writable directory and its map version subdirectories.
std::set<std::string> MapDirs()
{
  std::string const root = GetPlatform().WritableDir();
  std::set<std::string> dirs{root};
  Platform::TFilesWithType subdirs;
  Platform::GetFilesByType(root, Platform::EFileType::Directory, subdirs);
  for (auto const & [d, type] : subdirs)
    if (!d.empty() && d.find_first_not_of("0123456789") == std::string::npos)
      dirs.insert(base::JoinPath(root, d));
  return dirs;
}

bool NeedsBaseBuild(DownloadedMap const & m)
{
  return !plus_overlay::OverlayFileMatches(m.overlayPath, m.version, plus_overlay::DefaultParamsHash());
}

bool NeedsAvoidBuild(DownloadedMap const & m, uint32_t avoidMask)
{
  if (avoidMask == 0)
    return false;
  uint64_t buildId = 0;
  if (!plus_overlay::OverlayFileMatches(m.overlayPath, m.version, plus_overlay::DefaultParamsHash(), &buildId))
    return true;
  return !plus_overlay::AvoidCliquesFileMatches(m.avoidPath, buildId, avoidMask);
}
}  // namespace

extern "C"
{
// Downloaded maps whose base overlay file is missing or was built for another map version / build
// parameters, and with |withAvoid| also those lacking the file for the current avoid options (after all
// the others: routes with avoid options are exact without it, only slower). Most useful first: the map you
// are in, then the nearest ones (without a position: smallest first).
JNIEXPORT jobjectArray JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeMapsToBuild(JNIEnv * env, jclass,
                                                                                         jboolean hasPosition,
                                                                                         jdouble lat, jdouble lon,
                                                                                         jboolean withAvoid)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  m2::PointD const here = mercator::FromLatLon(lat, lon);
  uint32_t const mask = withAvoid ? CurrentAvoidMask() : 0;
  std::vector<std::tuple<bool, double, std::string>> todo;
  for (auto const & m : DownloadedMaps(mask))
  {
    bool const base = NeedsBaseBuild(m);
    if (!base && !NeedsAvoidBuild(m, mask))
      continue;
    double key = 0;
    if (hasPosition)
    {
      if (!m.rect.IsPointInside(here))
        key = mercator::DistanceOnEarth(here, m.rect.Center());
    }
    else
    {
      uint64_t size = 0;
      Platform::GetFileSizeByFullPath(m.localFile.GetPath(MapFileType::Map), size);
      key = static_cast<double>(size);
    }
    todo.emplace_back(!base, key, m.countryId);
  }
  std::sort(todo.begin(), todo.end());
  std::vector<std::string> ids;
  for (auto const & t : todo)
    ids.push_back(std::get<2>(t));
  return jni::ToJavaStringArray(env, ids);
}

// Whether a downloaded map lacks its base file or the file for the current avoid options.
JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeNeedsBuild(JNIEnv * env, jclass,
                                                                                       jstring countryId)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  std::string const id = jni::ToNativeString(env, countryId);
  uint32_t const mask = CurrentAvoidMask();
  for (auto const & m : DownloadedMaps(mask))
    if (m.countryId == id)
      return NeedsBaseBuild(m) || NeedsAvoidBuild(m, mask);
  return false;
}

// Builds the base file if needed, then (with |withAvoid|) the file for the current avoid options.
// 0: built, 1: cancelled, 2: failed, 3: the map isn't downloaded (any more), 4: already up to date.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeBuild(JNIEnv * env, jclass,
                                                                                jstring countryId, jint threads,
                                                                                jboolean withAvoid)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  std::string const id = jni::ToNativeString(env, countryId);
  uint32_t const mask = CurrentAvoidMask();
  std::unique_ptr<DownloadedMap> map;
  for (auto & m : DownloadedMaps(mask))
    if (m.countryId == id)
      map = std::make_unique<DownloadedMap>(std::move(m));
  if (!map)
    return 3;
  bool const needBase = NeedsBaseBuild(*map);
  if (!needBase && !(withAvoid && NeedsAvoidBuild(*map, mask)))
    return 4;

  g_cancel = false;
  g_progress = 0;
  plus_overlay::BuildOptions o;
  o.threads = static_cast<unsigned>(std::max(1, static_cast<int>(threads)));
  o.cancelled = []() { return g_cancel.load(); };
  // Called from worker threads too: only atomics and a mutex here, no JNI.
  o.progress = [](double f, std::string const & stage)
  {
    g_progress = f;
    std::lock_guard<std::mutex> lock(g_stageMutex);
    if (g_stage != stage)
      g_stage = stage;
  };
  auto & storage = frm()->GetStorage();
  auto const parentGetter = [&storage](std::string const & cid) { return storage.GetParentIdFor(cid); };
  auto const report = [&id](plus_overlay::BuildResult const & r, char const * what) -> jint
  {
    switch (r.status)
    {
    case plus_overlay::BuildResult::Status::Ok:
      LOG(LINFO, ("Comaps++", what, "built for", id, ":", r.fileBytes / 1048576.0, "MB in", r.seconds,
                  "s, peak memory", r.peakRssKb / 1024, "MB"));
      return 0;
    case plus_overlay::BuildResult::Status::Cancelled: LOG(LINFO, ("Comaps++", what, "cancelled for", id)); return 1;
    case plus_overlay::BuildResult::Status::Failed:
      LOG(LWARNING, ("Comaps++", what, "failed for", id, ":", r.error));
      return 2;
    }
    return 2;
  };
  if (needBase)
  {
    g_buildingAvoidMask = 0;
    if (Platform::MkDirChecked(TreeDir()))
      o.partitionTreePath = TreePath(id);
    jint const code =
        report(plus_overlay::BuildOverlayFile(map->localFile, parentGetter, map->overlayPath, o), "overlay");
    if (code != 0)
      return code;
  }
  o.partitionTreePath.clear();
  if (withAvoid && NeedsAvoidBuild(*map, mask))
  {
    g_buildingAvoidMask = mask;
    g_progress = 0;
    jint const code = report(plus_overlay::BuildAvoidCliquesFile(map->localFile, parentGetter, mask,
                                                                 map->overlayPath, map->avoidPath, o),
                             "avoid-option overlay");
    g_buildingAvoidMask = 0;
    return code;
  }
  return 0;
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeCancel(JNIEnv *, jclass)
{
  g_cancel = true;
}

// Avoid options (RoutingOptions bits) of the file being built; 0 while building a base file or idle.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeBuildingAvoidMask(JNIEnv *, jclass)
{
  return static_cast<jint>(g_buildingAvoidMask.load());
}

JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeCurrentAvoidMask(JNIEnv *, jclass)
{
  return static_cast<jint>(CurrentAvoidMask());
}

JNIEXPORT jdouble JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeProgress(JNIEnv *, jclass)
{
  return g_progress.load();
}

JNIEXPORT jstring JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeStage(JNIEnv * env, jclass)
{
  std::lock_guard<std::mutex> lock(g_stageMutex);
  return jni::ToJavaString(env, g_stage);
}

// Deletes overlay files whose map was deleted or replaced by an update, and interrupted builds; unless
// |keepAllAvoid|, also avoid-option files for other options than the current ones.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeDeleteOrphans(JNIEnv *, jclass,
                                                                                        jboolean keepAllAvoid)
{
  // Avoid-option files: keep those of the current options; with none set, keep the last ones used.
  uint32_t const mask = CurrentAvoidMask();
  int64_t const keepMask = (keepAllAvoid || mask == 0) ? -1 : static_cast<int64_t>(mask);
  size_t deleted = 0;
  for (auto const & d : MapDirs())
    deleted += plus_overlay::DeleteOrphanedOverlayFiles(d, keepMask);
  // Partition splits of maps that are no longer downloaded.
  std::set<std::string> downloaded;
  for (auto const & m : DownloadedMaps(0))
    downloaded.insert(m.countryId);
  Platform::FilesList trees;
  Platform::GetFilesByExt(TreeDir(), ".tree", trees);
  for (auto const & t : trees)
    if (downloaded.count(t.substr(0, t.size() - 5)) == 0 && Platform::RemoveFileIfExists(base::JoinPath(TreeDir(), t)))
      ++deleted;
  if (deleted)
    LOG(LINFO, ("Comaps++ overlay: deleted", deleted, "files of removed or updated maps"));
  return static_cast<jint>(deleted);
}
// {engine, seconds, reason} of the last car route: engine is "none", "fast", "rejoin" or "normal".
JNIEXPORT jobjectArray JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeLastEngine(JNIEnv * env, jclass)
{
  auto const r = plus_overlay::GetLastEngineReport();
  char const * engine = "none";
  switch (r.engine)
  {
  case plus_overlay::EngineReport::Engine::None: engine = "none"; break;
  case plus_overlay::EngineReport::Engine::Overlay: engine = "fast"; break;
  case plus_overlay::EngineReport::Engine::Rejoin: engine = "rejoin"; break;
  case plus_overlay::EngineReport::Engine::Normal: engine = "normal"; break;
  case plus_overlay::EngineReport::Engine::Approximate: engine = "approximate"; break;
  }
  std::vector<std::string> out{engine, std::to_string(r.seconds), r.reason};
  out.insert(out.end(), r.approxMaps.begin(), r.approxMaps.end());
  return jni::ToJavaStringArray(env, out);
}

JNIEXPORT jobjectArray JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeLastAvoidMaps(JNIEnv * env, jclass)
{
  return jni::ToJavaStringArray(env, plus_overlay::GetLastEngineReport().avoidMaps);
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeRefreshRoute(JNIEnv *, jclass)
{
  // Builds the current route again from the current position (navigation) or its start (planning),
  // as on a traffic update; the new route replaces the old one only if it is found.
  frm()->GetRoutingManager().RoutingSession().RebuildRouteOnTrafficUpdate();
}

JNIEXPORT jlong JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeStorageBytes(JNIEnv *, jclass)
{
  uint64_t bytes = 0;
  for (auto const & d : MapDirs())
  {
    Platform::FilesList files;
    Platform::GetFilesByExt(d, plus_overlay::kFileExtension, files);
    for (auto const & f : files)
    {
      uint64_t size = 0;
      if (Platform::GetFileSizeByFullPath(base::JoinPath(d, f), size))
        bytes += size;
    }
  }
  return static_cast<jlong>(bytes);
}
}  // extern "C"
