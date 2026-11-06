// Comaps++: building the per-map overlay files that make long car routes fast.
// Called from app.organicmaps.background.PlusOverlayWork on a WorkManager thread.

#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "routing/plus_overlay/overlay_builder.hpp"
#include "routing/plus_overlay/overlay_file.hpp"

#include "routing/routing_options.hpp"

#include "indexer/data_source.hpp"
#include "indexer/mwm_set.hpp"

#include "platform/local_country_file.hpp"
#include "platform/platform.hpp"

#include "base/file_name_utils.hpp"
#include "base/logging.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
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
                    avoidMask ? plus_overlay::AvoidOverlayPathForMap(mwmPath, avoidMask) : std::string()});
  }
  return maps;
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
// Downloaded maps whose overlay file is missing or was built for another map version / build parameters.
JNIEXPORT jobjectArray JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeMapsToBuild(JNIEnv * env, jclass)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  std::vector<std::string> ids;
  uint32_t const mask = CurrentAvoidMask();
  for (auto const & m : DownloadedMaps(mask))
    if (NeedsBaseBuild(m) || NeedsAvoidBuild(m, mask))
      ids.push_back(m.countryId);
  return jni::ToJavaStringArray(env, ids);
}

// 0: built, 1: cancelled, 2: failed, 3: the map isn't downloaded (any more), 4: already up to date.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeBuild(JNIEnv * env, jclass,
                                                                                jstring countryId, jint threads)
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
  if (!needBase && !NeedsAvoidBuild(*map, mask))
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
    jint const code =
        report(plus_overlay::BuildOverlayFile(map->localFile, parentGetter, map->overlayPath, o), "overlay");
    if (code != 0)
      return code;
  }
  if (NeedsAvoidBuild(*map, mask))
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

// Deletes overlay files whose map was deleted or replaced by an update, and interrupted builds.
JNIEXPORT jint JNICALL Java_app_organicmaps_sdk_routing_PlusOverlay_nativeDeleteOrphans(JNIEnv *, jclass)
{
  std::string const root = GetPlatform().WritableDir();
  std::set<std::string> dirs{root};
  Platform::TFilesWithType subdirs;
  Platform::GetFilesByType(root, Platform::EFileType::Directory, subdirs);
  for (auto const & [d, type] : subdirs)
    if (!d.empty() && d.find_first_not_of("0123456789") == std::string::npos)
      dirs.insert(base::JoinPath(root, d));
  // Avoid-option files: keep those of the current options; with none set, keep the last ones used.
  uint32_t const mask = CurrentAvoidMask();
  size_t deleted = 0;
  for (auto const & d : dirs)
    deleted += plus_overlay::DeleteOrphanedOverlayFiles(d, mask != 0 ? static_cast<int64_t>(mask) : -1);
  if (deleted)
    LOG(LINFO, ("Comaps++ overlay: deleted", deleted, "files of removed or updated maps"));
  return static_cast<jint>(deleted);
}
}  // extern "C"
