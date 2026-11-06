// Comaps++ overlay builder for one map: a desktop front end for plus_overlay::BuildOverlayFile, the
// same function the app runs on the phone after a map download or update.

#include "routing/plus_overlay/overlay_builder.hpp"
#include "routing/plus_overlay/overlay_file.hpp"

#include "storage/country_parent_getter.hpp"

#include "indexer/classificator_loader.hpp"

#include "platform/local_country_file.hpp"
#include "platform/platform.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <gflags/gflags.h>

DEFINE_string(resources_path, "", "CoMaps data/ directory (classificator, countries.txt).");
DEFINE_string(maps_path, "", "Directory that contains <version>/<mwm>.mwm.");
DEFINE_string(mwm, "India_Uttar Pradesh", "Map to build.");
DEFINE_int64(mwm_version, 261004, "Map version directory.");
DEFINE_uint32(threads, 2, "Threads for partition and shortcuts.");
DEFINE_string(out, "", "Overlay file to write (empty: next to the map, <map>.plusovl).");
DEFINE_bool(check_file, false, "Read the written file back and compare it with what was built.");
DEFINE_double(cancel_after_s, 0, "Test cancellation: cancel the build after this many seconds (0: never).");
DEFINE_bool(cleanup, false, "Only delete overlay files whose map is gone (and interrupted builds) in the version "
                            "directory, then exit.");

int main(int argc, char ** argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << std::unitbuf;
  auto & pl = GetPlatform();
  pl.SetResourceDir(FLAGS_resources_path + "/");
  pl.SetWritableDirForTests(FLAGS_maps_path + "/");
  std::string const dir = FLAGS_maps_path + "/" + std::to_string(FLAGS_mwm_version);
  if (FLAGS_cleanup)
  {
    std::cout << "deleted " << plus_overlay::DeleteOrphanedOverlayFiles(dir) << " files\n";
    return 0;
  }
  classificator::Load();

  platform::LocalCountryFile localFile(dir, platform::CountryFile(FLAGS_mwm), FLAGS_mwm_version);
  auto cpg = std::make_shared<storage::CountryParentGetter>();

  auto const t0 = std::chrono::steady_clock::now();
  plus_overlay::BuildOptions o;
  o.threads = FLAGS_threads;
  o.checkFile = FLAGS_check_file;
  o.log = [](std::string const & s) { std::cout << "  " << s << "\n"; };
  std::atomic<int> lastTen{-1};
  o.progress = [&](double f, std::string const & stage)
  {
    int const ten = static_cast<int>(f * 10);
    int prev = lastTen.load();
    while (ten > prev)
    {
      if (lastTen.compare_exchange_weak(prev, ten))
      {
        std::cout << "  [" << ten * 10 << "%] " << stage << "\n";
        break;
      }
    }
  };
  if (FLAGS_cancel_after_s > 0)
  {
    o.cancelled = [&]()
    {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > FLAGS_cancel_after_s;
    };
  }

  std::cout << "== Comaps++ overlay build: " << FLAGS_mwm << " v" << FLAGS_mwm_version << ", " << FLAGS_threads
            << " threads\n";
  auto const r = plus_overlay::BuildOverlayFile(localFile, [cpg](std::string const & id) { return (*cpg)(id); },
                                                FLAGS_out, o);
  switch (r.status)
  {
  case plus_overlay::BuildResult::Status::Ok:
    std::cout << "  written " << r.path << ": " << r.fileBytes / 1048576.0 << " MB (format v"
              << plus_overlay::kFileFormatVersion << ", map version " << r.mwmVersion << ") in " << r.seconds
              << " s, peak " << r.peakRssKb / 1024 << " MB\n";
    return 0;
  case plus_overlay::BuildResult::Status::Cancelled:
    std::cout << "  cancelled after " << r.seconds << " s; file exists: "
              << Platform::IsFileExistsByFullPath(r.path) << ", tmp exists: "
              << Platform::IsFileExistsByFullPath(r.path + ".tmp") << "\n";
    return 2;
  case plus_overlay::BuildResult::Status::Failed: std::cout << "  failed: " << r.error << "\n"; return 1;
  }
  return 1;
}
