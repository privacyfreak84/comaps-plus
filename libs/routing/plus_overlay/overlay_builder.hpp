#pragma once

// Comaps++: builds one map's overlay file on the device (or on a desktop, same code).
//
// No raw segment graph is held in memory: CoMaps' IndexGraph is walked twice (degrees, then chains
// between junctions) to get a compressed graph, via-way restrictions are expanded on it, the map is
// partitioned and the cell cliques computed, and the result is written next to the map as
// <map>.plusovl (see overlay_file.hpp). Measured on a desktop for India_Uttar Pradesh: ~170 s on two
// threads, ~1.1 GB peak memory.

#include "routing_common/vehicle_model.hpp"

#include "platform/local_country_file.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace plus_overlay
{
struct BuildOptions
{
  unsigned threads = 2;
  // Polled regularly (also from worker threads); once it returns true the build stops and nothing is
  // written.
  std::function<bool()> cancelled;
  // Progress in [0, 1] with a short stage name. May be called from the build's worker threads.
  std::function<void(double fraction, std::string const & stage)> progress;
  // Log lines (default: LOG(LINFO)).
  std::function<void(std::string const & line)> log;
  // After writing, read the file back and compare it with what was built (structure and cliques).
  bool checkFile = false;
};

struct BuildResult
{
  enum class Status
  {
    Ok,
    Cancelled,
    Failed,
  };
  Status status = Status::Failed;
  std::string error;
  std::string path;
  int64_t mwmVersion = 0;
  uint64_t fileBytes = 0;
  double seconds = 0;
  uint64_t peakRssKb = 0;  // process peak (VmHWM) at the end of the build; 0 if unknown
};

// Builds the overlay for |localFile| (a downloaded country map) and writes it to |outPath| (empty:
// OverlayPathForMap(the map's path)). The classificator must be loaded. |countryParentNameGetter|
// is the one the router's vehicle model uses (storage's country tree).
BuildResult BuildOverlayFile(platform::LocalCountryFile const & localFile,
                             routing::VehicleModelFactory::CountryParentNameGetterFn const & countryParentNameGetter,
                             std::string const & outPath, BuildOptions const & options);

// Overlay files (*.plusovl) in |dir| whose map (same name, .mwm) is gone. Returns how many were deleted.
size_t DeleteOrphanedOverlayFiles(std::string const & dir);
}  // namespace plus_overlay
