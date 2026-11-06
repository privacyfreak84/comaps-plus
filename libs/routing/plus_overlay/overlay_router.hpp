#pragma once

// Comaps++ overlay routing at query time.
//
// Answers a car route on the overlay files of the maps involved, using CoMaps' own IndexGraph for the
// base edges (the search near the start and finish, edges leaving a cell, and unpacking cells into
// segments). The result is a complete IndexGraphStarter path - fake start segment, real segments,
// fake finish segment - in the same form the A* search produces, so IndexRouter's usual route
// building (RedressRoute: geometry, turns, street names, ETA) applies unchanged.

#include "routing/plus_overlay/overlay_file.hpp"

#include "routing/segment.hpp"

#include "routing_common/num_mwm_id.hpp"

#include <memory>
#include <string>
#include <vector>

namespace routing
{
class IndexGraphStarter;
}

namespace plus_overlay
{
struct OverlayMap
{
  routing::NumMwmId id = 0;
  std::string path;        // the map's overlay file
  int64_t mwmVersion = 0;  // the downloaded map's version; the file must have been built for it
};

class OverlayRouter
{
public:
  explicit OverlayRouter(uint64_t paramsHash);
  ~OverlayRouter();

  // Maps that may take part in a route (normally every downloaded map). Files are opened on first use.
  void SetMaps(std::vector<OverlayMap> const & maps);
  // Opens every map's file; false (and the maps without a usable file in |missing|) unless all are ready.
  bool AllMapsReady(std::vector<routing::NumMwmId> * missing = nullptr);

  enum class Result
  {
    Ok,
    NoRoute,        // the overlay is complete and there is no route
    NotApplicable,  // something the overlay can't answer: the caller should use the normal search
  };

  struct Stats
  {
    double ms = 0;
    double weight = 0;       // integrated weight of the path (fake parts included)
    uint64_t settled = 0;    // vertices settled by the overlay search
    uint64_t baseScans = 0;  // CoMaps edge lists read for base edges
    size_t mapsUsed = 0;
    // Diagnostics: cost the search assigned to the start part (fake + chain up to the first overlay
    // vertex, path[0 .. srcEnd]) and the finish part (path[tgtBegin .. end]).
    double srcCost = 0, tgtCost = 0;
    size_t srcEnd = 0, tgtBegin = 0;
    std::string reason;  // why NotApplicable
  };

  // |path| starts with starter.GetStartSegment() and ends with starter.GetFinishSegment().
  Result FindPath(routing::IndexGraphStarter & starter, std::vector<routing::Segment> & path, Stats & stats);

  // The same physical segment in the neighbouring map, if |exitSegment| leaves its map at a border
  // transition joined to a loaded map (for checking paths).
  bool TwinOfExit(routing::Segment const & exitSegment, routing::Segment & twin);

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};
}  // namespace plus_overlay
