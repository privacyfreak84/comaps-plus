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

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace routing
{
class IndexGraphStarter;
}

namespace plus_overlay
{
// Which engine answered the last car route, for the app's diagnostics (any thread).
struct EngineReport
{
  enum class Engine
  {
    None,     // no car route yet
    Overlay,  // every leg from the overlay files
    Rejoin,   // CoMaps' short rejoin search after leaving the route (AdjustRoute)
    Normal,   // CoMaps' normal search for at least one leg
    Approximate,  // overlay, but crossing maps whose overlay isn't built yet on CoMaps' border times
  };
  Engine engine = Engine::None;
  std::vector<std::string> approxMaps;  // Approximate: those maps (country ids)
  std::string reason;  // why the normal search ran
  double seconds = 0;
};
void SetLastEngineReport(EngineReport const & r);
EngineReport GetLastEngineReport();

struct OverlayMap
{
  routing::NumMwmId id = 0;
  std::string path;        // the map's overlay file
  int64_t mwmVersion = 0;  // the downloaded map's version; the file must have been built for it
  // With avoid options (avoidMask != 0): the map's avoid-option cliques for that mask, which must
  // belong to |path| (AvoidOverlayPathForMap). Without that file the map counts as having no overlay.
  uint32_t avoidMask = 0;
  std::string avoidPath;
};

class OverlayRouter
{
public:
  explicit OverlayRouter(uint64_t paramsHash);
  ~OverlayRouter();

  // Maps that may take part in a route (normally every downloaded map). A map's file is opened only
  // when a query first reaches that map. Maps without a usable file are crossed approximately (see
  // Stats::approxMaps).
  void SetMaps(std::vector<OverlayMap> const & maps);
  // Opens every map's file (tools, tests); false (and the maps without a usable file in |missing|)
  // unless all are ready.
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
    double fakeMs = 0, seedMs = 0, searchMs = 0, unpackMs = 0;  // phases
    // Maps without a usable overlay the route crosses: there it follows CoMaps' border-to-border times
    // (like CoMaps' long-route mode), so the route is approximate until their files are built.
    std::vector<routing::NumMwmId> approxMaps;
    std::string reason;  // why NotApplicable
    bool hasMissingMap = false;        // NotApplicable because the route may pass through a map
    routing::NumMwmId missingMap = 0;  // without a usable overlay file: this one
  };

  // |path| starts with starter.GetStartSegment() and ends with starter.GetFinishSegment().
  Result FindPath(routing::IndexGraphStarter & starter, std::vector<routing::Segment> & path, Stats & stats);

  // Finds a leg inside a map without overlay (from an end to a border transition, between two border
  // transitions, or end to end), as [from, ..., to] with its weight; with |anyMap| the leg may cross
  // other maps. IndexRouter sets CoMaps' own joint search here; without it a plain A* on segments is used.
  using LegRefiner = std::function<bool(routing::Segment const & from, routing::Segment const & to, bool anyMap,
                                        std::vector<routing::Segment> & path, double & weight)>;
  void SetLegRefiner(LegRefiner refiner);

  // The same physical segment in the neighbouring map, if |exitSegment| leaves its map at a border
  // transition joined to a loaded map (for checking paths).
  bool TwinOfExit(routing::Segment const & exitSegment, routing::Segment & twin);

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};
}  // namespace plus_overlay
