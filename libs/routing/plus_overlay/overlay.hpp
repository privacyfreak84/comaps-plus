#pragma once

// Comaps++ cell-overlay prototype (multi-level Dijkstra / CRP style).
//
// Everything here works on a plain directed graph whose vertices are CoMaps routing segments
// (feature, segment index, direction) and whose edge weights are exactly what
// IndexGraph::GetEdgeList returns (transition + penalties + weight of the target segment),
// reduced to RouteWeight::GetIntegratedWeight(). Because vertices are directed segments, simple
// (two-feature) turn restrictions and U-turn rules are already edges that are missing, so the
// overlay needs no special turn handling. Via-way restrictions are not represented (see tool).
//
// No CoMaps dependencies on purpose: this is the part that would move into the router later.

#include <cstdint>
#include <limits>
#include <vector>

namespace plus_overlay
{
using VertexId = uint32_t;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

struct Graph
{
  uint32_t n = 0;
  std::vector<uint64_t> fwdOff;  // n + 1
  std::vector<VertexId> fwdTo;
  std::vector<double> fwdW;
  std::vector<uint64_t> revOff;  // n + 1
  std::vector<VertexId> revFrom;
  std::vector<double> revW;
  std::vector<float> lat, lon;

  uint64_t NumEdges() const { return fwdTo.size(); }
  void BuildReverse();
  // Sets the weight of edge u->v (all parallel copies) in both directions.
  void SetWeight(VertexId u, VertexId v, double w);
};

// Removes vertices with exactly one incoming and one outgoing edge (road points that are not
// junctions). Exact: chain weights are summed. origId maps a compressed vertex to its raw id.
struct Compressed
{
  Graph g;
  std::vector<uint32_t> origId;
};
Compressed CompressChains(Graph const & raw, std::vector<uint8_t> const & valid);

// Vertices reachable from |seed| and reaching it.
std::vector<uint8_t> StronglyConnectedFrom(Graph const & g, VertexId seed, uint32_t & size);

struct PartitionParams
{
  std::vector<uint32_t> cellSizes;  // max vertices per cell, level 1 first, increasing
  double minBalance = 0.3;           // each half gets at least this share of the vertices
  int directions = 8;                // projection angles tried per bisection
};

struct Partition
{
  std::vector<std::vector<uint32_t>> cell;  // cell[level - 1][vertex]
  std::vector<uint32_t> numCells;           // per level
  size_t Levels() const { return cell.size(); }
};
Partition Bisect(Graph const & g, PartitionParams const & params);

struct LevelOverlay
{
  std::vector<uint64_t> entryOff, exitOff, matOff;  // per cell, numCells + 1
  std::vector<VertexId> entries, exits;
  std::vector<double> mat;                          // per cell: row-major entry x exit
  std::vector<uint32_t> entryIdx, exitIdx;          // per vertex: index in its cell, or kNone
};

struct Overlay
{
  std::vector<LevelOverlay> levels;
};

void BuildBoundaries(Graph const & g, Partition const & p, Overlay & ov);
// Computes all clique matrices bottom-up. Returns seconds per level.
std::vector<double> Customize(Graph const & g, Partition const & p, Overlay & ov, unsigned threads);
// Recomputes only the cells (at every level) that contain any of |touched|. Used after edge
// weights change, e.g. avoided places. Returns number of cells recomputed.
class Scratch;
uint64_t Recustomize(Graph const & g, Partition const & p, Overlay & ov, std::vector<VertexId> const & touched,
                     Scratch & sc);

struct QueryResult
{
  double weight = kInf;
  uint64_t settled = 0;
  std::vector<VertexId> path;  // with wantPath: every vertex from s to t on this graph (overlay hops unpacked)
};

class Scratch
{
public:
  explicit Scratch(uint32_t n);

  struct Side
  {
    std::vector<double> dist;
    std::vector<uint32_t> stamp;
    std::vector<VertexId> parent;
    std::vector<uint8_t> parentLevel;  // 0 = graph edge, k = clique of a level-k cell
    struct Item
    {
      double d;
      VertexId v;
      bool operator>(Item const & o) const { return d > o.d; }
    };
    std::vector<Item> heap;
  };

  Side & Fwd() { return m_side[0]; }
  Side & Bwd() { return m_side[1]; }
  std::vector<uint32_t> & Done(int side) { return m_done[side]; }
  uint32_t NewStamp() { return ++m_stamp; }
  uint32_t Stamp() const { return m_stamp; }

private:
  Side m_side[2];
  std::vector<uint32_t> m_done[2];
  uint32_t m_stamp = 0;
};

QueryResult BidirectionalDijkstra(Graph const & g, VertexId s, VertexId t, Scratch & sc, bool wantPath = false);
QueryResult OverlayQuery(Graph const & g, Partition const & p, Overlay const & ov, VertexId s, VertexId t,
                         Scratch & sc, bool wantPath = false);

// Storage accounting for the overlay (what a device would have to keep next to the map).
struct OverlayStorage
{
  uint64_t matrixEntries = 0;
  uint64_t finiteEntries = 0;
  uint64_t boundaryIds = 0;  // entry + exit vertex ids over all cells and levels
  uint64_t cellIdBytes = 0;  // per vertex cell id per level
};
OverlayStorage MeasureStorage(Graph const & g, Partition const & p, Overlay const & ov);
}  // namespace plus_overlay
