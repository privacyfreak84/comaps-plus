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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>
#include <functional>
#include <string>
#include <vector>

namespace plus_overlay
{
using VertexId = uint32_t;
// "Unreachable"/"blocked". Deliberately finite: CoMaps builds with -ffast-math, under which the
// compiler may assume no value is ever infinite and fold comparisons against infinity away.
// Sums of a few kInf stay finite and compare >= kInf.
constexpr double kInf = 1e18;
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
  unsigned threads = 1;              // subtrees are split on this many threads
  // Optional: polled during the split; once it returns true the partition stops early (incomplete).
  std::function<bool()> cancelled;
  // Optional: share of vertices placed in final cells so far, in [0, 1]. Called from worker threads.
  std::function<void(double)> progress;
};

// The geometric splits of a partition, so that a later version of the same map (new feature ids,
// mostly the same roads) can be divided into the same cells without searching for cuts again.
struct PartitionTree
{
  static constexpr uint8_t kLeaf = 0xFF;
  struct Node
  {
    uint8_t dir = kLeaf;  // projection direction index, kLeaf for a level-1 cell
    uint8_t closes = 0;   // bit li: this node is a cell of level li
    uint16_t reserved = 0;
    float threshold = 0;  // projected value < threshold: left child
    uint32_t left = 0, right = 0;
  };
  static_assert(sizeof(Node) == 16, "");
  std::vector<Node> nodes;
  uint32_t root = 0;
  float k = 1;  // cos(mean latitude): x = lon * k
  int directions = 0;
  std::vector<uint32_t> cellSizes;

  bool Empty() const { return nodes.empty(); }
};
bool SavePartitionTree(PartitionTree const & t, std::string const & path);
bool LoadPartitionTree(std::string const & path, PartitionTree & t);

struct Partition
{
  std::vector<std::vector<uint32_t>> cell;  // cell[level - 1][vertex]
  std::vector<uint32_t> numCells;           // per level
  PartitionTree tree;                       // set by Bisect
  size_t Levels() const { return cell.size(); }
};
Partition Bisect(Graph const & g, PartitionParams const & params);
// Divides |g| with the splits of |tree| (from Bisect on an earlier version of the same map). False (and
// |why| set) if the tree doesn't fit |params| or the cells come out too unbalanced; then use Bisect.
bool ReplayPartition(Graph const & g, PartitionTree const & tree, PartitionParams const & params, Partition & p,
                     std::string * why = nullptr);

struct LevelOverlay
{
  std::vector<uint64_t> entryOff, exitOff, matOff;  // per cell, numCells + 1
  std::vector<VertexId> entries, exits;             // per cell, ascending vertex ids
  std::vector<double> mat;                          // per cell: row-major entry x exit
  std::vector<uint8_t> role;                        // per vertex: bit 1 entry, bit 2 exit (of its cell)

  bool IsEntry(VertexId v) const { return role[v] & 1; }
  bool IsExit(VertexId v) const { return role[v] & 2; }
  // Index of |v| among the entries (exits) of |cell|; |v| must be one.
  uint32_t EntryIdx(VertexId v, uint32_t cell) const
  {
    auto const b = entries.begin() + entryOff[cell], e = entries.begin() + entryOff[cell + 1];
    return static_cast<uint32_t>(std::lower_bound(b, e, v) - b);
  }
  uint32_t ExitIdx(VertexId v, uint32_t cell) const
  {
    auto const b = exits.begin() + exitOff[cell], e = exits.begin() + exitOff[cell + 1];
    return static_cast<uint32_t>(std::lower_bound(b, e, v) - b);
  }
};

struct Overlay
{
  std::vector<LevelOverlay> levels;
};

// Entries/exits of every cell at every level. |extraRole| (optional, per vertex: bit 1 entry, bit 2 exit)
// marks vertices that are boundary at every level, e.g. map border transitions joined to another map.
void BuildBoundaries(Graph const & g, Partition const & p, Overlay & ov,
                     std::vector<uint8_t> const * extraRole = nullptr);
// Computes all clique matrices bottom-up. Returns seconds per level.
// |cancelled| (optional) is polled between cells; once true the remaining cells are skipped.
// |progress| (optional) gets the done share in [0, 1], from worker threads.
std::vector<double> Customize(Graph const & g, Partition const & p, Overlay & ov, unsigned threads,
                              std::function<bool()> const & cancelled = {},
                              std::function<void(double)> const & progress = {});
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

// (vertex, initial distance) pairs: several start or end candidates, as a projected point gives.
using Seeds = std::vector<std::pair<VertexId, double>>;

QueryResult BidirectionalDijkstra(Graph const & g, VertexId s, VertexId t, Scratch & sc, bool wantPath = false);
QueryResult OverlayQuery(Graph const & g, Partition const & p, Overlay const & ov, VertexId s, VertexId t,
                         Scratch & sc, bool wantPath = false);
// Same with several sources and targets; path (if wanted) starts at the source used and ends at the
// target used.
QueryResult OverlayQueryMulti(Graph const & g, Partition const & p, Overlay const & ov, Seeds const & sources,
                              Seeds const & targets, Scratch & sc, bool wantPath = false);

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
