#pragma once

// Comaps++ per-map overlay file, format v2.
//
// One file per map (<map name>.plusovl next to the .mwm). It holds no road graph: CoMaps' own
// IndexGraph supplies the base edges at query time. Contents:
//   - which directed segments are overlay vertices ("kept": junction ends, border transitions, every
//     segment of a via-way feature), as a bitset over segment slots plus a rank index;
//   - via-way restriction patterns and the automaton-state copies of their segments;
//   - the nested partition (level-1 cell per vertex, parent tables above);
//   - per vertex a role byte (entry/exit of its cell, two bits per level);
//   - per level: entries/exits of every cell and the entry x exit cliques, sparse: one bit per
//     (entry, exit) pair (about a third of the pairs are unreachable) and the finite values as 16-bit
//     steps of a per-cell scale, plus a byte of penalty units per value in cells that have any;
//   - the map's border transitions (OSM way id + segment index), used to join maps at query time.
// The header carries the map version and a hash of the build parameters; a file whose map version or
// parameters don't match is treated as missing and rebuilt.

#include "routing/plus_overlay/overlay.hpp"
#include "routing/plus_overlay/via_way_automaton.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace plus_overlay
{
// Keeping prepared files valid across app updates:
// - kFileFormatVersion is the layout this code writes. A new layout must stay readable by adding
//   sections (the reader ignores unknown ones and must treat missing new ones as absent) and raising
//   kFileFormatVersion; kMinReadableFormatVersion goes up only if old files truly can't be read.
// - kRoutingDataVersion names what the cliques were computed from (CoMaps' edge weights, turn
//   penalties, restrictions handling, the overlay model). Raise it only when such a change makes old
//   cliques wrong; that is the only code change that makes the app rebuild every file.
// - The partition parameters (DefaultParamsHash) are fixed; changing them also forces a rebuild.
constexpr uint32_t kFileFormatVersion = 6;  // 4: special vertices; 5: smaller layout; 6: 16-bit clique values
constexpr uint32_t kMinReadableFormatVersion = 6;  // 6: 16-bit clique values, per-cell scales, penalty bytes
constexpr uint32_t kRoutingDataVersion = 2;  // 2: time-dependent edges, penalty units in cliques
constexpr char const kFileExtension[] = ".plusovl";
constexpr size_t kMaxLevels = 4;  // two role bits per level in one byte

#pragma pack(push, 1)
struct FileTransition
{
  uint64_t osmId = 0;    // CrossMwmId (encoded OSM way id) of the transition
  uint32_t local = 0;    // overlay vertex
  uint32_t segIdx = 0;   // segment index on the feature (the same in both maps)
  uint32_t isEnter = 0;  // 1: enters this map, 0: leaves it
  uint32_t reserved = 0;
};
#pragma pack(pop)
static_assert(sizeof(FileTransition) == 24, "");

// Everything the builder hands to the writer besides the partition and the overlay.
struct FileContents
{
  int64_t mwmVersion = 0;
  uint64_t paramsHash = 0;
  std::vector<uint32_t> featBase;  // numFeatures + 1
  std::vector<uint64_t> keptBits;  // per segment slot (2 * segment + (forward ? 0 : 1))
  std::vector<uint32_t> keptRank;  // kept slots before each 64-bit word
  uint32_t numSlots = 0;
  uint32_t baseN = 0;  // kept slots
  uint32_t n = 0;      // baseN + copies
  std::vector<uint32_t> copyOrig, copyState;
  std::vector<std::vector<uint32_t>> patterns;
  std::vector<FileTransition> transitions;
  std::vector<uint32_t> special;  // sorted (overlay.hpp)
  bool timeDependent = false;     // the map has time-dependent edges (dynamic_edges.hpp)
};

// Hash of what makes two builds comparable (format version, cell sizes, balance, directions).
uint64_t BuildParamsHash(std::vector<uint32_t> const & cellSizes, double minBalance, int directions);

// The parameters the app builds and reads files with.
inline std::vector<uint32_t> DefaultCellSizes() { return {256, 4096, 65536, 1048576}; }
constexpr double kDefaultMinBalance = 0.3;
constexpr int kDefaultDirections = 8;
inline uint64_t DefaultParamsHash()
{
  return BuildParamsHash(DefaultCellSizes(), kDefaultMinBalance, kDefaultDirections);
}
// <dir>/<map>.mwm -> <dir>/<map>.plusovl
inline std::string OverlayPathForMap(std::string const & mwmPath)
{
  auto const dot = mwmPath.rfind('.');
  return (dot == std::string::npos ? mwmPath : mwmPath.substr(0, dot)) + kFileExtension;
}
// <dir>/<map>.mwm -> <dir>/<map>.avoid<mask>.plusovl (cliques for routes with avoid options)
inline std::string AvoidOverlayPathForMap(std::string const & mwmPath, uint32_t avoidMask)
{
  auto const dot = mwmPath.rfind('.');
  return (dot == std::string::npos ? mwmPath : mwmPath.substr(0, dot)) + ".avoid" + std::to_string(avoidMask) +
         kFileExtension;
}

// Cheap check (header only): |path| is an overlay file of this format, built for map version
// |mwmVersion| with the build parameters |paramsHash|.
bool OverlayFileMatches(std::string const & path, int64_t mwmVersion, uint64_t paramsHash,
                        uint64_t * buildId = nullptr);
// Header only: |path| is an avoid-option file for the base file with |baseBuildId| and |avoidMask|.
bool AvoidCliquesFileMatches(std::string const & path, uint64_t baseBuildId, uint32_t avoidMask);

// Clique values are stored per cell as travel time in 16-bit steps of the cell's scale (its largest time
// / 65535, so at most half a step of rounding: under a second for any cell whose paths take less than
// 18 hours), plus the path's penalty units (dynamic_edges.hpp) as a byte, saturated at 255. The value a
// search uses is time + units * kCliquePenaltyS, as before.
constexpr float kCliquePenaltyS = 1800.0f;  // == kPenaltyUnitS (checked in overlay_file.cpp)
constexpr uint32_t kMaxCliqueSteps = 65535;
constexpr uint32_t kMaxCliquePenaltyUnits = 255;

class MappedOverlay;
// Cliques computed with avoid options (|avoidMask|, RoutingOptions bits) for the vertices, partition and
// boundaries of |base|. Everything else is shared with the base file.
bool WriteAvoidCliquesFile(std::string const & path, MappedOverlay const & base, uint32_t avoidMask,
                           Overlay const & ov, std::string * error = nullptr);

bool WriteOverlayFile(std::string const & path, FileContents const & fc, Partition const & p, Overlay const & ov,
                      std::string * error = nullptr);

// Read-only view of a file, memory-mapped: only the pages a query touches are read from storage.
class MappedOverlay
{
public:
  struct Level
  {
    uint32_t numCells = 0;
    uint32_t const * entryOff = nullptr;  // numCells + 1
    uint32_t const * exitOff = nullptr;
    uint32_t const * entries = nullptr;   // per cell, ascending
    uint32_t const * exits = nullptr;
    uint32_t const * rowRank = nullptr;   // per entry: values before its row
    uint64_t const * bits = nullptr;      // per cell, per entry row: ceil(exits / 64) words
    uint16_t const * values = nullptr;    // steps of the cell's scale
    uint8_t const * penalty = nullptr;    // penalty units per value of the cells that have any
    uint32_t const * penaltyOff = nullptr;  // numCells + 1: first penalty byte of each cell (none if equal)
    float const * scale = nullptr;        // per cell: seconds per step
    std::vector<uint64_t> wordOff;        // numCells + 1: first word of each cell
    // With an avoid-option file attached, the cells whose cliques differ from these: per cell, kNoAlt or
    // the cell's index in the avoid file's arrays (rows and words of its changed cells, in cell order).
    std::vector<uint32_t> altCell;
    uint32_t const * altRowRank = nullptr;
    uint64_t const * altBits = nullptr;
    uint16_t const * altValues = nullptr;
    uint8_t const * altPenalty = nullptr;
    uint32_t const * altPenaltyOff = nullptr;  // per changed cell + 1
    float const * altScale = nullptr;  // per changed cell
    std::vector<uint32_t> altRowOff;   // per changed cell: its first row in altRowRank
    std::vector<uint64_t> altWordOff;  // per changed cell: its first word in altBits
  };
  static constexpr uint32_t kNoAlt = 0xFFFFFFFFu;

  // One cell's clique rows: rowRank by the cell's entry row (values before it, as an index into values),
  // bits from the cell's first word.
  struct CellCliques
  {
    uint32_t const * rowRank;
    uint64_t const * bits;
    uint16_t const * values;
    uint8_t const * penalty;  // the cell's own penalty bytes, from its first value; null: no penalties
    uint64_t firstValue;      // index of the cell's first value
    float scale;
    uint32_t Units(uint64_t idx) const { return penalty ? penalty[idx - firstValue] : 0; }
    float Time(uint64_t idx) const { return static_cast<float>(values[idx]) * scale; }
    // What a search adds: travel time plus penalties.
    float Weight(uint64_t idx) const { return Time(idx) + static_cast<float>(Units(idx)) * kCliquePenaltyS; }
  };
  static CellCliques CliquesOf(Level const & L, uint32_t cell)
  {
    if (!L.altCell.empty() && L.altCell[cell] != kNoAlt)
    {
      uint32_t const k = L.altCell[cell];
      uint32_t const * rr = L.altRowRank + L.altRowOff[k];
      bool const pen = L.altPenaltyOff[k + 1] > L.altPenaltyOff[k];
      return {rr, L.altBits + L.altWordOff[k], L.altValues, pen ? L.altPenalty + L.altPenaltyOff[k] : nullptr,
              L.entryOff[cell + 1] > L.entryOff[cell] ? rr[0] : 0, L.altScale[k]};
    }
    uint32_t const * rr = L.rowRank + L.entryOff[cell];
    bool const pen = L.penaltyOff[cell + 1] > L.penaltyOff[cell];
    return {rr, L.bits + L.wordOff[cell], L.values, pen ? L.penalty + L.penaltyOff[cell] : nullptr,
            L.entryOff[cell + 1] > L.entryOff[cell] ? rr[0] : 0, L.scale[cell]};
  }
  // Calls f(vertex, weight) or f(vertex, weight, penalty units), whichever |f| takes.
  template <class F>
  static void CallClique(F && f, uint32_t vertex, CellCliques const & cc, uint64_t idx)
  {
    if constexpr (std::is_invocable_v<F, uint32_t, float, uint32_t>)
      f(vertex, cc.Weight(idx), cc.Units(idx));
    else
      f(vertex, cc.Weight(idx));
  }

  // Null (and |error| set) if the file is missing, damaged or doesn't match the expected map version
  // or build parameters. expectMwmVersion/expectParamsHash of 0 skip that check.
  // |damaged| (optional) is set when the file exists but its contents are inconsistent (damaged on
  // storage); not for a missing file or one built for another map version or parameters.
  static std::unique_ptr<MappedOverlay> Open(std::string const & path, int64_t expectMwmVersion,
                                             uint64_t expectParamsHash, std::string * error = nullptr,
                                             bool * damaged = nullptr);
  bool AttachAvoidCliques(std::string const & path, uint32_t avoidMask, std::string * error, bool * damaged);
  ~MappedOverlay();
  MappedOverlay(MappedOverlay const &) = delete;
  MappedOverlay & operator=(MappedOverlay const &) = delete;

  int64_t MwmVersion() const { return m_mwmVersion; }
  uint64_t FileBytes() const { return m_file.size; }
  uint64_t AvoidFileBytes() const { return m_avoidFile.size; }
  uint64_t ParamsHash() const { return m_paramsHash; }
  uint64_t BuildId() const { return m_buildId; }
  uint32_t AvoidMask() const { return m_avoidMask; }

  // Replaces the cliques with those of an avoid-option file built for this file and |avoidMask|
  // (WriteAvoidCliquesFile). False (and nothing changed) if it is missing or doesn't match.
  bool AttachAvoidCliques(std::string const & path, uint32_t avoidMask, std::string * error = nullptr);

  // Raw tables (for rebuilding cliques with other edge weights).
  uint64_t const * KeptBitsData() const { return m_keptBits; }
  uint32_t const * KeptRankData() const { return m_keptRank; }
  uint32_t NumKeptWords() const { return m_numWords; }
  uint32_t NumVertices() const { return m_n; }
  uint32_t BaseN() const { return m_baseN; }
  size_t Levels() const { return m_levels.size(); }
  Level const & GetLevel(size_t li) const { return m_levels[li]; }

  // Segment slots.
  uint32_t NumFeatures() const { return m_numFeatures; }
  uint32_t FeatBase(uint32_t feature) const { return m_featBlock[feature >> 6] + m_featLow[feature]; }
  bool HasSegment(uint32_t feature, uint32_t segIdx) const
  {
    return feature < m_numFeatures && segIdx < FeatBase(feature + 1) - FeatBase(feature);
  }
  uint32_t Slot(uint32_t feature, uint32_t segIdx, bool forward) const
  {
    return 2 * (FeatBase(feature) + segIdx) + (forward ? 0 : 1);
  }
  bool Kept(uint32_t slot) const { return (m_keptBits[slot >> 6] >> (slot & 63)) & 1; }
  uint32_t Rank(uint32_t slot) const
  {
    uint64_t const word = m_keptBits[slot >> 6] & ((uint64_t(1) << (slot & 63)) - 1);
    return m_keptRank[slot >> 6] + static_cast<uint32_t>(__builtin_popcountll(word));
  }
  // Base vertex -> slot (copies: of their original).
  uint32_t SlotOf(uint32_t local) const;
  // Slot -> (feature, segment index, forward).
  void Decode(uint32_t slot, uint32_t & feature, uint32_t & segIdx, bool & forward) const;
  uint32_t FeatureOfLocal(uint32_t local) const;

  // Via-way copies.
  bool IsCopy(uint32_t local) const { return local >= m_baseN; }
  uint32_t CopyOrig(uint32_t local) const { return m_copyOrig[local - m_baseN]; }
  uint32_t CopyState(uint32_t local) const { return m_copyState[local - m_baseN]; }
  // kNone if there is no copy of |base| for |state|.
  uint32_t CopyId(uint32_t state, uint32_t base) const;
  std::vector<uint32_t> const * CopiesOf(uint32_t base) const;
  ViaWayAutomaton const * Automaton() const { return m_automaton.get(); }

  // Partition and roles.
  // Level-1 cells are stored as runs: consecutive vertices mostly share a cell.
  uint32_t Cell1(uint32_t v) const
  {
    uint64_t const upTo = ~uint64_t(0) >> (63 - (v & 63));  // bits 0..v of the word
    uint32_t const run =
        m_runRank[v >> 6] + static_cast<uint32_t>(__builtin_popcountll(m_runStarts[v >> 6] & upTo)) - 1;
    return m_cell16 ? reinterpret_cast<uint16_t const *>(m_runCells)[run]
                    : reinterpret_cast<uint32_t const *>(m_runCells)[run];
  }
  uint32_t Cell(size_t li, uint32_t v) const
  {
    uint32_t c = Cell1(v);
    for (size_t k = 1; k <= li; ++k)
      c = m_parents[k][c];
    return c;
  }
  bool IsEntry(size_t li, uint32_t v) const
  {
    auto const & L = m_levels[li];
    uint32_t const c = Cell(li, v);
    return std::binary_search(L.entries + L.entryOff[c], L.entries + L.entryOff[c + 1], v);
  }
  bool IsExit(size_t li, uint32_t v) const
  {
    auto const & L = m_levels[li];
    uint32_t const c = Cell(li, v);
    return std::binary_search(L.exits + L.exitOff[c], L.exits + L.exitOff[c + 1], v);
  }
  uint32_t EntryIdx(size_t li, uint32_t cell, uint32_t v) const;
  uint32_t ExitIdx(size_t li, uint32_t cell, uint32_t v) const;

  // A cell scale is finite and >= 0 unless the file was damaged on storage; files with a damaged scale are
  // rejected when opened, so a search can't loop (a negative cycle) or get poisoned distances. Bit test,
  // because the build uses -ffast-math (NaN comparisons may be folded away).
  static bool IsUsableWeight(float w)
  {
    uint32_t b;
    std::memcpy(&b, &w, sizeof(b));
    return b < 0x7F800000u;  // non-negative and finite
  }

  // Clique weight from entry #i to exit #j of |cell| (>= 1e29 when unreachable).
  float Clique(size_t li, uint32_t cell, uint32_t i, uint32_t j) const;
  // Seconds per stored step in |cell| (the rounding of its values is at most half of it).
  float CellScale(size_t li, uint32_t cell) const { return CliquesOf(m_levels[li], cell).scale; }
  // f(exit vertex, weight) for every exit of |cell| reachable from its entry |v|.
  template <class F>
  void ForEachFromEntry(size_t li, uint32_t cell, uint32_t v, F && f) const
  {
    auto const & L = m_levels[li];
    uint64_t const nEx = L.exitOff[cell + 1] - L.exitOff[cell];
    uint64_t const wpr = (nEx + 63) / 64;
    uint32_t const i = EntryIdx(li, cell, v);
    if (i >= L.entryOff[cell + 1] - L.entryOff[cell] || L.entries[L.entryOff[cell] + i] != v)
      return;  // not an entry of that cell: damaged file
    auto const cc = CliquesOf(L, cell);
    uint64_t const row = i * wpr;
    uint64_t idx = cc.rowRank[i];
    uint32_t const * exits = L.exits + L.exitOff[cell];
    for (uint64_t k = 0; k < wpr; ++k)
      for (uint64_t w = cc.bits[row + k]; w != 0; w &= w - 1)
        CallClique(f, exits[k * 64 + static_cast<uint64_t>(__builtin_ctzll(w))], cc, idx++);
  }
  // f(entry vertex, weight) for every entry of |cell| that reaches its exit |v|.
  template <class F>
  void ForEachToExit(size_t li, uint32_t cell, uint32_t v, F && f) const
  {
    auto const & L = m_levels[li];
    uint64_t const nEn = L.entryOff[cell + 1] - L.entryOff[cell];
    uint64_t const nEx = L.exitOff[cell + 1] - L.exitOff[cell];
    uint64_t const wpr = (nEx + 63) / 64;
    uint32_t const j = ExitIdx(li, cell, v);
    if (j >= nEx || L.exits[L.exitOff[cell] + j] != v)
      return;  // not an exit of that cell: damaged file
    uint64_t const k = j / 64, bit = j % 64;
    uint64_t const below = (uint64_t(1) << bit) - 1;
    uint32_t const * entries = L.entries + L.entryOff[cell];
    auto const cc = CliquesOf(L, cell);
    for (uint64_t i = 0; i < nEn; ++i)
    {
      uint64_t const row = i * wpr;
      uint64_t const word = cc.bits[row + k];
      if (!((word >> bit) & 1))
        continue;
      uint64_t idx = cc.rowRank[i];
      for (uint64_t q = 0; q < k; ++q)
        idx += static_cast<uint64_t>(__builtin_popcountll(cc.bits[row + q]));
      idx += static_cast<uint64_t>(__builtin_popcountll(word & below));
      CallClique(f, entries[i], cc, idx);
    }
  }

  // Special vertices (overlay.hpp), sorted; open cells per level (OpenCells), computed once the levels are
  // parsed.
  std::vector<uint32_t> const & Special() const { return m_special; }
  bool IsSpecial(uint32_t v) const { return std::binary_search(m_special.begin(), m_special.end(), v); }
  bool IsOpen(size_t li, uint32_t cell) const { return !m_open[li].empty() && m_open[li][cell]; }
  bool HasTimeDependentEdges() const { return m_timeDependent; }

  uint32_t NumTransitions() const { return m_numTransitions; }
  FileTransition const & Transition(uint32_t i) const { return m_transitions[i]; }

private:
  MappedOverlay() = default;
  bool Parse(std::string * error);

  struct Mapping
  {
    int fd = -1;
    uint8_t const * data = nullptr;
    uint64_t size = 0;
    bool mapped = false;
    std::vector<uint8_t> buffer;  // fallback when mmap is unavailable
    bool Open(std::string const & path, std::string * error);
    ~Mapping();
  };
  Mapping m_file;
  Mapping m_avoidFile;  // AttachAvoidCliques
  uint64_t m_paramsHash = 0;
  uint64_t m_buildId = 0;
  uint32_t m_avoidMask = 0;

  int64_t m_mwmVersion = 0;
  uint32_t m_n = 0, m_baseN = 0, m_numSlots = 0, m_numFeatures = 0, m_numTransitions = 0;
  uint32_t const * m_featBlock = nullptr;
  uint16_t const * m_featLow = nullptr;
  bool m_cell16 = false;
  uint64_t const * m_keptBits = nullptr;
  uint32_t const * m_keptRank = nullptr;
  uint32_t m_numWords = 0;
  uint32_t const * m_copyOrig = nullptr;
  uint32_t const * m_copyState = nullptr;
  uint64_t const * m_runStarts = nullptr;  // bit per vertex: a run of equal level-1 cells starts here
  uint32_t const * m_runRank = nullptr;    // per word of m_runStarts: runs started before it
  uint8_t const * m_runCells = nullptr;    // uint16 or uint32 per run: its level-1 cell
  uint64_t m_numRuns = 0;
  std::vector<uint32_t const *> m_parents;  // [li] for li >= 1: level-(li-1) cell -> level-li cell
  FileTransition const * m_transitions = nullptr;
  std::vector<uint32_t> m_special;
  bool m_timeDependent = false;
  std::vector<std::vector<uint8_t>> m_open;
  std::vector<Level> m_levels;
  std::unique_ptr<ViaWayAutomaton> m_automaton;
  std::unordered_map<uint64_t, uint32_t> m_copyId;                    // (state << 32 | base) -> copy
  std::unordered_map<uint32_t, std::vector<uint32_t>> m_copiesOf;     // base -> copies
};
}  // namespace plus_overlay
