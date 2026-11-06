#pragma once

// Comaps++ per-map overlay file, format v1.
//
// One file per map (<map name>.plusovl next to the .mwm). It holds no road graph: CoMaps' own
// IndexGraph supplies the base edges at query time. Contents:
//   - which directed segments are overlay vertices ("kept": junction ends, border transitions, every
//     segment of a via-way feature), as a bitset over segment slots plus a rank index;
//   - via-way restriction patterns and the automaton-state copies of their segments;
//   - the nested partition (level-1 cell per vertex, parent tables above);
//   - per vertex a role byte (entry/exit of its cell, two bits per level);
//   - per level: entries/exits of every cell and the entry x exit cliques (float32);
//   - the map's border transitions (OSM way id + segment index), used to join maps at query time.
// The header carries the map version and a hash of the build parameters; a file whose map version or
// parameters don't match is treated as missing and rebuilt.

#include "routing/plus_overlay/overlay.hpp"
#include "routing/plus_overlay/via_way_automaton.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace plus_overlay
{
constexpr uint32_t kFileFormatVersion = 1;
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
    float const * mat = nullptr;          // per cell, row-major entry x exit
    std::vector<uint64_t> matOff;         // numCells + 1
  };

  // Null (and |error| set) if the file is missing, damaged or doesn't match the expected map version
  // or build parameters. expectMwmVersion/expectParamsHash of 0 skip that check.
  static std::unique_ptr<MappedOverlay> Open(std::string const & path, int64_t expectMwmVersion,
                                             uint64_t expectParamsHash, std::string * error = nullptr);
  ~MappedOverlay();
  MappedOverlay(MappedOverlay const &) = delete;
  MappedOverlay & operator=(MappedOverlay const &) = delete;

  int64_t MwmVersion() const { return m_mwmVersion; }
  uint64_t FileBytes() const { return m_size; }
  uint32_t NumVertices() const { return m_n; }
  uint32_t BaseN() const { return m_baseN; }
  size_t Levels() const { return m_levels.size(); }
  Level const & GetLevel(size_t li) const { return m_levels[li]; }

  // Segment slots.
  uint32_t NumFeatures() const { return m_numFeatures; }
  bool HasSegment(uint32_t feature, uint32_t segIdx) const
  {
    return feature < m_numFeatures && segIdx < m_featBase[feature + 1] - m_featBase[feature];
  }
  uint32_t Slot(uint32_t feature, uint32_t segIdx, bool forward) const
  {
    return 2 * (m_featBase[feature] + segIdx) + (forward ? 0 : 1);
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
  uint32_t Cell(size_t li, uint32_t v) const
  {
    uint32_t c = m_cell1[v];
    for (size_t k = 1; k <= li; ++k)
      c = m_parents[k][c];
    return c;
  }
  bool IsEntry(size_t li, uint32_t v) const { return (m_roles[v] >> (2 * li)) & 1; }
  bool IsExit(size_t li, uint32_t v) const { return (m_roles[v] >> (2 * li + 1)) & 1; }
  uint32_t EntryIdx(size_t li, uint32_t cell, uint32_t v) const;
  uint32_t ExitIdx(size_t li, uint32_t cell, uint32_t v) const;

  uint32_t NumTransitions() const { return m_numTransitions; }
  FileTransition const & Transition(uint32_t i) const { return m_transitions[i]; }

private:
  MappedOverlay() = default;
  bool Parse(std::string * error);

  int m_fd = -1;
  uint8_t const * m_data = nullptr;
  uint64_t m_size = 0;
  bool m_mapped = false;
  std::vector<uint8_t> m_buffer;  // fallback when mmap is unavailable

  int64_t m_mwmVersion = 0;
  uint32_t m_n = 0, m_baseN = 0, m_numSlots = 0, m_numFeatures = 0, m_numTransitions = 0;
  uint32_t const * m_featBase = nullptr;
  uint64_t const * m_keptBits = nullptr;
  uint32_t const * m_keptRank = nullptr;
  uint32_t m_numWords = 0;
  uint32_t const * m_copyOrig = nullptr;
  uint32_t const * m_copyState = nullptr;
  uint32_t const * m_cell1 = nullptr;
  std::vector<uint32_t const *> m_parents;  // [li] for li >= 1: level-(li-1) cell -> level-li cell
  uint8_t const * m_roles = nullptr;
  FileTransition const * m_transitions = nullptr;
  std::vector<Level> m_levels;
  std::unique_ptr<ViaWayAutomaton> m_automaton;
  std::unordered_map<uint64_t, uint32_t> m_copyId;                    // (state << 32 | base) -> copy
  std::unordered_map<uint32_t, std::vector<uint32_t>> m_copiesOf;     // base -> copies
};
}  // namespace plus_overlay
