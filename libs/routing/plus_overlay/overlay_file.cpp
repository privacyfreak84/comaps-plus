#include "routing/plus_overlay/overlay_file.hpp"

#include "routing/plus_overlay/dynamic_edges.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <cstdio>
#include <cstring>
#include <fstream>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace plus_overlay
{
namespace
{
// Fixed; the format version is in the header.
constexpr char kMagic[8] = {'P', 'L', 'U', 'S', 'O', 'V', 'L', '3'};
constexpr size_t kMaxSections = 64;

enum Section : uint32_t
{
  kFeatBlock = 0,  // u32 per 64 features: first segment of the block's first feature
  kKeptBits,
  kKeptRank,
  kCopyOrig,
  kCopyState,
  kPatterns,
  kCell1,          // format <= 4: level-1 cell per vertex (now kRunStarts/kRunRank/kRunCells)
  kParents,
  kRoles,          // format <= 4: role byte per vertex (now derived from the boundary lists)
  kTransitions,
  kFeatLow,        // u16 per feature (+1): first segment relative to its block
  kSpecial,        // u32, ascending: special vertices (overlay.hpp); format 4 on, absent means none
  kRunStarts,      // u64 bits, one per vertex: a run of equal level-1 cells starts at this vertex
  kRunRank,        // u32 per kRunStarts word: runs started before that word
  kRunCells,       // u16 (kCell1Is16Bit) or u32 per run: the run's level-1 cell
  kCellRoads,      // u8 per level-1 cell: RoutingOptions road types inside it (format 7; read when the three
                   // sections below are missing)
  kLevelBase = 16,  // + kLevelSections * level + LevelSection
  // Format 8 on (after the level sections, kLevelBase + kLevelSections * kMaxLevels):
  kCellPathRoads = 56,  // u8 per cell, level 1 first: road types the cell's clique paths use, leaving out
                        // each path's own entry and exit (LevelOverlay::cellRoads)
  kEntryRoads,          // u8 per entry of every cell, level 1 first: road types of the entry's own segment
                        // (format 8), packed with those its row's paths rely on (format 9, PackBoundaryRoads)
  kExitRoads,           // u8 per exit, likewise (its column's paths)
};

enum LevelSection : uint32_t
{
  kEntryOff = 0,  // u32 per cell + 1
  kExitOff,       // u32 per cell + 1
  kEntries,       // u32, per cell ascending
  kExits,         // u32, per cell ascending
  kRowRank,       // u32 per entry: finite clique values before its row (in the level)
  kBits,          // u64: per cell, entry rows of ceil(exits / 64) words; bit set = finite
  kValues,        // u16: the finite clique values, row by row, in steps of the cell's scale
  kScale,         // float32 per cell (avoid files: per changed cell): seconds per step
  kPenalty,       // u8 per value of the cells that have penalties: penalty units
  kPenaltyOff,    // u32 per cell + 1 (avoid files: per changed cell + 1): first byte in kPenalty
  kLevelSections
};

enum HeaderFlags : uint32_t
{
  kCell1Is16Bit = 1,
  kCliquesOnly = 2,  // an avoid-option file: only the per-level clique sections, for one base file
  kTimeDependent = 4,  // the map has time-dependent edges (conditional access): routes search forward only
  kDeltaCliques = 8,   // an avoid-option file holding only the cells whose cliques differ from the base file;
                       // per level, kEntries lists those cells (ascending) and the clique sections hold
                       // their rows only
};

#pragma pack(push, 1)
struct Header
{
  char magic[8];
  uint32_t formatVersion;
  uint32_t numLevels;
  int64_t mwmVersion;
  uint64_t paramsHash;
  uint32_t numSlots;
  uint32_t baseN;
  uint32_t n;
  uint32_t numFeatures;
  uint32_t numCells[8];
  uint32_t flags;
  uint64_t buildId;      // random per build of a base file; avoid-option files name their base file by it
  uint64_t baseBuildId;  // avoid-option files: buildId of the base file
  uint32_t avoidMask;    // avoid-option files: RoutingOptions bits the cliques were computed without
  uint32_t reserved[1];
  uint64_t offset[kMaxSections];
  uint64_t size[kMaxSections];
};
#pragma pack(pop)

// Every row's rank equals the values before it, and all values are used: row reads stay in bounds. Each
// cell's penalty bytes are none or one per value.
bool CliquesConsistent(MappedOverlay::Level const & L, uint32_t const * rowRank, uint64_t const * bits,
                       uint64_t nValues, uint32_t const * penaltyOff, uint64_t nPenalty)
{
  uint64_t seen = 0, row = 0;
  if (penaltyOff[0] != 0)
    return false;
  for (uint32_t c = 0; c < L.numCells; ++c)
  {
    uint64_t const cellFirst = seen;
    uint64_t const nEn = L.entryOff[c + 1] - L.entryOff[c];
    uint64_t const nEx = L.exitOff[c + 1] - L.exitOff[c];
    uint64_t const wpr = (nEx + 63) / 64;
    for (uint64_t i = 0; i < nEn; ++i, ++row)
    {
      if (rowRank[row] != seen)
        return false;
      uint64_t const w0 = L.wordOff[c] + i * wpr;
      if (nEx % 64 != 0 && (bits[w0 + wpr - 1] >> (nEx % 64)) != 0)
        return false;  // bits past the last exit
      for (uint64_t k = 0; k < wpr; ++k)
        seen += static_cast<uint64_t>(__builtin_popcountll(bits[w0 + k]));
    }
    uint64_t const pen = uint64_t(penaltyOff[c + 1]) - penaltyOff[c];
    if (penaltyOff[c + 1] < penaltyOff[c] || (pen != 0 && pen != seen - cellFirst))
      return false;
  }
  return seen == nValues && penaltyOff[L.numCells] == nPenalty;
}

bool IsReadableVersion(uint32_t v) { return v >= kMinReadableFormatVersion && v <= kFileFormatVersion; }

uint64_t Fnv(uint64_t h, void const * data, size_t len)
{
  auto const * p = static_cast<uint8_t const *>(data);
  for (size_t i = 0; i < len; ++i)
  {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  return h;
}

void SetError(std::string * error, std::string const & msg)
{
  if (error)
    *error = msg;
}
}  // namespace

uint64_t BuildParamsHash(std::vector<uint32_t> const & cellSizes, double minBalance, int directions)
{
  uint64_t h = 1469598103934665603ULL;
  // Frozen at 3 (the format version when the hash was introduced): the hash names the routing data in a
  // file, not its layout, so a new layout doesn't make existing files stale.
  uint32_t const v = 3;
  h = Fnv(h, &v, sizeof(v));
  if (kRoutingDataVersion != 1)
  {
    uint32_t const d = kRoutingDataVersion;
    h = Fnv(h, &d, sizeof(d));
  }
  h = Fnv(h, cellSizes.data(), cellSizes.size() * sizeof(uint32_t));
  h = Fnv(h, &minBalance, sizeof(minBalance));
  h = Fnv(h, &directions, sizeof(directions));
  return h;
}

namespace
{
static_assert(kCliquePenaltyS == kPenaltyUnitS, "");

// Sparse cliques of one level: a bit per (entry, exit), rows word-aligned, and the finite values in row order
// as steps of a per-cell scale, with their penalty units (penalty empty if all are zero).
struct EncodedCliques
{
  std::vector<uint32_t> rowRank;
  std::vector<uint64_t> bits;
  std::vector<uint16_t> values;
  std::vector<uint8_t> penalty;      // the cells with penalties only
  std::vector<uint32_t> penaltyOff;  // numCells + 1
  std::vector<float> scale;  // per cell
  std::vector<uint64_t> cellValue;  // numCells + 1: first value of each cell
};

bool EncodeCliques(LevelOverlay const & L, uint32_t numCells, EncodedCliques & e, std::string * error)
{
  e = {};
  e.rowRank.reserve(L.entries.size());
  e.scale.reserve(numCells);
  e.cellValue.reserve(numCells + 1);
  std::vector<double> time;
  std::vector<uint8_t> cellPenalty;
  for (uint32_t c = 0; c < numCells; ++c)
  {
    e.cellValue.push_back(e.values.size());
    e.penaltyOff.push_back(static_cast<uint32_t>(e.penalty.size()));
    cellPenalty.clear();
    bool anyPenalty = false;
    uint64_t const nEn = L.entryOff[c + 1] - L.entryOff[c], nEx = L.exitOff[c + 1] - L.exitOff[c];
    uint64_t const wpr = (nEx + 63) / 64;
    // Travel time of each finite value (the weight without its penalties), and the cell's largest.
    time.clear();
    double maxTime = 0;
    for (uint64_t q = 0; q < nEn * nEx; ++q)
    {
      double const w = L.mat[L.matOff[c] + q];
      if (w >= kInf)
        continue;
      uint32_t const k = std::min<uint32_t>(L.matK.empty() ? 0 : L.matK[L.matOff[c] + q], kMaxCliquePenaltyUnits);
      double const t = std::max(0.0, w - k * double(kCliquePenaltyS));
      time.push_back(t);
      maxTime = std::max(maxTime, t);
    }
    float const scale = static_cast<float>(maxTime / kMaxCliqueSteps);
    e.scale.push_back(scale);
    size_t ti = 0;
    for (uint64_t i = 0; i < nEn; ++i)
    {
      if (e.values.size() > 0xFFFFFFFFULL)
      {
        SetError(error, "too many clique values in one level");
        return false;
      }
      e.rowRank.push_back(static_cast<uint32_t>(e.values.size()));
      uint64_t const w0 = e.bits.size();
      e.bits.resize(w0 + wpr, 0);
      for (uint64_t j = 0; j < nEx; ++j)
      {
        uint64_t const q = i * nEx + j;
        if (L.mat[L.matOff[c] + q] >= kInf)
          continue;
        e.bits[w0 + j / 64] |= uint64_t(1) << (j % 64);
        double const steps = scale > 0 ? time[ti++] / scale : (++ti, 0.0);
        e.values.push_back(static_cast<uint16_t>(std::min<double>(std::llround(steps), kMaxCliqueSteps)));
        uint32_t const k = std::min<uint32_t>(L.matK.empty() ? 0 : L.matK[L.matOff[c] + q], kMaxCliquePenaltyUnits);
        cellPenalty.push_back(static_cast<uint8_t>(k));
        anyPenalty |= k != 0;
      }
    }
    if (anyPenalty)
    {
      if (e.penalty.size() + cellPenalty.size() > 0xFFFFFFFFULL)
      {
        SetError(error, "too many penalty values in one level");
        return false;
      }
      e.penalty.insert(e.penalty.end(), cellPenalty.begin(), cellPenalty.end());
    }
  }
  e.cellValue.push_back(e.values.size());
  e.penaltyOff.push_back(static_cast<uint32_t>(e.penalty.size()));
  return true;
}

// Writes |path|.tmp section by section, then renames it to |path|.
class SectionWriter
{
public:
  SectionWriter(std::string const & path, Header & h) : m_path(path), m_tmp(path + ".tmp"), m_h(h)
  {
    m_out.open(m_tmp, std::ios::binary | std::ios::trunc);
    if (m_out)
      m_out.write(reinterpret_cast<char const *>(&m_h), sizeof(m_h));
    m_pos = sizeof(m_h);
  }
  bool IsOpen() const { return static_cast<bool>(m_out); }
  void Put(uint32_t id, void const * data, uint64_t bytes)
  {
    static char const zeros[8] = {};
    uint64_t const pad = (8 - m_pos % 8) % 8;
    m_out.write(zeros, static_cast<std::streamsize>(pad));
    m_pos += pad;
    m_h.offset[id] = m_pos;
    m_h.size[id] = bytes;
    if (bytes)
      m_out.write(static_cast<char const *>(data), static_cast<std::streamsize>(bytes));
    m_pos += bytes;
  }
  template <class V>
  void PutVec(uint32_t id, V const & v)
  {
    Put(id, v.data(), v.size() * sizeof(v[0]));
  }
  bool Finish(std::string * error)
  {
    m_out.seekp(0);
    m_out.write(reinterpret_cast<char const *>(&m_h), sizeof(m_h));
    m_out.close();
    if (!m_out)
    {
      SetError(error, "write failed");
      std::remove(m_tmp.c_str());
      return false;
    }
    if (std::rename(m_tmp.c_str(), m_path.c_str()) != 0)
    {
      SetError(error, "rename failed");
      std::remove(m_tmp.c_str());
      return false;
    }
    return true;
  }

private:
  std::string m_path, m_tmp;
  Header & m_h;
  std::ofstream m_out;
  uint64_t m_pos = 0;
};

uint64_t NewBuildId()
{
  std::random_device rd;
  uint64_t const t = static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
  uint64_t id = (uint64_t(rd()) << 32) ^ rd() ^ (t * 0x9E3779B97F4A7C15ULL);
  return id == 0 ? 1 : id;
}
}  // namespace

bool WriteOverlayFile(std::string const & path, FileContents const & fc, Partition const & p, Overlay const & ov,
                      std::string * error)
{
  size_t const levels = p.Levels();
  if (levels == 0 || levels > kMaxLevels || ov.levels.size() != levels)
  {
    SetError(error, "unsupported number of levels");
    return false;
  }
  Header h;
  std::memset(&h, 0, sizeof(h));
  std::memcpy(h.magic, kMagic, sizeof(kMagic));
  h.formatVersion = kFileFormatVersion;
  h.numLevels = static_cast<uint32_t>(levels);
  h.mwmVersion = fc.mwmVersion;
  h.paramsHash = fc.paramsHash;
  h.numSlots = fc.numSlots;
  h.baseN = fc.baseN;
  h.n = fc.n;
  h.numFeatures = static_cast<uint32_t>(fc.featBase.size() - 1);
  if (fc.featBase.empty() || static_cast<uint64_t>(kLevelBase) + kLevelSections * levels > kMaxSections)
  {
    SetError(error, "bad contents");
    return false;
  }
  for (size_t li = 0; li < levels; ++li)
    h.numCells[li] = p.numCells[li];
  h.buildId = NewBuildId();

  SectionWriter w(path, h);
  if (!w.IsOpen())
  {
    SetError(error, "can't create " + path + ".tmp");
    return false;
  }
  auto putVec = [&](uint32_t id, auto const & v) { w.PutVec(id, v); };

  // Feature -> first segment, as a u32 per 64 features plus a u16 offset per feature.
  {
    uint64_t const nf = fc.featBase.size();  // numFeatures + 1
    std::vector<uint32_t> block((nf + 63) / 64);
    std::vector<uint16_t> low(nf);
    for (uint64_t f = 0; f < nf; ++f)
    {
      if (f % 64 == 0)
        block[f / 64] = fc.featBase[f];
      uint64_t const d = fc.featBase[f] - block[f / 64];
      if (d > 0xFFFF)
      {
        SetError(error, "a block of 64 features has more than 65535 road segments");
        return false;
      }
      low[f] = static_cast<uint16_t>(d);
    }
    putVec(kFeatBlock, block);
    putVec(kFeatLow, low);
  }
  putVec(kKeptBits, fc.keptBits);
  putVec(kKeptRank, fc.keptRank);
  putVec(kCopyOrig, fc.copyOrig);
  putVec(kCopyState, fc.copyState);
  std::vector<uint32_t> pat{static_cast<uint32_t>(fc.patterns.size())};
  for (auto const & pt : fc.patterns)
  {
    pat.push_back(static_cast<uint32_t>(pt.size()));
    pat.insert(pat.end(), pt.begin(), pt.end());
  }
  putVec(kPatterns, pat);
  if (fc.timeDependent)
    h.flags |= kTimeDependent;
  {
    // Level-1 cells as runs of consecutive vertices.
    std::vector<uint64_t> starts((fc.n + 63) / 64, 0);
    std::vector<uint32_t> runCells;
    for (uint32_t v = 0; v < fc.n; ++v)
      if (v == 0 || p.cell[0][v] != p.cell[0][v - 1])
      {
        starts[v >> 6] |= uint64_t(1) << (v & 63);
        runCells.push_back(p.cell[0][v]);
      }
    std::vector<uint32_t> rank(starts.size(), 0);
    uint32_t seen = 0;
    for (size_t w = 0; w < starts.size(); ++w)
    {
      rank[w] = seen;
      seen += static_cast<uint32_t>(__builtin_popcountll(starts[w]));
    }
    putVec(kRunStarts, starts);
    putVec(kRunRank, rank);
    if (p.numCells[0] <= 0x10000)
    {
      h.flags |= kCell1Is16Bit;
      std::vector<uint16_t> c16(runCells.begin(), runCells.end());
      putVec(kRunCells, c16);
    }
    else
    {
      putVec(kRunCells, runCells);
    }
  }
  // Parent tables: for each level >= 1, the level-li cell of every level-(li-1) cell.
  std::vector<uint32_t> parents;
  for (size_t li = 1; li < levels; ++li)
  {
    std::vector<uint32_t> par(p.numCells[li - 1], kNone);
    for (uint32_t v = 0; v < fc.n; ++v)
      par[p.cell[li - 1][v]] = p.cell[li][v];
    parents.insert(parents.end(), par.begin(), par.end());
  }
  putVec(kParents, parents);
  putVec(kTransitions, fc.transitions);
  putVec(kSpecial, fc.special);
  {
    std::vector<uint8_t> cellRoads(p.numCells[0], 0);
    if (fc.roads.size() != fc.n)
    {
      SetError(error, "road types missing");
      return false;
    }
    for (uint32_t v = 0; v < fc.n; ++v)
      cellRoads[p.cell[0][v]] |= fc.roads[v];
    putVec(kCellRoads, cellRoads);
  }
  if (fc.selfRoads.size() == fc.n &&
      std::all_of(ov.levels.begin(), ov.levels.end(),
                  [&](LevelOverlay const & L)
                  {
                    return L.cellRoads.size() == L.entryOff.size() - 1 &&
                           L.entryPathRoads.size() == L.entries.size() && L.exitPathRoads.size() == L.exits.size();
                  }))
  {
    std::vector<uint8_t> cells, en, ex;
    for (auto const & L : ov.levels)
    {
      cells.insert(cells.end(), L.cellRoads.begin(), L.cellRoads.end());
      for (size_t i = 0; i < L.entries.size(); ++i)
        en.push_back(PackBoundaryRoads(fc.selfRoads[L.entries[i]], L.entryPathRoads[i]));
      for (size_t j = 0; j < L.exits.size(); ++j)
        ex.push_back(PackBoundaryRoads(fc.selfRoads[L.exits[j]], L.exitPathRoads[j]));
    }
    putVec(kCellPathRoads, cells);
    putVec(kEntryRoads, en);
    putVec(kExitRoads, ex);
  }
  for (size_t li = 0; li < levels; ++li)
  {
    auto const & L = ov.levels[li];
    std::vector<uint32_t> eo(L.entryOff.begin(), L.entryOff.end()), xo(L.exitOff.begin(), L.exitOff.end());
    EncodedCliques e;
    if (!EncodeCliques(L, p.numCells[li], e, error))
      return false;
    uint32_t const base = kLevelBase + kLevelSections * static_cast<uint32_t>(li);
    putVec(base + kEntryOff, eo);
    putVec(base + kExitOff, xo);
    putVec(base + kEntries, L.entries);
    putVec(base + kExits, L.exits);
    putVec(base + kRowRank, e.rowRank);
    putVec(base + kBits, e.bits);
    putVec(base + kValues, e.values);
    putVec(base + kScale, e.scale);
    putVec(base + kPenalty, e.penalty);
    putVec(base + kPenaltyOff, e.penaltyOff);
  }
  return w.Finish(error);
}

namespace
{
bool ReadHeader(std::string const & path, Header & h)
{
  int const fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0)
    return false;
  bool const ok = ::pread(fd, &h, sizeof(h), 0) == static_cast<ssize_t>(sizeof(h));
  ::close(fd);
  return ok && std::memcmp(h.magic, kMagic, sizeof(kMagic)) == 0 && IsReadableVersion(h.formatVersion);
}
}  // namespace

bool OverlayFileMatches(std::string const & path, int64_t mwmVersion, uint64_t paramsHash, uint64_t * buildId)
{
  Header h;
  if (!ReadHeader(path, h) || (h.flags & kCliquesOnly) || h.mwmVersion != mwmVersion || h.paramsHash != paramsHash)
    return false;
  if (buildId)
    *buildId = h.buildId;
  return true;
}

bool OverlayFileIsLatestFormat(std::string const & path)
{
  Header h;
  return ReadHeader(path, h) && h.formatVersion == kFileFormatVersion;
}

bool MappedOverlay::Mapping::Open(std::string const & path, std::string * error)
{
  fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0)
  {
    SetError(error, "no file");
    return false;
  }
  struct stat st;
  if (::fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(Header)))
  {
    SetError(error, "too small");
    return false;
  }
  size = static_cast<uint64_t>(st.st_size);
  void * ptr = ::mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
  if (ptr != MAP_FAILED)
  {
    data = static_cast<uint8_t const *>(ptr);
    mapped = true;
    return true;
  }
  buffer.resize(size);
  if (::pread(fd, buffer.data(), size, 0) != static_cast<ssize_t>(size))
  {
    SetError(error, "read failed");
    return false;
  }
  data = buffer.data();
  return true;
}

MappedOverlay::Mapping::~Mapping()
{
  if (mapped)
    ::munmap(const_cast<uint8_t *>(data), size);
  if (fd >= 0)
    ::close(fd);
}

std::unique_ptr<MappedOverlay> MappedOverlay::Open(std::string const & path, int64_t expectMwmVersion,
                                                   uint64_t expectParamsHash, std::string * error, bool * damaged)
{
  if (damaged)
    *damaged = false;
  std::unique_ptr<MappedOverlay> m(new MappedOverlay());
  if (!m->m_file.Open(path, error))
  {
    if (damaged && m->m_file.fd >= 0)
      *damaged = true;  // exists but too small or unreadable
    return nullptr;
  }
  auto const & h = *reinterpret_cast<Header const *>(m->m_file.data);
  if (std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0)
  {
    if (damaged)
      *damaged = true;
    SetError(error, "not an overlay file");
    return nullptr;
  }
  if (!IsReadableVersion(h.formatVersion))
  {
    SetError(error, "not a v" + std::to_string(kFileFormatVersion) + " overlay file");
    return nullptr;
  }
  if (h.flags & kCliquesOnly)
  {
    SetError(error, "an avoid-option file, not a base overlay file");
    return nullptr;
  }
  if (expectMwmVersion != 0 && h.mwmVersion != expectMwmVersion)
  {
    SetError(error, "map version " + std::to_string(h.mwmVersion) + " != " + std::to_string(expectMwmVersion));
    return nullptr;
  }
  if (expectParamsHash != 0 && h.paramsHash != expectParamsHash)
  {
    SetError(error, "build parameters changed");
    return nullptr;
  }
  if (!m->Parse(error))
  {
    if (damaged)
      *damaged = true;
    return nullptr;
  }
  return m;
}

MappedOverlay::~MappedOverlay() = default;

bool MappedOverlay::Parse(std::string * error)
{
  auto const & h = *reinterpret_cast<Header const *>(m_file.data);
  for (size_t i = 0; i < kMaxSections; ++i)
  {
    if (h.offset[i] + h.size[i] > m_file.size || (h.size[i] && h.offset[i] % 8 != 0))
    {
      SetError(error, "bad section table");
      return false;
    }
  }
  auto sec = [&](uint32_t id) { return m_file.data + h.offset[id]; };
  auto count = [&](uint32_t id, size_t elem) { return h.size[id] / elem; };

  m_mwmVersion = h.mwmVersion;
  m_paramsHash = h.paramsHash;
  m_buildId = h.buildId;
  m_n = h.n;
  m_baseN = h.baseN;
  m_numSlots = h.numSlots;
  m_numFeatures = h.numFeatures;
  m_cell16 = (h.flags & kCell1Is16Bit) != 0;
  m_timeDependent = (h.flags & kTimeDependent) != 0;
  if (h.numLevels == 0 || h.numLevels > kMaxLevels || count(kFeatLow, 2) != m_numFeatures + 1ULL ||
      count(kFeatBlock, 4) != (m_numFeatures + 1ULL + 63) / 64 || count(kRunStarts, 8) != (m_n + 63ULL) / 64 ||
      count(kRunRank, 4) != (m_n + 63ULL) / 64 || count(kCopyOrig, 4) != m_n - m_baseN ||
      count(kCopyState, 4) != m_n - m_baseN)
  {
    SetError(error, "inconsistent sizes");
    return false;
  }
  m_featBlock = reinterpret_cast<uint32_t const *>(sec(kFeatBlock));
  m_featLow = reinterpret_cast<uint16_t const *>(sec(kFeatLow));
  m_keptBits = reinterpret_cast<uint64_t const *>(sec(kKeptBits));
  m_keptRank = reinterpret_cast<uint32_t const *>(sec(kKeptRank));
  m_numWords = static_cast<uint32_t>(count(kKeptBits, 8));
  m_copyOrig = reinterpret_cast<uint32_t const *>(sec(kCopyOrig));
  m_copyState = reinterpret_cast<uint32_t const *>(sec(kCopyState));
  m_runStarts = reinterpret_cast<uint64_t const *>(sec(kRunStarts));
  m_runRank = reinterpret_cast<uint32_t const *>(sec(kRunRank));
  m_runCells = sec(kRunCells);
  m_numRuns = count(kRunCells, m_cell16 ? 2 : 4);
  {
    // Runs: the first vertex starts one, the ranks count them, no bits past the last vertex.
    uint64_t seen = 0;
    uint64_t const words = (m_n + 63ULL) / 64;
    bool ok = m_n > 0 && (m_runStarts[0] & 1);
    for (uint64_t w = 0; ok && w < words; ++w)
    {
      ok = m_runRank[w] == seen;
      seen += static_cast<uint64_t>(__builtin_popcountll(m_runStarts[w]));
    }
    if (ok && m_n % 64 != 0)
      ok = (m_runStarts[words - 1] >> (m_n % 64)) == 0;
    if (!ok || seen != m_numRuns)
    {
      SetError(error, "bad cell runs");
      return false;
    }
  }
  m_transitions = reinterpret_cast<FileTransition const *>(sec(kTransitions));
  m_numTransitions = static_cast<uint32_t>(count(kTransitions, sizeof(FileTransition)));
  {
    auto const * sp = reinterpret_cast<uint32_t const *>(sec(kSpecial));
    size_t const ns = count(kSpecial, 4);
    m_special.assign(sp, sp + ns);
    for (size_t i = 0; i < ns; ++i)
    {
      if (m_special[i] >= m_n || (i > 0 && m_special[i] <= m_special[i - 1]))
      {
        SetError(error, "bad special vertices");
        return false;
      }
    }
  }

  m_levels.resize(h.numLevels);
  m_parents.assign(h.numLevels, nullptr);
  auto const * par = reinterpret_cast<uint32_t const *>(sec(kParents));
  uint64_t parUsed = 0;
  for (size_t li = 0; li < h.numLevels; ++li)
  {
    Level & L = m_levels[li];
    L.numCells = h.numCells[li];
    if (li >= 1)
    {
      m_parents[li] = par + parUsed;
      parUsed += h.numCells[li - 1];
    }
    uint32_t const base = kLevelBase + kLevelSections * static_cast<uint32_t>(li);
    if (count(base + kEntryOff, 4) != L.numCells + 1ULL || count(base + kExitOff, 4) != L.numCells + 1ULL)
    {
      SetError(error, "bad level offsets");
      return false;
    }
    L.entryOff = reinterpret_cast<uint32_t const *>(sec(base + kEntryOff));
    L.exitOff = reinterpret_cast<uint32_t const *>(sec(base + kExitOff));
    L.entries = reinterpret_cast<uint32_t const *>(sec(base + kEntries));
    L.exits = reinterpret_cast<uint32_t const *>(sec(base + kExits));
    L.rowRank = reinterpret_cast<uint32_t const *>(sec(base + kRowRank));
    L.bits = reinterpret_cast<uint64_t const *>(sec(base + kBits));
    L.values = reinterpret_cast<uint16_t const *>(sec(base + kValues));
    L.penalty = sec(base + kPenalty);
    L.penaltyOff = reinterpret_cast<uint32_t const *>(sec(base + kPenaltyOff));
    L.scale = reinterpret_cast<float const *>(sec(base + kScale));
    L.wordOff.assign(L.numCells + 1, 0);
    for (uint32_t c = 0; c < L.numCells; ++c)
    {
      uint64_t const nEx = L.exitOff[c + 1] - L.exitOff[c];
      L.wordOff[c + 1] = L.wordOff[c] + uint64_t(L.entryOff[c + 1] - L.entryOff[c]) * ((nEx + 63) / 64);
    }
    uint64_t const nEntries = L.entryOff[L.numCells], nValues = count(base + kValues, 2);
    if (L.wordOff[L.numCells] != count(base + kBits, 8) || count(base + kEntries, 4) != nEntries ||
        count(base + kExits, 4) != L.exitOff[L.numCells] || count(base + kRowRank, 4) != nEntries ||
        count(base + kScale, 4) != L.numCells || count(base + kPenaltyOff, 4) != L.numCells + 1ULL)
    {
      SetError(error, "bad level sizes");
      return false;
    }
    // Every row's values must lie inside the values section.
    if (!CliquesConsistent(L, L.rowRank, L.bits, nValues, L.penaltyOff, count(base + kPenalty, 1)) ||
        !std::all_of(L.scale, L.scale + L.numCells, IsUsableWeight))
    {
      SetError(error, "bad clique values");
      return false;
    }
  }
  if (parUsed != count(kParents, 4))
  {
    SetError(error, "bad parent tables");
    return false;
  }
  // Cell ids index the per-cell tables: all must be in range (a damaged file is rejected, not read
  // out of bounds). Boundary lists must hold vertex ids.
  for (uint64_t r = 0; r < m_numRuns; ++r)
  {
    uint32_t const c = m_cell16 ? reinterpret_cast<uint16_t const *>(m_runCells)[r]
                                : reinterpret_cast<uint32_t const *>(m_runCells)[r];
    if (c >= m_levels[0].numCells)
    {
      SetError(error, "bad cell ids");
      return false;
    }
  }
  for (size_t li = 1; li < m_levels.size(); ++li)
    for (uint32_t c = 0; c < m_levels[li - 1].numCells; ++c)
      if (m_parents[li][c] >= m_levels[li].numCells)
      {
        SetError(error, "bad parent tables");
        return false;
      }
  for (auto const & L : m_levels)
  {
    for (uint32_t c = 0; c < L.numCells; ++c)
      if (L.entryOff[c] > L.entryOff[c + 1] || L.exitOff[c] > L.exitOff[c + 1])
      {
        SetError(error, "bad level offsets");
        return false;
      }
    for (uint64_t i = 0; i < L.entryOff[L.numCells]; ++i)
      if (L.entries[i] >= m_n)
      {
        SetError(error, "bad boundary lists");
        return false;
      }
    for (uint64_t i = 0; i < L.exitOff[L.numCells]; ++i)
      if (L.exits[i] >= m_n)
      {
        SetError(error, "bad boundary lists");
        return false;
      }
  }

  // Via-way automaton and copy lookups (tiny).
  auto const * pat = reinterpret_cast<uint32_t const *>(sec(kPatterns));
  size_t const patWords = count(kPatterns, 4);
  std::vector<std::vector<uint32_t>> patterns;
  if (patWords > 0)
  {
    size_t i = 1;
    for (uint32_t k = 0; k < pat[0] && i < patWords; ++k)
    {
      uint32_t const len = pat[i++];
      patterns.emplace_back(pat + i, pat + std::min<size_t>(patWords, i + len));
      i += len;
    }
  }
  if (!patterns.empty())
    m_automaton = std::make_unique<ViaWayAutomaton>(patterns);
  for (uint32_t i = 0; i < m_n - m_baseN; ++i)
  {
    uint32_t const local = m_baseN + i;
    m_copyId[(uint64_t(m_copyState[i]) << 32) | m_copyOrig[i]] = local;
    m_copiesOf[m_copyOrig[i]].push_back(local);
  }
  std::vector<uint32_t> numCells;
  for (auto const & L : m_levels)
    numCells.push_back(L.numCells);
  m_open = OpenCells(numCells, m_special, [this](size_t li, uint32_t v) { return Cell(li, v); });
  // Road types per cell: level 1 from the file, each level above as the union of its children.
  m_cellRoads.assign(m_levels.size(), {});
  if (count(kCellRoads, 1) != numCells[0])
  {
    SetError(error, "bad road types per cell");
    return false;
  }
  m_cellRoads[0].assign(sec(kCellRoads), sec(kCellRoads) + numCells[0]);
  for (size_t li = 1; li < m_levels.size(); ++li)
  {
    m_cellRoads[li].assign(numCells[li], 0);
    for (uint32_t c = 0; c < numCells[li - 1]; ++c)
    {
      uint32_t const parent = m_parents[li][c];
      if (parent < numCells[li])
        m_cellRoads[li][parent] |= m_cellRoads[li - 1][c];
    }
  }
  // Format 8: the road types the cells' clique paths use (fewer cells than the road types inside them), with
  // the road types of every entry and exit, which the search then skips when a route avoids them.
  // Format 8 (never released) stored them without the rows and columns: read such files like format 7.
  if (count(kCellPathRoads, 1) != 0 && h.formatVersion >= 9)
  {
    uint64_t cells = 0, entries = 0, exits = 0;
    for (auto const & L : m_levels)
    {
      cells += L.numCells;
      entries += L.entryOff[L.numCells];
      exits += L.exitOff[L.numCells];
    }
    if (count(kCellPathRoads, 1) != cells || count(kEntryRoads, 1) != entries || count(kExitRoads, 1) != exits)
    {
      SetError(error, "bad road types of the cells' paths");
      return false;
    }
    uint8_t const * pc = sec(kCellPathRoads);
    uint8_t const * pe = sec(kEntryRoads);
    uint8_t const * px = sec(kExitRoads);
    for (size_t li = 0; li < m_levels.size(); ++li)
    {
      auto & L = m_levels[li];
      m_cellRoads[li].assign(pc, pc + L.numCells);
      L.entryRoads = pe;
      L.exitRoads = px;
      L.packedRoads = true;
      pc += L.numCells;
      pe += L.entryOff[L.numCells];
      px += L.exitOff[L.numCells];
    }
    m_pathRoads = true;
  }
  return true;
}

uint32_t MappedOverlay::SlotOf(uint32_t local) const
{
  if (local >= m_baseN)
    local = m_copyOrig[local - m_baseN];
  // Last word whose rank <= local, then the (local - rank)-th set bit in it.
  uint32_t lo = 0, hi = m_numWords;
  while (hi - lo > 1)
  {
    uint32_t const mid = (lo + hi) / 2;
    (m_keptRank[mid] <= local ? lo : hi) = mid;
  }
  uint64_t w = m_keptBits[lo];
  uint32_t k = local - m_keptRank[lo];
  while (k--)
    w &= w - 1;
  return lo * 64 + static_cast<uint32_t>(__builtin_ctzll(w));
}

void MappedOverlay::Decode(uint32_t slot, uint32_t & feature, uint32_t & segIdx, bool & forward) const
{
  uint32_t const seg = slot / 2;
  // Last feature whose first segment <= seg: first by block, then inside the block.
  uint32_t const numBlocks = (m_numFeatures + 1 + 63) / 64;
  uint32_t const blk =
      static_cast<uint32_t>(std::upper_bound(m_featBlock, m_featBlock + numBlocks, seg) - m_featBlock) - 1;
  uint32_t lo = blk * 64, hi = std::min(blk * 64 + 64, m_numFeatures + 1);  // FeatBase(lo) <= seg
  while (hi - lo > 1)
  {
    uint32_t const mid = (lo + hi) / 2;
    (FeatBase(mid) <= seg ? lo : hi) = mid;
  }
  // Features without segments share their first segment with the next one: take the last.
  feature = lo;
  segIdx = seg - FeatBase(feature);
  forward = (slot % 2) == 0;
}

uint32_t MappedOverlay::FeatureOfLocal(uint32_t local) const
{
  uint32_t f, s;
  bool fwd;
  Decode(SlotOf(local), f, s, fwd);
  return f;
}

uint32_t MappedOverlay::CopyId(uint32_t state, uint32_t base) const
{
  auto it = m_copyId.find((uint64_t(state) << 32) | base);
  return it == m_copyId.end() ? kNone : it->second;
}

std::vector<uint32_t> const * MappedOverlay::CopiesOf(uint32_t base) const
{
  auto it = m_copiesOf.find(base);
  return it == m_copiesOf.end() ? nullptr : &it->second;
}

uint32_t MappedOverlay::EntryIdx(size_t li, uint32_t cell, uint32_t v) const
{
  auto const & L = m_levels[li];
  auto const * b = L.entries + L.entryOff[cell];
  auto const * e = L.entries + L.entryOff[cell + 1];
  return static_cast<uint32_t>(std::lower_bound(b, e, v) - b);
}

float MappedOverlay::Clique(size_t li, uint32_t cell, uint32_t i, uint32_t j) const
{
  auto const & L = m_levels[li];
  uint64_t const nEx = L.exitOff[cell + 1] - L.exitOff[cell];
  uint64_t const wpr = (nEx + 63) / 64;
  auto const cc = CliquesOf(L, cell);
  uint64_t const row = i * wpr;
  uint64_t const word = cc.bits[row + j / 64];
  if (!((word >> (j % 64)) & 1))
    return 1e30f;
  uint64_t idx = cc.rowRank[i];
  for (uint64_t k = 0; k < j / 64; ++k)
    idx += static_cast<uint64_t>(__builtin_popcountll(cc.bits[row + k]));
  idx += static_cast<uint64_t>(__builtin_popcountll(word & ((uint64_t(1) << (j % 64)) - 1)));
  return cc.Weight(idx);
}

uint32_t MappedOverlay::ExitIdx(size_t li, uint32_t cell, uint32_t v) const
{
  auto const & L = m_levels[li];
  auto const * b = L.exits + L.exitOff[cell];
  auto const * e = L.exits + L.exitOff[cell + 1];
  return static_cast<uint32_t>(std::lower_bound(b, e, v) - b);
}
}  // namespace plus_overlay
