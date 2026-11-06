#include "routing/plus_overlay/overlay_file.hpp"

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
  kCell1,
  kParents,
  kRoles,
  kTransitions,
  kFeatLow,        // u16 per feature (+1): first segment relative to its block
  kLevelBase = 16,  // + kLevelSections * level + LevelSection
};

enum LevelSection : uint32_t
{
  kEntryOff = 0,  // u32 per cell + 1
  kExitOff,       // u32 per cell + 1
  kEntries,       // u32, per cell ascending
  kExits,         // u32, per cell ascending
  kRowRank,       // u32 per entry: finite clique values before its row (in the level)
  kBits,          // u64: per cell, entry rows of ceil(exits / 64) words; bit set = finite
  kValues,        // float32: the finite clique values, row by row
  kLevelSections
};

enum HeaderFlags : uint32_t
{
  kCell1Is16Bit = 1,
  kCliquesOnly = 2,  // an avoid-option file: only the per-level clique sections, for one base file
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
  uint32_t const v = kFileFormatVersion;
  h = Fnv(h, &v, sizeof(v));
  h = Fnv(h, cellSizes.data(), cellSizes.size() * sizeof(uint32_t));
  h = Fnv(h, &minBalance, sizeof(minBalance));
  h = Fnv(h, &directions, sizeof(directions));
  return h;
}

namespace
{
// Sparse cliques of one level: a bit per (entry, exit), rows word-aligned, and the finite values in row order.
bool EncodeCliques(LevelOverlay const & L, uint32_t numCells, std::vector<uint32_t> & rowRank,
                   std::vector<uint64_t> & bits, std::vector<float> & values, std::string * error)
{
  rowRank.clear();
  bits.clear();
  values.clear();
  rowRank.reserve(L.entries.size());
  for (uint32_t c = 0; c < numCells; ++c)
  {
    uint64_t const nEn = L.entryOff[c + 1] - L.entryOff[c], nEx = L.exitOff[c + 1] - L.exitOff[c];
    uint64_t const wpr = (nEx + 63) / 64;
    for (uint64_t i = 0; i < nEn; ++i)
    {
      if (values.size() > 0xFFFFFFFFULL)
      {
        SetError(error, "too many clique values in one level");
        return false;
      }
      rowRank.push_back(static_cast<uint32_t>(values.size()));
      uint64_t const w0 = bits.size();
      bits.resize(w0 + wpr, 0);
      for (uint64_t j = 0; j < nEx; ++j)
      {
        double const w = L.mat[L.matOff[c] + i * nEx + j];
        if (w >= kInf)
          continue;
        bits[w0 + j / 64] |= uint64_t(1) << (j % 64);
        values.push_back(static_cast<float>(w));
      }
    }
  }
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
  if (p.numCells[0] <= 0x10000)
  {
    h.flags |= kCell1Is16Bit;
    std::vector<uint16_t> c16(p.cell[0].begin(), p.cell[0].end());
    putVec(kCell1, c16);
  }
  else
  {
    putVec(kCell1, p.cell[0]);
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
  std::vector<uint8_t> roles(fc.n, 0);
  for (size_t li = 0; li < levels; ++li)
    for (uint32_t v = 0; v < fc.n; ++v)
      roles[v] |= static_cast<uint8_t>((ov.levels[li].role[v] & 3) << (2 * li));
  putVec(kRoles, roles);
  putVec(kTransitions, fc.transitions);
  for (size_t li = 0; li < levels; ++li)
  {
    auto const & L = ov.levels[li];
    std::vector<uint32_t> eo(L.entryOff.begin(), L.entryOff.end()), xo(L.exitOff.begin(), L.exitOff.end());
    std::vector<uint32_t> rowRank;
    std::vector<uint64_t> bits;
    std::vector<float> values;
    if (!EncodeCliques(L, p.numCells[li], rowRank, bits, values, error))
      return false;
    uint32_t const base = kLevelBase + kLevelSections * static_cast<uint32_t>(li);
    putVec(base + kEntryOff, eo);
    putVec(base + kExitOff, xo);
    putVec(base + kEntries, L.entries);
    putVec(base + kExits, L.exits);
    putVec(base + kRowRank, rowRank);
    putVec(base + kBits, bits);
    putVec(base + kValues, values);
  }
  return w.Finish(error);
}

bool WriteAvoidCliquesFile(std::string const & path, MappedOverlay const & base, uint32_t avoidMask,
                           Overlay const & ov, std::string * error)
{
  size_t const levels = base.Levels();
  if (ov.levels.size() != levels)
  {
    SetError(error, "level count differs from the base file");
    return false;
  }
  Header h;
  std::memset(&h, 0, sizeof(h));
  std::memcpy(h.magic, kMagic, sizeof(kMagic));
  h.formatVersion = kFileFormatVersion;
  h.numLevels = static_cast<uint32_t>(levels);
  h.mwmVersion = base.MwmVersion();
  h.paramsHash = base.ParamsHash();
  h.baseN = base.BaseN();
  h.n = base.NumVertices();
  h.numFeatures = base.NumFeatures();
  for (size_t li = 0; li < levels; ++li)
    h.numCells[li] = base.GetLevel(li).numCells;
  h.flags = kCliquesOnly;
  h.buildId = NewBuildId();
  h.baseBuildId = base.BuildId();
  h.avoidMask = avoidMask;

  SectionWriter w(path, h);
  if (!w.IsOpen())
  {
    SetError(error, "can't create " + path + ".tmp");
    return false;
  }
  for (size_t li = 0; li < levels; ++li)
  {
    std::vector<uint32_t> rowRank;
    std::vector<uint64_t> bits;
    std::vector<float> values;
    if (!EncodeCliques(ov.levels[li], h.numCells[li], rowRank, bits, values, error))
      return false;
    uint32_t const sec = kLevelBase + kLevelSections * static_cast<uint32_t>(li);
    w.PutVec(sec + kRowRank, rowRank);
    w.PutVec(sec + kBits, bits);
    w.PutVec(sec + kValues, values);
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
  return ok && std::memcmp(h.magic, kMagic, sizeof(kMagic)) == 0 && h.formatVersion == kFileFormatVersion;
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

bool AvoidCliquesFileMatches(std::string const & path, uint64_t baseBuildId, uint32_t avoidMask)
{
  Header h;
  return ReadHeader(path, h) && (h.flags & kCliquesOnly) && h.baseBuildId == baseBuildId && h.avoidMask == avoidMask;
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
                                                   uint64_t expectParamsHash, std::string * error)
{
  std::unique_ptr<MappedOverlay> m(new MappedOverlay());
  if (!m->m_file.Open(path, error))
    return nullptr;
  auto const & h = *reinterpret_cast<Header const *>(m->m_file.data);
  if (std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0 || h.formatVersion != kFileFormatVersion)
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
    return nullptr;
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
  if (h.numLevels == 0 || h.numLevels > kMaxLevels || count(kFeatLow, 2) != m_numFeatures + 1ULL ||
      count(kFeatBlock, 4) != (m_numFeatures + 1ULL + 63) / 64 || count(kCell1, m_cell16 ? 2 : 4) != m_n ||
      count(kRoles, 1) != m_n || count(kCopyOrig, 4) != m_n - m_baseN || count(kCopyState, 4) != m_n - m_baseN)
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
  m_cell1 = sec(kCell1);
  m_roles = sec(kRoles);
  m_transitions = reinterpret_cast<FileTransition const *>(sec(kTransitions));
  m_numTransitions = static_cast<uint32_t>(count(kTransitions, sizeof(FileTransition)));

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
    L.values = reinterpret_cast<float const *>(sec(base + kValues));
    L.wordOff.assign(L.numCells + 1, 0);
    for (uint32_t c = 0; c < L.numCells; ++c)
    {
      uint64_t const nEx = L.exitOff[c + 1] - L.exitOff[c];
      L.wordOff[c + 1] = L.wordOff[c] + uint64_t(L.entryOff[c + 1] - L.entryOff[c]) * ((nEx + 63) / 64);
    }
    uint64_t const nEntries = L.entryOff[L.numCells], nValues = count(base + kValues, 4);
    if (L.wordOff[L.numCells] != count(base + kBits, 8) || count(base + kEntries, 4) != nEntries ||
        count(base + kExits, 4) != L.exitOff[L.numCells] || count(base + kRowRank, 4) != nEntries)
    {
      SetError(error, "bad level sizes");
      return false;
    }
    // Every row's values must lie inside the values section.
    uint64_t bitsSet = 0;
    for (uint64_t w = 0; w < L.wordOff[L.numCells]; ++w)
      bitsSet += static_cast<uint64_t>(__builtin_popcountll(L.bits[w]));
    if (bitsSet != nValues || (nEntries > 0 && L.rowRank[nEntries - 1] > nValues))
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
  return true;
}

bool MappedOverlay::AttachAvoidCliques(std::string const & path, uint32_t avoidMask, std::string * error)
{
  Mapping mp;
  if (!mp.Open(path, error))
    return false;
  auto const & h = *reinterpret_cast<Header const *>(mp.data);
  if (std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0 || h.formatVersion != kFileFormatVersion ||
      !(h.flags & kCliquesOnly) || h.baseBuildId != m_buildId || h.avoidMask != avoidMask ||
      h.numLevels != m_levels.size())
  {
    SetError(error, "avoid-option file doesn't belong to this overlay file");
    return false;
  }
  for (size_t i = 0; i < kMaxSections; ++i)
  {
    if (h.offset[i] + h.size[i] > mp.size || (h.size[i] && h.offset[i] % 8 != 0))
    {
      SetError(error, "bad section table");
      return false;
    }
  }
  struct Cliques
  {
    uint32_t const * rowRank;
    uint64_t const * bits;
    float const * values;
  };
  std::vector<Cliques> cl(m_levels.size());
  for (size_t li = 0; li < m_levels.size(); ++li)
  {
    auto const & L = m_levels[li];
    uint32_t const base = kLevelBase + kLevelSections * static_cast<uint32_t>(li);
    uint64_t const nEntries = L.entryOff[L.numCells];
    uint64_t const nWords = L.wordOff[L.numCells];
    uint64_t const nValues = h.size[base + kValues] / 4;
    if (h.numCells[li] != L.numCells || h.size[base + kRowRank] != nEntries * 4 || h.size[base + kBits] != nWords * 8)
    {
      SetError(error, "avoid-option file has other level sizes");
      return false;
    }
    cl[li].rowRank = reinterpret_cast<uint32_t const *>(mp.data + h.offset[base + kRowRank]);
    cl[li].bits = reinterpret_cast<uint64_t const *>(mp.data + h.offset[base + kBits]);
    cl[li].values = reinterpret_cast<float const *>(mp.data + h.offset[base + kValues]);
    uint64_t bitsSet = 0;
    for (uint64_t w = 0; w < nWords; ++w)
      bitsSet += static_cast<uint64_t>(__builtin_popcountll(cl[li].bits[w]));
    if (bitsSet != nValues || (nEntries > 0 && cl[li].rowRank[nEntries - 1] > nValues))
    {
      SetError(error, "bad clique values");
      return false;
    }
  }
  for (size_t li = 0; li < m_levels.size(); ++li)
  {
    m_levels[li].rowRank = cl[li].rowRank;
    m_levels[li].bits = cl[li].bits;
    m_levels[li].values = cl[li].values;
  }
  std::swap(m_avoidFile.fd, mp.fd);
  std::swap(m_avoidFile.data, mp.data);
  std::swap(m_avoidFile.size, mp.size);
  std::swap(m_avoidFile.mapped, mp.mapped);
  std::swap(m_avoidFile.buffer, mp.buffer);
  m_avoidMask = avoidMask;
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
  uint64_t const row = L.wordOff[cell] + i * wpr;
  uint64_t const word = L.bits[row + j / 64];
  if (!((word >> (j % 64)) & 1))
    return 1e30f;
  uint64_t idx = L.rowRank[L.entryOff[cell] + i];
  for (uint64_t k = 0; k < j / 64; ++k)
    idx += static_cast<uint64_t>(__builtin_popcountll(L.bits[row + k]));
  idx += static_cast<uint64_t>(__builtin_popcountll(word & ((uint64_t(1) << (j % 64)) - 1)));
  return L.values[idx];
}

uint32_t MappedOverlay::ExitIdx(size_t li, uint32_t cell, uint32_t v) const
{
  auto const & L = m_levels[li];
  auto const * b = L.exits + L.exitOff[cell];
  auto const * e = L.exits + L.exitOff[cell + 1];
  return static_cast<uint32_t>(std::lower_bound(b, e, v) - b);
}
}  // namespace plus_overlay
