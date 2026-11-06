#include "routing/plus_overlay/overlay_file.hpp"

#include <algorithm>
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
constexpr char kMagic[8] = {'P', 'L', 'U', 'S', 'O', 'V', 'L', '1'};
constexpr size_t kMaxSections = 64;

enum Section : uint32_t
{
  kFeatBase = 0,
  kKeptBits,
  kKeptRank,
  kCopyOrig,
  kCopyState,
  kPatterns,
  kCell1,
  kParents,
  kRoles,
  kTransitions,
  kLevelBase = 16,  // + 5 * level + {0 entryOff, 1 exitOff, 2 entries, 3 exits, 4 mat}
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
  for (size_t li = 0; li < levels; ++li)
    h.numCells[li] = p.numCells[li];

  std::string const tmp = path + ".tmp";
  std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
  if (!out)
  {
    SetError(error, "can't create " + tmp);
    return false;
  }
  out.write(reinterpret_cast<char const *>(&h), sizeof(h));
  uint64_t pos = sizeof(h);
  auto put = [&](uint32_t id, void const * data, uint64_t bytes)
  {
    static char const zeros[8] = {};
    uint64_t const pad = (8 - pos % 8) % 8;
    out.write(zeros, static_cast<std::streamsize>(pad));
    pos += pad;
    h.offset[id] = pos;
    h.size[id] = bytes;
    if (bytes)
      out.write(static_cast<char const *>(data), static_cast<std::streamsize>(bytes));
    pos += bytes;
  };
  auto putVec = [&](uint32_t id, auto const & v) { put(id, v.data(), v.size() * sizeof(v[0])); };

  putVec(kFeatBase, fc.featBase);
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
  putVec(kCell1, p.cell[0]);
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
    std::vector<float> m32(L.mat.size());
    for (size_t i = 0; i < L.mat.size(); ++i)
      m32[i] = L.mat[i] >= kInf ? 1e30f : static_cast<float>(L.mat[i]);
    uint32_t const base = kLevelBase + 5 * static_cast<uint32_t>(li);
    putVec(base + 0, eo);
    putVec(base + 1, xo);
    putVec(base + 2, L.entries);
    putVec(base + 3, L.exits);
    putVec(base + 4, m32);
  }
  out.seekp(0);
  out.write(reinterpret_cast<char const *>(&h), sizeof(h));
  out.close();
  if (!out)
  {
    SetError(error, "write failed");
    std::remove(tmp.c_str());
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0)
  {
    SetError(error, "rename failed");
    std::remove(tmp.c_str());
    return false;
  }
  return true;
}

std::unique_ptr<MappedOverlay> MappedOverlay::Open(std::string const & path, int64_t expectMwmVersion,
                                                   uint64_t expectParamsHash, std::string * error)
{
  std::unique_ptr<MappedOverlay> m(new MappedOverlay());
  m->m_fd = ::open(path.c_str(), O_RDONLY);
  if (m->m_fd < 0)
  {
    SetError(error, "no file");
    return nullptr;
  }
  struct stat st;
  if (::fstat(m->m_fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(Header)))
  {
    SetError(error, "too small");
    return nullptr;
  }
  m->m_size = static_cast<uint64_t>(st.st_size);
  void * ptr = ::mmap(nullptr, m->m_size, PROT_READ, MAP_SHARED, m->m_fd, 0);
  if (ptr != MAP_FAILED)
  {
    m->m_data = static_cast<uint8_t const *>(ptr);
    m->m_mapped = true;
  }
  else
  {
    m->m_buffer.resize(m->m_size);
    if (::pread(m->m_fd, m->m_buffer.data(), m->m_size, 0) != static_cast<ssize_t>(m->m_size))
    {
      SetError(error, "read failed");
      return nullptr;
    }
    m->m_data = m->m_buffer.data();
  }
  auto const & h = *reinterpret_cast<Header const *>(m->m_data);
  if (std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0 || h.formatVersion != kFileFormatVersion)
  {
    SetError(error, "not a v1 overlay file");
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

MappedOverlay::~MappedOverlay()
{
  if (m_mapped)
    ::munmap(const_cast<uint8_t *>(m_data), m_size);
  if (m_fd >= 0)
    ::close(m_fd);
}

bool MappedOverlay::Parse(std::string * error)
{
  auto const & h = *reinterpret_cast<Header const *>(m_data);
  for (size_t i = 0; i < kMaxSections; ++i)
  {
    if (h.offset[i] + h.size[i] > m_size || (h.size[i] && h.offset[i] % 8 != 0))
    {
      SetError(error, "bad section table");
      return false;
    }
  }
  auto sec = [&](uint32_t id) { return m_data + h.offset[id]; };
  auto count = [&](uint32_t id, size_t elem) { return h.size[id] / elem; };

  m_mwmVersion = h.mwmVersion;
  m_n = h.n;
  m_baseN = h.baseN;
  m_numSlots = h.numSlots;
  m_numFeatures = h.numFeatures;
  if (h.numLevels == 0 || h.numLevels > kMaxLevels || count(kFeatBase, 4) != m_numFeatures + 1ULL ||
      count(kCell1, 4) != m_n || count(kRoles, 1) != m_n || count(kCopyOrig, 4) != m_n - m_baseN ||
      count(kCopyState, 4) != m_n - m_baseN)
  {
    SetError(error, "inconsistent sizes");
    return false;
  }
  m_featBase = reinterpret_cast<uint32_t const *>(sec(kFeatBase));
  m_keptBits = reinterpret_cast<uint64_t const *>(sec(kKeptBits));
  m_keptRank = reinterpret_cast<uint32_t const *>(sec(kKeptRank));
  m_numWords = static_cast<uint32_t>(count(kKeptBits, 8));
  m_copyOrig = reinterpret_cast<uint32_t const *>(sec(kCopyOrig));
  m_copyState = reinterpret_cast<uint32_t const *>(sec(kCopyState));
  m_cell1 = reinterpret_cast<uint32_t const *>(sec(kCell1));
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
    uint32_t const base = kLevelBase + 5 * static_cast<uint32_t>(li);
    if (count(base + 0, 4) != L.numCells + 1ULL || count(base + 1, 4) != L.numCells + 1ULL)
    {
      SetError(error, "bad level offsets");
      return false;
    }
    L.entryOff = reinterpret_cast<uint32_t const *>(sec(base + 0));
    L.exitOff = reinterpret_cast<uint32_t const *>(sec(base + 1));
    L.entries = reinterpret_cast<uint32_t const *>(sec(base + 2));
    L.exits = reinterpret_cast<uint32_t const *>(sec(base + 3));
    L.mat = reinterpret_cast<float const *>(sec(base + 4));
    L.matOff.assign(L.numCells + 1, 0);
    for (uint32_t c = 0; c < L.numCells; ++c)
      L.matOff[c + 1] = L.matOff[c] + uint64_t(L.entryOff[c + 1] - L.entryOff[c]) * (L.exitOff[c + 1] - L.exitOff[c]);
    if (L.matOff[L.numCells] != count(base + 4, 4) || count(base + 2, 4) != L.entryOff[L.numCells] ||
        count(base + 3, 4) != L.exitOff[L.numCells])
    {
      SetError(error, "bad level sizes");
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
  auto const * it = std::upper_bound(m_featBase, m_featBase + m_numFeatures + 1, seg);
  feature = static_cast<uint32_t>(it - m_featBase) - 1;
  segIdx = seg - m_featBase[feature];
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

uint32_t MappedOverlay::ExitIdx(size_t li, uint32_t cell, uint32_t v) const
{
  auto const & L = m_levels[li];
  auto const * b = L.exits + L.exitOff[cell];
  auto const * e = L.exits + L.exitOff[cell + 1];
  return static_cast<uint32_t>(std::lower_bound(b, e, v) - b);
}
}  // namespace plus_overlay
