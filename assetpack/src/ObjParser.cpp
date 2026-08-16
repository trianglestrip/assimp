#include "assetpack/ObjParser.h"
#include "assetpack/Log.h"

#include "FastParse.h"   // fast number kernels (from_chars fallback inside)

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include <unordered_map>

#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/for_each.hpp>

namespace ap {

using Clock = std::chrono::steady_clock;

static uint64_t microsSince(Clock::time_point t0) {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            Clock::now() - t0).count());
}

// ============================================================
// low-level text scanning over the mapping
// ============================================================
namespace {

inline bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r'; }

// Advance *p to the start of the next non-empty token; returns nullptr
// at line end. Token start/len written to tok/tokLen.
inline const char* nextToken(const char* p, const char* end,
                             const char*& tok, size_t& tokLen) {
    while (p < end && isSpace(*p)) ++p;
    if (p >= end) return nullptr;
    tok = p;
    while (p < end && !isSpace(*p)) ++p;
    tokLen = size_t(p - tok);
    return p;
}

// OBJ numbers overwhelmingly match the fast kernels' shapes; anything
// unusual (exotic exponents, 19+ digits, malformed) falls back to
// std::from_chars inside FastParse.h, preserving the old semantics
inline float parseFloat(const char* s, size_t n) {
    return ap::fast::parseFloat(s, n);
}

inline int64_t parseInt(const char* s, size_t n) {
    return ap::fast::parseInt(s, n);
}

enum class LineKind { Other, V, VN, VT, F, UseMtl, Object, Group, Mtllib };

inline LineKind classify(const char* p, const char* end) {
    if (p >= end) return LineKind::Other;
    switch (p[0]) {
    case 'v':
        if (p + 1 < end) {
            if (isSpace(p[1]))        return LineKind::V;
            if (p[1] == 'n' && p + 2 <= end && (p + 2 == end || isSpace(p[2]))) return LineKind::VN;
            if (p[1] == 't' && p + 2 <= end && (p + 2 == end || isSpace(p[2]))) return LineKind::VT;
        }
        return LineKind::Other;
    case 'f':
        if (p + 1 < end && isSpace(p[1])) return LineKind::F;
        return LineKind::Other;
    case 'u':
        if (end - p >= 7 && std::memcmp(p, "usemtl", 6) == 0 && isSpace(p[6]))
            return LineKind::UseMtl;
        return LineKind::Other;
    case 'o':
        if (p + 1 < end && isSpace(p[1])) return LineKind::Object;
        return LineKind::Other;
    case 'g':
        if (p + 1 < end && isSpace(p[1])) return LineKind::Group;
        return LineKind::Other;
    case 'm':
        if (end - p >= 7 && std::memcmp(p, "mtllib", 6) == 0 && isSpace(p[6]))
            return LineKind::Mtllib;
        return LineKind::Other;
    default:
        return LineKind::Other;
    }
}

// number of vertex tokens on an 'f' line (tokens starting with digit/'-'),
// scanned 8 bytes per iteration (SWAR). Each word is turned into per-byte
// flag masks (flag = top bit 0x80 of each byte): sepHigh marks separators
// (' ' / '\t' / '\r'), startHigh marks the first byte of each maximal
// non-space run (non-sep byte whose predecessor was a separator; the run
// state carried across word boundaries is prevSep, initialized true so a
// token starting at p itself counts). A run start is counted when its byte
// is an ASCII digit or '-'. All per-byte additions stay <= 0xFE, so no
// carries bleed between lanes and every mask is exact per byte; bytes with
// the high bit set are never digits (matching the old signed-char tests).
inline uint32_t countFaceVerts(const char* p, const char* end) {
    constexpr uint64_t kOnes  = 0x0101010101010101ull; // 0x01 per byte
    constexpr uint64_t kHighs = 0x8080808080808080ull; // 0x80 per byte
    constexpr uint64_t kLow7  = 0x7F7F7F7F7F7F7F7Full; // 0x7F per byte
    const uint64_t kSp    = uint64_t(' ') * kOnes;
    const uint64_t kTab   = uint64_t('\t') * kOnes;
    const uint64_t kCr    = uint64_t('\r') * kOnes;
    const uint64_t kMinus = uint64_t('-') * kOnes;

    // exact per-byte equality: 0x80 flag in every byte of w equal to c
    auto eqHigh = [&](uint64_t w, uint64_t c) {
        const uint64_t t = w ^ c;
        return ~(((t & kLow7) + kLow7) | t) & kHighs;
    };

    uint32_t n = 0;
    bool prevByteNonSep = false; // byte before p: none, so acts as separator
    while (end - p >= 8) {
        uint64_t w;
        std::memcpy(&w, p, sizeof w); // unaligned-safe load; p+8 <= end
        const uint64_t sepHigh = eqHigh(w, kSp) | eqHigh(w, kTab) | eqHigh(w, kCr);
        // digit-or-'-': 7-bit range test for 0x30..0x39 (b + 0x50 flags
        // b >= 0x30, b + 0x46 flags b >= 0x3A) plus exact '-' equality;
        // & ~w stops high-bit bytes from aliasing onto the digit range
        const uint64_t b7 = w & kLow7;
        const uint64_t dmHigh = (((b7 + 0x50 * kOnes) & ~(b7 + 0x46 * kOnes) & ~w & kHighs)
                                 | eqHigh(w, kMinus));
        const uint64_t nonSepHigh = ~sepHigh & kHighs;
        // token START: a non-sep byte whose PREDECESSOR is a separator.
        // Within a word the predecessor flag comes from << 8 (byte k-1);
        // byte 0's predecessor is the previous word's last byte. (The
        // shift direction matters: >> 8 would mark run ENDS, which
        // silently inflated the triangle counts.)
        uint64_t prevHigh = nonSepHigh << 8;
        if (prevByteNonSep) prevHigh |= 0x80ull;   // block byte 0 start
        const uint64_t startHigh = nonSepHigh & ~prevHigh;
        prevByteNonSep = (nonSepHigh >> 56) != 0;  // this word's last byte
        // one 0x80 hit bit per counted byte: >> 3 makes lanes 0x10, the
        // multiply by kOnes sums them into the top byte carry-free, and
        // >> 60 extracts the count (<= 8 per word)
        n += uint32_t((((startHigh & dmHigh) >> 3) * kOnes) >> 60);
        p += 8;
    }
    for (; p < end; ++p) { // tail: fewer than 8 bytes, byte-at-a-time
        const unsigned char c = (unsigned char)*p;
        const bool sep = (c == ' ' || c == '\t' || c == '\r');
        if (!sep) {
            if (!prevByteNonSep && ((c - '0') <= 9u || c == '-')) ++n;
            prevByteNonSep = true;
        } else {
            prevByteNonSep = false;
        }
    }
    return n;
}

// Resolve a 1-based or negative-relative OBJ index against its pool.
// Negative indices count back from the last definition so far, which
// is base (chunk prefix) + running (definitions seen within the chunk).
inline int32_t resolveIndex(int64_t raw, size_t base, size_t running) {
    if (raw > 0) return int32_t(raw - 1);
    return int32_t(int64_t(base + running) + raw);
}

// ---- texture-seam splitting ------------------------------------------------
// An OBJ "vertex" is really the (v, vt, vn) triplet. One position can be
// referenced with different vt/vn combos (texture seams, mirrored uvs,
// atlases); collapsing those onto the position with an arbitrary winner
// maps the wrong uv region onto some faces. Pass 2 claims each
// per-position slot with the FIRST (v, vt, vn) combo and splits every
// conflicting combo into its own output vertex.
struct SeamVert {
    uint32_t v, vt, vn;   // pool indices; 0xFFFFFFFF = part absent
};
// Open-addressing map for seam lookups: the sharded unordered_map made
// ~3M heap node allocations under a lock per load (the dominant cost of
// pass2 after seam splitting landed); a flat power-of-two table with
// linear probing keeps every operation allocation-free and cache-local.
// Key = (v, vt, vn) split across two lanes (68 bits won't fit one u64).
struct SeamShard {
    std::mutex mx;
    std::vector<uint64_t> keys0;   // vt | (vn << 32)
    std::vector<uint32_t> keys1;   // v
    std::vector<uint32_t> vals;    // output vertex; 0xFFFFFFFF = empty
    std::vector<std::pair<uint32_t, SeamVert>> added;   // data for seamFill
    size_t mask = 0;
    size_t count = 0;

    void seed(size_t slots) {
        size_t s = 1024;
        while (s < slots) s <<= 1;
        keys0.assign(s, 0);
        keys1.assign(s, 0);
        vals.assign(s, 0xFFFFFFFFu);
        mask = s - 1;
        count = 0;
    }
    size_t probe(uint32_t v, uint64_t k0) const {
        size_t i = size_t((k0 ^ (uint64_t(v) * 0x9E3779B97F4A7C15ull))
                          >> 20) & mask;
        while (vals[i] != 0xFFFFFFFFu
               && !(keys1[i] == v && keys0[i] == k0))
            i = (i + 1) & mask;
        return i;
    }
    void grow() {   // caller holds mx
        std::vector<uint64_t> o0 = std::move(keys0);
        std::vector<uint32_t> o1 = std::move(keys1);
        std::vector<uint32_t> ov = std::move(vals);
        seed((mask + 1) * 2);
        for (size_t i = 0; i < ov.size(); ++i) {
            if (ov[i] == 0xFFFFFFFFu) continue;
            const size_t j = probe(o1[i], o0[i]);
            keys0[j] = o0[i];
            keys1[j] = o1[i];
            vals[j] = ov[i];
            ++count;
        }
    }
};

// first writer claims the slot; a different value already there = seam
inline bool claimIdx(uint32_t& slot, uint32_t val) {
    std::atomic_ref<uint32_t> a(slot);
    uint32_t cur = a.load(std::memory_order_relaxed);
    for (;;) {
        if (cur == val) return true;
        if (cur != 0xFFFFFFFFu) return false;
        if (a.compare_exchange_weak(cur, val, std::memory_order_relaxed))
            return true;
    }
}

// Parse one 'f' line; invoke fn(v, vt, vn) per vertex with resolved
// pool indices (-1 when that part is absent). The three parts share
// no state, so a single pass serves both the index writer and the
// later normal/uv expander.
template <typename Fn>
inline void forEachFaceVertex(const char* p, const char* eol,
                              size_t baseV, size_t runV,
                              size_t baseUv, size_t runUv,
                              size_t baseNrm, size_t runNrm, Fn&& fn) {
    const char* q = p + 1;
    const char* tok; size_t len;
    while ((q = nextToken(q, eol, tok, len)) != nullptr) {
        if (!((tok[0] >= '0' && tok[0] <= '9') || tok[0] == '-'))
            continue;
        // split "a/b/c" (each part optional)
        const char* seg[3] = {nullptr, nullptr, nullptr};
        size_t segLen[3] = {0, 0, 0};
        int part = 0;
        for (size_t i = 0; i < len; ++i) {
            if (tok[i] == '/') part = std::min(part + 1, 2);
            else {
                if (!seg[part]) seg[part] = tok + i;
                segLen[part] = size_t(tok + i - seg[part]) + 1;
            }
        }
        const int32_t v  = segLen[0] ? resolveIndex(parseInt(seg[0], segLen[0]),
                                                     baseV, runV) : -1;
        const int32_t vt = segLen[1] ? resolveIndex(parseInt(seg[1], segLen[1]),
                                                     baseUv, runUv) : -1;
        const int32_t vn = segLen[2] ? resolveIndex(parseInt(seg[2], segLen[2]),
                                                     baseNrm, runNrm) : -1;
        fn(v, vt, vn);
    }
}

struct Marker {
    LineKind kind;
    uint32_t localTri;      // triangles parsed before this marker in its chunk
    const char* name;
    uint32_t nameLen;
};

struct ChunkInfo {
    size_t begin = 0, end = 0;    // byte range, begin/end on line boundaries
    // pass 1 outputs
    size_t nPos = 0, nNrm = 0, nUv = 0, nTri = 0;
    std::vector<Marker> markers;
    // prefix bases (filled between passes)
    size_t basePos = 0, baseNrm = 0, baseUv = 0, baseTri = 0;
};

// split [0,size) into `n` chunks snapped to '\n' boundaries
std::vector<ChunkInfo> makeChunks(const char* data, size_t size, size_t n) {
    std::vector<ChunkInfo> chunks;
    chunks.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        ChunkInfo c;
        c.begin = size * i / n;
        c.end   = size * (i + 1) / n;
        if (c.begin > size) c.begin = size;
        // snap begin forward to the next line start
        while (c.begin > 0 && c.begin < size && data[c.begin - 1] != '\n') ++c.begin;
        // snap end forward to include the partial line
        while (c.end < size && c.end > 0 && data[c.end - 1] != '\n') ++c.end;
        if (c.end < c.begin) c.end = c.begin;
        chunks.push_back(c);
    }
    return chunks;
}

// ---- intermediate parse products (not part of the public result) ----
struct RawGroup {
    std::string_view name;          // 'o'/'g' name (view into obj mapping)
    std::string_view material;      // usemtl name (view into obj mapping)
    uint32_t firstTriangle = 0;
    uint32_t triangleCount = 0;
};

struct RawMaterial {
    std::string_view name;          // newmtl (view into mtl mapping)
    float Kd[3] = {0.8f, 0.8f, 0.8f};
    float Ka[3] = {0.f, 0.f, 0.f};
    float Ks[3] = {0.f, 0.f, 0.f};
    float Ns = 10.f;
    float d = 1.f;                  // dissolve (alpha)
    int32_t illum = 1;
    // texture refs (views into mtl mapping; empty when absent)
    std::string_view mapKd, mapKa, mapKs, mapD, mapBump;
};

inline int slotToTexType(std::string_view slot) {
    if (slot == "map_Kd")   return TexDiffuse;
    if (slot == "map_Ka")   return TexAmbient;
    if (slot == "map_Ks")   return TexSpecular;
    if (slot == "map_d")    return TexOpacity;
    if (slot == "map_Bump") return TexNormal;
    return TexDiffuse;
}

} // namespace

// ============================================================
// graph
// ============================================================
struct ObjParser::FlowDeleter { void operator()(tf::Taskflow* f) const { delete f; } };

ObjParser::ObjParser(unsigned threads)
    : executor_(std::make_unique<tf::Executor>(threads == 0
              ? std::thread::hardware_concurrency() : threads)) {}

ObjParser::~ObjParser() {
    // draining the executor joins any in-flight async load
    const auto t0 = Clock::now();
    executor_.reset();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - t0).count();
    if (ms > 5)
        AP_LOG("obj", "async executor drained in %lld ms",
               static_cast<long long>(ms));
}

PackResult& ObjParser::result() { return *result_; }

bool ObjParser::load(std::string_view path) { return runGraph(path, false); }
void ObjParser::loadAsync(std::string_view path) { runGraph(path, true); }

bool ObjParser::runGraph(std::string_view path, bool async) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();

    auto flow = std::shared_ptr<void>(new tf::Taskflow, FlowDeleter{});
    flow_ = flow; // keep alive for async completion
    auto& tf = *static_cast<tf::Taskflow*>(flow.get());

    const auto t0 = Clock::now();
    auto result = result_;
    auto pathStr = std::make_shared<std::string>(path);

    // raw parse products shared between the parallel stages
    auto groups = std::make_shared<std::vector<RawGroup>>();
    auto rawMats = std::make_shared<std::vector<RawMaterial>>();
    auto mtlNames = std::make_shared<std::vector<std::string_view>>();
    // raw vn/vt pools, indexed by their own counters; the expand pass
    // (pass 3) re-indexes them per vertex into result->normals/texcoords
    auto vnPool = std::make_shared<std::vector<float>>();
    auto uvPool = std::make_shared<std::vector<float>>();
    // per-vertex vn/vt reference (relaxed atomic: several faces may
    // reference one vertex from different chunks; the last writer wins)
    auto vnIdx = std::make_shared<std::vector<uint32_t>>();
    auto uvIdx = std::make_shared<std::vector<uint32_t>>();
    // seam-split state: sharded (v, vt, vn) -> output-vertex table
    auto shards = std::make_shared<std::array<SeamShard, 16>>();
    auto seamCounter = std::make_shared<std::atomic<uint32_t>>(0);
    auto baseVerts = std::make_shared<size_t>(0);

    // ---- stage 1: mmap obj + quick mtllib head-scan ----
    tf::Task mapTask = tf.emplace([this, result, pathStr]() {
        const auto tStart = Clock::now();
        result->objFile = MappedFile::openShared(*pathStr);
        if (!result->objFile) {
            lastError_ = "cannot open file: " + *pathStr;
            AP_LOG_WARN("obj", "%s", lastError_.c_str());
            return;
        }
        const auto slash = pathStr->find_last_of("/\\");
        if (slash != std::string::npos)
            result->sourceDir = pathStr->substr(0, slash);

        // mtllib is conventionally near the top; scan the head only
        // (ponytail: first 4 MB, else materials come back empty)
        const auto& bytes = result->objFile->text();
        const size_t head = std::min<size_t>(bytes.size(), size_t(4) << 20);
        size_t off = 0;
        while (off < head) {
            size_t eol = bytes.find('\n', off);
            if (eol == std::string_view::npos) eol = head;
            const std::string_view line = bytes.substr(off, eol - off);
            if (!line.empty() && classify(line.data(), line.data() + line.size())
                    == LineKind::Mtllib) {
                const char* tok; size_t len;
                if (nextToken(line.data() + 7, line.data() + line.size(), tok, len)) {
                    std::string mtlPath(tok, len);
                    std::string full = result->sourceDir.empty()
                        ? mtlPath : result->sourceDir + "/" + mtlPath;
                    result->mtlFile = MappedFile::openShared(full);
                    if (!result->mtlFile)
                        AP_LOG_WARN("obj", "mtllib '%s' not found", mtlPath.c_str());
                    else
                        AP_LOG("obj", "mtl mapped: %s (%zu bytes)",
                               mtlPath.c_str(), result->mtlFile->size());
                    break;
                }
            }
            off = eol + 1;
        }
        fireProgress(5.f);
        AP_LOG("obj", "obj mapped: %zu bytes in %llu us",
               result->objFile->size(),
               static_cast<unsigned long long>(microsSince(tStart)));
    });

    // ---- stage 2a: geometry (chunk-parallel two-pass + expand) ----
    tf::Task geometryTask = tf.emplace([this, result, groups, mtlNames, vnPool, uvPool, vnIdx, uvIdx, shards, seamCounter, baseVerts](tf::Subflow& sf) {
        if (!result->objFile) return;
        const auto tStart = Clock::now();
        AP_LOG("geometry", "begin");
        // local copy for the subflow lambdas (read once, before any task)
        const bool wantN = wantNormals_;
        if (!wantN) AP_LOG("geometry", "normals skipped (wantNormals=false)");
        const std::string_view text = result->objFile->text();
        const char* data = text.data();
        const size_t size = text.size();

        const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
        const int nChunks = int(std::min<size_t>(size / (1 << 16) + 1, size_t(hw) * 8));
        auto chunks = std::make_shared<std::vector<ChunkInfo>>(
            makeChunks(data, size, size_t(nChunks)));

        // pass 1 (parallel): per-chunk counts + ordered markers
        tf::Task pass1 = sf.for_each_index(0, nChunks, 1, [chunks, data](int ci) {
            ChunkInfo& c = (*chunks)[size_t(ci)];
            const char* p = data + c.begin;
            const char* end = data + c.end;
            while (p < end) {
                const char* eol = static_cast<const char*>(
                    memchr(p, '\n', size_t(end - p)));
                if (!eol) eol = end;
                const LineKind k = classify(p, eol);
                switch (k) {
                case LineKind::V:  ++c.nPos; break;
                case LineKind::VN: ++c.nNrm; break;
                case LineKind::VT: ++c.nUv;  break;
                case LineKind::F: {
                    const uint32_t verts = countFaceVerts(p + 1, eol);
                    if (verts >= 3) c.nTri += verts - 2;
                    break;
                }
                case LineKind::UseMtl: case LineKind::Object:
                case LineKind::Group: case LineKind::Mtllib: {
                    const size_t skip = (k == LineKind::UseMtl || k == LineKind::Mtllib) ? 7 : 1;
                    const char* tok; size_t len;
                    if (nextToken(p + skip, eol, tok, len))
                        c.markers.push_back({ k, uint32_t(c.nTri), tok, uint32_t(len) });
                    break;
                }
                default: break;
                }
                p = eol + 1;
            }
        });

        // prefix sums + flat-array allocation (serial, tiny)
        tf::Task prefix = sf.emplace([chunks, result, vnPool, uvPool, vnIdx, uvIdx, nChunks, wantN, baseVerts, shards, tStart]() {
            AP_LOG("geometry", "pass1 done at %llu us",
                   static_cast<unsigned long long>(microsSince(tStart)));
            size_t p = 0, n = 0, u = 0, t = 0;
            for (int i = 0; i < nChunks; ++i) {
                ChunkInfo& c = (*chunks)[size_t(i)];
                c.basePos = p; c.baseNrm = n; c.baseUv = u; c.baseTri = t;
                p += c.nPos; n += c.nNrm; u += c.nUv; t += c.nTri;
            }
            result->positions.resize(p * 3);
            result->posIndices.resize(t * 3);
            if (wantN) vnPool->resize(n * 3);
            uvPool->resize(u * 2);
            // per-vertex vn/vt reference slots; 0xFFFFFFFF marks "no
            // reference yet" so unreferenced vertices keep zero normals.
            // vnIdx stays empty when normals are not wanted; pass 2's
            // store and pass 3's expansion both turn off on that.
            if (wantN) vnIdx->assign(p, 0xFFFFFFFFu);
            uvIdx->assign(p, 0xFFFFFFFFu);
            // expansion targets (per-vertex indexed, filled by pass 3)
            if (wantN && n) result->normals.assign(p * 3, 0.f);
            if (u) result->texcoords.assign(p * 2, 0.f);
            *baseVerts = p;   // seam splits append past this in seamFill
            // seed the seam shards (they grow themselves on load): San
            // Miguel splits half its positions, so start generous
            for (SeamShard& sh : *shards) {
                sh.seed(1u << 17);
                sh.added.reserve(1u << 17);
            }
            AP_LOG("geometry", "pass1: %zu v / %zu vn / %zu vt / %zu tris",
                   p, n, u, t);
        });

        // pass 2 (parallel): fill arrays; fan-triangulate faces and
        // resolve 1-based / negative-relative indices per chunk
        tf::Task pass2 = sf.for_each_index(0, nChunks, 1,
            [chunks, result, vnPool, uvPool, vnIdx, uvIdx, data, wantN,
             shards, seamCounter, baseVerts](int ci) {
                ChunkInfo& c = (*chunks)[size_t(ci)];
                const char* p = data + c.begin;
                const char* end = data + c.end;
                size_t lp = c.basePos, ln = c.baseNrm, lu = c.baseUv;
                float* pos = result->positions.data();
                float* nrm = vnPool->data();
                float* uv  = uvPool->data();
                uint32_t* pi = result->posIndices.data() + c.baseTri * 3;

                while (p < end) {
                    const char* eol = static_cast<const char*>(
                        memchr(p, '\n', size_t(end - p)));
                    if (!eol) eol = end;
                    const LineKind k = classify(p, eol);
                    const char* tok; size_t len; const char* q;
                    if (k == LineKind::V) {
                        q = p + 1;
                        for (int comp = 0; comp < 3; ++comp) {
                            if (!(q = nextToken(q, eol, tok, len))) break;
                            pos[lp * 3 + comp] = parseFloat(tok, len);
                        }
                        ++lp;
                    } else if (k == LineKind::VN) {
                        // the counter advances even when the floats are
                        // skipped: chunk prefix bases and negative-index
                        // resolution depend on vn counts regardless. The
                        // per-vertex vnIdx store below is disabled by its
                        // !vnIdx->empty() guard (vnIdx is left empty when
                        // !wantN) -- do not weaken that guard.
                        if (wantN) {
                            q = p + 2;
                            for (int comp = 0; comp < 3; ++comp) {
                                if (!(q = nextToken(q, eol, tok, len))) break;
                                nrm[ln * 3 + comp] = parseFloat(tok, len);
                            }
                        }
                        ++ln;
                    } else if (k == LineKind::VT) {
                        q = p + 2;
                        for (int comp = 0; comp < 2; ++comp) {
                            if (!(q = nextToken(q, eol, tok, len))) break;
                            uv[lu * 2 + comp] = parseFloat(tok, len);
                        }
                        ++lu;
                    } else if (k == LineKind::F) {
                        // fan triangulation: emit (v0, vi-1, vi) from the
                        // third vertex on; pass1 counts verts-2, so exactly
                        // verts-2 triangles are written per face
                        uint32_t ip0 = 0, prevIp = 0;
                        uint32_t fv = 0;
                        forEachFaceVertex(p, eol, c.basePos, lp,
                                          c.baseUv, lu, c.baseNrm, ln,
                            [&](int32_t v, int32_t vt, int32_t vn) {
                                if (v < 0) return;
                                const uint32_t ip = uint32_t(v);
                                const uint32_t vtU =
                                    vt >= 0 ? uint32_t(vt) : 0xFFFFFFFFu;
                                const uint32_t vnU =
                                    vn >= 0 ? uint32_t(vn) : 0xFFFFFFFFu;
                                // first (vt, vn) combo claims the position
                                // slot; a conflicting combo is a texture
                                // seam and splits into its own vertex
                                bool uniq = true;
                                if (vtU != 0xFFFFFFFFu && !uvIdx->empty()
                                    && !claimIdx((*uvIdx)[ip], vtU))
                                    uniq = false;
                                if (uniq && vnU != 0xFFFFFFFFu
                                    && !vnIdx->empty()
                                    && !claimIdx((*vnIdx)[ip], vnU))
                                    uniq = false;
                                uint32_t out = ip;
                                if (!uniq) {
                                    const SeamVert sv{ip, vtU, vnU};
                                    const uint64_t k0 =
                                        uint64_t(vtU)
                                        | (uint64_t(vnU) << 32);
                                    SeamShard& sh = (*shards)[size_t(
                                        (k0 ^ (uint64_t(ip)
                                               * 0x9E3779B97F4A7C15ull))
                                        >> 20) & 15];
                                    std::lock_guard<std::mutex> lk(sh.mx);
                                    size_t i = sh.probe(ip, k0);
                                    if (sh.vals[i] != 0xFFFFFFFFu) {
                                        out = sh.vals[i];
                                    } else {
                                        if ((sh.count + 1) * 4
                                            >= (sh.mask + 1) * 3) {
                                            sh.grow();
                                            i = sh.probe(ip, k0);
                                        }
                                        out = uint32_t(*baseVerts
                                                       + seamCounter->fetch_add(1));
                                        sh.keys0[i] = k0;
                                        sh.keys1[i] = ip;
                                        sh.vals[i] = out;
                                        ++sh.count;
                                        sh.added.emplace_back(out, sv);
                                    }
                                }
                                if (fv == 0) ip0 = out;
                                else if (fv >= 2) {
                                    *pi++ = ip0; *pi++ = prevIp; *pi++ = out;
                                }
                                prevIp = out;
                                ++fv;
                            });
                    }
                    p = eol + 1;
                }
            });

        // pass 3 (parallel): expand the per-face vn/vt references into
        // per-vertex arrays so that normals/texcoords index 1:1 with
        // positions (the render contract). pass 2 recorded each face's
        // reference per vertex into vnIdx/uvIdx; this pass walks the
        // vertex range and copies, so every slot has exactly one writer
        // (no cross-chunk races). Vertices no face references keep
        // their zero default (viewer falls back to face normals).
        // The loop range depends on positions, which the prefix task
        // resizes only after this subflow is built - the range must be
        // evaluated when this task RUNS, not when the graph is declared
        // (the old form captured an empty range and silently skipped
        // the whole pass, leaving every texcoord/normal zero).
        tf::Task pass3 = sf.emplace(
            [result, vnIdx, uvIdx, vnPool, uvPool, tStart,
             baseVerts](tf::Subflow& inner) {
                // base slots only: seam splits were filled directly by
                // seamFill, and vnIdx/uvIdx stop at the base count
                const int nVerts = int(*baseVerts);
                if (nVerts <= 0) return;
                const auto t3 = Clock::now();
                inner.for_each_index(0, nVerts, 1,
                    [result, vnIdx, uvIdx, vnPool, uvPool](int i) {
                        const size_t iv = size_t(i);
                        const float* vn = vnPool->data();
                        const float* uv = uvPool->data();
                        if (!result->normals.empty()) {
                            const uint32_t in =
                                std::atomic_ref<const uint32_t>((*vnIdx)[iv])
                                    .load(std::memory_order_relaxed);
                            if (in != 0xFFFFFFFFu) {
                                const size_t in3 = size_t(in) * 3;
                                float* dst = result->normals.data() + iv * 3;
                                dst[0] = vn[in3];
                                dst[1] = vn[in3 + 1];
                                dst[2] = vn[in3 + 2];
                            }
                        }
                        if (!result->texcoords.empty()) {
                            const uint32_t iu =
                                std::atomic_ref<const uint32_t>((*uvIdx)[iv])
                                    .load(std::memory_order_relaxed);
                            if (iu != 0xFFFFFFFFu) {
                                const size_t iu2 = size_t(iu) * 2;
                                float* dst = result->texcoords.data() + iv * 2;
                                dst[0] = uv[iu2];
                                dst[1] = uv[iu2 + 1];
                            }
                        }
                    });
                inner.join();
                AP_LOG("geometry", "pass3 gather: %llu us over %d verts",
                       static_cast<unsigned long long>(
                           std::chrono::duration_cast<std::chrono::microseconds>(
                               Clock::now() - t3).count()),
                       nVerts);
            });

        // build groups from ordered markers; then pack PackMesh views
        // (per-group spans into the pools + per-group bounds) in parallel
        tf::Task build = sf.emplace([this, result, chunks, groups, mtlNames,
                                     nChunks, tStart](tf::Subflow& inner) {
            RawGroup cur;
            cur.firstTriangle = 0;
            auto closeGroup = [&](uint32_t endTri) {
                if (endTri > cur.firstTriangle) {
                    cur.triangleCount = endTri - cur.firstTriangle;
                    groups->push_back(cur);
                }
            };
            for (int i = 0; i < nChunks; ++i) {
                const ChunkInfo& c = (*chunks)[size_t(i)];
                for (const Marker& m : c.markers) {
                    const uint32_t t = uint32_t(c.baseTri) + m.localTri;
                    switch (m.kind) {
                    case LineKind::UseMtl:
                        closeGroup(t);
                        cur.firstTriangle = t;
                        cur.material = std::string_view(m.name, m.nameLen);
                        break;
                    case LineKind::Object:
                    case LineKind::Group:
                        closeGroup(t);
                        cur.firstTriangle = t;
                        cur.name = std::string_view(m.name, m.nameLen);
                        break;
                    default: break;
                    }
                }
            }
            closeGroup(uint32_t(result->posIndices.size() / 3));

            const auto tPack = Clock::now();

            // PackMesh views: positions/normals/texcoords point at the
            // whole pool (OBJ indices are pool-relative); indices are
            // the group's triangle range. materialIndex is a temporary
            // first-seen order of usemtl names (real index bound by the
            // bind task once materials finished).
            const uint32_t poolVerts = uint32_t(result->positions.size() / 3);
            result->meshes.reserve(groups->size());
            std::span<const float> allPos(result->positions);
            std::span<const float> allNrm(result->normals);
            std::span<const float> allUv(result->texcoords);
            for (const RawGroup& g : *groups) {
                PackMesh pm;
                pm.name = g.name;
                pm.positions = allPos;
                pm.normals   = allNrm;
                pm.texcoords = allUv;
                pm.indices = { result->posIndices.data()
                                   + size_t(g.firstTriangle) * 3,
                               size_t(g.triangleCount) * 3 };
                int mi = -1;
                if (!g.material.empty()) {
                    auto it = std::find(mtlNames->begin(), mtlNames->end(),
                                        g.material);
                    if (it == mtlNames->end()) {
                        mi = int(mtlNames->size());
                        mtlNames->push_back(g.material);
                    } else {
                        mi = int(it - mtlNames->begin());
                    }
                }
                pm.materialIndex = mi;
                result->meshes.push_back(pm);
            }
            (void)poolVerts;

            // per-mesh bounds from the referenced pool vertices (parallel)
            const int n = int(result->meshes.size());
            inner.for_each_index(0, n, 1, [result](int i) {
                PackMesh& pm = result->meshes[size_t(i)];
                float mn[3] = { std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max() };
                float mx[3] = { -std::numeric_limits<float>::max(),
                                -std::numeric_limits<float>::max(),
                                -std::numeric_limits<float>::max() };
                for (uint32_t idx : pm.indices) {
                    const float* p = pm.positions.data() + size_t(idx) * 3;
                    for (int k = 0; k < 3; ++k) {
                        mn[k] = std::min(mn[k], p[k]);
                        mx[k] = std::max(mx[k], p[k]);
                    }
                }
                for (int k = 0; k < 3; ++k) {
                    pm.boundsMin[k] = mn[k];
                    pm.boundsMax[k] = mx[k];
                }
            });
            inner.join();

            result->importMicros = microsSince(tStart);
            result->verticesMicros = microsSince(tPack);
            AP_LOG("geometry", "done in %llu us: %u verts, %u tris, %zu groups",
                   static_cast<unsigned long long>(result->importMicros),
                   poolVerts, uint32_t(result->posIndices.size() / 3),
                   groups->size());

            fireProgress(100.f);
            fireVertices(*result, result->meshes);
            AP_LOG("event", "onVerticesReady fired (%zu meshes)",
                   result->meshes.size());
        });

        // seam resolve: grow the pools by the split vertices and fill
        // their data (position copy from the base slot + their own
        // uv/normal straight from the vn/vt pools). Runs once, after all
        // pass2 chunks finish, so every split index is final here.
        tf::Task seamFill = sf.emplace([result, vnPool, uvPool, shards,
                                        seamCounter,
                                        baseVerts](tf::Subflow& inner) {
            const size_t base = *baseVerts;
            const size_t n = seamCounter->load();
            if (!n) return;
            result->positions.resize((base + n) * 3);
            if (!result->normals.empty())
                result->normals.resize((base + n) * 3);
            if (!result->texcoords.empty())
                result->texcoords.resize((base + n) * 2);
            // flatten the shard lists (cheap pointer gather), then fill
            // in parallel: entries write disjoint output slots
            std::vector<const std::pair<uint32_t, SeamVert>*> entries;
            size_t total = 0;
            for (const SeamShard& sh : *shards) total += sh.added.size();
            entries.reserve(total);
            for (const SeamShard& sh : *shards)
                for (const auto& e : sh.added) entries.push_back(&e);
            float* pos = result->positions.data();
            float* nrm = result->normals.data();
            float* uv = result->texcoords.data();
            const float* np = vnPool->data();
            const float* up = uvPool->data();
            const bool wantNrm = !result->normals.empty();
            const bool wantUv = !result->texcoords.empty();
            inner.for_each_index(0, int(total), 1, [&](int i) {
                const auto& [idx, sv] = *entries[size_t(i)];
                // dst is always past base, src below it: no overlap
                const size_t d = size_t(idx) * 3, s = size_t(sv.v) * 3;
                pos[d] = pos[s];
                pos[d + 1] = pos[s + 1];
                pos[d + 2] = pos[s + 2];
                if (wantNrm && sv.vn != 0xFFFFFFFFu) {
                    float* dn = nrm + d;
                    const float* sn = np + size_t(sv.vn) * 3;
                    dn[0] = sn[0]; dn[1] = sn[1]; dn[2] = sn[2];
                }
                if (wantUv && sv.vt != 0xFFFFFFFFu) {
                    float* dt = uv + size_t(idx) * 2;
                    const float* st = up + size_t(sv.vt) * 2;
                    dt[0] = st[0]; dt[1] = st[1];
                }
            });
            inner.join();
            AP_LOG("geometry", "seam split: %zu extra vertices (%zu -> %zu)",
                   n, base, base + n);
        });

        // per-stage timing: prefix runs right after pass1 completes, the
        // stamp task right after pass2, and the pass3 wrapper times its
        // own gather - so one load logs where the geometry time goes
        tf::Task stamp2 = sf.emplace([tStart]() {
            AP_LOG("geometry", "pass2 done at %llu us",
                   static_cast<unsigned long long>(microsSince(tStart)));
        });

        pass1.precede(prefix);
        prefix.precede(pass2);
        pass2.precede(seamFill, stamp2);   // stamp2 is a side branch: it
                                           // logs without blocking pass3
        seamFill.precede(pass3);
        pass3.precede(build);
        sf.join();
    });

    // ---- stage 2b: materials (parse + convert, parallel to geometry) ----
    tf::Task materialsTask = tf.emplace([this, result, rawMats](tf::Subflow& sf) {
        if (!result->mtlFile) return;
        const auto tStart = Clock::now();
        AP_LOG("materials", "begin");

        const std::string_view text = result->mtlFile->text();
        const char* p = text.data();
        const char* end = p + text.size();

        auto cur = [&]() -> RawMaterial* {
            return rawMats->empty() ? nullptr : &rawMats->back();
        };
        auto setPath = [](std::string_view& dst,
                          const char* tok, size_t len) {
            // strip surrounding quotes when present
            if (len >= 2 && tok[0] == '"') { ++tok; len -= 2; }
            dst = std::string_view(tok, len);
        };

        while (p < end) {
            const char* eol = static_cast<const char*>(memchr(p, '\n', size_t(end - p)));
            if (!eol) eol = end;
            const char* tok; size_t len;
            const char* q = p;

            if (eol - p > 7 && memcmp(p, "newmtl ", 7) == 0) {
                if (nextToken(p + 7, eol, tok, len)) {
                    rawMats->emplace_back();
                    cur()->name = std::string_view(tok, len);
                }
            } else if (RawMaterial* m = cur()) {
                q = p;
                if (eol - p > 3 && memcmp(p, "Kd ", 3) == 0) {
                    q = p + 3;
                    for (int i = 0; i < 3; ++i) {
                        if (!(q = nextToken(q, eol, tok, len))) break;
                        m->Kd[i] = parseFloat(tok, len);
                    }
                } else if (eol - p > 3 && memcmp(p, "Ka ", 3) == 0) {
                    q = p + 3;
                    for (int i = 0; i < 3; ++i) {
                        if (!(q = nextToken(q, eol, tok, len))) break;
                        m->Ka[i] = parseFloat(tok, len);
                    }
                } else if (eol - p > 3 && memcmp(p, "Ks ", 3) == 0) {
                    q = p + 3;
                    for (int i = 0; i < 3; ++i) {
                        if (!(q = nextToken(q, eol, tok, len))) break;
                        m->Ks[i] = parseFloat(tok, len);
                    }
                } else if (eol - p > 3 && memcmp(p, "Ns ", 3) == 0) {
                    if (nextToken(p + 3, eol, tok, len)) m->Ns = parseFloat(tok, len);
                } else if (eol - p > 2 && memcmp(p, "d ", 2) == 0) {
                    if (nextToken(p + 2, eol, tok, len)) m->d = parseFloat(tok, len);
                } else if (eol - p > 6 && memcmp(p, "illum ", 6) == 0) {
                    if (nextToken(p + 6, eol, tok, len))
                        m->illum = int32_t(parseInt(tok, len));
                } else if (eol - p > 7 && memcmp(p, "map_Kd ", 7) == 0) {
                    if (nextToken(p + 7, eol, tok, len)) setPath(m->mapKd, tok, len);
                } else if (eol - p > 7 && memcmp(p, "map_Ka ", 7) == 0) {
                    if (nextToken(p + 7, eol, tok, len)) setPath(m->mapKa, tok, len);
                } else if (eol - p > 7 && memcmp(p, "map_Ks ", 7) == 0) {
                    if (nextToken(p + 7, eol, tok, len)) setPath(m->mapKs, tok, len);
                } else if (eol - p > 6 && memcmp(p, "map_d ", 6) == 0) {
                    if (nextToken(p + 6, eol, tok, len)) setPath(m->mapD, tok, len);
                } else if (eol - p > 9 && memcmp(p, "map_Bump ", 9) == 0) {
                    if (nextToken(p + 9, eol, tok, len)) setPath(m->mapBump, tok, len);
                } else if (eol - p > 5 && memcmp(p, "bump ", 5) == 0) {
                    if (nextToken(p + 5, eol, tok, len)) setPath(m->mapBump, tok, len);
                }
            }
            p = eol + 1;
        }

        // convert to the public material form (Kd -> diffuse etc.)
        result->materials.reserve(rawMats->size());
        for (const RawMaterial& m : *rawMats) {
            PackMaterial pm;
            pm.name = m.name;
            pm.diffuse[0] = m.Kd[0]; pm.diffuse[1] = m.Kd[1];
            pm.diffuse[2] = m.Kd[2]; pm.diffuse[3] = 1.f;
            pm.ambient[0] = m.Ka[0]; pm.ambient[1] = m.Ka[1];
            pm.ambient[2] = m.Ka[2]; pm.ambient[3] = 1.f;
            pm.specular[0] = m.Ks[0]; pm.specular[1] = m.Ks[1];
            pm.specular[2] = m.Ks[2]; pm.specular[3] = 1.f;
            pm.opacity = m.d;
            pm.shininess = m.Ns;
            auto add = [&](const std::string_view& path, int type) {
                if (!path.empty()) pm.textures.push_back({ type, 0, path });
            };
            add(m.mapKd,   TexDiffuse);
            add(m.mapKa,   TexAmbient);
            add(m.mapKs,   TexSpecular);
            add(m.mapD,    TexOpacity);
            add(m.mapBump, TexNormal);
            result->materials.push_back(pm);
        }

        result->materialsMicros = microsSince(tStart);
        AP_LOG("materials", "done in %llu us: %zu materials",
               static_cast<unsigned long long>(result->materialsMicros),
               result->materials.size());
        sf.join();
    });

    // ---- stage 2c: textures (from materials) ----
    tf::Task texturesTask = tf.emplace([this, result, rawMats]() {
        if (!result->mtlFile) return;
        const auto tStart = Clock::now();

        struct Ref { std::string_view path; std::string_view slot; };
        std::vector<Ref> refs;
        for (const RawMaterial& m : *rawMats) {
            auto add = [&](const std::string_view& p, std::string_view slot) {
                if (!p.empty()) refs.push_back({ p, slot });
            };
            add(m.mapKd,   "map_Kd");
            add(m.mapKa,   "map_Ka");
            add(m.mapKs,   "map_Ks");
            add(m.mapD,    "map_d");
            add(m.mapBump, "map_Bump");
        }

        result->textures.reserve(refs.size());
        for (const Ref& r : refs) {
            PackTexture pt;
            pt.path = r.path;
            pt.type = slotToTexType(r.slot);
            pt.slot = 0;
            pt.embedded = false;
            pt.resolvedPath = result->sourceDir.empty()
                ? std::string(r.path)
                : result->sourceDir + "/" + std::string(r.path);
            result->textures.push_back(pt);
        }

        result->texturesMicros = microsSince(tStart);
        AP_LOG("textures", "done in %llu us: %zu refs",
               static_cast<unsigned long long>(result->texturesMicros),
               result->textures.size());

        fireTextures(*result, result->textures);
        AP_LOG("event", "onTexturesReady fired (%zu refs)",
               result->textures.size());
    });

    // ---- stage 2d: bind usemtl -> material index, then fire materials ----
    tf::Task bindTask = tf.emplace([this, result, mtlNames]() {
        const auto tStart = Clock::now();
        std::unordered_map<std::string_view, int> byName;
        byName.reserve(result->materials.size() * 2);
        for (int i = 0; i < int(result->materials.size()); ++i)
            byName.emplace(result->materials[size_t(i)].name, i);

        for (PackMesh& m : result->meshes) {
            if (m.materialIndex < 0) continue;
            const std::string_view usemtl = (*mtlNames)[size_t(m.materialIndex)];
            const auto it = byName.find(usemtl);
            m.materialIndex = it == byName.end() ? -1 : it->second;
        }
        AP_LOG("materials", "bound usemtl -> material indices in %llu us",
               static_cast<unsigned long long>(microsSince(tStart)));

        fireMaterials(*result, result->materials);
        AP_LOG("event", "onMaterialsReady fired (%zu materials)",
               result->materials.size());
    });

    // ---- sink ----
    tf::Task doneTask = tf.emplace([this, result, t0]() {
        result->totalMicros = microsSince(t0);
        const bool ok = result->objFile != nullptr;
        AP_LOG("done", "%s total %llu us (geometry %llu / pack %llu / "
               "materials %llu / textures %llu)",
               ok ? "OK" : "FAILED",
               static_cast<unsigned long long>(result->totalMicros),
               static_cast<unsigned long long>(result->importMicros),
               static_cast<unsigned long long>(result->verticesMicros),
               static_cast<unsigned long long>(result->materialsMicros),
               static_cast<unsigned long long>(result->texturesMicros));
        fireAllDone(*result, ok, ok ? std::string_view{} : std::string_view(lastError_));
        AP_LOG("event", "onAllDone fired");
    });

    mapTask.precede(geometryTask, materialsTask);
    geometryTask.precede(bindTask, doneTask);
    materialsTask.precede(bindTask, texturesTask, doneTask);
    texturesTask.precede(doneTask);
    bindTask.precede(doneTask);

    if (async) {
        executor_->run(tf);
        return true;
    }
    executor_->run(tf).wait();
    return result->objFile != nullptr;
}

} // namespace ap
