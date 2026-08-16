#pragma once
// ============================================================
// Scan - OBJ text-layer scanning (internal): line classification,
// per-line face-corner walking with index resolution, and the
// per-chunk bookkeeping the chunk-parallel passes fill in.
// ============================================================

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "detail/TextScan.h"   // nextToken / parseInt

namespace ap::obj {

enum class LineKind { Other, V, VN, VT, F, UseMtl, Object, Group, Mtllib };

LineKind classify(const char* p, const char* end);
uint32_t countFaceVerts(const char* p, const char* end);

// Resolve a 1-based or negative-relative OBJ index against its pool.
// Negative indices count back from the last definition so far, which
// is base (chunk prefix) + running (definitions seen within the chunk).
inline int32_t resolveIndex(int64_t raw, size_t base, size_t running) {
    if (raw > 0) return int32_t(raw - 1);
    return int32_t(int64_t(base + running) + raw);
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
    // usemtl in effect at this chunk's first face (filled by the prefix
    // task; empty when no usemtl precedes the chunk). A chunk boundary
    // can fall between a usemtl and the faces it covers, so the running
    // material is threaded across chunks, not resolved per chunk.
    std::string_view startMtl;
};

} // namespace ap::obj
