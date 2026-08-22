#include "Geometry.h"
#include <assetpack/AssetPack.h>

#include <taskflow/algorithm/for_each.hpp>  // tf::FlowBuilder::for_each_index

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

// ---- texture-seam splitting table. Only this translation unit uses
// it, so the whole machinery lives here instead of a shared header
// (Geometry.h forward-declares SeamShard for its storage member). ----
namespace ap::detail {

struct SeamVert {
    uint32_t v, vt, vn;   // pool indices; 0xFFFFFFFF = part absent
};

// Key = (v, vt, vn) split across two lanes (68 bits won't fit one u64)
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
bool claimIdx(uint32_t& slot, uint32_t val) {
    std::atomic_ref<uint32_t> a(slot);
    uint32_t cur = a.load(std::memory_order_relaxed);
    for (;;) {
        if (cur == val) return true;
        if (cur != 0xFFFFFFFFu) return false;
        if (a.compare_exchange_weak(cur, val, std::memory_order_relaxed))
            return true;
    }
}

} // namespace ap::detail

namespace ap::obj {

namespace {

// intermediate group bookkeeping (not part of the public result)
struct RawGroup {
    std::string_view name;          // 'o'/'g' name (view into obj mapping)
    std::string_view material;      // usemtl name (view into obj mapping)
    uint32_t firstTriangle = 0;
    uint32_t triangleCount = 0;
};

// Split [0, size) into `n` byte ranges, each boundary snapped forward
// to the next '\n' so a range never starts or ends mid-line (a partial
// line at a boundary belongs to the earlier chunk). This is the unit
// of work for chunk-parallel line scans over an mmap.
std::vector<std::pair<size_t, size_t>> splitLineRanges(
    const char* data, size_t size, size_t n) {
    std::vector<std::pair<size_t, size_t>> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        size_t b = size * i / n;
        size_t e = size * (i + 1) / n;
        if (b > size) b = size;
        // snap begin forward to the next line start
        while (b > 0 && b < size && data[b - 1] != '\n') ++b;
        // snap end forward to include the partial line
        while (e < size && e > 0 && data[e - 1] != '\n') ++e;
        if (e < b) e = b;
        out.emplace_back(b, e);
    }
    return out;
}

} // namespace

GeometryStage::GeometryStage(
    ObjParser& owner, std::shared_ptr<PackResult> result,
    std::shared_ptr<std::vector<std::string_view>> mtlNames, bool wantNormals,
    bool wantTexcoords)
    : owner_(owner), result_(std::move(result)), mtlNames_(std::move(mtlNames)),
      wantN_(wantNormals), wantUv_(wantTexcoords),
      chunks_(std::make_shared<std::vector<ChunkInfo>>()),
      vnPool_(std::make_shared<std::vector<float>>()),
      uvPool_(std::make_shared<std::vector<float>>()),
      vnIdx_(std::make_shared<std::vector<uint32_t>>()),
      uvIdx_(std::make_shared<std::vector<uint32_t>>()),
      shards_(std::make_shared<std::array<detail::SeamShard, 16>>()),
      seamCounter_(std::make_shared<std::atomic<uint32_t>>(0)),
      baseVerts_(std::make_shared<size_t>(0)),
      texMats_(std::make_shared<std::unordered_set<std::string_view>>()) {}

void GeometryStage::run(tf::Subflow& sf) {
    if (!result_->objFile || owner_.isCancelled()) return;
    const auto tStart = Clock::now();
    AP_LOG("geometry", "begin");
    const std::string_view text = result_->objFile->text();
    const char* data = text.data();
    const size_t size = text.size();

    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const int nChunks = int(std::min<size_t>(size / (1 << 16) + 1, size_t(hw) * 8));
    {
        // line-aligned byte ranges, one ChunkInfo each
        const auto ranges = splitLineRanges(data, size, size_t(nChunks));
        chunks_->clear();
        chunks_->resize(ranges.size());
        for (size_t i = 0; i < ranges.size(); ++i) {
            (*chunks_)[i].begin = ranges[i].first;
            (*chunks_)[i].end = ranges[i].second;
        }
    }

    const auto result = result_;
    const auto chunks = chunks_;
    const auto vnPool = vnPool_;
    const auto uvPool = uvPool_;
    const auto vnIdx = vnIdx_;
    const auto uvIdx = uvIdx_;
    const auto shards = shards_;
    const auto seamCounter = seamCounter_;
    const auto baseVerts = baseVerts_;
    const auto mtlNames = mtlNames_;
    const auto texMats = texMats_;
    const bool wantN = wantN_;

    // Sequential prefetch thread: a multi-GB mapping can never stay
    // fully resident on a 16 GB box, so each scan pass would refault
    // pages evicted since the last one. Walking the file front to back
    // with PrefetchVirtualMemory issues the disk reads up front (as
    // one ordered stream) while the passes fault into the standby
    // list instead of the disk. Joined after the graph drains; the
    // thread is harmless (and near-instant) when pages are hot.
    std::atomic<bool> pfStop{false};
    std::thread pf([&]() {
        const auto f = result->objFile;
        if (!f) return;
        const size_t sz = f->size();
        constexpr size_t kStep = 8u << 20;
        size_t off = 0;
        while (!pfStop.load(std::memory_order_relaxed) && off < sz) {
            f->prefetch(off, kStep);
            off += kStep;
        }
    });

    // pass 1 (parallel): per-chunk counts + ordered markers
    tf::Task pass1 = sf.for_each_index(0, nChunks, 1, [chunks, data, this](int ci) {
        ChunkInfo& c = (*chunks)[size_t(ci)];
         const char* p = data + c.begin;
         const char* end = data + c.end;
         if (owner_.isCancelled()) return;
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
    tf::Task prefix = sf.emplace([chunks, result, vnPool, uvPool, vnIdx, uvIdx,
                                  nChunks, wantN, baseVerts, shards, tStart,
                                  this]() {
        if (owner_.isCancelled()) return;
        AP_LOG("geometry", "pass1 done at %llu us",
               static_cast<unsigned long long>(microsSince(tStart)));
        size_t p = 0, n = 0, u = 0, t = 0;
        for (int i = 0; i < nChunks; ++i) {
            ChunkInfo& c = (*chunks)[size_t(i)];
            c.basePos = p; c.baseNrm = n; c.baseUv = u; c.baseTri = t;
            p += c.nPos; n += c.nNrm; u += c.nUv; t += c.nTri;
        }
        if (!wantUv_) u = 0;   // texcoords opt-out: drop the whole uv pool
        // geometry streaming: publish the buffer plan as soon as the
        // counts are known (the split count is still unknown, so files
        // with vt lines reserve one extra vertex per base vertex -
        // covers every real model, and seamFill skips splits that would
        // overflow). Firing early lets the consumer create its buffers
        // (a few hundred ms of D3D12 resource allocation) overlap the
        // pool allocation below instead of delaying pass 2's publish.
        const GeoStreamSink& sink = owner_.geoStream();
        if (sink.onMeta) {
            const size_t cap = u ? p * 2 : p;   // split capacity
            sink.onMeta(cap, t, u > 0);
        }
        // thread the running usemtl across chunk boundaries so pass2 can
        // skip seam-splitting for untextured materials; markers are
        // position-ordered, so a marker at chunk start covers no faces
        // before it and the pre-marker material is the right default
        std::string_view curMtl;
        for (int i = 0; i < nChunks; ++i) {
            ChunkInfo& c = (*chunks)[size_t(i)];
            c.startMtl = curMtl;
            for (const Marker& m : c.markers)
                if (m.kind == LineKind::UseMtl)
                    curMtl = std::string_view(m.name, m.nameLen);
        }
        result->positions.resize(p * 3);
        result->posIndices.resize(t * 3);
        if (wantN) vnPool->resize(n * 3);
        uvPool->resize(u * 2);
        // per-vertex vn/vt reference slots; 0xFFFFFFFF marks "no
        // reference yet" so unreferenced vertices keep zero normals.
        // vnIdx stays empty when normals are not wanted; uvIdx stays
        // empty when the file has no vt at all - pass 2's claim and
        // pass 3's expansion both already guard on empty()
        if (wantN) vnIdx->assign(p, 0xFFFFFFFFu);
        if (u) uvIdx->assign(p, 0xFFFFFFFFu);
        // expansion targets (per-vertex indexed, filled by pass 3)
        if (wantN && n) result->normals.assign(p * 3, 0.f);
        if (u) result->texcoords.assign(p * 2, 0.f);
        *baseVerts = p;   // seam splits append past this in seamFill
        // seed the seam shards (they grow themselves on load): San
        // Miguel splits half its positions, so start generous
        for (detail::SeamShard& sh : *shards) {
            sh.seed(1u << 17);
            sh.added.reserve(1u << 17);
        }
        AP_LOG("geometry", "pass1: %zu v / %zu vn / %zu vt / %zu tris",
               p, n, u, t);
    });

    // which materials carry a diffuse map. The seam split exists to keep
    // UV seams watertight for textured rendering; meshes whose material
    // has no map_Kd sample nothing (the viewer falls back to a white
    // texel), so splitting them just duplicates vertices for nothing.
    // The scan mirrors Mtl.cpp's exact-case keyword matching: a material
    // the mtl parser would not bind a texture to must not split either.
    // Runs ahead of pass2 while the materials task (a sibling stage)
    // parses the same file in parallel; this only reads it.
    tf::Task mtlTex = sf.emplace([result, texMats, this]() {
        if (owner_.isCancelled()) return;
        const std::shared_ptr<MappedFile>& mf = result->mtlFile;
        if (!mf) return;
        const std::string_view text = mf->text();
        const char* p = text.data();
        const char* end = p + text.size();
        std::string_view cur;
        while (p < end) {
            const char* eol = static_cast<const char*>(
                memchr(p, '\n', size_t(end - p)));
            if (!eol) eol = end;
            if (eol - p > 7 && memcmp(p, "newmtl ", 7) == 0) {
                const char* tok; size_t len;
                if (nextToken(p + 7, eol, tok, len))
                    cur = std::string_view(tok, len);
            } else if (!cur.empty() && eol - p > 7
                       && memcmp(p, "map_Kd ", 7) == 0) {
                texMats->insert(cur);
            }
            p = eol + 1;
        }
        AP_LOG("geometry", "mtl diffuse maps: %zu materials",
               texMats->size());
    });

    // pass 2 (parallel): fill arrays; fan-triangulate faces and
    // resolve 1-based / negative-relative indices per chunk
    tf::Task pass2 = sf.for_each_index(0, nChunks, 1,
        [chunks, result, vnPool, uvPool, vnIdx, uvIdx, data, wantN,
         shards, seamCounter, baseVerts, texMats, this](int ci) {
         ChunkInfo& c = (*chunks)[size_t(ci)];
         const char* p = data + c.begin;
         const char* end = data + c.end;
         if (owner_.isCancelled()) return;
         size_t lp = c.basePos, ln = c.baseNrm, lu = c.baseUv;
            float* pos = result->positions.data();
            float* nrm = vnPool->data();
            float* uv  = uvPool->data();
            uint32_t* pi = result->posIndices.data() + c.baseTri * 3;
            // the usemtl in effect here decides whether faces split on
            // texture seams; untextured materials never split (see mtlTex)
            std::string_view curMtl = c.startMtl;
            bool split = !curMtl.empty()
                         && texMats->find(curMtl) != texMats->end();

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
                    // resolution depend on vn counts regardless.
                    if (wantN) {
                        q = p + 2;
                        for (int comp = 0; comp < 3; ++comp) {
                            if (!(q = nextToken(q, eol, tok, len))) break;
                            nrm[ln * 3 + comp] = parseFloat(tok, len);
                        }
                    }
                    ++ln;
                } else if (k == LineKind::VT) {
                    if (wantUv_) {
                        q = p + 2;
                        for (int comp = 0; comp < 2; ++comp) {
                            if (!(q = nextToken(q, eol, tok, len))) break;
                            uv[lu * 2 + comp] = parseFloat(tok, len);
                        }
                    }
                    ++lu;
                } else if (k == LineKind::UseMtl) {
                    const char* tok; size_t len;
                    if (nextToken(p + 7, eol, tok, len)) {
                        curMtl = std::string_view(tok, len);
                        split = !curMtl.empty()
                                && texMats->find(curMtl) != texMats->end();
                    }
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
                            // seam and splits into its own vertex.
                            // Untextured materials skip the claim entirely:
                            // their UVs are never sampled, so a position
                            // may alias any number of (vt, vn) combos
                            bool uniq = true;
                            if (split) {
                                if (vtU != 0xFFFFFFFFu && !uvIdx->empty()
                                    && !detail::claimIdx((*uvIdx)[ip], vtU))
                                    uniq = false;
                                if (uniq && vnU != 0xFFFFFFFFu
                                    && !vnIdx->empty()
                                    && !detail::claimIdx((*vnIdx)[ip], vnU))
                                    uniq = false;
                            }
                            uint32_t out = ip;
                            if (!uniq) {
                                const detail::SeamVert sv{ip, vtU, vnU};
                                const uint64_t k0 =
                                    uint64_t(vtU)
                                    | (uint64_t(vnU) << 32);
                                detail::SeamShard& sh = (*shards)[size_t(
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
            // this chunk's pos/idx ranges are final: publish them so
            // the consumer copies while later chunks still parse
            const GeoStreamSink& sink = owner_.geoStream();
            if (sink.onRange) {
                if (c.nPos)
                    sink.onRange(GeoRangeKind::Pos, c.basePos * 12,
                                 result->positions.data() + c.basePos * 3,
                                 c.nPos * 12);
                if (c.nTri)
                    sink.onRange(GeoRangeKind::Idx, c.baseTri * 12,
                                 result->posIndices.data() + c.baseTri * 3,
                                 c.nTri * 12);
            }
        });

    // seam resolve: grow the pools by the split vertices and fill
    // their data (position copy from the base slot + their own
    // uv/normal straight from the vn/vt pools). Runs once, after all
    // pass2 chunks finish, so every split index is final here.
    tf::Task seamFill = sf.emplace([result, vnPool, uvPool, shards,
                                    seamCounter, baseVerts,
                                    this](tf::Subflow& inner) {
        if (owner_.isCancelled()) return;
        const size_t base = *baseVerts;
        const size_t n = seamCounter->load();
        if (n) {
            result->positions.resize((base + n) * 3);
            if (!result->normals.empty())
                result->normals.resize((base + n) * 3);
            if (!result->texcoords.empty())
                result->texcoords.resize((base + n) * 2);
        }
        if (!n) return;
        // flatten the shard lists (cheap pointer gather), then fill
        // in parallel: entries write disjoint output slots
        std::vector<const std::pair<uint32_t, detail::SeamVert>*> entries;
        size_t total = 0;
        for (const detail::SeamShard& sh : *shards) total += sh.added.size();
        entries.reserve(total);
        for (const detail::SeamShard& sh : *shards)
            for (const auto& e : sh.added) entries.push_back(&e);
        float* pos = result->positions.data();
        float* nrm = result->normals.data();
        float* uv = result->texcoords.data();
        const float* np = vnPool->data();
        const float* up = uvPool->data();
        const bool wantNrm = !result->normals.empty();
        const bool wantUv = !result->texcoords.empty();
         inner.for_each_index(0, int(total), 1, [&](int i) {
             if (owner_.isCancelled()) return;
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
        // geometry streaming: the meta (buffer plan) went out from the
        // prefix task; pass2 chunks already published their pos/idx
        // ranges, so only the seam-split range is new here. The prefix
        // plan reserved one extra vertex per base vertex when the file
        // has vt lines; a split count beyond that would overflow the
        // GPU buffer, so cap the publish at the reserved capacity.
        // Published here, after the fill: earlier would stream the
        // zero-initialized tail from the resizes above.
        const GeoStreamSink& sink = owner_.geoStream();
        if (sink.onRange && !result->texcoords.empty()) {
            const size_t cap = uvPool->empty() ? base : base * 2;
            if (base + n <= cap) {
                sink.onRange(GeoRangeKind::Pos, base * 12,
                             result->positions.data() + base * 3,
                             n * 12);
                sink.onRange(GeoRangeKind::Uv, base * 8,
                             result->texcoords.data() + base * 2, n * 8);
            } else {
                AP_LOG("geometry",
                       "seam splits exceed streamed capacity (%zu>%zu); "
                       "split verts not uploaded",
                       n, cap - base);
            }
        }
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
        [result, vnIdx, uvIdx, vnPool, uvPool, chunks, nChunks, tStart,
         baseVerts, this](tf::Subflow& inner) {
            if (owner_.isCancelled()) return;
            // base slots only: seam splits were filled directly by
            // seamFill, and vnIdx/uvIdx stop at the base count
            const int nVerts = int(*baseVerts);
            if (nVerts <= 0) return;
            const auto t3 = Clock::now();
            // per-chunk blocks: pass2's claims are final, so slot i can
            // expand in any order - walking chunk by chunk lets each
            // completed block publish its texcoord range while the
            // remaining chunks still parse (streaming upload)
            inner.for_each_index(0, nChunks, 1,
                [chunks, result, vnIdx, uvIdx, vnPool, uvPool,
                 this](int ci) {
                     if (owner_.isCancelled()) return;
                     const ChunkInfo& c = (*chunks)[size_t(ci)];
                     const size_t begin = c.basePos;
                    const size_t end = c.basePos + c.nPos;
                    const float* vn = vnPool->data();
                    const float* uv = uvPool->data();
                    const bool wantUv = !result->texcoords.empty();
                    for (size_t iv = begin; iv < end; ++iv) {
                        if (!result->normals.empty()) {
                            const uint32_t in =
                                std::atomic_ref<const uint32_t>(
                                    (*vnIdx)[iv]).load(
                                        std::memory_order_relaxed);
                            if (in != 0xFFFFFFFFu) {
                                const size_t in3 = size_t(in) * 3;
                                float* dst =
                                    result->normals.data() + iv * 3;
                                dst[0] = vn[in3];
                                dst[1] = vn[in3 + 1];
                                dst[2] = vn[in3 + 2];
                            }
                        }
                        if (wantUv) {
                            const uint32_t iu =
                                std::atomic_ref<const uint32_t>(
                                    (*uvIdx)[iv]).load(
                                        std::memory_order_relaxed);
                            if (iu != 0xFFFFFFFFu) {
                                const size_t iu2 = size_t(iu) * 2;
                                float* dst =
                                    result->texcoords.data() + iv * 2;
                                dst[0] = uv[iu2];
                                dst[1] = uv[iu2 + 1];
                            }
                        }
                    }
                    if (wantUv && owner_.geoStream().onRange)
                        owner_.geoStream().onRange(
                            GeoRangeKind::Uv, begin * 2 * sizeof(float),
                            result->texcoords.data() + begin * 2,
                            (end - begin) * 2 * sizeof(float));
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
    tf::Task build = sf.emplace([this, chunks, result, nChunks, mtlNames,
                                 tStart](tf::Subflow& inner) {
        if (owner_.isCancelled()) { owner_.fireProgress(100.f); return; }
        std::vector<RawGroup> groups;
        RawGroup cur;
        cur.firstTriangle = 0;
        auto closeGroup = [&](uint32_t endTri) {
            if (endTri > cur.firstTriangle) {
                cur.triangleCount = endTri - cur.firstTriangle;
                groups.push_back(cur);
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
        result->meshes.reserve(groups.size());
        std::span<const float> allPos(result->positions);
        std::span<const float> allNrm(result->normals);
        std::span<const float> allUv(result->texcoords);
        for (const RawGroup& g : groups) {
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
         inner.for_each_index(0, n, 1, [this, result](int i) {
             if (owner_.isCancelled()) return;
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
               groups.size());

        owner_.fireProgress(100.f);
        owner_.fireVertices(*result, result->meshes);
        AP_LOG("event", "onVerticesReady fired (%zu meshes)",
               result->meshes.size());
    });

    pass1.precede(prefix);
    prefix.precede(pass2);
    mtlTex.precede(pass2);
    pass2.precede(seamFill, stamp2);   // stamp2 is a side branch: it
                                       // logs without blocking pass3
    seamFill.precede(pass3);
    pass3.precede(build);
    sf.join();

    pfStop = true;
    pf.join();
}

} // namespace ap::obj
