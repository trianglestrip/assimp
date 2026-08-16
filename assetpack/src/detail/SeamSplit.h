#pragma once
// ============================================================
// SeamSplit - texture-seam vertex splitting machinery (internal).
//
// An OBJ "vertex" is really the (v, vt, vn) triplet. One position can
// be referenced with different vt/vn combos (texture seams, mirrored
// uvs, atlases); collapsing those onto the position with an arbitrary
// winner maps the wrong uv region onto some faces. The parse claims
// each per-position slot with the FIRST combo and splits every
// conflicting combo into its own output vertex via a sharded
// open-addressing table (allocation-free probes, cache-linear).
// ============================================================

#include <atomic>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

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

} // namespace ap::detail
