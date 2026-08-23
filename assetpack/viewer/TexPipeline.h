#pragma once
// ============================================================
// TexPipeline - parallel texture decode, shared by both render
// paths (DX12 + software rasterizer).
//
// decodeAll() launches a background pool (mmap + stb_image) and
// returns immediately: the parser's onTexturesReady thread never
// blocks, so rendering starts while textures are still decoding.
// Slots become visible monotonically through readyCount(); draws
// must only consume that prefix. Decoded images keep stb's RGBA
// byte layout: the DX12 path uploads them verbatim, the software
// rasterizer samples the same bytes.
// ============================================================

#include <assetpack/AssetPack.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <taskflow/taskflow.hpp>

namespace texp {

// stb's own malloc'd buffer; the deleter lives in TexPipeline.cpp so this
// header stays free of stb includes
using TexBytes = std::unique_ptr<unsigned char[], void (*)(void*)>;

struct DecodedTex {
    int w = 0, h = 0;
    // RGBA bytes, row 0 = image top. Holds stb's buffer directly (no
    // copy); released by releaseSlots() once the GPU owns its copy.
    TexBytes rgba{nullptr, [](void*) {}};
    // mip chain beyond base (mip 1..N-1) when cacheImage DDS was loaded
    // or generated on save; empty means renderer must GenerateMips.
    std::vector<std::vector<uint8_t>> mips;
};

class TexPipeline {
public:
    static void setGenMips(bool v);
    static bool genMipsEnabled();
    // Enqueue external texture references for parallel decode on a
    // background pool. May be called repeatedly as the parser discovers
    // references (incremental), so decode overlaps the parse tail;
    // returns immediately. Slots become visible monotonically through
    // readyCount(). Call finish() once no more references will arrive.
    void enqueue(std::span<const ap::PackTexture> texs);

    // Inform the pipeline that no further enqueues are coming; lets the
    // background worker exit once its queue drains.
    void finish();

    // Free the decoded pixels of slots [from, to) after the renderer
    // uploaded them to the GPU; w/h stay valid for the slot count.
    void releaseSlots(size_t from, size_t to);

    // total allocated slots (>= readyCount()); only the ready prefix is
    // safe to consume
    size_t count() const { return total_.load(std::memory_order_acquire); }
    // number of leading slots fully decoded and immutable
    size_t readyCount() const { return ready_.load(std::memory_order_acquire); }
    bool done() const { return done_.load(std::memory_order_acquire); }

    const DecodedTex* data() const { return texs_.data(); }
    const DecodedTex& at(size_t i) const { return texs_[i]; }
    int slotFor(std::string_view mtlPath) const;   // -1 when absent

    std::atomic<uint64_t> bytesMapped{0};
    std::atomic<int> filesMapped{0};
    std::atomic<uint64_t> pixels{0};
    std::atomic<uint64_t> wicMicros{0};
    std::atomic<uint64_t> stbMicros{0};
    std::atomic<size_t> wicCount{0};
    std::atomic<size_t> stbCount{0};
    std::atomic<uint64_t> ddsMicros{0};
    std::atomic<size_t> ddsCount{0};
    uint64_t bytesReleased = 0;

    TexPipeline() = default;
    ~TexPipeline();
    TexPipeline(const TexPipeline&) = delete;
    TexPipeline& operator=(const TexPipeline&) = delete;

private:
    // owned copies: the background worker must not touch parser-owned
    // memory, which can be freed while a decode is still in flight
    struct Ref {
        std::string key;      // material path used for slot lookup
        std::string resolved; // absolute file path to mmap
        bool        diffuse = true;
    };
    void decodeOne(const Ref& t, size_t slot);
    void advancePrefix();

    std::vector<Ref> diffuseQ_;   // decoded + uploaded
    std::vector<Ref> othersQ_;    // normals etc., stat-only (stat-mmap only)
    std::vector<DecodedTex> texs_;   // sized to diffuseQ_.size()
    std::atomic<size_t> total_{0};   // == diffuseQ_.size()
    std::unique_ptr<std::atomic<uint8_t>[]> slotDone_;
    std::atomic<size_t> ready_{0};   // longest decoded prefix
    std::atomic<bool> done_{false};
    std::mutex qMx_;               // guards queues + texs_/slotDone_ growth + futures
    std::vector<tf::Future<void>> futures_;
    mutable std::mutex pathMx_;    // guards byPath_
    std::unordered_map<std::string_view, int> byPath_;
};

} // namespace texp
