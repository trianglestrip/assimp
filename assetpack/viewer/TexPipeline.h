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
#include <thread>
#include <unordered_map>
#include <vector>

namespace texp {

// stb's own malloc'd buffer; the deleter lives in TexPipeline.cpp so this
// header stays free of stb includes
using TexBytes = std::unique_ptr<unsigned char[], void (*)(void*)>;

struct DecodedTex {
    int w = 0, h = 0;
    // RGBA bytes, row 0 = image top. Holds stb's buffer directly (no
    // copy); released by releaseSlots() once the GPU owns its copy.
    TexBytes rgba{nullptr, [](void*) {}};
};

class TexPipeline {
public:
    // Start decoding every external reference in parallel on a
    // background pool; returns immediately. Slots become visible
    // monotonically through readyCount().
    void decodeAll(std::span<const ap::PackTexture> texs);

    // Free the decoded pixels of slots [from, to) after the renderer
    // uploaded them to the GPU; w/h stay valid for the slot count.
    void releaseSlots(size_t from, size_t to);

    // total allocated slots (>= readyCount()); only the ready prefix is
    // safe to consume
    size_t count() const { return total_; }
    // number of leading slots fully decoded and immutable
    size_t readyCount() const { return ready_.load(std::memory_order_acquire); }
    bool done() const { return done_.load(std::memory_order_acquire); }

    const DecodedTex* data() const { return texs_.data(); }
    const DecodedTex& at(size_t i) const { return texs_[i]; }
    int slotFor(std::string_view mtlPath) const;   // -1 when absent

    std::atomic<uint64_t> bytesMapped{0};
    std::atomic<int> filesMapped{0};
    std::atomic<uint64_t> pixels{0};
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
    };
    void runDecode(const std::vector<Ref>& diffuse,
                   const std::vector<Ref>& others);

    std::vector<DecodedTex> texs_;     // fixed after decodeAll starts
    size_t total_ = 0;                 // == texs_.size() once started
    std::unique_ptr<std::atomic<uint8_t>[]> slotDone_;
    std::atomic<size_t> ready_{0};     // longest decoded prefix
    std::atomic<bool> done_{false};
    std::thread worker_;
    mutable std::mutex pathMx_;
    std::unordered_map<std::string_view, int> byPath_;
};

} // namespace texp
