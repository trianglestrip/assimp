#pragma once
// ============================================================
// TexPipeline - parallel texture decode, shared by both render
// paths (DX12 + software rasterizer).
//
// onTexturesReady fires on a parser thread; decodeAll() runs the
// whole reference list through a thread pool (mmap + stb_image)
// and blocks until done. Decoded images keep stb's RGBA byte
// layout: the DX12 path uploads them verbatim (no swizzle round
// trip), the software rasterizer samples the same bytes.
// ============================================================

#include <assetpack/ModelParser.h>

#include <cstdint>
#include <memory>
#include <string_view>
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
    // Decode every external diffuse reference in parallel. Blocking;
    // call from the parser's onTexturesReady thread.
    void decodeAll(std::span<const ap::PackTexture> texs);

    // Free the decoded pixels of slots [from, to) after the renderer
    // uploaded them to the GPU; w/h stay valid for the slot count.
    void releaseSlots(size_t from, size_t to);

    size_t count() const { return texs_.size(); }
    const DecodedTex* data() const { return texs_.data(); }
    const DecodedTex& at(size_t i) const { return texs_[i]; }
    int slotFor(std::string_view mtlPath) const;   // -1 when absent

    uint64_t bytesMapped = 0;    // sum of mapped texture file sizes
    int filesMapped = 0;
    uint64_t pixels = 0;         // decoded pixel count
    uint64_t bytesReleased = 0;  // pixels handed to the GPU and freed

private:
    std::vector<DecodedTex> texs_;
    std::unordered_map<std::string_view, int> byPath_;
};

} // namespace texp
