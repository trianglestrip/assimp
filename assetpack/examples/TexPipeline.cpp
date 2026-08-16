#include "TexPipeline.h"
#include <assetpack/AssetPack.h>

#include <atomic>
#include <thread>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

namespace texp {

namespace {
// deleter for stb's malloc'd decode output (kept out of the header)
void stbiFree(void* p) { stbi_image_free(p); }
} // namespace

int TexPipeline::slotFor(std::string_view mtlPath) const {
    const auto it = byPath_.find(mtlPath);
    return it == byPath_.end() ? -1 : it->second;
}

void TexPipeline::releaseSlots(size_t from, size_t to) {
    if (to > texs_.size()) to = texs_.size();
    size_t freed = 0;
    for (size_t i = from; i < to; ++i) {
        DecodedTex& t = texs_[i];
        if (!t.rgba) continue;
        freed += size_t(t.w) * size_t(t.h) * 4;
        t.rgba.reset();   // stbi_image_free
    }
    bytesReleased += freed;
    if (freed)
        AP_LOG("tex", "released %.1f MB decoded pixels (%zu/%zu handed to GPU)",
               double(freed) / 1048576.0, to, texs_.size());
}

void TexPipeline::decodeAll(std::span<const ap::PackTexture> texs) {
    // count the diffuse refs first so slots are dense
    int ndiff = 0;
    for (const ap::PackTexture& t : texs)
        if (!t.embedded && !t.resolvedPath.empty() &&
            t.type == int(ap::TexType::TexDiffuse))
            ++ndiff;

    texs_.clear();
    texs_.resize(size_t(ndiff));
    byPath_.clear();
    bytesMapped = 0;
    filesMapped = 0;
    pixels = 0;

    std::vector<int> slotByPos(texs.size(), -1);
    std::atomic<size_t> next{0};
    std::atomic<int> slot{0};
    std::atomic<uint64_t> bytes{0};
    std::atomic<int> files{0};
    std::atomic<uint64_t> px{0};

    const unsigned n = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> pool;
    pool.reserve(n);
    for (unsigned i = 0; i < n; ++i) {
        pool.emplace_back([&]() {
            for (;;) {
                const size_t k = next.fetch_add(1);
                if (k >= texs.size()) break;
                const ap::PackTexture& t = texs[k];
                if (t.embedded || t.resolvedPath.empty()) continue;
                auto mf = ap::MappedFile::openShared(t.resolvedPath);
                if (!mf) continue;
                bytes += mf->size();
                ++files;
                if (t.type != int(ap::TexType::TexDiffuse)) continue;
                int w = 0, h = 0, ch = 0;
                // stb already outputs upload-ready RGBA bytes: keep them
                unsigned char* rgba = stbi_load_from_memory(
                    reinterpret_cast<const unsigned char*>(mf->bytes().data()),
                    int(mf->size()), &w, &h, &ch, 4);
                if (!rgba) {
                    AP_LOG_WARN("tex", "decode failed: %.*s", int(t.path.size()),
                                t.path.data());
                    continue;
                }
                DecodedTex dt;
                dt.w = w;
                dt.h = h;
                // take stb's buffer as-is: no full-image copy into an
                // owning vector (saves the 366MB round trip on load);
                // releaseSlots() frees it after the GPU upload
                dt.rgba = {rgba, &stbiFree};
                const int si = slot.fetch_add(1);
                slotByPos[k] = si;
                texs_[size_t(si)] = std::move(dt);
                px += size_t(w) * size_t(h);
            }
        });
    }
    for (auto& th : pool) th.join();

    for (size_t k = 0; k < texs.size(); ++k)
        if (slotByPos[k] >= 0) byPath_.emplace(texs[k].path, slotByPos[k]);
    bytesMapped = bytes.load();
    filesMapped = files.load();
    pixels = px.load();
}

} // namespace texp
