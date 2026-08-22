#include "TexPipeline.h"
#include <assetpack/AssetPack.h>

#include <algorithm>
#include <thread>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

namespace texp {

namespace {
// deleter for stb's malloc'd decode output (kept out of the header)
void stbiFree(void* p) { stbi_image_free(p); }
} // namespace

int TexPipeline::slotFor(std::string_view mtlPath) const {
    std::lock_guard<std::mutex> lock(pathMx_);
    const auto it = byPath_.find(mtlPath);
    return it == byPath_.end() ? -1 : it->second;
}

void TexPipeline::releaseSlots(size_t from, size_t to) {
    if (to > total_) to = total_;
    if (from >= total_) return;
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
               double(freed) / 1048576.0, to, total_);
}

TexPipeline::~TexPipeline() {
    if (worker_.joinable()) worker_.join();
}

void TexPipeline::runDecode(
    const std::vector<const ap::PackTexture*>& diffuse,
    const std::vector<const ap::PackTexture*>& others) {
    const unsigned n = std::max(1u, std::thread::hardware_concurrency());

    // publish the longest fully-decoded prefix: consumers only ever see a
    // contiguous range of finished slots, never holes
    auto advancePrefix = [&]() {
        size_t cur = ready_.load(std::memory_order_acquire);
        while (cur < total_ && slotDone_[cur].load(std::memory_order_acquire)) {
            const size_t want = cur + 1;
            if (ready_.compare_exchange_strong(cur, want,
                                               std::memory_order_acq_rel,
                                               std::memory_order_relaxed)) {
                {
                    std::lock_guard<std::mutex> lock(pathMx_);
                    byPath_.emplace(paths_[cur], int(cur));
                }
                cur = ready_.load(std::memory_order_acquire);
            } else {
                cur = ready_.load(std::memory_order_acquire);
            }
        }
    };

    auto decodeOne = [&](const ap::PackTexture& t, size_t slot) {
        auto mf = ap::MappedFile::openShared(t.resolvedPath);
        if (!mf) return;
        bytesMapped += mf->size();
        ++filesMapped;
        int w = 0, h = 0, ch = 0;
        // stb already outputs upload-ready RGBA bytes: keep them
        unsigned char* rgba = stbi_load_from_memory(
            reinterpret_cast<const unsigned char*>(mf->bytes().data()),
            int(mf->size()), &w, &h, &ch, 4);
        if (!rgba) {
            AP_LOG_WARN("tex", "decode failed: %.*s", int(t.path.size()),
                        t.path.data());
            return;
        }
        DecodedTex dt;
        dt.w = w;
        dt.h = h;
        dt.rgba = {rgba, &stbiFree};
        texs_[slot] = std::move(dt);
        pixels += size_t(w) * size_t(h);
    };

    std::vector<std::thread> pool;
    pool.reserve(n);
    for (unsigned tid = 0; tid < n; ++tid) {
        pool.emplace_back([&, tid]() {
            // strided assignment keeps global completion roughly even so
            // the visible prefix advances smoothly
            for (size_t j = tid; j < diffuse.size(); j += n) {
                decodeOne(*diffuse[j], j);
                slotDone_[j].store(1, std::memory_order_release);
                advancePrefix();
            }
            // stat-only mmaps for the non-diffuse refs (normals etc.)
            for (size_t k = tid; k < others.size(); k += n) {
                auto mf =
                    ap::MappedFile::openShared(others[k]->resolvedPath);
                if (!mf) continue;
                bytesMapped += mf->size();
                ++filesMapped;
            }
        });
    }
    for (auto& th : pool) th.join();
    done_.store(true, std::memory_order_release);
}

void TexPipeline::decodeAll(std::span<const ap::PackTexture> texs) {
    if (worker_.joinable()) return;   // a decode is already in flight
    std::vector<const ap::PackTexture*> diffuse, others;
    for (const ap::PackTexture& t : texs) {
        if (t.embedded || t.resolvedPath.empty()) continue;
        (t.type == int(ap::TexType::TexDiffuse) ? diffuse : others)
            .push_back(&t);
    }
    total_ = diffuse.size();
    texs_.clear();
    texs_.resize(total_);
    paths_.clear();
    paths_.reserve(total_);
    for (const ap::PackTexture* t : diffuse)
        paths_.push_back(t->path);
    slotDone_.reset(new std::atomic<uint8_t>[total_ ? total_ : 1]);
    for (size_t i = 0; i < total_; ++i)
        slotDone_[i].store(0, std::memory_order_relaxed);
    ready_.store(0, std::memory_order_relaxed);
    done_.store(false, std::memory_order_relaxed);

    worker_ = std::thread([this, diffuse = std::move(diffuse),
                           others = std::move(others)]() mutable {
        runDecode(diffuse, others);
    });
}

} // namespace texp
