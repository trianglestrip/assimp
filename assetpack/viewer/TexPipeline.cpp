#include "TexPipeline.h"
#include <assetpack/AssetPack.h>

#include <algorithm>
#include <condition_variable>
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
    const size_t total = total_.load(std::memory_order_acquire);
    if (to > total) to = total;
    if (from >= total) return;
    size_t freed = 0;
    for (size_t i = from; i < to; ++i) {
        DecodedTex& t = texs_[i];
        if (!t.rgba) continue;
        freed += size_t(t.w) * size_t(t.h) * 4;
        t.rgba.reset(); // stbi_image_free
    }
    bytesReleased += freed;
    if (freed)
        AP_LOG("tex", "released %.1f MB decoded pixels (%zu/%zu handed to GPU)",
               double(freed) / 1048576.0, to, total);
}

TexPipeline::~TexPipeline() {
    // guarantee the worker can never block join() on an unset finish
    finished_.store(true, std::memory_order_release);
    qCv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void TexPipeline::enqueue(std::span<const ap::PackTexture> texs) {
    if (texs.empty()) return;
    std::vector<Ref> nd, no;
    for (const auto& t : texs) {
        if (t.embedded || t.resolvedPath.empty()) continue;
        const bool diff = (t.type == int(ap::TexType::TexDiffuse));
        Ref r{std::string(t.path), t.resolvedPath, diff};
        (diff ? nd : no).push_back(std::move(r));
    }
    if (nd.empty() && no.empty()) return;
    {
        std::lock_guard<std::mutex> lk(qMx_);
        const size_t old = texs_.size();
        for (auto& r : nd) diffuseQ_.push_back(std::move(r));
        for (auto& r : no) othersQ_.push_back(std::move(r));
        const size_t need = diffuseQ_.size();
        if (need > old) {
            texs_.resize(need);
            auto nsd = std::make_unique<std::atomic<uint8_t>[]>(need);
            for (size_t i = 0; i < old; ++i)
                nsd[i].store(slotDone_[i].load(std::memory_order_relaxed),
                              std::memory_order_relaxed);
            for (size_t i = old; i < need; ++i)
                nsd[i].store(0, std::memory_order_relaxed);
            slotDone_ = std::move(nsd);
        }
        total_.store(need, std::memory_order_release);
    }
    qCv_.notify_all();
    if (!running_.exchange(true))
        worker_ = std::thread([this]() { runDecodeLoop(); });
}

void TexPipeline::finish() {
    finished_.store(true, std::memory_order_release);
    qCv_.notify_all();
}

void TexPipeline::runDecodeLoop() {
    const unsigned n = std::max(1u, std::thread::hardware_concurrency());

    // caller must hold qMx_: publishes the longest contiguous decoded
    // prefix so consumers only ever see a prefix, never holes
    auto advancePrefix = [&]() {
        size_t cur = ready_.load(std::memory_order_acquire);
        while (cur < total_.load() &&
               slotDone_[cur].load(std::memory_order_acquire)) {
            const size_t want = cur + 1;
            if (ready_.compare_exchange_strong(cur, want,
                                                std::memory_order_acq_rel,
                                                std::memory_order_relaxed)) {
                if (diffuseQ_[cur].diffuse) {
                    std::lock_guard<std::mutex> plk(pathMx_);
                    byPath_.emplace(diffuseQ_[cur].key, int(cur));
                }
                cur = ready_.load(std::memory_order_acquire);
            } else {
                cur = ready_.load(std::memory_order_acquire);
            }
        }
    };

    auto decodeOne = [&](const Ref& t, size_t slot) {
        auto mf = ap::MappedFile::openShared(t.resolved);
        if (!mf) return;
        bytesMapped += mf->size();
        ++filesMapped;
        int w = 0, h = 0, ch = 0;
        // stb already outputs upload-ready RGBA bytes: keep them
        unsigned char* rgba = stbi_load_from_memory(
            reinterpret_cast<const unsigned char*>(mf->bytes().data()),
            int(mf->size()), &w, &h, &ch, 4);
        if (!rgba) {
            AP_LOG_WARN("tex", "decode failed: %s", t.key.c_str());
            return;
        }
        DecodedTex dt;
        dt.w = w;
        dt.h = h;
        dt.rgba = {rgba, &stbiFree};
        // publish under qMx_ so queue growth (which also holds qMx_)
        // can never race with this write
        {
            std::lock_guard<std::mutex> lk(qMx_);
            texs_[slot] = std::move(dt);
            slotDone_[slot].store(1, std::memory_order_release);
            advancePrefix();
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(n);
    for (unsigned tid = 0; tid < n; ++tid) {
        pool.emplace_back([&, tid]() {
            (void)tid; // strided assignment keeps completion even; here we
                       // claim the next free slot under the lock instead
            for (;;) {
                Ref r;
                bool isDiff = false, isOther = false;
                size_t j = 0, k = 0;
                {
                    std::lock_guard<std::mutex> lk(qMx_);
                    if (diffuseCur_ < diffuseQ_.size()) {
                        j = diffuseCur_++;
                        r = diffuseQ_[j];
                        isDiff = true;
                    } else if (othersCur_ < othersQ_.size()) {
                        k = othersCur_++;
                        r = othersQ_[k];
                        isOther = true;
                    }
                }
                if (isDiff) {
                    decodeOne(r, j);
                    continue;
                }
                if (isOther) {
                    // stat-only mmap for non-diffuse refs (normals etc.)
                    auto mf = ap::MappedFile::openShared(r.resolved);
                    if (mf) {
                        bytesMapped += mf->size();
                        ++filesMapped;
                    }
                    continue;
                }
                // nothing claimable right now
                if (finished_.load(std::memory_order_acquire)) {
                    std::lock_guard<std::mutex> lk(qMx_);
                    if (diffuseCur_ >= diffuseQ_.size() &&
                        othersCur_ >= othersQ_.size())
                        break;
                }
                std::unique_lock<std::mutex> lk(qMx_);
                qCv_.wait(lk, [&] {
                    return (diffuseCur_ < diffuseQ_.size()) ||
                           (othersCur_ < othersQ_.size()) ||
                           finished_.load(std::memory_order_acquire);
                });
            }
        });
    }
    for (auto& th : pool) th.join();
    done_.store(true, std::memory_order_release);
    running_.store(false, std::memory_order_release);
}

} // namespace texp
