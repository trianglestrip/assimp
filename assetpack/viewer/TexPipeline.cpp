#include "TexPipeline.h"
#include <assetpack/AssetPack.h>
#include "../src/core/TaskExecutor.h"
#include <taskflow/algorithm/for_each.hpp>

#include <algorithm>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <wincodec.h>
#include <shlwapi.h>
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

namespace texp {

namespace {
// deleter for stb's malloc'd decode output (kept out of the header)
void stbiFree(void* p) { stbi_image_free(p); }

#ifdef _WIN32
bool tryDecodeWIC(const uint8_t* data, size_t size, int* w, int* h,
                  unsigned char** out) {
    thread_local IWICImagingFactory* factory = nullptr;
    if (!factory) {
        HRESULT hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr,
                                      CLSCTX_INPROC_SERVER,
                                      IID_PPV_ARGS(&factory));
        if (FAILED(hr) || !factory) return false;
    }
    IStream* stream = SHCreateMemStream(data, UINT(size));
    if (!stream) return false;
    IWICBitmapDecoder* decoder = nullptr;
    HRESULT hr = factory->CreateDecoderFromStream(
        stream, nullptr, WICDecodeMetadataCacheOnDemand, &decoder);
    if (FAILED(hr)) {
        stream->Release();
        return false;
    }
    IWICBitmapFrameDecode* frame = nullptr;
    hr = decoder->GetFrame(0, &frame);
    if (FAILED(hr)) {
        decoder->Release();
        stream->Release();
        return false;
    }
    IWICFormatConverter* conv = nullptr;
    hr = factory->CreateFormatConverter(&conv);
    if (FAILED(hr)) {
        frame->Release();
        decoder->Release();
        stream->Release();
        return false;
    }
    hr = conv->Initialize(frame, GUID_WICPixelFormat32bppRGBA,
                          WICBitmapDitherTypeNone, nullptr, 0.f,
                          WICBitmapPaletteTypeCustom);
    if (FAILED(hr)) {
        conv->Release();
        frame->Release();
        decoder->Release();
        stream->Release();
        return false;
    }
    UINT width = 0, height = 0;
    hr = conv->GetSize(&width, &height);
    if (FAILED(hr)) {
        conv->Release();
        frame->Release();
        decoder->Release();
        stream->Release();
        return false;
    }
    *w = int(width);
    *h = int(height);
    const size_t stride = size_t(width) * 4;
    const size_t imageSize = stride * size_t(height);
    unsigned char* buffer = (unsigned char*)malloc(imageSize);
    if (!buffer) {
        conv->Release();
        frame->Release();
        decoder->Release();
        stream->Release();
        return false;
    }
    hr = conv->CopyPixels(nullptr, UINT(stride), UINT(imageSize), buffer);
    conv->Release();
    frame->Release();
    decoder->Release();
    stream->Release();
    if (FAILED(hr)) {
        free(buffer);
        return false;
    }
    *out = buffer;
    return true;
}
#endif
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
    finish();
}

void TexPipeline::advancePrefix() {
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
}

void TexPipeline::decodeOne(const Ref& t, size_t slot) {
#ifdef _WIN32
    thread_local bool coInited = [] {
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        return true;
    }();
    (void)coInited;
#endif
    auto mf = ap::MappedFile::openShared(t.resolved);
    if (!mf) return;
    mf->prefetch(0, mf->size());
    bytesMapped += mf->size();
    ++filesMapped;
    int w = 0, h = 0, ch = 0;
    unsigned char* rgba = nullptr;
#ifdef _WIN32
    if (!tryDecodeWIC(reinterpret_cast<const uint8_t*>(mf->bytes().data()),
                      mf->size(), &w, &h, &rgba)) {
        rgba = nullptr;
    }
#endif
    if (!rgba) {
        rgba = stbi_load_from_memory(
            reinterpret_cast<const unsigned char*>(mf->bytes().data()),
            int(mf->size()), &w, &h, &ch, 4);
    }
    if (!rgba) {
        AP_LOG_WARN("tex", "decode failed: %s", t.key.c_str());
        return;
    }
    DecodedTex dt;
    dt.w = w;
    dt.h = h;
    dt.rgba = {rgba, &stbiFree};
    pixels += size_t(w) * size_t(h);
    {
        std::lock_guard<std::mutex> lk(qMx_);
        texs_[slot] = std::move(dt);
        slotDone_[slot].store(1, std::memory_order_release);
        advancePrefix();
    }
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
    size_t old = 0, need = 0;
    {
        std::lock_guard<std::mutex> lk(qMx_);
        old = texs_.size();
        for (auto& r : nd) diffuseQ_.push_back(std::move(r));
        for (auto& r : no) othersQ_.push_back(std::move(r));
        need = diffuseQ_.size();
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
    // stat-only others: just mmap for counters (cheap)
    for (auto& r : no) {
        auto mf = ap::MappedFile::openShared(r.resolved);
        if (mf) {
            bytesMapped += mf->size();
            ++filesMapped;
        }
    }
    if (nd.empty()) return;
    // dispatch diffuse decodes as a taskflow on the shared global executor
    // so PbrtParser's ply loads and texture decodes share the same pool
    tf::Taskflow flow;
    flow.for_each_index(int(old), int(need), 1,
                        [&](int idx) { decodeOne(diffuseQ_[size_t(idx)], size_t(idx)); });
    auto fut = ap::globalExecutor().run(std::move(flow));
    {
        std::lock_guard<std::mutex> lk(qMx_);
        futures_.push_back(std::move(fut));
    }
    // done_ will be set in finish() after all futures complete
}

void TexPipeline::finish() {
    std::vector<tf::Future<void>> local;
    {
        std::lock_guard<std::mutex> lk(qMx_);
        local = std::move(futures_);
        futures_.clear();
    }
    for (auto& f : local) f.wait();
    // ensure any still-pending enqueues that raced with this finish are also drained
    // (enqueue may have added new futures after we moved)
    while (true) {
        std::vector<tf::Future<void>> more;
        {
            std::lock_guard<std::mutex> lk(qMx_);
            if (futures_.empty()) break;
            more = std::move(futures_);
            futures_.clear();
        }
        for (auto& f : more) f.wait();
    }
    done_.store(true, std::memory_order_release);
}

} // namespace texp
