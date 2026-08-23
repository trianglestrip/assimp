#include "TexPipeline.h"
#include <assetpack/AssetPack.h>
#include <assetpack/Config.h>
#include "../src/core/TaskExecutor.h"
#include <taskflow/algorithm/for_each.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
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

std::string cachePathFor(const std::string& resolved) {
    const auto& cfg = ap::Config::instance();
    if (cfg.cacheDir.empty()) return {};
    size_t h = std::hash<std::string>{}(resolved);
    std::error_code ec;
    auto sz = std::filesystem::file_size(resolved, ec);
    if (!ec) h ^= std::hash<size_t>{}(size_t(sz)) + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
    char name[64];
    std::snprintf(name, sizeof name, "%016llx.dds", (unsigned long long)h);
    std::filesystem::path p = cfg.cacheDir;
    p /= name;
    return p.string();
}

// ---- DDS with full mip chain (runtime cache, not offline) ----
#pragma pack(push, 1)
struct DDS_PIXELFORMAT { uint32_t dwSize; uint32_t dwFlags; uint32_t dwFourCC; uint32_t dwRGBBitCount; uint32_t dwRBitMask; uint32_t dwGBitMask; uint32_t dwBBitMask; uint32_t dwABitMask; };
struct DDS_HEADER { uint32_t dwSize; uint32_t dwFlags; uint32_t dwHeight; uint32_t dwWidth; uint32_t dwPitchOrLinearSize; uint32_t dwDepth; uint32_t dwMipMapCount; uint32_t dwReserved1[11]; DDS_PIXELFORMAT ddspf; uint32_t dwCaps; uint32_t dwCaps2; uint32_t dwCaps3; uint32_t dwCaps4; uint32_t dwReserved2; };
#pragma pack(pop)
static constexpr uint32_t DDS_MAGIC = 0x20534444; // "DDS "
static constexpr uint32_t DDSD_CAPS=0x1, DDSD_HEIGHT=0x2, DDSD_WIDTH=0x4, DDSD_PITCH=0x8, DDSD_PIXELFORMAT=0x1000, DDSD_MIPMAPCOUNT=0x20000;
static constexpr uint32_t DDPF_ALPHAPIXELS=0x1, DDPF_RGB=0x40;
static constexpr uint32_t DDSCAPS_COMPLEX=0x8, DDSCAPS_MIPMAP=0x400000, DDSCAPS_TEXTURE=0x1000;

// box-filter mip chain from base RGBA (w*h*4)
std::vector<std::vector<uint8_t>> generateMipsCPU(const unsigned char* base, int w, int h) {
    std::vector<std::vector<uint8_t>> out;
    if (!base || w<=0 || h<=0) return out;
    int cw=w, ch=h;
    std::vector<uint8_t> cur(base, base + size_t(w)*size_t(h)*4);
    std::vector<uint8_t> nxt;
    while (cw>1 || ch>1) {
        int nw = std::max(1, cw/2), nh = std::max(1, ch/2);
        nxt.assign(size_t(nw)*size_t(nh)*4, 0);
        for (int y=0;y<nh;++y) for (int x=0;x<nw;++x) {
            int r=0,g=0,b=0,a=0,cnt=0;
            for (int dy=0; dy<2; ++dy) for (int dx=0; dx<2; ++dx) {
                int sx=x*2+dx, sy=y*2+dy;
                if (sx>=cw || sy>=ch) continue;
                size_t si = (size_t(sy)*size_t(cw) + size_t(sx))*4;
                r+=cur[si+0]; g+=cur[si+1]; b+=cur[si+2]; a+=cur[si+3]; ++cnt;
            }
            size_t di = (size_t(y)*size_t(nw)+size_t(x))*4;
            nxt[di+0]=uint8_t(r/cnt); nxt[di+1]=uint8_t(g/cnt); nxt[di+2]=uint8_t(b/cnt); nxt[di+3]=uint8_t(a/cnt);
        }
        out.push_back(nxt);
        cur = nxt;
        cw=nw; ch=nh;
    }
    return out;
}
bool writeDDS(const std::string& path, int w, int h, const unsigned char* base, const std::vector<std::vector<uint8_t>>& mips) {
    uint32_t mipCount = 1 + uint32_t(mips.size());
    DDS_HEADER hdr{};
    hdr.dwSize=124;
    hdr.dwFlags = DDSD_CAPS|DDSD_HEIGHT|DDSD_WIDTH|DDSD_PIXELFORMAT|DDSD_PITCH;
    if (mipCount>1) hdr.dwFlags |= DDSD_MIPMAPCOUNT;
    hdr.dwHeight=uint32_t(h); hdr.dwWidth=uint32_t(w);
    hdr.dwPitchOrLinearSize = uint32_t(w)*4;
    hdr.dwMipMapCount = mipCount;
    hdr.ddspf.dwSize=32; hdr.ddspf.dwFlags=DDPF_RGB|DDPF_ALPHAPIXELS; hdr.ddspf.dwRGBBitCount=32;
    hdr.ddspf.dwRBitMask=0x000000ff; hdr.ddspf.dwGBitMask=0x0000ff00; hdr.ddspf.dwBBitMask=0x00ff0000; hdr.ddspf.dwABitMask=0xff000000;
    hdr.dwCaps = DDSCAPS_TEXTURE; if (mipCount>1) hdr.dwCaps |= DDSCAPS_COMPLEX|DDSCAPS_MIPMAP;
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    uint32_t magic=DDS_MAGIC;
    f.write(reinterpret_cast<const char*>(&magic),4);
    f.write(reinterpret_cast<const char*>(&hdr), sizeof hdr);
    size_t baseSz=size_t(w)*size_t(h)*4;
    f.write(reinterpret_cast<const char*>(base), baseSz);
    for (auto& m: mips) f.write(reinterpret_cast<const char*>(m.data()), m.size());
    return !!f;
}
bool tryLoadDDS(const std::string& path, int* w, int* h, unsigned char** outBase, std::vector<std::vector<uint8_t>>* outMips) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    uint32_t magic=0; f.read(reinterpret_cast<char*>(&magic),4);
    if (!f || magic!=DDS_MAGIC) return false;
    DDS_HEADER hdr{}; f.read(reinterpret_cast<char*>(&hdr), sizeof hdr);
    if (!f || hdr.dwSize!=124 || hdr.dwWidth==0 || hdr.dwHeight==0 || hdr.dwWidth>16384 || hdr.dwHeight>16384) return false;
    if (!(hdr.ddspf.dwFlags & DDPF_RGB)) return false;
    int wi=int(hdr.dwWidth), hi=int(hdr.dwHeight);
    uint32_t mipCount = (hdr.dwFlags & DDSD_MIPMAPCOUNT) ? std::max(1u, hdr.dwMipMapCount) : 1u;
    size_t baseSz=size_t(wi)*size_t(hi)*4;
    unsigned char* base=(unsigned char*)malloc(baseSz);
    if (!base) return false;
    f.read(reinterpret_cast<char*>(base), baseSz);
    if (!f) { free(base); return false; }
    std::vector<std::vector<uint8_t>> mips;
    int cw=wi, ch=hi;
    for (uint32_t i=1;i<mipCount;++i) {
        int nw=std::max(1,cw/2), nh=std::max(1,ch/2);
        size_t sz=size_t(nw)*size_t(nh)*4;
        std::vector<uint8_t> buf(sz);
        f.read(reinterpret_cast<char*>(buf.data()), sz);
        if (!f) { free(base); return false; }
        mips.push_back(std::move(buf));
        cw=nw; ch=nh;
    }
    *w=wi; *h=hi; *outBase=base;
    if (outMips) *outMips = std::move(mips);
    return true;
}
bool tryLoadCache(const std::string& resolved, int* w, int* h, unsigned char** out, std::vector<std::vector<uint8_t>>* outMips) {
    const auto& cfg = ap::Config::instance();
    if (!cfg.cacheImage) return false;
    std::string cp = cachePathFor(resolved);
    if (cp.empty()) return false;
    // try DDS first, then legacy raw cache (wi,hi,rgba) for compat
    if (tryLoadDDS(cp, w, h, out, outMips)) return true;
    std::ifstream f(cp, std::ios::binary);
    if (!f) return false;
    int32_t wi=0, hi=0; f.read(reinterpret_cast<char*>(&wi),4); f.read(reinterpret_cast<char*>(&hi),4);
    if (!f || wi<=0 || hi<=0 || wi>16384 || hi>16384) return false;
    // peek if remaining is exactly wi*hi*4 (legacy) vs DDS header already failed
    size_t sz=size_t(wi)*size_t(hi)*4;
    unsigned char* buf=(unsigned char*)malloc(sz);
    if (!buf) return false;
    f.read(reinterpret_cast<char*>(buf), sz);
    if (!f) { free(buf); return false; }
    *w=wi; *h=hi; *out=buf; if (outMips) outMips->clear();
    return true;
}
void saveCache(const std::string& resolved, int w, int h, const unsigned char* rgba, const std::vector<std::vector<uint8_t>>& mips) {
    const auto& cfg = ap::Config::instance();
    if (!cfg.cacheImage) return;
    std::string cp = cachePathFor(resolved);
    if (cp.empty()) return;
    auto dm = mips.empty() ? generateMipsCPU(rgba, w, h) : mips;
    writeDDS(cp, w, h, rgba, dm);
}
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
        for (auto& m: t.mips) freed += m.size();
        t.rgba.reset(); // stbi_image_free
        t.mips.clear(); t.mips.shrink_to_fit();
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
    int w = 0, h = 0, ch = 0;
    unsigned char* rgba = nullptr;
    std::vector<std::vector<uint8_t>> cachedMips;
    if (tryLoadCache(t.resolved, &w, &h, &rgba, &cachedMips)) {
        DecodedTex dt; dt.w=w; dt.h=h; dt.rgba={rgba, &stbiFree}; dt.mips=std::move(cachedMips);
        pixels += size_t(w)*size_t(h);
        {
            std::lock_guard<std::mutex> lk(qMx_);
            texs_[slot] = std::move(dt);
            slotDone_[slot].store(1, std::memory_order_release);
            advancePrefix();
        }
        auto mf2 = ap::MappedFile::openShared(t.resolved);
        if (mf2) { bytesMapped += mf2->size(); ++filesMapped; }
        return;
    }
    auto mf = ap::MappedFile::openShared(t.resolved);
    if (!mf) return;
    mf->prefetch(0, mf->size());
    bytesMapped += mf->size();
    ++filesMapped;
    bool usedWic = false;
#ifdef _WIN32
    {
        auto t0 = std::chrono::steady_clock::now();
        usedWic = tryDecodeWIC(reinterpret_cast<const uint8_t*>(mf->bytes().data()),
                               mf->size(), &w, &h, &rgba);
        auto t1 = std::chrono::steady_clock::now();
        wicMicros.fetch_add(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count(),
            std::memory_order_relaxed);
        wicCount.fetch_add(1, std::memory_order_relaxed);
        if (!usedWic) rgba = nullptr;
    }
#endif
    if (!rgba) {
        auto t0 = std::chrono::steady_clock::now();
        rgba = stbi_load_from_memory(
            reinterpret_cast<const unsigned char*>(mf->bytes().data()),
            int(mf->size()), &w, &h, &ch, 4);
        auto t1 = std::chrono::steady_clock::now();
        stbMicros.fetch_add(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count(),
            std::memory_order_relaxed);
        stbCount.fetch_add(1, std::memory_order_relaxed);
        (void)usedWic;
    }
    if (!rgba) {
        AP_LOG_WARN("tex", "decode failed: %s", t.key.c_str());
        return;
    }
    std::vector<std::vector<uint8_t>> mips;
    if (ap::Config::instance().cacheImage) {
        mips = generateMipsCPU(rgba, w, h);
        saveCache(t.resolved, w, h, rgba, mips);
    }
    DecodedTex dt;
    dt.w = w;
    dt.h = h;
    dt.rgba = {rgba, &stbiFree};
    dt.mips = std::move(mips);
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
    AP_LOG("tex", "enqueue %zu..%zu on %zu workers", old, need, ap::globalExecutor().num_workers());
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
    if (wicCount.load() || stbCount.load()) {
        AP_LOG("tex", "decode breakdown: WIC %.1f ms (%zu) | stb %.1f ms (%zu)",
               wicMicros.load() / 1000.0, size_t(wicCount.load()),
               stbMicros.load() / 1000.0, size_t(stbCount.load()));
    }
    done_.store(true, std::memory_order_release);
}

} // namespace texp
