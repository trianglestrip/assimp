#include "DxRenderer.h"
#include <assetpack/Log.h>

#include <SDL2/SDL_syswm.h>

#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <dxgi1_4.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

#include <GraphicsMemory.h>
#include <RenderTargetState.h>
#include <ResourceUploadBatch.h>
#include <SpriteBatch.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <future>
#include <vector>

using Microsoft::WRL::ComPtr;
using namespace DirectX;

namespace {

constexpr UINT kBackBuffers = 2;
constexpr UINT kMsaaSamples = 4;
constexpr UINT kMaxTex = 8192;     // SRV heap: white + textures + HUD + load

// ---- tiny d3dx12 stand-ins (header removed from the Windows SDK) ----
D3D12_RESOURCE_DESC bufDesc(UINT64 size) {
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Width = size;
    d.Height = 1;
    d.DepthOrArraySize = 1;
    d.MipLevels = 1;
    d.SampleDesc = {1, 0};
    d.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    return d;
}

D3D12_RESOURCE_DESC texDesc(UINT w, UINT h, DXGI_FORMAT fmt) {
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = w;
    d.Height = h;
    d.DepthOrArraySize = 1;
    d.MipLevels = 1;
    d.Format = fmt;
    d.SampleDesc = {1, 0};
    d.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    return d;
}

// 24-bit BMP, bottom-up rows
void saveBmp(const std::string& path, const uint32_t* px, int w, int h) {
    const int rowBytes = (w * 3 + 3) & ~3;
    const uint32_t imgBytes = uint32_t(rowBytes) * uint32_t(h);
    std::vector<uint8_t> out(54 + imgBytes);
    auto put16 = [&](size_t o, uint16_t v) {
        out[o] = uint8_t(v);
        out[o + 1] = uint8_t(v >> 8);
    };
    auto put32 = [&](size_t o, uint32_t v) {
        out[o] = uint8_t(v);
        out[o + 1] = uint8_t(v >> 8);
        out[o + 2] = uint8_t(v >> 16);
        out[o + 3] = uint8_t(v >> 24);
    };
    put16(0, 0x4D42);
    put32(2, 54 + imgBytes);
    put32(10, 54);
    put32(14, 40);
    put32(18, uint32_t(w));
    put32(22, uint32_t(h));
    put16(26, 1);
    put16(28, 24);
    put32(34, imgBytes);
    for (int y = 0; y < h; ++y) {
        const uint32_t* row = px + size_t(h - 1 - y) * size_t(w);
        uint8_t* dst = out.data() + 54 + size_t(y) * rowBytes;
        for (int x = 0; x < w; ++x) {
            dst[3 * x] = uint8_t(row[x]);
            dst[3 * x + 1] = uint8_t(row[x] >> 8);
            dst[3 * x + 2] = uint8_t(row[x] >> 16);
        }
    }
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(out.data()),
            std::streamsize(out.size()));
    AP_LOG("dx", "saved %s", path.c_str());
}

} // namespace

struct DxRenderer::Impl {
    // device / presentation
    ComPtr<ID3D12Device> dev;
    ComPtr<ID3D12CommandQueue> queue;
    ComPtr<IDXGISwapChain3> swap;
    ComPtr<ID3D12CommandAllocator> alloc[kBackBuffers];
    ComPtr<ID3D12GraphicsCommandList> list;
    ComPtr<ID3D12DescriptorHeap> rtvHeap, dsvHeap, srvHeap;
    ComPtr<ID3D12Resource> back[kBackBuffers];
    // 4x MSAA scene target: rendered here, resolved into the back buffer
    // before present (sprites ride along on the same resolve)
    ComPtr<ID3D12Resource> msaaRT, msaaDepth;
    ComPtr<ID3D12Resource> readback;      // --shot staging
    UINT srvSize = 0;
    int winW = 0, winH = 0;
    D3D12_VIEWPORT vp{};
    D3D12_RECT scissor{};

    // frame pacing: per-backbuffer fence values + one shared counter
    ComPtr<ID3D12Fence> fence;
    HANDLE fenceEvt = nullptr;
    UINT64 fenceCounter = 0;
    UINT64 frameFence[kBackBuffers] = {0, 0};

    // DirectXTK helpers
    std::unique_ptr<ResourceUploadBatch> upload;
    std::unique_ptr<GraphicsMemory> gfxMem;
    std::unique_ptr<SpriteBatch> sprites;
    std::future<void> uploadDone;         // retained End() future
    bool uploadOpen = false;

    // pipeline (scene PSO built at setGeometry; blobs compiled at init)
    ComPtr<ID3D12RootSignature> rs;
    ComPtr<ID3DBlob> vsBlob, psBlob;
    ComPtr<ID3D12PipelineState> pso;

    // geometry: parser pools uploaded verbatim (pos / uv / index)
    ComPtr<ID3D12Resource> vbPos, vbUv, ib;
    D3D12_VERTEX_BUFFER_VIEW vbvPos{}, vbvUv{};
    D3D12_INDEX_BUFFER_VIEW ibv{};

    // overlay + texture SRV table: [0]=white, [1..]=textures,
    // [kMaxTex+1]=HUD, [kMaxTex+2]=loading
    ComPtr<ID3D12Resource> white, hudTex, loadTex;
    D3D12_GPU_DESCRIPTOR_HANDLE whiteGpu{}, hudGpu{}, loadGpu{};
    bool hudReady = false, loadReady = false;
    std::vector<ComPtr<ID3D12Resource>> texRes;
    std::vector<D3D12_GPU_DESCRIPTOR_HANDLE> texGpu;

    // profile
    uint64_t frames = 0;

    ~Impl() {
        if (fenceEvt) CloseHandle(fenceEvt);
    }

    // ---- helpers ----
    void chk(HRESULT hr, const char* what) {
        if (FAILED(hr))
            AP_LOG_WARN("dx", "%s -> %08X", what, unsigned(hr));
    }

    ComPtr<ID3D12Resource> makeBuf(UINT64 size, D3D12_HEAP_TYPE heap,
                                   D3D12_RESOURCE_STATES state) {
        ComPtr<ID3D12Resource> res;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = heap;
        const D3D12_RESOURCE_DESC d = bufDesc(size);
        chk(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d,
                                         state, nullptr, IID_PPV_ARGS(&res)),
            "CreateCommittedResource(buffer)");
        return res;
    }

    ComPtr<ID3D12Resource> makeTex(UINT w, UINT h, DXGI_FORMAT fmt,
                                   D3D12_RESOURCE_STATES state, UINT mips = 1,
                                   D3D12_RESOURCE_FLAGS flags =
                                       D3D12_RESOURCE_FLAG_NONE) {
        ComPtr<ID3D12Resource> res;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        D3D12_RESOURCE_DESC d = texDesc(w, h, fmt);
        d.MipLevels = mips;
        d.Flags = flags;
        chk(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d,
                                         state, nullptr, IID_PPV_ARGS(&res)),
            "CreateCommittedResource(texture)");
        return res;
    }

    void srv(ID3D12Resource* res, D3D12_CPU_DESCRIPTOR_HANDLE cpu) {
        D3D12_SHADER_RESOURCE_VIEW_DESC d{};
        d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        d.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        d.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        d.Texture2D.MipLevels = 0xFFFFFFFF;   // all mip levels
        dev->CreateShaderResourceView(res, &d, cpu);
    }

    void barrier(ID3D12Resource* r, D3D12_RESOURCE_STATES a,
                 D3D12_RESOURCE_STATES b) {
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = r;
        br.Transition.Subresource = 0;
        br.Transition.StateBefore = a;
        br.Transition.StateAfter = b;
        list->ResourceBarrier(1, &br);
    }

    void waitFence(UINT64 v) {
        if (fence->GetCompletedValue() >= v) return;
        fence->SetEventOnCompletion(v, fenceEvt);
        WaitForSingleObject(fenceEvt, INFINITE);
    }

    void drain() {   // full idle (shutdown / screenshots)
        const UINT64 v = ++fenceCounter;
        queue->Signal(fence.Get(), v);
        waitFence(v);
    }

    // ---- frame prologue: wait only for the frame two submissions ago ----
    UINT beginFrame() {
        const UINT f = swap->GetCurrentBackBufferIndex();
        waitFence(frameFence[f]);
        if (uploadDone.valid())
            uploadDone.wait();          // uploads overlap, never pre-empt
        gfxMem->GarbageCollect();
        upload->Begin();
        uploadOpen = true;
        alloc[f]->Reset();
        list->Reset(alloc[f].Get(), nullptr);

        // everything (scene + overlay sprites) renders into the MSAA
        // target; the back buffer only receives the resolved image
        barrier(msaaRT.Get(), D3D12_RESOURCE_STATE_RESOLVE_SOURCE,
                D3D12_RESOURCE_STATE_RENDER_TARGET);
        barrier(back[f].Get(), D3D12_RESOURCE_STATE_PRESENT,
                D3D12_RESOURCE_STATE_RESOLVE_DEST);
        const UINT rtvSize =
            dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
        D3D12_CPU_DESCRIPTOR_HANDLE rtv =
            rtvHeap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += kBackBuffers * rtvSize;   // MSAA RTV sits after the backs
        D3D12_CPU_DESCRIPTOR_HANDLE dsv =
            dsvHeap->GetCPUDescriptorHandleForHeapStart();
        dsv.ptr += dev->GetDescriptorHandleIncrementSize(
                       D3D12_DESCRIPTOR_HEAP_TYPE_DSV);   // [1] = MSAA depth
        list->OMSetRenderTargets(1, &rtv, FALSE, &dsv);
        const float clearCol[4] = {0x14 / 255.f, 0x14 / 255.f, 0x14 / 255.f, 1.f};
        list->ClearRenderTargetView(rtv, clearCol, 0, nullptr);
        list->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.f, 0, 0,
                                    nullptr);
        list->RSSetViewports(1, &vp);
        list->RSSetScissorRects(1, &scissor);
        ID3D12DescriptorHeap* heaps[1] = {srvHeap.Get()};
        list->SetDescriptorHeaps(1, heaps);
        // scene root signature up front (SpriteBatch::Begin rebinds its own
        // for the overlay draws); without this the scene's root constants
        // are invalid and the driver crashes on execute
        list->SetGraphicsRootSignature(rs.Get());
        return f;
    }

    // ---- frame epilogue: resolve the MSAA target, submit, present ----
    void endFrame(UINT f, const char* shotPath) {
        barrier(msaaRT.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET,
                D3D12_RESOURCE_STATE_RESOLVE_SOURCE);
        list->ResolveSubresource(back[f].Get(), 0, msaaRT.Get(), 0,
                                 DXGI_FORMAT_R8G8B8A8_UNORM);
        if (shotPath) {
            barrier(back[f].Get(), D3D12_RESOURCE_STATE_RESOLVE_DEST,
                    D3D12_RESOURCE_STATE_COPY_SOURCE);
            D3D12_TEXTURE_COPY_LOCATION dst{};
            dst.pResource = readback.Get();
            dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            dst.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            dst.PlacedFootprint.Footprint.Width = UINT(winW);
            dst.PlacedFootprint.Footprint.Height = UINT(winH);
            dst.PlacedFootprint.Footprint.Depth = 1;
            dst.PlacedFootprint.Footprint.RowPitch = UINT(winW) * 4;
            D3D12_TEXTURE_COPY_LOCATION src{};
            src.pResource = back[f].Get();
            src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
            barrier(back[f].Get(), D3D12_RESOURCE_STATE_COPY_SOURCE,
                    D3D12_RESOURCE_STATE_PRESENT);
        } else {
            barrier(back[f].Get(), D3D12_RESOURCE_STATE_RESOLVE_DEST,
                    D3D12_RESOURCE_STATE_PRESENT);
        }

        // flush this frame's uploads BEFORE the frame list executes (the
        // queue is FIFO, so the copies/transitions land first); the future
        // is retained and awaited in beginFrame, not here
        uploadDone = upload->End(queue.Get());
        uploadOpen = false;

        list->Close();
        ID3D12CommandList* lists[] = {list.Get()};
        queue->ExecuteCommandLists(1, lists);
        frameFence[f] = ++fenceCounter;
        queue->Signal(fence.Get(), frameFence[f]);
        gfxMem->Commit(queue.Get());   // SpriteBatch dynamic memory
        swap->Present(0, 0);

        if (shotPath) {
            waitFence(frameFence[f]);  // the copy has completed
            void* p = nullptr;
            readback->Map(0, nullptr, &p);
            std::vector<uint32_t> px(size_t(winW) * size_t(winH));
            const unsigned char* s = static_cast<const unsigned char*>(p);
            for (size_t i = 0; i < px.size(); ++i)
                px[i] = 0xFF000000u | (uint32_t(s[4 * i]) << 16)
                        | (uint32_t(s[4 * i + 1]) << 8) | uint32_t(s[4 * i + 2]);
            readback->Unmap(0, nullptr);
            saveBmp(shotPath, px.data(), winW, winH);
        }
    }

    // overlay upload: ARGB u32 strip -> RGBA texture through the batch
    void uploadOverlay(ComPtr<ID3D12Resource>& tex, bool& ready,
                       const uint32_t* argb, int w, int h) {
        if (ready)
            upload->Transition(tex.Get(),
                               D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                               D3D12_RESOURCE_STATE_COPY_DEST);
        std::vector<unsigned char> rgba(size_t(w) * size_t(h) * 4);
        for (size_t p = 0; p < size_t(w) * size_t(h); ++p) {
            const uint32_t c = argb[p];
            rgba[4 * p] = unsigned char(c >> 16);
            rgba[4 * p + 1] = unsigned char(c >> 8);
            rgba[4 * p + 2] = unsigned char(c);
            rgba[4 * p + 3] = 255;
        }
        const D3D12_SUBRESOURCE_DATA sd{rgba.data(), LONG_PTR(size_t(w)) * 4,
                                        LONG_PTR(rgba.size())};
        upload->Upload(tex.Get(), 0, &sd, 1);
        upload->Transition(tex.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                           D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        ready = true;
    }
};

DxRenderer::DxRenderer() = default;

DxRenderer::~DxRenderer() { shutdown(); }

bool DxRenderer::init(SDL_Window* win, int w, int h, bool warp) {
    impl_ = std::make_unique<Impl>();
    Impl& dx = *impl_;
    dx.winW = w;
    dx.winH = h;

    ComPtr<ID3D12Debug> dbg;
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&dbg))))
        dbg->EnableDebugLayer();
    ComPtr<IDXGIFactory4> fac;
    if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&fac)))) {
        AP_LOG_WARN("dx", "CreateDXGIFactory1 failed");
        return false;
    }
    // virtual display adapters (remote-desktop / emulator drivers) also
    // report D3D12 support; pick the most dedicated video memory so the
    // real GPU wins
    ComPtr<IDXGIAdapter1> best;
    SIZE_T bestMem = 0;
    for (UINT i = 0;; ++i) {
        ComPtr<IDXGIAdapter1> a;
        if (fac->EnumAdapters1(i, &a) != S_OK) break;
        DXGI_ADAPTER_DESC1 d;
        a->GetDesc1(&d);
        if (d.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) continue;
        if (warp) continue;
        ComPtr<ID3D12Device> probe;
        if (SUCCEEDED(D3D12CreateDevice(a.Get(), D3D_FEATURE_LEVEL_11_0,
                                        IID_PPV_ARGS(&probe))) &&
            d.DedicatedVideoMemory > bestMem) {
            bestMem = d.DedicatedVideoMemory;
            best = a;
        }
    }
    if (!best && warp) {
        for (UINT i = 0;; ++i) {
            ComPtr<IDXGIAdapter1> a;
            if (fac->EnumAdapters1(i, &a) != S_OK) break;
            DXGI_ADAPTER_DESC1 d;
            a->GetDesc1(&d);
            if (d.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
                best = a;
                bestMem = d.DedicatedVideoMemory;
                break;
            }
        }
    }
    if (best) {
        char name[128] = {};
        DXGI_ADAPTER_DESC1 d;
        best->GetDesc1(&d);
        wcstombs(name, d.Description, sizeof name - 1);
        if (FAILED(D3D12CreateDevice(best.Get(), D3D_FEATURE_LEVEL_11_0,
                                     IID_PPV_ARGS(&dx.dev))))
            dx.dev.Reset();
        AP_LOG("dx", "adapter: %s (%zu MB)", name, size_t(bestMem) / 1048576);
    }
    if (!dx.dev &&
        FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0,
                                 IID_PPV_ARGS(&dx.dev)))) {
        AP_LOG_WARN("dx", "D3D12CreateDevice failed (no hardware, no WARP)");
        return false;
    }

    D3D12_COMMAND_QUEUE_DESC qd{};
    qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    dx.chk(dx.dev->CreateCommandQueue(&qd, IID_PPV_ARGS(&dx.queue)),
           "CreateCommandQueue");
    for (UINT i = 0; i < kBackBuffers; ++i) {
        dx.chk(dx.dev->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                              IID_PPV_ARGS(&dx.alloc[i])),
               "CreateCommandAllocator");
    }
    dx.chk(dx.dev->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT,
                                     dx.alloc[0].Get(), nullptr,
                                     IID_PPV_ARGS(&dx.list)),
           "CreateCommandList");
    dx.list->Close();   // start closed; every frame resets it

    SDL_SysWMinfo info;
    SDL_VERSION(&info.version);
    if (!SDL_GetWindowWMInfo(win, &info)) {
        AP_LOG_WARN("dx", "SDL_GetWindowWMInfo failed");
        return false;
    }
    DXGI_SWAP_CHAIN_DESC1 sc{};
    sc.Width = UINT(w);
    sc.Height = UINT(h);
    sc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sc.SampleDesc = {1, 0};
    sc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sc.BufferCount = kBackBuffers;
    sc.Scaling = DXGI_SCALING_STRETCH;
    sc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    sc.AlphaMode = DXGI_ALPHA_MODE_UNSPECIFIED;
    ComPtr<IDXGISwapChain1> sc1;
    dx.chk(fac->CreateSwapChainForHwnd(dx.queue.Get(), info.info.win.window,
                                       &sc, nullptr, nullptr, &sc1),
           "CreateSwapChainForHwnd");
    sc1.As(&dx.swap);

    D3D12_DESCRIPTOR_HEAP_DESC hd{};
    hd.NumDescriptors = kBackBuffers + 1;   // backs + the MSAA target
    hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    dx.chk(dx.dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&dx.rtvHeap)),
           "RTV heap");
    hd.NumDescriptors = 2;                  // depth + the MSAA depth
    hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    dx.chk(dx.dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&dx.dsvHeap)),
           "DSV heap");
    hd.NumDescriptors = kMaxTex + 3;
    hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    hd.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    dx.chk(dx.dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&dx.srvHeap)),
           "SRV heap");
    dx.srvSize = dx.dev->GetDescriptorHandleIncrementSize(
        D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    const UINT rtvSize =
        dx.dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    D3D12_CPU_DESCRIPTOR_HANDLE rtv =
        dx.rtvHeap->GetCPUDescriptorHandleForHeapStart();
    for (UINT i = 0; i < kBackBuffers; ++i) {
        dx.chk(dx.swap->GetBuffer(i, IID_PPV_ARGS(&dx.back[i])), "GetBuffer");
        dx.dev->CreateRenderTargetView(dx.back[i].Get(), nullptr, rtv);
        rtv.ptr += rtvSize;
    }
    // MSAA scene target (+ depth) and their views at the heap tails
    {
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        D3D12_RESOURCE_DESC rt = texDesc(UINT(w), UINT(h),
                                         DXGI_FORMAT_R8G8B8A8_UNORM);
        rt.SampleDesc = {kMsaaSamples, 0};
        rt.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
        dx.chk(dx.dev->CreateCommittedResource(
                   &hp, D3D12_HEAP_FLAG_NONE, &rt,
                   D3D12_RESOURCE_STATE_RESOLVE_SOURCE, nullptr,
                   IID_PPV_ARGS(&dx.msaaRT)),
               "MSAA render target");
        dx.dev->CreateRenderTargetView(dx.msaaRT.Get(), nullptr, rtv);
        D3D12_CLEAR_VALUE cv{};
        cv.Format = DXGI_FORMAT_D32_FLOAT;
        cv.DepthStencil.Depth = 1.f;
        D3D12_RESOURCE_DESC dd = texDesc(UINT(w), UINT(h),
                                         DXGI_FORMAT_D32_FLOAT);
        dd.SampleDesc = {kMsaaSamples, 0};
        dd.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
        dx.chk(dx.dev->CreateCommittedResource(
                   &hp, D3D12_HEAP_FLAG_NONE, &dd,
                   D3D12_RESOURCE_STATE_DEPTH_WRITE, &cv,
                   IID_PPV_ARGS(&dx.msaaDepth)),
               "MSAA depth buffer");
        D3D12_CPU_DESCRIPTOR_HANDLE dsv =
            dx.dsvHeap->GetCPUDescriptorHandleForHeapStart();
        dsv.ptr += dx.dev->GetDescriptorHandleIncrementSize(
                       D3D12_DESCRIPTOR_HEAP_TYPE_DSV);
        dx.dev->CreateDepthStencilView(dx.msaaDepth.Get(), nullptr, dsv);
    }
    D3D12_CLEAR_VALUE cv{};   // (scene depth lives in the MSAA block above)
    (void)cv;

    if (FAILED(dx.dev->CreateFence(0, D3D12_FENCE_FLAG_NONE,
                                   IID_PPV_ARGS(&dx.fence))) ||
        !(dx.fenceEvt = CreateEvent(nullptr, FALSE, FALSE, nullptr))) {
        AP_LOG_WARN("dx", "fence creation failed");
        return false;
    }

    // root signature: b0 = camera math (16 DWORDs), b1 = material color
    // (4) + hasTex (1), t0 = texture table, s0 = linear/clamp (sprites),
    // s1 = nearest/clamp (scene)
    D3D12_DESCRIPTOR_RANGE range{};
    range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    range.NumDescriptors = 1;
    D3D12_ROOT_PARAMETER params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.ShaderRegister = 0;
    params[0].Constants.Num32BitValues = 16;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[1].Constants.ShaderRegister = 1;
    params[1].Constants.Num32BitValues = 5;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable = {1, &range};
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    D3D12_STATIC_SAMPLER_DESC ss[2]{};
    ss[0].Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    ss[0].AddressU = ss[0].AddressV = ss[0].AddressW =
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    ss[0].ShaderRegister = 0;
    ss[1].Filter = D3D12_FILTER_ANISOTROPIC;   // 16x anisotropic: keeps the
        // mip chain's bandwidth win but stays sharp on grazing-angle
        // surfaces (floors/walls), where point+linear-mip looked mushy
    ss[1].MaxAnisotropy = 16;   // no LOD bias: correct mip selection is the
        // cleanest under motion; sharpening biases made tile grout and
        // chair caning shimmer while the camera moved
    ss[1].AddressU = ss[1].AddressV = ss[1].AddressW =
        D3D12_TEXTURE_ADDRESS_MODE_WRAP;   // OBJ tiling uvs go 0..N; clamp
                                           // smeared them into edge streaks
    ss[1].ShaderRegister = 1;
    D3D12_ROOT_SIGNATURE_DESC rsd{};
    rsd.NumParameters = 3;
    rsd.pParameters = params;
    rsd.NumStaticSamplers = 2;
    rsd.pStaticSamplers = ss;
    rsd.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
    ComPtr<ID3DBlob> rsBlob, rsErr;
    if (FAILED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1,
                                           &rsBlob, &rsErr))) {
        AP_LOG_WARN("dx", "SerializeRootSignature failed");
        return false;
    }
    if (FAILED(dx.dev->CreateRootSignature(0, rsBlob->GetBufferPointer(),
                                           rsBlob->GetBufferSize(),
                                           IID_PPV_ARGS(&dx.rs)))) {
        AP_LOG_WARN("dx", "CreateRootSignature failed");
        return false;
    }

    // compile the scene shaders now so the first scene frame doesn't pay
    // for D3DCompile; the PSO itself needs the winding result and is
    // built in setGeometry
    static const char* kVs = R"(
struct VSIn { float3 pos : POSITION; float2 uv : TEXCOORD; };
struct VSOut { float4 pos : SV_Position; float2 uv : TEXCOORD; };
cbuffer Cam : register(b0) { float4 g_s[4]; };
VSOut main(VSIn v) {
    float3 p = (v.pos - g_s[1].yzw) * g_s[1].x;
    float x1 = g_s[0].x * p.x + g_s[0].y * p.z;
    float z1 = -g_s[0].y * p.x + g_s[0].x * p.z;
    float vx = x1 + g_s[2].y;
    float vy = g_s[0].z * p.y - g_s[0].w * z1;
    float vz = g_s[0].w * p.y + g_s[0].z * z1 + g_s[2].x + g_s[2].z;
    VSOut o;
    o.pos = float4(vx * g_s[3].x, vy * g_s[3].y,
                   vz * g_s[3].z + g_s[3].w, vz);
    o.uv = v.uv;
    return o;
}
)";
    static const char* kPs = R"(
Texture2D t : register(t0);
SamplerState sTex : register(s1);
cbuffer Mat : register(b1) { float4 g_mat; int g_hasTex; };
struct PSIn { float4 pos : SV_Position; float2 uv : TEXCOORD; };
float4 main(PSIn i) : SV_Target {
    float4 col = g_mat;
    if (g_hasTex != 0) {
        float2 uv = float2(i.uv.x, 1.0 - i.uv.y);
        col = t.Sample(sTex, uv);
        col.rgb *= g_mat.rgb;   // texel x material diffuse
        // cutout via alpha-to-coverage, distance-aware: mip generation
        // averages alpha, so foliage/caning edges go gray at range and the
        // 4-sample coverage dithers frame to frame while the camera moves.
        // Widening the sharpen slope with the sampled LOD keeps cutout
        // silhouettes binary where the mips have smeared them.
        float lod = t.CalculateLevelOfDetail(sTex, uv);
        col.a = saturate((col.a - 0.25f) * (2.0f + 0.5f * lod));
    }
    return col;
}
)";
    auto compile = [&](const char* src, const char* target,
                       ComPtr<ID3DBlob>& out) {
        ComPtr<ID3DBlob> err;
        const HRESULT hr =
            D3DCompile(src, std::strlen(src), nullptr, nullptr, nullptr, "main",
                       target, 0, 0, &out, &err);
        if (FAILED(hr)) {
            const char* m = err ? static_cast<const char*>(err->GetBufferPointer())
                                : "unknown error";
            AP_LOG_WARN("dx", "D3DCompile %s failed: %s", target, m);
            return false;
        }
        return true;
    };
    if (!compile(kVs, "vs_5_0", dx.vsBlob) || !compile(kPs, "ps_5_0", dx.psBlob))
        return false;

    // overlays + SRV table: [0]=white, [1..kMaxTex]=textures,
    // [kMaxTex+1]=HUD, [kMaxTex+2]=loading
    dx.white = dx.makeTex(1, 1, DXGI_FORMAT_R8G8B8A8_UNORM,
                          D3D12_RESOURCE_STATE_COPY_DEST);
    dx.hudTex = dx.makeTex(UINT(w), 80, DXGI_FORMAT_R8G8B8A8_UNORM,
                           D3D12_RESOURCE_STATE_COPY_DEST);
    dx.loadTex = dx.makeTex(UINT(w), UINT(h), DXGI_FORMAT_R8G8B8A8_UNORM,
                            D3D12_RESOURCE_STATE_COPY_DEST);
    const D3D12_CPU_DESCRIPTOR_HANDLE srvBase =
        dx.srvHeap->GetCPUDescriptorHandleForHeapStart();
    const D3D12_GPU_DESCRIPTOR_HANDLE srvGpuBase =
        dx.srvHeap->GetGPUDescriptorHandleForHeapStart();
    dx.srv(dx.white.Get(), srvBase);
    dx.whiteGpu = srvGpuBase;
    D3D12_CPU_DESCRIPTOR_HANDLE c = srvBase;
    D3D12_GPU_DESCRIPTOR_HANDLE g = srvGpuBase;
    c.ptr += (kMaxTex + 1) * SIZE_T(dx.srvSize);
    g.ptr += (kMaxTex + 1) * dx.srvSize;
    dx.srv(dx.hudTex.Get(), c);
    dx.hudGpu = g;
    c.ptr += dx.srvSize;
    g.ptr += dx.srvSize;
    dx.srv(dx.loadTex.Get(), c);
    dx.loadGpu = g;

    dx.readback = dx.makeBuf(UINT64(w) * UINT64(h) * 4,
                             D3D12_HEAP_TYPE_READBACK,
                             D3D12_RESOURCE_STATE_COPY_DEST);
    dx.vp = {0.f, 0.f, float(w), float(h), 0.f, 1.f};
    dx.scissor = {0, 0, LONG(w), LONG(h)};

    // DirectXTK: GraphicsMemory backs SpriteBatch's dynamic allocations;
    // SpriteBatch uploads its static buffers through the batch, so it is
    // constructed while the batch is open. Its PSO sample count must match
    // the MSAA render target the sprites draw into.
    dx.gfxMem = std::make_unique<GraphicsMemory>(dx.dev.Get());
    dx.upload = std::make_unique<ResourceUploadBatch>(dx.dev.Get());
    dx.upload->Begin();
    dx.uploadOpen = true;
    RenderTargetState rtState(DXGI_FORMAT_R8G8B8A8_UNORM,
                              DXGI_FORMAT_D32_FLOAT, kMsaaSamples, 0);
    const SpriteBatchPipelineStateDescription spso(rtState);
    dx.sprites =
        std::make_unique<SpriteBatch>(dx.dev.Get(), *dx.upload, spso, &dx.vp);

    // white 1x1 fallback (untextured meshes still bind a valid SRV)
    const unsigned char white[4] = {255, 255, 255, 255};
    const D3D12_SUBRESOURCE_DATA wd{white, 4, 4};
    dx.upload->Upload(dx.white.Get(), 0, &wd, 1);
    dx.upload->Transition(dx.white.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                          D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    dx.upload->End(dx.queue.Get()).wait();   // needed before the first frame
    dx.uploadOpen = false;

    up_ = true;
    AP_LOG("dx", "ready: %dx%d swapchain, %u back buffers", w, h, kBackBuffers);
    return true;
}

void DxRenderer::shutdown() {
    if (!up_ || !impl_) return;
    Impl& dx = *impl_;
    dx.drain();
    if (dx.uploadDone.valid())
        dx.uploadDone.wait();
    if (dx.uploadOpen) {
        dx.upload->End(dx.queue.Get()).wait();
        dx.uploadOpen = false;
    }
    dx.sprites.reset();
    dx.gfxMem.reset();
    dx.upload.reset();
    dx.pso.Reset();
    dx.rs.Reset();
    dx.vsBlob.Reset();
    dx.psBlob.Reset();
    dx.vbPos.Reset();
    dx.vbUv.Reset();
    dx.ib.Reset();
    dx.white.Reset();
    dx.hudTex.Reset();
    dx.loadTex.Reset();
    dx.readback.Reset();
    dx.texRes.clear();
    for (auto& b : dx.back) b.Reset();
    dx.msaaRT.Reset();
    dx.msaaDepth.Reset();
    dx.swap.Reset();
    dx.list.Reset();
    for (auto& a : dx.alloc) a.Reset();
    dx.queue.Reset();
    dx.fence.Reset();
    dx.rtvHeap.Reset();
    dx.dsvHeap.Reset();
    dx.srvHeap.Reset();
    dx.dev.Reset();
    impl_.reset();
    up_ = false;
    psoUp_ = false;
    texUploaded_ = 0;
}

void DxRenderer::setGeometry(const float* pos, size_t vertCount, const float* uv,
                             const uint32_t* idx, size_t idxCount,
                             bool frontCCW) {
    if (!up_ || psoUp_) return;
    Impl& dx = *impl_;

    D3D12_INPUT_ELEMENT_DESC layout[2] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 1, 0,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
    };
    D3D12_RASTERIZER_DESC rast{};        // zero-init = D3D12 default states
    rast.FillMode = D3D12_FILL_MODE_SOLID;
    rast.CullMode = D3D12_CULL_MODE_BACK;
    rast.FrontCounterClockwise = frontCCW;
    rast.DepthClipEnable = TRUE;
    D3D12_DEPTH_STENCIL_DESC dss{};
    dss.DepthEnable = TRUE;
    dss.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    dss.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
    D3D12_BLEND_DESC blend{};
    blend.RenderTarget[0].RenderTargetWriteMask =
        D3D12_COLOR_WRITE_ENABLE_ALL;
    // alpha-to-coverage instead of a PS discard: foliage cutout edges get
    // real MSAA coverage (1-3 of 4 samples) instead of one-bit jaggies
    blend.AlphaToCoverageEnable = TRUE;
    D3D12_GRAPHICS_PIPELINE_STATE_DESC pd{};
    pd.pRootSignature = dx.rs.Get();
    pd.VS = {dx.vsBlob->GetBufferPointer(), dx.vsBlob->GetBufferSize()};
    pd.PS = {dx.psBlob->GetBufferPointer(), dx.psBlob->GetBufferSize()};
    pd.InputLayout = {layout, 2};
    pd.RasterizerState = rast;
    pd.DepthStencilState = dss;
    pd.BlendState = blend;
    pd.SampleMask = UINT_MAX;
    pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pd.NumRenderTargets = 1;
    pd.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    pd.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    pd.SampleDesc = {kMsaaSamples, 0};
    if (FAILED(dx.dev->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&dx.pso)))) {
        AP_LOG_WARN("dx", "scene PSO failed");
        return;
    }

    // the parser pools upload verbatim: positions (3f), texcoords (2f) and
    // the packed index pool are already GPU layouts - no interleave, no
    // concatenation. Models without texcoords get a zero uv buffer.
    const UINT64 posBytes = UINT64(vertCount) * 3 * sizeof(float);
    const UINT64 uvBytes = UINT64(vertCount) * 2 * sizeof(float);
    const UINT64 ibBytes = UINT64(idxCount) * sizeof(uint32_t);
    dx.vbPos = dx.makeBuf(posBytes, D3D12_HEAP_TYPE_DEFAULT,
                          D3D12_RESOURCE_STATE_COPY_DEST);
    dx.vbUv = dx.makeBuf(uvBytes, D3D12_HEAP_TYPE_DEFAULT,
                         D3D12_RESOURCE_STATE_COPY_DEST);
    dx.ib = dx.makeBuf(ibBytes, D3D12_HEAP_TYPE_DEFAULT,
                       D3D12_RESOURCE_STATE_COPY_DEST);
    std::vector<float> uvZero;
    if (!uv) {
        uvZero.assign(vertCount * 2, 0.f);
        uv = uvZero.data();
    }

    if (dx.uploadDone.valid())
        dx.uploadDone.wait();
    dx.upload->Begin();
    dx.uploadOpen = true;
    const D3D12_SUBRESOURCE_DATA pd_{pos, LONG_PTR(posBytes),
                                     LONG_PTR(posBytes)};
    const D3D12_SUBRESOURCE_DATA ud{uv, LONG_PTR(uvBytes), LONG_PTR(uvBytes)};
    const D3D12_SUBRESOURCE_DATA id{idx, LONG_PTR(ibBytes), LONG_PTR(ibBytes)};
    dx.upload->Upload(dx.vbPos.Get(), 0, &pd_, 1);
    dx.upload->Upload(dx.vbUv.Get(), 0, &ud, 1);
    dx.upload->Upload(dx.ib.Get(), 0, &id, 1);
    dx.upload->Transition(dx.vbPos.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                          D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
    dx.upload->Transition(dx.vbUv.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                          D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
    dx.upload->Transition(dx.ib.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                          D3D12_RESOURCE_STATE_INDEX_BUFFER);
    dx.uploadDone = dx.upload->End(dx.queue.Get());   // awaited next frame
    dx.uploadOpen = false;

    dx.vbvPos = {dx.vbPos->GetGPUVirtualAddress(), UINT(posBytes),
                 sizeof(float) * 3};
    dx.vbvUv = {dx.vbUv->GetGPUVirtualAddress(), UINT(uvBytes),
                sizeof(float) * 2};
    dx.ibv = {dx.ib->GetGPUVirtualAddress(), UINT(ibBytes),
              DXGI_FORMAT_R32_UINT};

    psoUp_ = true;
    AP_LOG("dx", "geometry staged: %zu verts, %zu indices (zero-copy pools)",
           vertCount, idxCount);
}

void DxRenderer::drawLoading(const uint32_t* argb) {
    if (!up_) return;
    Impl& dx = *impl_;
    const auto t0 = std::chrono::steady_clock::now();
    const UINT f = dx.beginFrame();

    if (argb)
        dx.uploadOverlay(dx.loadTex, dx.loadReady, argb, dx.winW, dx.winH);
    dx.sprites->Begin(dx.list.Get());
    dx.sprites->Draw(dx.loadGpu, XMUINT2(UINT(dx.winW), UINT(dx.winH)),
                     XMFLOAT2(0.f, 0.f));
    dx.sprites->End();

    dx.endFrame(f, nullptr);
    ++dx.frames;
    if (dx.frames % 60 == 0) {
        AP_LOG("dx", "loading frame: %.2f ms",
               std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0).count());
    }
}

void DxRenderer::drawScene(const float cam[16], std::span<const DrawItem> items,
                           const texp::DecodedTex* texs, size_t texTotal,
                           size_t texBudget, const unsigned char* hudRgba,
                           const char* shotPath) {
    if (!up_ || !psoUp_) return;
    Impl& dx = *impl_;
    const auto t0 = std::chrono::steady_clock::now();
    const UINT f = dx.beginFrame();

    // chunked texture uploads: a bounded number per frame so the moment
    // textures finish decoding never becomes a single-frame hitch. The
    // decoded RGBA bytes upload verbatim (no swizzle round trip).
    if (texUploaded_ < texTotal) {
        if (dx.texRes.size() < texTotal) {
            dx.texRes.resize(texTotal);
            dx.texGpu.resize(texTotal);
        }
        const D3D12_CPU_DESCRIPTOR_HANDLE srvBase =
            dx.srvHeap->GetCPUDescriptorHandleForHeapStart();
        const D3D12_GPU_DESCRIPTOR_HANDLE srvGpuBase =
            dx.srvHeap->GetGPUDescriptorHandleForHeapStart();
        size_t n = 0;
        while (texUploaded_ + n < texTotal && n < texBudget) {
            const size_t i = texUploaded_ + n;
            const texp::DecodedTex& t = texs[i];
            // full mip chain + UAV access: GenerateMips (compute shader)
            // fills the lower levels after the base upload, so minified
            // fragments sample a fitting mip instead of thrashing the
            // texture cache with full-resolution texels
            UINT mips = 1;
            for (UINT s = std::max(t.w, t.h); s > 1; s >>= 1) ++mips;
            dx.texRes[i] =
                dx.makeTex(UINT(t.w), UINT(t.h), DXGI_FORMAT_R8G8B8A8_UNORM,
                           D3D12_RESOURCE_STATE_COPY_DEST, mips,
                           D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
            const D3D12_SUBRESOURCE_DATA sd{
                t.rgba.get(), LONG_PTR(size_t(t.w)) * 4,
                LONG_PTR(size_t(t.w)) * size_t(t.h) * 4};
            dx.upload->Upload(dx.texRes[i].Get(), 0, &sd, 1);
            dx.upload->Transition(dx.texRes[i].Get(),
                                  D3D12_RESOURCE_STATE_COPY_DEST,
                                  D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            dx.upload->GenerateMips(dx.texRes[i].Get());
            D3D12_CPU_DESCRIPTOR_HANDLE c = srvBase;
            D3D12_GPU_DESCRIPTOR_HANDLE g = srvGpuBase;
            c.ptr += (i + 1) * SIZE_T(dx.srvSize);
            g.ptr += (i + 1) * dx.srvSize;
            dx.srv(dx.texRes[i].Get(), c);
            dx.texGpu[i] = g;
            ++n;
        }
        texUploaded_ += n;
        if (texUploaded_ == texTotal)
            AP_LOG("dx", "textures uploaded: %zu", texUploaded_);
    }

    // scene
    dx.list->SetGraphicsRoot32BitConstants(0, 16, cam, 0);
    dx.list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    const D3D12_VERTEX_BUFFER_VIEW vbvs[2] = {dx.vbvPos, dx.vbvUv};
    dx.list->IASetVertexBuffers(0, 2, vbvs);
    dx.list->IASetIndexBuffer(&dx.ibv);
    dx.list->SetPipelineState(dx.pso.Get());
    for (const DrawItem& it : items) {
        if (!it.indexCount) continue;
        D3D12_GPU_DESCRIPTOR_HANDLE texGpu = dx.whiteGpu;
        int hasTex = 0;
        if (it.texSlot >= 0 && size_t(it.texSlot) < texUploaded_) {
            texGpu = dx.texGpu[size_t(it.texSlot)];
            hasTex = 1;
        }
        const uint32_t c = it.color;
        const float mats[5] = {
            float((c >> 16) & 255) / 255.f,
            float((c >> 8) & 255) / 255.f,
            float(c & 255) / 255.f,
            1.f,
            float(hasTex),
        };
        dx.list->SetGraphicsRootDescriptorTable(2, texGpu);
        dx.list->SetGraphicsRoot32BitConstants(1, 5, mats, 0);
        dx.list->DrawIndexedInstanced(it.indexCount, 1, it.startIndex, 0, 0);
    }

    // HUD strip (native 960x80, drawn 1:1 - no vertical stretch); the
    // viewer only passes pixels when the text changed, otherwise the SRV
    // is reused
    if (hudRgba) {
        if (dx.hudReady)
            dx.upload->Transition(dx.hudTex.Get(),
                                  D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                                  D3D12_RESOURCE_STATE_COPY_DEST);
        const D3D12_SUBRESOURCE_DATA hd{hudRgba,
                                        LONG_PTR(size_t(dx.winW)) * 4,
                                        LONG_PTR(size_t(dx.winW)) * 80 * 4};
        dx.upload->Upload(dx.hudTex.Get(), 0, &hd, 1);
        dx.upload->Transition(dx.hudTex.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                              D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        dx.hudReady = true;
    }
    if (dx.hudReady) {
        dx.sprites->Begin(dx.list.Get());
        dx.sprites->Draw(dx.hudGpu, XMUINT2(UINT(dx.winW), 80),
                         XMFLOAT2(0.f, 0.f));
        dx.sprites->End();
    }

    dx.endFrame(f, shotPath);
    ++dx.frames;
    if (dx.frames % 60 == 0) {
        AP_LOG("dx", "dx12 frame: %.2f ms",
               std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0).count());
    }
}
