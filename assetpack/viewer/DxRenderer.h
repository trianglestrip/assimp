#pragma once
// ============================================================
// DxRenderer - native Direct3D 12 backend (the default viewer
// path). Owns the device, swapchain, scene PSO and the DirectXTK
// helpers (ResourceUploadBatch / GraphicsMemory / SpriteBatch).
// Implements the backend-agnostic IRenderer surface; created via
// RendererFactory ("dx12").
//
// Frame pacing (the part that used to serialize CPU against GPU):
//   - two in-flight frames: one command allocator per back buffer
//     and a per-frame fence value, so the CPU only waits for the
//     frame from two submissions ago instead of a full GPU idle
//   - the ResourceUploadBatch End() future is retained and awaited
//     at the next Begin(), letting uploads overlap scene execution
//   - HUD uploads carry a dirty flag; unchanged frames reuse the
//     existing SRV without touching the upload path
//
// Loading (the part that used to hitch on the first scene frame):
//   - geometry uploads straight from the parser pools (positions /
//     texcoords in their native 3f / 2f layouts as two VBOs, and
//     the whole packed index pool as one IBO) - no CPU-side
//     interleave or concatenation copies
//   - decoded textures upload in bounded per-frame chunks
// ============================================================

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <SDL2/SDL.h>

#include <assetpack/AssetPack.h>

#include "IRenderer.h"
#include "TexPipeline.h"

class DxRenderer final : public IRenderer {
public:
    DxRenderer();
    ~DxRenderer();

    DxRenderer(const DxRenderer&) = delete;
    DxRenderer& operator=(const DxRenderer&) = delete;

    bool init(SDL_Window* win, int w, int h, bool warp) override;
    void shutdown() override;

    // full-window software-rendered overlay (ARGB, row 0 = image top);
    // null pixels = unchanged (the existing texture is reused)
    void drawLoading(const uint32_t* argb) override;

    // stage the parser pools once (idempotent); PSO is built here so
    // shader compilation happens at init, not mid-frame
    void setGeometry(const float* pos, size_t vertCount, const float* uv,
                     const uint32_t* idx, size_t idxCount,
                     bool frontCCW) override;

    // streaming geometry upload (large models): begin allocates the
    // VB/IB and a staging ring and leaves a command list open;
    // stageGeometryRange copies one parser pool range into the ring
    // (called from the upload thread); finalize builds the PSO,
    // transitions the buffers, submits the copies and waits the GPU.
    // Uploads overlap the parse tail, and the ring caps staging at
    // ~256 MB instead of one full pool-sized copy.
    void beginGeometryStream(size_t verts, size_t tris, bool hasUv) override;
    void stageGeometryRange(ap::GeoRangeKind kind, size_t offsetBytes,
                            const void* data, size_t sizeBytes) override;
    void finalizeGeometryStream(bool frontCCW) override;

    // scene frame: camera constants (see the viewer for the layout),
    // one DrawItem per mesh, decoded textures (a bounded chunk uploads
    // per frame), optional screenshot path (captured this frame). The
    // ImGui overlay (built by the viewer) is drawn after the MSAA
    // resolve, inside this call.
    void drawScene(const float cam[16], std::span<const DrawItem> items,
                   const texp::DecodedTex* texs, size_t texTotal,
                   size_t texBudget, const char* shotPath) override;

    bool ready() const override { return up_; }
    bool geometryReady() const override { return psoUp_; }
    size_t texturesUploaded() const override { return texUploaded_; }

    // GPU frame timing from timestamp queries (ms, updated every frame):
    // frame = scene draws + MSAA resolve + ImGui overlay; scene = mesh
    // draws only. Zero until a couple of frames have elapsed.
    void gpuTiming(float& frameMs, float& sceneMs,
                   float& overlayMs) const override;

    // CPU-side per-frame timings (ms, updated every frame): wait = fence
    // wait at Begin() (compositor pacing shows up here), record = time to
    // record the scene draws into the command list
    void cpuStats(float& waitMs, float& recordMs) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool up_ = false;
    bool psoUp_ = false;
    size_t texUploaded_ = 0;
};
