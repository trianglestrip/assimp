#pragma once
// ============================================================
// IRenderer - backend-agnostic rendering interface for the viewer.
//
// The viewer drives everything through this surface; concrete
// backends implement it (DxRenderer = native D3D12, NullRenderer =
// headless no-op for CI/smoke runs). RendererFactory picks one by
// name (--renderer dx12|null).
// ============================================================

#include <cstddef>
#include <cstdint>
#include <span>

#include <SDL2/SDL.h>

#include <assetpack/AssetPack.h>

#include "TexPipeline.h"

class IRenderer {
public:
    // one draw call per mesh: an index range into the shared pool
    struct DrawItem {
        uint32_t indexCount = 0;
        uint32_t startIndex = 0;   // uint32 index offset into the IBO
        uint32_t color = 0xFF8888A8;   // 0xAARRGGBB material diffuse
        int texSlot = -1;              // TexPipeline slot or -1
        float world[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        bool hasWorld = false;
    };

    virtual ~IRenderer() = default;

    virtual bool init(SDL_Window* win, int w, int h, bool warp) = 0;
    virtual void shutdown() = 0;

    // full-window software-rendered overlay (ARGB, row 0 = image top);
    // null pixels = unchanged (the existing texture is reused)
    virtual void drawLoading(const uint32_t* argb) = 0;

    // stage the parser pools once (idempotent); PSO is built here so
    // shader compilation happens at init, not mid-frame
    virtual void setGeometry(const float* pos, size_t vertCount,
                             const float* uv, const uint32_t* idx,
                             size_t idxCount, bool frontCCW) = 0;

    // streaming geometry upload (large models): begin allocates the
    // VB/IB and a staging ring; stageGeometryRange copies one parser
    // pool range into the ring; finalize builds the PSO, transitions
    // the buffers, submits and waits the GPU.
    virtual void beginGeometryStream(size_t verts, size_t tris,
                                     bool hasUv) = 0;
    virtual void stageGeometryRange(ap::GeoRangeKind kind, size_t offsetBytes,
                                    const void* data,
                                    size_t sizeBytes) = 0;
    virtual void finalizeGeometryStream(bool frontCCW) = 0;

    // scene frame: camera constants (see the viewer for the layout),
    // one DrawItem per mesh, decoded textures (a bounded chunk uploads
    // per frame), optional screenshot path (captured this frame)
    virtual void drawScene(const float cam[16],
                           std::span<const DrawItem> items,
                           const texp::DecodedTex* texs, size_t texTotal,
                           size_t texBudget, const char* shotPath) = 0;

    virtual bool ready() const = 0;
    virtual bool geometryReady() const = 0;
    virtual size_t texturesUploaded() const = 0;

    // GPU frame timing from timestamp queries (ms): frame = scene +
    // resolve + overlay; scene = mesh draws only. Zeros until warm-up.
    virtual void gpuTiming(float& frameMs, float& sceneMs,
                           float& overlayMs) const = 0;

    // CPU-side per-frame timings (ms): wait = fence wait at Begin(),
    // record = command-list recording time
    virtual void cpuStats(float& waitMs, float& recordMs) const = 0;
};
