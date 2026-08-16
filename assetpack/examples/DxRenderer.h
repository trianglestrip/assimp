#pragma once
// ============================================================
// DxRenderer - native Direct3D 12 backend (the default viewer
// path). Owns the device, swapchain, scene PSO and the DirectXTK
// helpers (ResourceUploadBatch / GraphicsMemory / SpriteBatch).
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

#include "TexPipeline.h"

class DxRenderer {
public:
    // one draw call per mesh: an index range into the shared pool
    struct DrawItem {
        uint32_t indexCount = 0;
        uint32_t startIndex = 0;   // uint32 index offset into the IBO
        uint32_t color = 0xFF8888A8;   // 0xAARRGGBB material diffuse
        int texSlot = -1;              // TexPipeline slot or -1
    };

    DxRenderer();
    ~DxRenderer();

    DxRenderer(const DxRenderer&) = delete;
    DxRenderer& operator=(const DxRenderer&) = delete;

    bool init(SDL_Window* win, int w, int h, bool warp);
    void shutdown();

    // full-window software-rendered overlay (ARGB, row 0 = image top);
    // null pixels = unchanged (the existing texture is reused)
    void drawLoading(const uint32_t* argb);

    // stage the parser pools once (idempotent); PSO is built here so
    // shader compilation happens at init, not mid-frame
    void setGeometry(const float* pos, size_t vertCount, const float* uv,
                     const uint32_t* idx, size_t idxCount, bool frontCCW);

    // scene frame: camera constants (see the viewer for the layout),
    // one DrawItem per mesh, decoded textures (a bounded chunk uploads
    // per frame), optional HUD strip (w * 40 RGBA bytes, null =
    // unchanged), optional screenshot path (captured this frame)
    void drawScene(const float cam[16], std::span<const DrawItem> items,
                   const texp::DecodedTex* texs, size_t texTotal,
                   size_t texBudget, const unsigned char* hudRgba,
                   const char* shotPath);

    bool ready() const { return up_; }
    bool geometryReady() const { return psoUp_; }
    size_t texturesUploaded() const { return texUploaded_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool up_ = false;
    bool psoUp_ = false;
    size_t texUploaded_ = 0;
};
