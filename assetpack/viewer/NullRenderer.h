#pragma once
// ============================================================
// NullRenderer - headless IRenderer: every call is a no-op. Lets the
// viewer run (--frames N smoke loops) and the factory be exercised
// on machines without a GPU/D3D12 device.
// ============================================================

#include "IRenderer.h"

class NullRenderer final : public IRenderer {
public:
    bool init(SDL_Window*, int, int, bool) override { return true; }
    void shutdown() override {}
    void drawLoading(const uint32_t*) override {}

    void setGeometry(const float*, size_t, const float*, const uint32_t*,
                     size_t, bool) override { geometry_ = true; }

    void beginGeometryStream(size_t, size_t, bool) override {}
    void stageGeometryRange(ap::GeoRangeKind, size_t, const void*,
                            size_t) override {}
    void finalizeGeometryStream(bool) override { geometry_ = true; }

    void drawScene(const float cam[16], std::span<const DrawItem> items,
                   const texp::DecodedTex* texs, size_t texTotal,
                   size_t texBudget, const char* shotPath) override {
        (void)cam; (void)items; (void)texs;
        (void)texTotal; (void)texBudget; (void)shotPath;
    }

    bool ready() const override { return true; }
    bool geometryReady() const override { return geometry_; }
    size_t texturesUploaded() const override { return 0; }

    void gpuTiming(float& frameMs, float& sceneMs,
                   float& overlayMs) const override {
        frameMs = sceneMs = overlayMs = 0.f;
    }
    void cpuStats(float& waitMs, float& recordMs) const override {
        waitMs = recordMs = 0.f;
    }

private:
    bool geometry_ = false;
};
