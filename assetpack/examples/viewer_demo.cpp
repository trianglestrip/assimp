// assetpack_viewer - bind assetpack parse events to the olive.c software
// renderer.
//
// Usage:  assetpack_viewer [model.obj] [--frames N] [--tris N] [--wait]
//                          [--shot N file.bmp]
//
// With no model argument the default scene (San Miguel) is loaded, so the
// built viewer opens a window immediately and streams the model in as the
// parser stages complete: vertices first (meshes appear, default colors),
// then materials (diffuse colors), then textures (decoded in parallel with
// stb_image and applied via uv sampling).
// By default every triangle is drawn (no sampling); --tris N caps the
// per-frame triangle budget (stride-sampled), Up/Down adjust it live.
//
// Keys: Esc quit | Space pause rotation | Up/Down triangle budget x2 / /2
// Mouse: drag = orbit | wheel = zoom

#include "olive_bridge.h"

#include <assetpack/AssetPack.h>
#include <assetpack/Log.h>

#include <SDL2/SDL.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kWinW = 960;
constexpr int kWinH = 720;
constexpr int kThreads = 16;
constexpr const char* kDefaultModel =
    "F:/project/meshToBrowser/models/San_Miguel/san-miguel.obj";

constexpr uint32_t kBg     = 0xFF141414;   // 0xAARRGGBB
constexpr uint32_t kSlot   = 0xFF2E2E2E;   // loading bar slot
constexpr uint32_t kBar    = 0xFF2E8B57;   // loading bar fill
constexpr uint32_t kText   = 0xFF9A9A9A;
constexpr uint32_t kColor0 = 0xFF8888A8;   // default mesh color (before materials)

// ---- window / framebuffers ----
SDL_Window*   g_win = nullptr;
SDL_Renderer* g_renderer = nullptr;
SDL_Texture*  g_screenTex = nullptr;
std::vector<uint32_t> g_px;                // presented framebuffer (ARGB)
std::vector<uint32_t> g_tpx[kThreads];     // per-raster-thread pixel buffers
std::vector<float>    g_tzb[kThreads];     // per-raster-thread depth buffers

// ---- parser event data (spans point into parser-owned pools) ----
std::vector<ap::PackMesh> g_meshes;
std::vector<uint32_t> g_matColor;          // per-mesh diffuse color (ARGB)
std::vector<uint64_t> g_meshTriStart;      // prefix sums of mesh triangle counts
std::atomic<uint64_t> g_texBytes{0};
std::atomic<int>      g_texFiles{0};
std::atomic<float>    g_progress{0.f};     // 0..100
std::atomic<bool>     g_vertsReady{false};
std::atomic<bool>     g_allDone{false};
double g_importMs = 0, g_totalMs = 0;

// ---- materials / textures ----
struct ViewerTex {                        // decoded diffuse texture
    int w = 0, h = 0;
    std::vector<uint32_t> px;             // ARGB, row 0 = image top
};
std::vector<ap::PackMaterial> g_materials;       // materials snapshot (string_views stay valid)
std::vector<ViewerTex> g_texs;                   // decoded diffuse textures
std::unordered_map<std::string_view, int> g_texByPath;  // mtl path -> g_texs index
std::vector<int> g_meshTex;                      // per-mesh texture index or -1
std::atomic<uint64_t> g_texPixels{0};            // decoded pixel count

// ---- scene / camera ----
float g_scale = 1.f;
float g_center[3] = {0, 0, 0};
float g_camDist = 2.2f;
float g_rotY = 0.6f, g_pitch = 0.35f;
std::atomic<bool> g_paused{false};
std::atomic<bool> g_quit{false};
bool g_dragging = false;                   // left-button orbit in progress
int g_mouseX = 0, g_mouseY = 0;
int g_triBudget = 0;                       // triangles/frame, 0 = draw all (--tris)
int g_lastStride = 1;
bool g_cullChecked = false;                // winding probe (set once, first frame)
bool g_cullFrontZ = true;                  // true: CCW front faces have +z winding

// transformed vertex pool for the current frame: 6 floats per vertex
// (x,y,z + nx,ny,nz in view space); all meshes share one pool (OBJ indices
// are pool-relative), so the whole pool is transformed once per frame
std::vector<float> g_tv;

// ---- runtime flags ----
int g_frames = 0;                          // 0 = until quit
bool g_waitAll = false;                    // count frames only after all-done
int g_shotAt = 0;
std::string g_shotFile;
std::string g_modelName;
uint64_t g_frameCount = 0;
Clock::time_point g_tLoad;

// ============================================================
// utils
// ============================================================

static std::string timestamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &tm);
    return buf;
}

static void benchAppend(const std::string& line) {
    std::ofstream f("benchmark.md", std::ios::app);
    if (f) f << line << '\n';
}

static double secsSince(Clock::time_point t0) {
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

static std::string msStr(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

// ============================================================
// rasterization: per-thread local buffers, depth-merged on main thread
// ============================================================

struct ShadeV {                   // one projected vertex
    float x, y;                   // screen space
    float z;                      // view space depth (for the z-buffer)
    float w;                      // 1/z, interpolated perspective-correctly
    uint32_t c;                   // lit color (0xAARRGGBB)
    float u, v;                   // texture coordinates (textured meshes)
};

static uint32_t shadeColor(uint32_t rgb, float s) {
    const uint32_t r = uint32_t(((rgb >> 16) & 255) * s + 0.5f);
    const uint32_t g = uint32_t(((rgb >> 8) & 255) * s + 0.5f);
    const uint32_t b = uint32_t((rgb & 255) * s + 0.5f);
    return 0xFF000000u | (r << 16) | (g << 8) | b;
}

static float shadeDot(float nx, float ny, float nz) {
    // key light from above-front (normalized -0.35, 0.65, 0.8), 25% ambient
    constexpr float lx = -0.32f, ly = 0.60f, lz = 0.735f;
    const float d = nx * lx + ny * ly + nz * lz;
    const float s = (d < 0.f ? 0.f : (d > 1.f ? 1.f : d)) * 0.75f + 0.25f;
    return s;
}

static void rasterTri(uint32_t* px, float* zb, int W, int H,
                      const ShadeV& a, const ShadeV& b, const ShadeV& c,
                      const ViewerTex* tex) {
    int lx, hx, ly, hy;
    if (!obv_normalize_triangle(size_t(W), size_t(H), int(a.x), int(a.y),
                                int(b.x), int(b.y), int(c.x), int(c.y),
                                &lx, &hx, &ly, &hy))
        return;
    const int x0 = int(a.x), y0 = int(a.y), x1 = int(b.x), y1 = int(b.y),
              x2 = int(c.x), y2 = int(c.y);
    for (int y = ly; y <= hy; ++y) {
        for (int x = lx; x <= hx; ++x) {
            int u1, u2, det;
            if (!obv_barycentric(x0, y0, x1, y1, x2, y2, x, y, &u1, &u2, &det))
                continue;
            const float l1 = float(u1) / float(det);
            const float l2 = float(u2) / float(det);
            const float l0 = 1.f - l1 - l2;
            const float w = l0 * a.w + l1 * b.w + l2 * c.w;
            if (w <= 0.f) continue;
            const float z = 1.f / w;
            const size_t i = size_t(y) * size_t(W) + size_t(x);
            if (z >= zb[i]) continue;
            // perspective-correct attribute blending
            const float wa = l0 * a.w, wb = l1 * b.w, wc = l2 * c.w;
            float r = (wa * ((a.c >> 16) & 255) + wb * ((b.c >> 16) & 255)
                       + wc * ((c.c >> 16) & 255)) / w;
            float g = (wa * ((a.c >> 8) & 255) + wb * ((b.c >> 8) & 255)
                       + wc * ((c.c >> 8) & 255)) / w;
            float bl = (wa * (a.c & 255) + wb * (b.c & 255)
                        + wc * (c.c & 255)) / w;
            if (tex) {
                // sample the diffuse texture and modulate by the light
                const float u = (wa * a.u + wb * b.u + wc * c.u) / w;
                const float v = (wa * a.v + wb * b.v + wc * c.v) / w;
                int tx = int(u * tex->w);
                int ty = int((1.f - v) * tex->h);
                if (tx < 0) tx = 0;
                else if (tx >= tex->w) tx = tex->w - 1;
                if (ty < 0) ty = 0;
                else if (ty >= tex->h) ty = tex->h - 1;
                const uint32_t tcol =
                    tex->px[size_t(ty) * size_t(tex->w) + size_t(tx)];
                r *= float((tcol >> 16) & 255) / 255.f;
                g *= float((tcol >> 8) & 255) / 255.f;
                bl *= float(tcol & 255) / 255.f;
            }
            px[i] = 0xFF000000u
                  | (uint32_t(r + 0.5f) << 16)
                  | (uint32_t(g + 0.5f) << 8)
                  | uint32_t(bl + 0.5f);
            zb[i] = z;
        }
    }
}

// Transform + shade + rasterize the global triangle range [t0, t1) into
// one thread's local buffers. No shared state, so no races.
static void rasterRange(uint32_t* px, float* zb, uint64_t t0, uint64_t t1,
                        int stride) {
    if (t0 >= t1) return;
    const float* tv = g_tv.data();
    constexpr float f = float(kWinH) * 0.5f / 0.5773503f;  // fov 60 deg

    size_t mi = 0;                       // mesh owning t0 (prefix sums)
    while (mi + 1 < g_meshTriStart.size() && g_meshTriStart[mi + 1] <= t0) ++mi;

    uint64_t t = t0;
    while (t < t1) {
        const uint64_t mEnd = g_meshTriStart[mi + 1];
        if (t >= mEnd) { ++mi; continue; }
        const ap::PackMesh& m = g_meshes[mi];
        const uint32_t mat = g_matColor[mi];
        const bool hasNrm = !m.normals.empty();
        const int texIdx = g_meshTex[mi];
        const ViewerTex* tex =
            texIdx >= 0 && !m.texcoords.empty() ? &g_texs[size_t(texIdx)]
                                                : nullptr;
        const float* tc = tex ? m.texcoords.data() : nullptr;
        const uint32_t* idx = m.indices.data();
        const uint64_t base = g_meshTriStart[mi];
        const uint64_t stop = t1 < mEnd ? t1 : mEnd;

        for (; t < stop; t += stride) {
            const size_t ii = size_t(t - base) * 3;
            const uint32_t ia = idx[ii], ib = idx[ii + 1], ic = idx[ii + 2];
            const float* A = tv + size_t(ia) * 6;
            const float* B = tv + size_t(ib) * 6;
            const float* C = tv + size_t(ic) * 6;
            const float ax = A[0], ay = A[1], az = A[2];
            const float bx = B[0], by = B[1], bz = B[2];
            const float cx = C[0], cy = C[1], cz = C[2];
            // backface cull: keep faces whose view-space winding matches the
            // model's front side (probed once on the first frame)
            const float fnz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            if (g_cullFrontZ ? fnz <= 0.f : fnz >= 0.f) continue;
            const float iax = 1.f / az, ibx = 1.f / bz, icx = 1.f / cz;
            ShadeV sA, sB, sC;
            sA.x = float(kWinW) * 0.5f + f * ax * iax;
            sA.y = float(kWinH) * 0.5f - f * ay * iax;
            sA.z = az;
            sA.w = iax;
            sB.x = float(kWinW) * 0.5f + f * bx * ibx;
            sB.y = float(kWinH) * 0.5f - f * by * ibx;
            sB.z = bz;
            sB.w = ibx;
            sC.x = float(kWinW) * 0.5f + f * cx * icx;
            sC.y = float(kWinH) * 0.5f - f * cy * icx;
            sC.z = cz;
            sC.w = icx;
            float shA, shB, shC;
            if (hasNrm) {
                shA = shadeDot(A[3], A[4], A[5]);
                shB = shadeDot(B[3], B[4], B[5]);
                shC = shadeDot(C[3], C[4], C[5]);
            } else {
                // face normal (cross product of the edges)
                const float nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
                const float ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
                const float nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
                const float inv =
                    1.f / std::sqrt(nx * nx + ny * ny + nz * nz);
                const float sh = shadeDot(nx * inv, ny * inv, nz * inv);
                shA = shB = shC = sh;
            }
            sA.c = shadeColor(mat, shA);
            sB.c = shadeColor(mat, shB);
            sC.c = shadeColor(mat, shC);
            if (tc) {
                sA.u = tc[ia * 2];
                sA.v = tc[ia * 2 + 1];
                sB.u = tc[ib * 2];
                sB.v = tc[ib * 2 + 1];
                sC.u = tc[ic * 2];
                sC.v = tc[ic * 2 + 1];
            }
            rasterTri(px, zb, kWinW, kWinH, sA, sB, sC, tex);
        }
    }
}

// ============================================================
// frame drawing
// ============================================================

static void drawLoading() {
    std::fill(g_px.begin(), g_px.end(), kBg);
    const Obv_Canvas c = obv_canvas(g_px.data(), kWinW, kWinH, kWinW);
    const int bw = int(kWinW * 0.6f), bh = 12;
    const int bx = (kWinW - bw) / 2, by = kWinH - 130;
    obv_rect(c, bx, by, bw, bh, kSlot);
    const float pct = g_progress.load();
    if (pct > 0.f)
        obv_rect(c, bx + 2, by + 2, int((bw - 4) * pct / 100.f), bh - 4, kBar);
    char txt[192];
    std::snprintf(txt, sizeof txt, "Loading %s  %.0f%%", g_modelName.c_str(), pct);
    const int tw = 6 * 2 * int(std::strlen(txt));   // 6px glyph, scale 2
    obv_text(c, txt, (kWinW - tw) / 2, by - 40, obv_default_font(), 2, kText);
}

static void drawFrame(double fpsNow) {
    if (!g_vertsReady.load()) {
        drawLoading();
        return;
    }

    const size_t nVerts = g_tv.size() / 6;
    const uint64_t nTris = g_meshTriStart.back();
    const int stride = (g_triBudget > 0 && nTris > uint64_t(g_triBudget))
                           ? int(nTris / uint64_t(g_triBudget))
                           : 1;
    g_lastStride = stride;

    // 1) rotate/translate the shared vertex pool (parallel)
    const float cY = std::cos(g_rotY), sY = std::sin(g_rotY);
    const float cP = std::cos(g_pitch), sP = std::sin(g_pitch);
    const float s = g_scale, cx = g_center[0], cy = g_center[1], cz = g_center[2];
    const float dist = g_camDist;
    const float* src = g_meshes[0].positions.data();
    const float* nrm = g_meshes[0].normals.empty()
                           ? nullptr
                           : g_meshes[0].normals.data();
    std::vector<std::thread> pool;
    pool.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        const size_t b = nVerts * size_t(t) / kThreads;
        const size_t e = nVerts * (size_t(t) + 1) / kThreads;
        pool.emplace_back([=]() {
            for (size_t i = b; i < e; ++i) {
                const float px = (src[3 * i] - cx) * s;
                const float py = (src[3 * i + 1] - cy) * s;
                const float pz = (src[3 * i + 2] - cz) * s;
                const float x1 = cY * px + sY * pz;
                const float z1 = -sY * px + cY * pz;
                float* d = g_tv.data() + 6 * i;
                d[0] = x1;
                d[1] = cP * py - sP * z1;
                d[2] = sP * py + cP * z1 + dist;
                if (nrm) {
                    const float nx = nrm[3 * i], ny = nrm[3 * i + 1], nz = nrm[3 * i + 2];
                    const float nx1 = cY * nx + sY * nz;
                    const float nz1 = -sY * nx + cY * nz;
                    d[3] = nx1;
                    d[4] = cP * ny - sP * nz1;
                    d[5] = sP * ny + cP * nz1;
                } else {
                    d[3] = 0.f;
                    d[4] = 0.f;
                    d[5] = 0.f;
                }
            }
        });
    }
    for (auto& th : pool) th.join();

    // winding probe: models may be CW or CCW; sample the first triangles
    // in view space once and cull the back side accordingly
    if (!g_cullChecked) {
        int pos = 0, neg = 0;
        uint64_t s = 0;
        for (size_t mi = 0; mi < g_meshes.size() && s < 4096; ++mi) {
            const ap::PackMesh& m = g_meshes[mi];
            const uint32_t* idx = m.indices.data();
            const uint64_t n = std::min<uint64_t>(m.triangleCount(), 4096 - s);
            for (uint64_t t = 0; t < n; ++t) {
                const float* A = g_tv.data() + size_t(idx[3 * t]) * 6;
                const float* B = g_tv.data() + size_t(idx[3 * t + 1]) * 6;
                const float* C = g_tv.data() + size_t(idx[3 * t + 2]) * 6;
                const float fnz = (B[0] - A[0]) * (C[1] - A[1])
                                - (B[1] - A[1]) * (C[0] - A[0]);
                fnz > 0.f ? ++pos : ++neg;
            }
            s += n;
        }
        g_cullFrontZ = pos >= neg;
        g_cullChecked = true;
        AP_LOG("viewer", "winding probe: %d ccw %d cw -> front=%s", pos, neg,
               g_cullFrontZ ? "+z" : "-z");
    }

    // 2) clear per-thread buffers and raster triangle ranges (parallel)
    for (int t = 0; t < kThreads; ++t) {
        const uint64_t b = nTris * uint64_t(t) / uint64_t(kThreads);
        const uint64_t e = nTris * (uint64_t(t) + 1) / uint64_t(kThreads);
        pool[size_t(t)] = std::thread([t, b, e, stride]() {
            std::fill(g_tpx[t].begin(), g_tpx[t].end(), kBg);
            std::fill(g_tzb[t].begin(), g_tzb[t].end(), INFINITY);
            rasterRange(g_tpx[t].data(), g_tzb[t].data(), b, e, stride);
        });
    }
    for (auto& th : pool) th.join();

    // 3) merge the depth-sorted thread buffers on the main thread
    for (size_t i = 0; i < g_px.size(); ++i) {
        float best = g_tzb[0][i];
        int bi = 0;
        for (int t = 1; t < kThreads; ++t)
            if (g_tzb[t][i] < best) { best = g_tzb[t][i]; bi = t; }
        g_px[i] = g_tpx[bi][i];
    }

    // 4) HUD
    const Obv_Canvas c = obv_canvas(g_px.data(), kWinW, kWinH, kWinW);
    char info[256];
    std::snprintf(info, sizeof info,
                  "%s | %llu tris | drawing %llu (x%d) | %.1f fps | Esc quit",
                  g_modelName.c_str(), (unsigned long long)nTris,
                  (unsigned long long)(nTris / uint64_t(stride)), stride,
                  fpsNow);
    obv_text(c, info, 10, 10, obv_default_font(), 2, kText);
    obv_text(c, "drag: orbit   wheel: zoom   Space: pause", 10, 30,
             obv_default_font(), 2, kText);
}

// ============================================================
// screenshot (24-bit BMP, bottom-up)
// ============================================================

static void saveBmp(const std::string& path) {
    const int rowBytes = (kWinW * 3 + 3) & ~3;
    const uint32_t imgBytes = uint32_t(rowBytes) * uint32_t(kWinH);
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
    put32(18, uint32_t(kWinW));
    put32(22, uint32_t(kWinH));
    put16(26, 1);
    put16(28, 24);
    put32(34, imgBytes);
    for (int y = 0; y < kWinH; ++y) {
        const uint32_t* row = g_px.data() + size_t(kWinH - 1 - y) * kWinW;
        uint8_t* dst = out.data() + 54 + size_t(y) * rowBytes;
        for (int x = 0; x < kWinW; ++x) {
            dst[3 * x] = uint8_t(row[x]);          // B
            dst[3 * x + 1] = uint8_t(row[x] >> 8); // G
            dst[3 * x + 2] = uint8_t(row[x] >> 16);// R
        }
    }
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(out.data()), std::streamsize(out.size()));
    AP_LOG("viewer", "saved %s", path.c_str());
}

// ============================================================
// parser events -> renderer state + benchmark lines
// ============================================================

// Bind each mesh's material diffuse texture (by mtl path) to a decoded
// texture slot. Idempotent: called from every event that may complete
// the materials/textures picture.
static void bindTextures() {
    if (g_materials.empty() || g_meshTex.empty() || g_texByPath.empty()) return;
    g_meshTex.assign(g_meshTex.size(), -1);
    for (size_t i = 0; i < g_meshes.size(); ++i) {
        const int mi = g_meshes[i].materialIndex;
        if (mi < 0 || size_t(mi) >= g_materials.size()) continue;
        const ap::PackMaterial& m = g_materials[size_t(mi)];
        for (const ap::PackTexRef& ref : m.textures) {
            if (ref.type != int(ap::TexType::TexDiffuse)) continue;
            const auto it = g_texByPath.find(ref.path);
            if (it != g_texByPath.end()) {
                g_meshTex[i] = it->second;
                break;
            }
        }
    }
    size_t bound = 0;
    for (int t : g_meshTex)
        if (t >= 0) ++bound;
    AP_LOG("viewer", "textures bound: %zu/%zu meshes", bound, g_meshTex.size());
}

static void bindEvents(ap::AssetPack& pack) {
    pack.setProgress([](float pct) { g_progress.store(pct); });

    pack.setOnVerticesReady(
        [](ap::PackResult& r, std::span<const ap::PackMesh> meshes) {
            g_meshes.assign(meshes.begin(), meshes.end());
            float mn[3] = {INFINITY, INFINITY, INFINITY};
            float mx[3] = {-INFINITY, -INFINITY, -INFINITY};
            for (const ap::PackMesh& m : g_meshes)
                for (int k = 0; k < 3; ++k) {
                    if (m.boundsMin[k] < mn[k]) mn[k] = m.boundsMin[k];
                    if (m.boundsMax[k] > mx[k]) mx[k] = m.boundsMax[k];
                }
            for (int k = 0; k < 3; ++k) g_center[k] = (mn[k] + mx[k]) * 0.5f;
            const float ex = (mx[0] - mn[0]) * 0.5f;
            const float ey = (mx[1] - mn[1]) * 0.5f;
            const float ez = (mx[2] - mn[2]) * 0.5f;
            g_scale = 1.f / std::max({ex, ey, ez, 1e-4f});
            g_meshTriStart.clear();
            g_meshTriStart.push_back(0);
            for (const ap::PackMesh& m : g_meshes)
                g_meshTriStart.push_back(g_meshTriStart.back() + m.triangleCount());
            g_matColor.assign(g_meshes.size(), kColor0);
            g_meshTex.assign(g_meshes.size(), -1);
            const size_t poolVerts = r.positions.size() / 3;
            g_tv.resize(poolVerts * 6);
            const uint64_t nTris = g_meshTriStart.back();
            AP_LOG("viewer", "vertices ready: %zu meshes, %zu verts, %llu tris",
                   g_meshes.size(), poolVerts, (unsigned long long)nTris);
            AP_LOG("viewer", "bounds min %.2f %.2f %.2f max %.2f %.2f %.2f scale %.4f",
                   mn[0], mn[1], mn[2], mx[0], mx[1], mx[2], g_scale);
            benchAppend("| " + timestamp() + " | [async] vertices-ready | "
                        + msStr(double(r.verticesMicros) / 1000.0)
                        + " ms | verts " + std::to_string(poolVerts)
                        + " tris " + std::to_string(nTris) + " |");
            bindTextures();
            g_vertsReady.store(true);
        });

    pack.setOnMaterialsReady(
        [](ap::PackResult& r, std::span<const ap::PackMaterial> mats) {
            g_materials.assign(mats.begin(), mats.end());
            for (size_t i = 0; i < g_meshes.size(); ++i) {
                const int mi = g_meshes[i].materialIndex;
                if (mi < 0 || size_t(mi) >= mats.size()) continue;
                const float* d = mats[size_t(mi)].diffuse;
                g_matColor[i] = 0xFF000000u
                    | (uint32_t(d[0] * 255.f + 0.5f) << 16)
                    | (uint32_t(d[1] * 255.f + 0.5f) << 8)
                    | uint32_t(d[2] * 255.f + 0.5f);
            }
            AP_LOG("viewer", "materials ready: %zu materials bound to %zu meshes",
                   mats.size(), g_meshes.size());
            benchAppend("| " + timestamp() + " | [async] materials-ready | "
                        + msStr(double(r.materialsMicros) / 1000.0)
                        + " ms | mats " + std::to_string(mats.size()) + " |");
            bindTextures();
        });

    pack.setOnTexturesReady(
        [](ap::PackResult& r, std::span<const ap::PackTexture> texs) {
            // parallel mmap + decode of the diffuse textures (stb_image)
            int ndiff = 0;
            for (const ap::PackTexture& t : texs)
                if (!t.embedded && !t.resolvedPath.empty() &&
                    t.type == int(ap::TexType::TexDiffuse))
                    ++ndiff;
            g_texs.clear();
            g_texs.resize(size_t(ndiff));
            g_texByPath.clear();
            g_texPixels = 0;
            std::vector<int> slotByPos(texs.size(), -1);
            std::atomic<size_t> next{0};
            std::atomic<int> slot{0};
            const unsigned n = std::max(1u, std::thread::hardware_concurrency());
            std::vector<std::thread> pool;
            pool.reserve(n);
            for (unsigned i = 0; i < n; ++i)
                pool.emplace_back([&]() {
                    for (;;) {
                        const size_t k = next.fetch_add(1);
                        if (k >= texs.size()) break;
                        const ap::PackTexture& t = texs[k];
                        if (t.embedded || t.resolvedPath.empty()) continue;
                        auto mf = ap::MappedFile::openShared(t.resolvedPath);
                        if (!mf) continue;
                        g_texBytes += mf->size();
                        ++g_texFiles;
                        if (t.type != int(ap::TexType::TexDiffuse)) continue;
                        int w = 0, h = 0, ch = 0;
                        unsigned char* rgba = stbi_load_from_memory(
                            reinterpret_cast<const unsigned char*>(
                                mf->bytes().data()),
                            int(mf->size()), &w, &h, &ch, 4);
                        if (!rgba) {
                            AP_LOG_WARN("viewer", "decode failed: %.*s",
                                        int(t.path.size()), t.path.data());
                            continue;
                        }
                        ViewerTex vt;
                        vt.w = w;
                        vt.h = h;
                        vt.px.resize(size_t(w) * size_t(h));
                        for (size_t p = 0; p < vt.px.size(); ++p) {
                            const unsigned char* s = rgba + p * 4;
                            vt.px[p] = 0xFF000000u
                                     | (uint32_t(s[0]) << 16)
                                     | (uint32_t(s[1]) << 8)
                                     | uint32_t(s[2]);
                        }
                        stbi_image_free(rgba);
                        const int si = slot.fetch_add(1);
                        slotByPos[k] = si;
                        g_texs[size_t(si)] = std::move(vt);
                        g_texPixels += size_t(w) * size_t(h);
                    }
                });
            for (auto& th : pool) th.join();
            for (size_t k = 0; k < texs.size(); ++k)
                if (slotByPos[k] >= 0)
                    g_texByPath.emplace(texs[k].path, slotByPos[k]);
            AP_LOG("viewer",
                   "textures ready: %zu refs, %d mmap'd (%.1f MB), %d decoded (%llu px)",
                   texs.size(), g_texFiles.load(),
                   double(g_texBytes.load()) / 1048576.0, ndiff,
                   (unsigned long long)g_texPixels.load());
            benchAppend("| " + timestamp() + " | [async] textures-ready | "
                        + msStr(double(r.texturesMicros) / 1000.0)
                        + " ms | files " + std::to_string(g_texFiles.load())
                        + ", " + msStr(double(g_texBytes.load()) / 1048576.0)
                        + " MB, " + std::to_string(ndiff) + " decoded |");
            bindTextures();
        });

    pack.setOnAllDone([](ap::PackResult& r, bool ok, std::string_view err) {
        g_importMs = double(r.importMicros) / 1000.0;
        g_totalMs = double(r.totalMicros) / 1000.0;
        const double deltaMs = secsSince(g_tLoad) * 1000.0;
        g_allDone.store(true);
        if (!ok)
            AP_LOG_WARN("viewer", "parse failed: %.*s", int(err.size()),
                        err.data());
        AP_LOG("viewer", "all done in %.1f ms (import %.1f ms)", g_totalMs,
               g_importMs);
        benchAppend("| " + timestamp() + " | [async] all-done | "
                    + msStr(g_totalMs) + " ms | + " + msStr(deltaMs)
                    + " ms | import " + msStr(g_importMs) + " ms, total "
                    + msStr(g_totalMs) + " ms |");
    });
}

static void benchRenderLine(uint64_t frames, double fps) {
    const char* header =
        "## render\n"
        "| time | model | frames | avg fps | ms/frame | budget | stride |\n"
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |\n";
    std::ifstream in("benchmark.md", std::ios::binary);
    std::string tail;
    if (in) {
        in.seekg(0, std::ios::end);
        const std::streamoff len = in.tellg();
        in.seekg(len > 2048 ? len - 2048 : 0);   // keep the header in view
        tail.assign(std::istreambuf_iterator<char>(in),
                    std::istreambuf_iterator<char>());
    }
    std::ofstream out("benchmark.md", std::ios::app);
    if (tail.find("## render") == std::string::npos) out << "\n" << header;
    const double ms = frames > 0 ? 1000.0 / fps : 0.0;
    const std::string budget =
        g_triBudget <= 0 ? "ALL" : std::to_string(g_triBudget);
    char line[256];
    std::snprintf(line, sizeof line,
                  "| %s | %s | %llu | %.1f | %.1f | %s | %d |\n",
                  timestamp().c_str(), g_modelName.c_str(),
                  (unsigned long long)frames, fps, ms, budget.c_str(),
                  g_lastStride);
    out << line;
}

} // namespace

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    std::string model = kDefaultModel;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) g_frames = std::atoi(argv[++i]);
        else if (a == "--tris" && i + 1 < argc) g_triBudget = std::atoi(argv[++i]);
        else if (a == "--wait") g_waitAll = true;
        else if (a == "--shot" && i + 2 < argc) {
            g_shotAt = std::atoi(argv[i + 1]);
            g_shotFile = argv[i + 2];
            i += 2;
        } else if (a.rfind("--", 0) != 0) {
            model = a;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return 2;
        }
    }
    g_modelName = model.substr(model.find_last_of("/\\") + 1);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    g_win = SDL_CreateWindow(("assetpack viewer - " + g_modelName).c_str(),
                             SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             kWinW, kWinH, 0);
    if (!g_win) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    g_renderer = SDL_CreateRenderer(g_win, -1, 0);
    g_screenTex = SDL_CreateTexture(g_renderer, SDL_PIXELFORMAT_ARGB8888,
                                    SDL_TEXTUREACCESS_STREAMING, kWinW, kWinH);
    g_px.resize(size_t(kWinW) * size_t(kWinH));
    for (int t = 0; t < kThreads; ++t) {
        g_tpx[t].resize(size_t(kWinW) * size_t(kWinH));
        g_tzb[t].resize(size_t(kWinW) * size_t(kWinH));
    }

    AP_LOG("viewer", "window ready, loading %s async", model.c_str());
    g_tLoad = Clock::now();
    {
        ap::AssetPack pack;              // outlives the frame loop; its
        bindEvents(pack);                // destructor drains the executor
        pack.loadAsync(model);

        std::optional<Clock::time_point> tAll;
        uint64_t framesAll = 0;
        double fpsNow = 0;
        while (!g_quit.load()) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) {
                    g_quit.store(true);
                } else if (e.type == SDL_KEYDOWN) {
                    switch (e.key.keysym.sym) {
                    case SDLK_ESCAPE: g_quit.store(true); break;
                    case SDLK_SPACE: g_paused.store(!g_paused.load()); break;
                    case SDLK_UP:   // raise the budget; 0 = draw all
                        g_triBudget = g_triBudget <= 0
                                          ? 100000
                                          : std::min(g_triBudget * 2, 4000000);
                        break;
                    case SDLK_DOWN:
                        g_triBudget = std::max(g_triBudget / 2, 0);
                        break;
                    default: break;
                    }
                } else if (e.type == SDL_MOUSEBUTTONDOWN) {
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        g_dragging = true;
                        g_mouseX = e.button.x;
                        g_mouseY = e.button.y;
                    }
                } else if (e.type == SDL_MOUSEBUTTONUP) {
                    if (e.button.button == SDL_BUTTON_LEFT)
                        g_dragging = false;
                } else if (e.type == SDL_MOUSEMOTION) {
                    if (g_dragging) {
                        g_rotY -= float(e.motion.xrel) * 0.006f;
                        g_pitch += float(e.motion.yrel) * 0.006f;
                        if (g_pitch > 1.4f) g_pitch = 1.4f;
                        if (g_pitch < -1.4f) g_pitch = -1.4f;
                    }
                } else if (e.type == SDL_MOUSEWHEEL) {
                    g_camDist *= std::pow(0.9f, float(e.wheel.y));
                    if (g_camDist < 1.05f) g_camDist = 1.05f;
                    if (g_camDist > 10.0f) g_camDist = 10.0f;
                }
            }
            if (!g_paused.load() && !g_dragging) g_rotY += 0.004f;

            drawFrame(fpsNow);
            SDL_UpdateTexture(g_screenTex, nullptr, g_px.data(), kWinW * 4);
            SDL_RenderClear(g_renderer);
            SDL_RenderCopy(g_renderer, g_screenTex, nullptr, nullptr);
            SDL_RenderPresent(g_renderer);
            ++g_frameCount;

            if (g_allDone.load()) {
                if (!tAll) tAll = Clock::now();
                ++framesAll;
                fpsNow = framesAll / std::max(secsSince(*tAll), 1e-6);
            }
            const uint64_t limit = g_waitAll ? framesAll : g_frameCount;
            if (g_shotAt > 0 && limit == uint64_t(g_shotAt))
                saveBmp(g_shotFile);
            if (g_frames > 0 && (!g_waitAll || g_allDone.load()) &&
                limit >= uint64_t(g_frames))
                break;
        }
        benchRenderLine(framesAll, fpsNow);
        AP_LOG("viewer", "rendered %llu frames (%.1f fps)",
               (unsigned long long)framesAll, fpsNow);
    }

    SDL_DestroyTexture(g_screenTex);
    SDL_DestroyRenderer(g_renderer);
    SDL_DestroyWindow(g_win);
    SDL_Quit();
    AP_LOG("viewer", "bye");
    return 0;
}
