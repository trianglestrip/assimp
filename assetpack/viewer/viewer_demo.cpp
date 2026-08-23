// assetpack_viewer - DX12-GPU-only viewer. Parser events feed texp::TexPipeline
// (TexPipeline.h), the parallel mmap + stb_image texture decode, and DxRenderer
// (DxRenderer.h), which owns the device, swapchain, scene PSO and the
// DirectXTK12 display layer. The loading overlay (progress bar + caption)
// rasterizes on a taskflow executor and uploads as a texture; the in-game
// stats overlay is Dear ImGui (third_party/imgui), drawn by DxRenderer after
// the MSAA resolve.
//
// Usage:  assetpack_viewer [model.obj] [--frames N] [--wait]
//                          [--shot N file.bmp] [--untex] [--warp]
//                          [--stat name] [--pix file.wpix [--pixStart N]]
// With no model argument the default scene (San Miguel) is loaded, so the
// built viewer opens a window immediately and streams the model in as the
// parser stages complete: vertices first (meshes appear, default colors),
// then materials (diffuse colors), then textures (decoded in parallel with
// stb_image and applied via uv sampling).
//
// Keys: Esc quit | T textures on/off (when the overlay doesn't take them)
// Mouse: drag = orbit | wheel = zoom
// WASD: move along the view axis / strafe | Q/E: camera height

#include <assetpack/AssetPack.h>
#include <assetpack/Config.h>

#include <SDL2/SDL.h>

#include "TexPipeline.h"     // parallel texture decode (mmap + stb_image)
#include "IRenderer.h"       // backend-agnostic render surface
#include "RendererFactory.h" // --renderer dx12|null

#include <windows.h>   // before pix3.h (PCWSTR/BOOL types)
#include <WinPixEventRuntime/pix3.h>   // programmatic GPU capture (--pix)

#include <imgui.h>
#include <imgui_impl_dx12.h>
#include <imgui_impl_sdl2.h>

#include <taskflow/taskflow.hpp>   // display-side CPU jobs run as tasks too

#define STB_TRUETYPE_IMPLEMENTATION
#include <stb/stb_truetype.h>      // TrueType HUD/loading text (Consolas)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kWinW = 960;
constexpr int kWinH = 720;
constexpr const char* kDefaultModel =
    "F:/project/meshToBrowser/models/San_Miguel/san-miguel.obj";

constexpr uint32_t kBg     = 0xFF141414;   // 0xAARRGGBB
constexpr uint32_t kSlot   = 0xFF2E2E2E;   // loading bar slot
constexpr uint32_t kBar    = 0xFF2E8B57;   // loading bar fill
constexpr uint32_t kText   = 0xFF9A9A9A;
constexpr uint32_t kColor0 = 0xFF8888A8;   // default mesh color (before materials)

// ---- window / loading overlay ----
SDL_Window* g_win = nullptr;

// display-side taskflow: loading overlay rasterization runs here so the
// render thread never pays the text rasterization or swizzle on its
// critical path (one worker pool for the viewer's whole lifetime)
tf::Executor g_tf;

// latest-completed handoff for the loading overlay: a task rasterizes
// into a private buffer and parks it here; the render loop consumes
// (moves) it, so stale results are dropped and the pointer stays owned
// through the GPU upload of that frame
std::mutex g_ovlMx;
std::shared_ptr<std::vector<uint32_t>> g_loadLatest;
std::atomic<bool> g_loadBusy{false};
int g_loadPct = -1;

// ---- parser event data (spans point into parser-owned pools) ----
std::vector<ap::PackMesh> g_meshes;
std::vector<uint32_t> g_matColor;          // per-mesh diffuse color (ARGB)
std::vector<uint64_t> g_meshTriStart;      // prefix sums of mesh triangle counts
std::atomic<float>    g_progress{0.f};     // 0..100
std::atomic<bool>     g_vertsReady{false};
std::atomic<bool>     g_allDone{false};
double g_importMs = 0, g_totalMs = 0;

// ---- materials / textures ----
std::vector<ap::PackMaterial> g_materials;       // materials snapshot (string_views stay valid)
texp::TexPipeline g_texPipe;                     // parallel mmap + stb_image decode
std::atomic<size_t> g_texBudget{32};             // adaptive per-frame upload cap
std::vector<int> g_meshTex;                      // per-mesh texture slot or -1
std::atomic<bool> g_noTex{false};                // --untex / T: disable textures

// ---- scene / camera ----
float g_scale = 1.f;
float g_center[3] = {0, 0, 0};
float g_camDist = 2.2f;
float g_rotY = 0.6f, g_pitch = 0.35f;
std::atomic<bool> g_quit{false};
bool g_dragging = false;                   // left-button orbit in progress
float g_offX = 0.f, g_offZ = 0.f;          // WASD view-space offsets
float g_offY = 0.f;                        // Q/E world-space camera height
bool g_cullFrontZ = true;                  // true: CCW front faces have +z winding
bool g_autoRotate = false;                 // --autoRotate: orbit yaw each frame
bool g_nostream = false;                   // --nostream: blocking setGeometry upload
bool g_useSceneCamera = false;             // --useSceneCamera: initial camera from file
bool g_useSceneLights = false;             // --useSceneLights: use file lights
std::vector<ap::PackCamera> g_cameras;     // scene cameras snapshot
std::vector<ap::PackLight> g_lights;       // scene lights snapshot
int g_activeCamera = -1;

// parser pool pointers captured at onVerticesReady (the pack outlives the
// frame loop, so they stay valid); DxRenderer stages its geometry straight
// from these pools
const float* g_posPool = nullptr;
const float* g_uvPool = nullptr;           // null when the model has no uvs
const uint32_t* g_idxPool = nullptr;
size_t g_idxCount = 0;
size_t g_poolVerts = 0;

// ---- geometry streaming (parse/upload overlap) ----
// The parser publishes pool ranges as they finish filling (see
// GeoStreamSink); the onRange callback copies them into the GPU staging
// ring right on the parser worker, so the memcpy bandwidth is hidden
// under the parse itself. onRange waits for beginGeometryStream, which
// the onMeta callback starts on its own thread (so buffer creation
// overlaps the parse instead of the frame loop); it also handles the
// zero uv fill for models without texcoords.
std::mutex g_geoMx;
std::condition_variable g_geoCv;
std::atomic<bool> g_geoBegun{false};       // buffers created, ring up
std::atomic<uint64_t> g_geoBytes{0};       // bytes copied to the GPU
std::thread g_geoThread;                   // joinable begin (see bindEvents)
std::atomic<bool> g_geoThreadLive{false};

// ---- runtime flags ----
int g_frames = 0;                          // 0 = until quit
bool g_waitAll = false;                    // count frames only after all-done
int g_shotAt = 0;
std::string g_shotFile;
std::string g_modelName;
uint64_t g_frameCount = 0;
Clock::time_point g_tLoad;
double g_cpuFrameMs = 0;                 // last loop iteration, shown by the overlay
Clock::time_point g_tPrevLoop;           // frame pacing reference for the above

// ---- per-frame STAT record (UE STAT UNIT style): --stat <name> writes
// one CSV row per frame (cpu/gpu/scene/ovl/wait/record) plus the run
// averages for the ## compare table in benchmark.md ----
std::string g_statName;
std::ofstream g_statFile;
double g_accCpu = 0, g_accGpu = 0, g_accScene = 0, g_accOvl = 0;
double g_accWait = 0, g_accRecord = 0;
size_t g_statN = 0;                    // averaged frames (skips the first 10)

// ---- PIX programmatic GPU capture: --pix <file.wpix> starts recording on
// the first scene frame and saves the whole run (for PIX timeline analysis
// of per-draw GPU work) ----
std::string g_pixPath;
int g_pixStartFrame = 0;        // begin PIX capture at this frame (skip load)
bool g_pixStarted = false;

// Loads the newest %ProgramFiles%\Microsoft PIX\<version>\WinPixGpuCapturer
// .dll BEFORE the D3D12 device exists: PIX only captures devices created
// after the capturer is in the process, and WinPixEventRuntime's own lazy
// load would be too late (it happens on the first scene frame).
static HMODULE loadPixGpuCapturer() {
    HMODULE h = GetModuleHandleW(L"WinPixGpuCapturer.dll");
    if (h) return h;
    wchar_t pattern[MAX_PATH];
    const DWORD plen =
        GetEnvironmentVariableW(L"ProgramFiles", pattern, MAX_PATH);
    if (plen == 0 || plen >= MAX_PATH) return nullptr;
    wcscat_s(pattern, L"\\Microsoft PIX\\*");
    WIN32_FIND_DATAW fd;
    HANDLE find = FindFirstFileW(pattern, &fd);
    if (find == INVALID_HANDLE_VALUE) return nullptr;
    wchar_t bestVer[MAX_PATH] = {}, bestDll[MAX_PATH] = {};
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) ||
            fd.cFileName[0] == L'.')
            continue;
        wchar_t dir[MAX_PATH];
        wcscpy_s(dir, pattern);
        dir[wcslen(dir) - 1] = L'\0';   // drop the '*' wildcard
        wcscat_s(dir, fd.cFileName);
        wcscat_s(dir, L"\\WinPixGpuCapturer.dll");
        if (GetFileAttributesW(dir) != INVALID_FILE_ATTRIBUTES &&
            (bestVer[0] == L'\0' || wcscmp(fd.cFileName, bestVer) > 0)) {
            wcscpy_s(bestVer, fd.cFileName);
            wcscpy_s(bestDll, dir);
        }
    } while (FindNextFileW(find, &fd));
    FindClose(find);
    return bestDll[0] ? LoadLibraryW(bestDll) : nullptr;
}

// ---- render backend state (created via RendererFactory) ----
std::string g_rendererName = "dx12";       // --renderer dx12|null
std::unique_ptr<IRenderer> g_dx;
std::string g_dxShotNext;                  // screenshot path, captured next frame
bool g_forceWarp = false;                  // --warp

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

// Upload entry point: called on parser workers with a finished pool
// range. Copies straight into the GPU staging ring (stageGeometryRange
// is mutex-serialized); the range's data stays valid until the
// publishing task returns, and the copy happens inside this call, so
// there is no pool-lifetime race with seamFill's resize. Parser
// workers wait only until the onMeta thread finished creating the
// buffers (beginGeometryStream); with the meta published at the prefix
// task start that wait is normally already satisfied, and the counters
// below catch any regression in the handshake.
static std::atomic<uint64_t> g_geoWaitUs{0};
static std::atomic<uint64_t> g_geoWaitCalls{0};
static void stageGeoRange(ap::GeoRangeKind kind, size_t offsetBytes,
                          const void* data, size_t sizeBytes) {
    {
        const auto tW = Clock::now();
        std::unique_lock<std::mutex> lk(g_geoMx);
        g_geoCv.wait(lk, [] { return g_geoBegun.load(); });
        g_geoWaitUs.fetch_add(
            uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
                         Clock::now() - tW)
                         .count()));
    }
    g_dx->stageGeometryRange(kind, offsetBytes, data, sizeBytes);
    g_geoBytes.fetch_add(sizeBytes, std::memory_order_relaxed);
    g_geoWaitCalls.fetch_add(1);
}

static std::string msStr(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

// ============================================================
// loading overlay (CPU-drawn progress strip, uploaded as a texture)
// ============================================================

// overlay rasterization, run as g_tf tasks (off the render thread):

// ---- TrueType text (Consolas): one baked glyph atlas, loading caption ---
struct BakedFont {
    bool ok = false;
    std::vector<unsigned char> atlas;   // 8-bit alpha
    int W = 512, H = 256;
    float ascent = 0.f;                 // pixels above the baseline
    stbtt_bakedchar chars[96];
};
static BakedFont g_fontBig;

static void initFonts() {
    const char* candidates[] = {
        "C:/Windows/Fonts/consola.ttf",   // Consolas: monospace, crisp stats
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
    };
    std::vector<char> data;
    const char* used = nullptr;
    for (const char* path : candidates) {
        std::ifstream f(path, std::ios::binary);
        if (!f) continue;
        data.assign(std::istreambuf_iterator<char>(f),
                    std::istreambuf_iterator<char>());
        if (data.size() > 1000) {
            used = path;
            break;
        }
    }
    if (!used) {
        AP_LOG_WARN("viewer", "no system font found; overlays render without text");
        return;
    }
    const auto bake = [&data](BakedFont& f, float px) {
        f.atlas.assign(size_t(f.W) * f.H, 0);
        const int r = stbtt_BakeFontBitmap(
            reinterpret_cast<const unsigned char*>(data.data()), 0, px,
            f.atlas.data(), f.W, f.H, 32, 96, f.chars);
        f.ok = r > 0;
        // glyph bodies sit above the baseline (GetBakedQuad's y is the
        // baseline, not the line top); ~0.8 * em for typical fonts
        f.ascent = px * 0.8f;
    };
    bake(g_fontBig, 36.f);
    AP_LOG("viewer", "font: %s (loading %s)", used,
           g_fontBig.ok ? "ok" : "off");
}

// alpha-blend baked glyphs onto an ARGB buffer; y = top of the line
static void drawTextTTF(uint32_t* px, int bufW, int bufH, const BakedFont& f,
                        const char* s, float x, float y, uint32_t col) {
    if (!f.ok || !s) return;
    const uint32_t cr = (col >> 16) & 255, cg = (col >> 8) & 255,
                   cb = col & 255;
    float fx = x, fy = y + f.ascent;   // baseline below the line top
    for (const char* p = s; *p; ++p) {
        const unsigned char c = (unsigned char)*p;
        if (c < 32 || c >= 128) continue;
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(f.chars, f.W, f.H, int(c) - 32, &fx, &fy, &q, 1);
        const int x0 = std::max(0, int(q.x0)), x1 = std::min(bufW, int(q.x1));
        const int y0 = std::max(0, int(q.y0)), y1 = std::min(bufH, int(q.y1));
        if (x1 <= x0 || y1 <= y0) continue;
        const float gw = q.x1 - q.x0, gh = q.y1 - q.y0;
        for (int yy = y0; yy < y1; ++yy) {
            const float tv =
                q.t0 + (q.t1 - q.t0) * (float(yy) - q.y0) / (gh + 0.01f);
            const unsigned char* arow =
                f.atlas.data() + size_t(tv * f.H) * f.W;
            for (int xx = x0; xx < x1; ++xx) {
                const float tu =
                    q.s0 + (q.s1 - q.s0) * (float(xx) - q.x0) / (gw + 0.01f);
                const unsigned char a = arow[int(tu * f.W)];
                if (!a) continue;
                uint32_t& dst = px[size_t(yy) * bufW + xx];
                const uint32_t br = (dst >> 16) & 255, bg = (dst >> 8) & 255,
                               bb = dst & 255;
                dst = 0xFF000000u
                    | ((br + (cr - br) * a / 255) << 16)
                    | ((bg + (cg - bg) * a / 255) << 8)
                    | (bb + (cb - bb) * a / 255);
            }
        }
    }
}

// plain color rect (the loading bar); no rendering library needed
static void fillRect(uint32_t* px, int bufW, int bufH,
                     int x, int y, int w, int h, uint32_t col) {
    const int x0 = std::max(0, x), y0 = std::max(0, y);
    const int x1 = std::min(bufW, x + w), y1 = std::min(bufH, y + h);
    for (int yy = y0; yy < y1; ++yy)
        for (int xx = x0; xx < x1; ++xx)
            px[size_t(yy) * bufW + xx] = col;
}

// full-window loading frame (ARGB), bar + caption at the given percent
static std::vector<uint32_t> rasterLoading(int pctI) {
    std::vector<uint32_t> px(size_t(kWinW) * size_t(kWinH), kBg);
    const int bw = int(kWinW * 0.6f), bh = 12;
    const int bx = (kWinW - bw) / 2, by = kWinH - 130;
    fillRect(px.data(), kWinW, kWinH, bx, by, bw, bh, kSlot);
    const float pct = float(pctI);
    if (pct > 0.f)
        fillRect(px.data(), kWinW, kWinH, bx + 2, by + 2,
                 int((bw - 4) * pct / 100.f), bh - 4, kBar);
    if (g_fontBig.ok) {
        char txt[192];
        std::snprintf(txt, sizeof txt, "Loading %s  %.0f%%",
                      g_modelName.c_str(), pct);
        // center the caption by walking the advances once
        float wq = 0.f, yq = 0.f;
        for (const char* p = txt; *p; ++p) {
            const unsigned char ch = (unsigned char)*p;
            if (ch < 32 || ch >= 128) continue;
            stbtt_aligned_quad q;
            stbtt_GetBakedQuad(g_fontBig.chars, g_fontBig.W, g_fontBig.H,
                               int(ch) - 32, &wq, &yq, &q, 1);
        }
        drawTextTTF(px.data(), kWinW, kWinH, g_fontBig, txt,
                    float((kWinW - int(wq)) / 2), float(by - 56), kText);
    }
    return px;
}

// in-game stats overlay: Dear ImGui (DxRenderer draws it after the MSAA
// resolve). One auto-sized window, rebuilt every frame.
static void buildUi(double fpsNow) {
    ImGui::SetNextWindowPos(ImVec2(10, 8), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("stats", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted(g_modelName.c_str());
    if (g_allDone.load()) {
        // verts may never arrive (failed load): back() on the empty
        // prefix-sums vector would be UB
        const uint64_t tris =
            g_meshTriStart.empty() ? 0 : g_meshTriStart.back();
        ImGui::Text("%llu tris | %.1f fps", (unsigned long long)tris,
                    fpsNow);
    } else {
        ImGui::ProgressBar(g_progress.load() / 100.f, ImVec2(-1.f, 0.f), "");
    }
    // CPU frame time (measured in the loop) vs GPU frame time (timestamp
    // queries, read back two frames late by the renderer); the gap shows
    // which side is the bottleneck
    float gFrame = 0.f, gScene = 0.f, gOvl = 0.f;
    if (g_dx) g_dx->gpuTiming(gFrame, gScene, gOvl);
    ImGui::Text("cpu %6.2f ms | gpu %6.2f ms", g_cpuFrameMs, gFrame);
    if (g_allDone.load()) {
        ImGui::Text("scene %6.2f + ovl %5.2f ms", gScene, gOvl);
        ImGui::Text("import %.1f ms | total %.1f ms", g_importMs, g_totalMs);
    }
    if (g_texPipe.count() > 0)
        ImGui::Text("%zu/%zu textures%s", g_texPipe.readyCount(),
                    g_texPipe.count(), g_noTex.load() ? " (off)" : "");
    ImGui::Checkbox("Use scene camera", &g_useSceneCamera);
    ImGui::Checkbox("Use scene lights", &g_useSceneLights);
    if (!g_cameras.empty())
        ImGui::Text("%zu cameras%s", g_cameras.size(), g_useSceneCamera ? " (active)" : "");
    if (!g_lights.empty())
        ImGui::Text("%zu lights%s", g_lights.size(), g_useSceneLights ? " (using file)" : " (default)");
    { // texBudget is display-side config (was formerly Config::texBudget)
        int b = int(g_texBudget.load(std::memory_order_relaxed));
        if (ImGui::SliderInt("texBudget", &b, 1, 128)) g_texBudget.store(size_t(b), std::memory_order_relaxed);
    }
    ImGui::Separator();
    ImGui::TextDisabled(
        "drag orbit | wheel zoom | WASD move | Q/E height | T tex | Esc quit");
    ImGui::End();
}

// ============================================================
// parser events -> renderer state + benchmark lines
// ============================================================

// Bind each mesh's material diffuse texture (by mtl path) to a decoded
// texture slot. Idempotent: called from every event that may complete
// the materials/textures picture.
static void bindTextures() {
    if (g_noTex) {
        g_meshTex.assign(g_meshTex.size(), -1);
        return;
    }
    if (g_materials.empty() || g_meshTex.empty() || g_texPipe.readyCount() == 0)
        return;
    g_meshTex.assign(g_meshTex.size(), -1);
    size_t nMat = 0, nNoDiff = 0, nLookupFail = 0, nNoUV = 0;
    uint64_t trisNoDiff = 0, trisBound = 0;
    for (size_t i = 0; i < g_meshes.size(); ++i) {
        const int mi = g_meshes[i].materialIndex;
        if (mi < 0 || size_t(mi) >= g_materials.size()) continue;
        ++nMat;
        const ap::PackMaterial& m = g_materials[size_t(mi)];
        bool hasDiff = false;
        for (const ap::PackTexRef& ref : m.textures)
            if (ref.type == int(ap::TexType::TexDiffuse)) { hasDiff = true; break; }
        if (!hasDiff) { ++nNoDiff; trisNoDiff += g_meshes[i].triangleCount(); continue; }
        bool found = false;
        for (const ap::PackTexRef& ref : m.textures) {
            if (ref.type != int(ap::TexType::TexDiffuse)) continue;
            const int slot = g_texPipe.slotFor(ref.path);
            if (slot >= 0) {
                g_meshTex[i] = slot;
                found = true;
                if (g_meshes[i].texcoords.empty()) ++nNoUV;
                break;
            }
        }
        if (!found) ++nLookupFail;
    }
    size_t bound = 0;
    for (int t : g_meshTex)
        if (t >= 0) ++bound;
    for (size_t i = 0; i < g_meshes.size(); ++i)
        if (g_meshTex[i] >= 0) trisBound += g_meshes[i].triangleCount();
    // progressive rebinds run every few frames while decoding; only log
    // when the bound count actually moved
    static int lastBoundLog = -1;
    if (bound == lastBoundLog) return;
    lastBoundLog = int(bound);
    AP_LOG("viewer", "textures bound: %zu/%zu meshes (mat %zu, no-diff %zu, lookup-fail %zu, bound-noUV %zu); tris: %llu bound / %llu no-diff",
           bound, g_meshTex.size(), nMat, nNoDiff, nLookupFail, nNoUV,
           (unsigned long long)trisBound, (unsigned long long)trisNoDiff);
}

static void bindEvents(ap::AssetPack& pack) {
    pack.setProgress([](float pct) { g_progress.store(pct); });

    // geometry streaming: publish pool ranges as they finish filling so
    // the consumer can copy them while the parse still runs (meta at
    // prefix, positions/indices per pass2 chunk, texcoords per pass3
    // chunk, seam splits at seamFill). Models without texcoords get a
    // zero uv buffer from beginGeometryStream. The meta callback kicks
    // the buffer creation on its own thread: creating the 2.5 GB
    // default-heap buffers takes a few hundred ms, and doing it from
    // the frame loop would stall every parser worker that publishes a
    // range before the loop got around to it (measured ~9.6 s of
    // cumulative publish stalls). Skipped with --nostream (blocking
    // setGeometry A/B).
    if (!g_nostream) {
        pack.setGeoStream({
            [](size_t verts, size_t tris, bool hasUv) {
                // joinable, not detached: main() joins it before g_dx
                // is released so a failed/aborted load cannot free the
                // device under the buffer-creation thread
                g_geoThread = std::thread([verts, tris, hasUv]() {
                    if (g_dx) {
                        g_dx->beginGeometryStream(verts, tris, hasUv);
                    }
                    {
                        std::lock_guard<std::mutex> lk(g_geoMx);
                        g_geoBegun.store(true);
                    }
                    g_geoCv.notify_all();
                });
                g_geoThreadLive.store(true);
            },
            [](ap::GeoRangeKind kind, size_t offsetBytes, const void* data,
               size_t sizeBytes) {
                stageGeoRange(kind, offsetBytes, data, sizeBytes);
            }});
    }

    pack.setOnVerticesReady(
        [](ap::PackResult& r, std::span<const ap::PackMesh> meshes) {
            g_meshes.assign(meshes.begin(), meshes.end());
            // the copies above may hold stale pre-bind temp material
            // indices; refresh them from the live result before binding
            for (size_t i = 0; i < g_meshes.size() && i < r.meshes.size(); ++i)
                g_meshes[i].materialIndex = r.meshes[i].materialIndex;
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
            g_posPool = r.positions.data();
            g_uvPool = r.texcoords.empty() ? nullptr : r.texcoords.data();
            g_idxPool = r.posIndices.data();
            g_idxCount = r.posIndices.size();
            g_poolVerts = poolVerts;
            const uint64_t nTris = g_meshTriStart.back();
            AP_LOG("viewer", "vertices ready: %zu meshes, %zu verts, %llu tris",
                   g_meshes.size(), poolVerts, (unsigned long long)nTris);
            AP_LOG("viewer", "bounds min %.2f %.2f %.2f max %.2f %.2f %.2f scale %.4f",
                   mn[0], mn[1], mn[2], mx[0], mx[1], mx[2], g_scale);
            benchAppend("| " + timestamp() + " | [async] vertices-ready | "
                        + msStr(double(r.verticesMicros) / 1000.0)
                        + " ms | verts " + std::to_string(poolVerts)
                        + " tris " + std::to_string(nTris) + " |");
            // winding probe: models may be CW or CCW; sample triangles in
            // view space with the initial camera once, here on the parser
            // thread (same math as the vertex shader), and pick the front
            // side for the rasterizer state
            {
                const float cY = std::cos(g_rotY), sY = std::sin(g_rotY);
                const float cP = std::cos(g_pitch), sP = std::sin(g_pitch);
                const float s = g_scale, cx = g_center[0], cy = g_center[1],
                            cz = g_center[2];
                const float dist = g_camDist, offX = g_offX, offZ = g_offZ;
                const float* pool = r.positions.data();
                int pos = 0, neg = 0;
                uint64_t cnt = 0;
                for (const ap::PackMesh& m : g_meshes) {
                    const uint32_t* idx = m.indices.data();
                    const uint64_t lim =
                        std::min<uint64_t>(m.triangleCount(), 4096 - cnt);
                    for (uint64_t t = 0; t < lim; ++t) {
                        float V[3][3];
                        for (int k = 0; k < 3; ++k) {
                            const float* p = pool + size_t(idx[3 * t + k]) * 3;
                            const float px = (p[0] - cx) * s,
                                        py = (p[1] - cy) * s,
                                        pz = (p[2] - cz) * s;
                            const float x1 = cY * px + sY * pz;
                            const float z1 = -sY * px + cY * pz;
                            V[k][0] = x1 + offX;
                            V[k][1] = cP * py - sP * z1;
                            V[k][2] = sP * py + cP * z1 + dist + offZ;
                        }
                        const float fnz =
                            (V[1][0] - V[0][0]) * (V[2][1] - V[0][1])
                            - (V[1][1] - V[0][1]) * (V[2][0] - V[0][0]);
                        fnz > 0.f ? ++pos : ++neg;
                    }
                    cnt += lim;
                    if (cnt >= 4096) break;
                }
                g_cullFrontZ = pos >= neg;
                AP_LOG("viewer", "winding probe: %d ccw %d cw -> front=%s",
                       pos, neg, g_cullFrontZ ? "+z" : "-z");
            }
            bindTextures();
            g_vertsReady.store(true);
        });

    pack.setOnMaterialsReady(
        [](ap::PackResult& r, std::span<const ap::PackMaterial> mats) {
            g_materials.assign(mats.begin(), mats.end());
            // refresh stale pre-bind indices from the live result before
            // the diffuse lookup below uses them
            for (size_t i = 0; i < g_meshes.size() && i < r.meshes.size(); ++i)
                g_meshes[i].materialIndex = r.meshes[i].materialIndex;
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
            // background mmap + stb_image decode of every external
            // reference; slots publish progressively via readyCount()
            g_texPipe.enqueue(texs);
            AP_LOG("viewer", "textures queued: %zu refs (background decode)",
                   texs.size());
            benchAppend("| " + timestamp() + " | [async] textures-queued | "
                        + msStr(double(r.texturesMicros) / 1000.0)
                        + " ms | refs " + std::to_string(texs.size()) + " |");
        });

    pack.setOnAllDone([](ap::PackResult& r, bool ok, std::string_view err) {
        g_importMs = double(r.importMicros) / 1000.0;
        g_totalMs = double(r.totalMicros) / 1000.0;
        const double deltaMs = secsSince(g_tLoad) * 1000.0;
        g_cameras.assign(r.cameras.begin(), r.cameras.end());
        g_lights.assign(r.lights.begin(), r.lights.end());
        g_activeCamera = r.activeCamera;
        if (g_useSceneCamera && !g_cameras.empty() && g_activeCamera >=0 &&
            size_t(g_activeCamera) < g_cameras.size()) {
            const auto& c = g_cameras[size_t(g_activeCamera)];
            float dir[3] = {c.target[0]-c.position[0], c.target[1]-c.position[1], c.target[2]-c.position[2]};
            float dist = std::sqrt(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2]);
            if (dist > 1e-4f) {
                g_rotY = std::atan2(dir[0], dir[2]);
                g_pitch = std::asin(dir[1]/dist);
                if (g_pitch > 1.4f) g_pitch = 1.4f;
                if (g_pitch < -1.4f) g_pitch = -1.4f;
                g_camDist = dist;
                g_center[0]=c.target[0]; g_center[1]=c.target[1]; g_center[2]=c.target[2];
                AP_LOG("viewer", "scene camera '%.*s' -> yaw %.2f pitch %.2f dist %.2f fov %.1f",
                       int(c.name.size()), c.name.data(), g_rotY, g_pitch, g_camDist, c.fovYDegrees);
            }
        }
        if (g_useSceneLights) {
            AP_LOG("viewer", "scene lights: %zu (using file)", g_lights.size());
        } else if (!g_lights.empty()) {
            AP_LOG("viewer", "scene lights: %zu (ignored, default)", g_lights.size());
        }
        g_allDone.store(true);
        // no further texture references will arrive; let the decode
        // worker drain and exit
        g_texPipe.finish();
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
    char line[256];
    std::snprintf(line, sizeof line,
                  "| %s | %s | %llu | %.1f | %.1f | ALL | 1 |\n",
                  timestamp().c_str(), g_modelName.c_str(),
                  (unsigned long long)frames, fps, ms);
    out << line;
}

// one row per --frames run: the same metric in the same column across
// runs/optimizations, so successive runs accumulate a comparison table
static void benchCompareLine(double fps) {
    const char* header =
        "## compare (per-run; same metric per column)\n"
        "| version | time | import ms | verts | avg cpu ms | avg gpu ms |"
        " avg scene ms | avg ovl ms | avg wait ms | avg record ms | avg fps |\n"
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"
        " ---: | ---: |\n";
    std::ifstream in("benchmark.md", std::ios::binary);
    std::string tail;
    if (in) {
        in.seekg(0, std::ios::end);
        const std::streamoff len = in.tellg();
        in.seekg(len > 2048 ? len - 2048 : 0);
        tail.assign(std::istreambuf_iterator<char>(in),
                    std::istreambuf_iterator<char>());
    }
    std::ofstream out("benchmark.md", std::ios::app);
    if (tail.find("## compare") == std::string::npos) out << "\n" << header;
    const double n = std::max(size_t(1), g_statN);
    const std::string& ver =
        g_statName.empty() ? g_modelName : g_statName;
    char line[512];
    std::snprintf(line, sizeof line,
                  "| %s | %s | %.1f | %zu | %.2f | %.2f | %.2f | %.3f |"
                  " %.2f | %.2f | %.1f |\n",
                  ver.c_str(), timestamp().c_str(), g_importMs, g_poolVerts,
                  g_accCpu / n, g_accGpu / n, g_accScene / n, g_accOvl / n,
                  g_accWait / n, g_accRecord / n, fps);
    out << line;
}

} // namespace

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    bool _cfgOk = ap::Config::instance().loadFromFile("F:/project/assimp/assetpack/config.json");
    if (!_cfgOk) _cfgOk = ap::Config::instance().loadFromFile();
    AP_LOG("cfg", "load %s cacheImage=%d dir=%s", _cfgOk?"ok":"fail", int(ap::Config::instance().cacheImage), ap::Config::instance().cacheDir.c_str());
    std::string model = kDefaultModel;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) g_frames = std::atoi(argv[++i]);
        else if (a == "--wait") g_waitAll = true;
        else if (a == "--warp") g_forceWarp = true;
        else if (a == "--untex") g_noTex = true;
        else if (a == "--shot" && i + 2 < argc) {
            g_shotAt = std::atoi(argv[i + 1]);
            g_shotFile = argv[i + 2];
            i += 2;
        } else if (a == "--stat" && i + 1 < argc) {
            g_statName = argv[++i];   // per-frame CSV + compare-table tag
        } else if (a == "--pix" && i + 1 < argc) {
            g_pixPath = argv[++i];    // programmatic GPU capture target
        } else if (a == "--pixStart" && i + 1 < argc) {
            g_pixStartFrame = std::atoi(argv[++i]);   // skip warm-up frames
        } else if (a == "--autoRotate") {
            g_autoRotate = true;          // orbit the model each frame
        } else if (a == "--nostream") {
            g_nostream = true;            // A/B: blocking setGeometry upload
        } else if (a == "--useSceneCamera") {
            g_useSceneCamera = true;
        } else if (a == "--useSceneLights") {
            g_useSceneLights = true;
        } else if (a == "--useScene") {
            g_useSceneCamera = g_useSceneLights = true;
        } else if (a == "--texBudget" && i + 1 < argc) {
            g_texBudget.store(size_t(std::atoi(argv[++i])), std::memory_order_relaxed);
        } else if (a == "--camDist" && i + 1 < argc) {
            g_camDist = float(std::atof(argv[++i]));   // allow < 0.15 (inside)
        } else if (a == "--renderer" && i + 1 < argc) {
            g_rendererName = argv[++i];   // backend pick: dx12 | null
        } else if (a.rfind("--", 0) != 0) {
            model = a;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return 2;
        }
    }
    g_modelName = model.substr(model.find_last_of("/\\") + 1);

    // PIX capturer must be in-process before the D3D12 device is created
    // (PIX does not capture devices that already exist)
    if (!g_pixPath.empty() && !loadPixGpuCapturer())
        std::fprintf(stderr,
                     "warning: PIX GPU capturer not found; --pix will no-op\n");

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
    // Dear ImGui context + SDL2 backend first: DxRenderer::init calls
    // ImGui_ImplDX12_Init and needs the context to exist
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForD3D(g_win);
    g_dx = view::createRenderer(g_rendererName);
    if (!g_dx) {
        std::fprintf(stderr, "unknown renderer '%s' (expected dx12|null)\n",
                     g_rendererName.c_str());
        SDL_DestroyWindow(g_win);
        SDL_Quit();
        return 2;
    }
    if (!g_dx->init(g_win, kWinW, kWinH, g_forceWarp)) {
        std::fprintf(stderr, "%s renderer init failed\n",
                     g_rendererName.c_str());
        return 1;
    }


    initFonts();   // bake Consolas before the first overlay raster task
    AP_LOG("viewer", "window ready, loading %s async", model.c_str());
    g_tLoad = Clock::now();
    {
        ap::AssetPack pack;              // outlives the frame loop; its
        bindEvents(pack);                // destructor drains the executor
        // normals stay parsed: the loader's sequential prefetch
        // overlaps disk reads with the scan passes, so keeping the
        // normal pool costs memory but barely moves the parse time
        pack.loadAsync(model);

        std::optional<Clock::time_point> tAll;
        uint64_t framesAll = 0;
        double fpsNow = 0;
        bool g_texAllBound = false;   // stop rebinding once decode finished
        size_t texReleasedUpTo = 0;   // decoded slots freed after GPU upload
        while (!g_quit.load()) {
            SDL_Event e;
            const ImGuiIO& io = ImGui::GetIO();
            const auto tEv0 = Clock::now();
            Clock::time_point tIt0 = tEv0, tDs0 = tEv0;   // set in the scene
                                                          // branch below
            while (SDL_PollEvent(&e)) {
                ImGui_ImplSDL2_ProcessEvent(&e);
                if (e.type == SDL_QUIT) {
                    g_quit.store(true);
                } else if (e.type == SDL_KEYDOWN && !io.WantCaptureKeyboard) {
                    switch (e.key.keysym.sym) {
                    case SDLK_ESCAPE: g_quit.store(true); break;
                    case SDLK_t: {
                        g_noTex.store(!g_noTex.load());
                        AP_LOG("viewer", "textures %s",
                               g_noTex.load() ? "OFF" : "ON");
                        break;
                    }
                    default: break;
                    }
                } else if (e.type == SDL_MOUSEBUTTONDOWN &&
                           !io.WantCaptureMouse) {
                    if (e.button.button == SDL_BUTTON_LEFT)
                        g_dragging = true;
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
                } else if (e.type == SDL_MOUSEWHEEL &&
                           !io.WantCaptureMouse) {
                    g_camDist *= std::pow(0.9f, float(e.wheel.y));
                    if (g_camDist < 0.15f) g_camDist = 0.15f;
                    if (g_camDist > 10.0f) g_camDist = 10.0f;
                }
            }
            // WASD: move along the view axis (W/S) and strafe (A/D);
            // Q/E: raise/lower the camera in world space; the step scales
            // with the current zoom distance (halved for finer control).
            // Held keys are ignored while the ImGui overlay owns them
            if (!io.WantCaptureKeyboard) {
                const Uint8* ks = SDL_GetKeyboardState(nullptr);
                const float step = 0.01f * g_camDist;
                if (ks[SDL_SCANCODE_W]) g_offZ -= step;
                if (ks[SDL_SCANCODE_S]) g_offZ += step;
                if (ks[SDL_SCANCODE_A]) g_offX += step;
                if (ks[SDL_SCANCODE_D]) g_offX -= step;
                if (ks[SDL_SCANCODE_Q]) g_offY -= step;
                if (ks[SDL_SCANCODE_E]) g_offY += step;
            }

            // ImGui frame: the render data is consumed by DxRenderer's
            // endFrame pass (drawn after the MSAA resolve). The DX12
            // backend's NewFrame also creates its device objects (PSOs,
            // the texture-upload command list) on the first call.
            // ImGui frame (dx12 only): the render data is consumed by the
            // dx12 backend's endFrame pass. The null backend has no device
            // and no renderer-NewFrame, so nobody would build the font
            // atlas - drawing UI there faults. Skip the whole frame.
            if (g_rendererName == "dx12") {
                ImGui_ImplDX12_NewFrame();
                ImGui_ImplSDL2_NewFrame();
                ImGui::NewFrame();
                buildUi(fpsNow);
                ImGui::Render();
            }
            const auto tUi0 = Clock::now();

            // geometry: the streaming path has the parser publish pool
            // ranges as they finish (meta at prefix, positions/indices
            // per pass2 chunk, uv per pass3 chunk, seam splits at
            // seamFill); onRange copies them into GPU staging right on
            // the parser worker, so the upload overlaps the parse
            // itself. The meta callback creates the buffers on its own
            // thread (see bindEvents), so the frame loop only has to
            // finalize once every vertex is ready. --nostream keeps the
            // old blocking setGeometry upload for A/B comparisons.
            if (!g_nostream) {
                if (g_vertsReady.load() && g_geoBegun.load() &&
                    !g_dx->geometryReady()) {
                    g_dx->finalizeGeometryStream(g_cullFrontZ);
                    // the pools were streamed out as they filled; free
                    // the remainder (normals, scratch, unwritten slots)
                    // so the steady frame loop starts low-watermark
                    pack.releaseGeometry();
                    AP_LOG("viewer", "geometry streamed: %llu MB in %.1f s "
                           "(pub-wait %llu ms / %llu calls)",
                           (unsigned long long)(g_geoBytes.load() >> 20),
                           secsSince(g_tLoad),
                           (unsigned long long)(g_geoWaitUs.load() / 1000),
                           (unsigned long long)g_geoWaitCalls.load());
                } else if (g_vertsReady.load() && !g_geoBegun.load() &&
                           !g_dx->geometryReady()) {
                    // parsers without GeoStreamSink support (stl/ply/
                    // gltf/fbx/pbrt) never publish onMeta: upload the
                    // pools blocking instead of waiting forever
                    g_dx->setGeometry(g_posPool, g_poolVerts, g_uvPool,
                                      g_idxPool, g_idxCount, g_cullFrontZ);
                    pack.releaseGeometry();
                    AP_LOG("viewer", "geometry uploaded (blocking, parser "
                           "has no streaming) in %.1f s", secsSince(g_tLoad));
                }
            } else if (g_vertsReady.load() && !g_dx->geometryReady()) {
                g_dx->setGeometry(g_posPool, g_poolVerts, g_uvPool,
                                  g_idxPool, g_idxCount, g_cullFrontZ);
                pack.releaseGeometry();
                AP_LOG("viewer", "geometry uploaded (blocking) in %.1f s",
                       secsSince(g_tLoad));
            }
            if (!g_vertsReady.load() || !g_dx->geometryReady()) {
                // loading overlay: rasterize off-thread on progress
                // changes, consume the latest completed frame
                const int pct = int(g_progress.load());
                if (pct != g_loadPct && !g_loadBusy.exchange(true)) {
                    g_loadPct = pct;
                    g_tf.async([pct]() {
                        auto px = std::make_shared<std::vector<uint32_t>>(
                            rasterLoading(pct));
                        {
                            const std::lock_guard<std::mutex> lk(g_ovlMx);
                            g_loadLatest = std::move(px);
                        }
                        g_loadBusy.store(false);
                    });
                }
                std::shared_ptr<std::vector<uint32_t>> load;
                {
                    const std::lock_guard<std::mutex> lk(g_ovlMx);
                    load = std::move(g_loadLatest);
                }
                g_dx->drawLoading(load ? load->data() : nullptr);
            } else {
                // per-frame STAT record: open on the first scene frame so
                // the header carries the run tag; one CSV row per frame
                // (UE STAT UNIT style) for offline plotting
                if (g_statFile.is_open() == false && !g_statName.empty()) {
                    std::filesystem::create_directories("stat");
                    g_statFile.open("stat/" + g_statName + ".csv");
                    g_statFile
                        << "# version=" << g_statName
                        << " model=" << g_modelName
                        << " time=" << timestamp() << '\n'
                        << "frame,cpu_ms,gpu_ms,scene_ms,ovl_ms,wait_ms,"
                           "record_ms\n";
                }
                // PIX GPU capture: start at --pixStart (default: the first
                // scene frame, after the loading overlay and parser ran).
                // A later start skips the texture-upload-heavy first frames
                // so PIX's high-frequency counter window lands on
                // steady-state rendering. Capture stops after the loop.
                if (!g_pixStarted && !g_pixPath.empty() &&
                    framesAll >= uint64_t(g_pixStartFrame)) {
                    std::wstring wp(g_pixPath.begin(), g_pixPath.end());
                    PIXCaptureParameters pix{};
                    pix.GpuCaptureParameters.FileName = wp.c_str();
                    const HRESULT hr =
                        PIXBeginCapture(PIX_CAPTURE_GPU, &pix);
                    AP_LOG("viewer", "pix capture begin (0x%08X) -> %s",
                           unsigned(hr), g_pixPath.c_str());
                    g_pixStarted = true;
                }
                // camera constants: identical math to the software
                // transform this viewer used to have (the vertex shader
                // mirrors the layout)
                if (g_autoRotate) {
                    g_rotY += 0.01f;      // ~10 s per full orbit at 60 fps
                }
                const float cY = std::cos(g_rotY), sY = std::sin(g_rotY);
                const float cP = std::cos(g_pitch), sP = std::sin(g_pitch);
                const float f = float(kWinH) * 0.5f / 0.5773503f;
                constexpr float nearPlane = 0.05f, farPlane = 60.f;
                const float zA = farPlane / (farPlane - nearPlane);
                const float zB = -nearPlane * farPlane / (farPlane - nearPlane);
                // NDC y is UP in D3D (unlike the software path's top-left
                // pixel rows), so the vertical scale is positive - the
                // old negative sign mirrored the whole scene vertically.
                // g_offY folds into the center's y: (p.y - (cy + h))*s
                // shifts the world down by h, i.e. raises the camera
                const float cam[16] = {
                    cY, sY, cP, sP,
                    g_scale, g_center[0], g_center[1] + g_offY, g_center[2],
                    g_camDist, g_offX, g_offZ, f,
                    2.f * f / float(kWinW), 2.f * f / float(kWinH), zA, zB,
                };

                // one draw item per mesh: an index range into the pool IBO
                std::vector<IRenderer::DrawItem> items;
                items.reserve(g_meshes.size());
                const bool texOn = !g_noTex.load();
                // progressive rebind: as background decoding publishes more
                // slots, refresh mesh->texture bindings (cheap path lookups)
                if (!g_texAllBound &&
                    (g_texPipe.done() || g_frameCount % 15 == 0)) {
                    bindTextures();
                    if (g_texPipe.done()) g_texAllBound = true;
                }
                for (size_t i = 0; i < g_meshes.size(); ++i) {
                    const ap::PackMesh& m = g_meshes[i];
                    if (m.indices.empty()) continue;
                    IRenderer::DrawItem it;
                    it.indexCount = uint32_t(m.indices.size());
                    it.startIndex = uint32_t(g_meshTriStart[i] * 3);
                    it.color = g_matColor[i];
                    it.texSlot = texOn && g_meshTex[i] >= 0 ? g_meshTex[i] : -1;
                    for (int k = 0; k < 16; ++k) it.world[k] = m.world[k];
                    it.hasWorld = m.hasWorld;
                    for (int k = 0; k < 9; ++k) it.normalWorld[k] = m.normalWorld[k];
                    items.push_back(it);
                }
                // group same-texture draws: the renderer binds the texture
                // descriptor table once per run instead of once per mesh
                std::stable_sort(items.begin(), items.end(),
                                 [](const IRenderer::DrawItem& a,
                                    const IRenderer::DrawItem& b) {
                                     return a.texSlot < b.texSlot;
                                 });
                tIt0 = Clock::now();

                // drawScene consumes g_dxShotNext; the frame after the shot
                // request is captured (content is identical: static camera
                // between frames)
                g_dx->drawScene(cam, items, g_texPipe.data(),
                                g_texPipe.readyCount(),
                                g_texBudget.load(std::memory_order_relaxed),
                                g_dxShotNext.empty() ? nullptr
                                                     : g_dxShotNext.c_str());
                tDs0 = Clock::now();
                g_dxShotNext.clear();
                // ResourceUploadBatch::End copies the sources into its
                // staging buffers synchronously, so once the frame is
                // submitted the decoded pixels this frame uploaded are
                // dead weight - hand them back immediately
                const size_t upNow = g_dx->texturesUploaded();
                if (upNow > texReleasedUpTo) {
                    g_texPipe.releaseSlots(texReleasedUpTo, upNow);
                    texReleasedUpTo = upNow;
                }
            }
            ++g_frameCount;
            // per-frame STAT row + running averages (skip the first ten
            // frames: timestamp queries settle and pacing stabilizes)
            if (g_statFile.is_open()) {
                float wMs = 0, rMs = 0, gF = 0, gS = 0, gO = 0;
                g_dx->cpuStats(wMs, rMs);
                g_dx->gpuTiming(gF, gS, gO);
                g_statFile << g_frameCount << ',' << g_cpuFrameMs << ',' << gF
                           << ',' << gS << ',' << gO << ',' << wMs << ','
                           << rMs << '\n';
                if (g_frameCount > 10) {
                    g_accCpu += g_cpuFrameMs;
                    g_accGpu += gF;
                    g_accScene += gS;
                    g_accOvl += gO;
                    g_accWait += wMs;
                    g_accRecord += rMs;
                    ++g_statN;
                }
            }
            // adaptive texture budget: keep GPU frame ~16ms
            {
                float af = 0, as = 0, ao = 0;
                g_dx->gpuTiming(af, as, ao);
                if (af > 0.1f) {
                    size_t cur = g_texBudget.load(std::memory_order_relaxed);
                    if (af < 12.0f && cur < 64)
                        g_texBudget.store(cur + 2, std::memory_order_relaxed);
                    else if (af > 16.6f && cur > 4)
                        g_texBudget.store(cur > 6 ? cur - 2 : 4, std::memory_order_relaxed);
                }
            }
            // CPU frame time (displayed by the overlay next frame)
            const auto tLoop = Clock::now();
            g_cpuFrameMs =
                std::chrono::duration<double, std::milli>(tLoop - g_tPrevLoop)
                    .count();
            g_tPrevLoop = tLoop;

            if (g_allDone.load()) {
                if (!tAll) tAll = Clock::now();
                ++framesAll;
                fpsNow = framesAll / std::max(secsSince(*tAll), 1e-6);
            }
            const uint64_t limit = g_waitAll ? framesAll : g_frameCount;
            if (g_shotAt > 0 && limit == uint64_t(g_shotAt))
                g_dxShotNext = g_shotFile;   // captured next frame
            if (g_frames > 0 && (!g_waitAll || g_allDone.load()) &&
                limit >= uint64_t(g_frames))
                break;
        }
        if (g_pixStarted) {
            PIXEndCapture(FALSE);   // flush + save the .wpix file
            AP_LOG("viewer", "pix capture saved: %s", g_pixPath.c_str());
        }
        benchRenderLine(framesAll, fpsNow);
        if (g_statN > 0) benchCompareLine(fpsNow);
        g_statFile.close();
        AP_LOG("viewer", "rendered %llu frames (%.1f fps)",
               (unsigned long long)framesAll, fpsNow);
    }

    if (g_geoThreadLive.load() && g_geoThread.joinable()) {
        g_geoThread.join();
        g_geoThreadLive.store(false);
    }
    g_dx.reset();                // ImGui_ImplDX12_Shutdown inside
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyWindow(g_win);
    SDL_Quit();
    AP_LOG("viewer", "bye");
    return 0;
}
