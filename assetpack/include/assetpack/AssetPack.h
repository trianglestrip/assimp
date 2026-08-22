#pragma once
// ============================================================
// assetpack - single public umbrella header
//
// mmap + taskflow model parsing with data-carrying stage events.
// Consumers include ONLY this header; concrete format parsers live
// inside the library (src/<format>/) and register themselves with
// ParserRegistry, so new formats plug in without touching the
// public surface or the facade:
//
//   [mmap source file + auxiliaries]
//     ├─► [geometry parse]   ──► onVerticesReady(span<PackMesh>)
//     ├─► [materials parse]  ──► onMaterialsReady(span<PackMaterial>)
//     └─► [textures parse]   ──► onTexturesReady(span<PackTexture>)
//   all ──► onAllDone
//
// Sections below: log / mapped file / data contract + ModelParser /
// parser registry / AssetPack facade.
// ============================================================

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ap {

// ============================================================
// Log - minimal thread-safe stage logger
// ============================================================

enum class LogLevel : uint8_t { Info, Warn, Error };

// Install/reset the log sink (default: stdout, line-buffered).
// The sink receives fully formatted lines.
void setLogSink(void (*sink)(std::string_view line));

void logLine(LogLevel level, std::string_view stage, std::string_view msg);

// milliseconds since first log call (process-anchored)
uint64_t logClockMs();

// ============================================================
// MappedFile - memory-mapped file (Windows)
//
// The physical basis of the zero-copy chain: file contents are
// never copied; all views point into the mapping, kept alive via
// shared_ptr ownership.
// ============================================================

class MappedFile {
public:
    MappedFile() = default;
    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;
    ~MappedFile();

    // Open and map `path` read-only. Returns false on any failure
    // (file missing, empty file, mapping denied).
    bool open(std::string_view path);

    // View over the whole file. Empty if not open.
    std::span<const std::byte> bytes() const;

    bool isOpen() const;
    size_t size() const;

    // Convenience: reinterpret the mapping as chars
    std::string_view text() const;

    // Windows: issue PrefetchVirtualMemory for [offset, offset+len) so
    // the next consumer of the mapping finds its pages resident.
    // Cheap when the pages are already cached; a no-op elsewhere.
    // Returns true when the range was accepted.
    bool prefetch(size_t offset, size_t len) const;

    // Shared ownership so views can outlive the loader object.
    static std::shared_ptr<MappedFile> openShared(std::string_view path);

private:
    void close();

    void*  file_       = nullptr;  // HANDLE
    void*  mapping_    = nullptr;  // HANDLE
    void*  base_       = nullptr;  // MapViewOfFile base
    std::span<const std::byte> bytes_;
};

// ============================================================
// Data contract + ModelParser base
//
// Every parser derives from ModelParser and emits the same
// data-carrying completion events; each handler receives the packed
// section of the result it just completed.
// ============================================================

// ---- texture slot types (parsers map their format onto these) ----
enum TexType : int {
    TexDiffuse  = 0,   // map_Kd / baseColorTexture
    TexAmbient  = 1,   // map_Ka
    TexSpecular = 2,   // map_Ks
    TexEmissive = 3,   // map_Ke / emissiveTexture
    TexOpacity  = 4,   // map_d / opacityTexture
    TexNormal   = 5,   // map_Bump / normalTexture
    TexCount
};

const char* texTypeName(int t);

// ---- vertices section (one entry per mesh/group) ----
struct PackMesh {
    std::string_view name;           // group/object name (file mapping view)
    int32_t  materialIndex = -1;     // index into PackResult::materials
    // zero-copy views into the result's global pools; the index array
    // is relative to those pools
    std::span<const float>   positions;   // 3 floats per vertex (local space)
    std::span<const float>   normals;     // 3 floats per vertex (empty when absent)
    std::span<const float>   texcoords;   // 2 floats per vertex (empty when absent)
    std::span<const uint32_t> indices;    // 3 per triangle (packed)
    float boundsMin[3] = {0,0,0};    // world space (after CTM)
    float boundsMax[3] = {0,0,0};
    // per-mesh world transform (row-major 4x4, identity when not used) -
    // lets the GPU do the CTM instead of per-vertex CPU transforms
    float world[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    bool hasWorld = false;

    uint32_t vertexCount()  const;
    uint32_t triangleCount() const;
};

// ---- materials section ----
struct PackTexRef {                 // a texture slot bound to a material
    int         type = TexDiffuse;  // ap::TexType
    unsigned    slot = 0;           // index within that type
    std::string_view path;          // as written in the file (view into mapping)
};

struct PackMaterial {
    std::string_view name;
    float diffuse[4]  = {1,1,1,1};
    float specular[4] = {0,0,0,1};
    float ambient[4]  = {0,0,0,1};
    float emissive[4] = {0,0,0,1};
    float opacity   = 1.f;
    float shininess = 0.f;
    bool  twoSided  = false;
    std::vector<PackTexRef> textures;       // bound slots (in type order)
};

// ---- scene-level entities (shared by scene formats: pbrt, glTF) -----
enum class LightKind : uint8_t {
    Unknown, Point, Directional, Infinite, Spot, Area,
};

struct PackLight {
    LightKind kind = LightKind::Unknown;
    std::string_view name;            // format-specific type/name string
    float color[4] = {1, 1, 1, 1};    // linear RGB + intensity in .a unused
    float intensity = 1.f;
    float position[3] = {0, 0, 0};    // point/spot (world space)
    float direction[3] = {0, 0, -1};  // directional/infinite/spot
    // area lights reference the mesh they emit from
    int32_t meshIndex = -1;
};

struct PackCamera {
    std::string_view name;            // e.g. "perspective"
    float position[3] = {0, 0, 0};
    float target[3] = {0, 0, -1};
    float up[3] = {0, 1, 0};
    float fovYDegrees = 45.f;
    float aspect = 16.f / 9.f;
    float nearZ = 0.05f, farZ = 100.f;
};

// ---- textures section ----
struct PackTexture {
    std::string_view path;          // as referenced in the material
    int         type = TexDiffuse;  // ap::TexType
    unsigned    slot = 0;
    bool        embedded = false;   // embedded bytes in the source (glTF etc.)
    std::span<const std::byte> data;    // embedded bytes (zero-copy view); empty if external
    std::string resolvedPath;           // absolute path for external loading (embedded: empty)
    size_t      byteSize = 0;           // embedded data size (or 0)
};

struct PackResult {
    // mappings own every string_view above; keep alive while views are in use
    std::shared_ptr<MappedFile> objFile;
    std::shared_ptr<MappedFile> mtlFile;    // OBJ/MTL parsers only
    // when texture bytes are loaded, owns the mmap backing each
    // PackTexture::data so the spans stay valid for this result's lifetime
    std::vector<std::shared_ptr<MappedFile>> textureFiles;
    // owns decoded embedded texture bytes (e.g. glTF data:- URIs); a
    // PackTexture::data span points into the matching entry
    std::vector<std::vector<std::byte>> embeddedTextures;
    std::string sourceDir;

    // global pools backing the PackMesh views
    std::vector<float>    positions;    // 3 floats per vertex
    std::vector<float>    normals;      // 3 floats per vertex (empty when absent)
    std::vector<float>    texcoords;    // 2 floats per vertex (empty when absent)
    std::vector<uint32_t> posIndices;   // 3 per triangle (packed)

    std::vector<PackMesh>      meshes;
    std::vector<PackMaterial>  materials;
    std::vector<PackTexture>   textures;

    // scene entities (populated by scene formats: pbrt, glTF)
    std::vector<PackCamera> cameras;
    std::vector<PackLight>  lights;
    int32_t activeCamera = -1;        // index into cameras; -1 = none

    // per-stage wall times (microseconds)
    uint64_t importMicros = 0;      // main-file parse (geometry)
    uint64_t verticesMicros = 0;    // packing meshes (per-mesh views + bounds)
    uint64_t materialsMicros = 0;   // materials extraction/convert
    uint64_t texturesMicros = 0;    // textures extraction/convert
    uint64_t totalMicros = 0;
};

// ---- geometry streaming (GPU upload overlap) ----
// Pool ranges the parser publishes as they finish filling, so a
// consumer can upload them while the rest of the file still parses
// (OBJ: meta at prefix, positions/indices per pass2 chunk, texcoords
// per pass3 chunk, seam splits at seamFill). Callbacks fire on parser
// worker threads and may run concurrently; they must only copy out or
// enqueue the bytes (the published memory stays valid until the
// publishing task returns). onMeta fires once, before the first
// onRange. data == nullptr in onRange means "zero-fill sizeBytes
// bytes" (models without texcoords upload a zero uv buffer).
enum class GeoRangeKind { Pos, Uv, Idx };
struct GeoStreamSink {
    std::function<void(size_t verts, size_t tris, bool hasUv)> onMeta;
    std::function<void(GeoRangeKind kind, size_t offsetBytes,
                       const void* data, size_t sizeBytes)> onRange;
};

// ---- base class: event plumbing, subclasses implement parsing ----
class ModelParser {
public:
    // Events carry their own data: each handler receives the packed
    // section of the result it just completed.
    using VerticesReady  = std::function<void(PackResult&, std::span<const PackMesh>)>;
    using MaterialsReady = std::function<void(PackResult&, std::span<const PackMaterial>)>;
    using TexturesReady  = std::function<void(PackResult&, std::span<const PackTexture>)>;
    using AllDone = std::function<void(PackResult&, bool ok, std::string_view error)>;
    using Progress = std::function<void(float percent)>;   // parse progress 0..100

    virtual ~ModelParser() = default;
    ModelParser() = default;
    ModelParser(const ModelParser&) = delete;
    ModelParser& operator=(const ModelParser&) = delete;

    // last parse error (empty when the load succeeded)
    const std::string& lastError() const;

    void setOnVerticesReady(VerticesReady cb);
    void setOnMaterialsReady(MaterialsReady cb);
    void setOnTexturesReady(TexturesReady cb);
    void setOnAllDone(AllDone cb);
    void setProgress(Progress cb);

    // Blocking load; events fire before returning.
    virtual bool load(std::string_view path) = 0;
    // Async load; events fire on the parser's executor threads.
    virtual void loadAsync(std::string_view path) = 0;

    virtual PackResult& result() = 0;

    // Attribute opt-out for consumers that never read a section (e.g.
    // the viewer uses positions/texcoords only). Parsers without the
    // section, or that cannot skip it, ignore the call.
    virtual void setWantNormals(bool want) { (void)want; }
    virtual void setWantTexcoords(bool want) { (void)want; }
    virtual void setWantPositions(bool want) { (void)want; }
    virtual void setWantTextureBytes(bool want) { (void)want; }

    // Request cancellation of an in-flight load (typically async). Parsers
    // check between stages and bail, firing onAllDone(false, "cancelled").
    void cancel();
    bool isCancelled() const;

    // Non-fatal issues from the last load (missing mtllib, unresolved
    // materials, texture load failures, ...). Empty on a clean load.
    const std::vector<std::string>& warnings() const;

    // Parsers and their helpers record non-fatal issues via this; surfaced
    // by warnings().
    void addWarning(std::string msg);

    // scene-entity factories shared by scene formats (pbrt, glTF): append
    // a defaulted entry and return a mutable reference to fill in
    static PackCamera& addCamera(PackResult& r, std::string_view name);
    static PackLight& addLight(PackResult& r, LightKind kind,
                               std::string_view name);

    // Free the geometry pools after the consumer has uploaded them to
    // the GPU (or copied the data out). PackMesh views into the pools
    // become dangling; only mesh metadata (bounds, material, triangle
    // counts) survives. Call only while no pool pointers are in use.
    // Saves ~3/4 of the parse-time pool on huge models.
    void releaseGeometry();

    // Streaming sink for GPU uploads (see GeoStreamSink); set before
    // load. Parsers that cannot stream (non-OBJ) ignore the call; the
    // sink is read-only while a parse runs, so the callbacks may fire
    // from any parser worker thread.
    void setGeoStream(GeoStreamSink sink) { geoStream_ = std::move(sink); }
    const GeoStreamSink& geoStream() const { return geoStream_; }

protected:
    // subclasses call these when a stage completes
    void fireVertices(PackResult& r, std::span<const PackMesh> m) const;
    void fireMaterials(PackResult& r, std::span<const PackMaterial> m) const;
    void fireTextures(PackResult& r, std::span<const PackTexture> t) const;
    void fireAllDone(PackResult& r, bool ok, std::string_view err) const;
    void fireProgress(float pct) const;

    // shared parser state: result storage + error text. Subclasses
    // allocate result_ at load start and report failures through
    // lastError_; the events above hand consumers views into it.
    std::shared_ptr<PackResult> result_;
    std::string lastError_;
    std::vector<std::string> warnings_;
    mutable std::mutex warnMx_;   // warnings may be recorded from workers
    std::atomic<bool> cancel_{false};

private:
    VerticesReady  onVerts_;
    MaterialsReady onMats_;
    TexturesReady  onTexs_;
    AllDone        onAll_;
    Progress       onProgress_;
    GeoStreamSink  geoStream_;   // set before load; read-only during it
};

// ============================================================
// ParserRegistry - format factory
//
// Concrete parsers register a creator under a format name (lowercase,
// no dot - "obj"); callers create by explicit format or by path
// extension. Registered creators must accept the ModelParser thread
// count as their only argument.
// ============================================================

class ParserRegistry {
public:
    using Creator = std::function<std::unique_ptr<ModelParser>(unsigned threads)>;

    static ParserRegistry& instance();

    ParserRegistry(const ParserRegistry&) = delete;
    ParserRegistry& operator=(const ParserRegistry&) = delete;

    // idempotent per format; later registrations replace earlier ones
    void add(std::string_view fmt, Creator creator);
    bool has(std::string_view fmt) const;

    // null when the format is unknown
    std::unique_ptr<ModelParser> create(std::string_view fmt,
                                        unsigned threads) const;
    // extension of `path` -> create; null when nothing matches
    std::unique_ptr<ModelParser> createForPath(std::string_view path,
                                               unsigned threads) const;

private:
    ParserRegistry();
    ~ParserRegistry();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================
// AssetPack - facade over ModelParser
//
// Owns one concrete parser selected via ParserRegistry (auto-detected
// from the loaded file's extension, or forced with setFormat) and
// forwards the unified event/load API.
// ============================================================

class AssetPack {
public:
    using VerticesReady  = ModelParser::VerticesReady;
    using MaterialsReady = ModelParser::MaterialsReady;
    using TexturesReady  = ModelParser::TexturesReady;
    using AllDone        = ModelParser::AllDone;
    using Progress       = ModelParser::Progress;

    explicit AssetPack(unsigned threads = 0);
    ~AssetPack();

    AssetPack(const AssetPack&) = delete;
    AssetPack& operator=(const AssetPack&) = delete;

    // Force the parser format for the next load ("obj" etc.); by
    // default the extension of the path passed to load() decides.
    void setFormat(std::string_view fmt);

    void setOnVerticesReady(VerticesReady cb);
    void setOnMaterialsReady(MaterialsReady cb);
    void setOnTexturesReady(TexturesReady cb);
    void setOnAllDone(AllDone cb);
    void setProgress(Progress cb);

    // Streaming geometry upload sink (see ModelParser::setGeoStream)
    void setGeoStream(GeoStreamSink sink);

    // Parser attribute opt-outs (applied to the parser the facade
    // creates; ignored by parsers without the section)
    void setWantNormals(bool want);
    void setWantTexcoords(bool want);
    void setWantPositions(bool want);
    void setWantTextureBytes(bool want);

    // Cancel an in-flight load and query non-fatal warnings.
    void cancel();
    const std::vector<std::string>& warnings();

    // Forwarded to the parser: frees the geometry pools once the
    // consumer has uploaded them (see ModelParser::releaseGeometry).
    void releaseGeometry();

    // Blocking load; events fire before returning.
    bool load(std::string_view path);
    // Async load; events fire on the parser's executor threads.
    void loadAsync(std::string_view path);

    PackResult& result();

private:
    bool ensureParser(std::string_view path);
    void attachCallbacks();   // hand buffered callbacks to the parser

    unsigned threads_;
    bool wantNormals_ = true;
    bool wantTexcoords_ = true;
    bool wantPositions_ = true;
    bool wantTextureBytes_ = false;
    std::string format_;
    std::unique_ptr<ModelParser> parser_;
    std::shared_ptr<PackResult> failResult_;   // returned when no parser is active

    VerticesReady  onVerts_;
    MaterialsReady onMats_;
    TexturesReady  onTexs_;
    AllDone        onAll_;
    Progress       onProgress_;
    GeoStreamSink  geoStream_;   // forwarded to the parser at load
};

} // namespace ap

// ============================================================
// printf-style logging: AP_LOG("import", "loaded %d meshes", n)
// ============================================================
#include <cstdio>
#define AP_LOG(stage, ...) do { \
    char _buf[512]; \
    std::snprintf(_buf, sizeof(_buf), __VA_ARGS__); \
    ::ap::logLine(::ap::LogLevel::Info, stage, _buf); \
} while (0)

#define AP_LOG_WARN(stage, ...) do { \
    char _buf[512]; \
    std::snprintf(_buf, sizeof(_buf), __VA_ARGS__); \
    ::ap::logLine(::ap::LogLevel::Warn, stage, _buf); \
} while (0)
