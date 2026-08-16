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

#include <cstdint>
#include <functional>
#include <memory>
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
    std::span<const std::byte> bytes() const { return bytes_; }

    bool isOpen() const { return bytes_.data() != nullptr; }
    size_t size() const { return bytes_.size(); }

    // Convenience: reinterpret the mapping as chars
    std::string_view text() const {
        return { reinterpret_cast<const char*>(bytes_.data()), bytes_.size() };
    }

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

inline const char* texTypeName(int t) {
    switch (t) {
    case TexDiffuse:  return "diffuse";
    case TexAmbient:  return "ambient";
    case TexSpecular: return "specular";
    case TexEmissive: return "emissive";
    case TexOpacity:  return "opacity";
    case TexNormal:   return "normal";
    default:          return "?";
    }
}

// ---- vertices section (one entry per mesh/group) ----
struct PackMesh {
    std::string_view name;           // group/object name (file mapping view)
    int32_t  materialIndex = -1;     // index into PackResult::materials
    // zero-copy views into the result's global pools; the index array
    // is relative to those pools
    std::span<const float>   positions;   // 3 floats per vertex
    std::span<const float>   normals;     // 3 floats per vertex (empty when absent)
    std::span<const float>   texcoords;   // 2 floats per vertex (empty when absent)
    std::span<const uint32_t> indices;    // 3 per triangle (packed)
    float boundsMin[3] = {0,0,0};
    float boundsMax[3] = {0,0,0};

    uint32_t vertexCount()  const { return uint32_t(positions.size() / 3); }
    uint32_t triangleCount() const { return uint32_t(indices.size() / 3); }
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
    std::string sourceDir;

    // global pools backing the PackMesh views
    std::vector<float>    positions;    // 3 floats per vertex
    std::vector<float>    normals;      // 3 floats per vertex (empty when absent)
    std::vector<float>    texcoords;    // 2 floats per vertex (empty when absent)
    std::vector<uint32_t> posIndices;   // 3 per triangle (packed)

    std::vector<PackMesh>      meshes;
    std::vector<PackMaterial>  materials;
    std::vector<PackTexture>   textures;

    // per-stage wall times (microseconds)
    uint64_t importMicros = 0;      // main-file parse (geometry)
    uint64_t verticesMicros = 0;    // packing meshes (per-mesh views + bounds)
    uint64_t materialsMicros = 0;   // materials extraction/convert
    uint64_t texturesMicros = 0;    // textures extraction/convert
    uint64_t totalMicros = 0;
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
    const std::string& lastError() const { return lastError_; }

    void setOnVerticesReady(VerticesReady cb)   { onVerts_ = std::move(cb); }
    void setOnMaterialsReady(MaterialsReady cb) { onMats_  = std::move(cb); }
    void setOnTexturesReady(TexturesReady cb)   { onTexs_  = std::move(cb); }
    void setOnAllDone(AllDone cb)               { onAll_   = std::move(cb); }
    void setProgress(Progress cb)               { onProgress_ = std::move(cb); }

    // Blocking load; events fire before returning.
    virtual bool load(std::string_view path) = 0;
    // Async load; events fire on the parser's executor threads.
    virtual void loadAsync(std::string_view path) = 0;

    virtual PackResult& result() = 0;

    // Attribute opt-out for consumers that never read a section (e.g.
    // the viewer uses positions/texcoords only). Parsers without the
    // section, or that cannot skip it, ignore the call.
    virtual void setWantNormals(bool want) { (void)want; }

protected:
    // subclasses call these when a stage completes
    void fireVertices(PackResult& r, std::span<const PackMesh> m) const {
        if (onVerts_) onVerts_(r, m);
    }
    void fireMaterials(PackResult& r, std::span<const PackMaterial> m) const {
        if (onMats_) onMats_(r, m);
    }
    void fireTextures(PackResult& r, std::span<const PackTexture> t) const {
        if (onTexs_) onTexs_(r, t);
    }
    void fireAllDone(PackResult& r, bool ok, std::string_view err) const {
        if (onAll_) onAll_(r, ok, err);
    }
    void fireProgress(float pct) const {
        if (onProgress_) onProgress_(pct);
    }

    // shared parser state: result storage + error text. Subclasses
    // allocate result_ at load start and report failures through
    // lastError_; the events above hand consumers views into it.
    std::shared_ptr<PackResult> result_;
    std::string lastError_;

private:
    VerticesReady  onVerts_;
    MaterialsReady onMats_;
    TexturesReady  onTexs_;
    AllDone        onAll_;
    Progress       onProgress_;
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

    void setOnVerticesReady(VerticesReady cb)   { onVerts_ = std::move(cb); }
    void setOnMaterialsReady(MaterialsReady cb) { onMats_  = std::move(cb); }
    void setOnTexturesReady(TexturesReady cb)   { onTexs_  = std::move(cb); }
    void setOnAllDone(AllDone cb)               { onAll_   = std::move(cb); }
    void setProgress(Progress cb)               { onProgress_ = std::move(cb); }

    // Parser attribute opt-outs (applied to the parser the facade
    // creates; ignored by parsers without the section)
    void setWantNormals(bool want)              { wantNormals_ = want; }

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
    std::string format_;
    std::unique_ptr<ModelParser> parser_;

    VerticesReady  onVerts_;
    MaterialsReady onMats_;
    TexturesReady  onTexs_;
    AllDone        onAll_;
    Progress       onProgress_;
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
