#pragma once
// ============================================================
// ModelParser - base class for custom model parsers
//
// Every parser derives from this interface and emits the same
// data-carrying completion events:
//
//   [mmap source file + auxiliaries]
//     ├─► [geometry parse]   ──► onVerticesReady(span<PackMesh>)
//     ├─► [materials parse]  ──► onMaterialsReady(span<PackMaterial>)
//     └─► [textures parse]   ──► onTexturesReady(span<PackTexture>)
//   all ──► onAllDone
//
// Concrete parsers (ap::ObjParser for OBJ/MTL today) implement the
// parse stages; the event plumbing lives here so consumers only ever
// talk to ModelParser.
//
// Zero-extra-copy: file contents are memory-mapped and parsed in
// place; vertex positions/normals/texcoords and the packed index
// array are views into the result's arrays (kept alive by it).
// ============================================================

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <assetpack/MappedFile.h>

namespace ap {

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
    std::string_view name;           // 'o'/'g' name (view into the file mapping)
    int32_t  materialIndex = -1;     // index into PackResult::materials
    // zero-copy views into the result's global pools; the index array
    // is relative to those pools (OBJ shares one vertex pool)
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

    // global pools backing the PackMesh views (OBJ: one shared pool)
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
    ModelParser() = default;                        // deleted copy ctor would
    ModelParser(const ModelParser&) = delete;       // suppress the default one
    ModelParser& operator=(const ModelParser&) = delete;

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

private:
    VerticesReady  onVerts_;
    MaterialsReady onMats_;
    TexturesReady  onTexs_;
    AllDone        onAll_;
    Progress       onProgress_;
};

} // namespace ap
