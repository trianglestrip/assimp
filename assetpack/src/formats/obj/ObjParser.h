#pragma once
// ============================================================
// ObjParser - OBJ/MTL parser (internal)
//
//   [mmap .obj] ─┬─ [geometry: chunk-parallel two-pass scan]
//   │            │     pass 1: count v/vn/vt/f + collect markers
//   │            │     pass 2: fill flat pools at prefix offsets
//   │            │        + texture-seam vertex splitting
//   │            │  └─► pack meshes (per-group views + bounds)
//   │            │       ──► onVerticesReady
//   │            └─ [mtllib -> mmap .mtl -> parse -> convert]
//   │                 └─► bind usemtl -> material indices
//   │                     ──► onMaterialsReady
//   └─ [textures from materials] ──► onTexturesReady
//   all ──► onAllDone
//
// Registered as "obj" with ap::ParserRegistry (see registerObjParser).
// ============================================================

#include <memory>
#include <string_view>

#include <assetpack/AssetPack.h>

namespace tf { class Executor; }

namespace ap {

// registers the OBJ/MTL parser with ParserRegistry; called by the
// AssetPack facade so static libraries keep this unit linked
void registerObjParser();

namespace obj { class GeometryStage; }

class ObjParser : public ModelParser {
public:
    explicit ObjParser(unsigned threads = 0);
    ~ObjParser() override;

    // Opt-out for consumers that never use normals: skips the vn pool
    // parse, the per-vertex normal expansion arrays and the expanded
    // output. Default true (behavior unchanged).
    void setWantNormals(bool want) override { wantNormals_ = want; }
    void setWantTexcoords(bool want) override { wantUv_ = want; }
    void setWantTextureBytes(bool want) override { wantTextureBytes_ = want; }

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;
    PackResult& result() override;

    // internal: the src/obj stage helpers fire these through the
    // parser (re-exposed here because they sit in the shared base)
    using ModelParser::fireProgress;
    using ModelParser::fireVertices;
    using ModelParser::fireMaterials;
    using ModelParser::fireTextures;
    using ModelParser::fireAllDone;

private:
    bool runGraph(std::string_view path, bool async);

    struct FlowDeleter;
    std::unique_ptr<tf::Executor> executor_;
    std::shared_ptr<void> flow_;      // tf::Taskflow, type-erased
    bool wantNormals_ = true;
    bool wantUv_ = true;
    bool wantTextureBytes_ = false;
};

} // namespace ap
