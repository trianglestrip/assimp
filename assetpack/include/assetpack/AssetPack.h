#pragma once
// ============================================================
// AssetPack - facade over ap::ModelParser
//
// AssetPack no longer embeds any format-specific parsing. It owns
// one concrete ModelParser (ap::ObjParser for OBJ/MTL today) and
// forwards the unified event/load API. Swap in another parser by
// constructing AssetPack with a different ModelParser.
//
//   [mmap source file + auxiliaries]
//     ├─► [geometry parse]   ──► onVerticesReady(span<PackMesh>)
//     ├─► [materials parse]  ──► onMaterialsReady(span<PackMaterial>)
//     └─► [textures parse]   ──► onTexturesReady(span<PackTexture>)
//   all ──► onAllDone
//
// Each completion event carries *its own data* (a span over the
// packed section), so a consumer can immediately use vertices,
// materials and textures independently, in parallel.
//
// Zero-extra-copy: file contents are memory-mapped and parsed in
// place; vertex positions/normals/texcoords and the packed index
// array are views into the result's arrays (kept alive by it).
// ============================================================

#include <memory>
#include <span>
#include <string_view>

#include <assetpack/ModelParser.h>

namespace ap {

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

    void setOnVerticesReady(VerticesReady cb)   { parser_->setOnVerticesReady(std::move(cb)); }
    void setOnMaterialsReady(MaterialsReady cb) { parser_->setOnMaterialsReady(std::move(cb)); }
    void setOnTexturesReady(TexturesReady cb)   { parser_->setOnTexturesReady(std::move(cb)); }
    void setOnAllDone(AllDone cb)               { parser_->setOnAllDone(std::move(cb)); }
    void setProgress(Progress cb)               { parser_->setProgress(std::move(cb)); }

    // Blocking load; events fire before returning.
    bool load(std::string_view path)       { return parser_->load(path); }
    // Async load; events fire on the parser's executor threads.
    void loadAsync(std::string_view path)  { parser_->loadAsync(path); }

    PackResult& result()                   { return parser_->result(); }

private:
    std::unique_ptr<ModelParser> parser_;
};

} // namespace ap
