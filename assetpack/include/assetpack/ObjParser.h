#pragma once
// ============================================================
// ObjParser - OBJ/MTL parser derived from ModelParser
//
//   [mmap .obj] ─┬─ [geometry: chunk-parallel two-pass scan]
//   │            │     pass 1: count v/vn/vt/f + collect markers
//   │            │     pass 2: fill flat pools at prefix offsets
//   │            │  └─► pack meshes (per-group views + bounds)
//   │            │       ──► onVerticesReady
//   │            └─ [mtllib -> mmap .mtl -> parse -> convert]
//   │                 └─► bind usemtl -> material indices
//   │                     ──► onMaterialsReady
//   └─ [textures from materials] ──► onTexturesReady
//   all ──► onAllDone
//
// Geometry pass 1 and the .mtl parse start at the same instant and
// run on separate workers; nothing waits for the whole file first.
// All string_views point into the mappings (kept alive by result).
//
// ponytail: v/vn/vt/f(usemtl/o/g/mtllib/s), polygon fan triangulation,
// negative indices, CRLF. Skips: continuation lines ('\'),
// curves/surfaces, smoothing groups.
// ============================================================

#include <memory>
#include <string>
#include <string_view>

#include <assetpack/ModelParser.h>

namespace tf { class Executor; }

namespace ap {

class ObjParser : public ModelParser {
public:
    explicit ObjParser(unsigned threads = 0);
    ~ObjParser() override;

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;
    PackResult& result() override;

private:
    bool runGraph(std::string_view path, bool async);

    struct FlowDeleter;
    std::unique_ptr<tf::Executor> executor_;
    std::shared_ptr<void> flow_;      // tf::Taskflow, type-erased
    std::shared_ptr<PackResult> result_;
    std::string lastError_;
};

} // namespace ap
