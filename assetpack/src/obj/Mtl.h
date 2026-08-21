#pragma once
// ============================================================
// Mtl - material/texture stages of the OBJ parse (internal):
// .mtl parsing, texture-reference collection, usemtl -> material
// index binding.
// ============================================================

#include <memory>
#include <string_view>
#include <vector>

#include <assetpack/AssetPack.h>

#include "ObjParser.h"   // fires stage events (re-exposed internal API)

namespace ap::obj {

// intermediate .mtl record (not part of the public result); the
// string_views point into the mtl mapping kept alive by PackResult
struct RawMaterial {
    std::string_view name;          // newmtl
    float Kd[3] = {0.8f, 0.8f, 0.8f};
    float Ka[3] = {0.f, 0.f, 0.f};
    float Ks[3] = {0.f, 0.f, 0.f};
    float Ns = 10.f;
    float d = 1.f;                  // dissolve (alpha)
    int32_t illum = 1;
    float Ke[3] = {0.f, 0.f, 0.f};  // emissive color
    float Tr = 0.f;                 // transmission (0 opaque, 1 transparent)
    bool hasTr = false;             // set true only when a Tr token is parsed
    // texture refs (views into mtl mapping; empty when absent)
    std::string_view mapKd, mapKa, mapKs, mapD, mapBump, mapKe, mapTr;
};

// parse the mapped .mtl into raw records + the public material list;
// fires nothing (materials fire after the usemtl bind)
void parseMtl(const std::shared_ptr<PackResult>& result,
              std::shared_ptr<std::vector<RawMaterial>> rawMats);

// collect texture references from the raw materials into the public
// texture list; fires onTexturesReady through the owner
void buildTextureRefs(ObjParser& owner,
                      const std::shared_ptr<PackResult>& result,
                      const std::shared_ptr<std::vector<RawMaterial>>& rawMats,
                      bool loadBytes = false);

// rewrite each mesh's temporary first-seen materialIndex into the
// real materials-array index; fires onMaterialsReady
void bindMaterials(ObjParser& owner,
                   const std::shared_ptr<PackResult>& result,
                   const std::shared_ptr<std::vector<std::string_view>>& mtlNames);

} // namespace ap::obj
