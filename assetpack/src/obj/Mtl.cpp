#include "Mtl.h"
#include <assetpack/AssetPack.h>

#include "detail/TextScan.h"

#include <chrono>
#include <cstring>
#include <unordered_map>

namespace ap::obj {

namespace {

using Clock = ap::Clock;

inline int slotToTexType(std::string_view slot) {
    if (slot == "map_Kd")   return TexDiffuse;
    if (slot == "map_Ka")   return TexAmbient;
    if (slot == "map_Ks")   return TexSpecular;
    if (slot == "map_d")    return TexOpacity;
    if (slot == "map_Bump") return TexNormal;
    return TexDiffuse;
}

} // namespace

void parseMtl(const std::shared_ptr<PackResult>& result,
              std::shared_ptr<std::vector<RawMaterial>> rawMats) {
    if (!result->mtlFile) return;
    const auto tStart = Clock::now();
    AP_LOG("materials", "begin");

    const std::string_view text = result->mtlFile->text();
    const char* p = text.data();
    const char* end = p + text.size();

    auto cur = [&]() -> RawMaterial* {
        return rawMats->empty() ? nullptr : &rawMats->back();
    };
    auto setPath = [](std::string_view& dst,
                      const char* tok, size_t len) {
        // strip surrounding quotes when present
        if (len >= 2 && tok[0] == '"') { ++tok; len -= 2; }
        dst = std::string_view(tok, len);
    };

    while (p < end) {
        const char* eol = static_cast<const char*>(memchr(p, '\n', size_t(end - p)));
        if (!eol) eol = end;
        const char* tok; size_t len;
        const char* q = p;

        if (eol - p > 7 && memcmp(p, "newmtl ", 7) == 0) {
            if (nextToken(p + 7, eol, tok, len)) {
                rawMats->emplace_back();
                cur()->name = std::string_view(tok, len);
            }
        } else if (RawMaterial* m = cur()) {
            q = p;
            if (eol - p > 3 && memcmp(p, "Kd ", 3) == 0) {
                q = p + 3;
                for (int i = 0; i < 3; ++i) {
                    if (!(q = nextToken(q, eol, tok, len))) break;
                    m->Kd[i] = parseFloat(tok, len);
                }
            } else if (eol - p > 3 && memcmp(p, "Ka ", 3) == 0) {
                q = p + 3;
                for (int i = 0; i < 3; ++i) {
                    if (!(q = nextToken(q, eol, tok, len))) break;
                    m->Ka[i] = parseFloat(tok, len);
                }
            } else if (eol - p > 3 && memcmp(p, "Ks ", 3) == 0) {
                q = p + 3;
                for (int i = 0; i < 3; ++i) {
                    if (!(q = nextToken(q, eol, tok, len))) break;
                    m->Ks[i] = parseFloat(tok, len);
                }
            } else if (eol - p > 3 && memcmp(p, "Ns ", 3) == 0) {
                if (nextToken(p + 3, eol, tok, len)) m->Ns = parseFloat(tok, len);
            } else if (eol - p > 2 && memcmp(p, "d ", 2) == 0) {
                if (nextToken(p + 2, eol, tok, len)) m->d = parseFloat(tok, len);
            } else if (eol - p > 6 && memcmp(p, "illum ", 6) == 0) {
                if (nextToken(p + 6, eol, tok, len))
                    m->illum = int32_t(parseInt(tok, len));
            } else if (eol - p > 7 && memcmp(p, "map_Kd ", 7) == 0) {
                if (nextToken(p + 7, eol, tok, len)) setPath(m->mapKd, tok, len);
            } else if (eol - p > 7 && memcmp(p, "map_Ka ", 7) == 0) {
                if (nextToken(p + 7, eol, tok, len)) setPath(m->mapKa, tok, len);
            } else if (eol - p > 7 && memcmp(p, "map_Ks ", 7) == 0) {
                if (nextToken(p + 7, eol, tok, len)) setPath(m->mapKs, tok, len);
            } else if (eol - p > 6 && memcmp(p, "map_d ", 6) == 0) {
                if (nextToken(p + 6, eol, tok, len)) setPath(m->mapD, tok, len);
            } else if (eol - p > 9 && memcmp(p, "map_Bump ", 9) == 0) {
                if (nextToken(p + 9, eol, tok, len)) setPath(m->mapBump, tok, len);
            } else if (eol - p > 5 && memcmp(p, "bump ", 5) == 0) {
                if (nextToken(p + 5, eol, tok, len)) setPath(m->mapBump, tok, len);
            }
        }
        p = eol + 1;
    }

    // convert to the public material form (Kd -> diffuse etc.)
    result->materials.reserve(rawMats->size());
    for (const RawMaterial& m : *rawMats) {
        PackMaterial pm;
        pm.name = m.name;
        pm.diffuse[0] = m.Kd[0]; pm.diffuse[1] = m.Kd[1];
        pm.diffuse[2] = m.Kd[2]; pm.diffuse[3] = 1.f;
        pm.ambient[0] = m.Ka[0]; pm.ambient[1] = m.Ka[1];
        pm.ambient[2] = m.Ka[2]; pm.ambient[3] = 1.f;
        pm.specular[0] = m.Ks[0]; pm.specular[1] = m.Ks[1];
        pm.specular[2] = m.Ks[2]; pm.specular[3] = 1.f;
        pm.opacity = m.d;
        pm.shininess = m.Ns;
        auto add = [&](const std::string_view& path, int type) {
            if (!path.empty()) pm.textures.push_back({ type, 0, path });
        };
        add(m.mapKd,   TexDiffuse);
        add(m.mapKa,   TexAmbient);
        add(m.mapKs,   TexSpecular);
        add(m.mapD,    TexOpacity);
        add(m.mapBump, TexNormal);
        result->materials.push_back(pm);
    }

    result->materialsMicros = microsSince(tStart);
    AP_LOG("materials", "done in %llu us: %zu materials",
           static_cast<unsigned long long>(result->materialsMicros),
           result->materials.size());
}

void buildTextureRefs(
    ObjParser& owner, const std::shared_ptr<PackResult>& result,
    const std::shared_ptr<std::vector<RawMaterial>>& rawMats) {
    if (!result->mtlFile) return;
    const auto tStart = Clock::now();

    struct Ref { std::string_view path; std::string_view slot; };
    std::vector<Ref> refs;
    for (const RawMaterial& m : *rawMats) {
        auto add = [&](const std::string_view& p, std::string_view slot) {
            if (!p.empty()) refs.push_back({ p, slot });
        };
        add(m.mapKd,   "map_Kd");
        add(m.mapKa,   "map_Ka");
        add(m.mapKs,   "map_Ks");
        add(m.mapD,    "map_d");
        add(m.mapBump, "map_Bump");
    }

    result->textures.reserve(refs.size());
    for (const Ref& r : refs) {
        PackTexture pt;
        pt.path = r.path;
        pt.type = slotToTexType(r.slot);
        pt.slot = 0;
        pt.embedded = false;
        pt.resolvedPath = result->sourceDir.empty()
            ? std::string(r.path)
            : result->sourceDir + "/" + std::string(r.path);
        result->textures.push_back(pt);
    }

    result->texturesMicros = microsSince(tStart);
    AP_LOG("textures", "done in %llu us: %zu refs",
           static_cast<unsigned long long>(result->texturesMicros),
           result->textures.size());

    owner.fireTextures(*result, result->textures);
    AP_LOG("event", "onTexturesReady fired (%zu refs)",
           result->textures.size());
}

void bindMaterials(
    ObjParser& owner, const std::shared_ptr<PackResult>& result,
    const std::shared_ptr<std::vector<std::string_view>>& mtlNames) {
    const auto tStart = Clock::now();
    std::unordered_map<std::string_view, int> byName;
    byName.reserve(result->materials.size() * 2);
    for (int i = 0; i < int(result->materials.size()); ++i)
        byName.emplace(result->materials[size_t(i)].name, i);

    for (PackMesh& m : result->meshes) {
        if (m.materialIndex < 0) continue;
        const std::string_view usemtl = (*mtlNames)[size_t(m.materialIndex)];
        const auto it = byName.find(usemtl);
        m.materialIndex = it == byName.end() ? -1 : it->second;
    }
    AP_LOG("materials", "bound usemtl -> material indices in %llu us",
           static_cast<unsigned long long>(microsSince(tStart)));

    owner.fireMaterials(*result, result->materials);
    AP_LOG("event", "onMaterialsReady fired (%zu materials)",
           result->materials.size());
}

} // namespace ap::obj
