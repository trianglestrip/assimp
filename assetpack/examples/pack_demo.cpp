// Demo: AssetPack - mmap + custom task-graph parser (ObjParser) with
// data-carrying stage events.
// Usage: pack_demo <model.obj>
//
// Shows each completion event receiving its own data:
//   vertices  -> span<PackMesh>    (positions/normals/texcoords/indices)
//   materials -> span<PackMaterial> (full attribute set + texture slots)
//   textures  -> span<PackTexture>  (path / embedded bytes / resolved path)

#include <assetpack/AssetPack.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

static std::string sv(std::string_view v) { return std::string(v); }

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: pack_demo <model>\n");
        return 1;
    }

    ap::AssetPack pack;

    pack.setProgress([](float pct) {
        std::printf("  [progress] import %g%%\n", pct);
    });

    // vertices event: carries positions/normals/texcoords/indices
    pack.setOnVerticesReady([](ap::PackResult& r,
                               std::span<const ap::PackMesh> meshes) {
        std::printf("  [event] vertices ready: %zu meshes\n", meshes.size());
        for (size_t i = 0; i < meshes.size() && i < 5; ++i) {
            const auto& m = meshes[i];
            std::printf("    mesh %zu: %-24s v=%u tri=%u mat=%d nrm=%s uv=%s "
                        "bounds=(%.2f %.2f %.2f)-(%.2f %.2f %.2f)\n",
                        i, sv(m.name).c_str(), m.vertexCount(), m.triangleCount(),
                        m.materialIndex, m.normals.empty() ? "-" : "yes",
                        m.texcoords.empty() ? "-" : "yes",
                        m.boundsMin[0], m.boundsMin[1], m.boundsMin[2],
                        m.boundsMax[0], m.boundsMax[1], m.boundsMax[2]);
        }
        // first triangle's indices as a sample of the packed data
        if (!meshes.empty() && !meshes[0].indices.empty()) {
            const auto& idx = meshes[0].indices;
            std::printf("    mesh0 first tri indices: %u %u %u\n",
                        idx[0], idx[1], idx[2]);
        }
    });

    // materials event: carries the full attribute set
    pack.setOnMaterialsReady([](ap::PackResult& r,
                                std::span<const ap::PackMaterial> mats) {
        std::printf("  [event] materials ready: %zu\n", mats.size());
        for (size_t i = 0; i < mats.size() && i < 5; ++i) {
            const auto& m = mats[i];
            std::printf("    mat %zu: %-24s Kd=(%.2f %.2f %.2f %.2f) "
                        "Ks=(%.2f %.2f %.2f) opacity=%.2f shininess=%.1f "
                        "twoSided=%d slots=%zu\n",
                        i, sv(m.name).c_str(),
                        m.diffuse[0], m.diffuse[1], m.diffuse[2], m.diffuse[3],
                        m.specular[0], m.specular[1], m.specular[2],
                        m.opacity, m.shininess, int(m.twoSided), m.textures.size());
            for (const auto& t : m.textures)
                std::printf("      [%s#%u] %s\n",
                            ap::texTypeName(t.type), t.slot, sv(t.path).c_str());
        }
    });

    // textures event: carries path / embedded bytes / resolved path
    pack.setOnTexturesReady([](ap::PackResult& r,
                               std::span<const ap::PackTexture> texs) {
        std::printf("  [event] textures ready: %zu\n", texs.size());
        for (size_t i = 0; i < texs.size() && i < 8; ++i) {
            const auto& t = texs[i];
            std::printf("    tex %zu: [%s#%u] %-36s %s%s\n",
                        i, ap::texTypeName(t.type), t.slot, sv(t.path).c_str(),
                        t.embedded ? "embedded" : "external",
                        t.embedded ? "" : sv(" -> " + t.resolvedPath).c_str());
            if (t.embedded)
                std::printf("      embedded bytes: %zu\n", t.byteSize);
        }
    });

    pack.setOnAllDone([](ap::PackResult& r, bool ok, std::string_view err) {
        std::printf("  [event] all done: ok=%d err=%s\n", int(ok), sv(err).c_str());
    });

    AP_LOG("main", "loading %s", argv[1]);
    const auto tBegin = std::chrono::steady_clock::now();
    const bool ok = pack.load(argv[1]);
    const double wallMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - tBegin).count();
    AP_LOG("main", "load() returned %d", int(ok));

    // benchmark: append this run's per-stage timings to benchmark.md
    if (ok) {
        ap::PackResult& r = pack.result();
        std::FILE* f = std::fopen("benchmark.md", "ab");
        if (f) {
            std::fseek(f, 0, SEEK_END);
            if (std::ftell(f) == 0) {
                std::fprintf(f, "# assetpack benchmark\n\n"
                                "每次运行自动追加一行（wall ms，进程总时长含事件回调）。\n\n"
                                "| time | model | import | vertices | materials | textures | total | wall | verts | tris | mats | texs |\n"
                                "|------|-------|--------|----------|-----------|----------|-------|------|------|------|------|------|\n");
            }
            const char* model = std::strrchr(argv[1], '/');
            const char* model2 = std::strrchr(argv[1], '\\');
            const char* name = model2 && (!model || model2 > model) ? model2 + 1
                            : model ? model + 1 : argv[1];
            // OBJ meshes share one global vertex pool, so count the
            // pool once instead of summing per-mesh vertexCount()
            uint64_t verts = r.positions.size() / 3, tris = 0;
            for (const auto& m : r.meshes) tris += m.triangleCount();
            const auto now = std::time(nullptr);
            char ts[32];
            std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
            std::fprintf(f,
                "| %s | %s | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %llu | %llu | %zu | %zu |\n",
                ts, name,
                r.importMicros / 1000.0, r.verticesMicros / 1000.0,
                r.materialsMicros / 1000.0, r.texturesMicros / 1000.0,
                r.totalMicros / 1000.0, wallMs,
                static_cast<unsigned long long>(verts),
                static_cast<unsigned long long>(tris),
                r.materials.size(), r.textures.size());
            std::fclose(f);
        }
    }
    return ok ? 0 : 1;
}
