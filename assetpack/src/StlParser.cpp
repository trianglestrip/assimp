#include "assetpack/AssetPack.h"

#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ap {

class StlParser final : public ModelParser {
public:
    explicit StlParser(unsigned /*threads*/) {}

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;
    PackResult& result() override { return *result_; }

private:
    void parse(std::string_view path);
    void finish();
};

void StlParser::finish() {
    PackResult& result = *result_;
    if (result.positions.empty()) {
        fireProgress(100.f);
        fireAllDone(result, true, {});
        return;
    }

    PackMesh pm;
    pm.name = std::string_view("stl");
    pm.materialIndex = -1;
    pm.positions = result.positions;
    pm.normals = result.normals;
    pm.texcoords = result.texcoords;
    pm.indices = result.posIndices;

    float bmin[3] = { +1e30f, +1e30f, +1e30f };
    float bmax[3] = { -1e30f, -1e30f, -1e30f };
    const float* p = result.positions.data();
    for (size_t i = 0; i < result.positions.size(); i += 3) {
        for (int k = 0; k < 3; ++k) {
            const float v = p[i + k];
            if (v < bmin[k]) bmin[k] = v;
            if (v > bmax[k]) bmax[k] = v;
        }
    }
    for (int k = 0; k < 3; ++k) {
        pm.boundsMin[k] = bmin[k];
        pm.boundsMax[k] = bmax[k];
    }

    result.meshes.push_back(pm);

    fireProgress(100.f);
    fireVertices(result, result.meshes);
    fireMaterials(result, {});
    fireTextures(result, {});
    fireAllDone(result, true, {});
}

void StlParser::parse(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);

    PackResult& result = *result_;
    result.objFile = MappedFile::openShared(path);
    if (!result.objFile) {
        lastError_ = std::string("cannot open file: ") + std::string(path);
        fireProgress(100.f);
        fireAllDone(result, false, lastError_);
        return;
    }

    const auto slash = path.find_last_of("/\\");
    if (slash != std::string_view::npos)
        result.sourceDir = std::string(path.substr(0, slash));

    const std::span<const std::byte> bytes = result.objFile->bytes();
    const size_t size = bytes.size();
    const std::string_view text = result.objFile->text();

    fireProgress(5.f);

    // ---- detect binary vs ascii ----
    bool isBinary = false;
    if (size >= 84) {
        uint32_t tri = 0;
        std::memcpy(&tri, bytes.data() + 80, sizeof(uint32_t));
        if (tri != 0 && size == size_t(84) + size_t(50) * tri) {
            // a sane count that fits in the file and is not absurd
            const uint64_t maxTri = (size - 84) / 50;
            if (tri <= maxTri)
                isBinary = true;
        }
    }

    if (isBinary) {
        uint32_t tri = 0;
        std::memcpy(&tri, bytes.data() + 80, sizeof(uint32_t));
        const std::byte* p = bytes.data() + 84;
        const std::byte* end = bytes.data() + bytes.size();
        result.positions.reserve(size_t(tri) * 9);
        result.normals.reserve(size_t(tri) * 9);
        result.posIndices.reserve(size_t(tri) * 3);
        for (uint32_t t = 0; t < tri; ++t) {
            if (isCancelled()) { lastError_ = "cancelled"; fireAllDone(result, false, lastError_); return; }
            if (p + 50 > end) break;
            float nx, ny, nz;
            std::memcpy(&nx, p + 0, 4);
            std::memcpy(&ny, p + 4, 4);
            std::memcpy(&nz, p + 8, 4);
            const size_t base = result.positions.size() / 3;
            for (int v = 0; v < 3; ++v) {
                float x, y, z;
                std::memcpy(&x, p + 12 + v * 12 + 0, 4);
                std::memcpy(&y, p + 12 + v * 12 + 4, 4);
                std::memcpy(&z, p + 12 + v * 12 + 8, 4);
                result.positions.push_back(x);
                result.positions.push_back(y);
                result.positions.push_back(z);
                // replicate the per-facet normal to each of its vertices
                result.normals.push_back(nx);
                result.normals.push_back(ny);
                result.normals.push_back(nz);
            }
            result.posIndices.push_back(static_cast<uint32_t>(base + 0));
            result.posIndices.push_back(static_cast<uint32_t>(base + 1));
            result.posIndices.push_back(static_cast<uint32_t>(base + 2));
            p += 50;
        }
    } else {
        // ---- ASCII ----
        size_t off = 0;
        const size_t n = text.size();
        size_t idx = 0;
        bool haveNormal = false;
        float curN[3] = { 0, 0, 0 };
        while (off < n) {
            if (isCancelled()) { lastError_ = "cancelled"; fireAllDone(result, false, lastError_); return; }
            size_t eol = text.find('\n', off);
            if (eol == std::string_view::npos) eol = n;
            std::string_view line = text.substr(off, eol - off);
            off = eol + 1;

            size_t s = 0;
            while (s < line.size() && (line[s] == ' ' || line[s] == '\t' || line[s] == '\r'))
                ++s;
            std::string_view l = line.substr(s);
            if (l.empty()) continue;

            if (l.substr(0, 13) == "facet normal ") {
                float a, b, c;
                char* ep1 = nullptr; char* ep2 = nullptr; char* ep3 = nullptr;
                const char* str = l.data() + 13;
                a = std::strtof(str, &ep1);
                b = std::strtof(ep1, &ep2);
                c = std::strtof(ep2, &ep3);
                curN[0] = a; curN[1] = b; curN[2] = c;
                haveNormal = (ep1 != str);
                continue;
            }
            if (l.substr(0, 6) == "vertex") {
                float x, y, z;
                char* ep1 = nullptr; char* ep2 = nullptr; char* ep3 = nullptr;
                const char* str = l.data() + 6;
                x = std::strtof(str, &ep1);
                y = std::strtof(ep1, &ep2);
                z = std::strtof(ep2, &ep3);
                const size_t base = result.positions.size() / 3;
                result.positions.push_back(x);
                result.positions.push_back(y);
                result.positions.push_back(z);
                if (haveNormal) {
                    result.normals.push_back(curN[0]);
                    result.normals.push_back(curN[1]);
                    result.normals.push_back(curN[2]);
                }
                result.posIndices.push_back(static_cast<uint32_t>(base));
                ++idx;
                if (idx % 3 == 0) haveNormal = false;
                continue;
            }
        }
        if (result.normals.size() != result.positions.size())
            result.normals.clear();
    }

    fireProgress(90.f);
    finish();
}

bool StlParser::load(std::string_view path) {
    parse(path);
    return !lastError_.empty() ? false : (result_ && result_->objFile != nullptr);
}

void StlParser::loadAsync(std::string_view path) {
    // NOTE: the caller must keep this parser alive until onAllDone fires;
    // the detached thread captures `this`, so a destroyed parser is UB.
    auto pathStr = std::make_shared<std::string>(path);
    std::thread([this, pathStr]() {
        parse(*pathStr);
    }).detach();
}

void registerStlParser() {
    static const bool once = [] {
        ParserRegistry::instance().add(
            "stl", [](unsigned threads) -> std::unique_ptr<ModelParser> {
                return std::make_unique<StlParser>(threads);
            });
        return true;
    }();
    (void)once;
}

} // namespace ap
