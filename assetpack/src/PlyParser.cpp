#include "assetpack/AssetPack.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ap {

namespace {

enum VRole { R_NONE, R_PX, R_PY, R_PZ, R_NX, R_NY, R_NZ };

struct VProp {
    VRole role = R_NONE;
    bool isList = false;
    std::string type;
    std::string listCountType;
    std::string listIdxType;
};

int scalarSize(std::string_view type) {
    if (type == "char" || type == "uchar") return 1;
    if (type == "short" || type == "ushort") return 2;
    if (type == "int" || type == "uint" || type == "float") return 4;
    if (type == "double") return 8;
    return 0;
}

// read a little-endian scalar of `type` from `cur`, advance `cur`.
// x86 is little-endian so a reinterpret_cast is correct for integers;
// floats/doubles are copied byte-for-byte.
bool readScalar(const std::byte*& cur, const std::byte* end,
                std::string_view type, float& out) {
    const int sz = scalarSize(type);
    if (sz == 0 || cur + sz > end) return false;
    if (type == "char") {
        out = static_cast<float>(*reinterpret_cast<const int8_t*>(cur));
    } else if (type == "uchar") {
        out = static_cast<float>(*reinterpret_cast<const uint8_t*>(cur));
    } else if (type == "short") {
        out = static_cast<float>(*reinterpret_cast<const int16_t*>(cur));
    } else if (type == "ushort") {
        out = static_cast<float>(*reinterpret_cast<const uint16_t*>(cur));
    } else if (type == "int") {
        out = static_cast<float>(*reinterpret_cast<const int32_t*>(cur));
    } else if (type == "uint") {
        out = static_cast<float>(*reinterpret_cast<const uint32_t*>(cur));
    } else if (type == "float") {
        float v;
        std::memcpy(&v, cur, 4);
        out = v;
    } else if (type == "double") {
        double v;
        std::memcpy(&v, cur, 8);
        out = static_cast<float>(v);
    } else {
        return false;
    }
    cur += sz;
    return true;
}

std::vector<std::string_view> splitWS(std::string_view s) {
    std::vector<std::string_view> out;
    size_t i = 0, n = s.size();
    while (i < n) {
        while (i < n && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        if (i >= n) break;
        size_t j = i;
        while (j < n && !std::isspace(static_cast<unsigned char>(s[j]))) ++j;
        out.push_back(s.substr(i, j - i));
        i = j;
    }
    return out;
}

VRole roleFromName(std::string_view name) {
    if (name == "x") return R_PX;
    if (name == "y") return R_PY;
    if (name == "z") return R_PZ;
    if (name == "nx") return R_NX;
    if (name == "ny") return R_NY;
    if (name == "nz") return R_NZ;
    return R_NONE;
}

void computeBounds(PackMesh& pm, const std::vector<float>& positions) {
    const size_t n = positions.size() / 3;
    if (n == 0) {
        pm.boundsMin[0] = pm.boundsMin[1] = pm.boundsMin[2] = 0.f;
        pm.boundsMax[0] = pm.boundsMax[1] = pm.boundsMax[2] = 0.f;
        return;
    }
    float mn[3] = {positions[0], positions[1], positions[2]};
    float mx[3] = {positions[0], positions[1], positions[2]};
    for (size_t i = 1; i < n; ++i) {
        for (int c = 0; c < 3; ++c) {
            const float v = positions[i * 3 + c];
            if (v < mn[c]) mn[c] = v;
            if (v > mx[c]) mx[c] = v;
        }
    }
    pm.boundsMin[0] = mn[0];
    pm.boundsMin[1] = mn[1];
    pm.boundsMin[2] = mn[2];
    pm.boundsMax[0] = mx[0];
    pm.boundsMax[1] = mx[1];
    pm.boundsMax[2] = mx[2];
}

} // namespace

class PlyParser : public ModelParser {
public:
    explicit PlyParser(unsigned /*threads*/) {}

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;
    PackResult& result() override { return *result_; }

private:
    bool execute(std::string_view path);
    bool parse(const std::shared_ptr<MappedFile>& mf);
};

bool PlyParser::execute(std::string_view path) {
    result_->objFile = MappedFile::openShared(path);
    if (!result_->objFile) {
        lastError_ = "cannot open file: " + std::string(path);
        fireAllDone(*result_, false, lastError_);
        return false;
    }
    const auto slash = path.find_last_of("/\\");
    if (slash != std::string_view::npos)
        result_->sourceDir = std::string(path.substr(0, slash));

    if (!parse(result_->objFile)) {
        fireAllDone(*result_, false, lastError_);
        return false;
    }

    fireProgress(100.f);
    fireVertices(*result_, result_->meshes);
    fireMaterials(*result_, {});
    fireTextures(*result_, {});
    fireAllDone(*result_, true, {});
    return true;
}

bool PlyParser::load(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    return execute(path);
}

void PlyParser::loadAsync(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    // NOTE: the caller must keep this parser object alive until onAllDone
    // fires, because the detached thread below operates on `this`.
    auto pathCopy = std::make_shared<std::string>(path);
    std::thread([this, pathCopy]() { execute(*pathCopy); }).detach();
}

bool PlyParser::parse(const std::shared_ptr<MappedFile>& mf) {
    const std::string_view text = mf->text();
    const std::span<const std::byte> bytes = mf->bytes();
    const std::byte* binBase = reinterpret_cast<const std::byte*>(text.data());

    bool isBinary = false;
    bool bigEndian = false;
    size_t vertexCount = 0;
    size_t faceCount = 0;
    std::vector<VProp> vprops;
    bool hasNormals = false;
    std::string faceCountType = "uchar";
    std::string faceIdxType = "int";
    bool faceHasList = true;

    bool inVertex = false;
    bool inFace = false;

    size_t pos = 0;
    std::string_view body;
    bool headerOk = false;

    while (pos < text.size()) {
        size_t nl = text.find('\n', pos);
        size_t lineEnd = (nl == std::string_view::npos) ? text.size() : nl;
        std::string_view line = text.substr(pos, lineEnd - pos);
        if (!line.empty() && line.back() == '\r')
            line = line.substr(0, line.size() - 1);
        const size_t nextPos = (nl == std::string_view::npos) ? text.size() : nl + 1;

        auto toks = splitWS(line);
        if (toks.empty()) { pos = nextPos; continue; }

        if (toks[0] == "ply") {
            // magic line; nothing to do
        } else if (toks[0] == "format") {
            if (toks.size() < 3) {
                lastError_ = "malformed format line";
                return false;
            }
            if (toks[1] == "ascii") {
                isBinary = false;
            } else if (toks[1] == "binary_little_endian") {
                isBinary = true;
                bigEndian = false;
            } else if (toks[1] == "binary_big_endian") {
                isBinary = true;
                bigEndian = true;
            } else {
                lastError_ = "unsupported ply format: " + std::string(toks[1]);
                return false;
            }
        } else if (toks[0] == "element") {
            inVertex = false;
            inFace = false;
            if (toks.size() < 3) {
                lastError_ = "malformed element line";
                return false;
            }
            size_t count = 0;
            try {
                count = std::stoul(std::string(toks[2]));
            } catch (...) {
                lastError_ = "malformed element count";
                return false;
            }
            if (toks[1] == "vertex") {
                inVertex = true;
                inFace = false;
                vertexCount = count;
                vprops.clear();
            } else if (toks[1] == "face") {
                inFace = true;
                inVertex = false;
                faceCount = count;
            }
        } else if (toks[0] == "property") {
            if (inVertex) {
                VProp p;
                if (toks.size() >= 5 && toks[1] == "list") {
                    p.isList = true;
                    p.listCountType = std::string(toks[2]);
                    p.listIdxType = std::string(toks[3]);
                    p.type = p.listIdxType;
                } else if (toks.size() >= 3) {
                    p.type = std::string(toks[1]);
                    p.role = roleFromName(toks[2]);
                    if (p.role == R_NX || p.role == R_NY || p.role == R_NZ)
                        hasNormals = true;
                } else {
                    lastError_ = "malformed vertex property line";
                    return false;
                }
                vprops.push_back(p);
            } else if (inFace) {
                if (toks.size() >= 5 && toks[1] == "list") {
                    faceHasList = true;
                    faceCountType = std::string(toks[2]);
                    faceIdxType = std::string(toks[3]);
                } else if (toks.size() >= 3) {
                    faceHasList = false;
                    faceIdxType = std::string(toks[1]);
                }
            }
        } else if (toks[0] == "end_header") {
            body = text.substr(nextPos);
            headerOk = true;
            pos = nextPos;
            break;
        }
        // comments / obj_info / other lines are ignored
        pos = nextPos;
    }

    if (!headerOk) {
        lastError_ = "missing end_header";
        return false;
    }
    if (bigEndian) {
        lastError_ = "binary_big_endian is not supported";
        return false;
    }
    if (vertexCount == 0) {
        lastError_ = "no vertices in ply";
        return false;
    }
    if (vprops.empty()) {
        lastError_ = "no vertex properties in ply";
        return false;
    }

    // ---- read vertices ----
    if (isBinary) {
        const std::byte* cur = binBase + pos;
        const std::byte* end = bytes.data() + bytes.size();

        for (size_t v = 0; v < vertexCount; ++v) {
            if (isCancelled()) { lastError_ = "cancelled"; return false; }
            float px = 0.f, py = 0.f, pz = 0.f;
            float nx = 0.f, ny = 0.f, nz = 0.f;
            for (const VProp& p : vprops) {
                if (p.isList) {
                    float cnt;
                    if (!readScalar(cur, end, p.listCountType, cnt))
                        { lastError_ = "short buffer reading vertex list"; return false; }
                    const size_t k = static_cast<size_t>(cnt);
                    for (size_t e = 0; e < k; ++e) {
                        float ignore;
                        if (!readScalar(cur, end, p.listIdxType, ignore))
                            { lastError_ = "short buffer reading vertex list entry"; return false; }
                    }
                } else {
                    float val;
                    if (!readScalar(cur, end, p.type, val))
                        { lastError_ = "short buffer reading vertex property"; return false; }
                    switch (p.role) {
                        case R_PX: px = val; break;
                        case R_PY: py = val; break;
                        case R_PZ: pz = val; break;
                        case R_NX: nx = val; break;
                        case R_NY: ny = val; break;
                        case R_NZ: nz = val; break;
                        default: break;
                    }
                }
            }
            result_->positions.push_back(px);
            result_->positions.push_back(py);
            result_->positions.push_back(pz);
            if (hasNormals) {
                result_->normals.push_back(nx);
                result_->normals.push_back(ny);
                result_->normals.push_back(nz);
            }
        }

        // ---- read faces (binary) ----
        for (size_t f = 0; f < faceCount; ++f) {
            if (isCancelled()) { lastError_ = "cancelled"; return false; }
            float kf;
            if (!readScalar(cur, end, faceCountType, kf))
                { lastError_ = "short buffer reading face count"; return false; }
            const size_t k = static_cast<size_t>(kf);
            if (k < 3) {
                for (size_t e = 0; e < k; ++e) {
                    float ignore;
                    if (!readScalar(cur, end, faceIdxType, ignore))
                        { lastError_ = "short buffer reading face index"; return false; }
                }
                continue;
            }
            std::vector<uint32_t> idx;
            idx.reserve(k);
            for (size_t e = 0; e < k; ++e) {
                float iv;
                if (!readScalar(cur, end, faceIdxType, iv))
                    { lastError_ = "short buffer reading face index"; return false; }
                idx.push_back(static_cast<uint32_t>(iv));
            }
            for (size_t j = 1; j + 1 < k; ++j) {
                result_->posIndices.push_back(idx[0]);
                result_->posIndices.push_back(idx[j]);
                result_->posIndices.push_back(idx[j + 1]);
            }
        }
    } else {
        // ---- ASCII ----
        size_t bpos = 0;
        auto nextLine = [&](std::string_view& out) -> bool {
            if (bpos >= body.size()) return false;
            size_t nl = body.find('\n', bpos);
            size_t e = (nl == std::string_view::npos) ? body.size() : nl;
            out = body.substr(bpos, e - bpos);
            if (!out.empty() && out.back() == '\r')
                out = out.substr(0, out.size() - 1);
            bpos = (nl == std::string_view::npos) ? body.size() : nl + 1;
            return true;
        };

        std::string_view line;
        for (size_t v = 0; v < vertexCount; ++v) {
            if (isCancelled()) { lastError_ = "cancelled"; return false; }
            if (!nextLine(line)) { lastError_ = "not enough vertex lines"; return false; }
            auto t = splitWS(line);
            if (t.size() < vprops.size())
                { lastError_ = "vertex line has too few tokens"; return false; }
            float px = 0.f, py = 0.f, pz = 0.f;
            float nx = 0.f, ny = 0.f, nz = 0.f;
            for (size_t pi = 0; pi < vprops.size(); ++pi) {
                const VProp& p = vprops[pi];
                if (p.isList) continue; // ASCII list vertex props are skipped
                const float val = static_cast<float>(std::atof(std::string(t[pi]).c_str()));
                switch (p.role) {
                    case R_PX: px = val; break;
                    case R_PY: py = val; break;
                    case R_PZ: pz = val; break;
                    case R_NX: nx = val; break;
                    case R_NY: ny = val; break;
                    case R_NZ: nz = val; break;
                    default: break;
                }
            }
            result_->positions.push_back(px);
            result_->positions.push_back(py);
            result_->positions.push_back(pz);
            if (hasNormals) {
                result_->normals.push_back(nx);
                result_->normals.push_back(ny);
                result_->normals.push_back(nz);
            }
        }

        for (size_t f = 0; f < faceCount; ++f) {
            if (isCancelled()) { lastError_ = "cancelled"; return false; }
            if (!nextLine(line)) { lastError_ = "not enough face lines"; return false; }
            auto t = splitWS(line);
            if (t.empty()) { lastError_ = "empty face line"; return false; }
            const size_t k = static_cast<size_t>(std::atoll(std::string(t[0]).c_str()));
            if (k < 3) continue;
            if (t.size() < k + 1)
                { lastError_ = "face line has too few indices"; return false; }
            std::vector<uint32_t> idx;
            idx.reserve(k);
            for (size_t e = 0; e < k; ++e)
                idx.push_back(static_cast<uint32_t>(std::atoll(std::string(t[e + 1]).c_str())));
            for (size_t j = 1; j + 1 < k; ++j) {
                result_->posIndices.push_back(idx[0]);
                result_->posIndices.push_back(idx[j]);
                result_->posIndices.push_back(idx[j + 1]);
            }
        }
    }

    // ---- assemble one mesh covering the whole pools ----
    PackMesh pm;
    pm.name = "ply";
    pm.materialIndex = -1;
    pm.positions = std::span<const float>(result_->positions);
    pm.normals = std::span<const float>(result_->normals);
    pm.texcoords = std::span<const float>(result_->texcoords);
    pm.indices = std::span<const uint32_t>(result_->posIndices);
    computeBounds(pm, result_->positions);
    result_->meshes.push_back(pm);

    return true;
}

void registerPlyParser() {
    static const bool once = [] {
        ParserRegistry::instance().add(
            "ply", [](unsigned threads) -> std::unique_ptr<ModelParser> {
                return std::make_unique<PlyParser>(threads);
            });
        return true;
    }();
    (void)once;
}

} // namespace ap
