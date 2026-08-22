// ============================================================
// PbrtParser - minimal PBRT-v4 scene-file parser for assetpack
//
// Self-contained: depends only on the assetpack public API
// (assetpack/AssetPack.h). Parses the text PBRT-v4 scene format
// (tokenizer + statement dispatch) and emits geometry/materials/
// textures through the ModelParser event pipeline.
//
// Supported:
//   - WorldBegin/End, Attribute/Transform Begin/End scoping
//   - Transform / ConcatTransform / Translate / Rotate / Scale / Identity
//   - Include file.pbrt (recursively resolved relative to current file)
//   - Shape trianglemesh / mesh (P, indices, N, uv)
//   - Shape plymesh (via the shared PLY parser: ascii + binary_little_endian)
//   - Material / MakeNamedMaterial / NamedMaterial (matte/plastic/metal + Kd/Ks)
//   - Texture ... imagemap string filename + texture Kd name refs
//   - LightSource / AreaLightSource (accepted, geometry priority)
//
// Transform matrix math uses GLM (vendored under third_party/glm,
// private include only -- the public header stays dependency-free).
// ============================================================

#include "assetpack/AssetPack.h"

#include <glm/geometric.hpp>             // normalize
#include <glm/gtc/matrix_inverse.hpp>    // inverseTranspose
#include <glm/gtc/matrix_transform.hpp>  // translate / scale / rotate
#include <glm/matrix.hpp>                // inverse / inverseTranspose
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>         // radians
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <taskflow/algorithm/for_each.hpp>   // parallel deferred .ply loading
#include <taskflow/taskflow.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ap {

namespace {

using Mat4 = glm::mat4;

constexpr float kInf = 1e30f;

float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

Mat4 matIdentity() { return glm::mat4(1.f); }
Mat4 matMul(const Mat4& a, const Mat4& b) { return a * b; }

Mat4 matTranslate(float x, float y, float z) {
    return glm::translate(glm::mat4(1.f), glm::vec3(x, y, z));
}

Mat4 matScale(float x, float y, float z) {
    return glm::scale(glm::mat4(1.f), glm::vec3(x, y, z));
}

// degree -> radians conversion happens here (glm takes radians)
Mat4 matRotate(float angDeg, float ax, float ay, float az) {
    return glm::rotate(glm::mat4(1.f), glm::radians(angDeg),
                       glm::vec3(ax, ay, az));
}

void matTransformPoint(const Mat4& m, float x, float y, float z,
                       float& ox, float& oy, float& oz) {
    const glm::vec4 p = m * glm::vec4(x, y, z, 1.f);
    ox = p.x;
    oy = p.y;
    oz = p.z;
    if (p.w != 0.f) { ox /= p.w; oy /= p.w; oz /= p.w; }
}

// normal matrix: inverse-transpose of the upper-left 3x3. The previous
// hand-written path dotted columns of the inverse -- the same M^-T --
// so this is behavior-preserving (and correct under non-uniform scale).
void matTransformNormal(const Mat4& m, float x, float y, float z,
                        float& ox, float& oy, float& oz) {
    const glm::vec3 n =
        glm::normalize(glm::mat3(glm::inverseTranspose(m)) *
                       glm::vec3(x, y, z));
    ox = n.x;
    oy = n.y;
    oz = n.z;
}

char decodeEscape(char c) {
    switch (c) {
        case 'n': return '\n';
        case 't': return '\t';
        case 'r': return '\r';
        case '\\': return '\\';
        case '"': return '"';
        case '\'': return '\'';
        case 'b': return '\b';
        case 'f': return '\f';
        default: return c;
    }
}

bool isQuoted(const std::string& s) {
    return s.size() >= 2 && s.front() == '"' && s.back() == '"';
}

std::string dequote(const std::string& s) {
    if (isQuoted(s)) return s.substr(1, s.size() - 2);
    return s;
}

std::string toLower(std::string_view s) {
    std::string r(s);
    for (char& c : r)
        if (c >= 'A' && c <= 'Z') c = char(c - 'A' + 'a');
    return r;
}

std::string combinePath(const std::string& dir, const std::string& name) {
    if (name.empty()) return name;
    if (name.size() >= 2 && name[1] == ':' &&
        ((name[0] >= 'A' && name[0] <= 'Z') || (name[0] >= 'a' && name[0] <= 'z')))
        return name;
    if (!dir.empty() && name[0] != '/' && name[0] != '\\')
        return dir + "/" + name;
    return name;
}

struct Param {
    std::string type;
    std::vector<float> floats;
    std::vector<int> ints;
    std::vector<std::string> strings;
};
using ParamMap = std::map<std::string, Param>;

struct MeshMeta {
    std::string_view name;
    int32_t mat = -1;
    size_t posStart = 0, posCount = 0;
    size_t nrmStart = 0, nrmCount = 0;
    size_t uvStart = 0, uvCount = 0;
    size_t idxStart = 0, idxCount = 0;
    float bmin[3] = {0, 0, 0};
    float bmax[3] = {0, 0, 0};
};

struct ParseState {
    Mat4 ctm = matIdentity();
    int mat = 0;
};

} // namespace

class PbrtParser final : public ModelParser {
public:
    explicit PbrtParser(unsigned threads = 0) : threads_(threads) {
        result_ = std::make_shared<PackResult>();
    }

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;

    PackResult& result() override { return *result_; }

    void setWantNormals(bool w) override { wantNormals_ = w; }
    void setWantTexcoords(bool w) override { wantTexcoords_ = w; }
    void setWantTextureBytes(bool w) override { wantTextureBytes_ = w; }

private:
    struct FileState {
        std::string_view text;
        size_t pos = 0;
        std::string dir;
    };
    std::vector<FileState> files_;
    std::optional<std::string> ungot_;
    std::vector<std::shared_ptr<MappedFile>> openFiles_;

    std::optional<std::string> nextTokenRaw();
    std::optional<std::string> nextToken();
    std::optional<std::string> peek();

    void parseStream();
    void dispatch(const std::string& stmt, const std::string& raw);
    ParamMap parseParams();
    std::string readName();
    float readFloat();
    Mat4 readMat16();
    void expectToken(const char* lit);

    void pushState();
    void popState();
    Mat4& ctm() { return states_.back().ctm; }

    void applyMaterial(const std::string& type, const ParamMap& params);
    int makeMaterial(const std::string& type, const ParamMap& params);
    void storeTexture(const std::string& name, const ParamMap& params);
    void storeNamedMaterial(const std::string& name, const ParamMap& params);
    void handleShape(const std::string& kind, const ParamMap& params);
    void handleInclude(const std::string& filename);
    void emitMesh(const std::string& kind, const std::vector<float>& P,
                  const std::vector<float>* N, const std::vector<float>* uv,
                  const std::vector<int>& indices);
    // deferred plymesh pipeline: shapes record file/CTM/material during
    // parsing; referenced .ply files then load in parallel and merge in
    // submission order once the token stream ends
    struct DeferredPly {
        std::string full;
        Mat4 ctm;
        int mat = 0;
    };
    struct MergedPly {                 // per-task local geometry
        int mat = 0;
        std::vector<float> pos, nrm, uv;
        std::vector<uint32_t> idx;
        std::vector<MeshMeta> meta;    // offsets relative to this buffer
    };
    void loadOnePly(const DeferredPly& t, MergedPly& out);
    void appendPacked(const PackMesh& m, const Mat4& M, MergedPly& out) const;
    void concatPly(const MergedPly& src);

    std::vector<DeferredPly> deferredPly_;
    unsigned threads_ = 0;

    void bindMaterialTexture(PackMaterial& mat, const std::string& texname, int type);
    void ensureTexture(std::string_view path, int type);

    std::string_view intern(const std::string& s) {
        interned_.push_back(s);
        return std::string_view(interned_.back());
    }

    bool wantNormals_ = true;
    bool wantTexcoords_ = true;
    bool wantTextureBytes_ = false;

    std::deque<std::string> interned_;
    std::vector<ParseState> states_;
    std::unordered_map<std::string, int> namedMaterials_;
    std::unordered_map<std::string, std::string_view> texNameToPath_;
    std::unordered_set<std::string> addedTextures_;

    std::vector<float> posPool_;
    std::vector<float> nrmPool_;
    std::vector<float> uvPool_;
    std::vector<uint32_t> idxPool_;
    std::vector<MeshMeta> meshMeta_;
    size_t meshCounter_ = 0;
    // verts already covered by nrm/uv pools; once a pool starts it must
    // stay aligned to the global vertex count (zeros where a shape lacks
    // the attribute) because consumers upload whole pools sized by the
    // position count
    size_t nrmEmittedVerts_ = 0;
    size_t uvEmittedVerts_ = 0;

    std::string sourceDir_;
};

std::optional<std::string> PbrtParser::nextTokenRaw() {
    while (!files_.empty()) {
        auto& f = files_.back();
        const char* base = f.text.data();
        const char* p = base + f.pos;
        const char* end = base + f.text.size();
        if (p >= end) { files_.pop_back(); continue; }
        while (p < end && (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r')) p++;
        if (p >= end) { f.pos = size_t(p - base); files_.pop_back(); continue; }
        if (*p == '#') { while (p < end && *p != '\n') p++; f.pos = size_t(p - base); continue; }
        const char* start = p;
        if (*p == '"') {
            std::string out;
            out.push_back('"');
            p++;
            while (p < end) {
                if (*p == '\\') {
                    if (p + 1 < end) { out.push_back(decodeEscape(*(p + 1))); p += 2; }
                    else { out.push_back('"'); p++; break; }
                } else if (*p == '"') { out.push_back('"'); p++; break; }
                else { out.push_back(*p); p++; }
            }
            f.pos = size_t(p - base);
            return out;
        }
        if (*p == '[' || *p == ']') {
            std::string out(1, *p);
            p++;
            f.pos = size_t(p - base);
            return out;
        }
        while (p < end && *p != ' ' && *p != '\n' && *p != '\t' && *p != '\r' &&
               *p != '"' && *p != '[' && *p != ']')
            p++;
        std::string out(start, p);
        f.pos = size_t(p - base);
        return out;
    }
    return std::nullopt;
}

std::optional<std::string> PbrtParser::nextToken() {
    if (ungot_) { auto t = ungot_; ungot_.reset(); return t; }
    return nextTokenRaw();
}

std::optional<std::string> PbrtParser::peek() {
    if (!ungot_) ungot_ = nextTokenRaw();
    return ungot_;
}

ParamMap PbrtParser::parseParams() {
    ParamMap out;
    while (true) {
        auto t = peek();
        if (!t) break;
        if (!isQuoted(*t)) break;
        nextToken();
        std::string decl = dequote(*t);
        size_t sp = decl.find(' ');
        std::string type = (sp == std::string::npos) ? decl : decl.substr(0, sp);
        std::string name = (sp == std::string::npos) ? std::string() : decl.substr(sp + 1);
        Param p;
        p.type = type;
        auto val = nextToken();
        if (!val) break;
        if (*val == "[") {
            while (true) {
                auto v = nextToken();
                if (!v || *v == "]") break;
                if (p.type == "integer")
                    p.ints.push_back((int)std::strtol(v->c_str(), nullptr, 10));
                else if (p.type == "string" || p.type == "texture")
                    p.strings.push_back(dequote(*v));
                else
                    p.floats.push_back((float)std::strtod(v->c_str(), nullptr));
            }
        } else {
            if (p.type == "integer")
                p.ints.push_back((int)std::strtol(val->c_str(), nullptr, 10));
            else if (p.type == "string" || p.type == "texture")
                p.strings.push_back(dequote(*val));
            else
                p.floats.push_back((float)std::strtod(val->c_str(), nullptr));
        }
        out[name] = std::move(p);
    }
    return out;
}

std::string PbrtParser::readName() {
    auto t = nextToken();
    if (!t) return std::string();
    return dequote(*t);
}

float PbrtParser::readFloat() {
    auto t = nextToken();
    if (!t) return 0.f;
    return (float)std::strtod(t->c_str(), nullptr);
}

Mat4 PbrtParser::readMat16() {
    Mat4 m{};
    // the file stores column-major floats: flat idx i maps to [i/4][i%4]
    for (int i = 0; i < 16; ++i) m[i / 4][i % 4] = readFloat();
    return m;
}

void PbrtParser::expectToken(const char* lit) {
    auto t = nextToken();
    if (!t || *t != lit)
        addWarning(std::string("pbrt: expected '") + lit + "' in matrix statement");
}

void PbrtParser::pushState() { states_.push_back(states_.back()); }
void PbrtParser::popState() { if (states_.size() > 1) states_.pop_back(); }

void PbrtParser::parseStream() {
    while (auto tok = nextToken()) {
        if (isCancelled()) {
            lastError_ = "cancelled";
            fireAllDone(*result_, false, lastError_);
            return;
        }
        const std::string& s = *tok;
        if (s.empty() || s[0] == '#') continue;
        if (s[0] == '"' || s[0] == '[' || s[0] == ']') {
            if (s[0] == '"') parseParams();
            continue;
        }
        dispatch(toLower(s), s);
    }
}

void PbrtParser::dispatch(const std::string& low, const std::string& /*raw*/) {
    if (low == "worldbegin" || low == "worldend") {
    } else if (low == "attributebegin" || low == "transformbegin") {
        pushState();
    } else if (low == "attributeend" || low == "transformend") {
        popState();
    } else if (low == "identity") {
        ctm() = matIdentity();
    } else if (low == "translate") {
        float x = readFloat(), y = readFloat(), z = readFloat();
        ctm() = matMul(ctm(), matTranslate(x, y, z));
    } else if (low == "scale") {
        float x = readFloat(), y = readFloat(), z = readFloat();
        ctm() = matMul(ctm(), matScale(x, y, z));
    } else if (low == "rotate") {
        float a = readFloat(), x = readFloat(), y = readFloat(), z = readFloat();
        ctm() = matMul(ctm(), matRotate(a, x, y, z));
    } else if (low == "transform") {
        expectToken("[");
        Mat4 m = readMat16();
        expectToken("]");
        ctm() = m;
    } else if (low == "concattransform") {
        expectToken("[");
        Mat4 m = readMat16();
        expectToken("]");
        ctm() = matMul(ctm(), m);
    } else if (low == "material") {
        std::string type = readName();
        ParamMap params = parseParams();
        applyMaterial(type, params);
    } else if (low == "makenamedmaterial") {
        std::string name = readName();
        ParamMap params = parseParams();
        storeNamedMaterial(name, params);
    } else if (low == "namedmaterial") {
        std::string name = readName();
        auto it = namedMaterials_.find(name);
        if (it != namedMaterials_.end())
            states_.back().mat = it->second;
        else
            addWarning("pbrt: NamedMaterial '" + name + "' not found");
    } else if (low == "texture") {
        std::string name = readName();
        readName();
        readName();
        ParamMap params = parseParams();
        storeTexture(name, params);
    } else if (low == "shape") {
        std::string kind = readName();
        ParamMap params = parseParams();
        handleShape(kind, params);
    } else if (low == "lightsource" || low == "arealightsource") {
        readName();
        parseParams();
    } else if (low == "objectbegin") {
        readName();
    } else if (low == "objectend") {
    } else if (low == "objectinstance") {
        readName();
    } else if (low == "reverseorientation") {
    } else if (low == "activetransform") {
        readName();
    } else if (low == "include") {
        std::string fn = readName();
        handleInclude(fn);
    } else {
        if (peek() && isQuoted(*peek())) {
            nextToken();
            parseParams();
        }
    }
}

void PbrtParser::handleInclude(const std::string& filename) {
    std::string dir = files_.empty() ? sourceDir_ : files_.back().dir;
    std::string full = combinePath(dir, filename);
    auto mf = MappedFile::openShared(full);
    if (!mf) {
        addWarning("pbrt: Include '" + filename + "' not found");
        return;
    }
    openFiles_.push_back(mf);
    std::string fdir = full;
    auto sl = fdir.find_last_of("/\\");
    if (sl != std::string::npos) fdir = fdir.substr(0, sl);
    files_.push_back(FileState{mf->text(), 0, fdir});
}

int PbrtParser::makeMaterial(const std::string& type, const ParamMap& params) {
    PackMaterial mat;
    mat.name = intern(type.empty() ? std::string("material") : type);
    mat.diffuse[0] = mat.diffuse[1] = mat.diffuse[2] = mat.diffuse[3] = 1.f;

    // legacy PBRT-v3 names (Kd/Ks) plus the pbrt-v4 renames: matte-style
    // diffuse is "reflectance" (either a texture reference or rgb/color),
    // and normals arrive as a plain "string normalmap" path
    ParamMap::const_iterator it;
    for (const char* key : {"Kd", "reflectance"}) {
        it = params.find(key);
        if (it == params.end()) continue;
        if (it->second.type == "texture") {
            std::string texname = it->second.strings.empty()
                                      ? std::string()
                                      : it->second.strings[0];
            bindMaterialTexture(mat, texname, TexDiffuse);
        } else if (!it->second.floats.empty()) {
            const auto& f = it->second.floats;
            mat.diffuse[0] = clamp01(f[0]);
            mat.diffuse[1] = clamp01(f.size() > 1 ? f[1] : f[0]);
            mat.diffuse[2] = clamp01(f.size() > 2 ? f[2] : f[0]);
            mat.diffuse[3] = 1.f;
        }
        break;   // first hit wins; never mix Kd and reflectance
    }
    it = params.find("normalmap");
    if (it != params.end() && !it->second.strings.empty()) {
        const std::string_view path = intern(it->second.strings[0]);
        mat.textures.push_back(PackTexRef{TexNormal, 0, path});
        ensureTexture(path, TexNormal);
    }
    it = params.find("Ks");
    if (it != params.end() && it->second.type != "texture" && !it->second.floats.empty()) {
        const auto& f = it->second.floats;
        mat.specular[0] = clamp01(f[0]);
        mat.specular[1] = clamp01(f.size() > 1 ? f[1] : f[0]);
        mat.specular[2] = clamp01(f.size() > 2 ? f[2] : f[0]);
        mat.specular[3] = 1.f;
    }
    it = params.find("roughness");
    if (it != params.end() && !it->second.floats.empty()) {
        float r = clamp01(it->second.floats[0]);
        mat.shininess = 100.f * (1.f - r) + 1.f;
    }
    result_->materials.push_back(std::move(mat));
    return (int)result_->materials.size() - 1;
}

void PbrtParser::applyMaterial(const std::string& type, const ParamMap& params) {
    int idx = makeMaterial(type, params);
    states_.back().mat = idx;
}

void PbrtParser::storeNamedMaterial(const std::string& name, const ParamMap& params) {
    int idx = makeMaterial(params.empty() ? std::string("named") : name, params);
    namedMaterials_[name] = idx;
}

void PbrtParser::storeTexture(const std::string& name, const ParamMap& params) {
    std::string fn;
    auto it = params.find("filename");
    if (it != params.end() && !it->second.strings.empty()) fn = it->second.strings[0];
    texNameToPath_[name] = intern(fn);
}

void PbrtParser::bindMaterialTexture(PackMaterial& mat, const std::string& texname,
                                     int type) {
    auto it = texNameToPath_.find(texname);
    if (it == texNameToPath_.end()) {
        addWarning("pbrt: texture '" + texname + "' referenced by material not declared");
        return;
    }
    mat.textures.push_back(PackTexRef{type, 0, it->second});
    ensureTexture(it->second, type);
}

void PbrtParser::ensureTexture(std::string_view path, int type) {
    std::string key(path);
    if (addedTextures_.count(key)) return;
    addedTextures_.insert(key);
    PackTexture tex;
    tex.path = path;
    tex.type = type;
    tex.slot = 0;
    tex.embedded = false;
    tex.resolvedPath = combinePath(sourceDir_, std::string(path));
    tex.byteSize = 0;
    result_->textures.push_back(std::move(tex));
}

void PbrtParser::emitMesh(const std::string& kind, const std::vector<float>& P,
                          const std::vector<float>* N, const std::vector<float>* uv,
                          const std::vector<int>& indices) {
    size_t vcount = P.size() / 3;
    if (vcount == 0) return;

    size_t base = posPool_.size() / 3;
    float bmin[3] = {kInf, kInf, kInf};
    float bmax[3] = {-kInf, -kInf, -kInf};
    for (size_t i = 0; i < vcount; ++i) {
        float ox, oy, oz;
        matTransformPoint(ctm(), P[i * 3], P[i * 3 + 1], P[i * 3 + 2], ox, oy, oz);
        posPool_.push_back(ox);
        posPool_.push_back(oy);
        posPool_.push_back(oz);
        for (int k = 0; k < 3; ++k) {
            float v = (k == 0) ? ox : (k == 1) ? oy : oz;
            if (v < bmin[k]) bmin[k] = v;
            if (v > bmax[k]) bmax[k] = v;
        }
    }

    size_t nbase = nrmPool_.size() / 3, ncount = 0;
    const bool shapeNrm = wantNormals_ && N && !N->empty();
    if (shapeNrm || !nrmPool_.empty()) {
        // backfill zeros for shapes emitted before the pool started,
        // pad zeros for this shape when it lacks normals
        if (nrmEmittedVerts_ < base) nrmPool_.resize(base * 3, 0.f);
        if (shapeNrm) {
            const size_t avail = N->size() / 3;
            for (size_t i = 0; i < vcount; ++i) {
                if (i < avail) {
                    float ox, oy, oz;
                    matTransformNormal(ctm(), (*N)[i * 3], (*N)[i * 3 + 1], (*N)[i * 3 + 2],
                                       ox, oy, oz);
                    nrmPool_.push_back(ox);
                    nrmPool_.push_back(oy);
                    nrmPool_.push_back(oz);
                } else {
                    nrmPool_.push_back(0.f); nrmPool_.push_back(0.f); nrmPool_.push_back(0.f);
                }
            }
        } else {
            nrmPool_.resize((base + vcount) * 3, 0.f);
        }
        nrmEmittedVerts_ = base + vcount;
        ncount = vcount;
    }

    size_t ubase = uvPool_.size() / 2, ucount = 0;
    const bool shapeUv = wantTexcoords_ && uv && !uv->empty();
    if (shapeUv || !uvPool_.empty()) {
        if (uvEmittedVerts_ < base) uvPool_.resize(base * 2, 0.f);
        if (shapeUv) {
            const size_t avail = uv->size() / 2;
            for (size_t i = 0; i < vcount; ++i) {
                if (i < avail) {
                    uvPool_.push_back((*uv)[i * 2]);
                    uvPool_.push_back((*uv)[i * 2 + 1]);
                } else {
                    uvPool_.push_back(0.f); uvPool_.push_back(0.f);
                }
            }
        } else {
            uvPool_.resize((base + vcount) * 2, 0.f);
        }
        uvEmittedVerts_ = base + vcount;
        ucount = vcount;
    }

    size_t idxBase = idxPool_.size();
    if (!indices.empty()) {
        for (int v : indices) idxPool_.push_back((uint32_t)(base + (size_t)v));
    } else {
        for (size_t i = 0; i < vcount; ++i) idxPool_.push_back((uint32_t)(base + i));
    }

    MeshMeta m;
    m.name = intern(kind + " " + std::to_string(meshCounter_++));
    m.mat = states_.back().mat;
    m.posStart = base;
    m.posCount = vcount;
    m.nrmStart = nbase;
    m.nrmCount = ncount;
    m.uvStart = ubase;
    m.uvCount = ucount;
    m.idxStart = idxBase;
    m.idxCount = idxPool_.size() - idxBase;
    for (int k = 0; k < 3; ++k) { m.bmin[k] = bmin[k]; m.bmax[k] = bmax[k]; }
    meshMeta_.push_back(m);
}

void PbrtParser::handleShape(const std::string& kind, const ParamMap& params) {
    if (kind == "trianglemesh" || kind == "mesh") {
        auto pit = params.find("P");
        if (pit == params.end() || pit->second.floats.empty()) {
            addWarning("pbrt: " + kind + " without P ignored");
            return;
        }
        std::vector<float> P = pit->second.floats;
        const std::vector<float>* N = nullptr;
        const std::vector<float>* uv = nullptr;
        std::vector<float> Nv, uvv;
        auto nit = params.find("N");
        if (nit != params.end() && !nit->second.floats.empty()) {
            Nv = nit->second.floats;
            N = &Nv;
        }
        auto uit = params.find("uv");
        if (uit != params.end() && !uit->second.floats.empty()) {
            uvv = uit->second.floats;
            uv = &uvv;
        }
        std::vector<int> indices;
        auto iit = params.find("indices");
        if (iit != params.end()) {
            indices = iit->second.ints;
            if (!indices.empty() && indices.size() % 3 != 0)
                addWarning("pbrt: " + kind + " indices not a multiple of 3");
        }
        emitMesh(kind, P, N, uv, indices);
    } else if (kind == "plymesh") {
        auto fit = params.find("filename");
        if (fit == params.end() || fit->second.strings.empty()) {
            addWarning("pbrt: plymesh without string filename ignored");
            return;
        }
        std::string dir = files_.empty() ? sourceDir_ : files_.back().dir;
        DeferredPly t;
        t.full = combinePath(dir, fit->second.strings[0]);
        t.ctm = ctm();
        t.mat = states_.back().mat;
        deferredPly_.push_back(std::move(t));
    }
}

bool PbrtParser::load(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    files_.clear();
    ungot_.reset();
    openFiles_.clear();
    interned_.clear();
    states_.clear();
    namedMaterials_.clear();
    texNameToPath_.clear();
    addedTextures_.clear();
    posPool_.clear();
    nrmPool_.clear();
    uvPool_.clear();
    idxPool_.clear();
    meshMeta_.clear();
    meshCounter_ = 0;
    nrmEmittedVerts_ = 0;
    uvEmittedVerts_ = 0;
    deferredPly_.clear();

    auto mainFile = MappedFile::openShared(path);
    if (!mainFile) {
        lastError_ = std::string("pbrt: cannot open file: ") + std::string(path);
        addWarning(lastError_);
        fireAllDone(*result_, false, lastError_);
        return false;
    }
    openFiles_.push_back(mainFile);
    result_->objFile = mainFile;

    std::string full(path);
    auto sl = full.find_last_of("/\\");
    sourceDir_ = (sl != std::string::npos) ? full.substr(0, sl) : std::string();
    result_->sourceDir = sourceDir_;

    {
        PackMaterial def;
        def.name = intern("default");
        def.diffuse[0] = def.diffuse[1] = def.diffuse[2] = def.diffuse[3] = 1.f;
        result_->materials.push_back(std::move(def));
    }
    states_.push_back(ParseState{matIdentity(), 0});

    files_.push_back(FileState{mainFile->text(), 0, sourceDir_});
    parseStream();

    // deferred plymesh: parallel load into per-task buffers, then merge
    // strictly in submission order so output stays deterministic
    if (!deferredPly_.empty()) {
        unsigned workers = threads_ ? threads_
                                    : std::thread::hardware_concurrency();
        if (!workers) workers = 4;
        std::vector<MergedPly> merged(deferredPly_.size());
        if (workers > 1 && deferredPly_.size() > 1) {
            tf::Executor exec(workers > 16 ? 16 : workers);
            tf::Taskflow flow;
            flow.for_each_index(0, int(deferredPly_.size()), 1,
                                [&](int i) {
                if (isCancelled()) return;
                loadOnePly(deferredPly_[size_t(i)], merged[size_t(i)]);
            });
            exec.run(flow).wait();
        } else {
            for (size_t i = 0; i < deferredPly_.size(); ++i) {
                if (isCancelled()) break;
                loadOnePly(deferredPly_[i], merged[i]);
            }
        }
        for (const MergedPly& m : merged) concatPly(m);
    }

    if (isCancelled()) {
        lastError_ = "cancelled";
        fireAllDone(*result_, false, lastError_);
        return false;
    }

    result_->positions = std::move(posPool_);
    result_->normals = std::move(nrmPool_);
    result_->texcoords = std::move(uvPool_);
    result_->posIndices = std::move(idxPool_);

    for (const auto& m : meshMeta_) {
        PackMesh pm;
        pm.name = m.name;
        pm.materialIndex = m.mat;
        pm.positions = std::span<const float>(result_->positions)
                           .subspan(m.posStart * 3, m.posCount * 3);
        if (m.nrmCount)
            pm.normals = std::span<const float>(result_->normals)
                             .subspan(m.nrmStart * 3, m.nrmCount * 3);
        if (m.uvCount)
            pm.texcoords = std::span<const float>(result_->texcoords)
                                .subspan(m.uvStart * 2, m.uvCount * 2);
        pm.indices = std::span<const uint32_t>(result_->posIndices)
                         .subspan(m.idxStart, m.idxCount);
        for (int k = 0; k < 3; ++k) { pm.boundsMin[k] = m.bmin[k]; pm.boundsMax[k] = m.bmax[k]; }
        result_->meshes.push_back(std::move(pm));
    }

    fireProgress(100.f);
    fireVertices(*result_, result_->meshes);
    fireMaterials(*result_, result_->materials);
    fireTextures(*result_, result_->textures);
    fireAllDone(*result_, true, {});
    return true;
}

void PbrtParser::loadAsync(std::string_view path) {
    // NOTE: caller must keep this parser alive until onAllDone fires.
    auto p = std::make_shared<std::string>(path);
    std::thread([this, p]() { load(*p); }).detach();
}

// ---- deferred plymesh pipeline -------------------------------------------
// Shapes were recorded during parsing (handleShape). Their .ply files load
// through the shared PlyParser (ascii + binary_little_endian) into
// per-task buffers, then concatenate in scene order deterministically.

// copy one ply sub-mesh under the shape's CTM into a task-local buffer.
// Attribute pools stay vertex-aligned once started (zeros where absent),
// matching the emitMesh contract consumers rely on. Thread-safe: reads
// only immutable state, writes only `out`.
void PbrtParser::appendPacked(const PackMesh& m, const Mat4& M,
                              MergedPly& out) const {
    if (m.positions.empty()) return;
    const size_t base = out.pos.size() / 3;
    const size_t nv = m.positions.size() / 3;
    out.pos.reserve(out.pos.size() + m.positions.size());
    if (!m.normals.empty())
        out.nrm.reserve(out.nrm.size() + m.normals.size());
    if (!m.texcoords.empty())
        out.uv.reserve(out.uv.size() + m.texcoords.size());
    out.idx.reserve(out.idx.size() + m.indices.size());
    float bmin[3] = {kInf, kInf, kInf};
    float bmax[3] = {-kInf, -kInf, -kInf};
    for (size_t i = 0; i < nv; ++i) {
        float ox, oy, oz;
        matTransformPoint(M, m.positions[i * 3], m.positions[i * 3 + 1],
                          m.positions[i * 3 + 2], ox, oy, oz);
        out.pos.push_back(ox);
        out.pos.push_back(oy);
        out.pos.push_back(oz);
        for (int k = 0; k < 3; ++k) {
            const float v = (k == 0) ? ox : (k == 1) ? oy : oz;
            if (v < bmin[k]) bmin[k] = v;
            if (v > bmax[k]) bmax[k] = v;
        }
    }

    const bool meshNrm = wantNormals_ && !m.normals.empty();
    if (meshNrm || !out.nrm.empty()) {
        if (out.nrm.size() / 3 < base) out.nrm.resize(base * 3, 0.f);
        if (meshNrm) {
            for (size_t i = 0; i < nv && i * 3 + 2 < m.normals.size(); ++i) {
                float ox, oy, oz;
                matTransformNormal(M, m.normals[i * 3], m.normals[i * 3 + 1],
                                   m.normals[i * 3 + 2], ox, oy, oz);
                out.nrm.push_back(ox);
                out.nrm.push_back(oy);
                out.nrm.push_back(oz);
            }
        }
        if (out.nrm.size() / 3 < base + nv)
            out.nrm.resize((base + nv) * 3, 0.f);
    }

    const bool meshUv = wantTexcoords_ && !m.texcoords.empty();
    if (meshUv || !out.uv.empty()) {
        if (out.uv.size() / 2 < base) out.uv.resize(base * 2, 0.f);
        out.uv.insert(out.uv.end(), m.texcoords.begin(), m.texcoords.end());
        if (out.uv.size() / 2 < base + nv)
            out.uv.resize((base + nv) * 2, 0.f);
    }

    const size_t idxBase = out.idx.size();
    for (uint32_t ix : m.indices) out.idx.push_back(uint32_t(base + ix));

    MeshMeta mm;
    mm.mat = out.mat;
    mm.posStart = base;
    mm.posCount = nv;
    // pools are vertex-aligned when active, so per-vertex attributes map
    // positionally onto this mesh's slice
    mm.nrmStart = base;
    mm.nrmCount = (out.nrm.size() / 3 == base + nv) ? nv : 0;
    mm.uvStart = base;
    mm.uvCount = (out.uv.size() / 2 == base + nv) ? nv : 0;
    mm.idxStart = idxBase;
    mm.idxCount = out.idx.size() - idxBase;
    for (int k = 0; k < 3; ++k) { mm.bmin[k] = bmin[k]; mm.bmax[k] = bmax[k]; }
    out.meta.push_back(mm);
}

void PbrtParser::loadOnePly(const DeferredPly& t, MergedPly& out) {
    out.mat = t.mat;
    AssetPack ply;
    if (!ply.load(t.full)) {
        addWarning("pbrt: plymesh failed: " + t.full);
        return;
    }
    const PackResult& pr = ply.result();
    for (const PackMesh& m : pr.meshes) appendPacked(m, t.ctm, out);
}

// merge one task's buffer into the global pools, shifting indices and
// continuing the vertex-aligned attribute coverage across mixed
// trianglemesh/plymesh emission
void PbrtParser::concatPly(const MergedPly& src) {
    if (src.meta.empty()) return;
    const size_t gv = posPool_.size() / 3;

    if (!src.nrm.empty() || !nrmPool_.empty()) {
        if (nrmEmittedVerts_ < gv) nrmPool_.resize(gv * 3, 0.f);
        nrmPool_.insert(nrmPool_.end(), src.nrm.begin(), src.nrm.end());
        if (nrmPool_.size() / 3 < gv + src.pos.size() / 3)
            nrmPool_.resize((gv + src.pos.size() / 3) * 3, 0.f);
        nrmEmittedVerts_ = nrmPool_.size() / 3;
    }
    if (!src.uv.empty() || !uvPool_.empty()) {
        if (uvEmittedVerts_ < gv) uvPool_.resize(gv * 2, 0.f);
        uvPool_.insert(uvPool_.end(), src.uv.begin(), src.uv.end());
        if (uvPool_.size() / 2 < gv + src.pos.size() / 3)
            uvPool_.resize((gv + src.pos.size() / 3) * 2, 0.f);
        uvEmittedVerts_ = uvPool_.size() / 2;
    }

    // local buffers start aligned at the global base, so per-mesh
    // attribute starts carry over unchanged; only positions/indices shift
    const size_t ib = idxPool_.size();
    posPool_.insert(posPool_.end(), src.pos.begin(), src.pos.end());
    idxPool_.reserve(idxPool_.size() + src.idx.size());
    for (uint32_t ix : src.idx) idxPool_.push_back(ix + uint32_t(gv));

    for (MeshMeta mm : src.meta) {
        mm.posStart += gv;
        mm.idxStart += ib;
        mm.name = intern("plymesh " + std::to_string(meshCounter_++));
        meshMeta_.push_back(mm);
    }
}

void registerPbrtParser() {
    static const bool once = [] {
        ParserRegistry::instance().add(
            "pbrt", [](unsigned threads) -> std::unique_ptr<ModelParser> {
                return std::make_unique<PbrtParser>(threads);
            });
        return true;
    }();
    (void)once;
}

} // namespace ap
