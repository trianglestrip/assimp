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
#include "../../core/TaskExecutor.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
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
inline bool isQuoted(std::string_view s) {
    return s.size() >= 2 && s.front() == '"' && s.back() == '"';
}

std::string dequote(const std::string& s) {
    if (isQuoted(s)) return s.substr(1, s.size() - 2);
    return s;
}
inline std::string_view dequoteView(std::string_view s) {
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
    float bmin[3] = {0, 0, 0}; // world space
    float bmax[3] = {0, 0, 0};
    float world[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    bool hasWorld = false;
    float normalWorld[9] = {1,0,0, 0,1,0, 0,0,1};
};

struct ParseState {
    Mat4 ctm = matIdentity();
    int mat = 0;
    bool area = false;          // AreaLightSource active for next shapes
    float areaCol[3] = {1, 1, 1};
    float areaInt = 1.f;
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
    std::optional<std::string_view> ungotView_;
    std::string dequotedViewBuf_;
    std::vector<std::shared_ptr<MappedFile>> openFiles_;

    std::optional<std::string> nextTokenRaw();
    std::optional<std::string> nextToken();
    std::optional<std::string> peek();
    std::optional<std::string_view> nextTokenView();
    std::optional<std::string_view> peekView();
    std::optional<std::string_view> nextTokenViewRaw();

    void parseStream();
    void dispatch(std::string_view low, std::string_view raw);
    ParamMap parseParams();
    std::string readName();
    float readFloat();
    Mat4 readMat16();
    void expectToken(std::string_view lit);

    void pushState();
    void popState();
    Mat4& ctm() { return states_.back().ctm; }

    void applyMaterial(const std::string& type, const ParamMap& params);
    int makeMaterial(const std::string& type, const ParamMap& params);
    void storeTexture(const std::string& name, const ParamMap& params);
    void storeNamedMaterial(const std::string& name, const ParamMap& params);
    void handleShape(const std::string& kind, const ParamMap& params);
    void handleInclude(std::string_view filename);
    void handleCamera(const std::string& type, const ParamMap& params);
    void handleLight(const std::string& type, const ParamMap& params,
                     bool area);
    static void copyColor_(const ParamMap& params, float out[3], float& inten);
    void attachAreaLight_(int taskIdx);
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

    // camera/film state
    bool hasLookAt_ = false;
    float lookPos_[3] = {0, 0, 0};
    float lookTarget_[3] = {0, 0, -1};
    float lookUp_[3] = {0, 1, 0};
    int filmW_ = 0, filmH_ = 0;
    std::string areaTypeName_;   // type of the active AreaLightSource

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

std::optional<std::string_view> PbrtParser::nextTokenViewRaw() {
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
            const char* q = p + 1;
            bool hasEscape = false;
            while (q < end) {
                if (*q == '\\') { hasEscape = true; if (q + 1 < end) q += 2; else { q++; break; } }
                else if (*q == '"') { q++; break; }
                else q++;
            }
            size_t len = size_t(q - start);
            bool closed = (q > start && *(q - 1) == '"');
            f.pos = size_t(q - base);
            if (!hasEscape) {
                return std::string_view(start, len);
            }
            dequotedViewBuf_.clear();
            dequotedViewBuf_.reserve(len);
            dequotedViewBuf_.push_back('"');
            const char* innerEnd = closed ? q - 1 : q;
            for (const char* r = start + 1; r < innerEnd; ) {
                if (*r == '\\' && r + 1 < innerEnd) {
                    dequotedViewBuf_.push_back(decodeEscape(*(r + 1)));
                    r += 2;
                } else {
                    dequotedViewBuf_.push_back(*r);
                    r++;
                }
            }
            if (closed) dequotedViewBuf_.push_back('"');
            return std::string_view(dequotedViewBuf_.data(), dequotedViewBuf_.size());
        }
        if (*p == '[' || *p == ']') {
            f.pos = size_t(p + 1 - base);
            return std::string_view(p, 1);
        }
        while (p < end && *p != ' ' && *p != '\n' && *p != '\t' && *p != '\r' &&
               *p != '"' && *p != '[' && *p != ']')
            p++;
        f.pos = size_t(p - base);
        return std::string_view(start, size_t(p - start));
    }
    return std::nullopt;
}

std::optional<std::string_view> PbrtParser::nextTokenView() {
    if (ungotView_) { auto t = *ungotView_; ungotView_.reset(); return t; }
    return nextTokenViewRaw();
}

std::optional<std::string_view> PbrtParser::peekView() {
    if (!ungotView_) ungotView_ = nextTokenViewRaw();
    return ungotView_;
}

// Compatibility wrappers: keep old string-based API by delegating to view
std::optional<std::string> PbrtParser::nextTokenRaw() {
    auto v = nextTokenView();
    if (!v) return std::nullopt;
    return std::string(*v);
}

std::optional<std::string> PbrtParser::nextToken() {
    if (ungot_) { auto t = std::move(*ungot_); ungot_.reset(); return t; }
    // prefer view cache if present (covers view path using old API)
    if (ungotView_) { auto v = *ungotView_; ungotView_.reset(); return std::string(v); }
    auto v = nextTokenViewRaw();
    if (!v) return std::nullopt;
    return std::string(*v);
}

std::optional<std::string> PbrtParser::peek() {
    if (ungot_) return ungot_;
    if (ungotView_) return std::string(*ungotView_);
    ungotView_ = nextTokenViewRaw();
    if (!ungotView_) return std::nullopt;
    return std::string(*ungotView_);
}

ParamMap PbrtParser::parseParams() {
    ParamMap out;
    while (true) {
        auto t = peekView();
        if (!t) break;
        if (!isQuoted(*t)) break;
        nextTokenView();
        std::string_view decl = dequoteView(*t);
        size_t sp = decl.find(' ');
        std::string_view typeSv = (sp == std::string_view::npos) ? decl : decl.substr(0, sp);
        std::string_view nameSv = (sp == std::string_view::npos) ? std::string_view() : decl.substr(sp + 1);
        Param p;
        p.type = std::string(typeSv);
        auto valOpt = nextTokenView();
        if (!valOpt) break;
        std::string_view val = *valOpt;
        if (val == "[") {
            while (true) {
                auto vOpt = nextTokenView();
                if (!vOpt || *vOpt == "]") break;
                std::string_view v = *vOpt;
                if (p.type == "integer") {
                    int iv = 0;
                    auto res = std::from_chars(v.data(), v.data() + v.size(), iv);
                    if (res.ec != std::errc() || res.ptr != v.data() + v.size()) {
                        // fallback for cases from_chars doesn't handle (e.g. hex)
                        iv = (int)std::strtol(std::string(v).c_str(), nullptr, 10);
                    }
                    p.ints.push_back(iv);
                } else if (p.type == "string" || p.type == "texture") {
                    std::string_view dv = dequoteView(v);
                    p.strings.push_back(std::string(dv));
                } else {
                    float fv = 0.f;
                    auto res = std::from_chars(v.data(), v.data() + v.size(), fv);
                    if (res.ec != std::errc() || res.ptr != v.data() + v.size()) {
                        fv = (float)std::strtod(std::string(v).c_str(), nullptr);
                    }
                    p.floats.push_back(fv);
                }
            }
        } else {
            if (p.type == "integer") {
                int iv = 0;
                auto res = std::from_chars(val.data(), val.data() + val.size(), iv);
                if (res.ec != std::errc() || res.ptr != val.data() + val.size()) {
                    iv = (int)std::strtol(std::string(val).c_str(), nullptr, 10);
                }
                p.ints.push_back(iv);
            } else if (p.type == "string" || p.type == "texture") {
                std::string_view dv = dequoteView(val);
                p.strings.push_back(std::string(dv));
            } else {
                float fv = 0.f;
                auto res = std::from_chars(val.data(), val.data() + val.size(), fv);
                if (res.ec != std::errc() || res.ptr != val.data() + val.size()) {
                    fv = (float)std::strtod(std::string(val).c_str(), nullptr);
                }
                p.floats.push_back(fv);
            }
        }
        out[std::string(nameSv)] = std::move(p);
    }
    return out;
}

std::string PbrtParser::readName() {
    auto t = nextTokenView();
    if (!t) return std::string();
    std::string_view dv = dequoteView(*t);
    return std::string(dv);
}

float PbrtParser::readFloat() {
    auto t = nextTokenView();
    if (!t) return 0.f;
    std::string_view v = *t;
    float fv = 0.f;
    auto res = std::from_chars(v.data(), v.data() + v.size(), fv);
    if (res.ec == std::errc() && res.ptr == v.data() + v.size()) return fv;
    return (float)std::strtod(std::string(v).c_str(), nullptr);
}

Mat4 PbrtParser::readMat16() {
    Mat4 m{};
    // the file stores column-major floats: flat idx i maps to [i/4][i%4]
    for (int i = 0; i < 16; ++i) m[i / 4][i % 4] = readFloat();
    return m;
}

void PbrtParser::expectToken(std::string_view lit) {
    auto t = nextTokenView();
    if (!t || *t != lit)
        addWarning(std::string("pbrt: expected '") + std::string(lit) + "' in matrix statement");
}

void PbrtParser::pushState() { states_.push_back(states_.back()); }
void PbrtParser::popState() { if (states_.size() > 1) states_.pop_back(); }

void PbrtParser::parseStream() {
    while (auto tok = nextTokenView()) {
        if (isCancelled()) {
            lastError_ = "cancelled";
            fireAllDone(*result_, false, lastError_);
            return;
        }
        std::string_view s = *tok;
        if (s.empty() || s[0] == '#') continue;
        if (s[0] == '"' || s[0] == '[' || s[0] == ']') {
            if (s[0] == '"') parseParams();
            continue;
        }
        std::string low = toLower(s);
        dispatch(low, s);
    }
}

void PbrtParser::dispatch(std::string_view low, std::string_view /*raw*/) {
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
    } else if (low == "lookat") {
        for (int k = 0; k < 3; ++k) lookPos_[k] = readFloat();
        for (int k = 0; k < 3; ++k) lookTarget_[k] = readFloat();
        for (int k = 0; k < 3; ++k) lookUp_[k] = readFloat();
        hasLookAt_ = true;
    } else if (low == "camera") {
        std::string type = readName();
        ParamMap params = parseParams();
        handleCamera(type, params);
    } else if (low == "film") {
        std::string /*type*/ t = readName();
        ParamMap params = parseParams();
        auto x = params.find("xresolution");
        auto y = params.find("yresolution");
        if (x != params.end() && !x->second.ints.empty())
            filmW_ = x->second.ints[0];
        if (y != params.end() && !y->second.ints.empty())
            filmH_ = y->second.ints[0];
    } else if (low == "lightsource") {
        std::string type = readName();
        ParamMap params = parseParams();
        handleLight(type, params, false);
    } else if (low == "arealightsource") {
        std::string type = readName();
        ParamMap params = parseParams();
        ParseState& s = states_.back();
        s.area = true;
        copyColor_(params, s.areaCol, s.areaInt);
        areaTypeName_ = type;
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
        if (peekView() && isQuoted(*peekView())) {
            nextTokenView();
            parseParams();
        }
    }
}

// resolve a light color/intensity from the common parameter spellings
void PbrtParser::copyColor_(const ParamMap& params, float out[3],
                            float& inten) {
    for (const char* key : {"L", "I", "color"}) {
        auto it = params.find(key);
        if (it == params.end() || it->second.type == "texture" ||
            it->second.floats.empty())
            continue;
        const auto& f = it->second.floats;
        out[0] = f[0];
        out[1] = f.size() > 1 ? f[1] : f[0];
        out[2] = f.size() > 2 ? f[2] : f[0];
        break;
    }
    auto sc = params.find("scale");
    if (sc == params.end()) sc = params.find("multiplier");
    if (sc != params.end() && !sc->second.floats.empty())
        inten = sc->second.floats[0];
}

void PbrtParser::handleCamera(const std::string& type, const ParamMap& params) {
    PackCamera& c = addCamera(*result_, intern(type));
    if (hasLookAt_) {
        for (int k = 0; k < 3; ++k) {
            c.position[k] = lookPos_[k];
            c.target[k] = lookTarget_[k];
            c.up[k] = lookUp_[k];
        }
    }
    auto fov = params.find("fov");
    if (fov != params.end() && !fov->second.floats.empty())
        c.fovYDegrees = fov->second.floats[0];
    auto zn = params.find("znear");
    if (zn != params.end() && !zn->second.floats.empty())
        c.nearZ = zn->second.floats[0];
    auto zf = params.find("zfar");
    if (zf != params.end() && !zf->second.floats.empty())
        c.farZ = zf->second.floats[0];
    if (filmW_ > 0 && filmH_ > 0)
        c.aspect = float(filmW_) / float(filmH_);
    result_->activeCamera = int32_t(result_->cameras.size()) - 1;
}

LightKind kindFromTypeName(std::string_view t) {
    if (t == "infinite") return LightKind::Infinite;
    if (t == "distant") return LightKind::Directional;
    if (t == "point") return LightKind::Point;
    if (t == "spot") return LightKind::Spot;
    return LightKind::Unknown;
}

void PbrtParser::handleLight(const std::string& type, const ParamMap& params,
                             bool /*area*/) {
    PackLight& L =
        addLight(*result_, kindFromTypeName(type), intern(type));
    copyColor_(params, L.color, L.intensity);
    auto pos = params.find("from");
    if (pos == params.end()) pos = params.find("position");
    if (pos != params.end() && pos->second.floats.size() >= 3)
        for (int q = 0; q < 3; ++q) L.position[q] = pos->second.floats[q];
    // placed lights inherit the current transform (pbrt semantics):
    // LightSource "point" after "Translate x y z" sits at that point
    if (L.kind == LightKind::Point || L.kind == LightKind::Spot) {
        float ox, oy, oz;
        matTransformPoint(ctm(), L.position[0], L.position[1],
                          L.position[2], ox, oy, oz);
        L.position[0] = ox;
        L.position[1] = oy;
        L.position[2] = oz;
    }
    auto dir = params.find("direction");
    if (dir == params.end()) dir = params.find("dir");
    if (dir == params.end()) dir = params.find("to");
    if (dir != params.end() && dir->second.floats.size() >= 3)
        for (int q = 0; q < 3; ++q) L.direction[q] = dir->second.floats[q];
}

// bind the active AreaLightSource (if any) to the geometry just emitted.
// Inline shapes know their mesh index immediately; deferred ply tasks get
// an encoded negative index resolved once their metas concatenate.
void PbrtParser::attachAreaLight_(int taskIdx) {
    const ParseState& s = states_.back();
    if (!s.area) return;
    PackLight& L = addLight(*result_, LightKind::Area,
                            intern(areaTypeName_));
    L.color[0] = s.areaCol[0];
    L.color[1] = s.areaCol[1];
    L.color[2] = s.areaCol[2];
    L.intensity = s.areaInt;
    L.meshIndex = taskIdx < 0
                      ? (meshMeta_.empty() ? -1 : int32_t(meshMeta_.size()) - 1)
                      : int32_t(-(2 + taskIdx));
}

void PbrtParser::handleInclude(std::string_view filename) {
    std::string dir = files_.empty() ? sourceDir_ : files_.back().dir;
    std::string full = combinePath(dir, std::string(filename));
    auto mf = MappedFile::openShared(full);
    if (!mf) {
        addWarning("pbrt: Include '" + std::string(filename) + "' not found");
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
    float lbmin[3] = {kInf, kInf, kInf};
    float lbmax[3] = {-kInf, -kInf, -kInf};
    for (size_t i = 0; i < vcount; ++i) {
        float x = P[i * 3], y = P[i * 3 + 1], z = P[i * 3 + 2];
        posPool_.push_back(x);
        posPool_.push_back(y);
        posPool_.push_back(z);
        for (int k = 0; k < 3; ++k) {
            float v = (k == 0) ? x : (k == 1) ? y : z;
            if (v < lbmin[k]) lbmin[k] = v;
            if (v > lbmax[k]) lbmax[k] = v;
        }
    }
    float bmin[3] = {kInf, kInf, kInf};
    float bmax[3] = {-kInf, -kInf, -kInf};
    for (int c = 0; c < 8; ++c) {
        float lx = (c & 1) ? lbmax[0] : lbmin[0];
        float ly = (c & 2) ? lbmax[1] : lbmin[1];
        float lz = (c & 4) ? lbmax[2] : lbmin[2];
        float wx, wy, wz;
        matTransformPoint(ctm(), lx, ly, lz, wx, wy, wz);
        float vs[3] = {wx, wy, wz};
        for (int k = 0; k < 3; ++k) {
            if (vs[k] < bmin[k]) bmin[k] = vs[k];
            if (vs[k] > bmax[k]) bmax[k] = vs[k];
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
                    nrmPool_.push_back((*N)[i * 3]);
                    nrmPool_.push_back((*N)[i * 3 + 1]);
                    nrmPool_.push_back((*N)[i * 3 + 2]);
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
    {
        Mat4 M = ctm();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) m.world[r * 4 + c] = M[c][r];
        m.hasWorld = !(M == matIdentity());
        glm::mat3 nm = glm::inverseTranspose(glm::mat3(M));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) m.normalWorld[r * 3 + c] = nm[c][r];
    }
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
        attachAreaLight_(-1);
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
        attachAreaLight_(int(deferredPly_.size()) - 1);
    }
}

// Cheap PLY header scan: returns the vertex and face element counts plus
// whether the vertex element carries uv roles. Used to pre-size the GPU
// geometry stream so pbrt can upload geometry concurrently with the
// (heavy) parallel .ply load instead of after it. Index count is taken as
// faceCount*3 (triangle assumption) -- correct for bistro and all triangle
// ply; non-triangle ply still parses correctly via the blocking path
// (--nostream), which sizes from the real index buffers.
static bool plyHeaderCounts(const std::string& path, size_t& verts,
                            size_t& faces, bool& hasUv) {
    auto mf = MappedFile::openShared(path);
    if (!mf) return false;
    std::string_view text = mf->text();
    verts = faces = 0;
    hasUv = false;
    bool inVertex = false;
    size_t pos = 0;
    const size_t n = text.size();
    while (pos < n) {
        size_t nl = text.find('\n', pos);
        size_t le = (nl == std::string_view::npos) ? n : nl;
        std::string_view line = text.substr(pos, le - pos);
        if (!line.empty() && line.back() == '\r')
            line = line.substr(0, line.size() - 1);
        size_t next = (nl == std::string_view::npos) ? n : nl + 1;
        size_t i = 0;
        auto tok = [&](std::string_view& out) {
            while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
            size_t s = i;
            while (i < line.size() && line[i] != ' ' && line[i] != '\t') ++i;
            out = line.substr(s, i - s);
        };
        std::string_view t0, t1, t2;
        tok(t0);
        tok(t1);
        tok(t2);
        if (t0 == "element") {
            inVertex = false;
            if (t1 == "vertex") {
                try {
                    verts = std::stoul(std::string(t2));
                } catch (...) {
                }
                inVertex = true;
            } else if (t1 == "face") {
                try {
                    faces = std::stoul(std::string(t2));
                } catch (...) {
                }
            }
        } else if (t0 == "property" && inVertex) {
            size_t sp = line.find_last_not_of(" \t");
            size_t lp = (sp == std::string_view::npos)
                            ? std::string_view::npos
                            : line.find_last_of(" \t", sp);
            std::string_view role = (lp != std::string_view::npos)
                                        ? line.substr(lp + 1)
                                        : t2;
            if (role == "u" || role == "v" || role == "s" || role == "t")
                hasUv = true;
        } else if (t0 == "end_header") {
            break;
        }
        pos = next;
    }
    return true;
}

bool PbrtParser::load(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    files_.clear();
    ungot_.reset();
    ungotView_.reset();
    dequotedViewBuf_.clear();
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
    const auto tTok = std::chrono::steady_clock::now();
    parseStream();
    AP_LOG("pbrt", "token pass: %.0f ms (%zu shapes, %zu deferred ply)",
           double(std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - tTok)
                      .count()),
           meshMeta_.size(), deferredPly_.size());

    // deferred plymesh: load + concat + (when streaming) GPU-upload in
    // parallel so the upload overlaps the heavy parallel .ply load rather
    // than serializing after it. The pools already hold the inline
    // trianglemesh data from the token pass.
    if (!deferredPly_.empty()) {
        const auto tPly = std::chrono::steady_clock::now();
        unsigned workers = threads_ ? threads_
                                    : std::thread::hardware_concurrency();
        if (!workers) workers = 4;
        std::vector<MergedPly> merged(deferredPly_.size());
        std::vector<int32_t> metaStart(deferredPly_.size(), -1);

        // Streaming pre-pass: size the GPU buffers up front (exact verts/
        // tris) so onMeta can fire before the load and the upload runs
        // concurrently with it. Index count assumes triangle ply.
        const bool streaming = (bool)geoStream().onMeta;
        if (streaming) {
            size_t inlineVerts = 0, inlineIdx = 0;
            for (const MeshMeta& m : meshMeta_) {
                inlineVerts += m.posCount;
                inlineIdx += m.idxCount;
            }
            size_t plyVerts = 0, plyFaces = 0;
            bool plyHasUv = false;
            for (const DeferredPly& t : deferredPly_) {
                size_t v = 0, f = 0;
                bool u = false;
                if (plyHeaderCounts(t.full, v, f, u)) {
                    plyVerts += v;
                    plyFaces += f;
                    if (u) plyHasUv = true;
                }
            }
            const size_t totalVerts = inlineVerts + plyVerts;
            const size_t totalIdx = inlineIdx + plyFaces * 3;
            const bool hasUv = !uvPool_.empty() || plyHasUv;
            geoStream().onMeta(totalVerts, totalIdx / 3, hasUv);
            posPool_.reserve(totalVerts * 3);
            nrmPool_.reserve(std::max(nrmPool_.size(), totalVerts * 3));
            if (hasUv)
                uvPool_.reserve(totalVerts * 2);
            idxPool_.reserve(totalIdx);
            // the inline (already-parsed) ranges sit at the front of the
            // pools; publish them now so their upload overlaps the load too
            if (!posPool_.empty())
                geoStream().onRange(ap::GeoRangeKind::Pos, 0,
                                    posPool_.data(),
                                    posPool_.size() * sizeof(float));
            if (!idxPool_.empty())
                geoStream().onRange(ap::GeoRangeKind::Idx, 0,
                                    idxPool_.data(),
                                    idxPool_.size() * sizeof(uint32_t));
        }

        if (streaming) {
            // each task: load (parallel, heavy) then concat + publish its
            // slice under the pool lock so appends stay ordered and the
            // published offsets line up with the final pool layout
            std::mutex poolMx;
            auto& exec = ap::globalExecutor();
            tf::Taskflow flow;
            flow.for_each_index(0, int(deferredPly_.size()), 1,
                                [&](int i) {
                if (isCancelled()) return;
                loadOnePly(deferredPly_[size_t(i)], merged[size_t(i)]);
                std::lock_guard<std::mutex> lk(poolMx);
                const size_t bPos = posPool_.size() * sizeof(float);
                const size_t bIdx = idxPool_.size() * sizeof(uint32_t);
                metaStart[size_t(i)] = int32_t(meshMeta_.size());
                concatPly(merged[size_t(i)]);
                const size_t ePos = posPool_.size() * sizeof(float);
                const size_t eIdx = idxPool_.size() * sizeof(uint32_t);
                if (ePos > bPos)
                    geoStream().onRange(
                        ap::GeoRangeKind::Pos, bPos,
                        &posPool_[bPos / sizeof(float)], ePos - bPos);
                if (eIdx > bIdx)
                    geoStream().onRange(
                        ap::GeoRangeKind::Idx, bIdx,
                        &idxPool_[bIdx / sizeof(uint32_t)], eIdx - bIdx);
            });
            exec.run(flow).wait();
            // uv is small (~50MB) and zero-filled for no-uv meshes; publish
            // it whole once everything is concatenated
            if (!uvPool_.empty())
                geoStream().onRange(ap::GeoRangeKind::Uv, 0,
                                    uvPool_.data(),
                                    uvPool_.size() * sizeof(float));
        } else {
            // ---- existing blocking path (parsers without GeoStreamSink
            // support, or --nostream): load all, then concat, then let the
            // viewer do one blocking setGeometry ----
            if (workers > 1 && deferredPly_.size() > 1) {
                auto& exec = ap::globalExecutor();
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
            const auto tLoaded = std::chrono::steady_clock::now();
            // pre-reserve the global pools: successive per-task inserts
            // would otherwise reallocate the whole pool every time
            // (O(n^2) copying - 6.2 s of concat on bistro before this)
            size_t totPos = posPool_.size(), totNrm = nrmPool_.size();
            size_t totUv = uvPool_.size(), totIdx = idxPool_.size();
            for (const MergedPly& m : merged) {
                totPos += m.pos.size();
                totNrm += m.nrm.size();
                totUv += m.uv.size();
                totIdx += m.idx.size();
            }
            const size_t totVerts = totPos / 3;
            posPool_.reserve(totPos);
            nrmPool_.reserve(std::max(totNrm, totVerts * 3));
            uvPool_.reserve(std::max(totUv, totVerts * 2));
            idxPool_.reserve(totIdx);
            for (const MergedPly& m : merged) {
                metaStart[&m - merged.data()] = int32_t(meshMeta_.size());
                concatPly(m);
            }
            AP_LOG("pbrt", "ply phase: %.0f ms load+transform, %.0f ms concat",
                   double(std::chrono::duration_cast<std::chrono::milliseconds>(
                              tLoaded - tPly)
                              .count()),
                   double(std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - tLoaded)
                              .count()));
        }

        const auto tDone = std::chrono::steady_clock::now();
        AP_LOG("pbrt", "ply phase: %.0f ms (streaming %d)",
               double(std::chrono::duration_cast<std::chrono::milliseconds>(
                          tDone - tPly)
                          .count()),
               int(streaming));

        // resolve area lights that were attached to deferred ply tasks
        for (PackLight& L : result_->lights)
            if (L.kind == LightKind::Area && L.meshIndex < -1)
                L.meshIndex = metaStart[size_t(-2 - L.meshIndex)];
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
        for (int i = 0; i < 16; ++i) pm.world[i] = m.world[i];
        pm.hasWorld = m.hasWorld;
        for (int i = 0; i < 9; ++i) pm.normalWorld[i] = m.normalWorld[i];
        result_->meshes.push_back(std::move(pm));
    }

    fireProgress(100.f);
    fireVertices(*result_, result_->meshes);
    fireMaterials(*result_, result_->materials);
    // textures: the consumer (viewer) reads + decodes these references on
    // its own background pool; firing them as one batch at parse end lets
    // the decode run uncontented (overlapping decode with the parse tail
    // regresses wall time here because both stages are CPU-bound and share
    // cores -- see benchmark.md).
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
    // store positions in local space; world CTM will be applied on GPU
    float lbmin[3] = {kInf, kInf, kInf};
    float lbmax[3] = {-kInf, -kInf, -kInf};
    for (size_t i = 0; i < nv; ++i) {
        float x = m.positions[i * 3], y = m.positions[i * 3 + 1],
              z = m.positions[i * 3 + 2];
        out.pos.push_back(x);
        out.pos.push_back(y);
        out.pos.push_back(z);
        for (int k = 0; k < 3; ++k) {
            float v = (k == 0) ? x : (k == 1) ? y : z;
            if (v < lbmin[k]) lbmin[k] = v;
            if (v > lbmax[k]) lbmax[k] = v;
        }
    }
    // world bounds from 8 local corners to avoid per-vertex CPU transform
    float bmin[3] = {kInf, kInf, kInf};
    float bmax[3] = {-kInf, -kInf, -kInf};
    for (int c = 0; c < 8; ++c) {
        float lx = (c & 1) ? lbmax[0] : lbmin[0];
        float ly = (c & 2) ? lbmax[1] : lbmin[1];
        float lz = (c & 4) ? lbmax[2] : lbmin[2];
        float wx, wy, wz;
        matTransformPoint(M, lx, ly, lz, wx, wy, wz);
        float vs[3] = {wx, wy, wz};
        for (int k = 0; k < 3; ++k) {
            if (vs[k] < bmin[k]) bmin[k] = vs[k];
            if (vs[k] > bmax[k]) bmax[k] = vs[k];
        }
    }

    const bool meshNrm = wantNormals_ && !m.normals.empty();
    if (meshNrm || !out.nrm.empty()) {
        if (out.nrm.size() / 3 < base) out.nrm.resize(base * 3, 0.f);
        if (meshNrm) {
            for (size_t i = 0; i < nv && i * 3 + 2 < m.normals.size(); ++i) {
                out.nrm.push_back(m.normals[i * 3]);
                out.nrm.push_back(m.normals[i * 3 + 1]);
                out.nrm.push_back(m.normals[i * 3 + 2]);
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
    // store world CTM for GPU (row-major)
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) mm.world[r * 4 + c] = M[c][r];
    mm.hasWorld = !(M == matIdentity());
    {
        glm::mat3 nm = glm::inverseTranspose(glm::mat3(M));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) mm.normalWorld[r * 3 + c] = nm[c][r];
    }
    out.meta.push_back(mm);
}

void PbrtParser::loadOnePly(const DeferredPly& t, MergedPly& out) {
    out.mat = t.mat;
    if (auto pf = MappedFile::openShared(t.full)) pf->prefetch(0, pf->size());
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
