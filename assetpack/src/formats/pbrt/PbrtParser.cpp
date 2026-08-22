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
// Transform matrix math uses a tiny self-contained mat4 (column-major).
// ============================================================

#include "assetpack/AssetPack.h"

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

using Mat4 = std::array<float, 16>;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kInf = 1e30f;

float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

Mat4 matIdentity() {
    Mat4 m{};
    m[0] = m[5] = m[10] = m[15] = 1.f;
    return m;
}

Mat4 matMul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            float s = 0.f;
            for (int k = 0; k < 4; ++k)
                s += a[k * 4 + row] * b[c * 4 + k];
            r[c * 4 + row] = s;
        }
    return r;
}

Mat4 matTranslate(float x, float y, float z) {
    Mat4 m = matIdentity();
    m[12] = x;
    m[13] = y;
    m[14] = z;
    return m;
}

Mat4 matScale(float x, float y, float z) {
    Mat4 m{};
    m[0] = x;
    m[5] = y;
    m[10] = z;
    m[15] = 1.f;
    return m;
}

Mat4 matRotate(float angDeg, float ax, float ay, float az) {
    float len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len == 0.f) return matIdentity();
    ax /= len; ay /= len; az /= len;
    float a = angDeg * kPi / 180.f;
    float c = std::cos(a), s = std::sin(a), t = 1.f - c;
    float R[4][4] = {};
    R[0][0] = t * ax * ax + c;
    R[0][1] = t * ax * ay - s * az;
    R[0][2] = t * ax * az + s * ay;
    R[1][0] = t * ax * ay + s * az;
    R[1][1] = t * ay * ay + c;
    R[1][2] = t * ay * az - s * ax;
    R[2][0] = t * ax * az - s * ay;
    R[2][1] = t * ay * az + s * ax;
    R[2][2] = t * az * az + c;
    R[3][3] = 1.f;
    Mat4 m{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[j * 4 + i] = R[i][j];
    return m;
}

void matTransformPoint(const Mat4& m, float x, float y, float z,
                       float& ox, float& oy, float& oz) {
    float w = m[3] * x + m[7] * y + m[11] * z + m[15];
    ox = m[0] * x + m[4] * y + m[8] * z + m[12];
    oy = m[1] * x + m[5] * y + m[9] * z + m[13];
    oz = m[2] * x + m[6] * y + m[10] * z + m[14];
    if (w != 0.f) { ox /= w; oy /= w; oz /= w; }
}

bool matInvert(const Mat4& m, Mat4& out) {
    const float* a = m.data();
    float inv[16];
    inv[0] = a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15] + a[9]*a[7]*a[14] + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
    inv[4] = -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15] - a[8]*a[7]*a[14] - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
    inv[8] = a[4]*a[9]*a[15] - a[4]*a[11]*a[13] - a[8]*a[5]*a[15] + a[8]*a[7]*a[13] + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
    inv[12] = -a[4]*a[9]*a[14] + a[4]*a[10]*a[13] + a[8]*a[5]*a[14] - a[8]*a[6]*a[13] - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];
    inv[1] = -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15] - a[9]*a[3]*a[14] - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
    inv[5] = a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15] + a[8]*a[3]*a[14] + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
    inv[9] = -a[0]*a[9]*a[15] + a[0]*a[11]*a[13] + a[8]*a[1]*a[15] - a[8]*a[3]*a[13] - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
    inv[13] = a[0]*a[9]*a[14] - a[0]*a[10]*a[13] - a[8]*a[1]*a[14] + a[8]*a[2]*a[13] + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];
    inv[2] = a[1]*a[6]*a[15] - a[1]*a[7]*a[14] - a[5]*a[2]*a[15] + a[5]*a[3]*a[14] + a[13]*a[2]*a[7] - a[13]*a[3]*a[6];
    inv[6] = -a[0]*a[6]*a[15] + a[0]*a[7]*a[14] + a[4]*a[2]*a[15] - a[4]*a[3]*a[14] - a[12]*a[2]*a[7] + a[12]*a[3]*a[6];
    inv[10] = a[0]*a[5]*a[15] - a[0]*a[7]*a[13] - a[4]*a[1]*a[15] + a[4]*a[3]*a[13] + a[12]*a[1]*a[7] - a[12]*a[3]*a[5];
    inv[14] = -a[0]*a[5]*a[14] + a[0]*a[6]*a[13] + a[4]*a[1]*a[14] - a[4]*a[2]*a[13] - a[12]*a[1]*a[6] + a[12]*a[2]*a[5];
    inv[3] = -a[1]*a[6]*a[11] + a[1]*a[7]*a[10] + a[5]*a[2]*a[11] - a[5]*a[3]*a[10] - a[9]*a[2]*a[7] + a[9]*a[3]*a[6];
    inv[7] = a[0]*a[6]*a[11] - a[0]*a[7]*a[10] - a[4]*a[2]*a[11] + a[4]*a[3]*a[10] + a[8]*a[2]*a[7] - a[8]*a[3]*a[6];
    inv[11] = -a[0]*a[5]*a[11] + a[0]*a[7]*a[9] + a[4]*a[1]*a[11] - a[4]*a[3]*a[9] - a[8]*a[1]*a[7] + a[8]*a[3]*a[5];
    inv[15] = a[0]*a[5]*a[10] - a[0]*a[6]*a[9] - a[4]*a[1]*a[10] + a[4]*a[2]*a[9] + a[8]*a[1]*a[6] - a[8]*a[2]*a[5];
    float det = a[0]*inv[0] + a[1]*inv[4] + a[2]*inv[8] + a[3]*inv[12];
    if (det == 0.f) return false;
    float idet = 1.f / det;
    for (int i = 0; i < 16; ++i) out[i] = inv[i] * idet;
    return true;
}

void matTransformNormal(const Mat4& m, float x, float y, float z,
                        float& ox, float& oy, float& oz) {
    Mat4 inv;
    if (!matInvert(m, inv)) { ox = x; oy = y; oz = z; return; }
    ox = inv[0]*x + inv[1]*y + inv[2]*z;
    oy = inv[4]*x + inv[5]*y + inv[6]*z;
    oz = inv[8]*x + inv[9]*y + inv[10]*z;
    float len = std::sqrt(ox*ox + oy*oy + oz*oz);
    if (len > 0.f) { ox /= len; oy /= len; oz /= len; }
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
    explicit PbrtParser(unsigned /*threads*/ = 0) {
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
    bool loadPlyMesh(const std::string& filename);
    bool loadPlyAsset(const std::string& full);
    bool mergePlyViaAssetPack(const std::string& full);

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
    for (int i = 0; i < 16; ++i) m[i] = readFloat();
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

    auto it = params.find("Kd");
    if (it != params.end()) {
        if (it->second.type == "texture") {
            std::string texname = it->second.strings.empty() ? std::string()
                                                             : it->second.strings[0];
            bindMaterialTexture(mat, texname, TexDiffuse);
        } else if (!it->second.floats.empty()) {
            const auto& f = it->second.floats;
            mat.diffuse[0] = clamp01(f[0]);
            mat.diffuse[1] = clamp01(f.size() > 1 ? f[1] : f[0]);
            mat.diffuse[2] = clamp01(f.size() > 2 ? f[2] : f[0]);
            mat.diffuse[3] = 1.f;
        }
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
    if (wantNormals_ && N && !N->empty()) {
        size_t avail = N->size() / 3;
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
        ncount = vcount;
    }

    size_t ubase = uvPool_.size() / 2, ucount = 0;
    if (wantTexcoords_ && uv && !uv->empty()) {
        size_t avail = uv->size() / 2;
        for (size_t i = 0; i < vcount; ++i) {
            if (i < avail) {
                uvPool_.push_back((*uv)[i * 2]);
                uvPool_.push_back((*uv)[i * 2 + 1]);
            } else {
                uvPool_.push_back(0.f); uvPool_.push_back(0.f);
            }
        }
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
        if (!loadPlyMesh(fit->second.strings[0]))
            addWarning("pbrt: plymesh external file skipped");
    }
}

// legacy fallback: minimal inline ASCII PLY reader. Binary little-endian
// files take the mergePlyViaAssetPack path first (see below).
bool PbrtParser::loadPlyAsset(const std::string& full) {
    auto mf = MappedFile::openShared(full);
    if (!mf) { addWarning("pbrt: plymesh cannot open '" + full + "'"); return false; }
    openFiles_.push_back(mf);
    std::string_view txt = mf->text();

    size_t eh = txt.find("end_header");
    if (eh == std::string_view::npos) { addWarning("pbrt: plymesh no end_header"); return false; }
    std::string_view header = txt.substr(0, eh);
    if (header.find("format binary") != std::string_view::npos) {
        addWarning("pbrt: plymesh binary PLY unsupported");
        return false;
    }

    long vcount = 0, fcount = 0, vprops = 0;
    bool hasFace = false;
    size_t line = 0;
    while (line < header.size()) {
        size_t eol = header.find('\n', line);
        if (eol == std::string_view::npos) eol = header.size();
        std::string_view l = header.substr(line, eol - line);
        line = eol + 1;
        if (l.find("element vertex") != std::string_view::npos) {
            size_t sp = l.rfind(' ');
            if (sp != std::string_view::npos)
                vcount = std::strtol(l.substr(sp + 1).data(), nullptr, 10);
        } else if (l.find("element face") != std::string_view::npos) {
            size_t sp = l.rfind(' ');
            if (sp != std::string_view::npos) {
                fcount = std::strtol(l.substr(sp + 1).data(), nullptr, 10);
                hasFace = true;
            }
        } else if (l.find("property") != std::string_view::npos) {
            vprops++;
        }
    }
    if (vcount <= 0 || !hasFace) {
        addWarning("pbrt: plymesh header lacks vertex/face counts");
        return false;
    }
    if (vprops < 3) vprops = 3;

    size_t dataStart = txt.find('\n', eh) + 1;
    std::string_view data = (dataStart < txt.size()) ? txt.substr(dataStart)
                                                     : std::string_view();
    std::vector<std::string_view> toks;
    size_t p = 0;
    while (p < data.size()) {
        while (p < data.size() && (data[p] == ' ' || data[p] == '\n' || data[p] == '\t' ||
                                    data[p] == '\r'))
            p++;
        if (p >= data.size()) break;
        size_t q = p;
        while (q < data.size() && data[q] != ' ' && data[q] != '\n' && data[q] != '\t' &&
               data[q] != '\r')
            q++;
        toks.push_back(data.substr(p, q - p));
        p = q;
    }

    size_t ti = 0;
    std::vector<float> P;
    P.reserve((size_t)vcount * 3);
    for (long v = 0; v < vcount && ti + (size_t)vprops <= toks.size(); ++v) {
        float x = (float)std::strtod(toks[ti + 0].data(), nullptr);
        float y = (float)std::strtod(toks[ti + 1].data(), nullptr);
        float z = (float)std::strtod(toks[ti + 2].data(), nullptr);
        P.push_back(x); P.push_back(y); P.push_back(z);
        ti += (size_t)vprops;
    }
    std::vector<int> indices;
    for (long f = 0; f < fcount; ++f) {
        if (ti >= toks.size()) break;
        long cnt = std::strtol(toks[ti].data(), nullptr, 10);
        ti++;
        if ((long)indices.size() + cnt > (long)toks.size()) break;
        if (cnt == 3) {
            indices.push_back((int)std::strtol(toks[ti + 0].data(), nullptr, 10));
            indices.push_back((int)std::strtol(toks[ti + 1].data(), nullptr, 10));
            indices.push_back((int)std::strtol(toks[ti + 2].data(), nullptr, 10));
        } else if (cnt == 4) {
            int a = (int)std::strtol(toks[ti + 0].data(), nullptr, 10);
            int b = (int)std::strtol(toks[ti + 1].data(), nullptr, 10);
            int c = (int)std::strtol(toks[ti + 2].data(), nullptr, 10);
            int d = (int)std::strtol(toks[ti + 3].data(), nullptr, 10);
            indices.push_back(a); indices.push_back(b); indices.push_back(c);
            indices.push_back(a); indices.push_back(c); indices.push_back(d);
        } else {
            for (long k = 1; k + 1 < cnt; ++k) {
                indices.push_back((int)std::strtol(toks[ti + 0].data(), nullptr, 10));
                indices.push_back((int)std::strtol(toks[ti + k].data(), nullptr, 10));
                indices.push_back((int)std::strtol(toks[ti + k + 1].data(), nullptr, 10));
            }
        }
        ti += (size_t)cnt;
    }

    if (P.empty()) return false;
    emitMesh("plymesh", P, nullptr, nullptr, indices);
    return true;
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

// plymesh via the shared PLY parser (AssetPack -> "ply" -> PlyParser):
// handles ascii AND binary_little_endian. Each ply mesh is appended under
// the current CTM so it lands in world space like a trianglemesh shape,
// with the active pbrt material assigned.
bool PbrtParser::mergePlyViaAssetPack(const std::string& full) {
    AssetPack ply;
    if (!ply.load(full)) {
        addWarning("pbrt: plymesh PLY parser failed on '" + full + "'");
        return false;
    }
    const PackResult& pr = ply.result();
    if (pr.meshes.empty()) return false;

    const Mat4 M = ctm();
    for (const PackMesh& m : pr.meshes) {
        if (m.positions.empty()) continue;
        const size_t base = posPool_.size() / 3;
        const size_t nv = m.positions.size() / 3;
        float mbmin[3] = {kInf, kInf, kInf};
        float mbmax[3] = {-kInf, -kInf, -kInf};
        for (size_t i = 0; i < nv; ++i) {
            float ox, oy, oz;
            matTransformPoint(M, m.positions[i * 3], m.positions[i * 3 + 1],
                              m.positions[i * 3 + 2], ox, oy, oz);
            posPool_.push_back(ox);
            posPool_.push_back(oy);
            posPool_.push_back(oz);
            for (int k = 0; k < 3; ++k) {
                const float v = (k == 0) ? ox : (k == 1) ? oy : oz;
                if (v < mbmin[k]) mbmin[k] = v;
                if (v > mbmax[k]) mbmax[k] = v;
            }
        }

        size_t nbase = nrmPool_.size() / 3, ncount = 0;
        if (wantNormals_ && !m.normals.empty()) {
            for (size_t i = 0; i < nv && i * 3 + 2 < m.normals.size(); ++i) {
                float ox, oy, oz;
                matTransformNormal(M, m.normals[i * 3], m.normals[i * 3 + 1],
                                   m.normals[i * 3 + 2], ox, oy, oz);
                nrmPool_.push_back(ox);
                nrmPool_.push_back(oy);
                nrmPool_.push_back(oz);
            }
            ncount = nrmPool_.size() / 3 - nbase;
        }

        size_t ubase = uvPool_.size() / 2, ucount = 0;
        if (wantTexcoords_ && !m.texcoords.empty()) {
            uvPool_.insert(uvPool_.end(), m.texcoords.begin(),
                           m.texcoords.end());
            ucount = uvPool_.size() / 2 - ubase;
        }

        size_t idxBase = idxPool_.size();
        for (uint32_t ix : m.indices)
            idxPool_.push_back(uint32_t(base + ix));

        MeshMeta mm;
        mm.name = intern("plymesh " + std::to_string(meshCounter_++));
        mm.mat = states_.back().mat;
        mm.posStart = base;
        mm.posCount = nv;
        mm.nrmStart = nbase;
        mm.nrmCount = ncount;
        mm.uvStart = ubase;
        mm.uvCount = ucount;
        mm.idxStart = idxBase;
        mm.idxCount = idxPool_.size() - idxBase;
        for (int k = 0; k < 3; ++k) {
            mm.bmin[k] = mbmin[k];
            mm.bmax[k] = mbmax[k];
        }
        meshMeta_.push_back(mm);
    }
    return true;
}

bool PbrtParser::loadPlyMesh(const std::string& filename) {
    std::string dir = files_.empty() ? sourceDir_ : files_.back().dir;
    const std::string full = combinePath(dir, filename);
    if (mergePlyViaAssetPack(full)) return true;
    return loadPlyAsset(full);
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
