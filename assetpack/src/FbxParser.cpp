// FbxParser.cpp - minimal binary FBX (7400+) reader for assetpack
//
// Self-contained: depends only on the assetpack public API. It implements a
// from-scratch binary FBX tokenizer/reader (no assimp internals).
//
// Scope (minimal but correct):
//   * Binary FBX 7400+ (detected by the "Kaydara FBX Binary" magic).
//   * ASCII FBX is NOT supported: we bail with a warning.
//   * Geometry: Vertices, PolygonVertexIndex (fan triangulation honoring the
//     negative last-vertex marker), LayerElementNormal, LayerElementUV, Layer.
//   * Normals/UVs with MappingInformationType ByVert or ByPolygonVertex and
//     ReferenceInformationType Direct or IndexToDirect.
//   * Materials: DiffuseColor/EmissiveColor/etc. arrays and Properties70 P
//     entries; default white when absent.
//   * Connections: Geometry->Model and Model->Material (OO), Texture->Material
//     (OP, property-named) for texture slot binding.
//   * Compressed (zlib/deflate, encoding==1) array payloads are skipped with a
//     warning; raw (encoding==0) arrays are decoded.
//   * Skins, animations, and nested pose/camera data are ignored.
//
// Each PackMesh spans its slice of the result's global pools; bounds are
// computed from positions.

#include "assetpack/AssetPack.h"

#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ap {

namespace {

uint32_t readU32(const std::byte* p) {
    uint32_t v;
    std::memcpy(&v, p, 4);
    return v;
}

uint16_t readU16(const std::byte* p) {
    uint16_t v;
    std::memcpy(&v, p, 2);
    return v;
}

uint64_t readU64(const std::byte* p) {
    uint64_t v;
    std::memcpy(&v, p, 8);
    return v;
}

// One FBX property value. Array payloads are kept zero-copy (pointer into the
// file mapping) and decoded lazily.
struct Prop {
    char type = 0;
    double d = 0;
    float f = 0;
    int64_t l = 0;
    int32_t i = 0;
    bool b = false;
    bool isArr = false;
    std::string str; // S / R (raw bytes) / C text
    const std::byte* arr = nullptr;
    int32_t arrCount = 0;
    int32_t arrEnc = 0;
};

struct Node {
    bool isNull = false;
    std::string name;
    std::vector<Prop> props;
    std::vector<Node> children;
};

// Read one property starting at p; return pointer just past it (nullptr on
// unknown type / overflow).
const std::byte* readProp(const std::byte* p, const std::byte* fileEnd, Prop& o) {
    if (p + 1 > fileEnd) return nullptr;
    o = Prop{};
    o.type = static_cast<char>(*p++);
    switch (o.type) {
        case 'D':
            if (p + 8 > fileEnd) return nullptr;
            std::memcpy(&o.d, p, 8);
            return p + 8;
        case 'F':
            if (p + 4 > fileEnd) return nullptr;
            std::memcpy(&o.f, p, 4);
            return p + 4;
        case 'L':
            if (p + 8 > fileEnd) return nullptr;
            std::memcpy(&o.l, p, 8);
            return p + 8;
        case 'I':
            if (p + 4 > fileEnd) return nullptr;
            std::memcpy(&o.i, p, 4);
            return p + 4;
        case 'Y': {
            if (p + 2 > fileEnd) return nullptr;
            int16_t v;
            std::memcpy(&v, p, 2);
            o.i = v;
            return p + 2;
        }
        case 'C':
            if (p + 1 > fileEnd) return nullptr;
            o.b = (static_cast<uint8_t>(p[0]) != 0);
            return p + 1;
        case 'S':
        case 'R': {
            if (p + 4 > fileEnd) return nullptr;
            uint32_t len = readU32(p);
            if (p + 4 + len > fileEnd) return nullptr;
            o.str.assign(reinterpret_cast<const char*>(p + 4), len);
            return p + 4 + len;
        }
        case 'd':
        case 'l':
        case 'i':
        case 'f':
        case 'b': {
            if (p + 8 > fileEnd) return nullptr;
            int32_t count;
            std::memcpy(&count, p, 4);
            int32_t enc;
            std::memcpy(&enc, p + 4, 4);
            o.isArr = true;
            o.arrCount = count;
            o.arrEnc = enc;
            size_t esz = (o.type == 'd' || o.type == 'l') ? 8
                       : (o.type == 'i' || o.type == 'f') ? 4
                                                          : 1;
            const std::byte* data = p + 8;
            if (enc == 1) {
                o.arr = nullptr; // deflate: skip payload
            } else {
                o.arr = data;
            }
            return data + static_cast<size_t>(count) * esz;
        }
        case 's':
        case 'r': {
            if (p + 8 > fileEnd) return nullptr;
            int32_t count;
            std::memcpy(&count, p, 4);
            int32_t enc;
            std::memcpy(&enc, p + 4, 4);
            (void)enc;
            o.isArr = true;
            o.arrCount = count;
            o.arr = p + 8;
            const std::byte* q = p + 8;
            for (int32_t k = 0; k < count; ++k) {
                if (q + 4 > fileEnd) return nullptr;
                uint32_t len;
                std::memcpy(&len, q, 4);
                q += 4;
                if (q + len > fileEnd) return nullptr;
                q += len;
            }
            return q;
        }
        default:
            return nullptr; // unknown type code: cannot safely advance
    }
}

// Read one node record at nodeStart (absolute offset). Returns the pointer just
// past this node's subtree (== base + endOffset), or nullptr on corrupt data.
// Sets out.isNull when a 13-byte null terminator record is read.
const std::byte* readNode(const std::byte* base, const std::byte* fileEnd,
                          size_t nodeStart, Node& out) {
    if (nodeStart + 13 > static_cast<size_t>(fileEnd - base)) return nullptr;
    const std::byte* p = base + nodeStart;
    uint32_t endOffset = readU32(p);
    uint32_t numProps = readU32(p + 4);
    uint32_t propLen = readU32(p + 8);
    uint8_t nameLen = static_cast<uint8_t>(p[12]);

    if (endOffset == 0) {
        out.isNull = true;
        out.name.clear();
        return base + nodeStart + 13;
    }

    size_t absEnd = endOffset;
    if (absEnd > static_cast<size_t>(fileEnd - base)) return nullptr;

    const std::byte* namePtr = p + 13;
    if (namePtr + nameLen > fileEnd) return nullptr;
    out.isNull = false;
    out.name.assign(reinterpret_cast<const char*>(namePtr), nameLen);

    size_t propsStart = nodeStart + 13 + nameLen;
    size_t childrenStart = propsStart + propLen;
    if (childrenStart > absEnd) return nullptr;

    const std::byte* pp = base + propsStart;
    const std::byte* propsEnd = base + childrenStart;
    out.props.clear();
    out.children.clear();
    for (uint32_t i = 0; i < numProps; ++i) {
        Prop pr;
        pp = readProp(pp, propsEnd, pr);
        if (pp == nullptr) return nullptr;
        out.props.push_back(std::move(pr));
    }

    size_t childPos = childrenStart;
    while (childPos < absEnd) {
        Node child;
        const std::byte* after = readNode(base, fileEnd, childPos, child);
        if (after == nullptr) return nullptr;
        if (child.isNull) {
            childPos = static_cast<size_t>(after - base);
            break;
        }
        out.children.push_back(std::move(child));
        childPos = static_cast<size_t>(after - base);
    }
    return base + absEnd;
}

const Node* findChild(const Node& n, std::string_view name) {
    for (const auto& c : n.children)
        if (c.name == name) return &c;
    return nullptr;
}

// First property of a child node, if that property is an array.
const Prop* childArray(const Node& n, std::string_view name) {
    const Node* c = findChild(n, name);
    if (!c || c->props.empty() || !c->props[0].isArr) return nullptr;
    return &c->props[0];
}

double getDouble(const Prop& p, int i) {
    if (!p.arr) return 0.0;
    if (p.type == 'd') {
        double v;
        std::memcpy(&v, p.arr + static_cast<size_t>(i) * 8, 8);
        return v;
    }
    if (p.type == 'f') {
        float v;
        std::memcpy(&v, p.arr + static_cast<size_t>(i) * 4, 4);
        return static_cast<double>(v);
    }
    return 0.0;
}

int32_t getInt32(const Prop& p, int i) {
    if (!p.arr) return 0;
    if (p.type == 'i') {
        int32_t v;
        std::memcpy(&v, p.arr + static_cast<size_t>(i) * 4, 4);
        return v;
    }
    if (p.type == 'l') {
        int64_t v;
        std::memcpy(&v, p.arr + static_cast<size_t>(i) * 8, 8);
        return static_cast<int32_t>(v);
    }
    return 0;
}

int texTypeFromName(std::string_view n) {
    if (n == "DiffuseColor" || n == "Diffuse") return TexDiffuse;
    if (n == "AmbientColor" || n == "Ambient") return TexAmbient;
    if (n == "SpecularColor" || n == "Specular") return TexSpecular;
    if (n == "EmissiveColor" || n == "Emissive") return TexEmissive;
    if (n == "TransparentColor" || n == "Opacity" || n == "TransparencyFactor")
        return TexOpacity;
    if (n == "NormalMap" || n == "Bump" || n == "NormalColor") return TexNormal;
    return TexDiffuse;
}

void stripPrefix(std::string& s) {
    // FBX object names look like "Geometry::box" / "Model::box". Drop the
    // leading "Type::" so the mesh/material name is just the user label.
    auto pos = s.find("::");
    if (pos != std::string::npos && pos + 2 <= s.size()) {
        s = s.substr(pos + 2);
    }
}

} // namespace

class FbxParser final : public ModelParser {
public:
    explicit FbxParser(unsigned /*threads*/) {}

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;
    PackResult& result() override { return *result_; }

    void setWantNormals(bool want) override { wantNormals_ = want; }
    void setWantTexcoords(bool want) override { wantTexcoords_ = want; }
    void setWantTextureBytes(bool want) override { wantTextureBytes_ = want; }

private:
    struct Geom {
        uint64_t uid = 0;
        std::string name;
        std::vector<float> positions;
        std::vector<uint32_t> indices;
        std::vector<float> normals;    // empty when absent
        std::vector<float> texcoords;  // empty when absent
        uint64_t modelUid = 0;
    };

    struct MatInfo {
        uint64_t uid = 0;
        PackMaterial mat;
        int index = -1;
    };

    bool execute(std::string_view path);
    bool parseTree(const std::byte* base, const std::byte* fileEnd, size_t rootStart);
    int matIndexByUid(uint64_t uid) const;
    void buildMesh(const Geom& g, size_t posBase, size_t idxBase, size_t nrmBase,
                   size_t uvBase, bool hasNrm, bool hasUv);

    std::deque<std::string> names_; // stable storage for string_views

    std::vector<Geom> geoms_;
    std::vector<MatInfo> mats_;
    std::vector<Node> objects_; // top-level "Objects" children

    // connections
    std::vector<std::pair<uint64_t, uint64_t>> geomToModel_; // geom -> model
    std::vector<std::pair<uint64_t, uint64_t>> modelToMat_;  // model -> material
    std::vector<std::pair<uint64_t, uint64_t>> geomToMat_;   // geom -> material
    // texture uid -> (material uid, texType)
    std::vector<std::tuple<uint64_t, uint64_t, int>> texToMat_;

    bool wantNormals_ = true;
    bool wantTexcoords_ = true;
    bool wantTextureBytes_ = false;
};

void FbxParser::buildMesh(const Geom& g, size_t posBase, size_t idxBase,
                          size_t nrmBase, size_t uvBase, bool hasNrm, bool hasUv) {
    PackMesh m;
    m.name = std::string_view(g.name.data(), g.name.size());
    m.materialIndex = -1;

    const size_t vCount = g.positions.size() / 3;
    m.positions = std::span<const float>(result_->positions.data() + posBase,
                                         g.positions.size());
    m.indices = std::span<const uint32_t>(result_->posIndices.data() + idxBase,
                                          g.indices.size());
    if (hasNrm)
        m.normals = std::span<const float>(result_->normals.data() + nrmBase,
                                           g.normals.size());
    if (hasUv)
        m.texcoords = std::span<const float>(result_->texcoords.data() + uvBase,
                                             g.texcoords.size());

    // resolve material via connections
    int matIndex = -1;
    for (const auto& pr : geomToMat_)
        if (pr.first == g.uid) { matIndex = matIndexByUid(pr.second); break; }
    if (matIndex < 0 && g.modelUid) {
        for (const auto& pr : modelToMat_)
            if (pr.first == g.modelUid) { matIndex = matIndexByUid(pr.second); break; }
    }
    if (matIndex < 0 && !mats_.empty()) matIndex = 0; // first / default
    m.materialIndex = matIndex;

    // bounds from positions
    if (vCount > 0) {
        float mn[3] = { +1e30f, +1e30f, +1e30f };
        float mx[3] = { -1e30f, -1e30f, -1e30f };
        for (size_t i = 0; i < vCount; ++i) {
            for (int c = 0; c < 3; ++c) {
                const float v = m.positions[i * 3 + c];
                if (v < mn[c]) mn[c] = v;
                if (v > mx[c]) mx[c] = v;
            }
        }
        for (int c = 0; c < 3; ++c) {
            m.boundsMin[c] = mn[c];
            m.boundsMax[c] = mx[c];
        }
    }
    result_->meshes.push_back(std::move(m));
}

int FbxParser::matIndexByUid(uint64_t uid) const {
    for (const auto& mi : mats_)
        if (mi.uid == uid) return mi.index;
    return -1;
}

bool FbxParser::parseTree(const std::byte* base, const std::byte* fileEnd,
                          size_t rootStart) {
    // Read top-level node list (terminated by a null record).
    std::vector<Node> roots;
    size_t pos = rootStart;
    while (pos < static_cast<size_t>(fileEnd - base)) {
        Node n;
        const std::byte* after = readNode(base, fileEnd, pos, n);
        if (after == nullptr) break;
        if (n.isNull) break;
        roots.push_back(std::move(n));
        pos = static_cast<size_t>(after - base);
    }

    const Node* objects = nullptr;
    const Node* connections = nullptr;
    for (const auto& n : roots) {
        if (n.name == "Objects") objects = &n;
        else if (n.name == "Connections") connections = &n;
    }
    if (!objects) {
        lastError_ = "FBX has no Objects node";
        return false;
    }

    // ---- pass 1: gather objects ----
    for (const auto& obj : objects->children) {
        if (obj.name == "Geometry") {
            Geom g;
            if (!obj.props.empty() && obj.props[0].type == 'L')
                g.uid = static_cast<uint64_t>(obj.props[0].l);
            if (obj.props.size() > 1) {
                g.name = obj.props[1].str;
                stripPrefix(g.name);
            }
            const Prop* verts = childArray(obj, "Vertices");
            const Prop* pvi = childArray(obj, "PolygonVertexIndex");
            if (!verts || !pvi) {
                addWarning("Geometry without Vertices/PolygonVertexIndex skipped");
                continue;
            }
            const int vcount = verts->arrCount / 3;
            g.positions.reserve(static_cast<size_t>(vcount) * 3);
            for (int i = 0; i < vcount; ++i) {
                g.positions.push_back(static_cast<float>(getDouble(*verts, i * 3 + 0)));
                g.positions.push_back(static_cast<float>(getDouble(*verts, i * 3 + 1)));
                g.positions.push_back(static_cast<float>(getDouble(*verts, i * 3 + 2)));
            }

            // triangulate PolygonVertexIndex (fan), record per-polygon vertex
            // order in `polyVerts` for ByPolygonVertex attribute mapping.
            std::vector<uint32_t> polyVerts;
            std::vector<uint32_t> polyStarts;
            polyVerts.reserve(pvi->arrCount);
            std::vector<uint32_t> cur;
            auto flushPoly = [&]() {
                for (size_t f = 1; f + 1 < cur.size(); ++f) {
                    g.indices.push_back(cur[0]);
                    g.indices.push_back(cur[f]);
                    g.indices.push_back(cur[f + 1]);
                }
                polyStarts.push_back(static_cast<uint32_t>(polyVerts.size()));
                for (uint32_t cv : cur) polyVerts.push_back(cv);
                cur.clear();
            };
            for (int i = 0; i < pvi->arrCount; ++i) {
                int32_t v = getInt32(*pvi, i);
                if (v < 0) {
                    cur.push_back(static_cast<uint32_t>(-v - 1));
                    flushPoly();
                } else {
                    cur.push_back(static_cast<uint32_t>(v));
                }
            }
            if (!cur.empty()) flushPoly();
            polyStarts.push_back(static_cast<uint32_t>(polyVerts.size()));

            // ---- normals ----
            const Node* nl = findChild(obj, "LayerElementNormal");
            if (nl) {
                const Prop* np = childArray(*nl, "Normals");
                if (np && np->arrEnc == 1) {
                    addWarning("compressed FBX array skipped (Normals)");
                } else if (np) {
                    const Node* mit = findChild(*nl, "MappingInformationType");
                    const Node* rit = findChild(*nl, "ReferenceInformationType");
                    std::string mapping = mit ? mit->props[0].str : std::string();
                    bool indexed = rit && rit->props[0].str == "IndexToDirect";
                    const Prop* nidx = indexed ? childArray(*nl, "NormalIndex") : nullptr;
                    g.normals.assign(static_cast<size_t>(vcount) * 3, 0.f);
                    if (mapping == "ByVert") {
                        for (int v = 0; v < vcount; ++v) {
                            int src = v;
                            if (nidx) src = getInt32(*nidx, v);
                            g.normals[v * 3 + 0] = static_cast<float>(getDouble(*np, src * 3 + 0));
                            g.normals[v * 3 + 1] = static_cast<float>(getDouble(*np, src * 3 + 1));
                            g.normals[v * 3 + 2] = static_cast<float>(getDouble(*np, src * 3 + 2));
                        }
                    } else { // ByPolygonVertex (default)
                        for (size_t ppv = 0; ppv < polyVerts.size(); ++ppv) {
                            int src = static_cast<int>(ppv);
                            if (nidx) src = getInt32(*nidx, static_cast<int>(ppv));
                            uint32_t gv = polyVerts[ppv];
                            if (gv >= static_cast<uint32_t>(vcount)) continue;
                            g.normals[gv * 3 + 0] = static_cast<float>(getDouble(*np, src * 3 + 0));
                            g.normals[gv * 3 + 1] = static_cast<float>(getDouble(*np, src * 3 + 1));
                            g.normals[gv * 3 + 2] = static_cast<float>(getDouble(*np, src * 3 + 2));
                        }
                    }
                }
            }

            // ---- uvs ----
            const Node* uv = findChild(obj, "LayerElementUV");
            if (uv) {
                const Prop* up = childArray(*uv, "UV");
                if (up && up->arrEnc == 1) {
                    addWarning("compressed FBX array skipped (UV)");
                } else if (up) {
                    const Node* mit = findChild(*uv, "MappingInformationType");
                    const Node* rit = findChild(*uv, "ReferenceInformationType");
                    std::string mapping = mit ? mit->props[0].str : std::string();
                    bool indexed = rit && rit->props[0].str == "IndexToDirect";
                    const Prop* uidx = indexed ? childArray(*uv, "UVIndex") : nullptr;
                    g.texcoords.assign(static_cast<size_t>(vcount) * 2, 0.f);
                    if (mapping == "ByVert") {
                        for (int v = 0; v < vcount; ++v) {
                            int src = v;
                            if (uidx) src = getInt32(*uidx, v);
                            g.texcoords[v * 2 + 0] = static_cast<float>(getDouble(*up, src * 2 + 0));
                            g.texcoords[v * 2 + 1] = static_cast<float>(getDouble(*up, src * 2 + 1));
                        }
                    } else { // ByPolygonVertex (default)
                        for (size_t ppv = 0; ppv < polyVerts.size(); ++ppv) {
                            int src = static_cast<int>(ppv);
                            if (uidx) src = getInt32(*uidx, static_cast<int>(ppv));
                            uint32_t gv = polyVerts[ppv];
                            if (gv >= static_cast<uint32_t>(vcount)) continue;
                            g.texcoords[gv * 2 + 0] = static_cast<float>(getDouble(*up, src * 2 + 0));
                            g.texcoords[gv * 2 + 1] = static_cast<float>(getDouble(*up, src * 2 + 1));
                        }
                    }
                }
            }

            geoms_.push_back(std::move(g));
        } else if (obj.name == "Model") {
            // Models are only used via connections here; record nothing extra.
            (void)obj;
        } else if (obj.name == "Material") {
            MatInfo mi;
            if (!obj.props.empty() && obj.props[0].type == 'L')
                mi.uid = static_cast<uint64_t>(obj.props[0].l);
            std::string nm = obj.props.size() > 1 ? obj.props[1].str : std::string();
            stripPrefix(nm);
            if (nm.empty()) nm = "material" + std::to_string(mats_.size());
            names_.push_back(std::move(nm));
            mi.mat.name = std::string_view(names_.back().data(), names_.back().size());
            mi.mat.diffuse[0] = mi.mat.diffuse[1] = mi.mat.diffuse[2] = mi.mat.diffuse[3] = 1.f;
            mi.mat.specular[0] = mi.mat.specular[1] = mi.mat.specular[2] = 0.f;
            mi.mat.specular[3] = 1.f;
            mi.mat.opacity = 1.f;

            auto readColor = [&](std::string_view node, float* dst) {
                const Prop* cp = childArray(obj, node);
                if (cp && cp->arrCount >= 3) {
                    dst[0] = static_cast<float>(getDouble(*cp, 0));
                    dst[1] = static_cast<float>(getDouble(*cp, 1));
                    dst[2] = static_cast<float>(getDouble(*cp, 2));
                    return true;
                }
                return false;
            };
            readColor("DiffuseColor", mi.mat.diffuse);
            readColor("EmissiveColor", mi.mat.emissive);
            readColor("AmbientColor", mi.mat.ambient);
            readColor("SpecularColor", mi.mat.specular);
            const Prop* op = childArray(obj, "Opacity");
            if (op) mi.mat.opacity = static_cast<float>(getDouble(*op, 0));
            const Prop* sh = childArray(obj, "Shininess");
            if (sh) mi.mat.shininess = static_cast<float>(getDouble(*sh, 0));

            // Properties70 P entries (fallback / extra)
            const Node* p70 = findChild(obj, "Properties70");
            if (p70) {
                for (const auto& p : p70->children) {
                    if (p.name != "P" || p.props.size() < 5) continue;
                    std::string_view pname = p.props[0].str;
                    auto getP = [&](int idx) -> double {
                        // values are stored as further properties (S/D/I...)
                        for (size_t k = 4; k < p.props.size(); ++k) {
                            if (p.props[k].type == 'D')
                                return idx == 0 ? p.props[k].d : 0.0;
                            if (p.props[k].type == 'I' && idx == 0)
                                return static_cast<double>(p.props[k].i);
                        }
                        return 0.0;
                    };
                    if (pname == "DiffuseColor") {
                        mi.mat.diffuse[0] = static_cast<float>(getP(0));
                        mi.mat.diffuse[1] = static_cast<float>(getP(1));
                        mi.mat.diffuse[2] = static_cast<float>(getP(2));
                    } else if (pname == "Opacity") {
                        mi.mat.opacity = static_cast<float>(getP(0));
                    } else if (pname == "Shininess") {
                        mi.mat.shininess = static_cast<float>(getP(0));
                    }
                }
            }
            mats_.push_back(std::move(mi));
        } else if (obj.name == "Texture" || obj.name == "Video") {
            // (texture gathering happens below via texToMat_ connections)
            (void)obj;
        }
    }

    // ---- connections ----
    if (connections) {
        for (const auto& c : connections->children) {
            if (c.name != "C" || c.props.size() < 3) continue;
            std::string_view type = c.props[0].str;
            if (c.props[1].type != 'L' || c.props[2].type != 'L') continue;
            uint64_t child = static_cast<uint64_t>(c.props[1].l);
            uint64_t parent = static_cast<uint64_t>(c.props[2].l);
            std::string_view prop = c.props.size() > 3 ? c.props[3].str : std::string_view();
            if (type == "OO") {
                geomToModel_.emplace_back(child, parent);
                modelToMat_.emplace_back(child, parent);
                geomToMat_.emplace_back(child, parent);
            } else if (type == "OP") {
                texToMat_.emplace_back(child, parent, texTypeFromName(prop));
            }
        }
    }

    // ---- materials: assign indices; default white if none ----
    for (size_t i = 0; i < mats_.size(); ++i) mats_[i].index = static_cast<int>(i);
    if (mats_.empty()) {
        MatInfo def;
        names_.push_back("default");
        def.mat.name = std::string_view(names_.back().data(), names_.back().size());
        def.mat.diffuse[0] = def.mat.diffuse[1] = def.mat.diffuse[2] = def.mat.diffuse[3] = 1.f;
        def.index = 0;
        mats_.push_back(std::move(def));
    }
    for (const auto& mi : mats_) result_->materials.push_back(mi.mat);

    // ---- textures (optional) ----
    if (connections && wantTextureBytes_) {
        for (const auto& obj : objects->children) {
            if (obj.name != "Texture" && obj.name != "Video") continue;
            uint64_t tuid = (!obj.props.empty() && obj.props[0].type == 'L')
                                ? static_cast<uint64_t>(obj.props[0].l)
                                : 0;
            std::string path;
            const Node* rf = findChild(obj, "RelativeFilename");
            if (rf && !rf->props.empty()) path = rf->props[0].str;
            else {
                const Node* fn = findChild(obj, "Filename");
                if (fn && !fn->props.empty()) path = fn->props[0].str;
            }
            const Prop* content = childArray(obj, "Content"); // embedded raw
            PackTexture pt;
            pt.type = TexDiffuse;
            pt.slot = 0;
            if (!path.empty()) {
                names_.push_back(path);
                pt.path = std::string_view(names_.back().data(), names_.back().size());
                pt.resolvedPath = result_->sourceDir + "/" + path;
                if (wantTextureBytes_) {
                    auto mf = MappedFile::openShared(pt.resolvedPath);
                    if (mf) {
                        const auto b = mf->bytes();
                        result_->textureFiles.push_back(mf);
                        pt.embedded = false;
                        pt.data = b;
                        pt.byteSize = b.size();
                    } else {
                        addWarning("cannot open FBX texture file: " + path);
                    }
                }
            } else if (content && content->type == 'R' && !content->str.empty()) {
                std::vector<std::byte> blob;
                blob.reserve(content->str.size());
                for (char ch : content->str)
                    blob.push_back(static_cast<std::byte>(ch));
                result_->embeddedTextures.push_back(std::move(blob));
                pt.embedded = true;
                pt.data = std::span<const std::byte>(
                    result_->embeddedTextures.back().data(),
                    result_->embeddedTextures.back().size());
                pt.byteSize = pt.data.size();
            }
            if (pt.path.empty() && pt.data.empty()) continue;
            result_->textures.push_back(pt);
            // bind to materials via OP connections
            for (const auto& tm : texToMat_) {
                if (std::get<0>(tm) != tuid) continue;
                int mi = matIndexByUid(std::get<1>(tm));
                if (mi < 0) continue;
                result_->materials[mi].textures.push_back(
                    PackTexRef{std::get<2>(tm), 0, pt.path});
            }
        }
    }

    // ---- build geometry pools + meshes ----
    bool allNrm = !geoms_.empty();
    bool allUv = !geoms_.empty();
    for (const auto& g : geoms_) {
        if (g.normals.empty()) allNrm = false;
        if (g.texcoords.empty()) allUv = false;
    }
    bool useNrm = wantNormals_ && allNrm;
    bool useUv = wantTexcoords_ && allUv;

    result_->positions.reserve(geoms_.size() * 1024);
    result_->posIndices.reserve(geoms_.size() * 1024);
    if (useNrm) result_->normals.reserve(geoms_.size() * 1024);
    if (useUv) result_->texcoords.reserve(geoms_.size() * 1024);

    for (const auto& g : geoms_) {
        const size_t posBase = result_->positions.size();
        const size_t idxBase = result_->posIndices.size();
        const size_t nrmBase = result_->normals.size();
        const size_t uvBase = result_->texcoords.size();
        result_->positions.insert(result_->positions.end(), g.positions.begin(),
                                  g.positions.end());
        result_->posIndices.insert(result_->posIndices.end(), g.indices.begin(),
                                   g.indices.end());
        if (useNrm)
            result_->normals.insert(result_->normals.end(), g.normals.begin(),
                                    g.normals.end());
        if (useUv)
            result_->texcoords.insert(result_->texcoords.end(),
                                      g.texcoords.begin(), g.texcoords.end());
        buildMesh(g, posBase, idxBase, nrmBase, uvBase, useNrm, useUv);
    }

    if (result_->meshes.empty()) {
        lastError_ = "FBX produced no meshes";
        return false;
    }
    return true;
}

bool FbxParser::execute(std::string_view path) {
    result_->objFile = MappedFile::openShared(path);
    if (!result_->objFile) {
        lastError_ = std::string("cannot open file: ") + std::string(path);
        fireAllDone(*result_, false, lastError_);
        return false;
    }
    const auto slash = path.find_last_of("/\\");
    if (slash != std::string_view::npos)
        result_->sourceDir = std::string(path.substr(0, slash));

    const auto bytes = result_->objFile->bytes();
    const std::byte* base = bytes.data();
    const std::byte* fileEnd = base + bytes.size();

    // ---- header / magic ----
    if (bytes.size() < 27) {
        lastError_ = "FBX file too small";
        fireAllDone(*result_, false, lastError_);
        return false;
    }
    std::string_view head(reinterpret_cast<const char*>(base), 21);
    if (head.substr(0, 18) != "Kaydara FBX Binary") {
        addWarning("ASCII FBX is not supported by this parser");
        lastError_ = "not a binary FBX file";
        fireAllDone(*result_, false, lastError_);
        return false;
    }

    // Locate the version u32 and node-tree start (handles the 2- or 6-byte
    // padding variants seen across exporters).
    uint32_t version = 0;
    size_t rootStart = 27;
    for (size_t off = 21; off <= 27; off += 2) {
        uint32_t v = readU32(base + off);
        if (v >= 6000 && v <= 9000) {
            version = v;
            rootStart = off + 4;
            break;
        }
    }
    if (version == 0) {
        addWarning("could not locate FBX version; assuming 7400 layout");
        rootStart = 27;
    }

    if (!parseTree(base, fileEnd, rootStart)) {
        if (lastError_.empty()) lastError_ = "failed to parse FBX tree";
        fireAllDone(*result_, false, lastError_);
        return false;
    }

    fireProgress(100.f);
    fireVertices(*result_, std::span<const PackMesh>(result_->meshes.data(),
                                                     result_->meshes.size()));
    fireMaterials(*result_, std::span<const PackMaterial>(result_->materials.data(),
                                                          result_->materials.size()));
    fireTextures(*result_, std::span<const PackTexture>(result_->textures.data(),
                                                        result_->textures.size()));
    fireAllDone(*result_, true, {});
    return true;
}

bool FbxParser::load(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    geoms_.clear();
    mats_.clear();
    names_.clear();
    geomToModel_.clear();
    modelToMat_.clear();
    geomToMat_.clear();
    texToMat_.clear();
    return execute(path);
}

void FbxParser::loadAsync(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    geoms_.clear();
    mats_.clear();
    names_.clear();
    geomToModel_.clear();
    modelToMat_.clear();
    geomToMat_.clear();
    texToMat_.clear();
    // NOTE: the caller must keep this parser object alive until onAllDone
    // fires; the detached thread below operates on `this`.
    auto pathCopy = std::make_shared<std::string>(path);
    std::thread([this, pathCopy]() { execute(*pathCopy); }).detach();
}

void registerFbxParser() {
    static const bool once = [] {
        auto make = [](unsigned /*threads*/) -> std::unique_ptr<ModelParser> {
            return std::make_unique<FbxParser>(0);
        };
        ParserRegistry::instance().add("fbx", make);
        return true;
    }();
    (void)once;
}

} // namespace ap
