#include "assetpack/AssetPack.h"

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ap {

namespace {

// glTF binary container constants
constexpr uint32_t kGlbMagic   = 0x46546C67; // "glTF"
constexpr uint32_t kGlbJson    = 0x4E4F534A; // "JSON"
constexpr uint32_t kGlbBin     = 0x004E4942; // "BIN\0"

// glTF componentType values
constexpr int kCTByte     = 5120;
constexpr int kCTUByte    = 5121;
constexpr int kCTShort    = 5122;
constexpr int kCTUShort   = 5123;
constexpr int kCTUInt     = 5125;
constexpr int kCTFloat    = 5126;

uint32_t readU32LE(const std::byte* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

int compSize(int ct) {
    switch (ct) {
        case kCTFloat:
        case kCTUInt:   return 4;
        case kCTShort:
        case kCTUShort: return 2;
        case kCTByte:
        case kCTUByte:  return 1;
        default:        return 0;
    }
}

int attribComponents(std::string_view type) {
    if (type == "SCALAR") return 1;
    if (type == "VEC2")   return 2;
    if (type == "VEC3")   return 3;
    if (type == "VEC4")   return 4;
    if (type == "MAT2")   return 4;
    if (type == "MAT3")   return 9;
    if (type == "MAT4")   return 16;
    return 1;
}

// Decode base64 (RFC 4648) into `out`. Returns number of bytes written.
size_t base64Decode(std::string_view in, std::vector<std::byte>& out) {
    auto val = [](char c) -> int {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 'a' + 26;
        if (c >= '0' && c <= '9') return c - '0' + 52;
        if (c == '+') return 62;
        if (c == '/') return 63;
        return -1;
    };
    const size_t n = in.size();
    size_t i = 0;
    while (i < n) {
        // skip whitespace / padding
        int quartet[4] = { -1, -1, -1, -1 };
        int got = 0;
        while (i < n && got < 4) {
            char c = in[i++];
            if (c == '=' || c == '\n' || c == '\r' || c == '\t' || c == ' ')
                continue;
            quartet[got++] = val(c);
        }
        if (got == 0) break; // trailing whitespace only
        if (got < 2) break;  // too few symbols: stop
        int b0 = quartet[0], b1 = quartet[1];
        out.push_back(static_cast<std::byte>((b0 << 2) | (b1 >> 4)));
        if (got >= 3 && quartet[2] >= 0) {
            int b2 = quartet[2];
            out.push_back(static_cast<std::byte>((b1 << 4) | (b2 >> 2)));
            if (got >= 4 && quartet[3] >= 0) {
                int b3 = quartet[3];
                out.push_back(static_cast<std::byte>((b2 << 6) | b3));
            }
        }
    }
    return out.size();
}

bool readComp(const std::byte* p, int ct, float& out) {
    switch (ct) {
        case kCTFloat:  { float v; std::memcpy(&v, p, 4); out = v; return true; }
        case kCTUInt:   out = static_cast<float>(*reinterpret_cast<const uint32_t*>(p)); return true;
        case kCTUShort: out = static_cast<float>(*reinterpret_cast<const uint16_t*>(p)); return true;
        case kCTUByte:  out = static_cast<float>(*reinterpret_cast<const uint8_t*>(p));  return true;
        case kCTShort:  out = static_cast<float>(*reinterpret_cast<const int16_t*>(p));  return true;
        case kCTByte:   out = static_cast<float>(*reinterpret_cast<const int8_t*>(p));   return true;
        default:        return false;
    }
}

bool readIdx(const std::byte* p, int ct, uint32_t& out) {
    switch (ct) {
        case kCTUInt:   out = *reinterpret_cast<const uint32_t*>(p); return true;
        case kCTUShort: out = *reinterpret_cast<const uint16_t*>(p); return true;
        case kCTUByte:  out = *reinterpret_cast<const uint8_t*>(p);  return true;
        default:        return false;
    }
}

void computeBounds(const float* p, size_t n, float mn[3], float mx[3]) {
    if (n == 0) {
        mn[0] = mn[1] = mn[2] = 0.f;
        mx[0] = mx[1] = mx[2] = 0.f;
        return;
    }
    for (int c = 0; c < 3; ++c) { mn[c] = p[c]; mx[c] = p[c]; }
    for (size_t i = 1; i < n; ++i) {
        for (int c = 0; c < 3; ++c) {
            const float v = p[i * 3 + c];
            if (v < mn[c]) mn[c] = v;
            if (v > mx[c]) mx[c] = v;
        }
    }
}

} // namespace

class GltfParser final : public ModelParser {
public:
    explicit GltfParser(unsigned /*threads*/) {}

    bool load(std::string_view path) override;
    void loadAsync(std::string_view path) override;
    PackResult& result() override { return *result_; }

    void setWantNormals(bool want) override { wantNormals_ = want; }
    void setWantTexcoords(bool want) override { wantTexcoords_ = want; }
    void setWantTextureBytes(bool want) override { wantTextureBytes_ = want; }

private:
    struct RB {                 // resolved bufferView
        const std::byte* base = nullptr;
        size_t stride = 0;
        size_t length = 0;
    };

    struct PrimMeta {
        std::string name;
        int32_t materialIndex = -1;
        size_t posBase = 0, posCount = 0;
        size_t normBase = 0, normCount = 0;
        size_t texBase = 0, texCount = 0;
        size_t idxBase = 0, idxCount = 0;
    };

    bool execute(std::string_view path);
    bool parseJson(std::string_view json);
    bool buildBuffers(const rapidjson::Value& root, bool isGlb);
    bool buildBufferViews(const rapidjson::Value& root);
    bool geometryPass(const rapidjson::Value& root);
    bool materialPass(const rapidjson::Value& root);

    std::string_view strMember(const rapidjson::Value& v, const char* k) const {
        if (v.HasMember(k) && v[k].IsString())
            return std::string_view(v[k].GetString(), v[k].GetStringLength());
        return {};
    }
    int intMember(const rapidjson::Value& v, const char* k, int def) const {
        if (v.HasMember(k) && v[k].IsInt()) return v[k].GetInt();
        return def;
    }
    uint64_t uintMember(const rapidjson::Value& v, const char* k, uint64_t def) const {
        if (v.HasMember(k) && v[k].IsUint64()) return v[k].GetUint64();
        return def;
    }

    // resolve an accessor to (base pointer, stride, compType, numComp, count)
    bool resolveAccessor(const rapidjson::Value& acc, const std::byte*& base,
                         size_t& stride, int& compType, int& numComp,
                         size_t& count) const;

    rapidjson::Document doc_;
    std::vector<std::vector<std::byte>> blobs_;  // per-buffer bytes
    std::vector<RB> rbv_;                        // resolved bufferViews
    std::string jsonChunk_;                      // .glb JSON chunk (so doc_ strings stay valid)
    std::vector<std::byte> binChunk_;            // .glb BIN chunk

    bool wantNormals_ = true;
    bool wantTexcoords_ = true;
    bool wantTextureBytes_ = false;
};

bool GltfParser::resolveAccessor(const rapidjson::Value& acc, const std::byte*& base,
                                 size_t& stride, int& compType, int& numComp,
                                 size_t& count) const {
    if (!acc.IsObject()) return false;
    count = static_cast<size_t>(uintMember(acc, "count", 0));
    compType = intMember(acc, "componentType", kCTFloat);
    numComp = attribComponents(strMember(acc, "type"));
    if (acc.HasMember("bufferView") && acc["bufferView"].IsInt()) {
        const int bvIdx = acc["bufferView"].GetInt();
        if (bvIdx < 0 || bvIdx >= static_cast<int>(rbv_.size())) return false;
        const size_t accOff = static_cast<size_t>(uintMember(acc, "byteOffset", 0));
        base = rbv_[bvIdx].base + accOff;
        const size_t packed = static_cast<size_t>(numComp) * compSize(compType);
        stride = rbv_[bvIdx].stride ? rbv_[bvIdx].stride : packed;
    } else {
        base = nullptr; // sparse / empty accessor
    }
    return true;
}

bool GltfParser::buildBuffers(const rapidjson::Value& root, bool isGlb) {
    if (!root.HasMember("buffers") || !root["buffers"].IsArray()) return true;
    const rapidjson::Value& bufs = root["buffers"];
    blobs_.clear();
    blobs_.reserve(bufs.Size());
    for (rapidjson::SizeType i = 0; i < bufs.Size(); ++i) {
        const rapidjson::Value& b = bufs[i];
        std::vector<std::byte> blob;
        std::string_view uri = strMember(b, "uri");
        if (uri.empty()) {
            // no uri -> .glb BIN chunk (or error for .gltf)
            if (isGlb) {
                blob = binChunk_;
            } else {
                addWarning("buffer without uri in .gltf (skipped)");
            }
        } else if (uri.starts_with("data:")) {
            const size_t comma = uri.find(',');
            if (comma == std::string_view::npos ||
                uri.substr(comma + 1).find("base64") == std::string_view::npos) {
                addWarning("unsupported non-base64 data URI buffer (skipped)");
            } else {
                base64Decode(uri.substr(comma + 1), blob);
            }
        } else {
            // external file relative to sourceDir
            std::string path = result_->sourceDir + "/" + std::string(uri);
            auto mf = MappedFile::openShared(path);
            if (!mf) {
                addWarning("cannot open buffer file: " + path);
            } else {
                const auto bytes = mf->bytes();
                blob.assign(bytes.begin(), bytes.end());
            }
        }
        blobs_.push_back(std::move(blob));
    }
    return true;
}

bool GltfParser::buildBufferViews(const rapidjson::Value& root) {
    if (!root.HasMember("bufferViews") || !root["bufferViews"].IsArray()) return true;
    const rapidjson::Value& bvs = root["bufferViews"];
    rbv_.clear();
    rbv_.reserve(bvs.Size());
    for (rapidjson::SizeType i = 0; i < bvs.Size(); ++i) {
        const rapidjson::Value& bv = bvs[i];
        RB r;
        const int bufIdx = intMember(bv, "buffer", -1);
        if (bufIdx >= 0 && bufIdx < static_cast<int>(blobs_.size())) {
            const size_t bvOff = static_cast<size_t>(uintMember(bv, "byteOffset", 0));
            if (!blobs_[bufIdx].empty())
                r.base = blobs_[bufIdx].data() + bvOff;
        }
        r.stride = static_cast<size_t>(uintMember(bv, "byteStride", 0));
        r.length = static_cast<size_t>(uintMember(bv, "byteLength", 0));
        rbv_.push_back(r);
    }
    return true;
}

bool GltfParser::geometryPass(const rapidjson::Value& root) {
    if (!root.HasMember("meshes") || !root["meshes"].IsArray()) {
        lastError_ = "glTF has no meshes";
        return false;
    }
    const rapidjson::Value& meshes = root["meshes"];

    std::vector<PrimMeta> metas;
    metas.reserve(16);

    // reserve pools roughly
    result_->positions.reserve(1024 * 3);

    for (rapidjson::SizeType mi = 0; mi < meshes.Size(); ++mi) {
        if (isCancelled()) { lastError_ = "cancelled"; return false; }
        const rapidjson::Value& mesh = meshes[mi];
        std::string meshName = std::string(strMember(mesh, "name"));
        if (meshName.empty()) meshName = "mesh" + std::to_string(mi);

        if (!mesh.HasMember("primitives") || !mesh["primitives"].IsArray()) continue;
        const rapidjson::Value& prims = mesh["primitives"];
        for (rapidjson::SizeType pi = 0; pi < prims.Size(); ++pi) {
            if (isCancelled()) { continue; }
            const rapidjson::Value& prim = prims[pi];
            if (!prim.IsObject()) continue;

            PrimMeta pm;
            pm.name = (prims.Size() > 1)
                          ? (meshName + "#" + std::to_string(pi))
                          : meshName;
            pm.materialIndex = prim.HasMember("material") && prim["material"].IsInt()
                                  ? prim["material"].GetInt()
                                  : -1;

            // ----- POSITION (required) -----
            const rapidjson::Value& attrs = prim["attributes"];
            if (!attrs.HasMember("POSITION") || !attrs["POSITION"].IsInt()) {
                addWarning("primitive without POSITION (skipped)");
                continue;
            }
            const rapidjson::Value& posAcc =
                root["accessors"][attrs["POSITION"].GetInt()];
            const std::byte* base = nullptr;
            size_t stride = 0, count = 0;
            int ct = 0, nc = 0;
            if (!resolveAccessor(posAcc, base, stride, ct, nc, count) ||
                base == nullptr || nc != 3) {
                lastError_ = "cannot resolve POSITION accessor";
                return false;
            }
            pm.posBase = result_->positions.size() / 3;
            pm.posCount = count;
            const size_t before = result_->positions.size();
            for (size_t v = 0; v < count; ++v) {
                const std::byte* e = base + v * stride;
                float x, y, z;
                if (!readComp(e, ct, x) || !readComp(e + compSize(ct), ct, y) ||
                    !readComp(e + 2 * compSize(ct), ct, z)) {
                    lastError_ = "bad POSITION data";
                    return false;
                }
                result_->positions.push_back(x);
                result_->positions.push_back(y);
                result_->positions.push_back(z);
            }
            (void)before;

            // ----- NORMAL (optional) -----
            if (wantNormals_ && attrs.HasMember("NORMAL") &&
                attrs["NORMAL"].IsInt()) {
                const rapidjson::Value& nAcc =
                    root["accessors"][attrs["NORMAL"].GetInt()];
                const std::byte* nbase = nullptr;
                size_t nstride = 0, ncount = 0;
                int nct = 0, nnc = 0;
                if (resolveAccessor(nAcc, nbase, nstride, nct, nnc, ncount) &&
                    nbase != nullptr && nnc == 3 && ncount == count) {
                    pm.normBase = result_->normals.size() / 3;
                    pm.normCount = ncount;
                    for (size_t v = 0; v < ncount; ++v) {
                        const std::byte* e = nbase + v * nstride;
                        float x, y, z;
                        readComp(e, nct, x);
                        readComp(e + compSize(nct), nct, y);
                        readComp(e + 2 * compSize(nct), nct, z);
                        result_->normals.push_back(x);
                        result_->normals.push_back(y);
                        result_->normals.push_back(z);
                    }
                } else {
                    addWarning("NORMAL accessor mismatch (skipped normals)");
                }
            }

            // ----- TEXCOORD_0 (optional) -----
            if (wantTexcoords_ && attrs.HasMember("TEXCOORD_0") &&
                attrs["TEXCOORD_0"].IsInt()) {
                const rapidjson::Value& tAcc =
                    root["accessors"][attrs["TEXCOORD_0"].GetInt()];
                const std::byte* tbase = nullptr;
                size_t tstride = 0, tcount = 0;
                int tct = 0, tnc = 0;
                if (resolveAccessor(tAcc, tbase, tstride, tct, tnc, tcount) &&
                    tbase != nullptr && tnc == 2 && tcount == count) {
                    pm.texBase = result_->texcoords.size() / 2;
                    pm.texCount = tcount;
                    for (size_t v = 0; v < tcount; ++v) {
                        const std::byte* e = tbase + v * tstride;
                        float u, w;
                        readComp(e, tct, u);
                        readComp(e + compSize(tct), tct, w);
                        result_->texcoords.push_back(u);
                        result_->texcoords.push_back(w);
                    }
                } else {
                    addWarning("TEXCOORD_0 accessor mismatch (skipped texcoords)");
                }
            }

            // ----- indices (optional) -----
            pm.idxBase = result_->posIndices.size();
            if (prim.HasMember("indices") && prim["indices"].IsInt()) {
                const rapidjson::Value& iAcc =
                    root["accessors"][prim["indices"].GetInt()];
                const std::byte* ibase = nullptr;
                size_t istride = 0, icount = 0;
                int ict = 0, inc = 0;
                if (resolveAccessor(iAcc, ibase, istride, ict, inc, icount) &&
                    ibase != nullptr && inc == 1) {
                    pm.idxCount = icount;
                    for (size_t v = 0; v < icount; ++v) {
                        const std::byte* e = ibase + v * istride;
                        uint32_t idx = 0;
                        if (!readIdx(e, ict, idx)) { lastError_ = "bad index data"; return false; }
                        result_->posIndices.push_back(idx);
                    }
                } else {
                    lastError_ = "cannot resolve indices accessor";
                    return false;
                }
            } else {
                // generate sequential indices
                pm.idxCount = count;
                for (size_t v = 0; v < count; ++v)
                    result_->posIndices.push_back(static_cast<uint32_t>(v));
            }

            metas.push_back(pm);
        }
    }

    // build PackMesh views now that pools are fully populated
    for (const PrimMeta& pm : metas) {
        PackMesh m;
        m.name = pm.name;
        m.materialIndex = pm.materialIndex;
        m.positions = std::span<const float>(
            result_->positions.data() + pm.posBase * 3, pm.posCount * 3);
        m.normals = std::span<const float>(
            result_->normals.data() + pm.normBase * 3, pm.normCount * 3);
        m.texcoords = std::span<const float>(
            result_->texcoords.data() + pm.texBase * 2, pm.texCount * 2);
        m.indices = std::span<const uint32_t>(
            result_->posIndices.data() + pm.idxBase, pm.idxCount);
        computeBounds(m.positions.data(), pm.posCount, m.boundsMin, m.boundsMax);
        result_->meshes.push_back(m);
    }

    if (result_->meshes.empty()) {
        lastError_ = "glTF produced no meshes";
        return false;
    }
    return true;
}

bool GltfParser::materialPass(const rapidjson::Value& root) {
    if (!root.HasMember("materials") || !root["materials"].IsArray()) return true;
    const rapidjson::Value& mats = root["materials"];
    const rapidjson::Value* textures = root.HasMember("textures") && root["textures"].IsArray()
                                           ? &root["textures"]
                                           : nullptr;
    const rapidjson::Value* images = root.HasMember("images") && root["images"].IsArray()
                                         ? &root["images"]
                                         : nullptr;

    // imageIndex -> texture index (dedupe embedded/external textures)
    std::vector<int> imageToTex;
    if (images) imageToTex.assign(images->Size(), -1);

    auto resolveImage = [&](int imageIndex, const std::byte*& dataOut,
                            size_t& sizeOut, std::string& resolvedPathOut,
                            std::string_view& uriOut) -> bool {
        if (!images || imageIndex < 0 ||
            imageIndex >= static_cast<int>(images->Size()))
            return false;
        const rapidjson::Value& img = (*images)[imageIndex];
        uriOut = strMember(img, "uri");
        if (uriOut.starts_with("data:")) {
            const size_t comma = uriOut.find(',');
            if (comma == std::string_view::npos) return false;
            std::vector<std::byte> dec;
            base64Decode(uriOut.substr(comma + 1), dec);
            if (dec.empty()) return false;
            result_->embeddedTextures.push_back(std::move(dec));
            const auto& back = result_->embeddedTextures.back();
            dataOut = back.data();
            sizeOut = back.size();
            return true;
        }
        // bufferView embedded image
        if (img.HasMember("bufferView") && img["bufferView"].IsInt()) {
            const int bvIdx = img["bufferView"].GetInt();
            if (bvIdx < 0 || bvIdx >= static_cast<int>(rbv_.size()) ||
                rbv_[bvIdx].base == nullptr)
                return false;
            const size_t len = rbv_[bvIdx].length;
            result_->embeddedTextures.emplace_back(
                rbv_[bvIdx].base, rbv_[bvIdx].base + len);
            const auto& back = result_->embeddedTextures.back();
            dataOut = back.data();
            sizeOut = back.size();
            return true;
        }
        // external file
        if (!uriOut.empty()) {
            resolvedPathOut = result_->sourceDir + "/" + std::string(uriOut);
            if (wantTextureBytes_) {
                auto mf = MappedFile::openShared(resolvedPathOut);
                if (mf) {
                    const auto bytes = mf->bytes();
                    result_->textureFiles.push_back(mf);
                    dataOut = bytes.data();
                    sizeOut = bytes.size();
                    return true;
                }
                addWarning("cannot open texture file: " + resolvedPathOut);
            }
            // still record resolved path even without bytes
            return true;
        }
        return false;
    };

    for (rapidjson::SizeType mi = 0; mi < mats.Size(); ++mi) {
        if (isCancelled()) { lastError_ = "cancelled"; return false; }
        const rapidjson::Value& m = mats[mi];
        PackMaterial pm;
        std::string name = std::string(strMember(m, "name"));
        if (name.empty()) name = "material" + std::to_string(mi);
        // name view into doc_ pool (stable while parser alive)
        pm.name = std::string_view(name.data(), name.size());

        pm.diffuse[0] = pm.diffuse[1] = pm.diffuse[2] = pm.diffuse[3] = 1.f;
        pm.opacity = 1.f;
        pm.twoSided = (m.HasMember("doubleSided") && m["doubleSided"].IsBool())
                          ? m["doubleSided"].GetBool()
                          : false;

        if (m.HasMember("emissiveFactor") && m["emissiveFactor"].IsArray()) {
            const auto& e = m["emissiveFactor"];
            for (int c = 0; c < 3 && c < static_cast<int>(e.Size()); ++c)
                pm.emissive[c] = static_cast<float>(e[c].GetDouble());
        }

        const rapidjson::Value* pbr =
            (m.HasMember("pbrMetallicRoughness") &&
             m["pbrMetallicRoughness"].IsObject())
                ? &m["pbrMetallicRoughness"]
                : nullptr;
        if (pbr && pbr->HasMember("baseColorFactor") &&
            (*pbr)["baseColorFactor"].IsArray()) {
            const auto& f = (*pbr)["baseColorFactor"];
            for (int c = 0; c < 4 && c < static_cast<int>(f.Size()); ++c)
                pm.diffuse[c] = static_cast<float>(f[c].GetDouble());
            pm.opacity = pm.diffuse[3];
        }

        // baseColorTexture -> diffuse texture
        if (pbr && pbr->HasMember("baseColorTexture") &&
            (*pbr)["baseColorTexture"].IsObject()) {
            const rapidjson::Value& bt = (*pbr)["baseColorTexture"];
            if (bt.HasMember("index") && bt["index"].IsInt() && textures) {
                const int texIndex = bt["index"].GetInt();
                if (texIndex >= 0 &&
                    texIndex < static_cast<int>(textures->Size())) {
                    const rapidjson::Value& tex = (*textures)[texIndex];
                    int imageIndex = tex.HasMember("source") && tex["source"].IsInt()
                                         ? tex["source"].GetInt()
                                         : -1;
                    if (imageIndex >= 0) {
                        // create/update PackTexture for this image
                        int texSlot = imageToTex[imageIndex];
                        if (texSlot < 0) {
                            PackTexture pt;
                            const std::byte* data = nullptr;
                            size_t size = 0;
                            std::string resolved;
                            std::string_view uri;
                            if (resolveImage(imageIndex, data, size, resolved, uri)) {
                                pt.path = uri;
                                pt.type = ap::TexDiffuse;
                                pt.slot = 0;
                                if (!resolved.empty()) pt.resolvedPath = resolved;
                                if (data != nullptr) {
                                    pt.embedded = true;
                                    pt.data = std::span<const std::byte>(data, size);
                                    pt.byteSize = size;
                                }
                                result_->textures.push_back(pt);
                                texSlot = static_cast<int>(result_->textures.size()) - 1;
                                imageToTex[imageIndex] = texSlot;
                            } else {
                                addWarning("could not resolve base color image");
                            }
                        }
                        if (texSlot >= 0) {
                            const auto& ref = result_->textures[texSlot];
                            pm.textures.push_back(
                                PackTexRef{ap::TexDiffuse, 0, ref.path});
                        }
                    }
                }
            }
        }

        result_->materials.push_back(pm);
    }
    return true;
}

bool GltfParser::parseJson(std::string_view json) {
    doc_.Parse<rapidjson::kParseDefaultFlags>(json.data(), json.size());
    if (doc_.HasParseError()) {
        lastError_ = std::string("glTF JSON parse error: ") +
                     rapidjson::GetParseError_En(doc_.GetParseError());
        return false;
    }
    if (!doc_.IsObject()) {
        lastError_ = "glTF JSON root is not an object";
        return false;
    }
    const rapidjson::Value& root = doc_;
    const bool isGlb = !binChunk_.empty() || json.data() == jsonChunk_.data();

    if (!buildBuffers(root, isGlb)) return false;
    if (!buildBufferViews(root)) return false;
    if (!geometryPass(root)) return false;
    if (!materialPass(root)) return false;
    return true;
}

bool GltfParser::execute(std::string_view path) {
    result_->objFile = MappedFile::openShared(path);
    if (!result_->objFile) {
        lastError_ = "cannot open file: " + std::string(path);
        fireAllDone(*result_, false, lastError_);
        return false;
    }
    const auto slash = path.find_last_of("/\\");
    if (slash != std::string_view::npos)
        result_->sourceDir = std::string(path.substr(0, slash));

    const auto bytes = result_->objFile->bytes();
    const bool isGlb = path.size() >= 4 &&
                       (path.substr(path.size() - 4) == ".glb" ||
                        (path.substr(path.size() - 4) == ".GLB"));

    if (isGlb) {
        if (bytes.size() < 12) {
            lastError_ = "glb too small";
            fireAllDone(*result_, false, lastError_);
            return false;
        }
        if (readU32LE(bytes.data()) != kGlbMagic) {
            lastError_ = "glb bad magic";
            fireAllDone(*result_, false, lastError_);
            return false;
        }
        size_t off = 12;
        const size_t total = readU32LE(bytes.data() + 8);
        (void)total;
        bool gotJson = false;
        while (off + 8 <= bytes.size()) {
            const size_t len = readU32LE(bytes.data() + off);
            const uint32_t type = readU32LE(bytes.data() + off + 4);
            const size_t dataStart = off + 8;
            if (dataStart + len > bytes.size()) break;
            if (type == kGlbJson) {
                jsonChunk_.assign(reinterpret_cast<const char*>(bytes.data() + dataStart),
                                  len);
                gotJson = true;
            } else if (type == kGlbBin) {
                binChunk_.assign(bytes.data() + dataStart,
                                 bytes.data() + dataStart + len);
            }
            off = dataStart + len;
            if (len % 4 != 0) off += 4 - (len % 4); // chunks padded to 4
        }
        if (!gotJson) {
            lastError_ = "glb missing JSON chunk";
            fireAllDone(*result_, false, lastError_);
            return false;
        }
        if (!parseJson(jsonChunk_)) {
            fireAllDone(*result_, false, lastError_);
            return false;
        }
    } else {
        std::string_view text = result_->objFile->text();
        if (!parseJson(text)) {
            fireAllDone(*result_, false, lastError_);
            return false;
        }
    }

    fireProgress(100.f);
    fireVertices(*result_, result_->meshes);
    fireMaterials(*result_, result_->materials);
    fireTextures(*result_, result_->textures);
    fireAllDone(*result_, true, {});
    return true;
}

bool GltfParser::load(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    jsonChunk_.clear();
    binChunk_.clear();
    blobs_.clear();
    rbv_.clear();
    doc_ = rapidjson::Document();
    return execute(path);
}

void GltfParser::loadAsync(std::string_view path) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();
    warnings_.clear();
    cancel_.store(false, std::memory_order_relaxed);
    jsonChunk_.clear();
    binChunk_.clear();
    blobs_.clear();
    rbv_.clear();
    doc_ = rapidjson::Document();
    // NOTE: the caller must keep this parser object alive until onAllDone
    // fires, because the detached thread below operates on `this`.
    auto pathCopy = std::make_shared<std::string>(path);
    std::thread([this, pathCopy]() { execute(*pathCopy); }).detach();
}

void registerGltfParser() {
    static const bool once = [] {
        auto make = [](unsigned /*threads*/) -> std::unique_ptr<ModelParser> {
            return std::make_unique<GltfParser>(0);
        };
        ParserRegistry::instance().add("gltf", make);
        ParserRegistry::instance().add("glb", make);
        return true;
    }();
    (void)once;
}

} // namespace ap
