#include "assetpack/AssetPack.h"
#include "formats/obj/ObjParser.h"   // registerObjParser

#include <cctype>
#include <unordered_map>

namespace ap {

void registerStlParser();   // defined in src/StlParser.cpp
void registerPlyParser();   // defined in src/PlyParser.cpp
void registerGltfParser();  // defined in src/GltfParser.cpp
void registerFbxParser();   // defined in src/FbxParser.cpp
void registerPbrtParser();  // defined in src/PbrtParser.cpp

// ---- ParserRegistry ----

struct ParserRegistry::Impl {
    std::unordered_map<std::string, Creator> byFmt;
};

ParserRegistry& ParserRegistry::instance() {
    static ParserRegistry registry;
    return registry;
}

ParserRegistry::ParserRegistry() : impl_(std::make_unique<Impl>()) {}
ParserRegistry::~ParserRegistry() = default;

// lowercase extension without the dot; empty when `path` has none
static std::string pathFormat(std::string_view path) {
    const size_t dot = path.find_last_of('.');
    const size_t slash = path.find_last_of("/\\");
    if (dot == std::string_view::npos
        || (slash != std::string_view::npos && dot < slash))
        return {};
    std::string fmt;
    fmt.reserve(path.size() - dot - 1);
    for (char c : path.substr(dot + 1))
        fmt.push_back(char(std::tolower(unsigned char(c))));
    return fmt;
}

void ParserRegistry::add(std::string_view fmt, Creator creator) {
    std::string key;
    key.reserve(fmt.size());
    for (char c : fmt)
        key.push_back(char(std::tolower(unsigned char(c))));
    impl_->byFmt[std::move(key)] = creator;
}

bool ParserRegistry::has(std::string_view fmt) const {
    std::string key;
    key.reserve(fmt.size());
    for (char c : fmt)
        key.push_back(char(std::tolower(unsigned char(c))));
    return impl_->byFmt.count(key) != 0;
}

std::unique_ptr<ModelParser> ParserRegistry::create(
    std::string_view fmt, unsigned threads) const {
    std::string key;
    key.reserve(fmt.size());
    for (char c : fmt)
        key.push_back(char(std::tolower(unsigned char(c))));
    const auto it = impl_->byFmt.find(key);
    return it == impl_->byFmt.end() ? nullptr : it->second(threads);
}

std::unique_ptr<ModelParser> ParserRegistry::createForPath(
    std::string_view path, unsigned threads) const {
    return create(pathFormat(path), threads);
}

// ---- data contract + ModelParser method definitions ----
// Declared in the umbrella header; defined here so the header stays
// declaration-only (consumers link one out-of-line copy, and the ABI
// no longer depends on their definition).

const char* texTypeName(int t) {
    switch (t) {
    case TexDiffuse:  return "diffuse";
    case TexAmbient:  return "ambient";
    case TexSpecular: return "specular";
    case TexEmissive: return "emissive";
    case TexOpacity:  return "opacity";
    case TexNormal:   return "normal";
    default:          return "?";
    }
}

uint32_t PackMesh::vertexCount()  const { return uint32_t(positions.size() / 3); }
uint32_t PackMesh::triangleCount() const { return uint32_t(indices.size() / 3); }

const std::string& ModelParser::lastError() const { return lastError_; }

void ModelParser::setOnVerticesReady(VerticesReady cb)   { onVerts_ = std::move(cb); }
void ModelParser::setOnMaterialsReady(MaterialsReady cb) { onMats_  = std::move(cb); }
void ModelParser::setOnTexturesReady(TexturesReady cb)   { onTexs_  = std::move(cb); }
void ModelParser::setOnAllDone(AllDone cb)               { onAll_   = std::move(cb); }
void ModelParser::setProgress(Progress cb)               { onProgress_ = std::move(cb); }

void ModelParser::cancel() {
    cancel_.store(true, std::memory_order_relaxed);
}
bool ModelParser::isCancelled() const {
    return cancel_.load(std::memory_order_relaxed);
}
const std::vector<std::string>& ModelParser::warnings() const {
    return warnings_;
}
void ModelParser::addWarning(std::string msg) {
    std::lock_guard<std::mutex> lock(warnMx_);
    warnings_.push_back(std::move(msg));
}

void ModelParser::fireVertices(PackResult& r, std::span<const PackMesh> m) const {
    if (onVerts_) onVerts_(r, m);
}
void ModelParser::fireMaterials(PackResult& r, std::span<const PackMaterial> m) const {
    if (onMats_) onMats_(r, m);
}
void ModelParser::fireTextures(PackResult& r, std::span<const PackTexture> t) const {
    if (onTexs_) onTexs_(r, t);
}
void ModelParser::fireAllDone(PackResult& r, bool ok, std::string_view err) const {
    if (onAll_) onAll_(r, ok, err);
}
void ModelParser::fireProgress(float pct) const {
    if (onProgress_) onProgress_(pct);
}

// Pools are shared state: clearing them here covers every parser. The
// consumer must have copied what it needs (e.g. uploaded to the GPU)
// and must not hold PackMesh pool views - only mesh metadata survives.
void ModelParser::releaseGeometry() {
    if (!result_) return;
    result_->positions.clear();
    result_->positions.shrink_to_fit();
    result_->normals.clear();
    result_->normals.shrink_to_fit();
    result_->texcoords.clear();
    result_->texcoords.shrink_to_fit();
    result_->posIndices.clear();
    result_->posIndices.shrink_to_fit();
}

// ---- AssetPack facade ----

AssetPack::AssetPack(unsigned threads)
    : threads_(threads) {
    failResult_ = std::make_shared<PackResult>();
    registerObjParser();    // keeps the unit linked in static builds
    registerStlParser();    // additional format parsers (stl / ply)
    registerPlyParser();
    registerGltfParser();   // glTF 2.0 (.gltf / .glb)
    registerFbxParser();    // binary FBX 7400+
    registerPbrtParser();   // PBRT-v4 scene (.pbrt)
}

AssetPack::~AssetPack() = default;

void AssetPack::setFormat(std::string_view fmt) { format_ = std::string(fmt); }

void AssetPack::setOnVerticesReady(VerticesReady cb)   { onVerts_ = std::move(cb); }
void AssetPack::setOnMaterialsReady(MaterialsReady cb) { onMats_  = std::move(cb); }
void AssetPack::setOnTexturesReady(TexturesReady cb)   { onTexs_  = std::move(cb); }
void AssetPack::setOnAllDone(AllDone cb)               { onAll_   = std::move(cb); }
void AssetPack::setProgress(Progress cb)               { onProgress_ = std::move(cb); }
void AssetPack::setGeoStream(GeoStreamSink sink)       { geoStream_ = std::move(sink); }
void AssetPack::setWantNormals(bool want)              { wantNormals_ = want; }
void AssetPack::setWantTexcoords(bool want)           { wantTexcoords_ = want; }
void AssetPack::setWantPositions(bool want)           { wantPositions_ = want; }
void AssetPack::setWantTextureBytes(bool want)        { wantTextureBytes_ = want; }

void AssetPack::cancel() { if (parser_) parser_->cancel(); }
const std::vector<std::string>& AssetPack::warnings() {
    if (!parser_) { static const std::vector<std::string> empty; return empty; }
    return parser_->warnings();
}

bool AssetPack::ensureParser(std::string_view path) {
    if (parser_) return true;
    const auto& registry = ParserRegistry::instance();
    std::string fmt = format_.empty() ? pathFormat(path) : format_;
    parser_ = fmt.empty() ? nullptr : registry.create(fmt, threads_);
    if (!parser_ && !fmt.empty())
        AP_LOG_WARN("pack", "no parser registered for '%s'", fmt.c_str());
    return parser_ != nullptr;
}

// hand the buffered facade callbacks to the freshly created parser
// (copies, so a second load still carries them)
void AssetPack::attachCallbacks() {
    parser_->setOnVerticesReady(onVerts_);
    parser_->setOnMaterialsReady(onMats_);
    parser_->setOnTexturesReady(onTexs_);
    parser_->setOnAllDone(onAll_);
    parser_->setProgress(onProgress_);
    parser_->setGeoStream(geoStream_);
    parser_->setWantNormals(wantNormals_);
    parser_->setWantTexcoords(wantTexcoords_);
    parser_->setWantPositions(wantPositions_);
    parser_->setWantTextureBytes(wantTextureBytes_);
}

bool AssetPack::load(std::string_view path) {
    if (!ensureParser(path)) return false;
    attachCallbacks();
    return parser_->load(path);
}

void AssetPack::loadAsync(std::string_view path) {
    if (!ensureParser(path)) {
        AP_LOG_WARN("pack", "loadAsync: no parser for '%.*s'",
                    int(path.size()), path.data());
        if (onAll_) onAll_(*failResult_, false, "no parser for format");
        return;
    }
    attachCallbacks();
    parser_->loadAsync(path);
}

PackResult& AssetPack::result() {
    if (!parser_) {
        AP_LOG_WARN("pack", "result() with no active parser (load failed?)");
        return *failResult_;
    }
    return parser_->result();
}

void AssetPack::releaseGeometry() {
    if (parser_) parser_->releaseGeometry();
}

} // namespace ap
