#include "assetpack/AssetPack.h"
#include "obj/ObjParser.h"   // registerObjParser

#include <cctype>
#include <unordered_map>

namespace ap {

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

// ---- AssetPack facade ----

AssetPack::AssetPack(unsigned threads)
    : threads_(threads) {
    registerObjParser();   // keeps the unit linked in static builds
}

AssetPack::~AssetPack() = default;

void AssetPack::setFormat(std::string_view fmt) { format_ = std::string(fmt); }

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
    parser_->setWantNormals(wantNormals_);
}

bool AssetPack::load(std::string_view path) {
    if (!ensureParser(path)) return false;
    attachCallbacks();
    return parser_->load(path);
}

void AssetPack::loadAsync(std::string_view path) {
    if (!ensureParser(path)) {
        static PackResult empty;
        AP_LOG_WARN("pack", "loadAsync: no parser for '%.*s'",
                    int(path.size()), path.data());
        if (onAll_) onAll_(empty, false, "no parser for format");
        return;
    }
    attachCallbacks();
    parser_->loadAsync(path);
}

PackResult& AssetPack::result() { return parser_->result(); }

} // namespace ap
