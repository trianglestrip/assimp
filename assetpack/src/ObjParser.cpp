#include "obj/ObjParser.h"
#include "assetpack/AssetPack.h"

#include "detail/TextScan.h"
#include "obj/Geometry.h"
#include "obj/Mtl.h"
#include "obj/Scan.h"

#include <memory>
#include <thread>

#include <taskflow/taskflow.hpp>

namespace ap {

void registerObjParser() {
    static const bool once = [] {
        ParserRegistry::instance().add(
            "obj", [](unsigned threads) -> std::unique_ptr<ModelParser> {
                return std::make_unique<ObjParser>(threads);
            });
        return true;
    }();
    (void)once;
}

struct ObjParser::FlowDeleter { void operator()(tf::Taskflow* f) const { delete f; } };

ObjParser::ObjParser(unsigned threads)
    : executor_(std::make_unique<tf::Executor>(
          threads == 0 ? std::thread::hardware_concurrency() : threads)) {}

ObjParser::~ObjParser() {
    // draining the executor joins any in-flight async load
    const auto t0 = Clock::now();
    executor_.reset();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - t0).count();
    if (ms > 5)
        AP_LOG("obj", "async executor drained in %lld ms",
               static_cast<long long>(ms));
}

PackResult& ObjParser::result() { return *result_; }

bool ObjParser::load(std::string_view path) { return runGraph(path, false); }
void ObjParser::loadAsync(std::string_view path) { runGraph(path, true); }

bool ObjParser::runGraph(std::string_view path, bool async) {
    result_ = std::make_shared<PackResult>();
    lastError_.clear();

    auto flow = std::shared_ptr<void>(new tf::Taskflow, FlowDeleter{});
    flow_ = flow; // keep alive for async completion
    auto& tf = *static_cast<tf::Taskflow*>(flow.get());

    const auto t0 = Clock::now();
    auto pathStr = std::make_shared<std::string>(path);
    auto rawMats = std::make_shared<std::vector<obj::RawMaterial>>();
    auto mtlNames = std::make_shared<std::vector<std::string_view>>();

    // ---- stage 1: mmap obj + quick mtllib head-scan ----
    tf::Task mapTask = tf.emplace([this, pathStr]() {
        const auto tStart = Clock::now();
        PackResult& result = *result_;
        result.objFile = MappedFile::openShared(*pathStr);
        if (!result.objFile) {
            lastError_ = "cannot open file: " + *pathStr;
            AP_LOG_WARN("obj", "%s", lastError_.c_str());
            return;
        }
        const auto slash = pathStr->find_last_of("/\\");
        if (slash != std::string::npos)
            result.sourceDir = pathStr->substr(0, slash);

        // mtllib is conventionally near the top; scan the head only
        const auto& bytes = result.objFile->text();
        const size_t head = std::min<size_t>(bytes.size(), size_t(4) << 20);
        size_t off = 0;
        while (off < head) {
            size_t eol = bytes.find('\n', off);
            if (eol == std::string_view::npos) eol = head;
            const std::string_view line = bytes.substr(off, eol - off);
            if (!line.empty() && obj::classify(line.data(), line.data() + line.size())
                    == obj::LineKind::Mtllib) {
                const char* tok; size_t len;
                if (nextToken(line.data() + 7, line.data() + line.size(), tok, len)) {
                    std::string mtlPath(tok, len);
                    std::string full = result.sourceDir.empty()
                        ? mtlPath : result.sourceDir + "/" + mtlPath;
                    result.mtlFile = MappedFile::openShared(full);
                    if (!result.mtlFile)
                        AP_LOG_WARN("obj", "mtllib '%s' not found", mtlPath.c_str());
                    else
                        AP_LOG("obj", "mtl mapped: %s (%zu bytes)",
                               mtlPath.c_str(), result.mtlFile->size());
                    break;
                }
            }
            off = eol + 1;
        }
        fireProgress(5.f);
        AP_LOG("obj", "obj mapped: %zu bytes in %llu us",
               result.objFile->size(),
               static_cast<unsigned long long>(microsSince(tStart)));
    });

    // ---- stage 2a: geometry (chunk-parallel two-pass + seam split) ----
    auto geo = std::make_shared<obj::GeometryStage>(*this, result_, mtlNames,
                                                    wantNormals_);
    tf::Task geometryTask =
        tf.emplace([geo](tf::Subflow& sf) { geo->run(sf); });

    // ---- stage 2b: materials (parse + convert, parallel to geometry) ----
    tf::Task materialsTask = tf.emplace([this, rawMats]() {
        obj::parseMtl(result_, rawMats);
    });

    // ---- stage 2c: textures (from materials) ----
    tf::Task texturesTask = tf.emplace([this, rawMats]() {
        obj::buildTextureRefs(*this, result_, rawMats);
    });

    // ---- stage 2d: bind usemtl -> material index, then fire materials ----
    tf::Task bindTask = tf.emplace([this, mtlNames]() {
        obj::bindMaterials(*this, result_, mtlNames);
    });

    // ---- sink ----
    tf::Task doneTask = tf.emplace([this, t0]() {
        PackResult& result = *result_;
        result.totalMicros = microsSince(t0);
        const bool ok = result.objFile != nullptr;
        AP_LOG("done", "%s total %llu us (geometry %llu / pack %llu / "
               "materials %llu / textures %llu)",
               ok ? "OK" : "FAILED",
               static_cast<unsigned long long>(result.totalMicros),
               static_cast<unsigned long long>(result.importMicros),
               static_cast<unsigned long long>(result.verticesMicros),
               static_cast<unsigned long long>(result.materialsMicros),
               static_cast<unsigned long long>(result.texturesMicros));
        fireAllDone(result, ok,
                    ok ? std::string_view{}
                       : std::string_view(lastError_));
        AP_LOG("event", "onAllDone fired");
    });

    mapTask.precede(geometryTask, materialsTask);
    geometryTask.precede(bindTask, doneTask);
    materialsTask.precede(bindTask, texturesTask, doneTask);
    texturesTask.precede(doneTask);
    bindTask.precede(doneTask);

    if (async) {
        executor_->run(tf);
        return true;
    }
    executor_->run(tf).wait();
    return result_->objFile != nullptr;
}

} // namespace ap
