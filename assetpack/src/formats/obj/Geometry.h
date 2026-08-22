#pragma once
// ============================================================
// Geometry - the geometry stage of the OBJ parse (internal):
// chunk-parallel pass1 counts / prefix sizing / pass2 fills with
// seam-splitting / parallel seamFill / per-vertex attribute expansion
// (pass3) / mesh packing + bounds, wired as one taskflow subflow.
// ============================================================

#include <array>
#include <atomic>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <taskflow/taskflow.hpp>

#include <assetpack/AssetPack.h>

#include "ObjParser.h"
#include "Scan.h"

namespace ap::detail { struct SeamShard; }   // defined in Geometry.cpp

namespace ap::obj {

// fires the parser's stage events on completion via the re-exposed
// internal fire* interface
class GeometryStage {
public:
    GeometryStage(ObjParser& owner,
                  std::shared_ptr<PackResult> result,
                  std::shared_ptr<std::vector<std::string_view>> mtlNames,
                  bool wantNormals, bool wantTexcoords);

    // build the geometry subflow into `sf`; fires progress/vertices
    // events through the owner when the build task completes
    void run(tf::Subflow& sf);

private:
    ObjParser& owner_;
    std::shared_ptr<PackResult> result_;
    std::shared_ptr<std::vector<std::string_view>> mtlNames_;
    const bool wantN_;
    const bool wantUv_;

    // shared state across the subflow's tasks
    std::shared_ptr<std::vector<ChunkInfo>> chunks_;
    std::shared_ptr<std::vector<float>> vnPool_, uvPool_;
    std::shared_ptr<std::vector<uint32_t>> vnIdx_, uvIdx_;
    std::shared_ptr<std::array<detail::SeamShard, 16>> shards_;
    std::shared_ptr<std::atomic<uint32_t>> seamCounter_;
    std::shared_ptr<size_t> baseVerts_;
    // material names that carry a diffuse map (views into the mtl
    // mapping); pass2 skips seam-splitting for every other material
    std::shared_ptr<std::unordered_set<std::string_view>> texMats_;
};

} // namespace ap::obj
