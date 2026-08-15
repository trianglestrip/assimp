#include <assetpack/AssetPack.h>
#include <assetpack/ObjParser.h>

namespace ap {

AssetPack::AssetPack(unsigned threads)
    : parser_(std::make_unique<ObjParser>(threads)) {}

AssetPack::~AssetPack() = default;

} // namespace ap
