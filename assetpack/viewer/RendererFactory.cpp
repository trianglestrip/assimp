#include "RendererFactory.h"

#include "DxRenderer.h"
#include "IRenderer.h"
#include "NullRenderer.h"

namespace view {

std::unique_ptr<IRenderer> createRenderer(const std::string& name) {
    if (name == "dx12" || name.empty()) return std::make_unique<DxRenderer>();
    if (name == "null") return std::make_unique<NullRenderer>();
    return nullptr;
}

} // namespace view
