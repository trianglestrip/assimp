#pragma once
// ============================================================
// RendererFactory - picks a viewer rendering backend by name.
// ============================================================

#include <memory>
#include <string>

class IRenderer;

namespace view {

// "dx12" -> native D3D12 backend (default); "null" -> headless no-op.
// Returns null for unknown names.
std::unique_ptr<IRenderer> createRenderer(const std::string& name);

} // namespace view
