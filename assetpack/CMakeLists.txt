cmake_minimum_required(VERSION 3.20)
project(assetpack CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# assetpack - 模型打包库：mmap + taskflow 自定义解析器（无 assimp 依赖）。
# 解析器：ap::ModelParser（基础接口）→ ap::ObjParser（OBJ/MTL，自定义
# 解析），模型/材质/贴图三段解析完成各自触发带数据的事件。

# ---- library ----
# 文件结构：include/ 只放对外 API；src/core 库骨架（门面/日志/mmap）；
# src/detail 通用解析工具；src/formats/<fmt> 每格式一目录（对齐
# assimp 上游 code/AssetLib/<Format>/ 惯例）。
add_library(assetpack
    src/core/AssetPack.cpp
    src/core/Config.cpp
    src/core/Log.cpp
    src/core/MappedFile.cpp
    src/formats/obj/ObjParser.cpp
    src/formats/obj/Scan.cpp
    src/formats/obj/Geometry.cpp
    src/formats/obj/Mtl.cpp
    src/formats/stl/StlParser.cpp
    src/formats/ply/PlyParser.cpp
    src/formats/gltf/GltfParser.cpp
    src/formats/fbx/FbxParser.cpp
    src/formats/pbrt/PbrtParser.cpp
)
target_include_directories(assetpack
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src           # core/ detail/ formats/ 内部头
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party   # <taskflow/taskflow.hpp>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rapidjson/include  # glTF JSON
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glm/include        # parser matrix math
)
target_compile_features(assetpack PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(assetpack PRIVATE /W4 /permissive- /utf-8)
    target_compile_definitions(assetpack PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

find_package(Threads REQUIRED)
target_link_libraries(assetpack PUBLIC Threads::Threads)

# ---- demo: 解析 + benchmark ----
add_executable(pack_demo viewer/pack_demo.cpp)
target_link_libraries(pack_demo PRIVATE assetpack)
if(MSVC)
    target_compile_options(pack_demo PRIVATE /utf-8)
endif()

# ---- viewer: 渐进式模型查看器（Direct3D 12 + DirectXTK12，SDL2 窗口）----
# 编译后直接运行 assetpack_viewer 即打开窗口并加载默认模型（San Miguel）。
# 源码拆分：viewer_demo.cpp 主循环 + TexPipeline 并行解码 + DxRenderer D3D12 后端。
set(ASSETPACK_WITH_VIEWER ON CACHE BOOL "Build the Direct3D 12 viewer demo")
if(ASSETPACK_WITH_VIEWER)
    set(SDL2_ROOT "F:/project/third_party/SDL2-2.32.10" CACHE PATH "SDL2 dev package root")
    if(NOT EXISTS "${SDL2_ROOT}/include/SDL2/SDL.h" OR NOT EXISTS "${SDL2_ROOT}/lib/x64/SDL2.lib")
        message(FATAL_ERROR "ASSETPACK_WITH_VIEWER=ON but SDL2 not found at SDL2_ROOT=${SDL2_ROOT}")
    endif()

    # Dear ImGui: immediate-mode UI, vendored under third_party/ and built
    # as a static library. Only the SDL2 + DX12 backends are compiled; the
    # other backend files in the tree are inert. The DX12 backend needs
    # d3dcompiler at link time (it compiles its shaders with D3DCompile).
    add_library(imgui STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_draw.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_tables.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_widgets.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends/imgui_impl_dx12.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends/imgui_impl_sdl2.cpp)
    target_include_directories(imgui PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends
        ${SDL2_ROOT}/include)
    target_compile_features(imgui PUBLIC cxx_std_20)
    target_link_libraries(imgui PUBLIC d3d12 dxgi d3dcompiler
        "${SDL2_ROOT}/lib/x64/SDL2.lib")

    add_executable(assetpack_viewer viewer/viewer_demo.cpp
                             viewer/TexPipeline.cpp
                             viewer/DxRenderer.cpp
                             viewer/RendererFactory.cpp)
    target_include_directories(assetpack_viewer PRIVATE
        ${SDL2_ROOT}/include ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    # DirectX Tool Kit for DX12: device/sprite/texture-upload plumbing
    set(BUILD_XAUDIO_WIN10 OFF CACHE BOOL "" FORCE)
    set(BUILD_XAUDIO_REDIST OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMEINPUT OFF CACHE BOOL "" FORCE)
    set(BUILD_WGI OFF CACHE BOOL "" FORCE)
    set(BUILD_XINPUT OFF CACHE BOOL "" FORCE)
    set(BUILD_DXIL_SHADERS OFF CACHE BOOL "" FORCE)   # this machine has fxc, not dxc
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/DirectXTK12
                     ${CMAKE_CURRENT_BINARY_DIR}/DirectXTK12)
    target_link_libraries(assetpack_viewer PRIVATE assetpack imgui DirectXTK12
        "${SDL2_ROOT}/lib/x64/SDL2.lib" "${SDL2_ROOT}/lib/x64/SDL2main.lib"
        d3d12 dxgi d3dcompiler windowscodecs shlwapi)
    # PIX GPU capture: WinPixEventRuntime (vendored under third_party/pix,
    # Microsoft license) provides programmatic capture - run the viewer with
    # --pix <file.wpix> to record a GPU timeline for PIX analysis. USE_PIX
    # turns the pix3.h stubs into the real event/capture API.
    target_include_directories(assetpack_viewer PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pix/include)
    target_compile_definitions(assetpack_viewer PRIVATE
        SDL_MAIN_HANDLED NOMINMAX USE_PIX=1)
    target_link_libraries(assetpack_viewer PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pix/lib/x64/WinPixEventRuntime.lib")
    if(MSVC)
        target_compile_options(assetpack_viewer PRIVATE /utf-8 /Zi)
        target_link_options(assetpack_viewer PRIVATE /DEBUG)
    endif()
    add_custom_command(TARGET assetpack_viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SDL2_ROOT}/lib/x64/SDL2.dll" $<TARGET_FILE_DIR:assetpack_viewer>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pix/lib/x64/WinPixEventRuntime.dll"
                 $<TARGET_FILE_DIR:assetpack_viewer>)
endif()

# ---- unit tests (dependency-free, self-contained harness) ----
add_executable(assetpack_tests test/assetpack_tests.cpp)
target_include_directories(assetpack_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
target_link_libraries(assetpack_tests PRIVATE assetpack)
if(MSVC)
    target_compile_options(assetpack_tests PRIVATE /W4 /permissive- /utf-8)
endif()
