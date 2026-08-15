cmake_minimum_required(VERSION 3.20)
project(assetpack CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# assetpack - 模型打包库：mmap + taskflow 自定义解析器（无 assimp 依赖）。
# 解析器：ap::ModelParser（基础接口）→ ap::ObjParser（OBJ/MTL，自定义
# 解析），模型/材质/贴图三段解析完成各自触发带数据的事件。

# ---- library ----
add_library(assetpack
    src/AssetPack.cpp
    src/ObjParser.cpp
    src/MappedFile.cpp
    src/Log.cpp
)
target_include_directories(assetpack
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party   # <taskflow/taskflow.hpp>
)
target_compile_features(assetpack PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(assetpack PRIVATE /W4 /permissive- /utf-8)
    target_compile_definitions(assetpack PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

find_package(Threads REQUIRED)
target_link_libraries(assetpack PUBLIC Threads::Threads)

# ---- demo: 解析 + benchmark ----
add_executable(pack_demo examples/pack_demo.cpp)
target_link_libraries(pack_demo PRIVATE assetpack)
if(MSVC)
    target_compile_options(pack_demo PRIVATE /utf-8)
endif()

# ---- viewer: 渐进式模型查看器（olive.c 软件渲染 + SDL2 窗口）----
# 编译后直接运行 assetpack_viewer 即打开窗口并加载默认模型（San Miguel）。
set(ASSETPACK_WITH_VIEWER ON CACHE BOOL "Build the olive.c software-renderer viewer demo")
if(ASSETPACK_WITH_VIEWER)
    set(OLIVEC_ROOT "F:/project/third_party/olive.c" CACHE PATH "olive.c single-header renderer root")
    set(SDL2_ROOT "F:/project/third_party/SDL2-2.32.10" CACHE PATH "SDL2 dev package root")
    if(NOT EXISTS "${OLIVEC_ROOT}/olive.c")
        message(FATAL_ERROR "ASSETPACK_WITH_VIEWER=ON but olive.c not found at OLIVEC_ROOT=${OLIVEC_ROOT}")
    endif()
    if(NOT EXISTS "${SDL2_ROOT}/include/SDL2/SDL.h" OR NOT EXISTS "${SDL2_ROOT}/lib/x64/SDL2.lib")
        message(FATAL_ERROR "ASSETPACK_WITH_VIEWER=ON but SDL2 not found at SDL2_ROOT=${SDL2_ROOT}")
    endif()

    # olive.c 需要 C99（字体表用 designated initializer），C++ 侧通过桥调用
    add_library(olive_bridge STATIC examples/olive_bridge.c)
    target_include_directories(olive_bridge PRIVATE ${OLIVEC_ROOT})
    if(MSVC)
        target_compile_options(olive_bridge PRIVATE /std:c11)
    endif()

    add_executable(assetpack_viewer examples/viewer_demo.cpp)
    target_include_directories(assetpack_viewer PRIVATE
        ${OLIVEC_ROOT} ${SDL2_ROOT}/include ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    target_link_libraries(assetpack_viewer PRIVATE assetpack olive_bridge
        "${SDL2_ROOT}/lib/x64/SDL2.lib" "${SDL2_ROOT}/lib/x64/SDL2main.lib")
    target_compile_definitions(assetpack_viewer PRIVATE SDL_MAIN_HANDLED NOMINMAX)
    if(MSVC)
        target_compile_options(assetpack_viewer PRIVATE /utf-8)
    endif()
    add_custom_command(TARGET assetpack_viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SDL2_ROOT}/lib/x64/SDL2.dll" $<TARGET_FILE_DIR:assetpack_viewer>)
endif()
