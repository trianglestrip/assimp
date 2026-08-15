cmake_minimum_required(VERSION 3.20)
project(assetpack CXX)

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
