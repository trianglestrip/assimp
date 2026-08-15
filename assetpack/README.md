# assetpack — 模型打包库（mmap + taskflow 自定义解析，无 assimp 依赖）

以任务图方式解析模型，每个阶段完成时触发**携带该阶段数据**的事件，
方便打包器/渲染器在各部分就绪后立即独立消费。

## 结构

```
[mmap 主文件 + 辅助文件（mtl 等）]
    ├─ [geometry   几何解析（chunk 并行两遍扫描 + vn/vt 展开）]
    │     ──► onVerticesReady(span<PackMesh>)
    ├─ [materials  mtl 解析]         ──► onMaterialsReady(span<PackMaterial>)
    └─ [textures   贴图引用收集]     ──► onTexturesReady(span<PackTexture>)
all ──► onAllDone
```

几何与材质/贴图解析同时开始、互不等待。几何是 chunk 并行两遍扫描：
pass1 计数 + 前缀和定位，pass2 填充扁平数组 + 扇形三角化 + 负索引
解析；pass3 把 OBJ 的 vn/vt 引用展开成与位置一一对应的每顶点数组
（渲染契约）。

解析器接口：`ap::ModelParser`（基础类，统一事件/输出），
`ap::ObjParser : ModelParser`（OBJ/MTL 实现），`ap::AssetPack` 是
外观类，默认持有一个 ObjParser，换解析器只需替换构造。

## 事件携带的数据

- **onVerticesReady** — `span<const PackMesh>`：每 mesh 的
  `positions`(3f)、`normals`(3f)、`texcoords`(2f) 零拷贝 span，
  `indices`(3×tri 紧凑打包)、包围盒、材质索引。
- **onMaterialsReady** — `span<const PackMaterial>`：diffuse/specular/
  ambient/emissive/opacity/shininess/twoSided + 绑定贴图槽列表。
- **onTexturesReady** — `span<const PackTexture>`：贴图路径、类型/槽位、
  嵌入字节（零拷贝视图）、外部文件解析出的绝对路径。
- **onProgress(float)** — 解析进度 0~100%。
- **onAllDone(result, ok, err)**。

## 构建

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

无第三方依赖（taskflow 3.11 header-only，随仓库 third_party/）。

## 运行

```bat
build\Release\pack_demo.exe F:\project\meshToBrowser\models\San_Miguel\san-miguel.obj
```

San Miguel（1.14GB OBJ）实测：5.93M 顶点 / 9.98M 三角形 / 2203 meshes /
287 材质 / 323 贴图引用，几何解析约 0.6s（assimp 版为 14s+）。
每次运行把各阶段耗时追加到 benchmark.md。
