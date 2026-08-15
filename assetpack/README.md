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

## 查看器（assetpack_viewer）

编译后直接运行（无参数默认加载 San Miguel）：窗口立即打开并显示
加载进度条，模型按解析进度渐进出现 —— 顶点事件先到（默认灰色），
材质事件替换 diffuse 颜色，贴图事件触发并行解码（stb_image），
随后网格切换到 uv 纹理采样渲染，全部完成后显示解析耗时。

```bat
build\Release\assetpack_viewer.exe                # 默认模型
build\Release\assetpack_viewer.exe model.obj      # 指定模型
build\Release\assetpack_viewer.exe --wait --frames 60   # 解析完成后计帧
```

- 16 线程软件光栅化（olive.c 渲染 API + 自写透视 z-buffer：线程局部
  缓冲 + 主线程深度合成），模型顶点池每帧并行变换一次；首帧自动
  探测模型 winding 并做背面剔除。
- 默认绘制全部三角形（San Miguel 全画约 12fps）；`--tris N` 限制每帧
  预算（超出时按 stride 均匀采样），窗口内 Up/Down 实时增减预算
  （Down 到底 = 全画）。
- 鼠标交互：左键拖拽旋转视角（拖拽时暂停自动旋转）、滚轮缩放
  （距离 0.15..10，可贴近模型表面）。
- 光照：无光（unlit）直出 —— 顶点色 = 材质 diffuse，纹理路径直接显示
  贴图像素原色（贴图自带烘焙光照），不做光照计算。
- 贴图：onTexturesReady 时按 `TexDiffuse` 引用并行 mmap + stb_image
  解码（San Miguel：323 引用 / 143 MB / 264 张解码 ≈ 9160 万像素），
  透视校正插值 uv，纹理色与光照 shade 相乘；无贴图的网格回落纯色。
- `--frames N` 渲染 N 帧退出（`--wait` 时从解析完成开始计）、
  `--shot N file.bmp` 第 N 帧截图、Space 暂停旋转、Esc 退出。
- 渲染性能与预算追加到 benchmark.md 的 `## render` 小节。

依赖（可选，`ASSETPACK_WITH_VIEWER` 默认 ON）：olive.c 单文件渲染库、
SDL2 开发包、stb_image（已随仓库提供 third_party/stb）；
路径用 `-DOLIVEC_ROOT=... -DSDL2_ROOT=...` 配置。
