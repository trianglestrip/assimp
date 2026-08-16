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

San Miguel（1.14GB OBJ）实测：5.93M 位置顶点 / 9.98M 三角形 / 2203 meshes /
287 材质 / 323 贴图引用。几何解析（含纹理接缝顶点分裂：5.93M → 9.02M
输出顶点，保证镜像/接缝 uv 正确）热缓存约 1.9s（assimp 版为 14s+）。
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

- 仅原生 Direct3D 12 GPU 渲染:场景由查看器自定义 PSO 绘制,
  HUD/加载画面和 GPU 上传使用微软 DirectXTK12 的 `SpriteBatch` /
  `ResourceUploadBatch` / `GraphicsMemory`;`--warp` 选择 WARP 软件设备。
- 源码按职责拆分:`viewer_demo.cpp` 主循环 + 事件绑定,
  `TexPipeline` 在解析线程并行 mmap + stb_image 解码贴图(保留
  RGBA 字节直接上传,无换色往返),`DxRenderer` 拥有设备/交换链/
  帧调度;解析与显示共用 taskflow:解析侧 chunk 并行 + 接缝分裂,
  显示侧 HUD/加载条在独立 `tf::Executor` 上异步光栅化(最新完成
  交接,渲染线程零等待)。帧调度为双缓冲在飞(按帧 fence 等待,
  不再全量等 GPU 空闲)、几何零拷贝上传(positions/texcoords/索引
  池原样进 VBO/IBO)、贴图分帧上传并即用即释。
- mmap + taskflow 的自定义 OBJ/MTL 解析器和 stb_image 贴图解码,
  不依赖 assimp;法线默认解析(`setWantNormals(false)` 可选跳过)。
- 贴图上传后用 DirectXTK12 `GenerateMips` 在 GPU 生成完整 mip 链,
  采样器 point + mip 线性(近处保持锐利 texel,远处按 mip 混合),
  San Miguel 全画约 38 fps(无 mip 时 16 fps)。
- 鼠标交互：左键拖拽旋转视角、滚轮缩放（距离 0.15..10，可贴近
  模型表面）；无自动旋转，WASD 沿视线前进/后退 + 左右平移
  （步长随缩放距离缩放）。
- `--untex` 禁用贴图绑定（与贴图版对比查看）。
- 光照：无光（unlit）直出 —— 顶点色 = 材质 diffuse，纹理路径直接显示
  贴图像素原色（贴图自带烘焙光照），不做光照计算。
- 贴图：onTexturesReady 时 TexPipeline 按 `TexDiffuse` 引用并行 mmap +
  stb_image 解码（San Miguel：323 引用 / 143 MB / 264 张解码 ≈ 9160 万
  像素），GPU 采样纹理色 × 材质色；无贴图的网格回落纯色。
- `--frames N` 渲染 N 帧退出（`--wait` 时从解析完成开始计）、
  `--shot N file.bmp` 第 N 帧截图、Esc 退出。
- 渲染性能追加到 benchmark.md 的 `## render` 小节。

依赖（可选，`ASSETPACK_WITH_VIEWER` 默认 ON）：SDL2 开发包、
stb_image / stb_truetype（已随仓库提供 third_party/stb，界面文字用
系统 Consolas 烘焙）、DirectXTK12（已随仓库提供 external/DirectXTK12）；
路径用 `-DSDL2_ROOT=...` 配置。
