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
（渲染契约）。纹理接缝顶点分裂（镜像/接缝 uv 需要）只对**含
map_Kd 贴图的材质**生效：解析前用一次毫秒级 mtl 扫描确定带 diffuse
贴图的材质集，无贴图网格直接复用基顶点，不再复制顶点。

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
287 材质 / 323 贴图引用。几何解析（纹理接缝顶点分裂只对带贴图材质：
5.93M → 8.04M 输出顶点，无贴图的 1086 个 mesh 不再复制顶点）热缓存
约 1.0s（assimp 版为 14s+；优化前为 1.2s / 9.02M 顶点）。
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
build\Release\assetpack_viewer.exe --wait --frames 300 --stat opt1   # 每帧记录
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
  池原样进 VBO/IBO)、贴图分帧上传并即用即释。绘制前按贴图槽对
  draw item 排序，渲染器每段贴图只绑定一次描述符表（2203 draws
  录制 ~3.2 ms，优化前 ~3.6 ms）。
- ImGui HUD（third_party/imgui 静态库）：面板实时显示模型名、三角形
  数、CPU/GPU 帧时间、scene/overlay 分段、贴图数；GPU 时间用 D3D12
  timestamp query 环测量（延迟两帧读回，不阻塞热路径）。
- `--stat <name>` 每帧记录（UE STAT UNIT 风格）：以
  `frame,cpu_ms,gpu_ms,scene_ms,ovl_ms,wait_ms,record_ms` 一行/帧写入
  `stat/<name>.csv`，并在 benchmark.md 的 `## compare` 同项对比表追加
  一行（版本标签 + 本次运行的导入/顶点数/各项均值）。`tools/stat_plot.html`
  是零依赖折线图查看器：浏览器打开后多选 stat/*.csv 即可叠加对比不同
  优化版本（下拉切换指标、图例点击隐藏、悬浮读值、虚线为均值）。
- mmap + taskflow 的自定义 OBJ/MTL 解析器和 stb_image 贴图解码,
  不依赖 assimp;法线默认解析(`setWantNormals(false)` 可选跳过)。
- 贴图上传后用 DirectXTK12 `GenerateMips` 在 GPU 生成完整 mip 链,
  采样器 point + mip 线性(近处保持锐利 texel,远处按 mip 混合),
  稳态约 60 fps（窗口合成器锁帧；无 mip 时远处过采样拉高 GPU 时间）。
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

## 性能（San Miguel 实测，RTX 2060 SUPER @ 1080p）

帧时间分解（每 30 帧遥测，D3D12 timestamp query 测 GPU、steady_clock
测 CPU）：

| 段 | 耗时 | 说明 |
| --- | ---: | --- |
| CPU 整帧 | 16.7 ms | 稳定 60 fps |
| └ waitFence | ~12.3 ms | 等两帧前 fence；与合成器节奏吻合 |
| └ 绘制录制 | ~3.2 ms | 2203 draws（优化前 3.6 ms） |
| └ 事件/UI | ~0.1 ms | SDL 事件 + ImGui |
| GPU 整帧 | ~3.7 ms | 其中 scene 3.7 + ImGui overlay 0.02 |
| Present | 0.08 ms | 非阻塞 |

**瓶颈定位**：GPU 每帧只忙 ~3.7 ms（22%），CPU 有效工作 ~5 ms（30%），
帧率精确锁 60 fps —— 限速的是窗口合成器（flip-model 交换链 + 2 个
后备缓冲被 DWM 以 60 Hz 节奏调度），不是 GPU 也不是 CPU。证明：
waitFence 12.3 ms ≈ 16.7 − CPU 有效 ~5 ms，且 GPU 时间刻度曾因缓存
timestamp 频率被低估（timestamp 时钟跟随 GPU 动态核心时钟，读回时
必须重新 `GetTimestampFrequency`，否则显示 4.5 ms 与墙钟矛盾）。

**已执行优化**：

1. **无贴图材质跳过纹理接缝分裂**（解析器）。接缝分裂的存在意义是
   贴图采样时 uv 接缝不漏缝；无 map_Kd 的 mesh 渲染回退白色，分裂
   纯属复制顶点。新增一次毫秒级 mtl 扫描得到带 diffuse 贴图的材质集，
   pass2 按当前 usemtl 决定是否分裂（chunk 边界处由前缀任务把材质名
   跨块串接）。效果：分裂顶点 3,088,160 → 2,104,434（-32%），输出
   顶点 9,021,393 → 8,037,667（-10.9%，约 -20 MB 顶点数据），导入
   1222 ms → 1010 ms（-17%）。贴图网格路径字节级不变。
2. **按贴图槽排序 draw item + 描述符表按段绑定**（渲染器）。2203 次
   逐 draw 的描述符表绑定改为每段贴图一次：录制 3.60 → 3.17 ms（-12%）。
3. **GPU 时间测量修复**：读回 timestamp 时重新查询频率（见上）。

**待评估的优化方向**：4x MSAA 是 ROP 大头（1x 可把 scene ~3.7 ms 降到
~1 ms 量级，但以抗锯齿质量为代价，可做运行时开关）；16-bit 索引
（29.9M 索引 120 MB → ~60 MB，需解析器/渲染器双池改造，纯加载期
收益）；mesh 合并（2203 draws → 按材质几百次，削减 CPU 录制）；
depth prepass（过绘制场景的 PS 减负，双倍 draw 数）。60 fps 之上
需要无合成器路径（独占全屏 / tearing），当前窗口化模式无法突破。

**基准复现**：`--wait --frames 300 --stat <标签>` 每次运行往 benchmark.md
追加一条 `## compare` 行（行=版本，列=指标，同项对比），并把每帧
cpu/gpu/scene/ovl/wait/record 写入 `stat/<标签>.csv`；浏览器打开
`tools/stat_plot.html` 多选 CSV 即可叠加折线对比不同优化版本。

**相机视角 1% low 测试**（`--autoRotate` 每帧绕 Y 轴转 0.01 rad ≈ 10 s
一圈，`--camDist <v>` 覆盖初始距离，可 < 0.15 滚轮下限以放进模型内部）
验证"视角是否影响瓶颈结论"：San Miguel 相机放模型内部（dist 0.15）
与外部远距（dist 2.2）各自动旋转 600 帧，稳态 1% low（最慢 1% 帧均值）：

| 视角 | GPU avg | GPU 1%low | GPU P99 | CPU 1%low |
| --- | ---: | ---: | ---: | ---: |
| 内部旋转 (0.15) | 4.13 ms | 5.16 ms | 5.05 ms | 19.6 ms |
| 外部旋转 (2.2) | 4.28 ms | 5.10 ms | 5.05 ms | 18.7 ms |

GPU 最差帧仅 5.2 ms（占 16.7 ms 帧预算 31%），两个视角几乎无差别
（外部 avg 略高：远距看到整个建筑外立面，近距被 near plane 裁掉
一部分三角形），1% low 完全落在合成器节奏内——GPU 在任何视角都
不是瓶颈。CPU 1% low ~19 ms 的尖峰帧构成：wait_ms 反而低（6.8–
11.7 ms）、record 恒定 ~3.1 ms，即尖峰来自 DWM 合成器节拍抖动，
与场景负载无关。

## PIX 分析（程序化抓帧 + 命令行导出）

viewer 集成了 WinPixEventRuntime（vendored 到 `third_party/pix`，
`USE_PIX=1` 编译宏激活真实 API），支持程序化 GPU 抓帧：

```
assetpack_viewer --pix cap.wpix [--pixStart N] model.obj
```

- `--pix`：从第 N 帧（默认第 0 帧）开始 PIXBeginCapture，帧循环结束
  PIXEndCapture 保存 .wpix。**必须传 `--pixStart` 跳过纹理上传密集段**：
  PIX 的高频计数器窗口只有抓帧开始后 ~100ms，若从第一帧抓，HFC 窗口
  全落在上传上，看不到稳态渲染。
- 要求机器装有 PIX（winget `Microsoft.PIX`）；程序启动时
  loadPixGpuCapturer() 在 D3D12 设备创建前加载 WinPixGpuCapturer.dll
  （PIX 不捕获已存在的设备）。
- DxRenderer 在 drawScene/endFrame 打了 PIXScopedEvent 时间线标记
  （texture uploads / scene draws / msaa resolve / imgui overlay /
  present），PIX GUI 里可按段查看每段 GPU 耗时。

命令行导出（需 Windows 开发者模式）：

```
pixtool open-capture cap.wpix save-high-frequency-counters hfc.csv
pixtool open-capture cap.wpix save-event-list events.csv
```

**San Miguel 稳态实测结论**（`build/Release/pix/ANALYSIS.md`，图表
`hfc_steady_chart.png`）：PIX HFC 的 `GPU Activity` / `3D/Compute
Engine Activity` 计数器在本机（虚拟显示适配器环境）恒报 99-100%，
与 nvidia-smi（30%）和自研 timestamp query（~4ms/帧）矛盾，不可信；
但吞吐类计数器（VS 3.6% / PS 0.4% / Rasterizer 4.4% / SM 4.0%）可信，
与"GPU 大部分空闲、瓶颈在 CPU 录制"的结论吻合。PIX 的每事件 GPU
耗时只能看 GUI 时间线；pixtool 的 timing capture 只能抓不能读回。

