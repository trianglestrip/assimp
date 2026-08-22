# TODO — 解析到渲染剩余优化空间（2026-08-23）

> 结论：**仍有较大空间，但重心已从几何转到纹理**。当前 bistro `8.4M 顶点 / 2.8M 三角 / 1591 mesh` 在 `RTX 2060 SUPER / Release` 上：
> - `token 85ms`（零拷贝 `string_view` + `from_chars` 后）+ `ply 400-500ms`（并行 `850` 文件 + `8 角点` 世界包围 + `CTM` 已上 GPU，`src/formats/pbrt/PbrtParser.cpp:1206`）
> - `geometry streamed 0.7s / 194MB`（`PbrtParser.cpp:1024` 的 `onMeta` 预分配 + `DxRenderer.cpp:1197` 的 `stageGeometryRange` 流式，`pub-wait 25ms`）
> - `textures 2.6s / 123 PNG / 432M 像素`（`viewer/TexPipeline.cpp:149` 的 `WIC 优先 + stb 回退` + `TexPipeline.h:47` 队列，`viewer/TexPipeline.cpp:203` 的 `prefetch`）
> - `30 帧 50-60fps`（`viewer/DxRenderer.cpp:1319` 的 `drawScene` 每帧 `1591` 次 `DrawIndexedInstanced`）
>
> **纹理占总时长 65-70%**，是唯一“大”空间；几何已接近下限，绘制次之。

## 已完成（基准）
- `PbrtParser` 零拷贝分词 + `from_chars`（`121ms→85ms`）
- `ply O(n²) concat` 预分配、`CTM` 上 `GPU`（`world[16]/normalWorld[9]`，`8 角点` 求包围）、`globalExecutor` 统一 `PbrtParser` 与 `TexPipeline` 共享 `HC` 池
- `geometry` 流式上传重叠解析尾段（`1.13s→0.70s`）
- `WIC` + `prefetch`（`stb` 回退）

## 剩余大优化空间（按 ROI）

### P0 — 纹理（`2.6s`，保持一次性解析、并行解码到 GPU）
- **当前即目标形态**：`PbrtParser.cpp:715` 的 `ensureTexture` 去重后 `fireTextures` 一次性全量（`254 refs`），`viewer/viewer_demo.cpp:640` 的 `onTexturesReady` 批量 `enqueue`，`TexPipeline.h:47` 队列 + `TexPipeline.cpp:149` 的 `WIC 优先/stb 回退` 在 `globalExecutor` 上并行解码，`viewer/DxRenderer.cpp:1363` 生成 `mips` 后分帧 `texBudget` 上传 — 已是一次性解析、并行解码到 `GPU` 的简洁路径，不做可见性/降采样分支
- **小增益**：更快的 `PNG` 路径 `libspng+zlib-ng` 可再降 `15-25%`；`EXR` 天空盒 `sky.exr` 需 `tinyexr` 补齐（`stb` 不支持）

### P1 — 几何（`0.4-0.5s → 0.25s`）
- **顶点量化/压缩**：`posPool` `float32×3` → `int16/uint16` 量化 + `dequant` 在 `VS`（`g_world` 已上 `GPU`），`194MB → ~80MB`，上传与 `concat` 同步受益；`meshopt` 顶点缓存优化 `idxPool` 可提 `GPU` 顶点命中
- **Ply 解析**：`PlyParser:192` 的逐行 `splitWS` + `stoul/from_chars` 仍为标量，`P/N/uv` 大数组可用 `fast_float` 批量 + `SIMD`，`850` 小文件 `openShared` 开销可用单次目录预扫描合并

### P2 — 绘制（`1591 draws / 12-60fps 抖动`）
- **状态排序已做**（`viewer_demo.cpp:1042` 按 `texSlot` 稳定排序），但仍 `1591` 次 `SetGraphicsRoot* + DrawIndexed`；`133` 材质中同材质多 `mesh` 可 `ExecuteIndirect` 或实例化合并，`recordMs` 可降 `30-40%`
- **纹理预算自适应**：`viewer_demo.cpp:1052` 的 `texBudget=32` 固定，改为 `gpuTiming:88` 的 `frameMs` 反馈自适应（`<16.6ms` 增、`>16.6ms` 减），已在 `todo` 预留

### P3 — 架构
- `taskflow` 已统一单池 `src/core/TaskExecutor.h:3`，但 `viewer` 的 `ImGui`/`font` 仍 `std::thread`，可纳入同一 `DAG`：`token→ply→concat→publish→textureEnqueue→WIC` 显式 `succeed/precede`，`corun` 重叠更精细

## 下一步建议（无离线约束，按用户定案：纹理一次性）
1. **纹理保持一次性并行解码到 GPU**（`P0` 已是目标形态，后续仅 `WIC→libspng` 小优 + `tinyexr`）
2. **顶点量化**（`P1`，运行时 `VS` 反量化，`194MB` 减半）
3. **预算自适应**（`P2`，`texBudget` 按 `frameMs` 反馈，`2` 行逻辑）

> 注：单次测量受本机温控波动 `0.4s↔13s` 影响，已改用交替 `A/B` 均值（`R1/R2 BASE 429/440ms vs PLAN 489/481ms`）为准，`benchmark.md` 保留原始波动记录。
