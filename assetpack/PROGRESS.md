# assetpack 处理进度与剩余工作

> 更新日期：2026-08-22　分支：`master`　远端：`origin/trianglestrip/assimp`

## 一、项目现状总览

assetpack 是位于 `F:\project\assimp\assetpack` 的独立 C++20 库：
mmap 零拷贝 + taskflow 并行解析，统一事件模型（onVertices/onMaterials/
onTextures/onAllDone），支持几何流式上传（GeoStreamSink）。现已支持 **6 种格式**：

| 格式 | 扩展名 | 说明 |
|---|---|---|
| OBJ/MTL | `.obj` | chunk 并行两遍扫描、seam 拆分、emissive(Ke/map_Ke)、透射(Tr/map_Tr)、贴图字节加载 |
| STL | `.stl` | ASCII + 二进制 |
| PLY | `.ply` | ASCII + binary_little_endian |
| glTF 2.0 | `.gltf` `.glb` | rapidjson（已 vendor 到 third_party）、内嵌 data:/bufferView 与外部贴图 |
| FBX | `.fbx` | 二进制 7400+；ASCII 不支持；压缩数组跳过并告警 |
| PBRT-v4 | `.pbrt` | trianglemesh/mesh、CTM 变换栈、材质、Texture、Include |

通用能力：`cancel()` 取消、`warnings()` 非致命问题、
`setWantNormals/Texcoords/Positions/TextureBytes` opt-out、POSIX mmap 后端。

## 二、已完成里程碑（均已推送）

| 提交 | 内容 |
|---|---|
| `115843257` | stl/ply/gltf/fbx 解析器 + cancel/warnings/textureBytes/opt-out + POSIX mmap + 单测脚手架（68 checks 全过） |
| `4b7adcccf` | PBRT-v4 解析器初版（trianglemesh、材质、变换；plymesh 仅 ASCII 内联读取） |

验证基线：`assetpack_tests.exe` **68/68 通过**（含 assimp 真实样例
BoxTextured.gltf/.glb、box.fbx 功能用例）。

## 三、本轮已完成（待随下次提交入库）

- **plymesh 升级完成**：`PbrtParser` 经 `ap::AssetPack` 复用共享 PLY 解析器加载
  引用的 `.ply`（ASCII+二进制），按当前 CTM 变换后并入结果池；
  `PlyParser` 补齐 `int8..uint64` 数据类型别名（bistro 用 `uint8`）。
- **渲染工厂化完成**：`viewer/IRenderer.h` 抽象接口（DxRenderer 实现之）、
  `NullRenderer` 无头后端、`RendererFactory` 按名创建；viewer 新增
  `--renderer dx12|null`。
- **顺带修掉两个真 bug**：
  1. 非 OBJ 解析器不发 onMeta → 主循环永卡加载页：新增「无流式支持则阻塞
     setGeometry 上传」回退分支（dx12 渲染 .pbrt 同样受益）。
  2. null 后端跳过渲染后无人建字体图集，首次画文本即崩：null 时整帧跳过
     ImGui。
- **测试**：单测 71/71（新增 bistro 冒烟 ≥1591 meshes）；viewer 无头端到端
  `--renderer null --wait --frames 2` 加载 bistro：1591 meshes /
  133 materials / 2.5 s / EXIT=0。
- **GLM 与结构设计**（sub-agent 产出）：GLM 1.0.1 已 vendor 至
  `third_party/glm`（未接 CMake）；方案见
  `docs/DESIGN-glm-and-layout.md`——建议内部集成 GLM（PbrtParser 的手写
  mat4 可逐函数替换；公共 API 保持零依赖），src 重构为
  `core/ + formats/{obj,stl,ply,gltf,fbx,pbrt}/`（仅 4 处 include +
  CMake 列表重排，成本极低）。

## 四、剩余工作
```powershell
cmake -G "Visual Studio 17 2022" -A x64 -DASSETPACK_WITH_VIEWER=OFF -S . -B build
cmake --build build --config Release --target assetpack_tests
build\Release\assetpack_tests.exe
# viewer 编译验证（SDL2 在 F:/project/third_party/SDL2-2.32.10，已就绪）
cmake -DASSETPACK_WITH_VIEWER=ON -S . -B build
cmake --build build --config Release --target assetpack_viewer
```

### 5. 提交推送
一次提交涵盖：examples→viewer 重命名、CMake 路径、registerPbrtParser、
plymesh 升级、渲染工厂、新测试。**排除 `assetpack/.codegraph/`**。

### 6. 人工验收（需 GPU，本机执行）
```powershell
assetpack_viewer D:\models\pbrt-v4-scenes\bistro\bistro_cafe.pbrt --frames 300 --stat bistro
```
预期：窗口出现、几何渐进流入、材质漫反射色正确。

## 五、已知限制（按解析器）

| 解析器 | 不支持 |
|---|---|
| glTF | draco/meshopt、skin/animation/morph、sparse、KHR_texture_* 扩展 |
| FBX | ASCII、zlib 压缩数组、skin/animation、多 UV 层仅取第一层 |
| PBRT | 曲面/曲线/disk 等非网格 Shape、ObjectBegin/Instance 实例化、动画变换、EXR 贴图解码（stb_image 不识别 .exr，回退漫反射色） |
| OBJ | mtllib 仅扫文件头 4MB |

## 六、备注
- 会话工具链曾出现长参数 JSON 截断（agent/edit 偶发失败），遇到时改用
  小步编辑即可。
- 测试样例路径硬编码于 `test/assetpack_tests.cpp`，缺失自动跳过以保证
  hermetic。
