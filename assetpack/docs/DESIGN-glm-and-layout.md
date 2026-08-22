# assetpack 设计文档：GLM 数学库引入与 `src/` 目录重构

| | |
|---|---|
| 状态 | 设计稿（本文档仅为设计研究产出；除 `third_party/glm` 头文件副本外未改动任何源码/CMake/include/test/viewer） |
| 日期 | 2026-08-22 |
| 关联范围 | `src/PbrtParser.cpp`、`src/FbxParser.cpp`、`src/GltfParser.cpp`、`viewer/viewer_demo.cpp`（只读分析）、`CMakeLists.txt`（diff 草案）、`include/assetpack/AssetPack.h`（零依赖约束） |

---

## 0. TL;DR

1. **GLM**：建议引入，且已完成 vendor——从本机既有检出 `F:\project\meshToBrowser\third_party\glm`（GLM **1.0.1**）复制到 `assetpack/third_party/glm/include/glm`（426 个文件，约 2.07 MB，纯 header-only）。本阶段**不接入 CMake**；接入时仅作为 `assetpack` 目标的 PRIVATE 头搜索路径，`include/` 公共头保持零第三方依赖。
2. **目录重构**：将平铺在 `src/` 根的解析器迁入 `src/formats/<format>/`，公共设施迁入 `src/core/`，对齐 assimp 上游 `code/AssetLib/<Format>/` 惯例。得益于现有 `-I src` 的根相对包含风格，**真正需要改 #include 的内部文件只有 2 个**（`core/AssetPack.cpp`、`formats/obj/ObjParser.cpp`），另有 `test/assetpack_tests.cpp` 1 行需同步修正。

---

## 1. 背景与现状盘点

### 1.1 数学代码分布（重复度审计）

| 位置 | 内容 | 规模 |
|---|---|---|
| `src/PbrtParser.cpp:46-156` | 匿名命名空间内自研 `Mat4 = std::array<float,16>`（**列主序**）：identity/mul/translate/scale/rotate/transformPoint/invert/transformNormal 共 8 个函数，约 110 行 | 完整矩阵套件 |
| `src/FbxParser.cpp` | **当前无任何矩阵代码**。二进制 FBX 读的是 Geometry/Material 对象 + Connections；`Properties70` 只用于取材质颜色（L585）。Model 节点的 `Lcl Translation/Rotation/Scaling` 及几何变换**尚未实现** | 未来需求 |
| `src/GltfParser.cpp` | **当前无任何节点/场景图代码**（全文无 node/matrix 字样）：直接从 GLB JSON 读 accessors/bufferViews/meshes/materials。glTF 的 node TRS 合成与 `node.matrix` 属性**尚未实现** | 未来需求 |
| `viewer/viewer_demo.cpp:962-978` | 手工构造行主序 `float cam[16]`：把 orbit 相机（yaw/pitch）、缩放、中心偏移、焦距 `f`、屏幕比例 `2f/W, 2f/H`、D3D 深度重映射 `zA/zB` **融合成一组常量**，喂给 `DxRenderer::drawScene` 的根签名 b0（"camera math 16 DWORDs"）；**顶点着色器镜像了该布局** | 特化布局，非标准 MVP |
| 其余（Stl/Ply/Obj） | 只有逐分量 bbox/float 三元组运算，无矩阵 | 无需求 |

结论：真正的“手写矩阵重复”今天只有 PbrtParser 一处，但 **glTF/FBX 一旦实现节点变换（这是两者最常见的数据组织方式）就各自需要一套 mat4/TRS 合成**；viewer 也有一处手写相机数学。与其将来出现第 2、第 3 份私有 mat4，不如现在统一。

### 1.2 `src/` 目录现状

```
src/
├── AssetPack.cpp        (234 行)  门面 + ParserRegistry
├── ObjParser.cpp        (172 行)  ← 平铺在根，实现在 src/obj/ 的分层之上
├── StlParser.cpp        (224 行)  ← 平铺
├── PlyParser.cpp        (487 行)  ← 平铺
├── GltfParser.cpp       (750 行)  ← 平铺
├── FbxParser.cpp        (855 行)  ← 平铺
├── PbrtParser.cpp      (1012 行)  ← 平铺
├── MappedFile.cpp       (155 行)  公共设施
├── Log.cpp               (46 行)  公共设施
├── detail/
│   ├── FastParse.h      (225 行)  数值快解析内核
│   └── TextScan.h        (50 行)  token 扫描
└── obj/                            ← 仅 OBJ 有子目录（6 组文件）
    ├── Scan.h/.cpp      (87/105)
    ├── ObjParser.h       (70)
    ├── Geometry.h/.cpp  (59/755)
    └── Mtl.h/.cpp       (53/239)
```

**现状问题**

1. 六个格式解析器 `.cpp` 平铺在 `src/` 根，与 assimp 上游 `code/AssetLib/<Format>/` 惯例相悖；每新增一个格式就多污染一层根目录。
2. 只有 OBJ 有子目录，命名也不对称：入口是 `src/ObjParser.cpp`，而它的头却在 `src/obj/ObjParser.h`——“同名不同位”。
3. `core/` 类设施（AssetPack 门面 / Log / MappedFile）与格式实现混在同一层，边界靠注释而非结构表达。
4. `detail/` 定位正确（通用工具），应原样保留。
5. 解析器普遍超过 500 行（Pbrt 1012 行），迁入独立目录后才有自然的位置按阶段拆分（如 pbrt 的 tokenizer / transform / emit）。

---

## 2. GLM 集成结论

### 2.1 结论与依据

> **建议集成 GLM，作为 library 私有依赖 vendored 进仓库；公共 API 保持零依赖。**

已完成动作（本次设计阶段唯一落盘变更）：

```powershell
# 来源：本机既有干净检出（GLM 1.0.1，F 盘，非 UE 插件内嵌副本）
Copy-Item "F:\project\meshToBrowser\third_party\glm" `
          "F:\project\assimp\assetpack\third_party\glm\include\glm" -Recurse
```

- 目标布局：`third_party/glm/include/glm/{glm.hpp, detail/, ext/, gtc/, gtx/, simd/}`，使用时 `-I third_party/glm/include` 后 `#include <glm/glm.hpp>`。
- 版本：`GLM_VERSION_MAJOR 1 / MINOR 0 / PATCH 1`（`detail/setup.hpp:6-8`）。
- 规模：426 文件 / 约 2.07 MB；纯头文件，不参与编译单元，不影响链接产物。
- 本阶段**未**修改 CMakeLists.txt 或任何 `.cpp/.h`（遵守任务硬性规则）。

**候选方案对比**

| 方案 | 优点 | 缺点 | 判定 |
|---|---|---|---|
| 维持各 parser 自带 mat4 | 零依赖彻底 | 已有 1 份、未来 glTF/FBX 还要再抄 2 份；inverse/rotate 这类代码每次重写都是 bug 温床（见 §2.4 语义注记） | 否 |
| 自研 `src/detail/Math.{h,cpp}` 统一 | 可控、最小 | 重复造轮子；SIMD/正确性要自己背；测试负担 | 否（除非坚持绝对零三方依赖） |
| **GLM vendored** | 成熟、header-only、列主序与现 Mat4 布局一致、`inverse/radians/rotate` 开箱即用、MIT；assimp 上游生态常用 | 编译时间（§2.5）；引入第三方目录 | **是** |
| DirectXMath | viewer 已有 | 行主序/左手系约定与解析器数据（列主序、右手系习惯）相反，跨平台差；不适合 library 层 | 否 |
| Eigen | 更全 | 重得多（编译时间显著）、面向线性代数而非图形管线 | 否 |

### 2.2 内部使用范围

| 使用方 | 用法 | 说明 |
|---|---|---|
| `src/formats/pbrt/PbrtParser.cpp` | 整体替换匿名命名空间的 8 个 mat 函数（对应表见 §2.4） | 唯一现存矩阵代码 |
| `src/formats/gltf/GltfParser.cpp`（未来） | node TRS 合成：`T*R*S`、`node.matrix` 直读、全局变换下推到 POSITION/NORMAL | 实现 scene graph 时引入，替代“再抄一份 mat4” |
| `src/formats/fbx/FbxParser.cpp`（未来） | Model 节点 `Lcl Translation/Rotation/Scaling` + 几何变换（GeometricTranslation 等）应用 | 同上 |
| `viewer/viewer_demo.cpp` | **建议本期不动**。现 `cam[16]` 是 view+proj 融合的自定义行主序布局，DX12 顶点着色器逐元素镜像它（`DxRenderer` 根签名 b0）；换成 `glm::lookAt*glm::perspective` 属于渲染重构而非去重，必须连 shader 一起改并回归视觉验证。若后续重构：用 `glm::perspectiveRH_NO`(或 NO) 生成投影、`glm::lookAt` 生成视图，再显式转置成着色器期望的行主序常量区 | 单独 PR，勿夹带 |
| `include/**` | **禁止出现任何 glm 头** | 见 §2.3 |

### 2.3 为何公共 API 必须保持零依赖

`include/assetpack/AssetPack.h` 是唯一公共伞形头，消费者只 include 它。若它传递暴露 glm：

1. 下游被迫可见/可传递获得一份 glm，破坏“拷来即用”的库定位；
2. glm 类型进入 ABI（`PackResult`/回调签名一旦含 `glm::mat4`，升级 glm 即 ABI 破坏）;
3. 与仓库现状矛盾：rapidjson/taskflow/stb 全部是 PRIVATE 包含，无一泄漏到公共头。

**实现约束（接线 CMake 时执行）**

- `target_include_directories(assetpack PRIVATE third_party/glm/include)` —— 必须 `PRIVATE`，绝不 `PUBLIC`/`INTERFACE`；
- glm 头只允许出现在 `src/**` 的 `.cpp` 与 `src/detail|formats` 内部头中；
- 公共头中的向量/矩阵字段维持 `float*`/`std::span<const float>` 原样（现状已是如此，无需改公共契约）；
- 若未来公共头确需数学类型：先定义自己的 POD（如 `ap::Vec3{float x,y,z}`）再做边界转换，仍不让 glm 出现在 `include/`。

### 2.4 迁移步骤清单（PbrtParser Mat4 → glm::mat4 逐函数对应表）

前置：CMake 增加 PRIVATE include 路径 + （建议）`target_compile_definitions(assetpack PRIVATE GLM_ENABLE_EXPERIMENTAL=0)` 保持默认严格模式。

| 现符号（`src/PbrtParser.cpp`） | 行 | GLM 替换 | 备注 |
|---|---|---|---|
| `using Mat4 = std::array<float,16>` | 46 | `glm::mat4` | 二者同为**列主序**，内存布局兼容 |
| `matIdentity()` | 53 | `glm::mat4(1)` | 构造即单位阵 |
| `matMul(a,b)` | 59 | `a * b` | 列主序乘法语义一致（右乘追加变换，现 `ctm = matMul(ctm, X)` → `ctm *= X`） |
| `matTranslate(x,y,z)` | 71 | `glm::translate(glm::mat4(1), {x,y,z})` | `gtc/matrix_transform.hpp` |
| `matScale(x,y,z)` | 79 | `glm::scale(glm::mat4(1), {x,y,z})` | 同上 |
| `matRotate(angDeg,ax,ay,az)` | 88 | `glm::rotate(glm::mat4(1), glm::radians(angDeg), {ax,ay,az})` | 现**手写轴角旋转矩阵**约 20 行 → 1 行；注意现实现接收**角度制**，glm 接收弧度，必须 `glm::radians` |
| `matTransformPoint(m,x,y,z,ox,oy,oz)` | 112 | `glm::vec4 p = m * glm::vec4{x,y,z,1}; if (p.w != 0.f) p /= p.w;` | glm 不自动做透视除法，保留现有 w 除法逻辑 |
| `matInvert(m,out)` | 121 | `if (glm::determinant(m) == 0.f) return false; out = glm::inverse(m); return true;` | 删除 24 行手写代数余子式；保留 det==0 早退以维持调用点语义 |
| `matTransformNormal(m,...)` | 147 | 见下方语义注记 | |
| `readMat16()` | 408 | 循环填 `m[c][r] = f` | pbrt 文件的 16 个 float 为列主序，与 glm `m[col][row]` 下标一致，**读取顺序不变** |

**语义注记（迁移时的决策点，须写进提交说明）**

- 现 `matTransformNormal` 用 `inv * n`（逆矩阵，**未转置**）。这对刚体变换与非均匀缩放前等价，但**非均匀缩放下数学上不正确**（正确法线矩阵是逆转置）。GLM 提供 `glm::inverseTranspose(m)`（`gtc/matrix_epsilon.hpp`… 实为 `glm::inverseTranspose` 于 `<glm/gtx/matrix_operation.hpp>` 之外的核心 `matrix.hpp`）。建议迁移时顺手改为：
  `n' = glm::normalize(glm::mat3(glm::inverseTranspose(m)) * n)`
  并在测试里加一个非均匀缩放用例锁定行为；若要求逐位回归一致，则先等价替换 `glm::mat3(glm::inverse(m)) * n` 再另行修复。

**步骤清单**

1. CMake：`assetpack` 目标加 `PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glm/include`；
2. 补 `third_party/glm/LICENSE`（vendor 来源目录未携带 `copying.txt`；从 https://github.com/g-truc/glm 取 MIT 文本，注明版本 1.0.1 与 commit）；
3. PbrtParser：按上表逐函数替换，删除匿名命名空间旧实现；`ParseState::ctm` 改 `glm::mat4`；
4. 构建 + `--gtest_filter` 相关用例（pbrt 场景回归：San Miguel/pbrt 测试模型几何哈希比对）；
5. （独立后续 PR）gltf/FBX 节点变换实现时复用同一 glm 依赖；
6. （可选后续 PR）viewer 相机数学重构（§2.2）。

### 2.5 编译时间影响与缓解

- **影响**：umbrella 头 `<glm/glm.hpp>` 会拉入全部核心 + gtc 子集（数百个头）。但受影响的 TU 目前只有 `PbrtParser.cpp` 1 个（未来 +gltf/fbx 2 个），且 glm 大量使用 `constexpr`/模板内联，实测社区经验单 TU 增量约 +1~3 s（MSVC /O2）。
- **缓解**
  1. **只 include 需要的细分头**，不用 umbrella：`<glm/mat4x4.hpp>`、`<glm/vec3.hpp>`、`<glm/geometric.hpp>`、`<glm/gtc/matrix_transform.hpp>`、`<glm/gtc/constants.hpp>`；`<glm/gtx/*>`（64 个实验头）一律禁用，避免拉入最大子树与不稳定 API；
  2. 如后续多个 parser 都要用，建 `src/detail/Math.h` 作为**项目内唯一 glm 聚合点**（内部头，非公共头），集中控制包含面；
  3. 不启用 `GLM_FORCE_SIMD_AVX2` 之类的强制指令宏（会传染 ODR），保持默认平台 SIMD；
  4. 如整体构建时间敏感，可为 `assetpack` 目标加 MSVC `/p:PrecompiledHeader` 式 PCH（CMake `target_precompile_headers`）把 glm 收进 PCH——属可选项，非必需；
  5. glm 是纯头库，**不影响**链接体积与运行时分发。

---

## 3. `src/` 目录重构方案

### 3.1 目标树（对齐 assimp 上游 `code/AssetLib/<Format>/` 惯例）

```
src/
├── core/                      # 库骨架：门面/日志/mmap
│   ├── AssetPack.cpp
│   ├── Log.cpp
│   └── MappedFile.cpp
├── detail/                    # 通用解析工具（原样保留，位置不变）
│   ├── FastParse.h
│   └── TextScan.h
└── formats/                   # 每格式一目录，对齐上游 AssetLib/<Format>
    ├── obj/                   # OBJ 现有 4 组文件整体迁入
    │   ├── ObjParser.cpp      # 原 src/ObjParser.cpp（入口实现并入目录）
    │   ├── ObjParser.h
    │   ├── Scan.h / Scan.cpp
    │   ├── Geometry.h / Geometry.cpp
    │   └── Mtl.h / Mtl.cpp
    ├── stl/StlParser.cpp
    ├── ply/PlyParser.cpp
    ├── gltf/GltfParser.cpp
    ├── fbx/FbxParser.cpp
    └── pbrt/PbrtParser.cpp
```

设计取向：
- `ObjParser.cpp` 从 `src/` 根**并入** `formats/obj/`，消除“入口 .cpp 在根、头在子目录”的错位；格式入口一律 `formats/<fmt>/<Fmt>Parser.cpp`；
- 单一库目标不变，不做 per-format OBJECT library（收益小、复杂度高）；将来某格式需要拆多文件时在其目录内自由生长即可；
- `-I src` 这一 PRIVATE 包含路径**保持不变**：所有 `"detail/..."`、`"formats/..."` 根相对包含继续成立。

### 3.2 精确的文件移动映射表

| # | 现路径 | 新路径 | `git mv` 命令 |
|---|---|---|---|
| 1 | `src/AssetPack.cpp` | `src/core/AssetPack.cpp` | `git mv src/AssetPack.cpp src/core/AssetPack.cpp` |
| 2 | `src/Log.cpp` | `src/core/Log.cpp` | `git mv src/Log.cpp src/core/Log.cpp` |
| 3 | `src/MappedFile.cpp` | `src/core/MappedFile.cpp` | `git mv src/MappedFile.cpp src/core/MappedFile.cpp` |
| 4 | `src/ObjParser.cpp` | `src/formats/obj/ObjParser.cpp` | `git mv src/ObjParser.cpp src/formats/obj/ObjParser.cpp` |
| 5 | `src/StlParser.cpp` | `src/formats/stl/StlParser.cpp` | `git mv src/StlParser.cpp src/formats/stl/StlParser.cpp` |
| 6 | `src/PlyParser.cpp` | `src/formats/ply/PlyParser.cpp` | `git mv src/PlyParser.cpp src/formats/ply/PlyParser.cpp` |
| 7 | `src/GltfParser.cpp` | `src/formats/gltf/GltfParser.cpp` | `git mv src/GltfParser.cpp src/formats/gltf/GltfParser.cpp` |
| 8 | `src/FbxParser.cpp` | `src/formats/fbx/FbxParser.cpp` | `git mv src/FbxParser.cpp src/formats/fbx/FbxParser.cpp` |
| 9 | `src/PbrtParser.cpp` | `src/formats/pbrt/PbrtParser.cpp` | `git mv src/PbrtParser.cpp src/formats/pbrt/PbrtParser.cpp` |
| 10-16 | `src/obj/{ObjParser.h, Scan.h, Scan.cpp, Geometry.h, Geometry.cpp, Mtl.h, Mtl.cpp}` | `src/formats/obj/<同名>` | `git mv src/obj src/formats/obj`（整目录一次完成） |

共 16 个文件移动；`src/detail/` 不动。

### 3.3 每个文件需要改的 `#include` 清单

前提：`target_include_directories(... PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)` 保留 ⇒ `"detail/..."`、`"assetpack/AssetPack.h"`、`<taskflow/...>`、`"rapidjson/..."` 全部不受影响。引号包含优先“包含者所在目录”，因此同目录互引（`"Scan.h"`、`"Geometry.h"`、`"Mtl.h"`、`"ObjParser.h"`）随整组搬迁后**天然继续有效**。

| 文件（新位置） | 现包含 | 动作 | 原因 |
|---|---|---|---|
| `src/core/AssetPack.cpp` | `#include "obj/ObjParser.h"` | → `#include "formats/obj/ObjParser.h"` | `src/obj/` 不复存在 |
| `src/formats/obj/ObjParser.cpp` | `#include "obj/ObjParser.h"` | → `#include "ObjParser.h"` | 已与新头同目录（亦可写根相对 `"formats/obj/ObjParser.h"`，推荐前者，与 `Scan.cpp` 的 `"Scan.h"` 风格一致） |
| `src/formats/obj/ObjParser.cpp` | `#include "detail/TextScan.h"` | 不变 | `-I src` 根相对 |
| `src/formats/obj/ObjParser.cpp` | `#include "obj/Geometry.h"` | → `#include "Geometry.h"` | 同目录化 |
| `src/formats/obj/ObjParser.cpp` | `#include "obj/Mtl.h"` | → `#include "Mtl.h"` | 同目录化 |
| `src/formats/obj/{Scan,Geometry,Mtl}.*`、`ObjParser.h` | 同目录互引 `"X.h"` / `"detail/TextScan.h"` / `<taskflow/...>` / `<assetpack/...>` | **不变** | 整组搬迁 + 根相对包含 |
| `src/core/Log.cpp`、`src/core/MappedFile.cpp` | `<assetpack/AssetPack.h>` + 系统/标准头 | **不变** | 无内部相对包含 |
| `src/formats/{stl,ply,gltf,fbx,pbrt}/<Fmt>Parser.cpp` | 仅 `"assetpack/AssetPack.h"`（gltf 另有 rapidjson） | **不变** | 无内部相对包含 |

汇总：**内部源码只需编辑 2 个文件、共 4 行 include**（`core/AssetPack.cpp` 1 行 + `formats/obj/ObjParser.cpp` 3 行）。

⚠️ **库外一处必改（迁移实施时）**：`test/assetpack_tests.cpp:22` 有 `#include "obj/Scan.h"`（该目标同样拿到 `-I src`），必须同步改为 `#include "formats/obj/Scan.h"`，否则测试 TU 编译失败。本次设计任务因硬性规则未触碰 `test/**`，实施迁移时把它列入同一原子提交。`viewer/**` 只包含 `<assetpack/AssetPack.h>`，完全不受影响。

### 3.4 `CMakeLists.txt` source 列表 diff 草案

```diff
@@ 库源文件（现 L15-28） @@
 add_library(assetpack
-    src/AssetPack.cpp
-    src/ObjParser.cpp
-    src/StlParser.cpp
-    src/PlyParser.cpp
-    src/MappedFile.cpp
-    src/Log.cpp
-    src/obj/Scan.cpp
-    src/obj/Geometry.cpp
-    src/obj/Mtl.cpp
-    src/GltfParser.cpp
-    src/FbxParser.cpp
-    src/PbrtParser.cpp
+    src/core/AssetPack.cpp
+    src/core/Log.cpp
+    src/core/MappedFile.cpp
+    src/formats/obj/ObjParser.cpp
+    src/formats/obj/Scan.cpp
+    src/formats/obj/Geometry.cpp
+    src/formats/obj/Mtl.cpp
+    src/formats/stl/StlParser.cpp
+    src/formats/ply/PlyParser.cpp
+    src/formats/gltf/GltfParser.cpp
+    src/formats/fbx/FbxParser.cpp
+    src/formats/pbrt/PbrtParser.cpp
 )
```

```diff
@@ 注释与包含目录（现 L13-14、L31） @@
-# 文件结构：include/ 只放对外 API；src/detail 通用工具；src/obj 是
-# OBJ/MTL 解析器内部分层（Scan 文本扫描 / Geometry 几何子流 / Mtl 材质）。
+# 文件结构：include/ 只放对外 API；src/core 库骨架（门面/日志/mmap）；
+# src/detail 通用解析工具；src/formats/<fmt> 每格式一目录（对齐
+# assimp 上游 code/AssetLib/<Format>/ 惯例）。
 target_include_directories(assetpack
     PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
-    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src           # detail/ obj/ 内部头
+    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src           # core/ detail/ formats/ 内部头
     PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party   # <taskflow/taskflow.hpp>
     PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rapidjson/include  # glTF JSON
 )
```

（若同期做 GLM 接线，在此追加一行 `PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glm/include`，见 §2.4 步骤 1。其余目标——`imgui`、`assetpack_viewer`、`assetpack_tests`——的源列表与包含目录**均不需要变化**；tests 目标的 `-I src` 继续覆盖新层级，仅需 §3.3 所述那 1 行 include 修正。）

### 3.5 迁移顺序（每步可编译）

字面上的“先移目录 → 再改 include → 再改 CMake”存在一个物理约束：`.cpp` 一旦挪走而 CMake 源列表仍指旧路径，配置阶段即失败；反之亦然。因此给出两种都保证**每个提交点可编译**的执行法，任选其一：

**方案 A（推荐）：单原子提交**

```
步骤 0  基线：cmake --build build && ctest        # 确认绿
步骤 1  mkdir core formats/{stl,ply,gltf,fbx,pbrt}
        git mv（§3.2 全部 16 个文件，src/obj 整目录搬）
步骤 2  编辑 4 行 include（§3.3：core/AssetPack.cpp ×1，
        formats/obj/ObjParser.cpp ×3）
步骤 3  同一提交内更新 CMakeLists.txt（§3.4 两处 diff）
步骤 4  test/assetpack_tests.cpp 第 22 行 "obj/Scan.h" → "formats/obj/Scan.h"
        （与步骤 1-3 同一提交——否则该提交不可编译）
步骤 5  clang-format 校验被移动文件（内容未变则无 diff）；
        cmake --build build && ctest 全绿后提交
```

理由：git 以相似度检测 rename，移动+少量 include 行修改仍在阈值内，历史可追溯；assimp 上游自身的目录整理也采用原子提交。

**方案 B（要求更细粒度提交历史）：转发头过渡**

```
提交 1  core/：git mv 三个 core 文件 + CMake 对应三行 + 构建
        （三者只依赖公共头，无内部相对包含，天然可编译）
提交 2  叶子解析器：git mv stl/ply/gltf/fbx/pbrt 五个 .cpp +
        CMake 五行 + 构建（同理只依赖公共头/第三方）
提交 3  obj 簇：git mv src/obj → src/formats/obj 且 git mv
        src/ObjParser.cpp 入内；同时完成该簇 3 行 include 修改、
        core/AssetPack.cpp 1 行修改、CMake 七行更新、tests 1 行修正；
        在旧位置留转发头 src/obj/*.h（#pragma once + include 新路径）
        以防未知外部消费者 + 构建
提交 4  清理：删除转发头（rg 确认无引用后）+ 构建
```

每一步结束都跑 `cmake --build build && ctest`。

### 3.6 风险点与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| `test/assetpack_tests.cpp` 直接包含内部头 `"obj/Scan.h"`、`"detail/FastParse.h"`、`"detail/TextScan.h"` | tests TU 编译失败 | detail 两项不受影响；`"obj/Scan.h"` 1 行随迁移同提交修正（§3.3/§3.5）；长期看 tests 依赖内部头属白盒测试，可在 CMake 注释中声明这一耦合 |
| 引号包含回退语义误判（以为同目录包含会失效而过度修改） | 无谓扩大 diff | MSVC/GCC 均先查包含者目录再查 `-I`；整组搬迁的同目录互引零修改（§3.3 已逐一核对） |
| git rename 相似度低于阈值导致历史断裂 | blame 困难 | 用 `git mv`；include 行修改控制在个位数；验证 `git log --follow src/formats/pbrt/PbrtParser.cpp` |
| 迁移与 GLM 接线混在一个提交 | 回归归因困难 | 分两个 PR/提交：先目录重构（纯位移，几何输出必须逐位一致），后 GLM 替换（允许法线矩阵语义升级，配非均匀缩放用例） |
| 构建目录缓存旧路径 | Ninja 报缺失源文件 | 迁移提交后重新 `cmake -G Ninja -S . -B build`（或删 build 重配） |
| viewer 误伤 | — | 已核实 viewer 全部只包含 `<assetpack/AssetPack.h>`，零风险；且本次硬规则禁止触碰 |
| 未来新增格式再次漂移 | 结构退化 | 在 `AGENTS.md`「Adding a New Importer」节补充：新格式一律 `src/formats/<fmt>/` 目录 + 注册进 `core/AssetPack.cpp` 的 registry |

---

## 4. 附录：现状内部 include 图（重构前基线）

```
core/AssetPack.cpp     ──► "assetpack/AssetPack.h", "obj/ObjParser.h"
core/Log.cpp           ──► <assetpack/AssetPack.h>
core/MappedFile.cpp    ──► <assetpack/AssetPack.h>, <Windows.h>|POSIX
formats/obj/ObjParser.cpp ──► "ObjParser.h", "assetpack/AssetPack.h",
                               "detail/TextScan.h", "Geometry.h", "Mtl.h"
formats/obj/ObjParser.h   ──► <memory>, <string_view>, <assetpack/AssetPack.h>
formats/obj/Scan.h        ──► std + "detail/TextScan.h"
formats/obj/Scan.cpp      ──► "Scan.h"
formats/obj/Geometry.h    ──► <taskflow/taskflow.hpp>, <assetpack/AssetPack.h>,
                               "ObjParser.h", "Scan.h"
formats/obj/Geometry.cpp  ──► "Geometry.h", <assetpack/AssetPack.h>,
                               <taskflow/algorithm/for_each.hpp>
formats/obj/Mtl.h         ──► <assetpack/AssetPack.h>, "ObjParser.h"
formats/obj/Mtl.cpp       ──► "Mtl.h", <assetpack/AssetPack.h>, "detail/TextScan.h"
formats/stl|ply|fbx|pbrt  ──► "assetpack/AssetPack.h" 仅此一个内部依赖
formats/gltf              ──► "assetpack/AssetPack.h" + rapidjson/document.h,
                               rapidjson/error/en.h
detail/TextScan.h         ──► "FastParse.h"（FastParse.h 无内部依赖）
test/assetpack_tests.cpp  ──► "assetpack/AssetPack.h", "detail/FastParse.h",
                               "obj/Scan.h"(迁移时须改), "detail/TextScan.h"
```

—— 完 ——
