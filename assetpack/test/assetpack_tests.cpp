// assetpack_tests.cpp
//
// Dependency-free unit-test scaffold for the assetpack library.
//
// A tiny self-contained assertion harness (no GoogleTest/Catch2): CHECK /
// CHECK_EQ / CHECK_NEAR print failures to stderr and count them; main()
// returns non-zero if any check failed. Exercises the public API
// (ap::) plus the internal pure functions via the src/ include dir.

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "assetpack/AssetPack.h"
#include "detail/FastParse.h"
#include "obj/Scan.h"
#include "detail/TextScan.h"

namespace {

int g_failures = 0;
int g_checks = 0;

} // namespace

#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::fprintf(stderr, "CHECK failed: %s\n  at %s:%d\n", #cond, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_EQ(a, b) do { \
    ++g_checks; \
    const auto _a = (a); \
    const auto _b = (b); \
    if (!(_a == _b)) { \
        ++g_failures; \
        std::fprintf(stderr, "CHECK_EQ failed: %s == %s\n  at %s:%d\n", #a, #b, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
    ++g_checks; \
    const auto _a = (a); \
    const auto _b = (b); \
    if (!(std::fabs(static_cast<double>(_a) - static_cast<double>(_b)) <= (tol))) { \
        ++g_failures; \
        std::fprintf(stderr, "CHECK_NEAR failed: %s ~= %s (tol %g)\n  at %s:%d\n", \
                     #a, #b, static_cast<double>(tol), __FILE__, __LINE__); \
    } \
} while (0)

static void testFastParseInt() {
    // Fast path: plain positive / negative / explicit-plus.
    CHECK_EQ(ap::fast::parseInt("16937", 5), static_cast<std::int64_t>(16937));
    CHECK_EQ(ap::fast::parseInt("-42", 3), static_cast<std::int64_t>(-42));
    CHECK_EQ(ap::fast::parseInt("+7", 2), static_cast<std::int64_t>(7));

    // Fallback to from_chars yields 0 on no-digit / whitespace tokens.
    CHECK_EQ(ap::fast::parseInt("  ", 2), static_cast<std::int64_t>(0));
    CHECK_EQ(ap::fast::parseInt("abc", 3), static_cast<std::int64_t>(0));

    // 20-digit token is out of the int64 fast range and outside from_chars
    // int64 too, so the established convention returns 0.
    CHECK_EQ(ap::fast::parseInt("12345678901234567890", 20), static_cast<std::int64_t>(0));
}

static void testFastParseFloat() {
    CHECK_NEAR(ap::fast::parseFloat("0.688357", 8), 0.688357f, 1e-5);
    CHECK_NEAR(ap::fast::parseFloat("-1.2e-3", 7), -0.0012f, 1e-5);
    CHECK_EQ(ap::fast::parseFloat("1.", 2), 1.f);
    CHECK_EQ(ap::fast::parseFloat(".5", 2), 0.5f);

    // Empty token falls back to 0.f; special tokens (inf/nan) are accepted
    // by std::from_chars on this platform and yield a non-finite value.
    CHECK_EQ(ap::fast::parseFloat("", 0), 0.f);
    CHECK(!(std::isfinite)(ap::fast::parseFloat("inf", 3)));
    CHECK(!(std::isfinite)(ap::fast::parseFloat("nan", 3)));
}

static void testObjClassify() {
    auto cls = [](const char* s) {
        const char* p = s;
        const char* e = p + std::strlen(p);
        return ap::obj::classify(p, e);
    };

    CHECK_EQ(cls("v 1 2 3"), ap::obj::LineKind::V);
    CHECK_EQ(cls("vn 1 2 3"), ap::obj::LineKind::VN);
    CHECK_EQ(cls("vt 1 2"), ap::obj::LineKind::VT);
    CHECK_EQ(cls("f 1 2 3"), ap::obj::LineKind::F);
    CHECK_EQ(cls("usemtl mat"), ap::obj::LineKind::UseMtl);
    CHECK_EQ(cls("o foo"), ap::obj::LineKind::Object);
    CHECK_EQ(cls("g bar"), ap::obj::LineKind::Group);
    CHECK_EQ(cls("mtllib m.mtl"), ap::obj::LineKind::Mtllib);
    CHECK_EQ(cls("# comment"), ap::obj::LineKind::Other);
}

static void testObjCountFaceVerts() {
    auto count = [](const char* s) {
        const char* p = s;
        const char* e = p + std::strlen(p);
        return ap::obj::countFaceVerts(p, e);
    };

    // Leading "f " is part of the line; three slash-separated corners.
    CHECK_EQ(count("f 1/2/3 4/5/6 7/8/9"), static_cast<uint32_t>(3));
    // Five bare indices.
    CHECK_EQ(count("f 1 2 3 4 5"), static_cast<uint32_t>(5));
}

static void testObjResolveIndex() {
    // 1-based forward; negative counts back from running count.
    CHECK_EQ(ap::obj::resolveIndex(3, 10), static_cast<int32_t>(2));
    CHECK_EQ(ap::obj::resolveIndex(-1, 10), static_cast<int32_t>(9));
    CHECK_EQ(ap::obj::resolveIndex(-2, 10), static_cast<int32_t>(8));
}

static void testTexTypeName() {
    CHECK(std::string(ap::texTypeName(ap::TexDiffuse)) == "diffuse");
    CHECK(std::string(ap::texTypeName(ap::TexNormal)) == "normal");
}

static void testMappedFile() {
    namespace fs = std::filesystem;
    const std::string payload = "hello assetpack\n";
    const fs::path tmp = fs::temp_directory_path() / "assetpack_test_mapped.tmp";

    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        out.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    }

    auto mf = ap::MappedFile::openShared(tmp.string());
    CHECK(mf != nullptr);
    if (mf) {
        CHECK(mf->isOpen());
        CHECK_EQ(mf->size(), payload.size());
        CHECK_EQ(mf->bytes().size(), payload.size());
        const std::string_view got(reinterpret_cast<const char*>(mf->bytes().data()),
                                   mf->bytes().size());
        CHECK(got == payload);
    }

    std::error_code ec;
    fs::remove(tmp, ec);
}

namespace {

std::string writeTemp(const std::string& name, const std::string& content) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "assetpack_test";
    std::error_code ec; fs::create_directories(dir, ec);
    const fs::path p = dir / name;
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    return p.string();
}

std::string writeTempBytes(const std::string& name, const std::vector<uint8_t>& data) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "assetpack_test";
    std::error_code ec; fs::create_directories(dir, ec);
    const fs::path p = dir / name;
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(data.data()),
              static_cast<std::streamsize>(data.size()));
    return p.string();
}

std::vector<uint8_t> makeBinaryStl() {
    std::vector<uint8_t> b(134, 0);
    uint32_t n = 1;
    std::memcpy(&b[80], &n, 4);
    float v[12] = {0,0,1, 0,0,0, 1,0,0, 0,1,0};
    std::memcpy(&b[84], v, sizeof(v));
    return b;
}

static void testStlParsers() {
    { // binary
        auto buf = makeBinaryStl();
        const std::string path = writeTempBytes("ap_stl_bin.stl", buf);
        ap::AssetPack pack;
        CHECK(pack.load(path));
        auto& r = pack.result();
        CHECK_EQ(r.meshes.size(), size_t(1));
        CHECK_EQ(r.positions.size(), size_t(9));
        CHECK_EQ(r.normals.size(), size_t(9));
        if (!r.meshes.empty()) {
            CHECK_EQ(r.meshes[0].vertexCount(), 3u);
            CHECK_EQ(r.meshes[0].triangleCount(), 1u);
        }
    }
    { // ascii
        const std::string asc =
            "solid t\n facet normal 0 0 1\n  outer loop\n"
            "   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n"
            "  endloop\n endfacet\nendsolid t\n";
        const std::string path = writeTemp("ap_stl_asc.stl", asc);
        ap::AssetPack pack;
        CHECK(pack.load(path));
        auto& r = pack.result();
        CHECK_EQ(r.meshes.size(), size_t(1));
        CHECK_EQ(r.positions.size(), size_t(9));
    }
}

static void testPlyParser() {
    const std::string ply =
        "ply\nformat ascii 1.0\nelement vertex 3\n"
        "property float x\nproperty float y\nproperty float z\n"
        "element face 1\nproperty list uchar int vertex_indices\n"
        "end_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n";
    const std::string path = writeTemp("ap_test.ply", ply);
    ap::AssetPack pack;
    CHECK(pack.load(path));
    auto& r = pack.result();
    CHECK_EQ(r.meshes.size(), size_t(1));
    CHECK_EQ(r.positions.size(), size_t(9));
    if (!r.meshes.empty()) CHECK_EQ(r.meshes[0].triangleCount(), 1u);
}

static void testObjFeatures() {
    const std::string obj =
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nf 1/1 2/2 3/3\n";
    const std::string path = writeTemp("ap_feat.obj", obj);
    { // default keeps texcoords
        ap::AssetPack pack;
        CHECK(pack.load(path));
        auto& r = pack.result();
        CHECK_EQ(r.positions.size(), size_t(9));
        CHECK_EQ(r.texcoords.size(), size_t(6));
    }
    { // opt-out drops the uv pool
        ap::AssetPack pack;
        pack.setWantTexcoords(false);
        CHECK(pack.load(path));
        auto& r = pack.result();
        CHECK_EQ(r.positions.size(), size_t(9));
        CHECK(r.texcoords.empty());
    }
}

static void testObjWarningsAndTextures() {
    { // missing mtllib + unresolved usemtl -> warnings
        const std::string obj =
            "mtllib nope.mtl\nusemtl ghost\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
        const std::string path = writeTemp("ap_warn.obj", obj);
        ap::AssetPack pack;
        CHECK(pack.load(path));
        CHECK(pack.warnings().size() > 0);
    }
    { // texture byte loading
        const std::string tex("PK\03\04dummy", 9);
        const std::string mtl = "newmtl m\nmap_Kd tex.bin\n";
        const std::string obj =
            "mtllib a.mtl\nusemtl m\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
        writeTemp("tex.bin", tex);
        writeTemp("a.mtl", mtl);
        const std::string path = writeTemp("ap_tex.obj", obj);
        ap::AssetPack pack;
        pack.setWantTextureBytes(true);
        CHECK(pack.load(path));
        auto& r = pack.result();
        bool loaded = false;
        for (const auto& t : r.textures)
            if (t.byteSize > 0 && !t.data.empty()) loaded = true;
        CHECK(loaded);
    }
}

static void testGltfAndFbxParsers() {
    // Exercises the real assimp sample models when present on this machine;
    // skips silently otherwise so the suite stays hermetic in other envs.
    namespace fs = std::filesystem;
    const char* gltf = "F:/project/assimp/test/models/GLTF2/BoxTextured-glTF/BoxTextured.gltf";
    const char* glb  = "F:/project/assimp/test/models/GLTF2/BoxTextured-glTF-Binary/BoxTextured.glb";
    const char* fbx  = "F:/project/assimp/test/models/FBX/box.fbx";

    if (fs::exists(gltf)) {
        ap::AssetPack pack;
        CHECK(pack.load(gltf));
        auto& r = pack.result();
        CHECK_EQ(r.positions.size(), size_t(72));   // 24 verts
        CHECK_EQ(r.meshes.size(), size_t(1));
        CHECK(r.materials.size() >= 1);
        CHECK(r.textures.size() >= 1);
    }
    if (fs::exists(glb)) {
        ap::AssetPack pack;
        CHECK(pack.load(glb));
        auto& r = pack.result();
        CHECK_EQ(r.positions.size(), size_t(72));
        bool embedded = false;
        for (const auto& t : r.textures)
            if (t.embedded && t.byteSize > 0) embedded = true;
        CHECK(embedded);
    }
    if (fs::exists(fbx)) {
        ap::AssetPack pack;
        CHECK(pack.load(fbx));
        auto& r = pack.result();
        CHECK_EQ(r.positions.size(), size_t(72));
        CHECK_EQ(r.meshes.size(), size_t(1));
    }
}

} // namespace

int main() {
    testFastParseInt();
    testFastParseFloat();
    testObjClassify();
    testObjCountFaceVerts();
    testObjResolveIndex();
    testTexTypeName();
    testMappedFile();
    testStlParsers();
    testPlyParser();
    testObjFeatures();
    testObjWarningsAndTextures();
    testGltfAndFbxParsers();

    if (g_failures == 0) {
        std::printf("assetpack_tests: all %d checks passed\n", g_checks);
        return 0;
    }
    std::fprintf(stderr, "assetpack_tests: %d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
}
