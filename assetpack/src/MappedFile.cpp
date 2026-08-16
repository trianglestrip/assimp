#include <assetpack/AssetPack.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

namespace ap {

MappedFile::~MappedFile() { close(); }

void MappedFile::close() {
    if (base_)    { ::UnmapViewOfFile(base_); base_ = nullptr; }
    if (mapping_) { ::CloseHandle(static_cast<HANDLE>(mapping_)); mapping_ = nullptr; }
    if (file_)    { ::CloseHandle(static_cast<HANDLE>(file_)); file_ = nullptr; }
    bytes_ = {};
}

bool MappedFile::open(std::string_view path) {
    close();
    if (path.empty()) return false;

    // CreateFileA needs a NUL-terminated string; the path is only
    // copied here once at open time, never afterwards.
    const std::string p(path);
    HANDLE h = ::CreateFileA(p.c_str(), GENERIC_READ, FILE_SHARE_READ,
                             nullptr, OPEN_EXISTING,
                             FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;

    LARGE_INTEGER sz{};
    if (!::GetFileSizeEx(h, &sz) || sz.QuadPart == 0) {
        ::CloseHandle(h);
        return false;
    }

    HANDLE map = ::CreateFileMappingA(h, nullptr, PAGE_READONLY,
                                      0, 0, nullptr);
    if (!map) { ::CloseHandle(h); return false; }

    void* base = ::MapViewOfFile(map, FILE_MAP_READ, 0, 0, 0);
    if (!base) {
        ::CloseHandle(map);
        ::CloseHandle(h);
        return false;
    }

    file_    = h;
    mapping_ = map;
    base_    = base;
    bytes_   = { static_cast<const std::byte*>(base),
                 static_cast<size_t>(sz.QuadPart) };
    return true;
}

std::shared_ptr<MappedFile> MappedFile::openShared(std::string_view path) {
    auto f = std::make_shared<MappedFile>();
    if (!f->open(path)) return nullptr;
    return f;
}

} // namespace ag
