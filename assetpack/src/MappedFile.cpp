#include <assetpack/AssetPack.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _WIN32_WINNT          // PrefetchVirtualMemory / MEMORY_RANGE_ENTRY
#define _WIN32_WINNT 0x0602   // need Windows 8+ API surface
#endif
#include <Windows.h>

#include <algorithm>

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

std::span<const std::byte> MappedFile::bytes() const { return bytes_; }

bool MappedFile::isOpen() const { return bytes_.data() != nullptr; }

size_t MappedFile::size() const { return bytes_.size(); }

std::string_view MappedFile::text() const {
    return { reinterpret_cast<const char*>(bytes_.data()), bytes_.size() };
}

// PrefetchVirtualMemory is only advisory and pages in up to 64 entries
// per call (Windows 8.1+); walk the range in 64-page batches. The call
// itself is a kernel hint - it never blocks on disk, the read happens
// in the background. Non-Windows builds get a compile-time no-op.
bool MappedFile::prefetch(size_t offset, size_t len) const {
    if (!base_ || offset >= bytes_.size()) return false;
    len = std::min(len, bytes_.size() - offset);
    if (len == 0) return false;
#ifdef _WIN32
    static constexpr size_t kPagesPerCall = 64;
    static constexpr size_t kPage = 4096;
    const size_t start = offset / kPage * kPage;
    const size_t end = (offset + len + kPage - 1) / kPage * kPage;
    size_t off = start;
    while (off < end) {
        WIN32_MEMORY_RANGE_ENTRY e;
        e.VirtualAddress = static_cast<char*>(base_) + off;
        e.NumberOfBytes = std::min(kPagesPerCall * kPage, end - off);
        if (!::PrefetchVirtualMemory(::GetCurrentProcess(), 1, &e, 0))
            return false;
        off += e.NumberOfBytes;
    }
    return true;
#else
    (void)offset;
    (void)len;
    return false;
#endif
}

} // namespace ag
